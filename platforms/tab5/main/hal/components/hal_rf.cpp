/*
 * SPDX-License-Identifier: MIT
 *
 *  RF sensor hub link  -  reads the passive RF sniffer (WROOM/C5 on Grove
 *  Port A) over UART2 and parses its compact line protocol into HAL state
 *  that the "RF LAB" screen displays. Also forwards module on/off commands
 *  back to the hub. RX/display only on this side.
 *
 *  Protocol (hub -> Tab5), one line each, '|'-separated:
 *    #|...                    banner / status (STATUS line carries module flags)
 *    B|ch|rssi|bssid|ssid     Wi-Fi AP present
 *    P|ch|rssi|src|ssid       Wi-Fi probe request
 *    D|ch|rssi|src|dst|reason Wi-Fi deauth     X|... disassoc
 *    A|ch|rssi|src|dst        Wi-Fi auth/assoc (ignored - low value)
 *    C|ch                     channel marker  (ignored)
 *    S24|v0,...,v125          2.4 GHz spectrum (126 bins, 0..9)
 *    SUB|start|stepkHz|n|r0,..  sub-GHz RSSI sweep
 *    R|mhz|rssi|<hex>         sub-GHz packet
 *    OK|name=val             command ack
 *
 *  Wiring (Grove Port A, 3.3 V):
 *    Tab5 G53 (RX) <- hub TX    Tab5 G54 (TX) -> hub RX    + 5V/GND
 */
#include "hal/hal_esp32.h"
#include <mooncake_log.h>
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>
#include <vector>
#include <cstdlib>
#include <cstring>
#include "mbedtls/base64.h"

// Decode a base64 chunk (no embedded whitespace) into raw bytes.
static std::string rf_b64dec(const std::string& in)
{
    if (in.empty()) return "";
    size_t cap = (in.size() / 4 + 1) * 3 + 4, olen = 0;
    std::string out; out.resize(cap);
    if (mbedtls_base64_decode((unsigned char*)out.data(), cap, &olen,
                              (const unsigned char*)in.data(), in.size()) != 0)
        return "";
    out.resize(olen);
    return out;
}

#define TAG        "hal_rf"
#define RF_UART    UART_NUM_2
// Link on the Tab5 M5-Bus (30-pin header on the back) so the hub can mount as a
// module. Both pins are free M5-Bus GPIOs. Power the hub from the M5-Bus 5V/GND.
#define RF_TX_PIN  47       // Tab5 M5-Bus G47 (TX) -> hub RX (WROOM GPIO16)
#define RF_RX_PIN  48       // Tab5 M5-Bus G48 (RX) <- hub TX (WROOM GPIO17)
#define RF_BUF     1024

static uint32_t rf_now_ms() { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

static std::vector<std::string> rf_split(const std::string& s, char sep)
{
    std::vector<std::string> out; size_t i = 0, j;
    while ((j = s.find(sep, i)) != std::string::npos) { out.push_back(s.substr(i, j - i)); i = j + 1; }
    out.push_back(s.substr(i));
    return out;
}

// caller holds the lock
static void rf_push_log(hal::HalBase::RfHubData_t& d, const std::string& s)
{
    d.log.push_back(s);
    while (d.log.size() > 80) d.log.pop_front();
    d.logSeq++;
}

// caller holds the lock: insert/update an AP in the analyzer table
static void rf_upsert_ap(hal::HalBase::RfHubData_t& d, const std::string& bssid,
                         const std::string& ssid, int ch, int rssi)
{
    int slot = -1, oldest = 0; uint32_t ot = 0xFFFFFFFF;
    for (int i = 0; i < d.apCount; i++) {
        if (strncmp(d.apList[i].bssid, bssid.c_str(), 17) == 0) { slot = i; break; }
        if (d.apList[i].lastMs < ot) { ot = d.apList[i].lastMs; oldest = i; }
    }
    if (slot < 0) slot = (d.apCount < 40) ? d.apCount++ : oldest;
    strncpy(d.apList[slot].bssid, bssid.c_str(), 17); d.apList[slot].bssid[17] = 0;
    strncpy(d.apList[slot].ssid,  ssid.c_str(),  32); d.apList[slot].ssid[32]  = 0;
    d.apList[slot].ch = ch; d.apList[slot].rssi = rssi; d.apList[slot].lastMs = rf_now_ms();
    d.apSeq++;
}

static void rf_parse(const std::string& ln)
{
    if (ln.empty()) return;
    auto& d = GetHAL()->rfHub;
    std::lock_guard<std::mutex> lk(d.mutex);
    d.linked = true; d.lastRxMs = rf_now_ms();

    char tag = ln[0];
    auto f = rf_split(ln, '|');

    if (tag == '#') {
        if (ln.find("STATUS") != std::string::npos) {
            auto kv = [&](const char* k, int def) -> int {
                std::string key = std::string(k) + "=";
                size_t p = ln.find(key);
                return (p == std::string::npos) ? def : atoi(ln.c_str() + p + key.size());
            };
            d.enWifi = kv("wifi", d.enWifi); d.enS24 = kv("s24", d.enS24);
            d.enSub  = kv("sub", d.enSub);   d.enSubrx = kv("subrx", d.enSubrx);
            d.nrfFound = kv("nrf", d.nrfFound); d.ccFound = kv("cc", d.ccFound);
            d.enApi  = kv("api", d.enApi);
        }
        rf_push_log(d, ln.substr(0, 72));
        return;
    }
    if (f[0] == "S24" && f.size() >= 2) {
        auto v = rf_split(f[1], ',');
        int n = (int)v.size(); if (n > 126) n = 126;
        for (int i = 0; i < n; i++) d.spec24[i] = atoi(v[i].c_str());
        for (int i = n; i < 126; i++) d.spec24[i] = 0;
        d.spec24Seq++;
        return;
    }
    if (f[0] == "SUB" && f.size() >= 5) {
        d.subStartMHz = atof(f[1].c_str());
        d.subStepkHz  = atof(f[2].c_str());
        int n = atoi(f[3].c_str()); if (n > 64) n = 64; if (n < 0) n = 0;
        d.subN = n;
        auto v = rf_split(f[4], ',');
        for (int i = 0; i < n && i < (int)v.size(); i++) d.subRssi[i] = atoi(v[i].c_str());
        d.subSeq++;
        return;
    }
    if (f[0] == "WST" && f.size() >= 2) {
        strncpy(d.staState, f[1].c_str(), sizeof(d.staState) - 1); d.staState[sizeof(d.staState) - 1] = 0;
        if (f.size() >= 3) { strncpy(d.staIp, f[2].c_str(), sizeof(d.staIp) - 1); d.staIp[sizeof(d.staIp) - 1] = 0; }
        if (f.size() >= 4) d.staRssi = atoi(f[3].c_str());
        if (f.size() >= 5) { strncpy(d.staSsid, f[4].c_str(), sizeof(d.staSsid) - 1); d.staSsid[sizeof(d.staSsid) - 1] = 0; }
        return;
    }
    // AI relay response: base64 body chunks, then AEND|<status> or AERR|<msg>.
    if (f[0] == "ARSP" && f.size() >= 2) { if (d.relayState == 1) d.relayResp += rf_b64dec(f[1]); return; }
    if (f[0] == "AEND" && f.size() >= 2) { d.relayStatus = atoi(f[1].c_str()); d.relayState = 2; return; }
    if (f[0] == "AERR") { d.relayResp = (f.size() >= 2 ? f[1] : "relay error"); d.relayStatus = 0; d.relayState = 3; return; }
    if (tag == 'B' && f.size() >= 5) {
        rf_upsert_ap(d, f[3], f[4], atoi(f[1].c_str()), atoi(f[2].c_str()));
        rf_push_log(d, "AP   ch" + f[1] + " " + f[2] + "dBm " + f[4] + " [" + f[3] + "]");
        return;
    }
    if (tag == 'P' && f.size() >= 5) { rf_push_log(d, "probe ch" + f[1] + " " + f[2] + "dBm \"" + f[4] + "\""); return; }
    if ((tag == 'D' || tag == 'X') && f.size() >= 6) {
        rf_push_log(d, std::string(tag == 'D' ? "DEAUTH" : "disasc") + " ch" + f[1] + " " + f[2] + "dBm " + f[3] + " r" + f[5]);
        uint32_t t = rf_now_ms();
        if (t - d.deauthWinMs > 3000) { d.deauthWinMs = t; d.deauthCount = 0; }
        if (++d.deauthCount >= 10) d.alarmUntilMs = t + 6000;
        return;
    }
    if (tag == 'R' && f.size() >= 4) { rf_push_log(d, "sub-pkt " + f[1] + "MHz " + f[2] + "dBm " + f[3].substr(0, 24)); return; }
    if (f[0] == "OK" && f.size() >= 2) {
        auto kv = rf_split(f[1], '=');
        if (kv.size() == 2) {
            int v = atoi(kv[1].c_str());
            if      (kv[0] == "wifi")  d.enWifi  = v;
            else if (kv[0] == "s24")   d.enS24   = v;
            else if (kv[0] == "sub")   d.enSub   = v;
            else if (kv[0] == "subrx") d.enSubrx = v;
        }
        rf_push_log(d, ln);
        return;
    }
    // 'A' (auth/assoc) and 'C' (channel marker) are ignored to reduce noise.
}

static void rf_task(void*)
{
    uint8_t* buf = (uint8_t*)malloc(RF_BUF);
    std::string acc;
    while (1) {
        int len = uart_read_bytes(RF_UART, buf, RF_BUF, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)buf[i];
                if (c == '\n') { if (!acc.empty()) { rf_parse(acc); acc.clear(); } }
                else if (c != '\r') { acc += c; if (acc.size() > 1024) acc.clear(); }
            }
        }
        {
            std::lock_guard<std::mutex> lk(GetHAL()->rfHub.mutex);
            auto& d = GetHAL()->rfHub;
            while (!d.txQueue.empty()) { uint8_t b = d.txQueue.front(); d.txQueue.pop(); uart_write_bytes(RF_UART, &b, 1); }
            uint32_t t = rf_now_ms();
            if (d.linked && t - d.lastRxMs > 4000) d.linked = false;
            d.alarm = (t < d.alarmUntilMs);
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void HalEsp32::rfHubBegin()
{
    mclog::tagInfo(TAG, "rf hub init (Port A UART2 on G53/G54)");
    uart_config_t cfg = {};
    cfg.baud_rate  = 115200;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    if (uart_driver_install(RF_UART, RF_BUF * 2, 0, 0, NULL, 0) != ESP_OK) {
        mclog::error("rf uart install failed"); return;
    }
    uart_param_config(RF_UART, &cfg);
    uart_set_pin(RF_UART, RF_TX_PIN, RF_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(rf_task, "rfhub", 4096, NULL, 5, NULL);
}
