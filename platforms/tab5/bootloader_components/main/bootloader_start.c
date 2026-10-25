/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * LOADOUT custom bootloader.
 *   Boots the 'factory' partition (LOADOUT) ALWAYS, except when LOADOUT has set
 *   a one-shot "launch" flag in the 'launchflag' partition. In that case the
 *   bootloader clears the flag and boots 'ota_0' exactly once; the next reset
 *   falls back to factory. This makes "reset = back to LOADOUT" reliable even
 *   for launched apps that mark themselves valid in otadata.
 */
#include <stdbool.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "bootloader_init.h"
#include "bootloader_utility.h"
#include "bootloader_common.h"
#include "bootloader_flash_priv.h"
#include "esp_flash_partitions.h"   /* FACTORY_INDEX */

static const char* TAG = "boot";

/* Must match fw_boot.c in the app. 'launchflag' is at this fixed flash offset
 * (see partitions.csv); the bootloader can't look it up by name, so it is
 * hard-coded here. Keep the two in sync if the partition table changes. */
#define LOADOUT_FLAG_OFFSET  0x12000
#define LOADOUT_FLAG_SIZE    0x1000
#define LOADOUT_FLAG_MAGIC   0x4C4F4144u   /* "LOAD" */

static int select_partition_number(bootloader_state_t *bs);

void __attribute__((noreturn)) call_start_cpu0(void)
{
    if (bootloader_init() != ESP_OK) {
        bootloader_reset();
    }

    bootloader_state_t bs = {0};
    int boot_index = select_partition_number(&bs);
    if (boot_index == INVALID_INDEX) {
        bootloader_reset();
    }

    bootloader_utility_load_boot_image(&bs, boot_index);
}

static int select_partition_number(bootloader_state_t *bs)
{
    if (!bootloader_utility_load_partition_table(bs)) {
        ESP_LOGE(TAG, "load partition table error!");
        return INVALID_INDEX;
    }

    uint32_t flag = 0;
    if (bootloader_flash_read(LOADOUT_FLAG_OFFSET, &flag, sizeof(flag), false) == ESP_OK &&
        flag == LOADOUT_FLAG_MAGIC) {
        /* One-shot launch: clear the flag FIRST so a crash/reset can't loop,
         * then boot ota_0. */
        bootloader_flash_erase_range(LOADOUT_FLAG_OFFSET, LOADOUT_FLAG_SIZE);
        esp_rom_printf("[boot] launch flag set -> booting ota_0 once\n");
        return 0;   /* ota_0 */
    }

    esp_rom_printf("[boot] booting factory (LOADOUT)\n");
    return FACTORY_INDEX;
}

/* Needed if any newlib functions get linked into the bootloader. */
struct _reent *__getreent(void)
{
    return _GLOBAL_REENT;
}
