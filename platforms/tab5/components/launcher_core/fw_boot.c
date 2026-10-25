#include "fw_boot.h"

#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_log.h"

#include <stdio.h>
#include <stdint.h>

static const char *TAG = "fw_boot";

#define FW_CHUNK 4096

esp_err_t fw_boot_file(const char *path, fw_boot_progress_cb_t progress, void *ctx)
{
    // Ranura de app LIBRE (la que NO esta corriendo el launcher). Con dos slots
    // (ota_0/ota_1) esto alterna; el externo se escribe ahi y, al no marcarse
    // valido, el siguiente reset hace rollback a la ranura del launcher.
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
        ESP_LOGE(TAG, "No hay particion OTA libre");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Destino: %s", target->label);

    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "No se pudo abrir %s", path);
        return ESP_FAIL;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size <= 0 || (size_t)size > target->size) {
        ESP_LOGE(TAG, "Tamano invalido (%ld bytes, ota_0=%u)", size, (unsigned)target->size);
        fclose(f);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Escribiendo %ld bytes de %s en ota_0...", size, path);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, (size_t)size, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        fclose(f);
        return err;
    }

    static uint8_t buf[FW_CHUNK];
    size_t written = 0, r;
    while ((r = fread(buf, 1, sizeof(buf), f)) > 0) {
        err = esp_ota_write(handle, buf, r);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
            break;
        }
        written += r;
        if (progress) {
            progress(written, (size_t)size, ctx);
        }
    }
    fclose(f);

    if (err != ESP_OK) {
        esp_ota_abort(handle);
        return err;
    }

    err = esp_ota_end(handle);  // valida magic / chip id / revision
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Imagen invalida (esp_ota_end): %s", esp_err_to_name(err));
        return err;
    }

    // NO usamos esp_ota_set_boot_partition / otadata. En su lugar escribimos un
    // flag de "arranque unico" que el bootloader custom lee: arranca ota_0 una
    // sola vez y vuelve a 'factory' (LOADOUT) en el siguiente reset, pase lo que
    // pase con el app lanzado (aunque se marque valido). Debe coincidir con
    // bootloader_components/main/bootloader_start.c
    const uint32_t LOADOUT_FLAG_MAGIC = 0x4C4F4144u;  // "LOAD"
    const esp_partition_t *flagp =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "launchflag");
    if (!flagp) {
        ESP_LOGE(TAG, "no existe la particion launchflag");
        return ESP_ERR_NOT_FOUND;
    }
    err = esp_partition_erase_range(flagp, 0, flagp->size);
    if (err == ESP_OK) {
        err = esp_partition_write(flagp, 0, &LOADOUT_FLAG_MAGIC, sizeof(LOADOUT_FLAG_MAGIC));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no se pudo escribir launchflag: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Firmware listo (ota_0). Reiniciando una vez...");
    esp_restart();
    return ESP_OK;  // inalcanzable
}
