#include "cJSON.h"
#include "esp_spi_flash.h"
#include "esp_err.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char* TAG = "ExtInfo";
#endif

#define INFO_SIZE 4 * 1024

cJSON *monitor_json = NULL;

esp_err_t ExtInfoInitAddr(uint32_t addr) {
    esp_err_t res = ESP_FAIL;
    uint8_t *buff = (uint8_t *)malloc(sizeof(uint8_t) * INFO_SIZE);
    if (buff == NULL) {
        ESP_LOGE(TAG, "BUFF malloc failed");
        goto end;
    }

    res = spi_flash_read(addr, buff, INFO_SIZE);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "SPI flash 0x%x read error", addr);
        goto end;
    }

    uint16_t len = (buff[0] << 8) | buff[1];
    if (len > (INFO_SIZE - 10)) {
        ESP_LOGE(TAG, "Data len format error");
        goto end;
    }
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len + 2; i++) {
        crc += buff[i];
    }

    if (crc != buff[len + 2]) {
        ESP_LOGE(TAG, "Data crc format error");
        goto end;
    }

    monitor_json = cJSON_Parse((char *)&buff[2]);
    if (monitor_json == NULL) {
        ESP_LOGE(TAG, "Data json format error");
        goto end;
    }

    res = ESP_OK;
    ESP_LOGI(TAG, "Data read from 0x%x success", addr);

end:
    if(buff) free(buff);
    return res;
}

esp_err_t ExtInfoGetString(char *key, char **string) {
    esp_err_t res = ESP_FAIL;
    if (monitor_json == NULL) {
        ESP_LOGE(TAG, "Not init");
        return res;
    }

    const cJSON* name = cJSON_GetObjectItemCaseSensitive(monitor_json, key);
    if (!cJSON_IsString(name) || name->valuestring == NULL) {
        ESP_LOGE(TAG, "Not found string key %s", key);
        return res;
    }

    *string = name->valuestring;
    res = ESP_OK;
    return res;
}

esp_err_t ExtInfoGetInt(char *key, int* value) {
    esp_err_t res = ESP_FAIL;
    if (monitor_json == NULL) {
        ESP_LOGE(TAG, "Not init");
        return res;
    }

    const cJSON* name = cJSON_GetObjectItemCaseSensitive(monitor_json, key);
    if (!cJSON_IsNumber(name)) {
        ESP_LOGE(TAG, "Not found number key %s", key);
        return res;
    }

    *value = name->valueint;
    res = ESP_OK;
    return res;
}

esp_err_t ExtInfoGetDouble(char *key, double* value) {
    esp_err_t res = ESP_FAIL;
    if (monitor_json == NULL) {
        ESP_LOGE(TAG, "Not init");
        return res;
    }

    const cJSON* name = cJSON_GetObjectItemCaseSensitive(monitor_json, key);
    if (!cJSON_IsNumber(name)) {
        ESP_LOGE(TAG, "Not found number key %s", key);
        return res;
    }

    *value = name->valuedouble;
    res = ESP_OK;
    return res;
}
