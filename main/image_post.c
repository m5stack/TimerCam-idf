#include <string.h>
#include <stdlib.h>

#include "esp_http_client.h"
#include "esp_err.h"
#include "esp_log.h"

#define SERVER_URL "http://192.168.0.104:5001/post"


bool http_post_image(const char* url, const char* tok, const uint8_t *data, uint32_t len, uint32_t voltage) {
    bool result = false;

    esp_http_client_config_t config = {
        .url = url,
    };
    // memset(&config, 0, sizeof(esp_http_client_config_t));
    esp_http_client_handle_t client = esp_http_client_init(&config);

    char *params = NULL; 
    asprintf(&params, "%s?tok=%s&voltage=%d", url, tok, voltage);

    if (params == NULL) {
        ESP_LOGE("HTTP_REQUEST", "Params malloc fails");
    }
    
    esp_http_client_set_url(client, params);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/binary");
    esp_http_client_set_post_field(client, (char *)data, len);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        if (status_code == 200) {
            result = true;
        }
    }
    if (params) {
        free(params);
    }
    return result;
}
