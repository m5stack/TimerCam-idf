#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "config.h"
#include "esp_camera.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "network.h"
#include "uart_frame.h"
#include "protocol.h"
#include "bmm8563.h"
#include "led.h"
#include "timer_cam_config.h"
#include "cam_cmd.h"

#define TAG "TIMERCAM"

static camera_config_t camera_config = {
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz
    .xclk_freq_hz = 10000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_UXGA,//QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 15, //0-63 lower number means higher quality
    .fb_count = 3 //if more than one, i2s runs in continuous mode. Use only with JPEG
};


char CAM_LOG[] =
" _____ _                      ____                \r\n"
"|_   _(_)_ __ ___   ___ _ __ / ___|__ _ _ __ ___  \r\n"
"  | | | | '_ ` _ \\ / _ \\ '__| |   / _` | '_ ` _ \\ \r\n"
"  | | | | | | | | |  __/ |  | |__| (_| | | | | | |\r\n"
"  |_| |_|_| |_| |_|\\___|_|   \\____\\__,_|_| |_| |_|\r\n";


#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char * part_buf[64];
    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            continue ;
        } 
        uint32_t sig = *((uint32_t *)&fb->buf[fb->len - 4]);

  
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;

        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);

            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }

        esp_camera_fb_return(fb);
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps), 0x%x", (uint32_t)(_jpg_buf_len/1024), (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time, sig);
    }

    last_frame = 0;
    return res;
}

esp_err_t test_httpd_handler(httpd_req_t *req) {
    uint32_t buf_len;
    char *buf;

    char cmd_str[30] = {0};
    char value_str[30] = {0};

    buf_len = httpd_req_get_url_query_len(req) + 1;
    
    if (buf_len < 2) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    buf = (char *)malloc(buf_len);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (httpd_req_get_url_query_str(req, buf, buf_len) != ESP_OK) {
        goto decode_fail;
    }

    if(httpd_query_key_value(buf, "cmd", cmd_str, sizeof(cmd_str)) != ESP_OK) {
        goto decode_fail;
    }
    
    if(httpd_query_key_value(buf, "value", value_str, sizeof(value_str)) != ESP_OK) {
        goto decode_fail;
    }
    
    int cmd = atoi(cmd_str);
    int value = atoi(value_str);
    
    int respond_len = 0;
    uint8_t* respond_buff;
    respond_buff = DealConfigMsg(cmd, (uint8_t *)&value, 2, &respond_len);
    httpd_resp_send(req, (char *)respond_buff, respond_len);

    free(buf);
    return ESP_OK;

decode_fail:
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static void start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_handle_t stream_httpd = NULL;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);

    httpd_uri_t jpeg_stream_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = jpg_stream_httpd_handler,
        .user_ctx = NULL
    };

    httpd_uri_t test_uri = {
        .uri = "/config",
        .method = HTTP_GET,
        .handler = test_httpd_handler,
        .user_ctx = NULL
    };

    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        httpd_register_uri_handler(server, &test_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &jpeg_stream_uri);
    }

    ESP_LOGI(TAG, "Starting http server!");
}

void start_uart_server(void) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();   
    }

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            continue ;
        } 
        uint32_t sig = *((uint32_t *)&fb->buf[fb->len - 4]);

        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
        uart_frame_send(kImage, _jpg_buf, _jpg_buf_len, true);
        esp_camera_fb_return(fb);

        if(res != ESP_OK){
            break;
        }

        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps), 0x%x", 
                        (uint32_t)(_jpg_buf_len/1024), (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time, sig);
    }

    last_frame = 0;
}

void get_uart_data(int cmd_in, const uint8_t* data, int len) {
    int respond_len = 0;
    uint8_t* respond_buff;
    respond_buff = DealConfigMsg(cmd_in, data, len, &respond_len);
    uart_frame_send(cmd_in | 0x80, respond_buff, respond_len, false);
}

void app_main()
{
    gpio_pad_select_gpio(GPIO_NUM_33);
    gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_33, 1);

    uart_init();
    uart_set_cb(get_uart_data);

    led_init();
    led_brightness(512);
    
    esp_log_level_set(TAG, ESP_LOG_ERROR);
    esp_err_t err;
    printf("%s", CAM_LOG);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    err = esp_camera_init(&camera_config);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cam Init Error: 0x%x", err);
        for (;;) {
            uint8_t error_code = 0x01;
            uart_frame_send(kErrorOccur, &error_code, 1, false);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } else {
        ESP_LOGI(TAG,  "Cam Init Success");
    }

    InitTimerCamConfig();
    InitCamFun();

    // bm8563_test();
    if (GetDeviceMode() == kUart) {
        start_uart_server();
    } else {
        char wifi_ssid[36], wifi_pwd[36];
        for (;;) {
            if (GetWifiConfig(wifi_ssid, wifi_pwd)) {
                wifi_init_sta(ESP_WIFI_SSID, ESP_WIFI_PASS);
                wifi_wait_connect(portMAX_DELAY);
                start_webserver();
                break;
            } else {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
        }
    }

}
