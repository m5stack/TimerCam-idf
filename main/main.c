#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

#include "timer_cam_config.h"
#include "battery.h"
#include "bmm8563.h"
#include "led.h"
#include "network.h"
#include "uart_frame.h"
#include "protocol.h"
#include "cam_cmd.h"
#include "app_httpd.h"

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
    .xclk_freq_hz = CAM_XCLK_FREQ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_UXGA,//QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 15, //0-63 lower number means higher quality
    .fb_count = 3 //if more than one, i2s runs in continuous mode. Use only with JPEG
};


char CAM_LOGO[] =
" _____ _                      ____                \r\n"
"|_   _(_)_ __ ___   ___ _ __ / ___|__ _ _ __ ___  \r\n"
"  | | | | '_ ` _ \\ / _ \\ '__| |   / _` | '_ ` _ \\ \r\n"
"  | | | | | | | | |  __/ |  | |__| (_| | | | | | |\r\n"
"  |_| |_|_| |_| |_|\\___|_|   \\____\\__,_|_| |_| |_|\r\n";

void start_uart_server(void) {
    camera_fb_t * fb = NULL;
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
            continue ;
        } 
        uint32_t sig = *((uint32_t *)&fb->buf[fb->len - 4]);

        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;

        if (!(_jpg_buf[_jpg_buf_len - 1] != 0xd9 || _jpg_buf[_jpg_buf_len - 2] != 0xd9)) {
            esp_camera_fb_return(fb);
            continue;
        }

        uart_frame_send(kImage, _jpg_buf, _jpg_buf_len, true);
        esp_camera_fb_return(fb);

        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps), 0x%x", 
                    (uint32_t)(_jpg_buf_len/1024), (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time, sig);
    }

    last_frame = 0;
}

bool restart = false;
void frame_post_callback(uint8_t cmd) {
    if (restart && (cmd == (kSetDeviceMode | 0x80))) {
        esp_restart();
    }
}

void frame_recv_callback(int cmd_in, const uint8_t* data, int len) {
    int respond_len = 0;
    uint8_t* respond_buff;
    
    if (cmd_in == kFactoryTest) {
        extern void factory_test();
        factory_test();
        return ;
    }

    if (cmd_in == kSetDeviceMode || GetDeviceMode() != data[0]) {
        restart = true;
    }

    respond_buff = DealConfigMsg(cmd_in, data, len, &respond_len);
    uart_frame_send(cmd_in | 0x80, respond_buff, respond_len, false);
}



void app_main()
{
    bat_init();
    
    led_init(CAMERA_LED_GPIO);
    led_brightness(256);

    uart_init();
    bmm8563_init();

    esp_log_level_set(TAG, ESP_LOG_ERROR);
    printf("%s", CAM_LOGO);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = esp_camera_init(&camera_config);

    if (ret != ESP_OK) {
        for (;;) {
            uint8_t error_code = kCamError;
            uart_frame_send(kErrorOccur, &error_code, 1, false);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    } else {
        ESP_LOGI(TAG,  "Cam Init Success");
    }

    InitTimerCamConfig();
    InitCamFun();

    esp_task_wdt_init(1, false);
    esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(0));

    if (GetDeviceMode() == kUart) {
        start_uart_server();
    } else {
        char wifi_ssid[36], wifi_pwd[36];
        while (GetWifiConfig(wifi_ssid, wifi_pwd) == false) {
            uint8_t error_code = kWifiMsgError;
            uart_frame_send(kErrorOccur, &error_code, 1, false);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        start_webserver(wifi_ssid, wifi_pwd);
    }
}
