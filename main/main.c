#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

#include "battery.h"
#include "bmm8563.h"
#include "led.h"
#include "network.h"
#include "image_post.h"
#include "esp_sleep.h"
#include "timer_cam_config.h"

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
    .frame_size = FRAMESIZE_VGA,//QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 15, //0-63 lower number means higher quality
    .fb_count = 3 //if more than one, i2s runs in continuous mode. Use only with JPEG
};

char CAM_LOGO[] =
" _____ _                      ____                \r\n"
"|_   _(_)_ __ ___   ___ _ __ / ___|__ _ _ __ ___  \r\n"
"  | | | | '_ ` _ \\ / _ \\ '__| |   / _` | '_ ` _ \\ \r\n"
"  | | | | | | | | |  __/ |  | |__| (_| | | | | | |\r\n"
"  |_| |_|_| |_| |_|\\___|_|   \\____\\__,_|_| |_| |_|\r\n";


#define POST_TIME 20

void app_main()
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("wifi", ESP_LOG_ERROR);
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    esp_log_level_set("camera", ESP_LOG_ERROR);
    esp_log_level_set("system_api", ESP_LOG_ERROR);
    esp_log_level_set("sccb", ESP_LOG_ERROR);

    bat_init();

    led_init(CAMERA_LED_GPIO);
    led_brightness(256);

    bmm8563_init();
    int16_t time_wake = bmm8563_setTimerIRQ(POST_TIME);

    printf("%s", CAM_LOGO);
    ESP_LOGI(TAG, "Cam will wake in next %d sec", time_wake);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
        ESP_ERROR_CHECK(ret);
    }

    wifi_init_sta("cam", "12345678");

    ret = esp_camera_init(&camera_config);

    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Cam Init Failed");
        goto exit;
    } else {
        ESP_LOGI(TAG, "Cam Init Success");
    }

    esp_task_wdt_init(1, false);
    esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(0));

    sensor_t *s = esp_camera_sensor_get();
    
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_vflip(s, 1);

    for (uint8_t i = 0; i < 5; i++) {
        camera_fb_t* fb = esp_camera_fb_get();
        esp_camera_fb_return(fb);
    }

    if(wifi_wait_connect(pdMS_TO_TICKS(8000))) {
        ESP_LOGI(TAG, "Connect to Wi-Fi Success");
    } else {
        ESP_LOGI(TAG, "Connect to Wi-Fi Failed");
        goto exit;
    }
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    char *token = NULL;
    asprintf(&token, "%x%x%x%x%x%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    camera_fb_t* fb = esp_camera_fb_get();
    const char *url = "http://192.168.0.104:5001/timer-cam/jpg";
    bool result =  http_post_image(url, token, fb->buf, fb->len, bat_get_voltage());

    if (result) {
        ESP_LOGI(TAG, "Post to server Success");
    } else {
        ESP_LOGE(TAG, "Post to server Failed");
    }

exit:
    bat_disable_output();

    ESP_LOGI(TAG, "Cam deep sleep start");
    esp_sleep_enable_timer_wakeup(POST_TIME * 1000000);
    esp_deep_sleep_start();
}
