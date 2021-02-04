#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "time.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <lwip/netdb.h>

#include "battery.h"
#include "bm8563.h"
#include "led.h"
#include "network.h"
#include "image_post.h"
#include "esp_sleep.h"
#include "timer_cam_config.h"
#include "base64.h"
#include "ext_info.h"

#include "alioss.h"

#define TAG "TIMERCAM"

static char* ssid = NULL;
static char* pwd = NULL;
static int wake_time = 60;
static int image_size = FRAMESIZE_SXGA;
static char* access_key;
static char* access_key_secret;
static char* server_addr;
static char* bucket; 

//access_key = "LTAI4xxxxxxxxxxxx";
//access_key_secret = "7JMCxIWlkwtnxxxxxxxxx";
//server_addr = "oss-cn-shenzhen.aliyuncs.com";
//bucket = "base-image-save";

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


uint32_t coverToTimestamp(rtc_date_t* data) {
    struct tm* tmp_time = (struct tm*)malloc(sizeof(struct tm));
    char time_str[100];
    sprintf(time_str, "%d-%d-%d %d:%d:%d\r\n", data->year, data->month, data->day, data->hour, data->minute, data->second);
    strptime(time_str,"%Y-%m-%d %H:%M:%S", tmp_time);
    time_t t = mktime(tmp_time);
    free(tmp_time);
    return (uint32_t)t;
}

void updateRTCTime(time_t time_diff) {
    time_t t_now;
    rtc_date_t data;
    bm8563_getTime(&data);
    t_now = coverToTimestamp(&data);
    t_now += time_diff;
    struct tm* time_now = localtime(&t_now);
    data.year = time_now->tm_year + 1900;
    data.month = time_now->tm_mon + 1;
    data.day = time_now->tm_mday;
    data.hour = time_now->tm_hour;
    data.minute = time_now->tm_min;
    data.second = time_now->tm_sec;
    bm8563_setTime(&data);
}

int16_t postImageToAlioss() {
    rtc_date_t data;
    bm8563_getTime(&data);
    camera_fb_t* fb = esp_camera_fb_get();
    oss_file_base_t file;
    file.data = fb->buf;
    file.len = fb->len;
    asprintf(&file.name, "%04d%02d%02dT%02d%02d%02dV%d.jpg", data.year, data.month, data.day, data.hour, data.minute, data.second, bat_get_voltage());
    file.type = "image/jpeg";
    file.access_key = access_key; 
    file.access_key_secret = access_key_secret;
    file.server_addr = server_addr;
    file.bucket = bucket;
    return oss_put_file(&file, coverToTimestamp(&data));
}

void app_main()
{
    // disable useless log
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("wifi", ESP_LOG_ERROR);
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    esp_log_level_set("camera", ESP_LOG_ERROR);
    esp_log_level_set("system_api", ESP_LOG_ERROR);
    esp_log_level_set("sccb", ESP_LOG_ERROR);

    // hold bat power before bm8563 clear irq
    bat_init();
    bm8563_init();
    led_init(CAMERA_LED_GPIO);
    led_brightness(128);
    esp_task_wdt_init(1, false);
    esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(0));

    printf("%s", CAM_LOGO);
    
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    int16_t result = ALIOSS_PUT_TIME_ERROR;
    TickType_t power_on_time = xTaskGetTickCount();

    // got base msg from flash
    ExtInfoInitAddr(0x3ff000);
    
    ret = ExtInfoGetString("ssid", &ssid);
    ret |= ExtInfoGetString("pwd", &pwd);
    ret |= ExtInfoGetInt("wake_time", &wake_time);
    ret |= ExtInfoGetInt("image_size", &image_size);

    ret |= ExtInfoGetString("access_key", &access_key);
    ret |= ExtInfoGetString("access_key_secret", &access_key_secret);
    ret |= ExtInfoGetString("server_url", &server_addr);
    ret |= ExtInfoGetString("bucket_name", &bucket);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error getting basic information from Flash");
        vTaskDelay(100);
        bat_disable_output();
        return ;
    }
    
    ESP_LOGI(TAG, "Connect to wifi %s", ssid);
    wifi_init_sta(ssid, pwd);

    camera_config.frame_size = FRAMESIZE_UXGA;
    ret = esp_camera_init(&camera_config);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cam Init Failed");
        goto exit;
    } else {
        ESP_LOGI(TAG, "Cam Init Success");
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);

    // wait cam sensor init
    for (uint8_t i = 0; i < 5; i++) {
        camera_fb_t* fb = esp_camera_fb_get();
        esp_camera_fb_return(fb);
    }

    if (wifi_wait_connect(pdMS_TO_TICKS(8000)) == false) {
        ESP_LOGE(TAG, "Connect wifi failed");
        goto exit;
    }

    ESP_LOGI(TAG, "Connect to Wi-Fi Success");

    result = postImageToAlioss();
    if (result == ALIOSS_PUT_TIME_ERROR) {
        updateRTCTime(getTimeDiff());
        result = postImageToAlioss();
    }

    if (result == ALIOSS_PUT_OK) {
        ESP_LOGI(TAG, "Post to alioss success");
    } else {
        ESP_LOGI(TAG, "Post to alioss failed");
    }

exit:
    if (result != ALIOSS_PUT_OK) {
        led_brightness(0);
        vTaskDelay(pdMS_TO_TICKS(100));
        led_brightness(256);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (bat_get_voltage() < 3300) {
        ESP_LOGI(TAG, "Volt too low");
    } else {
        ESP_LOGI(TAG, "Go to Sleep, %d sec", wake_time);

        if (wake_time > (xTaskGetTickCount() - power_on_time) / 1000) {
            bm8563_setTimerIRQ(wake_time - (xTaskGetTickCount() - power_on_time) / 1000);
        } else {
            bm8563_setTimerIRQ(1);
        }
    }

    bat_disable_output();

    ESP_LOGI(TAG, "Cam deep sleep start");
    esp_sleep_enable_timer_wakeup(wake_time * 1000000);
    esp_deep_sleep_start();
}
