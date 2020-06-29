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

#define TAG "TIMERCAM"

typedef int(*cmd_fun)(sensor_t *, int);

typedef struct _CamCmd_t {
    int cmd;
    cmd_fun fun;
    struct _CamCmd_t* next;    
} CamCmd_t;
CamCmd_t* cmd;

typedef enum {
    kImage = 0x00,
    kFrameSize,
    kQuality,
    kContrast,
    kBrightness,
    kSaturation,
    kGainceiling,
    kColorbar,
    kAwb,
    kAgc,
    kAec,
    kHmirror,
    kVflip,
    kAwbGain,
    kAgcGain,
    kAecValue,
    kAec2,
    kDcw,
    kBpc,
    kWpc,
    kRawGma,
    kLenc,
    kSpecialEffect,
    kWbMode,
    kAeLevel,
} CMD;

void AddCmdFun(CamCmd_t* cam_cmd,  int cmd, cmd_fun cmd_fun_in) {
    CamCmd_t* cam_cmd_new = cam_cmd;

    while (cam_cmd_new->next != NULL) {
        if (cam_cmd_new->next->cmd == cmd) {
            cam_cmd_new->next->fun = cmd_fun_in;
            return ;
        }
        cam_cmd_new = cam_cmd_new->next;
    }

    cam_cmd_new->next = calloc(1, sizeof(CamCmd_t));
    cam_cmd_new = cam_cmd_new->next;

    cam_cmd_new->cmd = cmd;
    cam_cmd_new->fun = cmd_fun_in;
    cam_cmd_new->next = NULL;
}

cmd_fun GetCmdFun(CamCmd_t* cam_cmd,  int cmd) {
    CamCmd_t* cam_cmd_new = cam_cmd;
    while (cam_cmd_new != NULL) {
        if (cam_cmd_new->cmd == cmd) {
            return cam_cmd_new->fun;
        }
        cam_cmd_new = cam_cmd_new->next;
    }

    return NULL;
}
// typedef int(*cmd_fun)(sensor_t *, int);

void call_cmd(CamCmd_t* cam_cmd, int cmd, int value) {
    cmd_fun fun = GetCmdFun(cam_cmd, cmd);
    sensor_t *s = esp_camera_sensor_get();
    if (fun == NULL) {
        return ;
    }
    (*fun)(s, value);
}


CamCmd_t* InitCamFun() {
    CamCmd_t* cmd = (CamCmd_t *)calloc(1, sizeof(CamCmd_t)); 
    sensor_t *s = esp_camera_sensor_get();
    AddCmdFun(cmd, kFrameSize,      (cmd_fun)s->set_framesize);
    AddCmdFun(cmd, kQuality,        (cmd_fun)s->set_quality);
    AddCmdFun(cmd, kContrast,       (cmd_fun)s->set_contrast);
    AddCmdFun(cmd, kBrightness,     (cmd_fun)s->set_brightness);
    AddCmdFun(cmd, kSaturation,     (cmd_fun)s->set_brightness);
    AddCmdFun(cmd, kGainceiling,    (cmd_fun)s->set_gainceiling);
    AddCmdFun(cmd, kColorbar,       (cmd_fun)s->set_colorbar);
    AddCmdFun(cmd, kAwb,            (cmd_fun)s->set_whitebal);
    AddCmdFun(cmd, kAgc,            (cmd_fun)s->set_gain_ctrl);
    AddCmdFun(cmd, kAec,            (cmd_fun)s->set_exposure_ctrl);
    AddCmdFun(cmd, kHmirror,        (cmd_fun)s->set_hmirror);
    AddCmdFun(cmd, kVflip,          (cmd_fun)s->set_vflip);
    AddCmdFun(cmd, kAwbGain,        (cmd_fun)s->set_awb_gain);
    AddCmdFun(cmd, kAgcGain,        (cmd_fun)s->set_agc_gain);
    AddCmdFun(cmd, kAecValue,       (cmd_fun)s->set_aec_value);
    AddCmdFun(cmd, kAec2,           (cmd_fun)s->set_aec2);
    AddCmdFun(cmd, kDcw,            (cmd_fun)s->set_dcw);
    AddCmdFun(cmd, kBpc,            (cmd_fun)s->set_bpc);
    AddCmdFun(cmd, kWpc,            (cmd_fun)s->set_wpc);
    AddCmdFun(cmd, kRawGma,         (cmd_fun)s->set_raw_gma);
    AddCmdFun(cmd, kLenc,           (cmd_fun)s->set_lenc);
    AddCmdFun(cmd, kSpecialEffect,  (cmd_fun)s->set_special_effect);
    AddCmdFun(cmd, kWbMode,         (cmd_fun)s->set_wb_mode);
    AddCmdFun(cmd, kAeLevel,        (cmd_fun)s->set_ae_level);
    return cmd;
} 

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

char CAM_LOG[] =
" _____ _                 ____                \r\n"
"|_   _(_)_ __ ___   ___ / ___|__ _ _ __ ___  \r\n"
"  | | | | '_ ` _ \\ / _ \\ |   / _` | '_ ` _ \\ \r\n"
"  | | | | | | | | |  __/ |__| (_| | | | | | |\r\n"
"  |_| |_|_| |_| |_|\\___|\\____\\__,_|_| |_| |_|\r\n";


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
        ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps), 0x%x",
            (uint32_t)(_jpg_buf_len/1024),
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time, sig);
    }

    last_frame = 0;
    return res;
}

static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);

    httpd_uri_t jpeg_stream_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = jpg_stream_httpd_handler,
        .user_ctx = NULL
    };

    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &jpeg_stream_uri);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void start_uart_server(void) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char * part_buf[64];
    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();   
    }

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
        uart_frame_send(kImage, _jpg_buf, _jpg_buf_len, false);
        esp_camera_fb_return(fb);

        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps), 0x%x",
            (uint32_t)(_jpg_buf_len/1024),
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time, sig);
    }

    last_frame = 0;
}

void get_uart_data(int cmd_in, const uint8_t* data, int len) {
    call_cmd(cmd, cmd_in, data[0]);
}

void app_main()
{
    uart_init();
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
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } else {
        ESP_LOGI(TAG,  "Cam Init Success");
    }

    cmd = InitCamFun();
    uart_set_cb(get_uart_data);
    
    call_cmd(cmd, kVflip, 1);
    call_cmd(cmd, kHmirror, 1);
    call_cmd(cmd, kFrameSize, 13);
    // sensor_t *s = esp_camera_sensor_get();
    // s->set_vflip(s, 1);

    // wifi_init_sta();
    // wifi_init_ap();
    // start_webserver();

    start_uart_server();
}
