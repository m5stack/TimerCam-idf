#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "config.h"
#include "esp_camera.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "driver/i2c.h"
#include "network.h"
#include "uart_frame.h"

#define TAG "TIMERCAM"
extern void bm8563_test();
extern void led_brightness(int duty);
extern void led_init();

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
    fun(s, value);
    // (*fun)(s, value);
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
    .xclk_freq_hz = 20000000,
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
    gpio_pad_select_gpio(GPIO_NUM_33);
    gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_33, 1);

    uart_init();
    led_init();
    led_brightness(512);
    // esp_log_level_set(TAG, ESP_LOG_ERROR);
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
    call_cmd(cmd, kFrameSize, FRAMESIZE_SVGA);
    // sensor_t *s = esp_camera_sensor_get();
    // s->set_vflip(s, 1);
    // bm8563_test();
    wifi_init_sta();
    // wifi_init_ap();
    start_webserver();

    // start_uart_server();
}

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d, line = %d", rc, __LINE__); } } while(0);
#define I2C_NUM 0

void i2c_init() {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)12;
	conf.scl_io_num = (gpio_num_t)14;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));
}

void i2c_write(uint8_t slave_addr, uint8_t addr, uint8_t* buf, uint8_t len) {
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, addr, 1));
	ESP_ERROR_CHECK(i2c_master_write(cmd, buf, len, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);
}

void i2c_write_byte(uint8_t slave_addr, uint8_t addr, uint8_t data) {
    i2c_write(slave_addr, addr, &data, 1);
}

uint8_t i2c_read(uint8_t slave_addr, uint8_t addr, uint8_t* buf, uint8_t len) {
    i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, addr, 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, 1));
	
    if (len>1) {
        ESP_ERROR_CHECK(i2c_master_read(cmd, buf, len - 1,I2C_MASTER_ACK));
    }

	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &buf[len-1], I2C_MASTER_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);
    return 0;
}

typedef struct _rtc_data_t {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} rtc_date_t;

static uint8_t byte2BCD(uint8_t data) {
    return ((data / 10) << 4) + data % 10;
}

static uint8_t BCD2Byte(uint8_t data) {
    return (data >> 4) * 10 + (data & 0x0f);
}

void bm8563_init() {
    i2c_init();
    i2c_write_byte(0x51, 0x00, 0x00);
    i2c_write_byte(0x51, 0x01, 0x00);
}

void bm8563_setTime(rtc_date_t* data) {
    if (data == NULL) {
        return ;
    }
    uint8_t time_buf[7];
    time_buf[0] = byte2BCD(data->second);
    time_buf[1] = byte2BCD(data->minute);
    time_buf[2] = byte2BCD(data->hour);
    time_buf[3] = byte2BCD(data->day);
    time_buf[5] = byte2BCD(data->month) | (data->year >= 2000 ? 0x00 : 0x80);
    time_buf[6] = byte2BCD(data->year % 100);
    i2c_write(0x51, 0x02, time_buf, 7);
}

void bm8563_getTime(rtc_date_t* data) {
    if (data == NULL) {
        return ;
    }
    uint8_t time_buf[7];
    i2c_read(0x51, 0x02, time_buf, 7);
    data->second = BCD2Byte(time_buf[0] & 0x7f);
    data->minute = BCD2Byte(time_buf[1] & 0x7f);
    data->hour = BCD2Byte(time_buf[2] & 0x3f);
    data->day = BCD2Byte(time_buf[3] & 0x3f);
    data->month = BCD2Byte(time_buf[5] & 0x07);
    data->year = BCD2Byte(time_buf[6]) + (time_buf[5] & 0x80 ? 1900 : 2000);
}

// -1 :disable
void bm8563_setAE(int8_t minute, int8_t hour, int8_t day, int8_t week) {
    uint8_t out_buf[4] = { 0x80, 0x80, 0x80, 0x80 };
    if(minute >= 0) {
        out_buf[0] = byte2BCD(minute) & 0x7f;
    }

    if(hour >= 0) {
        out_buf[1] = byte2BCD(hour) & 0x3f;
    }

    if(day >= 0) {
        out_buf[2] = byte2BCD(day) & 0x3f;
    }

    if(week >= 0) {
        out_buf[3] = byte2BCD(week) & 0x07;
    }

    i2c_write(0x51, 0x09, out_buf, 4);
}

// type: 0, 4096hz, 1: 64HZ, 2: 1HZ, 3: 1/60HZ
void bm8563_setTE(uint8_t enable, uint8_t type, uint8_t value) {
    uint8_t out_buf[2];
    if (enable) {
        out_buf[0] = 0x80;
    } else {
        out_buf[0] = 0x00;
    }

    out_buf[0] = out_buf[0] | (type & 0x03);
    out_buf[1] = value;
    i2c_write(0x51, 0x0e, out_buf, 2);
}

uint8_t bm8563_getTEValue() {
    uint8_t data;
    i2c_read(0x51, 0x0f, &data, 1);
    return data;
}

uint8_t bm8563_getIRQ() {
    uint8_t data;
    i2c_read(0x51, 0x01, &data, 1);
    return data;
    // return (data & 0x0c) >> 2;
}

void bm8563_clearIRQ() {
    uint8_t data;
    i2c_read(0x51, 0x01, &data, 1);
    i2c_write_byte(0x51, 0x01, data & 0xf3);
}

void bm8563_enableIRQ(uint8_t aie_enable, uint8_t tie_enable) {
    uint8_t data;
    i2c_read(0x51, 0x01, &data, 1);
    data = data & 0xfc;
    data = data | (aie_enable ? (0x01 << 1) : 0x00);
    data = data | (tie_enable ? (0x01 << 0) : 0x00);
    i2c_write_byte(0x51, 0x01, data);
}

void bm8563_test() {
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    bm8563_init();
    bm8563_enableIRQ(0, 1);
    bm8563_setTE(1, 2, 10);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(GPIO_NUM_33, 0);
    
    rtc_date_t date;
    // memset(&date, 0, sizeof(rtc_date_t));
    // date.year = 2020;
    // date.month = 5;
    // date.day = 14;
    // date.hour = 19;
    // date.minute = 50;
    // bm8563_setTime(&date);

    uint8_t irq; 
    for(;;) {
        // bm8563_getTime(&date);
        // printf("y: %d, m: %d, d: %d, h: %d, m:%d, s: %d\r\n", 
        //     date.year, date.month, date.day, date.hour, date.minute, date.second);
        vTaskDelay(pdMS_TO_TICKS(1000));
        irq = bm8563_getIRQ();
        bm8563_getTime(&date);
        printf("IRQ: %x, %d\r\n", irq, date.second);
        if(irq) {
            bm8563_clearIRQ();
        }
    }
}