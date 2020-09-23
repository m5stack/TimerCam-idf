#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "driver/i2c.h"
#include "timer_cam_config.h"

#include "battery.h"
#include "bm8563.h"
#include "uart_frame.h"


static EventGroupHandle_t wifi_event_group;//定义一个事件的句柄
const int SCAN_DONE_BIT = BIT0;//定义事件，占用事件变量的第0位，最多可以定义32个事件。
int8_t max_rssi = 0;
bool was_init = false;

#define I2C_NUM I2C_NUM_1

static void i2c_init() {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)Ext_PIN_1;
	conf.scl_io_num = (gpio_num_t)Ext_PIN_2;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));
}

static bool i2c_test(uint8_t slave_addr) {
    i2c_cmd_handle_t cmd;
    esp_err_t err;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
    return err;
}

static wifi_scan_config_t scanConf  = {
    .ssid = NULL,
    .bssid = NULL,
    .channel = 0,
    .show_hidden = 1
};

static esp_err_t event_handler(void *ctx, system_event_t *event) {
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            break;
        case SYSTEM_EVENT_SCAN_DONE:
            xEventGroupSetBits(wifi_event_group, SCAN_DONE_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void scan_task(void *pvParameters) {
    while(1) {        
        ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, true));//扫描所有可用的AP。

        uint16_t apCount = 0;
        esp_wifi_scan_get_ap_num(&apCount);//Get number of APs found in last scan
        
        wifi_ap_record_t *list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * apCount);//定义一个wifi_ap_record_t的结构体的链表空间
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, list));//获取上次扫描中找到的AP列表。
        max_rssi = list[0].rssi;
        free(list);
    }
}

void bm8563_task(void *arg) {
    rtc_date_t date;
    uint8_t out_buf[4] = {0x00, 0x00, 0x00, 0x00};
    for (;;) {
        bm8563_getTime(&date);
        out_buf[0] = date.second;
        out_buf[1] = i2c_test(0x44);
        out_buf[2] = (uint8_t)max_rssi;
        out_buf[3] = (bat_get_adc_raw() > 2000) ? 1 : 0;
        uart_frame_send(kFactoryTest, out_buf, 4, false);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void factory_test() {
    if (was_init) {
        return ; 
    }
    was_init = true;
    wifi_event_group = xEventGroupCreate();    //创建一个事件标志组

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = false;
    cfg.static_tx_buf_num = 24;
    cfg.static_rx_buf_num = 8;
    cfg.dynamic_rx_buf_num = 8;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));//Set the WiFi operating mode
    ESP_ERROR_CHECK(esp_wifi_start());
    i2c_init();

    xTaskCreatePinnedToCore(&scan_task, "scan_task", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(&bm8563_task, "rtc_task", 2048, NULL, 1, NULL, 1);
}

