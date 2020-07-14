#include "bmm8563.h"
#include "driver/i2c.h"

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