#pragma once
#include "esp_log.h"

typedef struct _rtc_data_t {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} rtc_date_t;

void bm8563_init();

void bm8563_setTime(rtc_date_t* data);

void bm8563_getTime(rtc_date_t* data);

void bm8563_setAE(int8_t minute, int8_t hour, int8_t day, int8_t week);

void bm8563_setTE(uint8_t enable, uint8_t type, uint8_t value);

uint8_t bm8563_getTEValue();

uint8_t bm8563_getIRQ();

void bm8563_clearIRQ();

void bm8563_enableIRQ(uint8_t aie_enable, uint8_t tie_enable);

void bm8563_test();


