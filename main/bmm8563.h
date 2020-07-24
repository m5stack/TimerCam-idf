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

void bm8563_setDateIRQ(int8_t minute, int8_t hour, int8_t day, int8_t week);

int16_t bm8563_setTimerIRQ(int16_t value);

int16_t bm8563_getTimerTime();

uint8_t bm8563_getIRQ();

void bm8563_clearIRQ();

void bm8563_test();


