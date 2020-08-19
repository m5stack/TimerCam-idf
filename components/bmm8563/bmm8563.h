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

void bmm8563_init();

void bmm8563_setTime(rtc_date_t* data);

void bmm8563_getTime(rtc_date_t* data);

void bmm8563_setDateIRQ(int8_t minute, int8_t hour, int8_t day, int8_t week);

int16_t bmm8563_setTimerIRQ(int16_t value);

int16_t bmm8563_getTimerTime();

uint8_t bmm8563_getIRQ();

void bmm8563_clearIRQ();

void bmm8563_test();


