#pragma once

// init bat hold gpio, adc gpio
void bat_init();

void bat_hold_output();

void bat_disable_output();

// 0 ~ 4096
uint32_t bat_get_adc_raw();

// return: xxxx mv
uint32_t bat_get_voltage();
