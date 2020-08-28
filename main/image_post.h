#pragma once

#include "freertos/FreeRTOS.h"

bool http_post_image(const char* url, const char* tok, const uint8_t *data, uint32_t len, uint32_t voltage);
