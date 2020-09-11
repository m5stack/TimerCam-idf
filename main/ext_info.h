#pragma once

#include "esp_err.h"

esp_err_t ExtInfoInitAddr(uint32_t addr);

esp_err_t ExtInfoGetString(char *key, char **string);

esp_err_t ExtInfoGetInt(char *key, int* value);

esp_err_t ExtInfoGetDouble(char *key, double* value);

