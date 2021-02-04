#pragma once

#define ALIOSS_PUT_OK 0
#define ALIOSS_PUT_TIME_ERROR -1
#define ALIOSS_POST_OTHER_ERROR -2

typedef struct {
    uint8_t* data;
    uint32_t len;

    char* name;
    char* type;

    char* server_addr;
    char* access_key;
    char* access_key_secret;
    char* bucket;
} oss_file_base_t;

int8_t oss_put_file(oss_file_base_t* file, uint32_t timestamp);

time_t getTimeDiff(void);