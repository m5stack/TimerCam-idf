
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "time.h"
#include "base64.h"
#include "esp32/rom/md5_hash.h"
#include "esp_http_client.h"
#include "alioss.h"
#include "esp_log.h"

extern int hmac_sha1(const uint8_t *, size_t , const uint8_t *, size_t data_len, uint8_t *);
static time_t t_diff;

char* generateMD5(const uint8_t* data, uint32_t len) {
    struct MD5Context ctx;
    MD5Init(&ctx);
    MD5Update(&ctx, (unsigned char*)data, len);
    uint8_t digest[16];
    MD5Final(digest, &ctx);
    size_t unuse_len;
    return (char *)base64_encode((unsigned char *)digest, 16, &unuse_len);
}

char* generateSignature(const char* access_key_secret, const char *http_method, const char* md5, const char* type, 
                    const char* date, const char* bucket_name, const char* file_name) {
    char* data;
    uint8_t hash[20];
    size_t len = 0;
    asprintf(&data, "%s\n%s\n%s\n%s\n/%s/%s", http_method, md5, type, date, bucket_name, file_name);
    hmac_sha1((uint8_t *)access_key_secret, strlen(access_key_secret), (uint8_t *)data, strlen(data), hash);
    free(data);
    return (char *)base64_encode((unsigned char *)hash, 20, &len);
}

int8_t oss_put_file(oss_file_base_t* file, uint32_t timestamp) {
    char gmt_time[100];
    char *md5;
    char *signature;
    char *authorization;
    char *url;
    char *length;
    
    time_t time_now = timestamp;
    strftime(gmt_time, 100, "%a, %d %b %Y %H:%M:%S GMT", gmtime(&time_now));
    md5  = generateMD5(file->data, file->len);
    signature = generateSignature(file->access_key_secret, "PUT", md5, file->type, gmt_time, file->bucket, file->name);
    asprintf(&authorization, "OSS %s:%s", file->access_key, signature);
    asprintf(&url, "http://%s.%s/%s", file->bucket, file->server_addr, file->name);
    asprintf(&length, "%d", file->len);

    esp_http_client_config_t config = {
        .url = url,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_method(client, HTTP_METHOD_PUT);
    esp_http_client_set_header(client, "content-type", file->type);
    esp_http_client_set_header(client, "content-md5", md5);
    esp_http_client_set_header(client, "content-length", length);
    esp_http_client_set_header(client, "date", gmt_time);
    esp_http_client_set_header(client, "authorization", authorization);
    esp_http_client_set_post_field(client, (char *)file->data, file->len);

    esp_err_t err = esp_http_client_perform(client);

    int8_t result = ALIOSS_POST_OTHER_ERROR;
    if (err != ESP_OK) {
        result = ALIOSS_POST_OTHER_ERROR;
        goto exit;
    }

    int status_code = esp_http_client_get_status_code(client);

    if (status_code == 403) {
        result = ALIOSS_POST_OTHER_ERROR;
        uint8_t *buffer = (uint8_t *)malloc(2048 * sizeof(uint8_t));
        int content_length =  esp_http_client_read(client, (char *)buffer, 2048);
        if (content_length > 10) {
            buffer[content_length - 1] = '\0';
            if (strstr((char *)buffer, "RequestTimeTooSkewed") != NULL) {
                char *request_time_start = strstr((char *)buffer, "<RequestTime>");
                char *request_time_end = strstr((char *)buffer, "</RequestTime>");

                char *real_time_start = strstr((char *)buffer, "<ServerTime>");
                char *real_time_end = strstr((char *)buffer, "</ServerTime>");
                if (request_time_start && request_time_end && real_time_start && real_time_end) {
                    *request_time_end = '\0';
                    *real_time_end = '\0';
                    request_time_start += strlen("<RequestTime>");
                    real_time_start += strlen("<ServerTime>");

                    struct tm tmp_time;
                    time_t t_req, t_real;
                    strptime(real_time_start, "%Y-%m-%dT%H:%M:%S.000Z", &tmp_time);
                    t_real = mktime(&tmp_time);
                    strptime(request_time_start, "%Y-%m-%dT%H:%M:%S.000Z", &tmp_time);
                    t_req = mktime(&tmp_time);
                    t_diff = t_real - t_req;
                    printf("%s - %s \r\n", request_time_start, real_time_start);
                    result = ALIOSS_PUT_TIME_ERROR;
                }
            }
        }
        free(buffer);
    } else if (status_code == 200) {
        result = ALIOSS_PUT_OK;
    }

exit:
    free(md5);
    free(signature);
    free(authorization);
    free(url);
    free(length);

    return result;
}

time_t getTimeDiff(void) {
    return t_diff;
}
