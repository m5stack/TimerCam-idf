#include "protocol.h"
#include "string.h"
#include "network.h"
#include "cam_cmd.h"

uint8_t respond_buff[1024];

#define IsCamCmd(cmd) (cmd < kCamCmdEnd)

static bool DealWifiData(const uint8_t* data, int len) {
    if (len < 4) {
        return false;
    } 

    int ssid_len = data[0];
    if (ssid_len > (len - 1) || data[ssid_len] != '\0') {
        // ssid length is too long or pwd str not found
        return false;
    }

    int pwd_len = data[ssid_len + 1];
    if ((pwd_len + ssid_len + 2 ) > len || data[ssid_len + pwd_len + 1] != '\0') {
        // pwd length is too long or pwd str not found
        return false;
    }

    const char* ssid_ptr = (char *)&data[1];
    const char* pwd_ptr = (char *)&data[ssid_len + 2];
    return UpdateWifiConfig(ssid_ptr, pwd_ptr);
}

uint8_t* DealConfigMsg(int cmd_in, const uint8_t* data, int len, int *out_len) {
    uint8_t* result = respond_buff;
    uint32_t ip;

    if (IsCamCmd(cmd_in)) {
        if (len == 2) {
            int16_t value = (data[1] << 8) | (data[0]);
            printf("%d\r\n",value);
            esp_err_t err = CallCamCmd(cmd_in, value);
            if (err == ESP_OK) {
                respond_buff[0] = 1;
            } else {
                respond_buff[0] = 0;
            }
            *out_len = 1;
            return respond_buff;
        }

    }

    bool run_success;
    switch (cmd_in) {
        case kSetDeviceMode:
            UpdateDeviceMode(data[0]);
            SaveTimerCamConfig();
            goto config_no_respond;
            break;

        case kGetDeviceMode:
            result[0] = GetDeviceMode();
            *out_len = 1;
            break;

        case kGetCamConfig:
            result = GetCamConfig(out_len);
            break;
        
        case kSaveCamConfig:
            SaveCamConfig();
            goto config_no_respond;
            break;
        
        case kSaveDeviceConfig:
            SaveTimerCamConfig();
            goto config_no_respond;
            break;

        case kSetWiFi:
            run_success = DealWifiData(data, len);
            if (run_success) {
                SaveTimerCamConfig();
                result[0] = 1;
            } else {
                result[0] = 0;
            }
            *out_len = 1;
            break;
    
        case kGetWifiSSID:
            result = (uint8_t *)GetWifiSSID();
            *out_len = strlen((char *)result) + 1;
            break;

        case kGetWifiIP:
            ip = GetWifiIP();
            memcpy(result, (uint8_t*)&ip, 4);
            *out_len = 4;
            break;

        case kGetWifiState:
            result[0] = GetWifiConnectStatus();
            *out_len = 1;
            break;

        default:
            goto config_no_respond;
            break;
    }

    return result;

config_no_respond:
    result[0] = 1;
    *out_len = 1;
    return result;
}


