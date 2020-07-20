#ifndef _TIMERCAM_NETWORK_H_
#define _TIMERCAM_NETWORK_H_

typedef enum {
    CONNECTING,
    CONNECT_FAIL,
    CONNECT_SUCCESS,
    NOT_CONNECT,
} WifiConnectStatus_t;

void wifi_init_sta(const char* ssid, const char* pwd);

WifiConnectStatus_t wifi_wait_connect(int32_t timeout);

void wifi_init_ap(void);

int GetWifiConnectStatus();

uint32_t GetWifiIP();

#endif