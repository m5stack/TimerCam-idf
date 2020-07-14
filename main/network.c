#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "network.h"
#include "string.h"
#include "mdns.h"

#define TAG "Network"

#define ESP_WIFI_AP_SSID      "M5-Cam"
#define ESP_WIFI_AP_PASS      ""
#define MAX_STA_CONN       1

static EventGroupHandle_t wifi_event_group;
static ip4_addr_t ip_addr;

const int CONNECTED_BIT = BIT0;
const int CONNECTED_FAIL_BIT = BIT1;

static void init_mdns(void);

static esp_err_t event_handler(void* ctx, system_event_t* event) 
{ 
    static int connect_fail_nums = 0;
    static int reconnect_nums = 65535;

    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            xEventGroupClearBits(wifi_event_group, CONNECTED_FAIL_BIT);
            esp_wifi_connect();
            break;

        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            ip_addr = event->event_info.got_ip.ip_info.ip;
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            init_mdns();
            break;

        case SYSTEM_EVENT_AP_STACONNECTED:
            connect_fail_nums = 0;
            reconnect_nums = 65536;
            ESP_LOGI(TAG, "station:" MACSTR " join, AID=%d", MAC2STR(event->event_info.sta_connected.mac), event->event_info.sta_connected.aid);
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            xEventGroupClearBits(wifi_event_group, CONNECTED_FAIL_BIT);
            break;

        case SYSTEM_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d", MAC2STR(event->event_info.sta_disconnected.mac), event->event_info.sta_disconnected.aid);
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;

        case SYSTEM_EVENT_STA_DISCONNECTED:
            if (connect_fail_nums < reconnect_nums) {
                connect_fail_nums += 1;
                ESP_LOGI(TAG, "Reconnect %d", connect_fail_nums);
                esp_wifi_connect();
            } else {
                xEventGroupSetBits(wifi_event_group, CONNECTED_FAIL_BIT);
            }
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;

        default:
        break;
    }
    return ESP_OK;
}

void wifi_sta_connect(const char* ssid, const char* pwd) {
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    memcpy(wifi_config.sta.ssid, ssid, strlen(ssid));
    memcpy(wifi_config.sta.password, pwd, strlen(pwd));

    wifi_config.sta.ssid[strlen(ssid)] = '\0';
    wifi_config.sta.password[strlen(pwd)] = '\0';

    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, false, portMAX_DELAY);
}

WifiConnectStatus_t wifi_wait_connect(int32_t timeout) {
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, false, pdMS_TO_TICKS(timeout));
    return ((bits & CONNECTED_BIT) == CONNECTED_BIT);
}

void wifi_init_sta(const char* ssid, const char* pwd)
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = false;
    cfg.static_tx_buf_num = 16;
    cfg.dynamic_tx_buf_num = 32;
    cfg.static_rx_buf_num = 8;
    cfg.dynamic_rx_buf_num = 8;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    wifi_sta_connect(ssid, pwd);
}


int GetWifiConnectStatus() {
    EventBits_t bits = xEventGroupGetBits(wifi_event_group);
    if (bits & CONNECTED_BIT) {
        return CONNECT_SUCCESS;
    }

    if (bits & CONNECTED_FAIL_BIT) {
        return CONNECT_FAIL;
    }

    return CONNECTING;
}

uint32_t GetWifiIP() {
    return ip_addr.addr;
}

void wifi_init_ap(void) {
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.wifi_task_core_id = 0;
    cfg.nvs_enable = false;
    cfg.static_tx_buf_num = 16;
    cfg.dynamic_tx_buf_num = 32;
    cfg.static_rx_buf_num = 8;
    cfg.dynamic_rx_buf_num = 8;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_AP_SSID,
            .ssid_len = strlen(ESP_WIFI_AP_SSID),
            .password = ESP_WIFI_AP_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(ESP_WIFI_AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s", ESP_WIFI_AP_SSID, ESP_WIFI_AP_PASS);
}

static void init_mdns(void) {
    char *host_name; 
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    asprintf(&host_name, "%s-%02X%02X%02X", "Timer-Cam", mac[3], mac[4], mac[5]);
    ESP_ERROR_CHECK( mdns_init() );
    mdns_hostname_set(host_name);
    free(host_name);

    mdns_instance_name_set("timer-cam-mdns");

    char *mac_str;
    asprintf(&mac_str, MACSTR, MAC2STR(mac));

    mdns_txt_item_t serviceTxtData[3] = {
        {"board", "timer-cam-test"},
        {"version", "0.0.1"},
        {"mac", mac_str},
    };
    
    ESP_ERROR_CHECK(mdns_service_add("Timer_Cam", "_http", "_tcp", 80, serviceTxtData, 3));
    free(mac_str);
}