#ifndef WIFI_AP_H
#define WIFI_AP_H

#include "esp_err.h"
#include "esp_wifi.h"

// WiFi AP Configuration
#define WIFI_AP_SSID "RTK_Base_Station"
#define WIFI_AP_PASSWORD "rtk123456"
#define WIFI_AP_CHANNEL 6
#define WIFI_AP_MAX_CONNECTIONS 4

// Function declarations
esp_err_t wifi_ap_init(void);
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

#endif // WIFI_AP_H