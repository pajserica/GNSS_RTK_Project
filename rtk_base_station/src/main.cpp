#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "gnss_module.h"
#include "espnow_comm.h"
#include "wifi_ap.h"
#include "web_server.h"

static const char *TAG = "RTK_BASE";

// Global queues and handles
QueueHandle_t rtcm_queue;
SemaphoreHandle_t status_mutex;

// Global status structure
base_station_status_t base_status = {
    .rtk_status = "INITIALIZING",
    .satellite_count = 0,
    .rtcm_packets_sent = 0,
    .uptime_seconds = 0,
    .gnss_connected = false,
    .rover_connected = false
};

void status_update_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1 second
    
    while (1) {
        xSemaphoreTake(status_mutex, portMAX_DELAY);
        base_status.uptime_seconds++;
        xSemaphoreGive(status_mutex);
        
        ESP_LOGI(TAG, "Uptime: %lu, Satellites: %d, RTCM sent: %lu", 
                 base_status.uptime_seconds, 
                 base_status.satellite_count,
                 base_status.rtcm_packets_sent);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void rtcm_forwarding_task(void *pvParameters) {
    rtcm_data_t rtcm_data;
    
    while (1) {
        if (xQueueReceive(rtcm_queue, &rtcm_data, portMAX_DELAY)) {
            // Forward RTCM data via ESP-NOW
            if (espnow_send_rtcm_data(&rtcm_data) == ESP_OK) {
                xSemaphoreTake(status_mutex, portMAX_DELAY);
                base_status.rtcm_packets_sent++;
                xSemaphoreGive(status_mutex);
                
                ESP_LOGD(TAG, "RTCM packet forwarded, size: %d", rtcm_data.length);
            } else {
                ESP_LOGW(TAG, "Failed to forward RTCM data");
            }
        }
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "RTK Base Station starting...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create queues and mutexes
    rtcm_queue = xQueueCreate(10, sizeof(rtcm_data_t));
    status_mutex = xSemaphoreCreateMutex();
    
    if (rtcm_queue == NULL || status_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create queues/mutexes");
        return;
    }
    
    // Initialize WiFi AP
    ESP_LOGI(TAG, "Initializing WiFi AP...");
    if (wifi_ap_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi AP");
        return;
    }
    
    // Initialize ESP-NOW
    ESP_LOGI(TAG, "Initializing ESP-NOW...");
    if (espnow_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW");
        return;
    }
    
    // Initialize Web Server
    ESP_LOGI(TAG, "Starting web server...");
    if (web_server_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server");
        return;
    }
    
    // Initialize GNSS module
    ESP_LOGI(TAG, "Initializing GNSS module...");
    if (gnss_init(rtcm_queue, &base_status) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize GNSS module");
        return;
    }
    
    // Create tasks
    xTaskCreate(rtcm_forwarding_task, "rtcm_forward", 4096, NULL, 5, NULL);
    xTaskCreate(status_update_task, "status_update", 2048, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "RTK Base Station initialization complete");
    
    // Main loop - can be used for additional monitoring
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10 seconds
        ESP_LOGI(TAG, "Base station running... Free heap: %lu", esp_get_free_heap_size());
    }
}