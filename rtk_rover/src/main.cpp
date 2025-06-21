#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_now.h"
#include "nvs_flash.h"

#include "gnss_rover.h"
#include "espnow_rover.h"
#include "imu_module.h"
#include "motor_control.h"
#include "navigation.h"

static const char *TAG = "RTK_ROVER";

// Global queues and handles
QueueHandle_t rtcm_receive_queue;
QueueHandle_t navigation_queue;
SemaphoreHandle_t position_mutex;

// Global rover status
rover_status_t rover_status = {
    .current_position = {0, 0, 0},
    .target_position = {0, 0, 0},
    .heading = 0.0,
    .speed = 0.0,
    .rtk_fix = false,
    .base_connected = false,
    .navigation_active = false,
    .current_waypoint = 0
};

// Predefined waypoints (example coordinates - replace with your actual coordinates)
const coordinate_t waypoints[] = {
    {45.123456, 19.654321, 150.0},  // Point 1
    {45.123556, 19.654421, 150.0},  // Point 2  
    {45.123656, 19.654521, 150.0},  // Point 3
    {45.123756, 19.654621, 150.0},  // Point 4
    {45.123456, 19.654321, 150.0}   // Return to start
};
const int num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);

void heartbeat_task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(5000); // 5 seconds
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // Send heartbeat to base station
        espnow_send_heartbeat();
        
        // Log status
        ESP_LOGI(TAG, "Rover Status - RTK: %s, Base: %s, Nav: %s, WP: %d/%d", 
                 rover_status.rtk_fix ? "FIX" : "NO_FIX",
                 rover_status.base_connected ? "CONN" : "DISC", 
                 rover_status.navigation_active ? "ACTIVE" : "IDLE",
                 rover_status.current_waypoint + 1, num_waypoints);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void navigation_control_task(void *pvParameters) {
    navigation_command_t nav_cmd;
    
    while (1) {
        if (xQueueReceive(navigation_queue, &nav_cmd, pdMS_TO_TICKS(100))) {
            // Process navigation command
            switch (nav_cmd.command) {
                case NAV_CMD_START:
                    rover_status.navigation_active = true;
                    rover_status.current_waypoint = 0;
                    ESP_LOGI(TAG, "Navigation started");
                    break;
                    
                case NAV_CMD_STOP:
                    rover_status.navigation_active = false;
                    motor_stop();
                    ESP_LOGI(TAG, "Navigation stopped");
                    break;
                    
                case NAV_CMD_NEXT_WAYPOINT:
                    if (rover_status.current_waypoint < num_waypoints - 1) {
                        rover_status.current_waypoint++;
                        ESP_LOGI(TAG, "Moving to waypoint %d", rover_status.current_waypoint + 1);
                    } else {
                        rover_status.navigation_active = false;
                        motor_stop();
                        ESP_LOGI(TAG, "Mission completed - all waypoints reached");
                    }
                    break;
            }
        }
        
        // Main navigation logic
        if (rover_status.navigation_active && rover_status.rtk_fix) {
            xSemaphoreTake(position_mutex, portMAX_DELAY);
            coordinate_t current = rover_status.current_position;
            xSemaphoreGive(position_mutex);
            
            coordinate_t target = waypoints[rover_status.current_waypoint];
            rover_status.target_position = target;
            
            // Calculate distance to target
            double distance = calculate_distance(&current, &target);
            double bearing = calculate_bearing(&current, &target);
            
            ESP_LOGD(TAG, "Distance to target: %.2f m, Bearing: %.1f°", distance, bearing);
            
            if (distance < 2.0) { // Within 2 meters of target
                ESP_LOGI(TAG, "Waypoint %d reached!", rover_status.current_waypoint + 1);
                navigation_command_t next_cmd = {.command = NAV_CMD_NEXT_WAYPOINT};
                xQueueSend(navigation_queue, &next_cmd, 0);
            } else {
                // Navigate towards target
                navigate_to_target(bearing, distance);
            }
        } else if (rover_status.navigation_active && !rover_status.rtk_fix) {
            // Stop if we lose RTK fix
            motor_stop();
            ESP_LOGW(TAG, "Navigation paused - waiting for RTK fix");
        }
        
        vTaskDelay(pdMS_TO_TICKS(200)); // 5 Hz navigation update
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "RTK Rover starting...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create queues and mutexes
    rtcm_receive_queue = xQueueCreate(10, sizeof(rtcm_packet_t));
    navigation_queue = xQueueCreate(5, sizeof(navigation_command_t));
    position_mutex = xSemaphoreCreateMutex();
    
    if (rtcm_receive_queue == NULL || navigation_queue == NULL || position_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create queues/mutexes");
        return;
    }
    
    // Initialize WiFi (needed for ESP-NOW)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Initialize ESP-NOW
    ESP_LOGI(TAG, "Initializing ESP-NOW...");
    if (espnow_rover_init(rtcm_receive_queue, &rover_status) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW");
        return;
    }
    
    // Initialize IMU
    ESP_LOGI(TAG, "Initializing IMU...");
    if (imu_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IMU");
        return;
    }
    
    // Initialize motor control
    ESP_LOGI(TAG, "Initializing motor control...");
    if (motor_control_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize motor control");
        return;
    }
    
    // Initialize GNSS rover module
    ESP_LOGI(TAG, "Initializing GNSS rover module...");
    if (gnss_rover_init(rtcm_receive_queue, position_mutex, &rover_status) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize GNSS rover module");
        return;
    }
    
    // Create tasks
    xTaskCreate(heartbeat_task, "heartbeat", 2048, NULL, 3, NULL);
    xTaskCreate(navigation_control_task, "navigation", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "RTK Rover initialization complete");
    
    // Wait a bit then start navigation
    vTaskDelay(pdMS_TO_TICKS(10000)); // Wait 10 seconds for systems to stabilize
    
    navigation_command_t start_cmd = {.command = NAV_CMD_START};
    xQueueSend(navigation_queue, &start_cmd, 0);
    
    // Main loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10 seconds
        ESP_LOGI(TAG, "Rover running... Free heap: %lu", esp_get_free_heap_size());
    }
}