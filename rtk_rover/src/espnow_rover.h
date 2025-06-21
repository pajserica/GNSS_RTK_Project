#ifndef ESPNOW_ROVER_H
#define ESPNOW_ROVER_H

#include "esp_now.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "gnss_rover.h"

// ESP-NOW Configuration for Rover
#define ESPNOW_ROVER_CHANNEL 1
#define ESPNOW_ROVER_MAX_DATA_LEN 250

// Base station MAC Address - Update this with your base station's MAC
extern uint8_t base_station_mac[6];

// ESP-NOW packet types
typedef enum {
    ESPNOW_ROVER_PACKET_RTCM = 1,
    ESPNOW_ROVER_PACKET_STATUS = 2,
    ESPNOW_ROVER_PACKET_HEARTBEAT = 3
} espnow_rover_packet_type_t;

// ESP-NOW packet structure for rover
typedef struct {
    espnow_rover_packet_type_t type;
    uint16_t sequence;
    uint16_t data_len;
    uint8_t data[ESPNOW_ROVER_MAX_DATA_LEN];
} espnow_rover_packet_t;

// Function declarations
esp_err_t espnow_rover_init(QueueHandle_t rtcm_queue, rover_status_t* status);
esp_err_t espnow_send_heartbeat(void);
esp_err_t espnow_send_status(void);
void espnow_rover_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status);
void espnow_rover_recv_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

#endif // ESPNOW_ROVER_H