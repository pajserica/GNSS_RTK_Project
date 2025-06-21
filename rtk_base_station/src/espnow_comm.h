#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include "esp_now.h"
#include "esp_err.h"
#include "gnss_module.h"

// ESP-NOW Configuration
#define ESPNOW_CHANNEL 1
#define ESPNOW_MAX_DATA_LEN 250

// Rover MAC Address - Update this with your rover's MAC
extern uint8_t rover_mac[6];

// ESP-NOW packet types
typedef enum {
    ESPNOW_PACKET_RTCM = 1,
    ESPNOW_PACKET_STATUS = 2,
    ESPNOW_PACKET_HEARTBEAT = 3
} espnow_packet_type_t;

// ESP-NOW packet structure
typedef struct {
    espnow_packet_type_t type;
    uint16_t sequence;
    uint16_t data_len;
    uint8_t data[ESPNOW_MAX_DATA_LEN];
} espnow_packet_t;

// Function declarations
esp_err_t espnow_init(void);
esp_err_t espnow_send_rtcm_data(const rtcm_data_t* rtcm_data);
void espnow_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status);
void espnow_recv_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

#endif // ESPNOW_COMM_H