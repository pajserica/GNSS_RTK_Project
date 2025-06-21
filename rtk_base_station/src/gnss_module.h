#ifndef GNSS_MODULE_H
#define GNSS_MODULE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

// GNSS Configuration
#define GNSS_UART_PORT UART_NUM_2
#define GNSS_TX_PIN GPIO_NUM_17
#define GNSS_RX_PIN GPIO_NUM_16
#define GNSS_BAUD_RATE 115200
#define GNSS_BUF_SIZE 2048

// RTCM Data Structure
typedef struct {
    uint8_t data[1024];
    size_t length;
    uint32_t timestamp;
} rtcm_data_t;

// Base Station Status
typedef struct {
    const char* rtk_status;
    int satellite_count;
    uint32_t rtcm_packets_sent;
    uint32_t uptime_seconds;
    bool gnss_connected;
    bool rover_connected;
} base_station_status_t;

// Function declarations
esp_err_t gnss_init(QueueHandle_t rtcm_queue, base_station_status_t* status);
void gnss_task(void *pvParameters);
bool is_rtcm_packet(const uint8_t* data, size_t length);
esp_err_t parse_nmea_sentence(const char* sentence, base_station_status_t* status);

#endif // GNSS_MODULE_H