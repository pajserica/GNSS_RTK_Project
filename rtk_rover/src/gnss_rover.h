#ifndef GNSS_ROVER_H
#define GNSS_ROVER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"

// GNSS Configuration for Rover
#define GNSS_ROVER_UART_PORT UART_NUM_2
#define GNSS_ROVER_TX_PIN GPIO_NUM_17
#define GNSS_ROVER_RX_PIN GPIO_NUM_16
#define GNSS_ROVER_BAUD_RATE 115200
#define GNSS_ROVER_BUF_SIZE 2048

// Coordinate structure
typedef struct {
    double latitude;
    double longitude;
    double altitude;
} coordinate_t;

// RTCM packet structure for rover
typedef struct {
    uint8_t data[1024];
    size_t length;
    uint32_t timestamp;
} rtcm_packet_t;

// Rover status structure
typedef struct {
    coordinate_t current_position;
    coordinate_t target_position;
    double heading;
    double speed;
    bool rtk_fix;
    bool base_connected;
    bool navigation_active;
    int current_waypoint;
} rover_status_t;

// Function declarations
esp_err_t gnss_rover_init(QueueHandle_t rtcm_queue, SemaphoreHandle_t pos_mutex, rover_status_t* status);
void gnss_rover_task(void *pvParameters);
void rtcm_forward_task(void *pvParameters);
esp_err_t parse_rover_nmea(const char* sentence, rover_status_t* status);
bool is_rtk_fixed(const char* gga_sentence);

#endif // GNSS_ROVER_H