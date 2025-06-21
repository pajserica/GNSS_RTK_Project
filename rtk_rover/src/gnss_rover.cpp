#include "gnss_rover.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "stdlib.h"

static const char *TAG = "GNSS_ROVER";

static QueueHandle_t g_rtcm_queue = NULL;
static SemaphoreHandle_t g_position_mutex = NULL;
static rover_status_t* g_rover_status = NULL;

bool is_rtk_fixed(const char* gga_sentence) {
    // Parse GGA sentence to check RTK fix quality
    // Quality indicator: 0=invalid, 1=GPS fix, 2=DGPS fix, 4=RTK fixed, 5=RTK float
    char* token = strtok((char*)gga_sentence, ",");
    int field = 0;
    
    while (token != NULL && field < 6) {
        token = strtok(NULL, ",");
        field++;
        if (field == 6 && token != NULL) {
            int quality = atoi(token);
            return (quality == 4); // RTK fixed
        }
    }
    return false;
}

esp_err_t parse_rover_nmea(const char* sentence, rover_status_t* status) {
    if (strncmp(sentence, "$GNGGA", 6) == 0 || strncmp(sentence, "$GPGGA", 6) == 0) {
        // Parse GGA sentence for position and RTK status
        char sentence_copy[256];
        strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
        sentence_copy[sizeof(sentence_copy) - 1] = '\0';
        
        char* token = strtok(sentence_copy, ",");
        int field = 0;
        double lat = 0, lon = 0, alt = 0;
        char lat_dir = 'N', lon_dir = 'E';
        int quality = 0;
        
        while (token != NULL) {
            switch (field) {
                case 2: // Latitude
                    if (strlen(token) > 0) {
                        double deg = atof(token) / 100.0;
                        int deg_int = (int)deg;
                        double min_frac = (deg - deg_int) * 100.0;
                        lat = deg_int + min_frac / 60.0;
                    }
                    break;
                case 3: // Latitude direction
                    lat_dir = token[0];
                    break;
                case 4: // Longitude
                    if (strlen(token) > 0) {
                        double deg = atof(token) / 100.0;
                        int deg_int = (int)deg;
                        double min_frac = (deg - deg_int) * 100.0;
                        lon = deg_int + min_frac / 60.0;
                    }
                    break;
                case 5: // Longitude direction
                    lon_dir = token[0];
                    break;
                case 6: // Fix quality
                    quality = atoi(token);
                    break;
                case 9: // Altitude
                    if (strlen(token) > 0) {
                        alt = atof(token);
                    }
                    break;
            }
            token = strtok(NULL, ",");
            field++;
        }
        
        // Update position if valid
        if (quality > 0 && lat != 0 && lon != 0) {
            xSemaphoreTake(g_position_mutex, portMAX_DELAY);
            status->current_position.latitude = (lat_dir == 'S') ? -lat : lat;
            status->current_position.longitude = (lon_dir == 'W') ? -lon : lon;
            status->current_position.altitude = alt;
            status->rtk_fix = (quality == 4); // RTK fixed
            xSemaphoreGive(g_position_mutex);
            
            ESP_LOGD(TAG, "Position updated: %.8f, %.8f, %.2f (Quality: %d)", 
                     status->current_position.latitude,
                     status->current_position.longitude, 
                     status->current_position.altitude,
                     quality);
        }
    }
    
    return ESP_OK;
}

void rtcm_forward_task(void *pvParameters) {
    rtcm_packet_t rtcm_packet;
    
    while (1) {
        if (xQueueReceive(g_rtcm_queue, &rtcm_packet, portMAX_DELAY)) {
            // Forward RTCM data to GNSS module via UART
            int written = uart_write_bytes(GNSS_ROVER_UART_PORT, 
                                         rtcm_packet.data, 
                                         rtcm_packet.length);
            
            if (written == rtcm_packet.length) {
                ESP_LOGD(TAG, "RTCM packet forwarded to GNSS, size: %d", rtcm_packet.length);
            } else {
                ESP_LOGW(TAG, "Failed to forward complete RTCM packet: %d/%d bytes", 
                         written, rtcm_packet.length);
            }
        }
    }
}

void gnss_rover_task(void *pvParameters) {
    uint8_t* data = (uint8_t*) malloc(GNSS_ROVER_BUF_SIZE);
    char nmea_buffer[256];
    int nmea_pos = 0;
    
    while (1) {
        int length = uart_read_bytes(GNSS_ROVER_UART_PORT, data, 
                                   GNSS_ROVER_BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        
        if (length > 0) {
            for (int i = 0; i < length; i++) {
                uint8_t byte = data[i];
                
                // Process NMEA sentences
                if (byte == '\r' || byte == '\n') {
                    if (nmea_pos > 0) {
                        nmea_buffer[nmea_pos] = '\0';
                        if (nmea_buffer[0] == '$') {
                            parse_rover_nmea(nmea_buffer, g_rover_status);
                        }
                        nmea_pos = 0;
                    }
                } else if (nmea_pos < sizeof(nmea_buffer) - 1) {
                    nmea_buffer[nmea_pos++] = byte;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    free(data);
}

esp_err_t gnss_rover_init(QueueHandle_t rtcm_queue, SemaphoreHandle_t pos_mutex, rover_status_t* status) {
    g_rtcm_queue = rtcm_queue;
    g_position_mutex = pos_mutex;
    g_rover_status = status;
    
    // Configure UART for GNSS communication
    uart_config_t uart_config = {
        .baud_rate = GNSS_ROVER_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    
    ESP_ERROR_CHECK(uart_param_config(GNSS_ROVER_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GNSS_ROVER_UART_PORT, GNSS_ROVER_TX_PIN, GNSS_ROVER_RX_PIN, 
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GNSS_ROVER_UART_PORT, GNSS_ROVER_BUF_SIZE * 2, 0, 0, NULL, 0));
    
    // Create tasks
    xTaskCreate(gnss_rover_task, "gnss_rover", 4096, NULL, 6, NULL);
    xTaskCreate(rtcm_forward_task, "rtcm_forward", 3072, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "GNSS rover module initialized");
    return ESP_OK;
}