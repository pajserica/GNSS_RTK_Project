#include "gnss_module.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"

static const char *TAG = "GNSS";

static QueueHandle_t g_rtcm_queue = NULL;
static base_station_status_t* g_status = NULL;

bool is_rtcm_packet(const uint8_t* data, size_t length) {
    // RTCM packets start with 0xD3
    if (length < 3) return false;
    if (data[0] != 0xD3) return false;
    
    // Check message length
    uint16_t msg_length = ((data[1] & 0x03) << 8) | data[2];
    return (msg_length + 6 <= length); // +3 header +3 CRC
}

esp_err_t parse_nmea_sentence(const char* sentence, base_station_status_t* status) {
    if (strncmp(sentence, "$GPGSV", 6) == 0 || strncmp(sentence, "$GLGSV", 6) == 0) {
        // Parse satellite count from GSV sentence
        char* token = strtok((char*)sentence, ",");
        int field = 0;
        
        while (token != NULL && field < 4) {
            token = strtok(NULL, ",");
            field++;
            if (field == 3 && token != NULL) {
                status->satellite_count = atoi(token);
                break;
            }
        }
    }
    return ESP_OK;
}

void gnss_task(void *pvParameters) {
    uint8_t* data = (uint8_t*) malloc(GNSS_BUF_SIZE);
    char nmea_buffer[256];
    int nmea_pos = 0;
    
    while (1) {
        int length = uart_read_bytes(GNSS_UART_PORT, data, GNSS_BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        
        if (length > 0) {
            g_status->gnss_connected = true;
            
            for (int i = 0; i < length; i++) {
                uint8_t byte = data[i];
                
                // Check for RTCM data (binary)
                if (byte == 0xD3) {
                    // Start of potential RTCM packet
                    if (i + 2 < length) {
                        uint16_t rtcm_length = ((data[i+1] & 0x03) << 8) | data[i+2];
                        int total_length = rtcm_length + 6;
                        
                        if (i + total_length <= length) {
                            rtcm_data_t rtcm_data;
                            rtcm_data.length = total_length;
                            rtcm_data.timestamp = xTaskGetTickCount();
                            memcpy(rtcm_data.data, &data[i], total_length);
                            
                            if (xQueueSend(g_rtcm_queue, &rtcm_data, 0) == pdTRUE) {
                                ESP_LOGD(TAG, "RTCM packet queued, type: %d, length: %d", 
                                        (data[i+3] << 4) | (data[i+4] >> 4), total_length);
                            }
                            
                            i += total_length - 1; // Skip processed bytes
                            continue;
                        }
                    }
                }
                
                // Process NMEA sentences
                if (byte == '\r' || byte == '\n') {
                    if (nmea_pos > 0) {
                        nmea_buffer[nmea_pos] = '\0';
                        if (nmea_buffer[0] == '$') {
                            parse_nmea_sentence(nmea_buffer, g_status);
                        }
                        nmea_pos = 0;
                    }
                } else if (nmea_pos < sizeof(nmea_buffer) - 1) {
                    nmea_buffer[nmea_pos++] = byte;
                }
            }
        } else {
            // No data received - check connection
            static int no_data_count = 0;
            no_data_count++;
            if (no_data_count > 50) { // 5 seconds without data
                g_status->gnss_connected = false;
                no_data_count = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    free(data);
}

esp_err_t gnss_init(QueueHandle_t rtcm_queue, base_station_status_t* status) {
    g_rtcm_queue = rtcm_queue;
    g_status = status;
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = GNSS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    
    ESP_ERROR_CHECK(uart_param_config(GNSS_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GNSS_UART_PORT, GNSS_TX_PIN, GNSS_RX_PIN, 
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GNSS_UART_PORT, GNSS_BUF_SIZE * 2, 0, 0, NULL, 0));
    
    // Create GNSS task
    xTaskCreate(gnss_task, "gnss_task", 4096, NULL, 6, NULL);
    
    ESP_LOGI(TAG, "GNSS module initialized");
    return ESP_OK;
}