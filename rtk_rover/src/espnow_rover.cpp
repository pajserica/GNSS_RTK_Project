#include "espnow_rover.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "string.h"

static const char *TAG = "ESP_NOW_ROVER";

// Base station MAC - should be updated with actual base station MAC
uint8_t base_station_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast initially

static uint16_t sequence_number = 0;
static QueueHandle_t g_rtcm_queue = NULL;
static rover_status_t* g_rover_status = NULL;

void espnow_rover_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "ESP-NOW send success to base station");
        g_rover_status->base_connected = true;
    } else {
        ESP_LOGW(TAG, "ESP-NOW send failed to base station");
        g_rover_status->base_connected = false;
    }
}

void espnow_rover_recv_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len < sizeof(espnow_rover_packet_t)) {
        ESP_LOGW(TAG, "Received packet too small");
        return;
    }
    
    espnow_rover_packet_t* packet = (espnow_rover_packet_t*)data;
    
    switch (packet->type) {
        case ESPNOW_ROVER_PACKET_RTCM:
            {
                rtcm_packet_t rtcm_packet;
                rtcm_packet.length = packet->data_len;
                rtcm_packet.timestamp = xTaskGetTickCount();
                memcpy(rtcm_packet.data, packet->data, packet->data_len);
                
                if (xQueueSend(g_rtcm_queue, &rtcm_packet, 0) == pdTRUE) {
                    ESP_LOGD(TAG, "RTCM packet received and queued, size: %d", packet->data_len);
                    g_rover_status->base_connected = true;
                } else {
                    ESP_LOGW(TAG, "RTCM queue full, dropping packet");
                }
            }
            break;
            
        case ESPNOW_ROVER_PACKET_STATUS:
            ESP_LOGD(TAG, "Status packet received from base station");
            g_rover_status->base_connected = true;
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown packet type received: %d", packet->type);
            break;
    }
    
    // Update base station MAC if needed
    memcpy(base_station_mac, recv_info->src_addr, 6);
}

esp_err_t espnow_send_heartbeat(void) {
    espnow_rover_packet_t packet;
    packet.type = ESPNOW_ROVER_PACKET_HEARTBEAT;
    packet.sequence = sequence_number++;
    packet.data_len = 0;
    
    esp_err_t result = esp_now_send(base_station_mac, (uint8_t*)&packet, 
                                   sizeof(espnow_rover_packet_t) - ESPNOW_ROVER_MAX_DATA_LEN);
    
    if (result != ESP_OK) {
        ESP_LOGD(TAG, "esp_now_send heartbeat failed: %s", esp_err_to_name(result));
    }
    
    return result;
}

esp_err_t espnow_send_status(void) {
    espnow_rover_packet_t packet;
    packet.type = ESPNOW_ROVER_PACKET_STATUS;
    packet.sequence = sequence_number++;
    
    // Pack rover status into data field
    uint8_t* data_ptr = packet.data;
    
    // Pack current position (12 bytes for 3 doubles as floats)
    *((float*)data_ptr) = (float)g_rover_status->current_position.latitude;
    data_ptr += 4;
    *((float*)data_ptr) = (float)g_rover_status->current_position.longitude;
    data_ptr += 4;
    *((float*)data_ptr) = (float)g_rover_status->current_position.altitude;
    data_ptr += 4;
    
    // Pack heading and speed (8 bytes)
    *((float*)data_ptr) = (float)g_rover_status->heading;
    data_ptr += 4;
    *((float*)data_ptr) = (float)g_rover_status->speed;
    data_ptr += 4;
    
    // Pack flags (4 bytes)
    *data_ptr++ = g_rover_status->rtk_fix ? 1 : 0;
    *data_ptr++ = g_rover_status->navigation_active ? 1 : 0;
    *data_ptr++ = g_rover_status->current_waypoint;
    *data_ptr++ = 0; // Reserved
    
    packet.data_len = 24; // Total packed data size
    
    esp_err_t result = esp_now_send(base_station_mac, (uint8_t*)&packet, 
                                   sizeof(espnow_rover_packet_t) - ESPNOW_ROVER_MAX_DATA_LEN + packet.data_len);
    
    if (result != ESP_OK) {
        ESP_LOGD(TAG, "esp_now_send status failed: %s", esp_err_to_name(result));
    }
    
    return result;
}

esp_err_t espnow_rover_init(QueueHandle_t rtcm_queue, rover_status_t* status) {
    g_rtcm_queue = rtcm_queue;
    g_rover_status = status;
    
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_rover_send_callback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_rover_recv_callback));
    
    // Set ESP-NOW channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_ROVER_CHANNEL, WIFI_SECOND_CHAN_NONE));
    
    // Add base station peer
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, base_station_mac, 6);
    peer_info.channel = ESPNOW_ROVER_CHANNEL;
    peer_info.encrypt = false;
    peer_info.ifidx = WIFI_IF_STA;
    
    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add ESP-NOW peer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "ESP-NOW rover initialized");
    return ESP_OK;
}