#include "espnow_comm.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "string.h"

static const char *TAG = "ESP_NOW";

// Default rover MAC - should be updated with actual rover MAC
uint8_t rover_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast initially

static uint16_t sequence_number = 0;
extern base_station_status_t base_status;

void espnow_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "ESP-NOW send success");
        base_status.rover_connected = true;
    } else {
        ESP_LOGW(TAG, "ESP-NOW send failed");
        base_status.rover_connected = false;
    }
}

void espnow_recv_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len < sizeof(espnow_packet_t)) {
        ESP_LOGW(TAG, "Received packet too small");
        return;
    }
    
    espnow_packet_t* packet = (espnow_packet_t*)data;
    
    switch (packet->type) {
        case ESPNOW_PACKET_HEARTBEAT:
            ESP_LOGD(TAG, "Heartbeat received from rover");
            base_status.rover_connected = true;
            // Update rover MAC if needed
            memcpy(rover_mac, recv_info->src_addr, 6);
            break;
            
        case ESPNOW_PACKET_STATUS:
            ESP_LOGD(TAG, "Status packet received from rover");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown packet type received: %d", packet->type);
            break;
    }
}

esp_err_t espnow_send_rtcm_data(const rtcm_data_t* rtcm_data) {
    if (rtcm_data->length > ESPNOW_MAX_DATA_LEN) {
        ESP_LOGW(TAG, "RTCM data too large: %d bytes", rtcm_data->length);
        return ESP_ERR_INVALID_SIZE;
    }
    
    espnow_packet_t packet;
    packet.type = ESPNOW_PACKET_RTCM;
    packet.sequence = sequence_number++;
    packet.data_len = rtcm_data->length;
    memcpy(packet.data, rtcm_data->data, rtcm_data->length);
    
    esp_err_t result = esp_now_send(rover_mac, (uint8_t*)&packet, 
                                   sizeof(espnow_packet_t) - ESPNOW_MAX_DATA_LEN + packet.data_len);
    
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "esp_now_send failed: %s", esp_err_to_name(result));
    }
    
    return result;
}

esp_err_t espnow_init(void) {
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_callback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_callback));
    
    // Set ESP-NOW channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    
    // Add broadcast peer initially
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, rover_mac, 6);
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.encrypt = false;
    peer_info.ifidx = WIFI_IF_AP;
    
    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to add ESP-NOW peer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "ESP-NOW initialized");
    return ESP_OK;
}