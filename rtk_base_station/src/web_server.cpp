#include "web_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include "gnss_module.h"

static const char *TAG = "WEB_SERVER";
static httpd_handle_t server = NULL;

extern base_station_status_t base_status;
extern SemaphoreHandle_t status_mutex;

// HTML page for the web interface
static const char* html_page = R"html(
<!DOCTYPE html>
<html>
<head>
    <title>RTK Base Station</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        .header { text-align: center; color: #333; margin-bottom: 30px; }
        .status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 20px; margin-bottom: 30px; }
        .status-card { background: #f8f9fa; padding: 15px; border-radius: 8px; border-left: 4px solid #007bff; }
        .status-label { font-weight: bold; color: #666; font-size: 0.9em; }
        .status-value { font-size: 1.5em; color: #333; margin-top: 5px; }
        .connected { color: #28a745; }
        .disconnected { color: #dc3545; }
        .refresh-btn { background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; }
        .refresh-btn:hover { background: #0056b3; }
        .logs { background: #000; color: #0f0; padding: 15px; border-radius: 5px; font-family: monospace; height: 200px; overflow-y: auto; margin-top: 20px; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🛰️ RTK Base Station</h1>
            <p>Real-Time Kinematic GNSS Base Station Monitor</p>
        </div>
        
        <div class="status-grid">
            <div class="status-card">
                <div class="status-label">RTK Status</div>
                <div class="status-value" id="rtk-status">Loading...</div>
            </div>
            <div class="status-card">
                <div class="status-label">Satellites</div>
                <div class="status-value" id="satellite-count">-</div>
            </div>
            <div class="status-card">
                <div class="status-label">RTCM Packets Sent</div>
                <div class="status-value" id="rtcm-packets">-</div>
            </div>
            <div class="status-card">
                <div class="status-label">Uptime</div>
                <div class="status-value" id="uptime">-</div>
            </div>
            <div class="status-card">
                <div class="status-label">GNSS Connection</div>
                <div class="status-value" id="gnss-status">-</div>
            </div>
            <div class="status-card">
                <div class="status-label">Rover Connection</div>
                <div class="status-value" id="rover-status">-</div>
            </div>
        </div>
        
        <div style="text-align: center;">
            <button class="refresh-btn" onclick="updateStatus()">🔄 Refresh Status</button>
        </div>
        
        <div class="logs" id="logs">
            RTK Base Station Log Output<br>
            System initialized...<br>
        </div>
    </div>

    <script>
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('rtk-status').textContent = data.rtk_status;
                    document.getElementById('satellite-count').textContent = data.satellite_count;
                    document.getElementById('rtcm-packets').textContent = data.rtcm_packets_sent;
                    
                    const hours = Math.floor(data.uptime_seconds / 3600);
                    const minutes = Math.floor((data.uptime_seconds % 3600) / 60);
                    const seconds = data.uptime_seconds % 60;
                    document.getElementById('uptime').textContent = 
                        `${hours.toString().padStart(2,'0')}:${minutes.toString().padStart(2,'0')}:${seconds.toString().padStart(2,'0')}`;
                    
                    const gnssStatus = document.getElementById('gnss-status');
                    gnssStatus.textContent = data.gnss_connected ? 'Connected' : 'Disconnected';
                    gnssStatus.className = 'status-value ' + (data.gnss_connected ? 'connected' : 'disconnected');
                    
                    const roverStatus = document.getElementById('rover-status');
                    roverStatus.textContent = data.rover_connected ? 'Connected' : 'Disconnected';
                    roverStatus.className = 'status-value ' + (data.rover_connected ? 'connected' : 'disconnected');
                    
                    // Add log entry
                    const logs = document.getElementById('logs');
                    const timestamp = new Date().toLocaleTimeString();
                    logs.innerHTML += `[${timestamp}] Status updated - Satellites: ${data.satellite_count}, RTCM: ${data.rtcm_packets_sent}<br>`;
                    logs.scrollTop = logs.scrollHeight;
                })
                .catch(error => {
                    console.error('Error fetching status:', error);
                    const logs = document.getElementById('logs');
                    logs.innerHTML += `[${new Date().toLocaleTimeString()}] Error: Failed to fetch status<br>`;
                });
        }
        
        // Auto-refresh every 5 seconds
        setInterval(updateStatus, 5000);
        
        // Initial load
        updateStatus();
    </script>
</body>
</html>
)html";

esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

esp_err_t api_status_handler(httpd_req_t *req) {
    xSemaphoreTake(status_mutex, portMAX_DELAY);
    
    cJSON *json = cJSON_CreateObject();
    cJSON *rtk_status = cJSON_CreateString(base_status.rtk_status);
    cJSON *satellite_count = cJSON_CreateNumber(base_status.satellite_count);
    cJSON *rtcm_packets_sent = cJSON_CreateNumber(base_status.rtcm_packets_sent);
    cJSON *uptime_seconds = cJSON_CreateNumber(base_status.uptime_seconds);
    cJSON *gnss_connected = cJSON_CreateBool(base_status.gnss_connected);
    cJSON *rover_connected = cJSON_CreateBool(base_status.rover_connected);
    
    cJSON_AddItemToObject(json, "rtk_status", rtk_status);
    cJSON_AddItemToObject(json, "satellite_count", satellite_count);
    cJSON_AddItemToObject(json, "rtcm_packets_sent", rtcm_packets_sent);
    cJSON_AddItemToObject(json, "uptime_seconds", uptime_seconds);
    cJSON_AddItemToObject(json, "gnss_connected", gnss_connected);
    cJSON_AddItemToObject(json, "rover_connected", rover_connected);
    
    xSemaphoreGive(status_mutex);
    
    char *json_string = cJSON_Print(json);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    
    return ESP_OK;
}

esp_err_t web_server_start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEB_SERVER_PORT;
    config.max_uri_handlers = 8;
    config.max_resp_headers = 8;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        // Root handler
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_uri);
        
        // API status handler
        httpd_uri_t api_status_uri = {
            .uri       = "/api/status",
            .method    = HTTP_GET,
            .handler   = api_status_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_status_uri);
        
        ESP_LOGI(TAG, "Web server started on port %d", WEB_SERVER_PORT);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Error starting web server");
    return ESP_FAIL;
}

esp_err_t web_server_stop(void) {
    if (server) {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "Web server stopped");
    }
    return ESP_OK;
}