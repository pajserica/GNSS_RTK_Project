#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_err.h"
#include "esp_http_server.h"

// Web Server Configuration
#define WEB_SERVER_PORT 80
#define WEB_SERVER_MAX_URI_LEN 512
#define WEB_SERVER_MAX_RESP_LEN 4096

// Function declarations
esp_err_t web_server_start(void);
esp_err_t web_server_stop(void);

// HTTP handlers
esp_err_t root_handler(httpd_req_t *req);
esp_err_t status_handler(httpd_req_t *req);
esp_err_t api_status_handler(httpd_req_t *req);

#endif // WEB_SERVER_H