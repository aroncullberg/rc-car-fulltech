//
// Created by aron on 5/6/2025.
//

#pragma once

#include <array>
#include <string>
#include <cstring>
#include <vector>

#include "esp_http_server.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "cJSON.h"

#define BASE_PATH "/www"
#define PARTITION_LABEL "dashboard"
#define MAX_OPEN_FILES 4
#define MAX_CLIENTS 4

namespace dashboard
{
class HttpServer
{
public:
    static  HttpServer& instance();
    explicit HttpServer() = default;
    ~HttpServer();

    esp_err_t start();
    void stop();
    esp_err_t push(const cJSON *obj);
private:
    static esp_err_t websocket_handler(httpd_req_t *request);
    static esp_err_t index_handler(httpd_req_t *request);
    static esp_err_t static_handler(httpd_req_t *request);

    esp_err_t init_fs();
    esp_err_t start_server();
    void close_clients();

    httpd_handle_t server{nullptr};
    std::vector<int> clients{};
    size_t client_count{};
};
}