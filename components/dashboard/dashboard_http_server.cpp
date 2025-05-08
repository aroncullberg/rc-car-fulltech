//
// Created by aron on 5/6/2025.
//

#include "dashboard_http_server.hpp"

#include <esp_check.h>
#include <sys/dirent.h>

using namespace dashboard;
static constexpr auto* tag = "HttpServer";

HttpServer& HttpServer::instance()
{
    static HttpServer instance;
    return instance;
}

HttpServer::~HttpServer()
{
    stop();
}

esp_err_t HttpServer::init_fs()
{
    constexpr esp_vfs_littlefs_conf_t conf {
        .base_path = BASE_PATH,
        .partition_label = PARTITION_LABEL,
        .format_if_mount_failed = false,
        .dont_mount = false,
    };
    ESP_RETURN_ON_ERROR(esp_vfs_littlefs_register(&conf), tag, "Failed to register littlefs");
    ESP_LOGI(tag, "LittleFS mounted at %s", BASE_PATH);

    DIR* dir = opendir(BASE_PATH);
    if (!dir) {
        ESP_LOGE(tag, "Failed to open directory: %s", BASE_PATH);
        return ESP_FAIL;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_REG) {
            ESP_LOGI(tag, "File: %s", entry->d_name);
        } else if (entry->d_type == DT_DIR) {
            ESP_LOGI(tag, "Directory: %s", entry->d_name);
        }
    }
    closedir(dir);
    return ESP_OK;
}

static esp_err_t send_file(httpd_req_t *req, const char *path)
{
    printf("DEBUG: %s\n", path);
    FILE *f = fopen(path, "r");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }
    char buf[512];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
        httpd_resp_send_chunk(req, buf, n);
    fclose(f);
    httpd_resp_send_chunk(req, nullptr, 0);   // signal end
    return ESP_OK;
}

esp_err_t HttpServer::index_handler(httpd_req_t* request)
{
    printf("index_handler: %s\n", request->uri);
    std:: string path = std::string(BASE_PATH) + "/index.html";
    ESP_LOGI(tag, "index_handler: %s", path.c_str());
    return send_file(request, path.c_str());
}

esp_err_t HttpServer::static_handler(httpd_req_t* request)
{
    printf("static_handler: %s\n", request->uri);
    std::string path = std::string(BASE_PATH) + std::string(request->uri);
    ESP_LOGI(tag, "static_handler: %s", path.c_str());
    return send_file(request, path.c_str());
}

esp_err_t HttpServer::websocket_handler(httpd_req_t* request)
{
    printf("websocket_handler: %s\n", request->uri);
    if (request->method == HTTP_GET) {
        ESP_LOGI(tag, "WS handshake from fd %d", httpd_req_to_sockfd(request));
        instance().clients.push_back(httpd_req_to_sockfd(request));
        return ESP_OK;
    }

    // TODO: handle incoming messages

    return ESP_OK;
}

esp_err_t HttpServer::start_server()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = 0;
    config.stack_size = 8192;
    config.max_open_sockets = MAX_CLIENTS;

    ESP_RETURN_ON_ERROR(httpd_start(&server, &config), tag, "Failed to start server");

    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .user_ctx = nullptr,
        .is_websocket = true
    };
    httpd_register_uri_handler(server, &ws_uri);

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &index_uri);

    httpd_uri_t static_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = static_handler,
        .user_ctx = nullptr
    };
    httpd_register_uri_handler(server, &static_uri);

    ESP_LOGI(tag, "HTTP/WS server ready - port %d", config.server_port);
    return ESP_OK;
}

esp_err_t HttpServer::start()
{
    ESP_RETURN_ON_ERROR(init_fs(), tag, "Failed to initialize FS");
    ESP_RETURN_ON_ERROR(start_server(), tag, "Failed to start server");
    return ESP_OK;
}

void HttpServer::close_clients()
{
    if (!server) return;
    for (auto client : clients) {
        if (client >= 0) {
            httpd_sess_trigger_close(server, client);
        }
    }
    clients.clear();
}
void HttpServer::stop()
{
    close_clients();
    if (server) {
        httpd_stop(server);
        server = nullptr;
    }
    esp_vfs_littlefs_unregister(PARTITION_LABEL);
}

esp_err_t HttpServer::push(const cJSON *obj)
{
    if (!server || clients.empty() || !obj) return ESP_ERR_INVALID_STATE;

    char* json = cJSON_PrintUnformatted(obj);
    if (!json) return ESP_ERR_NO_MEM;

    httpd_ws_frame_t frame = {};
    frame.payload = (uint8_t *)json;
    frame.len     = strlen(json);
    frame.type    = HTTPD_WS_TYPE_TEXT;
    frame.final = true;

    for (auto client : clients) {
        esp_err_t err = httpd_ws_send_frame_async(server, client, &frame);
        if (err != ESP_OK) {
            ESP_LOGW(tag, "Send failed (fd=%d) -> removing", client);
            close_clients();
            break;
        }
    }

    free(json);
    return ESP_OK;
}

