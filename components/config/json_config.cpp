//
// Created by aron on 5/4/2025.
//

// #include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <sstream>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_littlefs.h"
#include "esp_console.h"

#include "json_config.h"

#include <esp_check.h>

static auto* TAG = "JsonConfig";
namespace
{
constexpr const char* k_mount_point = "/config";
constexpr const char* k_filename   = "/config/config.json";

esp_err_t ensure_fs_mounted()
{
    static bool mounted = false;
    if (mounted) return ESP_OK;

    esp_vfs_littlefs_conf_t conf = {
        .base_path = k_mount_point,
        .partition_label = "config",
        .format_if_mount_failed = true,
        .dont_mount = false
    };
    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err == ESP_OK) {
        mounted = true;
        ESP_LOGI(TAG, "Mounted LittleFS partition 'config' at %s", k_mount_point);
    } else {
        ESP_LOGE(TAG, "Failed to mount LittleFS (%s)", esp_err_to_name(err));
    }
    return err;
}

std::vector<std::string> split(const std::string& s, char delim)
{
    std::stringstream ss{s};
    std::string item;
    std::vector<std::string> out;
    while (std::getline(ss, item, delim)) {
        if (!item.empty()) out.push_back(item);
    }
    return out;
}

static void walk_keys(cJSON* node, const std::string& prefix, std::vector<std::string>& out)
{
    if (!cJSON_IsObject(node)) return;
    cJSON* child = node->child;
    while (child) {
        std::string key = prefix.empty() ? child->string : prefix + "." + child->string;

        out.push_back(key);

        if (cJSON_IsObject(child)) {
            walk_keys(child, key, out);
        }
        child = child->next;
    }
}




}

using namespace config;

JsonConfig& JsonConfig::instance()
{
    static JsonConfig instance;
    return instance;
}

JsonConfig::~JsonConfig()
{
    if (root_) cJSON_Delete(root_);
}

esp_err_t JsonConfig::load()
{
    std::lock_guard<std::recursive_mutex> lock(mux_);

    ESP_RETURN_ON_ERROR(ensure_fs_mounted(), TAG, "Failed to mount LittleFS");

    int fd = ::open(k_filename, O_RDONLY);
    if (fd < 0) {
        ESP_LOGW(TAG, "No existing %s, starting with empty config", k_filename);
        root_ = cJSON_CreateObject();
        return ESP_OK;
    }

    struct stat st{};
    fstat(fd, &st);
    std::string buf; buf.resize(st.st_size);
    read(fd, buf.data(), st.st_size);
    ::close(fd);

    cJSON* root = cJSON_Parse(buf.c_str());
    if (!root) {
        ESP_LOGE(TAG, "Malformed JSON, starting from fresh");
        root = cJSON_CreateObject();
    }
    if (root_) cJSON_Delete(root_);
    root_ = root;
    dirty_ =false;

    return ESP_OK;
}

cJSON* JsonConfig::find_or_create_path(const std::string& dotted_key)
{
    std::vector<std::string> tokens = split(dotted_key, '.');
    if (tokens.empty()) return nullptr;

    cJSON* node = root_;
    for (size_t i = 0; i < tokens.size(); ++i) {
        const char* name = tokens[i].c_str();
        cJSON* child = cJSON_GetObjectItem(node, name);
        if (!child) {
            if (i == tokens.size() - 1) {
                return node;
            } else {
                child = cJSON_CreateObject();
                cJSON_AddItemToObject(node, name, child);
            }
        }
        node = child;
    }
    return node;
}


esp_err_t JsonConfig::set(const std::string& key, const std::string& value)
{
    std::lock_guard<std::recursive_mutex> lock(mux_);
    if (!root_) load();

    size_t dot = key.find_last_of('.');
    std::string path = (dot == std::string::npos) ? "" : key.substr(0, dot);
    std::string leaf = (dot == std::string::npos) ? key : key.substr((dot + 1));

    cJSON* parent = path.empty() ? root_ : find_or_create_path(path);
    if (!parent) {
        ESP_LOGE(TAG, "Failed to find or create path for %s", key.c_str());
        return ESP_FAIL;
    }

    cJSON_DeleteItemFromObject(parent, leaf.c_str());

    cJSON* item = cJSON_Parse(value.c_str());
    if (!item) {
        item = cJSON_CreateString(value.c_str());
    }
    cJSON_AddItemToObject(parent, leaf.c_str(), item);

    dirty_ = true;
    return ESP_OK;
}

esp_err_t JsonConfig::commit()
{
    std::lock_guard<std::recursive_mutex> lock(mux_);
    if (!dirty_) return ESP_OK;

    ESP_RETURN_ON_ERROR(ensure_fs_mounted(), TAG, "Mount failed");

    char* json = cJSON_PrintBuffered(root_, 1024, 0);
    if (!json) return ESP_ERR_NO_MEM;

    const char* tmpfile = "/config/config.json.new";
    int fd = ::open(tmpfile, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        free(json);
        ESP_LOGE(TAG, "open %s failed", tmpfile);
        return ESP_FAIL;
    }
    write(fd, json, strlen(json));
    fsync(fd);
    ::close(fd);
    free(json);

    if (::rename(tmpfile, k_filename) != 0) {
        ESP_LOGE(TAG, "rename failed");
        return ESP_FAIL;
    }
    dirty_ = false;
    return ESP_OK;
}

esp_err_t JsonConfig::serialize(char** out) const
{
    std::lock_guard<std::recursive_mutex> lock(mux_);
    if (!root_) return ESP_ERR_INVALID_STATE;
    *out = cJSON_Print(root_);
    return *out ? ESP_OK : ESP_ERR_NO_MEM;
}

std::vector<std::string> JsonConfig::list_keys() const
{
    std::lock_guard lock(mux_);
    std::vector<string> keys;
    if (root_) walk_keys(root_, "", keys);
    return keys;
}

esp_err_t JsonConfig::attach_to_console()
{
    const esp_console_cmd_t cmd = {
        .command = "config",
        .help = "get|set|diff|commit configuration",
        .hint = nullptr,
        .func = &handle_config,
    };
    ESP_RETURN_ON_ERROR( esp_console_cmd_register(&cmd), TAG, "Fialed to regiter commands" );
    ESP_LOGI(TAG, "Registered config commands");
    return ESP_OK;
}

int JsonConfig::handle_config(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: config get|set|diff|commit ...\n");
        return 0;
    }
    const char* verb = argv[1];
    auto& cfg = config::JsonConfig::instance();
    esp_err_t err;

    if (strcmp(verb, "get") == 0) {
        if (argc != 3) {
            printf("Usage: config get <key>\n");
            auto keys = config::JsonConfig::instance().list_keys();
            for (std::string& key : keys) {
                printf("\t%s\n", key.c_str());
            }
            printf("\n%d keys found\n", keys.size());
            return 0;
        }
        std::string key(argv[2]);
        char* out;
        err = cfg.serialize(&out);
        // To simplify, just get full JSON and print. TODO: dot‑lookup only
        if (err == ESP_OK) {
            printf("%s\n", out);
            free(out);
        }
    } else if (strcmp(verb, "set") == 0) {
        if (argc != 4) {
            printf("Usage: config set <key> <value>\n");
            return 0;
        }
        err = cfg.set(argv[2], argv[3]);
        if (err == ESP_OK) printf("OK\n");
    } else if (strcmp(verb, "commit") == 0) {
        err = cfg.commit();
        if (err == ESP_OK) printf("Committed to flash\n");
    } else if (strcmp(verb, "diff") == 0) {
        // Optional: show diff between in‑RAM and on‑disk
        printf("Diff not implemented\n");
    } else {
        printf("Unknown subcommand '%s'\n", verb);
    }
    return 0;
}

/**
 * How to use:
 * @code
 * // in init or similar
 * callback_ = [this] { this->updateFromConfig(); };
 * config::JsonConfig::instance().registerCallback(callback_);
 * @endcode
 * @code
 * void updateFromConfig() {
 *  // check if values have changed
 * }
 * @endcode
 *
 * @param callback
 */
void JsonConfig::register_callback(const std::function<void()>& callback)
{
    if (xSemaphoreTake(callback_mutex_, portMAX_DELAY) == pdTRUE) {
        callbacks_.push_back(callback);
        xSemaphoreGive(callback_mutex_);
    }
}





