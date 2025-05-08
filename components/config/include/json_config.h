//
// Created by aron on 5/4/2025.
//

#pragma once

#include <string>
#include <mutex>
#include <vector>

#include "esp_err.h"
#include "cJSON.h"

using std::string;

namespace config
{
class JsonConfig
{
public:
    static JsonConfig& instance();

    JsonConfig(const JsonConfig&) = delete;
    JsonConfig& operator=(const JsonConfig&) = delete;

    esp_err_t attach_to_console();

    esp_err_t load();

    template<typename T>
    T get(const std::string& key, T fallback);

    esp_err_t set(const std::string& key, const std::string& value);

    esp_err_t commit();

    esp_err_t serialize(char** out) const;

    void register_callback(const std::function<void()> &callback);
    // TODO: unregister_callback, but i dont think we will ever need it.

    std::vector<std::string> list_keys() const;
private:
    JsonConfig() = default;
    ~JsonConfig();

    static int handle_config(int argc, char** argv);


    cJSON* find_or_create_path(const std::string& dotted_key);

    std::vector<std::function<void()>> callbacks_;
    SemaphoreHandle_t callback_mutex_ = nullptr;

    mutable std::recursive_mutex mux_;
    cJSON* root_;
    bool dirty_ = false;
};

template<>
inline int JsonConfig::get<int>(const std::string& key, const int fallback)
{
    std::lock_guard lock(mux_);
    if (auto* item = find_or_create_path(key)) {
        return cJSON_IsNumber(item) ? item->valueint : fallback;
    }
    return fallback;
}

template<>
inline bool JsonConfig::get<bool>(const std::string&key, const bool fallback)
{
    std::lock_guard lock(mux_);
    if (auto* item = find_or_create_path(key)) {
        return cJSON_IsBool(item) ? item->valueint : fallback;
    }
    return fallback;
}

template<>
inline string JsonConfig::get<string>(const std::string& key, string fallback)
{
    std::lock_guard lock(mux_);
    if (auto* item = find_or_create_path(key)) {
        return cJSON_IsString(item) ? item->valuestring : fallback;
    }
    return fallback;
}

template<>
inline float JsonConfig::get<float>(const std::string& key, const float fallback)
{
    std::lock_guard lock(mux_);
    if (auto* item = find_or_create_path(key)) {
        return cJSON_IsNumber(item) ? static_cast<float>(item->valuedouble) : fallback;
    }
    return fallback;
}
}