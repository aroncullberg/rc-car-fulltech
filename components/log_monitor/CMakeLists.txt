idf_component_register(
        SRCS
            "log_monitor.cpp"
        INCLUDE_DIRS
            "."
        REQUIRES
            "ConfigManager"
            freertos
            esp_common
            esp_wifi
            esp_http_server
            nvs_flash
            esp_netif
)