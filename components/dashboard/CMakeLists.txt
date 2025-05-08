idf_component_register(
        SRCS dashboard_http_server.cpp
        INCLUDE_DIRS include
        REQUIRES esp_http_server littlefs json
)

littlefs_create_partition_image(
        dashboard
        ${CMAKE_SOURCE_DIR}/components/dashboard/web/
#        FLASH_IN_PROJECT
        )