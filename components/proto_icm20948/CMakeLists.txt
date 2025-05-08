idf_component_register(
    SRCS
        "icm_20948_driver.cpp"
    INCLUDE_DIRS
        "include"
    PRIV_REQUIRES
        "driver"
        "freertos"
        "esp_common"
        "motion"
    REQUIRES
        "cr-icm20948"
)

