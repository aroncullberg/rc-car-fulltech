idf_component_register(
    SRCS 
       "vdc.cpp"
       "steering_pid.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES         
        "driver"
        "freertos"
        "esp_common"
        "servo"
        "ConfigManager"
        "shared"
        "data_pool"
        "dshot_controller"
)