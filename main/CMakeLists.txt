idf_component_register(
    SRCS 
        "main.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES         
        "esp_timer"
        "freertos"
        "servo"
        "driver"
        "rc"
        "proto_sbus"
        "nav"
        "proto_nmea"
        "proto_icm20948"
        "motion"
        "console_interface"
)

target_compile_options(${COMPONENT_LIB} PRIVATE 
    -fno-rtti  # Disable RTTI
    -fno-exceptions  # Optional: also disable exceptions if not needed
)