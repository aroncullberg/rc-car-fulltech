idf_component_register(
    SRCS 
        "main.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES         
        "servo"
        "driver"
        "nav"
        "proto_nmea"
        "rc"
        "proto_sbus"
        "motion"
        "proto_icm20948"
        "console"
        "nvs_flash"
        "esp_driver_usb_serial_jtag"
        "config"
        "esp_wifi"
        "espressif__mavlink"
)

target_compile_options(${COMPONENT_LIB} PRIVATE 
    -fno-rtti  # Disable RTTI
    -fno-exceptions  # Optional: also disable exceptions if not needed
)

target_compile_definitions(${COMPONENT_LIB}
        PRIVATE MAVLINK_PRIMARY_DIALECT=ardupilotmega)