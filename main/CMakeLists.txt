idf_component_register(
    SRCS 
        "main.cpp"
        "dshot_controller.cpp"
    INCLUDE_DIRS 
        "include"
        "."
    REQUIRES         
        "driver"      # For MCPWM and UART
        "freertos"    # For FreeRTOS functionality
        "esp_driver_rmt"
        "esp_common"
)

target_compile_options(${COMPONENT_LIB} PRIVATE 
    -fno-rtti  # Disable RTTI
    -fno-exceptions  # Optional: also disable exceptions if not needed
)