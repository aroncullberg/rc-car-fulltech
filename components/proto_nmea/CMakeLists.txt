idf_component_register(
        SRCS "nmea_driver.cpp"
        INCLUDE_DIRS
            "include"
            "tinygps-espidf/include"
            "tinygps-espidf/TinyGPSPlus/src"
        REQUIRES driver freertos nav shared
)

add_subdirectory(tinygps-espidf)

target_link_libraries(${COMPONENT_LIB} PRIVATE tinygps-espidf)
