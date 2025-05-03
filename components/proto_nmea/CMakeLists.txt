idf_component_register(
        SRCS "nmea_driver.cpp"
        INCLUDE_DIRS
            "include"

        REQUIRES driver freertos nav shared tinygps-espidf
)

