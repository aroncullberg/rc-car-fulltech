idf_component_register(
        SRCS "nmea_driver.cpp"
        INCLUDE_DIRS
            "include"
        PRIV_REQUIRES driver freertos nav shared andyfinity__TinyGPSPlus-ESPIDF
)

