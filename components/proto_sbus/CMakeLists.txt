idf_component_register(
        SRCS "sbus_driver.cpp"
        INCLUDE_DIRS "include"
        REQUIRES driver freertos rc shared
)