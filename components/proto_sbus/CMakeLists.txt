idf_component_register(
        SRCS "sbus_driver.cpp"
        INCLUDE_DIRS "include"
        PRIV_REQUIRES driver freertos rc
)