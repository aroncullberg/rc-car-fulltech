idf_component_register(
        SRCS "elrs_backend.cpp"
        INCLUDE_DIRS "include"
        PRIV_REQUIRES driver freertos rc
)