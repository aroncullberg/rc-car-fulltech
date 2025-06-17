idf_component_register(
        SRCS vehicle_controller.cpp vehicle_data.cpp pid_controller.cpp
        INCLUDE_DIRS include
        PRIV_REQUIRES motion nav rc EscDriver servo config
)