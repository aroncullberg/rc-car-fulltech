idf_component_register(
        SRCS
        "json_config.cpp"
        INCLUDE_DIRS
        "include"
        REQUIRES json littlefs vfs console
)

#set(COMPONENT_REQUIRES "json")
