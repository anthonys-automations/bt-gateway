set(depends esp_timer esp_system esp_event driver)

idf_component_register(
    SRCS "hdc1080.c"
    INCLUDE_DIRS "include"
    REQUIRES ${depends})
