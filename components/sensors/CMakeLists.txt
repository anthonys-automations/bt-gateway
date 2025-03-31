set(EXCLUDE_COMPONENTS esp_lcd)

idf_component_register(SRCS "sensors.c" "esp_temp_sensor.c" "esp32-hdc1080/hdc1080.c" "esp32-hdc1080/hdc1080_main.c" "led/led_strip.c" "led/led_strip_encoder.c"
                    INCLUDE_DIRS "include"
                    PRIV_INCLUDE_DIRS "esp32-hdc1080/include" "led/include"
                    REQUIRES driver azure-iot esp_timer esp_system esp_event)
