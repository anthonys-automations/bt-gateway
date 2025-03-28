set(ROOT_PATH
    ${CMAKE_CURRENT_LIST_DIR}/../..
)

set(COMPONENT_SOURCES "esp_temp_sensor.c" "sensors.c")

set(COMPONENT_INCLUDE_DIRS
    ${ROOT_PATH}/components/sensors/include
)    

idf_component_register(
    SRCS ${COMPONENT_SOURCES}
    INCLUDE_DIRS ${COMPONENT_INCLUDE_DIRS}
    REQUIRES driver)
