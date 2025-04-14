idf_component_register(SRCS "ble_scan.c"
                       INCLUDE_DIRS "include"
                       REQUIRES nvs_flash bt azure-iot)
