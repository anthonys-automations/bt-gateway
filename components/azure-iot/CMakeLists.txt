# Copyright (c) Microsoft Corporation. All rights reserved.
# SPDX-License-Identifier: MIT

set(ROOT_PATH
    ${CMAKE_CURRENT_LIST_DIR}/../..
)

# kconfig does not support multiline strings.
# For certificates, we use as a workaround escaping the newlines
# in certificates and keys so they can be entered as a single
# string in kconfig.
# The routine below unescapes the newlines so the values
# can be correctly interpreted by the code.
if(EXISTS "${CMAKE_BINARY_DIR}/config/sdkconfig.h")
    file(READ "${CMAKE_BINARY_DIR}/config/sdkconfig.h" config_header)
    string(REPLACE "\\n" "n" client_certificate ${config_header})
    message("CLIENT_CERT: ${client_certificate}")
    file(WRITE "${CMAKE_BINARY_DIR}/config/sdkconfig.h" "${client_certificate}")
endif()

idf_component_get_property(MBEDTLS_DIR mbedtls COMPONENT_DIR)

list(APPEND COMPONENT_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/backoff_algorithm.c
    ${CMAKE_CURRENT_LIST_DIR}/transport_tls_esp32.c
    ${CMAKE_CURRENT_LIST_DIR}/crypto_esp32.c
    ${CMAKE_CURRENT_LIST_DIR}/connectivity.c
    ${CMAKE_CURRENT_LIST_DIR}/azure-iot.c
    ${CMAKE_CURRENT_LIST_DIR}/systeminfo.c
)

set(COMPONENT_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/config
    ${CMAKE_CURRENT_LIST_DIR}
    ${MBEDTLS_DIR}/mbedtls/include
    include
    common/transport
    common/utilities
    common/connection
)

idf_component_register(
    SRCS ${COMPONENT_SOURCES}
    INCLUDE_DIRS ${COMPONENT_INCLUDE_DIRS}
    REQUIRES mbedtls tcp_transport coreMQTT azure-sdk-for-c azure-iot-middleware-freertos esp_wifi esp_event nvs_flash)

