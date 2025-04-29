# Copyright (c) Microsoft Corporation. All rights reserved.
# SPDX-License-Identifier: MIT

set(COMPONENT_INCLUDE_DIRS
    .
)

idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ${COMPONENT_INCLUDE_DIRS}
                    REQUIRES espnow_scan ble_scan sensors azure-iot)
