/* Copyright (c) Microsoft Corporation. All rights reserved. */
/* SPDX-License-Identifier: MIT */

#include "azure-iot.h"
#include "azure_sample_connection.h"
#include "sensors.h"
#include "ble_scan.h"
#include "espnow_scan.h"

void app_main(void)
{
    connectivity_init();
    vStartSensorsLoop();
    vStartAzureIoT();
    ble_scan_init();
    espnow_scan_init();
}
