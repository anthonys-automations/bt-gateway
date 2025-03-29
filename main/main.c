/* Copyright (c) Microsoft Corporation. All rights reserved. */
/* SPDX-License-Identifier: MIT */

#include "azure_sample_connection.h"
#include "sensors.h"

extern void vStartDemoTask(void);
extern void connectivity_init(void);

void app_main(void)
{
    connectivity_init();
    vStartSensorsLoop();
    vStartDemoTask();
}
