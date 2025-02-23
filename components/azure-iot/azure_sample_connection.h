/* Copyright (c) Microsoft Corporation. All rights reserved. */
/* SPDX-License-Identifier: MIT */

#ifndef AZURE_SAMPLE_CONNECTION_H
#define AZURE_SAMPLE_CONNECTION_H

#include <stdbool.h>
#include <stdint.h>

void connectivity_init(void);
bool xAzureSample_IsConnectedToInternet(void);
uint64_t ullGetUnixTime(void);

#endif /* AZURE_SAMPLE_CONNECTION_H */ 