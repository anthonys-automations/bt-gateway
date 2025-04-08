/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ESP_TEMP_SENSOR_H
#define ESP_TEMP_SENSOR_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the ESP32 internal temperature sensor
 * 
 * @return ESP_OK on success, otherwise an error code
 */
esp_err_t esp_temp_sensor_init(void);

/**
 * @brief Read the current temperature from the sensor
 * 
 * @param temperature Pointer to store the temperature value in Celsius
 * @return ESP_OK on success, otherwise an error code
 */
esp_err_t esp_temp_sensor_read(float *temperature);

/**
 * @brief Deinitialize the temperature sensor
 * 
 * @return ESP_OK on success, otherwise an error code
 */
esp_err_t esp_temp_sensor_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* ESP_TEMP_SENSOR_H */ 