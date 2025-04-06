/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/temperature_sensor.h"
#include "esp_temp_sensor.h"

static const char *TAG = "esp_temp_sensor";
static temperature_sensor_handle_t temp_sensor = NULL;

esp_err_t esp_temp_sensor_init(void)
{
    ESP_LOGI(TAG, "Install temperature sensor, expected temp ranger range: -10~50 ℃");
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 50);
    esp_err_t ret = temperature_sensor_install(&temp_sensor_config, &temp_sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install temperature sensor");
        return ret;
    }

    ESP_LOGI(TAG, "Enable temperature sensor");
    ret = temperature_sensor_enable(temp_sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable temperature sensor");
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t esp_temp_sensor_read(float *temperature)
{
    if (temp_sensor == NULL || temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Reading temperature");
    esp_err_t ret = temperature_sensor_get_celsius(temp_sensor, temperature);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Temperature value %.02f ℃", *temperature);
    } else {
        ESP_LOGE(TAG, "Failed to read temperature");
    }
    
    return ret;
}

esp_err_t esp_temp_sensor_deinit(void)
{
    if (temp_sensor == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = temperature_sensor_disable(temp_sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable temperature sensor");
        return ret;
    }
    
    ret = temperature_sensor_uninstall(temp_sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to uninstall temperature sensor");
        return ret;
    }
    
    temp_sensor = NULL;
    return ESP_OK;
}
