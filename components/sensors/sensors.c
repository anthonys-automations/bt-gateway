#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "sensors.h"
#include "esp_temp_sensor.h"

static const char *TAG = "sensors";

esp_err_t sensors_init(void)
{
    ESP_LOGI(TAG, "Initializing sensors");
    
    // Initialize ESP temperature sensor
    esp_err_t ret = esp_temp_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP temperature sensor");
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t sensors_get_json(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    float temperature;
    esp_err_t ret = esp_temp_sensor_read(&temperature);
    if (ret != ESP_OK) {
        return ret;
    }
    
    int written = snprintf(buffer, buffer_size, "{\"esp_temperature\":%.1f}", temperature);
    
    if (written < 0 || written >= buffer_size) {
        return ESP_ERR_NOT_ALLOWED;
    }
    
    return ESP_OK;
}

esp_err_t sensors_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing sensors");
    
    esp_err_t ret = esp_temp_sensor_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinitialize ESP temperature sensor");
        return ret;
    }
    
    return ESP_OK;
}
