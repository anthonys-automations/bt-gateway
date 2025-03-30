#include <string.h>
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_temp_sensor.h"

#include "sensors.h"
#include "azure-iot.h"

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

static uint8_t TelemetryBuffer[ AZURE_IOT_TELEMETRY_MAXLEN ];
uint32_t TelemetryBufferLength = 0U;

static void sensors_loop( void * pvParameters )
{
    ( void ) pvParameters;

    sensors_init();
    esp_err_t err;

    for( ; ; )
    {
        err = sensors_get_json((char *)TelemetryBuffer, sizeof(TelemetryBuffer));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get sensor data, error: %d", err);
            // Fallback to a basic message if sensor reading fails
            TelemetryBufferLength = snprintf((char *)TelemetryBuffer, sizeof(TelemetryBuffer),
                                            "{\"error\":\"Failed to read sensors\"}");
        } else {
            TelemetryBufferLength = strlen((char *)TelemetryBuffer);
            ESP_LOGI(TAG, "Sensor data prepared: %s", TelemetryBuffer);
        }
        
        // Queue the telemetry data
        azure_iot_queue_telemetry(TelemetryBuffer, TelemetryBufferLength);

        ESP_LOGI(TAG, "This thread has %u bytes free stack\n", uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay( pdMS_TO_TICKS( 32000U ) );
    }
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
    
    // Get uptime in seconds
    uint32_t uptime_seconds = (uint32_t)(xTaskGetTickCount() / configTICK_RATE_HZ);
    
    int written = snprintf(buffer, buffer_size, 
                          "{\"esp_temperature\":%.1f,\"uptime_seconds\":%u}", 
                          temperature, (unsigned int)uptime_seconds);
    
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

/*
 * @brief Create the task that demonstrates the AzureIoTHub demo
 */
void vStartSensorsLoop( void )
{

    /* This example uses a single application task, which in turn is used to
     * connect, subscribe, publish, unsubscribe and disconnect from the IoT Hub */
    xTaskCreate( sensors_loop,         /* Function that implements the task. */
                 "SensorsLoop",          /* Text name for the task - only used for debugging. */
                 8192, /* Size of stack (in words, not bytes) to allocate for the task. */
                 NULL,                     /* Task parameter - not used in this case. */
                 tskIDLE_PRIORITY,         /* Task priority, must be between 0 and configMAX_PRIORITIES - 1. */
                 NULL );                   /* Used to pass out a handle to the created task - not used in this case. */
}
/*-----------------------------------------------------------*/

