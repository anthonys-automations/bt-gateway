#include <string.h>
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "esp_err.h"
#include "esp_temp_sensor.h"

#include "sensors.h"
#include "azure_iot.h"

static const char *TAG = "sensors";

/*
 * Include logging header files and define logging macros in the following order:
 * 1. Include the header file "esp_log.h".
 * 2. Define the LIBRARY_LOG_NAME and LIBRARY_LOG_LEVEL macros depending on
 * the logging configuration for DEMO.
 * 3. Define macros to replace module logging functions by esp logging functions.
 */

#include "esp_log.h"

#ifndef LIBRARY_LOG_NAME
    #define LIBRARY_LOG_NAME    "sensors"
#endif

#define SINGLE_PARENTHESIS_LOGE( x, ... )    ESP_LOGE( LIBRARY_LOG_NAME, x, ## __VA_ARGS__ )
#define LogError( message )                  SINGLE_PARENTHESIS_LOGE message

#define SINGLE_PARENTHESIS_LOGI( x, ... )    ESP_LOGI( LIBRARY_LOG_NAME, x, ## __VA_ARGS__ )
#define LogInfo( message )                   SINGLE_PARENTHESIS_LOGI message

#define SINGLE_PARENTHESIS_LOGW( x, ... )    ESP_LOGW( LIBRARY_LOG_NAME, x, ## __VA_ARGS__ )
#define LogWarn( message )                   SINGLE_PARENTHESIS_LOGW message

#define SINGLE_PARENTHESIS_LOGD( x, ... )    ESP_LOGD( LIBRARY_LOG_NAME, x, ## __VA_ARGS__ )
#define LogDebug( message )                  SINGLE_PARENTHESIS_LOGD message

/************ End of logging configuration ****************/

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

static uint8_t TelemetryBuffer[ 128 ];
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
            LogError(("Failed to get sensor data, error: %d", err));
            // Fallback to a basic message if sensor reading fails
            TelemetryBufferLength = snprintf((char *)TelemetryBuffer, sizeof(TelemetryBuffer),
                                            "{\"error\":\"Failed to read sensors\"}");
        } else {
            TelemetryBufferLength = strlen((char *)TelemetryBuffer);
            LogInfo(("Sensor data prepared: %s", TelemetryBuffer));
        }
        
        // Queue the telemetry data
        azure_iot_queue_telemetry(TelemetryBuffer, TelemetryBufferLength);
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

/*
 * @brief Create the task that demonstrates the AzureIoTHub demo
 */
void vStartSensorsLoop( void )
{
    /* This example uses a single application task, which in turn is used to
     * connect, subscribe, publish, unsubscribe and disconnect from the IoT Hub */
    xTaskCreate( sensors_loop,         /* Function that implements the task. */
                 "SensorsLoop",          /* Text name for the task - only used for debugging. */
                 4096, /* Size of stack (in words, not bytes) to allocate for the task. */
                 NULL,                     /* Task parameter - not used in this case. */
                 tskIDLE_PRIORITY,         /* Task priority, must be between 0 and configMAX_PRIORITIES - 1. */
                 NULL );                   /* Used to pass out a handle to the created task - not used in this case. */
}
/*-----------------------------------------------------------*/

