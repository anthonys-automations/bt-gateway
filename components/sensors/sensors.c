#include <string.h>
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_temp_sensor.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "sensors.h"
#include "hdc1080_main.h"
#include "led_strip.h"
#include "azure-iot.h"

static const char *TAG = "sensors";
adc_oneshot_unit_handle_t adc1_handle;

esp_err_t sensors_init(void)
{
    ESP_LOGI(TAG, "Initializing sensors");
    
    // Initialize ESP temperature sensor
    esp_err_t ret = esp_temp_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP temperature sensor");
        return ret;
    }

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));

    return ESP_OK;
}

static uint8_t TelemetryBuffer[ AZURE_IOT_TELEMETRY_MAXLEN ];
uint32_t TelemetryBufferLength = 0U;

static void sensors_loop( void * pvParameters )
{
    ( void ) pvParameters;

    sensors_init();
    hdc1080_main_init();
    led_init();

    esp_err_t err;

    for( ; ; )
    {
        hdc1080_main_request_readings();

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
        ESP_LOGI(TAG, "Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());
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
    
    // Read voltage from GPIO 1 (assuming it's connected to ADC1 channel 0)
    int adc_reading;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_reading));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_0, adc_reading);
    // Convert ADC reading to voltage (assuming 3.3V reference and 12-bit ADC)
    float voltage = (adc_reading * 3.3) / 4095.0;
    led_set(0,(int)temperature,(int)voltage*10);

    int written = snprintf(buffer, buffer_size, 
                          "{\"esp_temperature\":%.1f,\"uptime_seconds\":%u,\"battery_voltage\":%.2f}", 
                          temperature, (unsigned int)uptime_seconds, voltage);
    
    if (written < 0 || written >= buffer_size) {
        return ESP_ERR_NOT_ALLOWED;
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

