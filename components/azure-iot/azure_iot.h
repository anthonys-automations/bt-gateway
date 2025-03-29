/**
 * @brief Maximum size of Azure IoT Telemetry message.
 */
#define AZURE_IOT_TELEMETRY_MAXLEN          ( 256 )

/**
 * @brief Adds a message to the telemetry queue
 * @param pucMessage Pointer to the message buffer
 * @param xMessageLength Length of the message
 * @return BaseType_t pdPASS on success, pdFAIL on failure
 */
BaseType_t azure_iot_queue_telemetry(uint8_t *pucMessage, size_t xMessageLength); 

/*
 * @brief Create the task that demonstrates the AzureIoTHub demo
 */
void vStartDemoTask( void );
