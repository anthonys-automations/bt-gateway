/* Copyright (c) Microsoft Corporation.
 * Licensed under the MIT License. */

/* Standard includes. */
#include <string.h>
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

// #define DEEP_SLEEP_TIME 1500

#ifdef DEEP_SLEEP_TIME
#include "esp_sleep.h"
#endif                

/* Demo Specific configs. */
#include "demo_config.h"

/* Demo Specific Interface Functions. */
#include "azure_sample_connection.h"

/* Azure Provisioning/IoT Hub library includes */
#include "azure_iot_hub_client.h"
#include "azure_iot_provisioning_client.h"

/* Exponential backoff retry include. */
#include "backoff_algorithm.h"

/* Transport interface implementation include header for TLS. */
#include "transport_tls_socket.h"

/* Crypto helper header. */
#include "azure_sample_crypto.h"

/* ESP-IDF includes */
#include "esp_wifi.h"
#include "esp_netif.h"

#include "azure-iot.h"


// Create a message structure that includes the length
typedef struct {
    uint8_t *data;
    size_t length;     // Actual data length
    char source[32];   // Source identifier
} TelemetryMessage_t;

/**
 * @brief Gets the handle to the telemetry queue, creating it if it doesn't exist
 * @return QueueHandle_t Handle to the telemetry queue
 */
QueueHandle_t azure_iot_get_telemetry_queue(void)
{
    static QueueHandle_t xTelemetryQueue = NULL;
    
    if (xTelemetryQueue == NULL) {
        // Create a queue to hold 40 message structures (adjust as needed)
        xTelemetryQueue = xQueueCreate(40, sizeof(TelemetryMessage_t));
        
        if (xTelemetryQueue == NULL) {
            LogError(("Failed to create telemetry queue"));
        } else {
            LogInfo(("Telemetry queue created successfully"));
        }
    }
    
    return xTelemetryQueue;
}

/**
 * @brief Adds a message to the telemetry queue
 * @param pucMessage Pointer to the message buffer
 * @param xMessageLength Length of the message
 * @param pcSource Optional source identifier (will use democonfigDEVICE_ID if NULL)
 * @return BaseType_t pdPASS on success, pdFAIL on failure
 */
BaseType_t azure_iot_queue_telemetry(uint8_t *pucMessage, size_t xMessageLength, const char *pcSource)
{
    // Get queue handle (creates if needed)
    QueueHandle_t xQueue = azure_iot_get_telemetry_queue();
    
    TelemetryMessage_t message;
    
    // Ensure we don't exceed the queue item size
    if (xMessageLength > AZURE_IOT_TELEMETRY_MAXLEN) {
        xMessageLength = AZURE_IOT_TELEMETRY_MAXLEN;
        LogWarn(("Message truncated to fit AZURE IOT TELEMETRY MAXLEN"));
    }

    // Before sending to queue
    UBaseType_t uxQueueSpaces = uxQueueSpacesAvailable(xQueue);
    LogInfo(("Queue has %d spaces available before adding new message", uxQueueSpaces));

    if (uxQueueSpaces < 2) { // Getting low on space
        LogWarn(("Telemetry queue is nearly full! (%d spaces left)", uxQueueSpaces));
    }

    // Copy message data and length
    message.data = malloc(xMessageLength);
    if (message.data == NULL) {
        LogWarn(("Malloc receive data fail"));
        return pdFAIL;
    }
    memcpy(message.data, pucMessage, xMessageLength);
    message.length = xMessageLength;
    
    // Set source identifier (use default if NULL)
    if (pcSource != NULL) {
        // Copy with explicit null termination
        size_t source_len = strnlen(pcSource, sizeof(message.source) - 1);
        memcpy(message.source, pcSource, source_len);
        message.source[source_len] = '\0'; // Properly terminate at the end of the actual string
    } else {
        // Use democonfigDEVICE_ID instead when pcSource is NULL
        size_t device_id_len = strlen(democonfigDEVICE_ID);
        memcpy(message.source, democonfigDEVICE_ID, device_id_len);
        message.source[device_id_len] = '\0'; // Ensure null-termination
    }
    
    // Put message into queue
    BaseType_t xStatus = xQueueSendToBack(xQueue, &message, pdMS_TO_TICKS(1000)); // Increased timeout
    if (xStatus != pdPASS) {
        LogError(("Failed to send message to queue - queue might be full"));
    } else {
        LogInfo(("Message added to telemetry queue, length: %d, source: %s", xMessageLength, message.source));
    }

    return xStatus;
}

/*-----------------------------------------------------------*/

/* Compile time error for undefined configs. */
#if !defined( democonfigHOSTNAME ) && !defined( democonfigENABLE_DPS_SAMPLE )
    #error "Define the config democonfigHOSTNAME by following the instructions in file demo_config.h."
#endif

#if !defined( democonfigENDPOINT ) && defined( democonfigENABLE_DPS_SAMPLE )
    #error "Define the config dps endpoint by following the instructions in file demo_config.h."
#endif

#ifndef democonfigROOT_CA_PEM
    #error "Please define Root CA certificate of the IoT Hub(democonfigROOT_CA_PEM) in demo_config.h."
#endif

#if defined( democonfigDEVICE_SYMMETRIC_KEY ) && defined( democonfigCLIENT_CERTIFICATE_PEM )
    #error "Please define only one auth democonfigDEVICE_SYMMETRIC_KEY or democonfigCLIENT_CERTIFICATE_PEM in demo_config.h."
#endif

#if !defined( democonfigDEVICE_SYMMETRIC_KEY ) && !defined( democonfigCLIENT_CERTIFICATE_PEM )
    #error "Please define one auth democonfigDEVICE_SYMMETRIC_KEY or democonfigCLIENT_CERTIFICATE_PEM in demo_config.h."
#endif

/*-----------------------------------------------------------*/

/**
 * @brief The maximum number of retries for network operation with server.
 */
#define sampleazureiotRETRY_MAX_ATTEMPTS                      ( 5U )

/**
 * @brief The maximum back-off delay (in milliseconds) for retrying failed operation
 *  with server.
 */
#define sampleazureiotRETRY_MAX_BACKOFF_DELAY_MS              ( 5000U )

/**
 * @brief The base back-off delay (in milliseconds) to use for network operation retry
 * attempts.
 */
#define sampleazureiotRETRY_BACKOFF_BASE_MS                   ( 500U )

/**
 * @brief Timeout for receiving CONNACK packet in milliseconds.
 */
#define sampleazureiotCONNACK_RECV_TIMEOUT_MS                 ( 10 * 1000U )

/**
 * @brief The Telemetry message published in this example.
 */
#define sampleazureiotMESSAGE                                 "{\"message\":\"Hello World %d!\"}"

/**
 * @brief  The content type of the Telemetry message published in this example.
 * @remark Message properties must be url-encoded.
 *         This message property is not required to send telemetry.
 */
#define sampleazureiotMESSAGE_CONTENT_TYPE                    "application%2Fjson"

/**
 * @brief  The content encoding of the Telemetry message published in this example.
 * @remark Message properties must be url-encoded.
 *         This message property is not required to send telemetry.
 */
#define sampleazureiotMESSAGE_CONTENT_ENCODING                "utf-8"

/**
 * @brief The reported property payload to send to IoT Hub
 */
#define sampleazureiotPROPERTY                                "{ \"PropertyIterationForCurrentConnection\": \"%d\" }"

/**
 * @brief Time in ticks to wait between each cycle of the demo implemented
 * by prvMQTTDemoTask().
 */
#define sampleazureiotDELAY_BETWEEN_DEMO_ITERATIONS_TICKS     ( pdMS_TO_TICKS( 50000U ) )

/**
 * @brief Timeout for MQTT_ProcessLoop in milliseconds.
 */
#define sampleazureiotPROCESS_LOOP_TIMEOUT_MS                 ( 500U )

/**
 * @brief Delay (in ticks) between consecutive cycles of MQTT publish operations in a
 * demo iteration.
 *
 * Note that the process loop also has a timeout, so the total time between
 * publishes is the sum of the two delays.
 */
#define sampleazureiotDELAY_BETWEEN_PUBLISHES_TICKS           ( pdMS_TO_TICKS( 7000U ) )

/**
 * @brief Transport timeout in milliseconds for transport send and receive.
 */
#define sampleazureiotTRANSPORT_SEND_RECV_TIMEOUT_MS          ( 2000U )

/**
 * @brief Transport timeout in milliseconds for transport send and receive.
 */
#define sampleazureiotProvisioning_Registration_TIMEOUT_MS    ( 3 * 1000U )

/**
 * @brief Wait timeout for subscribe to finish.
 */
#define sampleazureiotSUBSCRIBE_TIMEOUT                       ( 10 * 1000U )
/*-----------------------------------------------------------*/

/**
 * @brief Unix time.
 *
 * @return Time in milliseconds.
 */
uint64_t ullGetUnixTime( void );
/*-----------------------------------------------------------*/

/* Define buffer for IoT Hub info.  */
#ifdef democonfigENABLE_DPS_SAMPLE
    static uint8_t ucSampleIotHubHostname[ 128 ];
    static uint8_t ucSampleIotHubDeviceId[ 128 ];
    static AzureIoTProvisioningClient_t xAzureIoTProvisioningClient;
#endif /* democonfigENABLE_DPS_SAMPLE */

static uint8_t ucPropertyBuffer[ 80 ];
static uint8_t ucScratchBuffer[ 128 ];

/* Each compilation unit must define the NetworkContext struct. */
struct NetworkContext
{
    void * pParams;
};

static AzureIoTHubClient_t xAzureIoTHubClient;
/*-----------------------------------------------------------*/

#ifdef democonfigENABLE_DPS_SAMPLE

/**
 * @brief Gets the IoT Hub endpoint and deviceId from Provisioning service.
 *   This function will block for Provisioning service for result or return failure.
 *
 * @param[in] pXNetworkCredentials  Network credential used to connect to Provisioning service
 * @param[out] ppucIothubHostname  Pointer to uint8_t* IoT Hub hostname return from Provisioning Service
 * @param[in,out] pulIothubHostnameLength  Length of hostname
 * @param[out] ppucIothubDeviceId  Pointer to uint8_t* deviceId return from Provisioning Service
 * @param[in,out] pulIothubDeviceIdLength  Length of deviceId
 */
    static uint32_t prvIoTHubInfoGet( NetworkCredentials_t * pXNetworkCredentials,
                                      uint8_t ** ppucIothubHostname,
                                      uint32_t * pulIothubHostnameLength,
                                      uint8_t ** ppucIothubDeviceId,
                                      uint32_t * pulIothubDeviceIdLength );

#endif /* democonfigENABLE_DPS_SAMPLE */

/**
 * @brief The task used to demonstrate the MQTT API.
 *
 * @param[in] pvParameters Parameters as passed at the time of task creation. Not
 * used in this example.
 */
static void prvAzureDemoTask( void * pvParameters );

/**
 * @brief Connect to endpoint with reconnection retries.
 *
 * If connection fails, retry is attempted after a timeout.
 * Timeout value will exponentially increase until maximum
 * timeout value is reached or the number of attempts are exhausted.
 *
 * @param pcHostName Hostname of the endpoint to connect to.
 * @param ulPort Endpoint port.
 * @param pxNetworkCredentials Pointer to Network credentials.
 * @param pxNetworkContext Point to Network context created.
 * @return uint32_t The status of the final connection attempt.
 */
static uint32_t prvConnectToServerWithBackoffRetries( const char * pcHostName,
                                                      uint32_t ulPort,
                                                      NetworkCredentials_t * pxNetworkCredentials,
                                                      NetworkContext_t * pxNetworkContext );
/*-----------------------------------------------------------*/

/**
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static uint8_t ucMQTTMessageBuffer[ democonfigNETWORK_BUFFER_SIZE ];

/*-----------------------------------------------------------*/

/**
 * @brief Cloud message callback handler
 */
static void prvHandleCloudMessage( AzureIoTHubClientCloudToDeviceMessageRequest_t * pxMessage,
                                   void * pvContext )
{
    ( void ) pvContext;

    LogInfo( ( "Cloud message payload : %.*s \r\n",
               ( int ) pxMessage->ulPayloadLength,
               ( const char * ) pxMessage->pvMessagePayload ) );
}
/*-----------------------------------------------------------*/

/**
 * @brief Command message callback handler
 */
static void prvHandleCommand( AzureIoTHubClientCommandRequest_t * pxMessage,
                              void * pvContext )
{
    LogInfo( ( "Command payload : %.*s \r\n",
               ( int ) pxMessage->ulPayloadLength,
               ( const char * ) pxMessage->pvMessagePayload ) );

    AzureIoTHubClient_t * xHandle = ( AzureIoTHubClient_t * ) pvContext;

    if( AzureIoTHubClient_SendCommandResponse( xHandle, pxMessage, 200,
                                               NULL, 0 ) != eAzureIoTSuccess )
    {
        LogInfo( ( "Error sending command response\r\n" ) );
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief Property mesage callback handler
 */
static void prvHandlePropertiesMessage( AzureIoTHubClientPropertiesResponse_t * pxMessage,
                                        void * pvContext )
{
    ( void ) pvContext;

    switch( pxMessage->xMessageType )
    {
        case eAzureIoTHubPropertiesRequestedMessage:
            LogInfo( ( "Device property document GET received" ) );
            break;

        case eAzureIoTHubPropertiesReportedResponseMessage:
            LogInfo( ( "Device property reported property response received" ) );
            break;

        case eAzureIoTHubPropertiesWritablePropertyMessage:
            LogInfo( ( "Device property desired property received" ) );
            break;

        default:
            LogError( ( "Unknown property message" ) );
    }

    LogInfo( ( "Property document payload : %.*s \r\n",
               ( int ) pxMessage->ulPayloadLength,
               ( const char * ) pxMessage->pvMessagePayload ) );
}
/*-----------------------------------------------------------*/

/**
 * @brief Setup transport credentials.
 */
static uint32_t prvSetupNetworkCredentials( NetworkCredentials_t * pxNetworkCredentials )
{
    pxNetworkCredentials->xDisableSni = pdFALSE;
    /* Set the credentials for establishing a TLS connection. */
    pxNetworkCredentials->pucRootCa = ( const unsigned char * ) democonfigROOT_CA_PEM;
    pxNetworkCredentials->xRootCaSize = sizeof( democonfigROOT_CA_PEM );
    #ifdef democonfigCLIENT_CERTIFICATE_PEM
        pxNetworkCredentials->pucClientCert = ( const unsigned char * ) democonfigCLIENT_CERTIFICATE_PEM;
        pxNetworkCredentials->xClientCertSize = sizeof( democonfigCLIENT_CERTIFICATE_PEM );
        pxNetworkCredentials->pucPrivateKey = ( const unsigned char * ) democonfigCLIENT_PRIVATE_KEY_PEM;
        pxNetworkCredentials->xPrivateKeySize = sizeof( democonfigCLIENT_PRIVATE_KEY_PEM );
    #endif

    return 0;
}
/*-----------------------------------------------------------*/

/**
 * @brief Get WiFi MAC address
 */
static void prvGetWiFiMacAddress(char *macStr, size_t maxLen) {
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(macStr, maxLen, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    LogInfo(("Device WiFi MAC Address: %s", macStr));
}

/**
 * @brief Azure IoT demo task that gets started in the platform specific project.
 *  In this demo task, middleware API's are used to connect to Azure IoT Hub.
 */
static void prvAzureDemoTask( void * pvParameters )
{
    int lPublishCount = 0;
    uint32_t ulScratchBufferLength = 0U;
    const int lMaxPublishCount = 2;
    NetworkCredentials_t xNetworkCredentials = { 0 };
    AzureIoTTransportInterface_t xTransport;
    NetworkContext_t xNetworkContext = { 0 };
    TlsTransportParams_t xTlsTransportParams = { 0 };
    AzureIoTResult_t xResult;
    uint32_t ulStatus;
    AzureIoTHubClientOptions_t xHubOptions = { 0 };
    AzureIoTMessageProperties_t xPropertyBag;
    bool xSessionPresent;

    #ifdef democonfigENABLE_DPS_SAMPLE
        uint8_t * pucIotHubHostname = NULL;
        uint8_t * pucIotHubDeviceId = NULL;
        uint32_t pulIothubHostnameLength = 0;
        uint32_t pulIothubDeviceIdLength = 0;
    #else
        uint8_t * pucIotHubHostname = ( uint8_t * ) democonfigHOSTNAME;
        uint8_t * pucIotHubDeviceId = ( uint8_t * ) democonfigDEVICE_ID;
        uint32_t pulIothubHostnameLength = sizeof( democonfigHOSTNAME ) - 1;
        uint32_t pulIothubDeviceIdLength = sizeof( democonfigDEVICE_ID ) - 1;
    #endif /* democonfigENABLE_DPS_SAMPLE */

    ( void ) pvParameters;

    // Delay before initializing Azure IoT
    vTaskDelay( pdMS_TO_TICKS( 17000U ) );

    /* Initialize Azure IoT Middleware.  */
    configASSERT( AzureIoT_Init() == eAzureIoTSuccess );

    ulStatus = prvSetupNetworkCredentials( &xNetworkCredentials );
    configASSERT( ulStatus == 0 );

    #ifdef democonfigENABLE_DPS_SAMPLE
        /* Run DPS.  */
        if( ( ulStatus = prvIoTHubInfoGet( &xNetworkCredentials, &pucIotHubHostname,
                                           &pulIothubHostnameLength, &pucIotHubDeviceId,
                                           &pulIothubDeviceIdLength ) ) != 0 )
        {
            LogError( ( "Failed on sample_dps_entry!: error code = 0x%08x\r\n", ( uint16_t ) ulStatus ) );
            return;
        }
    #endif /* democonfigENABLE_DPS_SAMPLE */

    xNetworkContext.pParams = &xTlsTransportParams;

    for( ; ; )
    {
        if( xAzureSample_IsConnectedToInternet() )
        {
            /* Attempt to establish TLS session with IoT Hub. If connection fails,
             * retry after a timeout. Timeout value will be exponentially increased
             * until  the maximum number of attempts are reached or the maximum timeout
             * value is reached. The function returns a failure status if the TCP
             * connection cannot be established to the IoT Hub after the configured
             * number of attempts. */
            ulStatus = prvConnectToServerWithBackoffRetries( ( const char * ) pucIotHubHostname,
                                                             democonfigIOTHUB_PORT,
                                                             &xNetworkCredentials, &xNetworkContext );
            configASSERT( ulStatus == 0 );

            /* Fill in Transport Interface send and receive function pointers. */
            xTransport.pxNetworkContext = &xNetworkContext;
            xTransport.xSend = TLS_Socket_Send;
            xTransport.xRecv = TLS_Socket_Recv;

            /* Init IoT Hub option */
            xResult = AzureIoTHubClient_OptionsInit( &xHubOptions );
            configASSERT( xResult == eAzureIoTSuccess );

            xHubOptions.pucModuleID = ( const uint8_t * ) democonfigMODULE_ID;
            xHubOptions.ulModuleIDLength = sizeof( democonfigMODULE_ID ) - 1;

            xResult = AzureIoTHubClient_Init( &xAzureIoTHubClient,
                                              pucIotHubHostname, pulIothubHostnameLength,
                                              pucIotHubDeviceId, pulIothubDeviceIdLength,
                                              &xHubOptions,
                                              ucMQTTMessageBuffer, sizeof( ucMQTTMessageBuffer ),
                                              ullGetUnixTime,
                                              &xTransport );
            configASSERT( xResult == eAzureIoTSuccess );

            #ifdef democonfigDEVICE_SYMMETRIC_KEY
                xResult = AzureIoTHubClient_SetSymmetricKey( &xAzureIoTHubClient,
                                                             ( const uint8_t * ) democonfigDEVICE_SYMMETRIC_KEY,
                                                             sizeof( democonfigDEVICE_SYMMETRIC_KEY ) - 1,
                                                             Crypto_HMAC );
                configASSERT( xResult == eAzureIoTSuccess );
            #endif /* democonfigDEVICE_SYMMETRIC_KEY */

            /* Sends an MQTT Connect packet over the already established TLS connection,
             * and waits for connection acknowledgment (CONNACK) packet. */
            LogInfo( ( "Creating an MQTT connection to %s.\r\n", pucIotHubHostname ) );

            xResult = AzureIoTHubClient_Connect( &xAzureIoTHubClient,
                                                 false, &xSessionPresent,
                                                 sampleazureiotCONNACK_RECV_TIMEOUT_MS );
            configASSERT( xResult == eAzureIoTSuccess );

            xResult = AzureIoTHubClient_SubscribeCloudToDeviceMessage( &xAzureIoTHubClient, prvHandleCloudMessage,
                                                                       &xAzureIoTHubClient, sampleazureiotSUBSCRIBE_TIMEOUT );
            configASSERT( xResult == eAzureIoTSuccess );

            xResult = AzureIoTHubClient_SubscribeCommand( &xAzureIoTHubClient, prvHandleCommand,
                                                          &xAzureIoTHubClient, sampleazureiotSUBSCRIBE_TIMEOUT );
            configASSERT( xResult == eAzureIoTSuccess );

            xResult = AzureIoTHubClient_SubscribeProperties( &xAzureIoTHubClient, prvHandlePropertiesMessage,
                                                             &xAzureIoTHubClient, sampleazureiotSUBSCRIBE_TIMEOUT );
            configASSERT( xResult == eAzureIoTSuccess );

            /* Get property document after initial connection */
            xResult = AzureIoTHubClient_RequestPropertiesAsync( &xAzureIoTHubClient );
            configASSERT( xResult == eAzureIoTSuccess );

            /* Send device MAC address as reported property */
            char macAddress[18];
            prvGetWiFiMacAddress(macAddress, sizeof(macAddress));
            
            LogInfo(("Sending device MAC address as reported property..."));
            ulScratchBufferLength = snprintf((char*)ucScratchBuffer, sizeof(ucScratchBuffer),
                "{ \"deviceInfo\": { \"wifiMacAddress\": \"%s\" } }", macAddress);
                
            xResult = AzureIoTHubClient_SendPropertiesReported(&xAzureIoTHubClient,
                ucScratchBuffer, ulScratchBufferLength, NULL);
            
            if (xResult == eAzureIoTSuccess) {
                LogInfo(("Successfully sent MAC address to device twin"));
            } else {
                LogError(("Failed to send MAC address to device twin, error: %d", xResult));
            }
            configASSERT(xResult == eAzureIoTSuccess);

            /* Publish messages with QoS1, send and process Keep alive messages. */
            for( lPublishCount = 0;
                 lPublishCount < lMaxPublishCount && xAzureSample_IsConnectedToInternet();
                 lPublishCount++ )
            {
                QueueHandle_t xQueue = azure_iot_get_telemetry_queue();
                if (xQueue != NULL) {
                    TelemetryMessage_t receivedMessage;
                    
                    for (int i = 0;
                         i < 10 && xQueueReceive(xQueue, &receivedMessage, pdMS_TO_TICKS(1000)) == pdTRUE;
                         i++) {
                        LogInfo(("Retrieved message from queue, length: %d, source: %s, sending to Azure IoT Hub", 
                                receivedMessage.length, receivedMessage.source));
                        
                        // Copy data to scratch buffer if needed, or use directly
                        memcpy(ucScratchBuffer, receivedMessage.data, receivedMessage.length);
                        free(receivedMessage.data);
                        ulScratchBufferLength = receivedMessage.length;
                        
                        // Create a bag of properties for the telemetry (or reset if already exists)
                        xResult = AzureIoTMessage_PropertiesInit(&xPropertyBag, ucPropertyBuffer, 0, sizeof(ucPropertyBuffer));
                        configASSERT(xResult == eAzureIoTSuccess);

                        // Add standard properties
                        xResult = AzureIoTMessage_PropertiesAppend(&xPropertyBag,
                                                                  (uint8_t *)AZ_IOT_MESSAGE_PROPERTIES_CONTENT_TYPE, 
                                                                  sizeof(AZ_IOT_MESSAGE_PROPERTIES_CONTENT_TYPE) - 1,
                                                                  (uint8_t *)sampleazureiotMESSAGE_CONTENT_TYPE, 
                                                                  sizeof(sampleazureiotMESSAGE_CONTENT_TYPE) - 1);
                        configASSERT(xResult == eAzureIoTSuccess);

                        xResult = AzureIoTMessage_PropertiesAppend(&xPropertyBag,
                                                                  (uint8_t *)AZ_IOT_MESSAGE_PROPERTIES_CONTENT_ENCODING, 
                                                                  sizeof(AZ_IOT_MESSAGE_PROPERTIES_CONTENT_ENCODING) - 1,
                                                                  (uint8_t *)sampleazureiotMESSAGE_CONTENT_ENCODING, 
                                                                  sizeof(sampleazureiotMESSAGE_CONTENT_ENCODING) - 1);
                        configASSERT(xResult == eAzureIoTSuccess);

                        // Add source property from the message
                        xResult = AzureIoTMessage_PropertiesAppend(&xPropertyBag, 
                                                                  (uint8_t *)"source", sizeof("source") - 1,
                                                                  (uint8_t *)receivedMessage.source, 
                                                                  strlen(receivedMessage.source));
                        configASSERT(xResult == eAzureIoTSuccess);
                        
                        xResult = AzureIoTHubClient_SendTelemetry(&xAzureIoTHubClient,
                                                                 ucScratchBuffer, ulScratchBufferLength,
                                                                 &xPropertyBag, eAzureIoTHubMessageQoS1, NULL);
                        configASSERT(xResult == eAzureIoTSuccess);
                    }
                } else {
                    LogError(("Telemetry queue not available"));
                    // Add appropriate error handling or delay
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                LogInfo( ( "Attempt to receive publish message from IoT Hub.\r\n" ) );
                xResult = AzureIoTHubClient_ProcessLoop( &xAzureIoTHubClient,
                                                         sampleazureiotPROCESS_LOOP_TIMEOUT_MS );
                if( xResult != eAzureIoTSuccess )
                {
                    LogError( ( "ProcessLoop failed with error %d, breaking out of publish loop to reconnect", xResult ) );
                    break; // Exit the publishing loop to attempt reconnection
                }

                if( lPublishCount % 2 == 0 )
                {
                    /* Send reported property every other cycle */
                    ulScratchBufferLength = snprintf( ( char * ) ucScratchBuffer, sizeof( ucScratchBuffer ),
                                                      sampleazureiotPROPERTY, lPublishCount / 2 + 1 );
                    xResult = AzureIoTHubClient_SendPropertiesReported( &xAzureIoTHubClient,
                                                                        ucScratchBuffer, ulScratchBufferLength,
                                                                        NULL );
                    configASSERT( xResult == eAzureIoTSuccess );
                }

                /* Leave Connection Idle for some time. */
                LogInfo( ( "This thread has %u bytes free stack\n", uxTaskGetStackHighWaterMark(NULL) ) );
                LogInfo( ( "Keeping Connection Idle...\r\n\r\n" ) );
                vTaskDelay( sampleazureiotDELAY_BETWEEN_PUBLISHES_TICKS );

#ifndef DEEP_SLEEP_TIME
                // // If we are not sleeping, this would be a forever loop.
                // lPublishCount--;
#endif                
            }

            if( xAzureSample_IsConnectedToInternet() )
            {
                xResult = AzureIoTHubClient_UnsubscribeProperties( &xAzureIoTHubClient );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTHubClient_UnsubscribeCommand( &xAzureIoTHubClient );
                configASSERT( xResult == eAzureIoTSuccess );

                xResult = AzureIoTHubClient_UnsubscribeCloudToDeviceMessage( &xAzureIoTHubClient );
                configASSERT( xResult == eAzureIoTSuccess );

                /* Send an MQTT Disconnect packet over the already connected TLS over
                 * TCP connection. There is no corresponding response for the disconnect
                 * packet. After sending disconnect, client must close the network
                 * connection. */
                xResult = AzureIoTHubClient_Disconnect( &xAzureIoTHubClient );
                configASSERT( xResult == eAzureIoTSuccess );
            }

            /* Close the network connection.  */
            TLS_Socket_Disconnect( &xNetworkContext );

            /* Wait for some time between two iterations to ensure that we do not
             * bombard the IoT Hub. */
            LogInfo( ( "Demo completed successfully.\r\n" ) );
        }

#ifdef DEEP_SLEEP_TIME
            // If we are sleeping, go to sleep
            LogInfo( ( "Setting up deep sleep for %ds\n", DEEP_SLEEP_TIME ) );

            esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME * 1000000);

            // enter deep sleep
            esp_deep_sleep_start();
#endif                

        LogInfo( ( "Short delay before starting the next iteration.... \r\n\r\n" ) );
        vTaskDelay( sampleazureiotDELAY_BETWEEN_DEMO_ITERATIONS_TICKS );
    }
}
/*-----------------------------------------------------------*/

#ifdef democonfigENABLE_DPS_SAMPLE

/**
 * @brief Get IoT Hub endpoint and device Id info, when Provisioning service is used.
 *   This function will block for Provisioning service for result or return failure.
 */
    static uint32_t prvIoTHubInfoGet( NetworkCredentials_t * pXNetworkCredentials,
                                      uint8_t ** ppucIothubHostname,
                                      uint32_t * pulIothubHostnameLength,
                                      uint8_t ** ppucIothubDeviceId,
                                      uint32_t * pulIothubDeviceIdLength )
    {
        NetworkContext_t xNetworkContext = { 0 };
        TlsTransportParams_t xTlsTransportParams = { 0 };
        AzureIoTResult_t xResult;
        AzureIoTTransportInterface_t xTransport;
        uint32_t ucSamplepIothubHostnameLength = sizeof( ucSampleIotHubHostname );
        uint32_t ucSamplepIothubDeviceIdLength = sizeof( ucSampleIotHubDeviceId );
        uint32_t ulStatus;

        /* Set the pParams member of the network context with desired transport. */
        xNetworkContext.pParams = &xTlsTransportParams;

        ulStatus = prvConnectToServerWithBackoffRetries( democonfigENDPOINT, democonfigIOTHUB_PORT,
                                                         pXNetworkCredentials, &xNetworkContext );
        configASSERT( ulStatus == 0 );

        /* Fill in Transport Interface send and receive function pointers. */
        xTransport.pxNetworkContext = &xNetworkContext;
        xTransport.xSend = TLS_Socket_Send;
        xTransport.xRecv = TLS_Socket_Recv;

        #ifdef democonfigUSE_HSM

            /* Redefine the democonfigREGISTRATION_ID macro using registration ID
             * generated dynamically using the HSM */

            /* We use a pointer instead of a buffer so that the getRegistrationId
             * function can allocate the necessary memory depending on the HSM */
            char * registration_id = NULL;
            ulStatus = getRegistrationId( &registration_id );
            configASSERT( ulStatus == 0 );
#undef democonfigREGISTRATION_ID
        #define democonfigREGISTRATION_ID    registration_id
        #endif

        xResult = AzureIoTProvisioningClient_Init( &xAzureIoTProvisioningClient,
                                                   ( const uint8_t * ) democonfigENDPOINT,
                                                   sizeof( democonfigENDPOINT ) - 1,
                                                   ( const uint8_t * ) democonfigID_SCOPE,
                                                   sizeof( democonfigID_SCOPE ) - 1,
                                                   ( const uint8_t * ) democonfigREGISTRATION_ID,
                                                   #ifdef democonfigUSE_HSM
                                                       strlen( democonfigREGISTRATION_ID ),
                                                   #else
                                                       sizeof( democonfigREGISTRATION_ID ) - 1,
                                                   #endif
                                                   NULL, ucMQTTMessageBuffer, sizeof( ucMQTTMessageBuffer ),
                                                   ullGetUnixTime,
                                                   &xTransport );
        configASSERT( xResult == eAzureIoTSuccess );

        #ifdef democonfigDEVICE_SYMMETRIC_KEY
            xResult = AzureIoTProvisioningClient_SetSymmetricKey( &xAzureIoTProvisioningClient,
                                                                  ( const uint8_t * ) democonfigDEVICE_SYMMETRIC_KEY,
                                                                  sizeof( democonfigDEVICE_SYMMETRIC_KEY ) - 1,
                                                                  Crypto_HMAC );
            configASSERT( xResult == eAzureIoTSuccess );
        #endif /* democonfigDEVICE_SYMMETRIC_KEY */

        do
        {
            xResult = AzureIoTProvisioningClient_Register( &xAzureIoTProvisioningClient,
                                                           sampleazureiotProvisioning_Registration_TIMEOUT_MS );
        } while( xResult == eAzureIoTErrorPending );

        configASSERT( xResult == eAzureIoTSuccess );

        xResult = AzureIoTProvisioningClient_GetDeviceAndHub( &xAzureIoTProvisioningClient,
                                                              ucSampleIotHubHostname, &ucSamplepIothubHostnameLength,
                                                              ucSampleIotHubDeviceId, &ucSamplepIothubDeviceIdLength );
        configASSERT( xResult == eAzureIoTSuccess );

        AzureIoTProvisioningClient_Deinit( &xAzureIoTProvisioningClient );

        /* Close the network connection.  */
        TLS_Socket_Disconnect( &xNetworkContext );

        *ppucIothubHostname = ucSampleIotHubHostname;
        *pulIothubHostnameLength = ucSamplepIothubHostnameLength;
        *ppucIothubDeviceId = ucSampleIotHubDeviceId;
        *pulIothubDeviceIdLength = ucSamplepIothubDeviceIdLength;

        return 0;
    }

#endif /* democonfigENABLE_DPS_SAMPLE */
/*-----------------------------------------------------------*/

/**
 * @brief Connect to server with backoff retries.
 */
static uint32_t prvConnectToServerWithBackoffRetries( const char * pcHostName,
                                                      uint32_t port,
                                                      NetworkCredentials_t * pxNetworkCredentials,
                                                      NetworkContext_t * pxNetworkContext )
{
    TlsTransportStatus_t xNetworkStatus;
    BackoffAlgorithmStatus_t xBackoffAlgStatus = BackoffAlgorithmSuccess;
    BackoffAlgorithmContext_t xReconnectParams;
    uint16_t usNextRetryBackOff = 0U;

    /* Initialize reconnect attempts and interval. */
    BackoffAlgorithm_InitializeParams( &xReconnectParams,
                                       sampleazureiotRETRY_BACKOFF_BASE_MS,
                                       sampleazureiotRETRY_MAX_BACKOFF_DELAY_MS,
                                       sampleazureiotRETRY_MAX_ATTEMPTS );

    /* Attempt to connect to IoT Hub. If connection fails, retry after
     * a timeout. Timeout value will exponentially increase till maximum
     * attempts are reached.
     */
    do
    {
        LogInfo( ( "Creating a TLS connection to %s:%lu.\r\n", pcHostName, port ) );
        /* Attempt to create a mutually authenticated TLS connection. */
        xNetworkStatus = TLS_Socket_Connect( pxNetworkContext,
                                             pcHostName, port,
                                             pxNetworkCredentials,
                                             sampleazureiotTRANSPORT_SEND_RECV_TIMEOUT_MS,
                                             sampleazureiotTRANSPORT_SEND_RECV_TIMEOUT_MS );

        if( xNetworkStatus != eTLSTransportSuccess )
        {
            /* Generate a random number and calculate backoff value (in milliseconds) for
             * the next connection retry.
             * Note: It is recommended to seed the random number generator with a device-specific
             * entropy source so that possibility of multiple devices retrying failed network operations
             * at similar intervals can be avoided. */
            xBackoffAlgStatus = BackoffAlgorithm_GetNextBackoff( &xReconnectParams, configRAND32(), &usNextRetryBackOff );

            if( xBackoffAlgStatus == BackoffAlgorithmRetriesExhausted )
            {
                LogError( ( "Connection to the IoT Hub failed, all attempts exhausted." ) );
            }
            else if( xBackoffAlgStatus == BackoffAlgorithmSuccess )
            {
                LogWarn( ( "Connection to the IoT Hub failed [%d]. "
                           "Retrying connection with backoff and jitter [%d]ms.",
                           xNetworkStatus, usNextRetryBackOff ) );
                vTaskDelay( pdMS_TO_TICKS( usNextRetryBackOff ) );
            }
        }
    } while( ( xNetworkStatus != eTLSTransportSuccess ) && ( xBackoffAlgStatus == BackoffAlgorithmSuccess ) );

    return xNetworkStatus == eTLSTransportSuccess ? 0 : 1;
}
/*-----------------------------------------------------------*/

/*
 * @brief Create the task that demonstrates the AzureIoTHub demo
 */
void vStartAzureIoT( void )
{
    /* This example uses a single application task, which in turn is used to
     * connect, subscribe, publish, unsubscribe and disconnect from the IoT Hub */
    xTaskCreatePinnedToCore(prvAzureDemoTask, /* Function that implements the task. */
                            "AzureDemoTask",  /* Text name for the task - only used for debugging. */
                            8192,             /* Size of stack (in words, not bytes) to allocate for the task. */
                            NULL,             /* Task parameter - not used in this case. */
                            tskIDLE_PRIORITY, /* Task priority, must be between 0 and configMAX_PRIORITIES - 1. */
                            NULL,             // Task handle
                            1                 // Core
    );
}
/*-----------------------------------------------------------*/

