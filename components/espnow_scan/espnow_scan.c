#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"

#include "azure-iot.h"

#define ESPNOW_QUEUE_SIZE           6

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

/* When ESPNOW receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    char addr_str[18]; // XX:XX:XX:XX:XX:XX\0
    signed rssi;
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef struct {
    uint16_t version;                     //Sequence number of ESPNOW data.
    uint32_t crc;                         //CRC16 value of ESPNOW data.
    uint8_t payload[0];                   //Real payload of ESPNOW data.
} __attribute__((packed)) espnow_data_t;

typedef struct {
    uint16_t version;                     //Sequence number of ESPNOW data.
    uint32_t crc;                         //CRC16 value of ESPNOW data.
    uint32_t bootCount;
    float voltage;
    int weight;
} __attribute__((packed)) espnow_data_7601_t;



#define ESPNOW_MAXDELAY 512

static const char *TAG = "espnow_scan";

static QueueHandle_t s_espnow_queue;

static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

static char *addr_to_str(const uint8_t *addr)
{
    static char buf[18];
    
    // Format: XX:XX:XX:XX:XX:XX
    sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
           addr[5], addr[4], addr[3],
           addr[2], addr[1], addr[0]);
    
    return buf;
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    espnow_event_recv_cb_t recv_cb;
    uint8_t * mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    recv_cb.rssi = recv_info->rx_ctrl->rssi;

    // Copy device address
    strncpy(recv_cb.addr_str, addr_to_str(mac_addr), sizeof(recv_cb.addr_str) - 1);
    recv_cb.addr_str[sizeof(recv_cb.addr_str) - 1] = '\0'; // Ensure null termination

    recv_cb.data = malloc(len);
    if (recv_cb.data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(recv_cb.data, data, len);
    recv_cb.data_len = len;
    if (xQueueSend(s_espnow_queue, &recv_cb, ESPNOW_MAXDELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(recv_cb.data);
    }
}

/* Validate received ESPNOW data. */
int espnow_data_validate(uint8_t *data, uint16_t data_len)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint32_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    crc = buf->crc;
    buf->crc = 0;
    // CRC-32/CKSUM
    // width=32 poly=0x04c11db7 init=0x00000000 refin=false refout=false xorout=0xffffffff check=0x765e7680 residue=0xc704dd7b name="CRC-32/CKSUM"
    crc_cal = esp_crc32_be(UINT32_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->version;
    }

    return -1;
}

/* Process advertisement data */
// size of TelemetryBuffer is AZURE_IOT_TELEMETRY_MAXLEN
size_t process_espnow_data(uint8_t *TelemetryBuffer, uint16_t version, const uint8_t *data, uint8_t data_len, const char *addr_str, int8_t rssi)
{
    size_t TelemetryBufferLength = 0;

    if (version == 0x7601 && data_len == sizeof(espnow_data_7601_t))
    {
        ESP_LOGI(TAG, "Receive broadcast data from: %s, len: %d, rssi: %d", addr_str, data_len, rssi);

        espnow_data_7601_t *data_7601 = (espnow_data_7601_t *)data;
        ESP_LOGI(TAG, "Boot count: %lu, weight: %d, voltage: %f", data_7601->bootCount, data_7601->weight, data_7601->voltage);

        // Create JSON with all required variables
        TelemetryBufferLength = snprintf((char *)TelemetryBuffer, AZURE_IOT_TELEMETRY_MAXLEN,
                                         "{\"battery_voltage\":%.2f,\"boot_count\":%lu,\"weight\":%u}",
                                         data_7601->voltage, data_7601->bootCount, data_7601->weight);

        // Check for snprintf buffer overflow
        if (TelemetryBufferLength >= AZURE_IOT_TELEMETRY_MAXLEN)
        {
            ESP_LOGW(TAG, "JSON message truncated, original length: %d", TelemetryBufferLength);
            TelemetryBufferLength = AZURE_IOT_TELEMETRY_MAXLEN - 1;
        }

        ESP_LOGI(TAG, "Sensor data prepared: %s", TelemetryBuffer);
    }
    else
    {
        ESP_LOGI(TAG, "Receive error data from: %s", addr_str);
        for (int i = 0; i < data_len; i++)
        {
            ESP_LOGI(TAG, "Received data, byte %d: 0x%02X", i, ((unsigned char *)data)[i]);
        }
    }
    return TelemetryBufferLength;
}

static void espnow_task(void *pvParameter)
{
    espnow_event_recv_cb_t recv_cb;
    int ret;

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Start espnow task");

    while (1) {
        if (xQueueReceive(s_espnow_queue, &recv_cb, portMAX_DELAY) == pdTRUE) {

            ret = espnow_data_validate(recv_cb.data, recv_cb.data_len);

            // Variable declarations
            uint8_t TelemetryBuffer[AZURE_IOT_TELEMETRY_MAXLEN];
            size_t TelemetryBufferLength = 0;
            BaseType_t queueResult;

            TelemetryBufferLength = process_espnow_data(TelemetryBuffer, ret, recv_cb.data, recv_cb.data_len, recv_cb.addr_str, recv_cb.rssi);

            if (TelemetryBufferLength > 0) {
                // Queue the telemetry data with addr_str as source identifier
                queueResult = azure_iot_queue_telemetry(TelemetryBuffer, TelemetryBufferLength, recv_cb.addr_str);
                if (queueResult != pdPASS) {
                    ESP_LOGE(TAG, "Failed to queue telemetry data");
                } else {
                    ESP_LOGI(TAG, "Successfully queued telemetry data from device: %s", recv_cb.addr_str);
                }
            }
            free(recv_cb.data);
        }
    }
}

static esp_err_t espnow_init(void)
{
    s_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_recv_cb_t));
    if (s_espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
#if CONFIG_ESPNOW_ENABLE_POWER_SAVE
    ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
    ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
#endif
    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(s_espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = 1;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = false;
    memcpy(peer->peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    xTaskCreatePinnedToCore(espnow_task, "espnow_task", 8192, NULL, 4, NULL, 1);

    return ESP_OK;
}

void espnow_scan_init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // example_wifi_init();
    espnow_init();
}
