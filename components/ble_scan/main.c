#include "esp_log.h"
#include "nvs_flash.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "esp_timer.h"
#include "azure-iot.h"

static const char *TAG = "BLE_SCANNER";
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static void ble_scan(void);

// 10 seconds in milliseconds
#define SCAN_DURATION_MS 10000

/* Convert BLE address to string */
static char *addr_to_str(const ble_addr_t *addr)
{
    static char buf[18];
    
    // Format: XX:XX:XX:XX:XX:XX
    sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
           addr->val[5], addr->val[4], addr->val[3],
           addr->val[2], addr->val[1], addr->val[0]);
    
    return buf;
}

/* Print advertisement data */
static void print_adv_fields(const struct ble_hs_adv_fields *fields, const char *addr_str, int8_t rssi)
{
    char name[BLE_HS_ADV_MAX_SZ];
    uint8_t i;

    ESP_LOGI(TAG, "--------------------------------------");

    // Print device address
    ESP_LOGI(TAG, "Device Address: %s", addr_str);
    // Print RSSI
    ESP_LOGI(TAG, "RSSI: %d", rssi);

    // Print device name if exists
    if (fields->name_len > 0) {
        memcpy(name, fields->name, fields->name_len);
        name[fields->name_len] = '\0';
        ESP_LOGI(TAG, "Device Name: %s", name);
    }

    // Print TX power level
    if (fields->tx_pwr_lvl_is_present) {
        ESP_LOGI(TAG, "TX Power Level: %d", fields->tx_pwr_lvl);
    }

    // Print flags
    if (fields->flags != 0) {
        ESP_LOGI(TAG, "Flags: 0x%02x", fields->flags);
    }

    // Print 16-bit service UUIDs
    if (fields->uuids16 != NULL) {
        ESP_LOGI(TAG, "16-bit Service UUIDs:");
        for (i = 0; i < fields->num_uuids16; i++) {
            // Using %u instead of %x to avoid format error
            uint16_t uuid = ble_uuid_u16(&fields->uuids16[i].u);
            ESP_LOGI(TAG, "  0x%04x", uuid);
        }
    }

    // Print 32-bit service UUIDs
    if (fields->uuids32 != NULL) {
        ESP_LOGI(TAG, "32-bit Service UUIDs:");
        for (i = 0; i < fields->num_uuids32; i++) {
            // Using a temporary variable and %lu format for 32-bit values
            unsigned long uuid_val = (unsigned long)ble_uuid_u16(&fields->uuids32[i].u);
            ESP_LOGI(TAG, "  0x%08lu", uuid_val);
        }
    }

    // Print 128-bit service UUIDs
    if (fields->uuids128 != NULL) {
        ESP_LOGI(TAG, "128-bit Service UUIDs:");
        for (i = 0; i < fields->num_uuids128; i++) {
            const uint8_t *uuid = fields->uuids128[i].value;
            // Print byte by byte to avoid format issues
            ESP_LOGI(TAG, "  %02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                     uuid[15], uuid[14], uuid[13], uuid[12], 
                     uuid[11], uuid[10], uuid[9], uuid[8],
                     uuid[7], uuid[6], uuid[5], uuid[4], 
                     uuid[3], uuid[2], uuid[1], uuid[0]);
        }
    }

    // Print manufacturer specific data if present
    if (fields->mfg_data_len > 0) {
        ESP_LOGI(TAG, "Manufacturer Specific Data (%d bytes):", fields->mfg_data_len);
        ESP_LOG_BUFFER_HEX(TAG, fields->mfg_data, fields->mfg_data_len);
    }

    ESP_LOGI(TAG, "--------------------------------------");
}

/* Print advertisement data */
static void process_ble_adv(const struct ble_hs_adv_fields *fields, const char *addr_str, int8_t rssi)
{
    // Print device address and RSSI
    ESP_LOGI(TAG, "Device Address: %s, RSSI: %d", addr_str, rssi);

    // Manufacturer data 0xEF71
    if (fields->mfg_data_len >= 14 && 
        fields->mfg_data[0] == 0x71 && fields->mfg_data[1] == 0xEF) {
        print_adv_fields(fields, addr_str, rssi);

        float battery_voltage = 0.03125 * fields->mfg_data[2];
        float chip_temperature = 0.01 * ( (fields->mfg_data[4] << 8) | fields->mfg_data[3]);
        float temperature = ((fields->mfg_data[6] << 8) | fields->mfg_data[5]) * 165 / 65536 - 40;
        float humidity = ((fields->mfg_data[8] << 8) | fields->mfg_data[7]) * 100 / 65536;
        unsigned long uptime = ( (fields->mfg_data[15] << 24) | (fields->mfg_data[14] << 16) | (fields->mfg_data[13] << 8) | fields->mfg_data[12]);

        ESP_LOGI(TAG, "voltage: %f, chip temperature: %f, temperature: %f, humidity: %f, uptime: %lu", battery_voltage, chip_temperature, temperature, humidity, uptime);

        // // Variable declarations
        // uint8_t TelemetryBuffer[AZURE_IOT_TELEMETRY_MAXLEN];
        // size_t TelemetryBufferLength = 0;
        // BaseType_t queueResult;

        // // Create JSON with all required variables
        // TelemetryBufferLength = snprintf((char *)TelemetryBuffer, sizeof(TelemetryBuffer),
        //                     "{\"battery_voltage\":%.2f,\"chip_temperature\":%.2f,\"temperature\":%.2f,\"humidity\":%.2f,\"uptime\":%lu}",
        //                     battery_voltage, chip_temperature, temperature, humidity, uptime);

        // // Check for snprintf buffer overflow
        // if (TelemetryBufferLength >= sizeof(TelemetryBuffer)) {
        //     ESP_LOGW(TAG, "JSON message truncated, original length: %d", TelemetryBufferLength);
        //     TelemetryBufferLength = sizeof(TelemetryBuffer) - 1;
        // }

        // ESP_LOGI(TAG, "Sensor data prepared: %s", TelemetryBuffer);

        // // Queue the telemetry data with addr_str as source identifier
        // queueResult = azure_iot_queue_telemetry(TelemetryBuffer, TelemetryBufferLength, addr_str);
        // if (queueResult != pdPASS) {
        //     ESP_LOGE(TAG, "Failed to queue telemetry data");
        // } else {
        //     ESP_LOGI(TAG, "Successfully queued telemetry data from device: %s", addr_str);
        // }
    }
}

/**
 * Initiates the GAP general discovery procedure.
 */
static void ble_scan(void)
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }

    /* Filter duplicates during the 10-second scan period */
    disc_params.filter_duplicates = 1;
    /**
     * Perform a passive scan.  I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    ESP_LOGI(TAG, "Starting 10-second BLE scan...");
    /* Set scan duration to 10 seconds */
    rc = ble_gap_disc(own_addr_type, SCAN_DURATION_MS, &disc_params,
                      ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error initiating GAP discovery procedure; rc=%d", rc);
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.
 */
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                     event->disc.length_data);
        if (rc != 0) {
            return 0;
        }

        /* An advertisement report was received during GAP discovery. */
        process_ble_adv(&fields, addr_to_str(&event->disc.addr), event->disc.rssi);
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "Discovery complete; reason=%d",
                 event->disc_complete.reason);
        /* Restart scanning when current scan completes */
        ESP_LOGI(TAG, "Restarting scan...");
        ble_scan();
        return 0;

    default:
        return 0;
    }
}

static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

static void ble_on_sync(void)
{
    int rc;

    /* Make sure we have proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to ensure address, rc=%d", rc);
        return;
    }

    /* Begin scanning for devices */
    ble_scan();
}

void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void ble_scan_init(void)
{
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init nimble %d", ret);
        return;
    }

    /* Configure the host. */
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Set the default device name. */
    ret = ble_svc_gap_device_name_set("ble-scanner");
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set device name, ret=%d", ret);
        return;
    }

    /* Initialize the NimBLE host task */
    nimble_port_freertos_init(ble_host_task);
}
