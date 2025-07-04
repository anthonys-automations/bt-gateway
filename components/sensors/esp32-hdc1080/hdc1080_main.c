#include <string.h>
#include <stdio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_event.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include "hdc1080.h"

#include "hdc1080_main.h"
#include "azure-iot.h"

#define I2C_MASTER_TX_BUF_DISABLE (0)
#define I2C_MASTER_RX_BUF_DISABLE (0)
#define I2C_READ_TIMEOUT_PERIOD   ((TickType_t)200 / portTICK_PERIOD_MS)

static bool i2c_init(void);

static uint8_t TelemetryBuffer[ 256 ];

/* THIS IS THE CALLBACK FOR THE SENSOR READINGS,
 * THE HDC1080 REQUIRES A SHORT CONVERSION PERIOD
 * WHEN THE READINGS ARE REQUESTED. INSTEAD OF BLOCKING 
 * A TIMER IS STARTED WHEN THE CONVERSION IS FINISHED
 * THE VALUES ARE READ AND THEN RETURNED TO THIS CALLBACK 
 * ON COMPLETE. IF BOTH VALUES ARE 0 THEN AN ERROR MAY HAVE OCCURED */
void temperature_readings_callback(hdc1080_sensor_readings_t sens_readings){
  /* HERE ARE SOME CONVERSION SAMPLES, THE MACROS ARE LOCATED IN hdc1080.h */
  float temp_in_f = CEL2FAH(sens_readings.temperature);
  float dewpoint = DEWPOINT(sens_readings.temperature, sens_readings.humidity);
  float sat_vp = SVP(sens_readings.temperature);
  float vpd_pasc = VPD(sat_vp, sens_readings.humidity);
  ESP_LOGI("HDC1080_SENSOR_DATA", "TEMPERATURE: %.2f°C | %.2f°F", sens_readings.temperature, temp_in_f);
  ESP_LOGI("HDC1080_SENSOR_DATA", "HUMIDITY: %.2f%%  -  DEWPOINT: %.2f°C | %.2f°F", sens_readings.humidity, dewpoint, CEL2FAH(dewpoint));
  ESP_LOGI("HDC1080_SENSOR_DATA", "AIR SATURATION VAPOR PRESSURE: %.2f kPa  -  AIR VAPOR PRESSURE DEFICIT: %.2f kPa", PAS2KPA(sat_vp), vpd_pasc);

  // Set GPIO 4 low
  gpio_set_level(4, 0);
      
  snprintf((char *)TelemetryBuffer, sizeof(TelemetryBuffer), 
         "{\"hdc1080_temperature\":%.2f,\"hdc1080_humidity\":%.2f}", 
         sens_readings.temperature, sens_readings.humidity);

  ESP_LOGI("HDC1080_SENSOR_DATA", "Sensor data prepared: %s", TelemetryBuffer);

  // Queue the telemetry data
  azure_iot_queue_telemetry(TelemetryBuffer, strlen((char *)TelemetryBuffer), NULL);

}

void hdc1080_main_request_readings(void){
  // Set GPIO 4 high
  gpio_set_level(4, 1);

  // FILL IN YOUR HDC SETTINGS
  hdc1080_settings_t hdc_settings = {
    .i2c_address = HDC1080_I2C_ADDRESS,
    .i2c_port_number = CONFIG_HDC1080_I2C_PORT_NUMBER,
    .timeout_length = I2C_READ_TIMEOUT_PERIOD,
    .callback = temperature_readings_callback
  };
  
  // SETUP YOUR HDC REGISTER CONFIGURATION
  hdc1080_config_t hdc_config = {
    .humidity_measurement_resolution = HDC1080_HUMIDITY_RESOLUTION_14BIT,
    .temperature_measurement_resolution = HDC1080_TEMPERATURE_RESOLUTION_14BIT,
    .mode_of_acquisition = HDC1080_ACQUISITION_HUMIDITY_AND_TEMPERATURE,
    .heater = HDC1080_HEATER_DISABLED
  };

  // SETUP AND CONFIGURE THE SENSOR AND ABSTRACTION
  if(hdc1080_configure(&hdc_settings, hdc_config) == ESP_OK){
    ESP_LOGI("HDC1080_MAIN", "HDC1080 CONFIGURATION SUCCESSFUL");
  }

  if(hdc1080_request_readings() == ESP_OK){
    ESP_LOGI("HDC1080_MAIN", "READINGS WERE REQUESTED");
  }
}

void hdc1080_main_init(void){
  // Configure GPIO 4 as output
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << 4),  // GPIO 4
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io_conf);

  // Set GPIO 4 high
  gpio_set_level(4, 1);

  // CONFIGURE YOUR I2C BUS
  i2c_init();
}

/* ----------------------------------------------------------------------
 * @name bool i2c_init(void)
 * ----------------------------------------------------------------------
 * @brief Configure i2c parameters, install i2c driver, 
 * perform device discovery
 * 
 * @return true on success
 */
static bool i2c_init(void){
  unsigned char devAddr = 0;
  unsigned char devCount = 0;
  unsigned char devList[128] = {0};
  /* I2C MASTER MODE, PULLUPS ENABLED */
  i2c_config_t i2c_conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = CONFIG_HDC1080_I2C_SDA,
      .sda_pullup_en = CONFIG_HDC1080_I2C_PULLUPS,
      .scl_io_num = CONFIG_HDC1080_I2C_SCL,
      .scl_pullup_en = CONFIG_HDC1080_I2C_PULLUPS,
      .master.clk_speed = CONFIG_HDC1080_I2C_PORT_FREQUENCY
  };
  /* CONFIGURE THE PORT */
  esp_err_t err = i2c_param_config(CONFIG_HDC1080_I2C_PORT_NUMBER, &i2c_conf);
  if (err != ESP_OK) {
    ESP_LOGE("HDC1080_I2C", "ERROR CONFIGURING I2C PORT %d", err);
    return false;
  }
  /* LOAD THE DRIVER */
  err = i2c_driver_install(CONFIG_HDC1080_I2C_PORT_NUMBER, i2c_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
  if (err != ESP_OK) {
    if(err == ESP_ERR_INVALID_ARG){
      ESP_LOGE("HDC1080_I2C", "ERROR INSTALLING I2C DRIVER, INVALID ARGUMENT");
    }else if(err == ESP_FAIL){
      ESP_LOGE("HDC1080_I2C", "I2C DRIVER INSTALLATION FAILED!");
    }
    return false;
  }
  
#if CONFIG_HDC1080_I2C_SCAN_FOR_DEVICES
  /* DEVICE DISCOVERY */
  for(int ol = 0; ol < 128; ol += 16){
    for(int il =0; il < 16; il++){
      devAddr = ol + il;  // CURRENT ADDRESS IS OUTER LOOP + INNER LOOP
      i2c_cmd_handle_t cmdlnk = i2c_cmd_link_create();
      i2c_master_start(cmdlnk);
      i2c_master_write_byte(cmdlnk, (devAddr << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
      i2c_master_stop(cmdlnk);
      esp_err_t lnkerr = i2c_master_cmd_begin(CONFIG_HDC1080_I2C_PORT_NUMBER, cmdlnk, I2C_READ_TIMEOUT_PERIOD);
      i2c_cmd_link_delete(cmdlnk);
      switch(lnkerr){
        case ESP_OK:
          if(devCount < 128){ //DON'T OVERFLOW
            if(devAddr == 0x00){ break; } //IGNORE ADDRESS 0
            devList[devCount++] = devAddr;  //ADD THE DEVICE TO THE LIST AND INCREMENT THE COUNT
          }
        break;
        case ESP_ERR_INVALID_ARG:
          ESP_LOGE("HDC1080_I2C_discover", "INVALID PARAMETER WAS PASSED TO i2c_master_cmd_begin");
        break;
        case ESP_ERR_NO_MEM:
          ESP_LOGE("HDC1080_I2C_discover", "THE CMD HANDLER BUFFER SIZE IS TOO SMALL");
        break;
        default: break; //TIMED OUT, MOVE ON
      } /** END - switch(lnkerr) */
    } /** END - for(int il =0; il < 16; il++) */
  } /** END - for(int ol = 0; ol < 128; ol += 16) */

  if(devCount == 0){
    ESP_LOGW("HDC1080_I2C", "NO DEVICES FOUND");
    return false;
  }

  /* PRINT DISCOVERED DEVICE ADDRESSES */
  if(devCount > 0){
    for(int x=0; x<devCount; x++){
      ESP_LOGI("HDC1080_I2C", "FOUND DEVICE AT ADDRESS: 0x%02X", devList[x]);
    } /** END - for(int x=0; x<devCount; x++) */
  }
#endif

  return true;
}