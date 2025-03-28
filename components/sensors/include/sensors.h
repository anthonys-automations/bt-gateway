#ifndef SENSORS_H
#define SENSORS_H

#include "esp_err.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize all sensors
 * 
 * @return ESP_OK on success, otherwise an error code
 */
esp_err_t sensors_init(void);

/**
 * @brief Get the sensors data as a JSON string
 * 
 * @param buffer Output buffer to write the JSON string
 * @param buffer_size Size of the output buffer
 * @return ESP_OK on success, otherwise an error code
 */
esp_err_t sensors_get_json(char *buffer, size_t buffer_size);

/**
 * @brief Deinitialize all sensors
 * 
 * @return ESP_OK on success, otherwise an error code
 */
esp_err_t sensors_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */ 