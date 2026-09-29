#ifndef TASK_BMP280_H
#define TASK_BMP280_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the FreeRTOS task for reading the BMP280 environmental sensor.
 *
 * @param out_queue FreeRTOS QueueHandle_t to which bmp280_data_t structs will be posted.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t task_bmp280_start(QueueHandle_t out_queue);

#ifdef __cplusplus
}
#endif

#endif // TASK_BMP280_H

