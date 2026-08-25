#ifndef TASK_ADXL345_H
#define TASK_ADXL345_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the FreeRTOS task for reading the ADXL345 accelerometer.
 *
 * @param out_queue FreeRTOS QueueHandle_t to which adxl345_data_t structs will be posted.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t task_adxl345_start(QueueHandle_t out_queue);

#ifdef __cplusplus
}
#endif

#endif // TASK_ADXL345_H

