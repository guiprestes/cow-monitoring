#ifndef TASK_LORA_H
#define TASK_LORA_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the FreeRTOS task for telemetry aggregation, JSON serialization and LoRa transmission.
 *
 * @param in_accel_queue Queue containing adxl345_data_t samples.
 * @param in_bmp_queue Queue containing bmp280_data_t samples.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t task_lora_start(QueueHandle_t in_accel_queue, QueueHandle_t in_bmp_queue);

#ifdef __cplusplus
}
#endif

#endif // TASK_LORA_H

