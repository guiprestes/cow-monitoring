#include "task_adxl345.h"
#include "app_config.h"
#include "adxl345.h"
#include "sensor_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TASK_ADXL345";
static QueueHandle_t s_out_queue = NULL;

static void adxl345_task_entry(void *pvParameters) {
    ESP_LOGI(TAG, "ADXL345 task started");
    adxl345_data_t data;

    while (1) {
        esp_err_t err = adxl345_read_data(ADXL345_I2C_ADDR, &data);
        if (err == ESP_OK) {
            ESP_LOGD(TAG, "Accel: X=%.2f, Y=%.2f, Z=%.2f, Mag=%.2f",
                     data.x, data.y, data.z, data.magnitude);

            if (s_out_queue != NULL) {
                // If queue is full, remove oldest and insert newest to ensure fresh telemetry
                if (xQueueSend(s_out_queue, &data, 0) != pdPASS) {
                    adxl345_data_t dummy;
                    xQueueReceive(s_out_queue, &dummy, 0);
                    xQueueSend(s_out_queue, &data, 0);
                }
            }
        } else {
            ESP_LOGW(TAG, "Failed to read ADXL345: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(ADXL345_SAMPLE_INTERVAL_MS));
    }
}

esp_err_t task_adxl345_start(QueueHandle_t out_queue) {
    s_out_queue = out_queue;

    BaseType_t ret = xTaskCreate(
        adxl345_task_entry,
        "Task_ADXL345",
        TASK_ADXL345_STACK_SIZE,
        NULL,
        TASK_ADXL345_PRIORITY,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ADXL345 task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

