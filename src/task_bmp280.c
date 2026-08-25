#include "task_bmp280.h"
#include "app_config.h"
#include "bmp280.h"
#include "sensor_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TASK_BMP280";
static QueueHandle_t s_out_queue = NULL;

static void bmp280_task_entry(void *pvParameters) {
    ESP_LOGI(TAG, "BMP280 task started");
    bmp280_data_t data;

    while (1) {
        esp_err_t err = bmp280_read_data(BMP280_I2C_ADDR, &data);
        if (err != ESP_OK) {
            // Try alternate address if primary failed
            err = bmp280_read_data(BMP280_I2C_ADDR_ALT, &data);
        }

        if (err == ESP_OK) {
            ESP_LOGD(TAG, "BMP280: Temp=%.2f C, Press=%.2f hPa", data.temperature, data.pressure);

            if (s_out_queue != NULL) {
                // If queue is full, remove oldest and insert newest
                if (xQueueSend(s_out_queue, &data, 0) != pdPASS) {
                    bmp280_data_t dummy;
                    xQueueReceive(s_out_queue, &dummy, 0);
                    xQueueSend(s_out_queue, &data, 0);
                }
            }
        } else {
            ESP_LOGW(TAG, "Failed to read BMP280: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(BMP280_SAMPLE_INTERVAL_MS));
    }
}

esp_err_t task_bmp280_start(QueueHandle_t out_queue) {
    s_out_queue = out_queue;

    BaseType_t ret = xTaskCreate(
        bmp280_task_entry,
        "Task_BMP280",
        TASK_BMP280_STACK_SIZE,
        NULL,
        TASK_BMP280_PRIORITY,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create BMP280 task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

