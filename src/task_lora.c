#include "task_lora.h"
#include "app_config.h"
#include "sensor_types.h"
#include "sx127x.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "TASK_LORA";
static QueueHandle_t s_in_accel_queue = NULL;
static QueueHandle_t s_in_bmp_queue = NULL;

static void lora_task_entry(void *pvParameters) {
    ESP_LOGI(TAG, "LoRa telemetry task started");

    adxl345_data_t latest_accel = {0};
    bmp280_data_t latest_bmp = {0};
    char payload_buffer[256];

    while (1) {
        // Collect latest accelerometer data if available
        if (s_in_accel_queue != NULL) {
            adxl345_data_t accel_sample;
            while (xQueueReceive(s_in_accel_queue, &accel_sample, 0) == pdPASS) {
                latest_accel = accel_sample;
            }
        }

        // Collect latest BMP280 data if available
        if (s_in_bmp_queue != NULL) {
            bmp280_data_t bmp_sample;
            while (xQueueReceive(s_in_bmp_queue, &bmp_sample, 0) == pdPASS) {
                latest_bmp = bmp_sample;
            }
        }

        uint32_t uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // Format clean, standardized JSON string
        int payload_len = snprintf(
            payload_buffer,
            sizeof(payload_buffer),
            "{\"nodeID\":%d,\"time\":%lu,\"temperature\":%.2f,\"pressure\":%.2f,\"accel\":%.2f,\"axisX\":%.2f,\"axisY\":%.2f,\"axisZ\":%.2f}",
            TELEMETRY_NODE_ID,
            (unsigned long)uptime_ms,
            latest_bmp.temperature,
            latest_bmp.pressure,
            latest_accel.magnitude,
            latest_accel.x,
            latest_accel.y,
            latest_accel.z
        );

        if (payload_len > 0 && payload_len < sizeof(payload_buffer)) {
            ESP_LOGI(TAG, "Transmitting LoRa packet (%d bytes): %s", payload_len, payload_buffer);

            esp_err_t err = sx127x_send_packet((const uint8_t *)payload_buffer, (size_t)payload_len);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Packet transmitted successfully");

                // Pulse status LED
                gpio_set_level(STATUS_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(LED_PULSE_DURATION_MS));
                gpio_set_level(STATUS_LED_PIN, 0);
            } else {
                ESP_LOGE(TAG, "LoRa transmission failed: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGE(TAG, "Failed to format payload string");
        }

        vTaskDelay(pdMS_TO_TICKS(LORA_SEND_INTERVAL_MS));
    }
}

esp_err_t task_lora_start(QueueHandle_t in_accel_queue, QueueHandle_t in_bmp_queue) {
    s_in_accel_queue = in_accel_queue;
    s_in_bmp_queue = in_bmp_queue;

    BaseType_t ret = xTaskCreate(
        lora_task_entry,
        "Task_LoRa",
        TASK_LORA_STACK_SIZE,
        NULL,
        TASK_LORA_PRIORITY,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LoRa task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

