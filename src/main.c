#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "app_config.h"
#include "sensor_types.h"
#include "i2c_manager.h"
#include "adxl345.h"
#include "bmp280.h"
#include "sx127x.h"
#include "task_adxl345.h"
#include "task_bmp280.h"
#include "task_lora.h"

static const char *TAG = "MAIN";

// FreeRTOS Queues
static QueueHandle_t s_queue_accel = NULL;
static QueueHandle_t s_queue_bmp = NULL;

static void init_status_led(void) {
    gpio_reset_pin(STATUS_LED_PIN);
    gpio_set_direction(STATUS_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(STATUS_LED_PIN, 0);
    ESP_LOGI(TAG, "Status LED initialized on GPIO %d", STATUS_LED_PIN);
}

static void init_lora_radio(void) {
    sx127x_config_t lora_cfg = {
        .pin_sck = LORA_SCK_PIN,
        .pin_miso = LORA_MISO_PIN,
        .pin_mosi = LORA_MOSI_PIN,
        .pin_cs = LORA_CS_PIN,
        .pin_rst = LORA_RST_PIN,
        .pin_dio0 = LORA_DIO0_PIN,
        .frequency_hz = LORA_FREQUENCY_HZ,
        .tx_power = LORA_TX_POWER_DBM,
        .spreading_factor = LORA_SPREADING_FACTOR,
        .bandwidth_hz = LORA_BANDWIDTH_KHZ * 1000L,
        .coding_rate = LORA_CODING_RATE,
        .sync_word = LORA_SYNC_WORD,
    };

    esp_err_t err = sx127x_init(&lora_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LoRa radio initialization failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "LoRa radio initialized successfully (915 MHz)");
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "   Cow Monitoring System (ESP-IDF / FreeRTOS)     ");
    ESP_LOGI(TAG, "==================================================");

    // 1. Initialize GPIOs
    init_status_led();

    // 2. Initialize Shared I2C Bus
    esp_err_t err = i2c_manager_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fatal: I2C initialization failed: %s", esp_err_to_name(err));
    }

    // Optional: Scan I2C bus for debugging
    i2c_manager_scan();

    // 3. Initialize Sensors
    err = adxl345_init(ADXL345_I2C_ADDR);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Warning: ADXL345 init failed: %s", esp_err_to_name(err));
    }

    err = bmp280_init(BMP280_I2C_ADDR);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Primary BMP280 init failed, trying alternate addr 0x%02X...", BMP280_I2C_ADDR_ALT);
        err = bmp280_init(BMP280_I2C_ADDR_ALT);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Warning: BMP280 init failed: %s", esp_err_to_name(err));
        }
    }

    // 4. Initialize LoRa Radio
    init_lora_radio();

    // 5. Create FreeRTOS Queues
    s_queue_accel = xQueueCreate(TELEMETRY_QUEUE_LEN, sizeof(adxl345_data_t));
    s_queue_bmp = xQueueCreate(TELEMETRY_QUEUE_LEN, sizeof(bmp280_data_t));

    if (s_queue_accel == NULL || s_queue_bmp == NULL) {
        ESP_LOGE(TAG, "Fatal: Failed to create FreeRTOS telemetry queues");
        return;
    }

    // 6. Start FreeRTOS Tasks
    task_adxl345_start(s_queue_accel);
    task_bmp280_start(s_queue_bmp);
    task_lora_start(s_queue_accel, s_queue_bmp);

    ESP_LOGI(TAG, "All FreeRTOS tasks launched successfully.");
}

