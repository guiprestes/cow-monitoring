#include "i2c_manager.h"
#include "app_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "I2C_MGR";
static SemaphoreHandle_t s_i2c_mutex = NULL;
static bool s_initialized = false;

esp_err_t i2c_manager_init(void) {
    if (s_initialized) {
        return ESP_OK;
    }

    s_i2c_mutex = xSemaphoreCreateMutex();
    if (s_i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return ESP_ERR_NO_MEM;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "I2C master initialized on SDA=%d, SCL=%d, Freq=%d Hz",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    return ESP_OK;
}

esp_err_t i2c_manager_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    uint8_t buf[2] = {reg_addr, data};
    return i2c_manager_write_bytes(dev_addr, reg_addr, &data, 1);
}

esp_err_t i2c_manager_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data) {
    return i2c_manager_read_bytes(dev_addr, reg_addr, data, 1);
}

esp_err_t i2c_manager_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    if (!s_initialized || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Mutex timeout reading from addr 0x%02X", dev_addr);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, dev_addr,
                                                &reg_addr, 1,
                                                data, len,
                                                pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    xSemaphoreGive(s_i2c_mutex);
    return err;
}

esp_err_t i2c_manager_write_bytes(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len) {
    if (!s_initialized || (data == NULL && len > 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t stack_buf[32];
    uint8_t *p_buf = stack_buf;
    size_t total_len = len + 1;

    if (total_len > sizeof(stack_buf)) {
        p_buf = (uint8_t *)malloc(total_len);
        if (p_buf == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    p_buf[0] = reg_addr;
    if (len > 0 && data != NULL) {
        memcpy(&p_buf[1], data, len);
    }

    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Mutex timeout writing to addr 0x%02X", dev_addr);
        if (p_buf != stack_buf) free(p_buf);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, dev_addr,
                                              p_buf, total_len,
                                              pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    xSemaphoreGive(s_i2c_mutex);

    if (p_buf != stack_buf) {
        free(p_buf);
    }
    return err;
}

void i2c_manager_scan(void) {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    int count = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)) == pdTRUE) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
            i2c_cmd_link_delete(cmd);
            xSemaphoreGive(s_i2c_mutex);

            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Found device at 0x%02X", addr);
                count++;
            }
        }
    }

    if (count == 0) {
        ESP_LOGW(TAG, "No I2C devices found");
    } else {
        ESP_LOGI(TAG, "I2C scan complete. Total devices: %d", count);
    }
}

