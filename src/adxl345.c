#include "adxl345.h"
#include "i2c_manager.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "ADXL345";

esp_err_t adxl345_init(uint8_t i2c_addr) {
    uint8_t dev_id = 0;
    esp_err_t err = i2c_manager_read_reg(i2c_addr, ADXL345_REG_DEVID, &dev_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read DEVID from addr 0x%02X: %s", i2c_addr, esp_err_to_name(err));
        return err;
    }

    if (dev_id != ADXL345_DEVICE_ID) {
        ESP_LOGE(TAG, "Invalid DEVID 0x%02X (expected 0x%02X)", dev_id, ADXL345_DEVICE_ID);
        return ESP_ERR_NOT_FOUND;
    }

    // Set Data Format: +/- 2g with Full Resolution (3.9 mg/LSB)
    err = i2c_manager_write_reg(i2c_addr, ADXL345_REG_DATA_FORMAT, ADXL345_RANGE_2G | ADXL345_FULL_RES);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DATA_FORMAT: %s", esp_err_to_name(err));
        return err;
    }

    // Set Power Control: Enable Measurement mode (bit 3 = 1)
    err = i2c_manager_write_reg(i2c_addr, ADXL345_REG_POWER_CTL, 0x08);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set POWER_CTL: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "ADXL345 successfully initialized at 0x%02X (DEVID: 0x%02X)", i2c_addr, dev_id);
    return ESP_OK;
}

esp_err_t adxl345_read_raw(uint8_t i2c_addr, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z) {
    if (raw_x == NULL || raw_y == NULL || raw_z == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[6];
    esp_err_t err = i2c_manager_read_bytes(i2c_addr, ADXL345_REG_DATAX0, buffer, 6);
    if (err != ESP_OK) {
        return err;
    }

    *raw_x = (int16_t)(buffer[0] | (buffer[1] << 8));
    *raw_y = (int16_t)(buffer[2] | (buffer[3] << 8));
    *raw_z = (int16_t)(buffer[4] | (buffer[5] << 8));

    return ESP_OK;
}

esp_err_t adxl345_read_data(uint8_t i2c_addr, adxl345_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int16_t raw_x = 0, raw_y = 0, raw_z = 0;
    esp_err_t err = adxl345_read_raw(i2c_addr, &raw_x, &raw_y, &raw_z);
    if (err != ESP_OK) {
        data->valid = false;
        return err;
    }

    // Convert raw ADC counts to m/s^2
    data->x = (float)raw_x * ADXL345_SCALE_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    data->y = (float)raw_y * ADXL345_SCALE_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
    data->z = (float)raw_z * ADXL345_SCALE_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

    // Vector magnitude
    data->magnitude = sqrtf((data->x * data->x) + (data->y * data->y) + (data->z * data->z));
    data->valid = true;

    return ESP_OK;
}

