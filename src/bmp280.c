#include "bmp280.h"
#include "i2c_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BMP280";

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_data_t;

static bmp280_calib_data_t s_calib;
static bool s_calib_loaded = false;

static esp_err_t bmp280_read_calibration(uint8_t i2c_addr) {
    uint8_t buffer[24];
    esp_err_t err = i2c_manager_read_bytes(i2c_addr, BMP280_REG_CALIB_START, buffer, 24);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data: %s", esp_err_to_name(err));
        return err;
    }

    s_calib.dig_T1 = (uint16_t)(buffer[0] | (buffer[1] << 8));
    s_calib.dig_T2 = (int16_t)(buffer[2] | (buffer[3] << 8));
    s_calib.dig_T3 = (int16_t)(buffer[4] | (buffer[5] << 8));

    s_calib.dig_P1 = (uint16_t)(buffer[6] | (buffer[7] << 8));
    s_calib.dig_P2 = (int16_t)(buffer[8] | (buffer[9] << 8));
    s_calib.dig_P3 = (int16_t)(buffer[10] | (buffer[11] << 8));
    s_calib.dig_P4 = (int16_t)(buffer[12] | (buffer[13] << 8));
    s_calib.dig_P5 = (int16_t)(buffer[14] | (buffer[15] << 8));
    s_calib.dig_P6 = (int16_t)(buffer[16] | (buffer[17] << 8));
    s_calib.dig_P7 = (int16_t)(buffer[18] | (buffer[19] << 8));
    s_calib.dig_P8 = (int16_t)(buffer[20] | (buffer[21] << 8));
    s_calib.dig_P9 = (int16_t)(buffer[22] | (buffer[23] << 8));

    s_calib_loaded = true;
    return ESP_OK;
}

esp_err_t bmp280_init(uint8_t i2c_addr) {
    uint8_t chip_id = 0;
    esp_err_t err = i2c_manager_read_reg(i2c_addr, BMP280_REG_CHIPID, &chip_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CHIP_ID from 0x%02X: %s", i2c_addr, esp_err_to_name(err));
        return err;
    }

    if (chip_id != BMP280_CHIP_ID1 && chip_id != BMP280_CHIP_ID2 &&
        chip_id != BMP280_CHIP_ID3 && chip_id != BME280_CHIP_ID) {
        ESP_LOGE(TAG, "Unknown CHIP_ID 0x%02X at 0x%02X", chip_id, i2c_addr);
        return ESP_ERR_NOT_FOUND;
    }

    // Soft reset
    i2c_manager_write_reg(i2c_addr, BMP280_REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read calibration parameters
    err = bmp280_read_calibration(i2c_addr);
    if (err != ESP_OK) {
        return err;
    }

    // Configure filter and standby time in CONFIG (standby 0.5ms, filter 16)
    i2c_manager_write_reg(i2c_addr, BMP280_REG_CONFIG, (0x00 << 5) | (0x04 << 2));

    // Configure CTRL_MEAS: Temperature oversampling x2, Pressure oversampling x16, Normal mode (0x03)
    // osrs_t = 010 (x2), osrs_p = 101 (x16), mode = 11 (normal) -> 0x57
    err = i2c_manager_write_reg(i2c_addr, BMP280_REG_CTRL_MEAS, 0x57);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CTRL_MEAS: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "BMP280 successfully initialized at 0x%02X (CHIP_ID: 0x%02X)", i2c_addr, chip_id);
    return ESP_OK;
}

esp_err_t bmp280_read_data(uint8_t i2c_addr, bmp280_data_t *data) {
    if (data == NULL || !s_calib_loaded) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buf[6];
    esp_err_t err = i2c_manager_read_bytes(i2c_addr, BMP280_REG_PRESS_MSB, buf, 6);
    if (err != ESP_OK) {
        data->valid = false;
        return err;
    }

    int32_t adc_P = (int32_t)((((uint32_t)buf[0]) << 12) | (((uint32_t)buf[1]) << 4) | (((uint32_t)buf[2]) >> 4));
    int32_t adc_T = (int32_t)((((uint32_t)buf[3]) << 12) | (((uint32_t)buf[4]) << 4) | (((uint32_t)buf[5]) >> 4));

    // Bosch standard compensation algorithm
    // Temperature compensation
    double var1 = (((double)adc_T) / 16384.0 - ((double)s_calib.dig_T1) / 1024.0) * ((double)s_calib.dig_T2);
    double var2 = ((((double)adc_T) / 131072.0 - ((double)s_calib.dig_T1) / 8192.0) *
                   (((double)adc_T) / 131072.0 - ((double)s_calib.dig_T1) / 8192.0)) * ((double)s_calib.dig_T3);
    int32_t t_fine = (int32_t)(var1 + var2);
    data->temperature = (float)((var1 + var2) / 5120.0);

    // Pressure compensation
    var1 = (((double)t_fine) / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)s_calib.dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)s_calib.dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)s_calib.dig_P4) * 65536.0);
    var1 = (((double)s_calib.dig_P3) * var1 * var1 / 524288.0 + ((double)s_calib.dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)s_calib.dig_P1);

    if (var1 != 0.0) {
        double p = 1048576.0 - (double)adc_P;
        p = (p - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)s_calib.dig_P9) * p * p / 2147483648.0;
        var2 = p * ((double)s_calib.dig_P8) / 32768.0;
        p = p + (var1 + var2 + ((double)s_calib.dig_P7)) / 16.0;
        data->pressure = (float)(p / 100.0); // Convert Pa to hPa
    } else {
        data->pressure = 0.0f;
    }

    data->valid = true;
    return ESP_OK;
}

