#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "sensor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMP280_REG_CALIB_START     0x88
#define BMP280_REG_CHIPID          0xD0
#define BMP280_REG_RESET           0xE0
#define BMP280_REG_STATUS          0xF3
#define BMP280_REG_CTRL_MEAS       0xF4
#define BMP280_REG_CONFIG          0xF5
#define BMP280_REG_PRESS_MSB       0xF7
#define BMP280_REG_TEMP_MSB        0xFA

#define BMP280_CHIP_ID1            0x58
#define BMP280_CHIP_ID2            0x56
#define BMP280_CHIP_ID3            0x57
#define BME280_CHIP_ID             0x60

/**
 * @brief Initialize the BMP280 sensor over I2C and read calibration coefficients.
 *
 * @param i2c_addr I2C device address (0x76 or 0x77).
 * @return ESP_OK on success, or an error code.
 */
esp_err_t bmp280_init(uint8_t i2c_addr);

/**
 * @brief Read temperature and pressure from BMP280 and apply factory compensation.
 *
 * @param i2c_addr I2C device address.
 * @param data Pointer to bmp280_data_t struct receiving results.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t bmp280_read_data(uint8_t i2c_addr, bmp280_data_t *data);

#ifdef __cplusplus
}
#endif

#endif // BMP280_H

