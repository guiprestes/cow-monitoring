#ifndef ADXL345_H
#define ADXL345_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "sensor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADXL345_REG_DEVID          0x00
#define ADXL345_REG_BW_RATE        0x2C
#define ADXL345_REG_POWER_CTL      0x2D
#define ADXL345_REG_DATA_FORMAT    0x31
#define ADXL345_REG_DATAX0         0x32

#define ADXL345_DEVICE_ID          0xE5

// Range options
#define ADXL345_RANGE_2G           0x00
#define ADXL345_RANGE_4G           0x01
#define ADXL345_RANGE_8G           0x02
#define ADXL345_RANGE_16G          0x03
#define ADXL345_FULL_RES           0x08

// Conversion constant: standard gravity (m/s^2) and scale (3.9 mg/LSB)
#define ADXL345_SCALE_MULTIPLIER   0.0039f
#define SENSORS_GRAVITY_STANDARD   9.80665f

/**
 * @brief Initialize the ADXL345 accelerometer over I2C.
 *
 * @param i2c_addr I2C device address (default is 0x53).
 * @return ESP_OK on success, or an error code.
 */
esp_err_t adxl345_init(uint8_t i2c_addr);

/**
 * @brief Read raw 16-bit acceleration values for X, Y, Z axes.
 *
 * @param i2c_addr I2C device address.
 * @param raw_x Output pointer for raw X.
 * @param raw_y Output pointer for raw Y.
 * @param raw_z Output pointer for raw Z.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t adxl345_read_raw(uint8_t i2c_addr, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z);

/**
 * @brief Read and compute calibrated acceleration in m/s^2 and vector magnitude.
 *
 * @param i2c_addr I2C device address.
 * @param data Pointer to adxl345_data_t struct receiving results.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t adxl345_read_data(uint8_t i2c_addr, adxl345_data_t *data);

#ifdef __cplusplus
}
#endif

#endif // ADXL345_H

