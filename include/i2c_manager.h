#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the shared I2C master interface and synchronization mutex.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t i2c_manager_init(void);

/**
 * @brief Thread-safe write of a single byte to an I2C device register.
 *
 * @param dev_addr 7-bit I2C slave address.
 * @param reg_addr Register address to write to.
 * @param data Data byte to write.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t i2c_manager_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Thread-safe read of a single byte from an I2C device register.
 *
 * @param dev_addr 7-bit I2C slave address.
 * @param reg_addr Register address to read from.
 * @param data Pointer to buffer receiving the read byte.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t i2c_manager_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Thread-safe burst read of multiple bytes from an I2C device starting at a register.
 *
 * @param dev_addr 7-bit I2C slave address.
 * @param reg_addr Starting register address.
 * @param data Pointer to buffer receiving the read bytes.
 * @param len Number of bytes to read.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t i2c_manager_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Thread-safe burst write of multiple bytes to an I2C device starting at a register.
 *
 * @param dev_addr 7-bit I2C slave address.
 * @param reg_addr Starting register address.
 * @param data Pointer to buffer containing the bytes to write.
 * @param len Number of bytes to write.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t i2c_manager_write_bytes(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len);

/**
 * @brief Utility function to scan the I2C bus and log found devices.
 */
void i2c_manager_scan(void);

#ifdef __cplusplus
}
#endif

#endif // I2C_MANAGER_H

