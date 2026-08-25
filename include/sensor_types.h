#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Structure containing 3-axis accelerometer data and calculated magnitude.
 */
typedef struct {
    float x;            /*!< X-axis acceleration in m/s^2 (or g) */
    float y;            /*!< Y-axis acceleration in m/s^2 (or g) */
    float z;            /*!< Z-axis acceleration in m/s^2 (or g) */
    float magnitude;    /*!< Vector magnitude sqrt(x^2 + y^2 + z^2) */
    bool valid;         /*!< True if reading succeeded */
} adxl345_data_t;

/**
 * @brief Structure containing environmental data from BMP280.
 */
typedef struct {
    float temperature;  /*!< Temperature in degrees Celsius */
    float pressure;     /*!< Pressure in hPa */
    bool valid;         /*!< True if reading succeeded */
} bmp280_data_t;

/**
 * @brief Consolidated telemetry packet structure for LoRa transmission.
 */
typedef struct {
    uint32_t node_id;       /*!< Identifier of this monitoring node */
    uint32_t timestamp_ms;  /*!< Uptime timestamp in milliseconds */
    adxl345_data_t accel;   /*!< Accelerometer readings */
    bmp280_data_t env;      /*!< Environmental readings */
} telemetry_packet_t;

#ifdef __cplusplus
}
#endif

#endif // SENSOR_TYPES_H

