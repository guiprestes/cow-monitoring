#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "driver/gpio.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Hardware & Pinout Configuration (Heltec WiFi LoRa 32 V2)
 * ========================================================================= */

// I2C Configuration
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           GPIO_NUM_21
#define I2C_MASTER_SCL_IO           GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// Sensor I2C Addresses
#define ADXL345_I2C_ADDR            0x53
#define BMP280_I2C_ADDR             0x76
#define BMP280_I2C_ADDR_ALT         0x77

// LoRa SPI & Control Pins (Heltec V2)
#define LORA_SCK_PIN                GPIO_NUM_5
#define LORA_MISO_PIN               GPIO_NUM_19
#define LORA_MOSI_PIN               GPIO_NUM_27
#define LORA_CS_PIN                 GPIO_NUM_18
#define LORA_RST_PIN                GPIO_NUM_14
#define LORA_DIO0_PIN               GPIO_NUM_26

// LoRa Radio Parameters
#define LORA_FREQUENCY_HZ           915000000L  // 915 MHz
#define LORA_TX_POWER_DBM           17
#define LORA_SPREADING_FACTOR       7
#define LORA_BANDWIDTH_KHZ          125
#define LORA_CODING_RATE            5           // 4/5
#define LORA_SYNC_WORD              0x12        // Default private sync word

// Status LED (Heltec V2 onboard LED)
#define STATUS_LED_PIN              GPIO_NUM_23
#define LED_PULSE_DURATION_MS       100

/* =========================================================================
 * Application & Telemetry Configuration
 * ========================================================================= */
#define TELEMETRY_NODE_ID           1
#define TELEMETRY_QUEUE_LEN         10

// Sampling & Transmission Intervals (in milliseconds)
#define ADXL345_SAMPLE_INTERVAL_MS  1000
#define BMP280_SAMPLE_INTERVAL_MS   1000
#define LORA_SEND_INTERVAL_MS       10000

/* =========================================================================
 * FreeRTOS Task Configuration
 * ========================================================================= */
#define TASK_ADXL345_STACK_SIZE     4096
#define TASK_ADXL345_PRIORITY       2

#define TASK_BMP280_STACK_SIZE      4096
#define TASK_BMP280_PRIORITY        1

#define TASK_LORA_STACK_SIZE        4096
#define TASK_LORA_PRIORITY          3

#ifdef __cplusplus
}
#endif

#endif // APP_CONFIG_H

