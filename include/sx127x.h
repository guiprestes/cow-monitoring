#ifndef SX127X_H
#define SX127X_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SX1276/77/78 Registers */
#define SX127X_REG_FIFO                 0x00
#define SX127X_REG_OP_MODE              0x01
#define SX127X_REG_FRF_MSB              0x06
#define SX127X_REG_FRF_MID              0x07
#define SX127X_REG_FRF_LSB              0x08
#define SX127X_REG_PA_CONFIG            0x09
#define SX127X_REG_PA_RAMP              0x0A
#define SX127X_REG_OCP                  0x0B
#define SX127X_REG_LNA                  0x0C
#define SX127X_REG_FIFO_ADDR_PTR        0x0D
#define SX127X_REG_FIFO_TX_BASE_ADDR    0x0E
#define SX127X_REG_FIFO_RX_BASE_ADDR    0x0F
#define SX127X_REG_FIFO_RX_CURRENT_ADDR 0x10
#define SX127X_REG_IRQ_FLAGS            0x12
#define SX127X_REG_RX_NB_BYTES          0x13
#define SX127X_REG_PKT_SNR_VALUE        0x19
#define SX127X_REG_PKT_RSSI_VALUE       0x1A
#define SX127X_REG_MODEM_CONFIG_1       0x1D
#define SX127X_REG_MODEM_CONFIG_2       0x1E
#define SX127X_REG_SYMB_TIMEOUT_LSB     0x1F
#define SX127X_REG_PREAMBLE_MSB         0x20
#define SX127X_REG_PREAMBLE_LSB         0x21
#define SX127X_REG_PAYLOAD_LENGTH       0x22
#define SX127X_REG_MODEM_CONFIG_3       0x26
#define SX127X_REG_SYNC_WORD            0x39
#define SX127X_REG_DIO_MAPPING_1        0x40
#define SX127X_REG_VERSION              0x42
#define SX127X_REG_PA_DAC               0x4D

/* Operation Modes */
#define SX127X_MODE_LONG_RANGE_MODE     0x80
#define SX127X_MODE_SLEEP               0x00
#define SX127X_MODE_STDBY               0x01
#define SX127X_MODE_TX                  0x03
#define SX127X_MODE_RX_CONTINUOUS       0x05
#define SX127X_MODE_RX_SINGLE           0x06

/* IRQ Masks */
#define SX127X_IRQ_TX_DONE_MASK         0x08
#define SX127X_IRQ_PAYLOAD_CRC_ERROR    0x20
#define SX127X_IRQ_RX_DONE_MASK         0x40

/**
 * @brief Configuration structure for the SX127x LoRa transceiver.
 */
typedef struct {
    int pin_sck;
    int pin_miso;
    int pin_mosi;
    int pin_cs;
    int pin_rst;
    int pin_dio0;
    long frequency_hz;
    int tx_power;
    int spreading_factor;
    long bandwidth_hz;
    int coding_rate;
    uint8_t sync_word;
} sx127x_config_t;

/**
 * @brief Initialize the SPI bus and SX127x transceiver.
 *
 * @param config Pointer to transceiver configuration structure.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t sx127x_init(const sx127x_config_t *config);

/**
 * @brief Set LoRa carrier frequency in Hz.
 *
 * @param frequency Frequency in Hz (e.g. 915000000 for 915 MHz).
 * @return ESP_OK on success, or an error code.
 */
esp_err_t sx127x_set_frequency(long frequency);

/**
 * @brief Set transmission power in dBm (2 to 17, or 20 for PA_BOOST).
 *
 * @param level Power in dBm.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t sx127x_set_tx_power(int level);

/**
 * @brief Set spreading factor (6 to 12).
 *
 * @param sf Spreading factor.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t sx127x_set_spreading_factor(int sf);

/**
 * @brief Set signal bandwidth in Hz.
 *
 * @param bw Bandwidth (e.g. 125E3, 250E3, 500E3).
 * @return ESP_OK on success, or an error code.
 */
esp_err_t sx127x_set_bandwidth(long bw);

/**
 * @brief Set coding rate denominator (5 to 8 for 4/5 to 4/8).
 *
 * @param denominator Denominator (5=4/5, 6=4/6, etc.).
 * @return ESP_OK on success, or an error code.
 */
esp_err_t sx127x_set_coding_rate(int denominator);

/**
 * @brief Set LoRa sync word.
 *
 * @param sw Sync word (e.g. 0x12 for private networks, 0x34 for LoRaWAN).
 * @return ESP_OK on success, or an error code.
 */
esp_err_t sx127x_set_sync_word(uint8_t sw);

/**
 * @brief Transmit a raw packet over LoRa (blocking until TX_DONE or timeout).
 *
 * @param buffer Pointer to data buffer.
 * @param size Length of data buffer (max 255 bytes).
 * @return ESP_OK on success, or an error code.
 */
esp_err_t sx127x_send_packet(const uint8_t *buffer, size_t size);

#ifdef __cplusplus
}
#endif

#endif // SX127X_H

