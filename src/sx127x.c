#include "sx127x.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SX127X";

static spi_device_handle_t s_spi_dev = NULL;
static sx127x_config_t s_cfg;
static bool s_initialized = false;

static uint8_t sx127x_read_reg(uint8_t reg) {
    uint8_t tx_data[2] = { reg & 0x7F, 0x00 };
    uint8_t rx_data[2] = { 0 };

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    spi_device_polling_transmit(s_spi_dev, &t);
    return rx_data[1];
}

static void sx127x_write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx_data[2] = { reg | 0x80, val };

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = tx_data;

    spi_device_polling_transmit(s_spi_dev, &t);
}

static void sx127x_write_burst(uint8_t reg, const uint8_t *buffer, size_t size) {
    if (size == 0) return;

    uint8_t stack_buf[256 + 1];
    stack_buf[0] = reg | 0x80;
    memcpy(&stack_buf[1], buffer, size);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = (size + 1) * 8;
    t.tx_buffer = stack_buf;

    spi_device_polling_transmit(s_spi_dev, &t);
}

static void sx127x_set_op_mode(uint8_t mode) {
    sx127x_write_reg(SX127X_REG_OP_MODE, SX127X_MODE_LONG_RANGE_MODE | mode);
}

esp_err_t sx127x_set_frequency(long frequency) {
    s_cfg.frequency_hz = frequency;
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    sx127x_write_reg(SX127X_REG_FRF_MSB, (uint8_t)(frf >> 16));
    sx127x_write_reg(SX127X_REG_FRF_MID, (uint8_t)(frf >> 8));
    sx127x_write_reg(SX127X_REG_FRF_LSB, (uint8_t)(frf >> 0));
    return ESP_OK;
}

esp_err_t sx127x_set_tx_power(int level) {
    if (level < 2) level = 2;
    if (level > 20) level = 20;

    if (level > 17) {
        // High power PA_BOOST with DAC enable
        sx127x_write_reg(SX127X_REG_PA_DAC, 0x87);
        sx127x_write_reg(SX127X_REG_PA_CONFIG, 0x80 | (level - 5));
    } else {
        // Normal PA_BOOST
        sx127x_write_reg(SX127X_REG_PA_DAC, 0x84);
        sx127x_write_reg(SX127X_REG_PA_CONFIG, 0x80 | (level - 2));
    }
    return ESP_OK;
}

esp_err_t sx127x_set_spreading_factor(int sf) {
    if (sf < 6) sf = 6;
    if (sf > 12) sf = 12;

    if (sf == 6) {
        sx127x_write_reg(SX127X_REG_MODEM_CONFIG_1, 0x72);
        sx127x_write_reg(SX127X_REG_MODEM_CONFIG_2, (sf << 4) | 0x04);
    } else {
        uint8_t mc2 = sx127x_read_reg(SX127X_REG_MODEM_CONFIG_2);
        sx127x_write_reg(SX127X_REG_MODEM_CONFIG_2, (mc2 & 0x0F) | (sf << 4));
    }
    return ESP_OK;
}

esp_err_t sx127x_set_bandwidth(long bw) {
    uint8_t bw_val = 7; // default 125 kHz
    if (bw <= 7.8E3) bw_val = 0;
    else if (bw <= 10.4E3) bw_val = 1;
    else if (bw <= 15.6E3) bw_val = 2;
    else if (bw <= 20.8E3) bw_val = 3;
    else if (bw <= 31.25E3) bw_val = 4;
    else if (bw <= 41.7E3) bw_val = 5;
    else if (bw <= 62.5E3) bw_val = 6;
    else if (bw <= 125E3)  bw_val = 7;
    else if (bw <= 250E3)  bw_val = 8;
    else bw_val = 9;

    uint8_t mc1 = sx127x_read_reg(SX127X_REG_MODEM_CONFIG_1);
    sx127x_write_reg(SX127X_REG_MODEM_CONFIG_1, (mc1 & 0x0F) | (bw_val << 4));
    return ESP_OK;
}

esp_err_t sx127x_set_coding_rate(int denominator) {
    if (denominator < 5) denominator = 5;
    if (denominator > 8) denominator = 8;
    int cr = denominator - 4;

    uint8_t mc1 = sx127x_read_reg(SX127X_REG_MODEM_CONFIG_1);
    sx127x_write_reg(SX127X_REG_MODEM_CONFIG_1, (mc1 & 0xF1) | (cr << 1));
    return ESP_OK;
}

esp_err_t sx127x_set_sync_word(uint8_t sw) {
    sx127x_write_reg(SX127X_REG_SYNC_WORD, sw);
    return ESP_OK;
}

esp_err_t sx127x_init(const sx127x_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *config;

    // Reset pin configuration
    if (s_cfg.pin_rst >= 0) {
        gpio_reset_pin((gpio_num_t)s_cfg.pin_rst);
        gpio_set_direction((gpio_num_t)s_cfg.pin_rst, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)s_cfg.pin_rst, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)s_cfg.pin_rst, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // SPI Bus configuration
    spi_bus_config_t buscfg = {
        .miso_io_num = s_cfg.pin_miso,
        .mosi_io_num = s_cfg.pin_mosi,
        .sclk_io_num = s_cfg.pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 512,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // SPI Device configuration
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8000000, // 8 MHz
        .mode = 0,
        .spics_io_num = s_cfg.pin_cs,
        .queue_size = 7,
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Check transceiver silicon version
    uint8_t version = sx127x_read_reg(SX127X_REG_VERSION);
    if (version != 0x12) {
        ESP_LOGE(TAG, "Invalid SX127x silicon version 0x%02X (expected 0x12)", version);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Found SX127x transceiver (version: 0x%02X)", version);

    // Sleep mode to enable LoRa mode
    sx127x_set_op_mode(SX127X_MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));
    sx127x_set_op_mode(SX127X_MODE_STDBY);

    // Configure frequency, power, SF, BW, CR, Sync Word
    sx127x_set_frequency(s_cfg.frequency_hz > 0 ? s_cfg.frequency_hz : 915000000L);
    sx127x_set_tx_power(s_cfg.tx_power > 0 ? s_cfg.tx_power : 17);
    sx127x_set_spreading_factor(s_cfg.spreading_factor > 0 ? s_cfg.spreading_factor : 7);
    sx127x_set_bandwidth(s_cfg.bandwidth_hz > 0 ? s_cfg.bandwidth_hz : 125000L);
    sx127x_set_coding_rate(s_cfg.coding_rate > 0 ? s_cfg.coding_rate : 5);
    sx127x_set_sync_word(s_cfg.sync_word != 0 ? s_cfg.sync_word : 0x12);

    // Set base addresses
    sx127x_write_reg(SX127X_REG_FIFO_TX_BASE_ADDR, 0x00);
    sx127x_write_reg(SX127X_REG_FIFO_RX_BASE_ADDR, 0x00);

    // Enable LNA Boost
    uint8_t lna = sx127x_read_reg(SX127X_REG_LNA);
    sx127x_write_reg(SX127X_REG_LNA, lna | 0x03);

    // Enable Auto AGC
    sx127x_write_reg(SX127X_REG_MODEM_CONFIG_3, 0x04);

    sx127x_set_op_mode(SX127X_MODE_STDBY);
    s_initialized = true;
    ESP_LOGI(TAG, "SX127x initialized at %ld Hz, TX power %d dBm", s_cfg.frequency_hz, s_cfg.tx_power);
    return ESP_OK;
}

esp_err_t sx127x_send_packet(const uint8_t *buffer, size_t size) {
    if (!s_initialized || buffer == NULL || size == 0 || size > 255) {
        return ESP_ERR_INVALID_ARG;
    }

    // Set to Standby mode
    sx127x_set_op_mode(SX127X_MODE_STDBY);

    // Set FIFO pointer to TX base address
    sx127x_write_reg(SX127X_REG_FIFO_ADDR_PTR, 0x00);
    sx127x_write_burst(SX127X_REG_FIFO, buffer, size);
    sx127x_write_reg(SX127X_REG_PAYLOAD_LENGTH, (uint8_t)size);

    // Start transmission
    sx127x_set_op_mode(SX127X_MODE_TX);

    // Wait for TX_DONE with timeout (~2 seconds)
    int timeout = 200; // 200 * 10ms = 2000ms
    while ((sx127x_read_reg(SX127X_REG_IRQ_FLAGS) & SX127X_IRQ_TX_DONE_MASK) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout--;
        if (timeout <= 0) {
            ESP_LOGE(TAG, "Transmission timed out!");
            sx127x_set_op_mode(SX127X_MODE_STDBY);
            return ESP_ERR_TIMEOUT;
        }
    }

    // Clear IRQ flags
    sx127x_write_reg(SX127X_REG_IRQ_FLAGS, SX127X_IRQ_TX_DONE_MASK);
    sx127x_set_op_mode(SX127X_MODE_STDBY);
    return ESP_OK;
}

