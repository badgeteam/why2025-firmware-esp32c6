
// SPDX-License-Identifier: MIT

#include "bsp.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "ledstrip.h"
#include "esp_log.h"

static const char TAG[] = "BSP";

static ledstrip_t ledstrip = {
    .pin = 10,
};

static uint8_t i2c_rx_buffer[256];
static uint8_t i2c_tx_buffer[256];

// Platform-specific BSP init code.
esp_err_t bsp_platform_init() {
    esp_err_t res;
    // Enable GPIO interrupts.
    gpio_install_isr_service(0);

    // Enable internal I²C bus.
    i2c_config_t i2c_conf = {
        .mode       = I2C_MODE_SLAVE,
        .scl_io_num = 7,
        .sda_io_num = 6,
        .slave = {
            .addr_10bit_en = false,
            .slave_addr = 0xc6,
        }
    };
    i2c_param_config(0, &i2c_conf);
    i2c_driver_install(0, i2c_conf.mode, sizeof(i2c_rx_buffer), sizeof(i2c_tx_buffer), 0);

    res = ledstrip_init(&ledstrip);
    if (res != ESP_OK) {
        return res;
    }

    // Turn on LED power
    gpio_set_direction(15, GPIO_MODE_OUTPUT);
    gpio_set_level(15, true);

    ESP_LOGI(TAG, "BSP initialized");

    return res;
}

esp_err_t bsp_ledstrip_send(uint8_t* data, int length) {
    return ledstrip_send(&ledstrip, data, length);
}
