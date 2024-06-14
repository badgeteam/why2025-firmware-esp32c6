
// SPDX-License-Identifier: MIT

#include "bsp.h"
#include "driver/i2c.h"
#include "pmic.h"
#include "ledstrip.h"

static const char TAG[] = "BSP";

static pmic_t pmic = {
    .pin_irq = 15,
    .i2c_bus = 0,
    .i2c_address = 0x6a,
};

static ledstrip_t ledstrip = {
    .pin = 10,
};

// Platform-specific BSP init code.
esp_err_t bsp_platform_init() {
    esp_err_t res;
    // Enable GPIO interrupts.
    gpio_install_isr_service(0);

    // Enable internal I²C bus.
    i2c_config_t i2c_conf = {
        .mode       = I2C_MODE_MASTER,
        .master = {
            .clk_speed = 400000,
        },
        .scl_io_num = 2,
        .sda_io_num = 3,
    };
    i2c_param_config(0, &i2c_conf);
    i2c_driver_install(0, I2C_MODE_MASTER, 0, 0, 0);

    // Install co-processor drivers.
    res = pmic_init(&pmic);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PMIC");
        return res;
    }

    res = pmic_get_info(&pmic);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get PMIC information");
        return res;
    }

    res = pmic_dump(&pmic);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to dump PMIC registers");
        return res;
    }

    res = ledstrip_init(&ledstrip);
    if (res != ESP_OK) {
        return res;
    }

    return res;
}

esp_err_t bsp_power_off() {
    const uint8_t led_data[3*5] = {0};
    esp_err_t res = bsp_ledstrip_send(led_data, sizeof(led_data));
    if (res != ESP_OK) {
        return res;
    }
    res = bsp_otg(false);
    if (res != ESP_OK) {
        return res;
    }
    return pmic_power_off(&pmic);
}

esp_err_t bsp_otg(bool enable) {
    return pmic_otg(&pmic, enable);
}

esp_err_t bsp_ledstrip_send(uint8_t* data, int length) {
    return ledstrip_send(&ledstrip, data, length);
}
