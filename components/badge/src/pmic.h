
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"

typedef struct PMIC {
    // Settings
    gpio_num_t pin_irq;
    i2c_port_t i2c_bus;
    uint8_t i2c_address;
    // Internal
    SemaphoreHandle_t interrupt_semaphore;
    SemaphoreHandle_t concurrency_semaphore;
    TaskHandle_t interrupt_task_handle;
} pmic_t;

esp_err_t pmic_init(pmic_t* device);

esp_err_t pmic_dump(pmic_t* device);

// REG00
esp_err_t pmic_set_input_current_limit(pmic_t* device, uint16_t current, bool enable_ilim_pin);
esp_err_t pmic_power_off(pmic_t* device);

// REG02
esp_err_t pmic_adc_control(pmic_t* device, bool enable, bool continuous);
esp_err_t pmic_ico(pmic_t* device, bool enable);

// REG03
esp_err_t pmic_otg(pmic_t* device, bool enable);
esp_err_t pmic_set_minimum_system_voltage_limit(pmic_t* device, uint16_t voltage);

// REG04
esp_err_t pmic_set_fast_charge_current(pmic_t* device, uint16_t current, bool en_pumpx);

// REG07
esp_err_t pmic_watchdog(pmic_t* device, uint8_t value);

// REG14
esp_err_t pmic_get_info(pmic_t* device);



esp_err_t pmic_adc_test(pmic_t* device);
