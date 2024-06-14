
// SPDX-License-Identifier: MIT

#include <string.h>
#include "pmic.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"



static const char TAG[] = "PMIC";

IRAM_ATTR static void pmic_isr(void* arg) {
    pmic_t* device = (pmic_t*) arg;
    xSemaphoreGiveFromISR(device->interrupt_semaphore, NULL);
    portYIELD_FROM_ISR();
}

static void pmic_thread(void* arg) {
    pmic_t* device = (pmic_t*) arg;
    while (true) {
        xSemaphoreTake(device->interrupt_semaphore, portMAX_DELAY);
        xSemaphoreTake(device->concurrency_semaphore, portMAX_DELAY);

        esp_err_t res = ESP_OK;/*i2c_master_write_read_device(
            BSP_I2CINT_NUM,
            BSP_PMIC_ADDR,
            (uint8_t[]){2}, // I2C_REG_KEYBOARD_0
            1,
            buttons,
            9,
            pdMS_TO_TICKS(50)
        );*/

        xSemaphoreGive(device->concurrency_semaphore);

        if (res != ESP_OK) {
            continue;
        }

        /*
            Process stuff.
        */
    }
}

esp_err_t pmic_init(pmic_t* device) {
    if (device->interrupt_semaphore || device->concurrency_semaphore || device->interrupt_task_handle) {
        return ESP_FAIL; // Already initialized
    }

    device->interrupt_semaphore = xSemaphoreCreateBinary();
    if (device->interrupt_semaphore == NULL) {
        return ESP_ERR_NO_MEM;
    }

    device->concurrency_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(device->concurrency_semaphore);
    if (device->concurrency_semaphore == NULL) {
        vSemaphoreDelete(device->interrupt_semaphore);
        device->interrupt_semaphore = NULL;
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(pmic_thread, "PMIC", 2048, (void*) device, 0, &device->interrupt_task_handle) != pdTRUE) {
        vSemaphoreDelete(device->interrupt_semaphore);
        device->interrupt_semaphore = NULL;
        vSemaphoreDelete(device->concurrency_semaphore);
        device->concurrency_semaphore = NULL;
        return ESP_ERR_NO_MEM;
    }

    gpio_set_direction(device->pin_irq, GPIO_MODE_INPUT);
    gpio_isr_handler_add(device->pin_irq, pmic_isr, (void*) device);
    return ESP_OK;
}

esp_err_t pmic_dump(pmic_t* device) {
    uint8_t buffer[21];
    xSemaphoreTake(device->concurrency_semaphore, portMAX_DELAY);
    esp_err_t result = i2c_master_write_read_device(
        device->i2c_bus,
        device->i2c_address,
        (uint8_t[]){0},
        1,
        buffer,
        sizeof(buffer),
        pdMS_TO_TICKS(50)
    );
    xSemaphoreGive(device->concurrency_semaphore);

    if (result != ESP_OK) {
        return result;
    }

    for (uint8_t i = 0; i < sizeof(buffer); i++) {
        ESP_LOGI(TAG, "#%02x: %02x", i, buffer[i]);
    }

    return result;
}

// REG00

esp_err_t pmic_set_input_current_limit(pmic_t* device, uint16_t current, bool enable_ilim_pin) {
    const uint8_t reg = 0x00;
    uint8_t buffer[2] = {reg, 0x00};

    if (current < 100) { // Minimum current is 100 mA
        current = 100;
    }
    current -= 100; // Offset is 100 mA

    if (enable_ilim_pin) {
        buffer[1] |= 0b1000000; // Enable ILIM pin
    }
    if (current >= 1600) {
        current -= 1600;
        buffer[1] |= 0b100000; // Add 1600 mA
    }
    if (current >= 800) {
        current -= 800;
        buffer[1] |= 0b10000; // Add 800 mA
    }
    if (current >= 400) {
        current -= 400;
        buffer[1] |= 0b1000; // Add 400 mA
    }
    if (current >= 200) {
        current -= 200;
        buffer[1] |= 0b100; // Add 200 mA
    }
    if (current >= 100) {
        current -= 100;
        buffer[1] |= 0b10; // Add 100 mA
    }
    if (current >= 50) {
        current -= 50;
        buffer[1] |= 0b1; // Add 50 mA
    }

    xSemaphoreTake(device->concurrency_semaphore, portMAX_DELAY);
    esp_err_t result = i2c_master_write_to_device(device->i2c_bus, device->i2c_address, buffer, sizeof(buffer), pdMS_TO_TICKS(50));
    xSemaphoreGive(device->concurrency_semaphore);
    return result;
}

esp_err_t pmic_power_off(pmic_t* device) {
    esp_err_t result = pmic_watchdog(device, 0);
    if (result != ESP_OK) {
        return result;
    }
    const uint8_t reg = 0x00;
    uint8_t buffer[2] = {reg, 0x00};
    xSemaphoreTake(device->concurrency_semaphore, portMAX_DELAY);
    result = i2c_master_write_read_device(device->i2c_bus, device->i2c_address, (uint8_t[]){reg}, 1, &buffer[1], 1, pdMS_TO_TICKS(50));
    if (result != ESP_OK) {
        xSemaphoreGive(device->concurrency_semaphore);
        return result;
    }
    buffer[1] |= (1 << 7); // Set EN_HIZ bit
    result = i2c_master_write_to_device(device->i2c_bus, device->i2c_address, buffer, sizeof(buffer), pdMS_TO_TICKS(50));
    xSemaphoreGive(device->concurrency_semaphore);
    return result;
}

// REG03

esp_err_t pmic_otg(pmic_t* device, bool enable) {
    const uint8_t reg = 0x03;
    uint8_t buffer[2] = {reg, 0x00};
    xSemaphoreTake(device->concurrency_semaphore, portMAX_DELAY);
    esp_err_t result = i2c_master_write_read_device(device->i2c_bus, device->i2c_address, (uint8_t[]){reg}, 1, &buffer[1], 1, pdMS_TO_TICKS(50));
    if (result != ESP_OK) {
        xSemaphoreGive(device->concurrency_semaphore);
        return result;
    }
    if (enable) {
        buffer[1] |= (1 << 5);
    } else {
        buffer[1] &= ~(1 << 5);
    }
    result = i2c_master_write_to_device(device->i2c_bus, device->i2c_address, buffer, sizeof(buffer), pdMS_TO_TICKS(50));
    xSemaphoreGive(device->concurrency_semaphore);
    return result;
}

// REG07

esp_err_t pmic_watchdog(pmic_t* device, uint8_t value) {
    const uint8_t reg = 0x07;
    uint8_t buffer[2] = {reg, 0x00};
    xSemaphoreTake(device->concurrency_semaphore, portMAX_DELAY);
    esp_err_t result = i2c_master_write_read_device(device->i2c_bus, device->i2c_address, (uint8_t[]){reg}, 1, &buffer[1], 1, pdMS_TO_TICKS(50));
    if (result != ESP_OK) {
        xSemaphoreGive(device->concurrency_semaphore);
        return result;
    }
    buffer[1] &= ~(0b00110000);
    buffer[1] |= (value & 3) << 4; // Watchdog
    result = i2c_master_write_to_device(device->i2c_bus, device->i2c_address, buffer, sizeof(buffer), pdMS_TO_TICKS(50));
    xSemaphoreGive(device->concurrency_semaphore);
    return result;
}

// REG14

esp_err_t pmic_get_info(pmic_t* device) {
    uint8_t buffer[1];
    xSemaphoreTake(device->concurrency_semaphore, portMAX_DELAY);
    esp_err_t result = i2c_master_write_read_device(
        device->i2c_bus,
        device->i2c_address,
        (uint8_t[]){0x14}, // REG14
        1,
        buffer,
        sizeof(buffer),
        pdMS_TO_TICKS(50)
    );
    xSemaphoreGive(device->concurrency_semaphore);

    if (result != ESP_OK) {
        return result;
    }

    uint8_t dev_rev = (buffer[0] >> 0) & 0b11;
    uint8_t pn = (buffer[0] >> 3) & 0b111;

    ESP_LOGI(TAG, "PN %x REV %x", pn, dev_rev);

    return result;
}