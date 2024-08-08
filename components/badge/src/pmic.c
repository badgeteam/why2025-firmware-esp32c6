
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

// REG02

esp_err_t pmic_adc_control(pmic_t* device, bool enable, bool continuous) {
    const uint8_t reg = 0x02;
    uint8_t buffer[2] = {reg, 0x00};
    xSemaphoreTake(device->concurrency_semaphore, portMAX_DELAY);
    esp_err_t result = i2c_master_write_read_device(device->i2c_bus, device->i2c_address, (uint8_t[]){reg}, 1, &buffer[1], 1, pdMS_TO_TICKS(50));
    if (result != ESP_OK) {
        xSemaphoreGive(device->concurrency_semaphore);
        return result;
    }
    if (enable) {
        buffer[1] |= (1 << 7);
    } else {
        buffer[1] &= ~(1 << 7);
    }
    if (continuous) {
        buffer[1] |= (1 << 6);
    } else {
        buffer[1] &= ~(1 << 6);
    }
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


esp_err_t pmic_set_minimum_system_voltage_limit(pmic_t* device, uint16_t voltage) {
    // Voltage in mV
    const uint8_t reg = 0x03;
    uint8_t buffer[2] = {reg, 0x00};

    xSemaphoreTake(device->concurrency_semaphore, portMAX_DELAY);
    esp_err_t result = i2c_master_write_read_device(device->i2c_bus, device->i2c_address, (uint8_t[]){reg}, 1, &buffer[1], 1, pdMS_TO_TICKS(50));

    printf("Read value: %02x\r\n", buffer[1]);

    if (result != ESP_OK) {
        xSemaphoreGive(device->concurrency_semaphore);
        return result;
    }

    buffer[1] &= 0b11110001; // Mask

    buffer[1] |= 0b10000000; // Enable battery load

    if (voltage < 3000) { // Minimum voltage is 3V
        voltage = 3000;
    }
    voltage -= 3000; // Offset is 3V

    if (voltage >= 400) {
        voltage -= 400;
        buffer[1] |= (0b100) << 1; // Add 0.4V
    }
    if (voltage >= 200) {
        voltage -= 200;
        buffer[1] |= (0b010) << 1; // Add 0.2V
    }
    if (voltage >= 100) {
        voltage -= 100;
        buffer[1] |= (0b001) << 1; // Add 0.1V
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

esp_err_t pmic_read_faults(pmic_t* device) {
    const uint8_t reg = 0x0C;
    uint8_t buffer[2] = {reg, 0x00};
    xSemaphoreTake(device->concurrency_semaphore, portMAX_DELAY);
    esp_err_t result = i2c_master_write_read_device(device->i2c_bus, device->i2c_address, (uint8_t[]){reg}, 1, &buffer[1], sizeof(buffer) - 1, pdMS_TO_TICKS(50));
    if (result != ESP_OK) {
        xSemaphoreGive(device->concurrency_semaphore);
        return result;
    }

    bool watchdog = (buffer[1] >> 7) & 1;
    bool boost = (buffer[1] >> 6) & 1;

    bool chrg_input = ((buffer[1] >> 4) & 3) == 0b01;
    bool chrg_thermal = ((buffer[1] >> 4) & 3) == 0b10;
    bool chrg_safety = ((buffer[1] >> 4) & 3) == 0b11;

    bool batt_ovp = (buffer[1] >> 3) & 1;

    bool ntc_cold = ((buffer[1] >> 0) & 3) == 0b01;
    bool ntc_hot = ((buffer[1] >> 0) & 3) == 0b10;
    bool ntc_boost = (buffer[1] >> 2) & 1;

    printf("Faults: %s%s%s%s%s%s%s%s%s (%02x)\r\n", watchdog?"WDOG ":"", boost?"BOOST ":"", chrg_input?"CHRG-INPUT ":"", chrg_thermal?"CHRG-THERMAL ":"", chrg_safety ? "CHRG-SAFETY ":"", batt_ovp ? "BATT-OVP ":"", ntc_cold ? "NTC-COLD ":"", ntc_hot ? "NTC-HOT ":"", ntc_boost ? "(boost) ":(ntc_hot || ntc_cold ? "(buck) ":""), buffer[1]);

    xSemaphoreGive(device->concurrency_semaphore);
    return result;
}

esp_err_t pmic_adc_test(pmic_t* device) {
    const uint8_t reg = 0x0E;
    uint8_t buffer[6] = {reg, 0x00, 0x00, 0x00, 0x00, 0x00};
    xSemaphoreTake(device->concurrency_semaphore, portMAX_DELAY);
    esp_err_t result = i2c_master_write_read_device(device->i2c_bus, device->i2c_address, (uint8_t[]){reg}, 1, &buffer[1], sizeof(buffer) - 1, pdMS_TO_TICKS(50));
    if (result != ESP_OK) {
        xSemaphoreGive(device->concurrency_semaphore);
        return result;
    }

    bool treg = (buffer[1] >> 7) & 1;
    uint16_t vbatt = 2304;
    if ((buffer[1] >> 6) & 1) vbatt += 1280;
    if ((buffer[1] >> 5) & 1) vbatt += 640;
    if ((buffer[1] >> 4) & 1) vbatt += 320;
    if ((buffer[1] >> 3) & 1) vbatt += 160;
    if ((buffer[1] >> 2) & 1) vbatt += 80;
    if ((buffer[1] >> 1) & 1) vbatt += 40;
    if ((buffer[1] >> 0) & 1) vbatt += 20;

    uint16_t vsys = 2304;
    if ((buffer[2] >> 6) & 1) vsys += 1280;
    if ((buffer[2] >> 5) & 1) vsys += 640;
    if ((buffer[2] >> 4) & 1) vsys += 320;
    if ((buffer[2] >> 3) & 1) vsys += 160;
    if ((buffer[2] >> 2) & 1) vsys += 80;
    if ((buffer[2] >> 1) & 1) vsys += 40;
    if ((buffer[2] >> 0) & 1) vsys += 20;

    float ts = 21;
    if ((buffer[3] >> 6) & 1) ts += 29.76;
    if ((buffer[3] >> 5) & 1) ts += 14.88;
    if ((buffer[3] >> 4) & 1) ts += 7.44;
    if ((buffer[3] >> 3) & 1) ts += 3.72;
    if ((buffer[3] >> 2) & 1) ts += 1.86;
    if ((buffer[3] >> 1) & 1) ts += 0.93;
    if ((buffer[3] >> 0) & 1) ts += 0.465;

    bool vbus_attached = (buffer[4] >> 7) & 1;
    uint16_t vbus = 2304;
    if ((buffer[4] >> 6) & 1) vbus += 1280;
    if ((buffer[4] >> 5) & 1) vbus += 640;
    if ((buffer[4] >> 4) & 1) vbus += 320;
    if ((buffer[4] >> 3) & 1) vbus += 160;
    if ((buffer[4] >> 2) & 1) vbus += 80;
    if ((buffer[4] >> 1) & 1) vbus += 40;
    if ((buffer[4] >> 0) & 1) vbus += 20;

    uint16_t charge_current = 0;
    if ((buffer[5] >> 6) & 1) charge_current += 3200;
    if ((buffer[5] >> 5) & 1) charge_current += 1600;
    if ((buffer[5] >> 4) & 1) charge_current += 800;
    if ((buffer[5] >> 3) & 1) charge_current += 400;
    if ((buffer[5] >> 2) & 1) charge_current += 200;
    if ((buffer[5] >> 1) & 1) charge_current += 100;
    if ((buffer[5] >> 0) & 1) charge_current += 50;

    printf("Treg: %s, Vbatt: %u mV, Vsys: %u mV, TS %f%%, Vbus: %s %u mV, Ichrg: %u mA\r\n", treg ? "Y" : "N", vbatt, vsys, ts, vbus_attached ? "Y" : "N", vbus, charge_current);

    printf("%02x %02x %02x %02x %02x\r\n", buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);


    xSemaphoreGive(device->concurrency_semaphore);
    //pmic_dump(device);
    pmic_read_faults(device);
    return ESP_OK;
}
