
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t bsp_platform_init(void);
esp_err_t bsp_power_off(void);
esp_err_t bsp_otg(bool enable);
esp_err_t bsp_ledstrip_send(uint8_t* data, int length);
esp_err_t bsp_test_pmic(void);
