
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t bsp_platform_init(void);
esp_err_t bsp_ledstrip_send(uint8_t* data, int length);
