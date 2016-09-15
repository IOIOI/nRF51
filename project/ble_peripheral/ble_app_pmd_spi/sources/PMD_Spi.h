/* Copyright (C) 2015 Daniel Tatzel
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "app_error.h"
#include "boards.h"
#include "sdk_common.h"
#include "app_util_platform.h"

#include "nrf_delay.h"

#define ERROR_WRONG_ADDRESS ((uint32_t) 0xFE000000)
#define ERROR_BINARY_SIZE ((uint32_t) 0xFF000000)
// #define SUCCESS ((uint32_t) 0xFF000000)


/**
 * @brief SPI user event handler.
 * @param event
 */
void pmd_spi_event_handler(nrf_drv_spi_evt_t const * p_event);

/**
 * @brief SPI initialisation.
 */
void pmd_spi_init();

void pmd_spi_transceiver();

void setMemWriteProtection(uint8_t blockProtection);
uint8_t getMemWriteProtection();

void getDeviceID();
void getManufacturerAndDeviceId();

void storeData(uint8_t* data, uint32_t data_length, const uint32_t addr);