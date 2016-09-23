/* Copyright (C) 2016 Daniel Tatzel
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

#include <stdbool.h>
#include <stdint.h>
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "app_error.h"
#include "boards.h"
#include "sdk_common.h"
#include "app_util_platform.h"
#include "nrf_delay.h"

#define SPI_BUFFER_SIZE ((uint8_t) 0x80)

union MemoryAddress {
 uint32_t address;
 uint8_t addressBytes[3];
};

enum AddressBytes {
    LOWER_BYTE,
    MIDDLE_BYTE,
    UPPER_BYTE
};

struct transmissionData {
    uint8_t tx_data[SPI_BUFFER_SIZE];
    uint8_t tx_length;
    uint8_t rx_data[SPI_BUFFER_SIZE];
    uint8_t rx_length;
};

void spi_event_handler(nrf_drv_spi_evt_t const * p_event);
void spi_init();
void spi_transceiver();

void setMemWriteProtection(uint8_t blockProtection);

uint8_t getMemWriteProtection();

void splitAndStoreData(uint8_t* data, uint32_t data_length, union MemoryAddress addr);

#ifndef USE_FAST_READ

void readData(union MemoryAddress addr, struct transmissionData* retData);

#else

void fastReadData(union MemoryAddress addr, struct transmissionData* retData);

#endif /* USE_FAST_READ */

void pageProgram(uint8_t* data, uint8_t data_length, union MemoryAddress addr);

void sectorErase(union MemoryAddress addr);
void blockErase(union MemoryAddress addr);
void chipErase();

void powerDown();
void releasePowerDown();

#ifdef DEBUG
uint8_t getDeviceID();
uint16_t getManufacturerAndDeviceId();
#endif /* DEBUG */

