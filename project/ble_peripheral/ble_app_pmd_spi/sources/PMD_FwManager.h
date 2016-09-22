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

#include <stdint.h>

#define RESET_PROGRESS ((uint8_t) 0x40)
#define WRITE_IN_PROGRESS ((uint8_t) 0x80)
#define ERROR_WRONG_ADDRESS ((uint8_t) 0xE)
#define ERROR_BIN_SIZE ((uint8_t) 0xF0)
#define SUCCESS ((uint8_t) 0x00)

struct binaryInfo {
    uint8_t  status;
    uint32_t size;
    uint32_t version;
    uint16_t hash;
    uint16_t crc;
};

void initFwManager();
void getBinaryInfo(uint32_t addr, struct binaryInfo* info);
uint8_t writeBinary(uint8_t* data, uint32_t data_length);
void writeBinaryStatus(uint8_t status, uint32_t binaryAddr);

void spi_testFncWrite();
void spi_testFncRead(uint32_t tmpAddr);
void spi_testFncInfo();
void spi_testFncBin();
void testFnc();
