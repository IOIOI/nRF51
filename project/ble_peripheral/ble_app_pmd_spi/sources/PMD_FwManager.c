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

#include "PMD_FwManager.h"
#include "PMD_SpiFlash.h"

#define BIN_1_ADDR ((uint32_t) 0x0)
#define BIN_2_ADDR ((uint32_t) 0x80000)

#define BIN_INFO_OFFEST ((uint32_t) 0)
#define INFO_STATUS_OFFEST ((uint8_t) BIN_INFO_OFFEST)
#define INFO_STATUS_LENGTH ((uint8_t) 1)
#define INFO_SIZE_OFFEST ((uint8_t) INFO_STATUS_OFFEST + INFO_STATUS_LENGTH)
#define INFO_SIZE_LENGTH ((uint8_t) 4)
#define INFO_VERSION_OFFEST ((uint8_t) INFO_SIZE_OFFEST + INFO_SIZE_LENGTH)
#define INFO_VERSION_LENGTH ((uint8_t) 4)
#define INFO_HASH_OFFEST ((uint8_t) INFO_VERSION_OFFEST + INFO_VERSION_LENGTH)
#define INFO_HASH_LENGTH ((uint8_t) 2)
#define INFO_CRC_OFFEST ((uint8_t) INFO_HASH_OFFEST + INFO_HASH_LENGTH)
#define INFO_CRC_LENGTH ((uint8_t) 2)

#define SIG_OFFEST ((uint32_t) 0x80)
#define SIG_LENGTH ((uint32_t) 0x80)

#define MASTER_KEY_OFFEST ((uint32_t) 0x100)
#define MASTER_KEY_LENGTH ((uint32_t) 0x80)

#define SLAVE_KEY_OFFEST ((uint32_t) 0x180)
#define SLAVE_KEY_LENGTH ((uint32_t) 0x80)

#define BIN_OFFEST ((uint32_t) 0x200)

#define CURRENT_BINARY ((uint8_t) 0x80)
#define OLD_BINARY ((uint8_t) 0xFF)
#define NEW_BINARY ((uint8_t) 0x00)

#define MEMORY_SIZE ((uint32_t) 0x100000)
#define MAX_BIN_SIZE ((uint32_t) 0x80000)

static void eraseBinary(uint32_t binaryStartAddress);
static uint32_t getAddrOfInactiveBinary();

/* ########################## */
/* BEGINN OF STATIC FUNCTIONS */
/* ########################## */
static void eraseBinary(uint32_t binaryStartAddress)
{
    int index;
    union MemoryAddress tmpAddr;

    tmpAddr.address = binaryStartAddress;

    for(index = 0; 8 > index; index++) // TODO: Magical Numbers ersetzen
    {
        blockErase(tmpAddr);
        tmpAddr.addressBytes[UPPER_BYTE]++;
    }
}

static uint32_t getAddrOfInactiveBinary()
{
    struct binaryInfo info;

    getBinaryInfo(BIN_1_ADDR, &info);

#ifdef DEBUG
    NRF_LOG("Bin_1 Status: ");
    NRF_LOG_HEX(info.status);
    NRF_LOG("\r\n");
#endif

    if(OLD_BINARY == info.status)
    {
        return BIN_1_ADDR;
    }
    else
    {
        return BIN_2_ADDR;
    }
}
/* ########################## */
/* END OF STATIC FUNCTIONS    */
/* ########################## */

/* ########################## */
/* BEGINN OF GLOBAL FUNCTIONS */
/* ########################## */

void initFwManager()
{
    spi_init();
}

void getBinaryInfo(uint32_t addr, struct binaryInfo* info)
{
    union MemoryAddress tmpAddr;
    struct transmissionData data;
    uint8_t index;

    tmpAddr.address = addr + BIN_INFO_OFFEST;

#ifndef USE_FAST_READ
    readData(tmpAddr, &data);
#else
    fastReadData(tmpAddr, &data);
#endif /* USE_FAST_READ */

    if(data.rx_length == 0)
    {
        return;
    }

    info->status = data.rx_data[0];

    for(index = INFO_SIZE_OFFEST; INFO_SIZE_OFFEST + INFO_SIZE_LENGTH > index; index++)
    {
        info->size = info->size << 8;
        info->size = info->size | data.rx_data[index];
    }

    for(index = INFO_VERSION_OFFEST; INFO_VERSION_OFFEST + INFO_VERSION_LENGTH > index; index++)
    {
        info->version = info->version << 8;
        info->version = info->version | data.rx_data[index];
    }

    for(index = INFO_HASH_OFFEST; INFO_HASH_OFFEST + INFO_HASH_LENGTH > index; index++)
    {
        info->hash = info->hash << 8;
        info->hash = info->hash | data.rx_data[index];
    }

    for(index = INFO_CRC_OFFEST; INFO_CRC_OFFEST + INFO_CRC_LENGTH > index; index++)
    {
        info->crc = info->crc << 8;
        info->crc = info->crc | data.rx_data[index];
    }
}

uint8_t writeBinary(uint8_t* data, uint32_t data_length)
{
    static volatile uint32_t addr = 0;
    static volatile uint32_t currAddress = 0;
    static volatile uint32_t binarySize = 0;
    union MemoryAddress tmpAddr;
    uint8_t index;

    if(NULL == (data) || (0 == data_length))
    {
#ifdef DEBUG
        NRF_LOG("RESET Progress\r\n");
#endif
        addr = 0;
        currAddress = 0;
        binarySize = 0;
        return RESET_PROGRESS;
    }

    if(0 == currAddress)
    {
        if(MAX_BIN_SIZE - BIN_OFFEST <= binarySize)
        {
#ifdef DEBUG
            NRF_LOG("ERROR BINARY SIZE\r\n");
#endif
            return ERROR_BIN_SIZE;
        }

        addr = getAddrOfInactiveBinary();

#ifdef DEBUG
        NRF_LOG("binaryAddr: ");
        NRF_LOG_HEX(addr);
        NRF_LOG("\r\n");
#endif

        eraseBinary(addr);

        for(index = 1; 4 >= index; index++)
        {
            binarySize = binarySize << 8;
            binarySize = binarySize | data[index];
        }

#ifdef DEBUG
        NRF_LOG("binarySize: ");
        NRF_LOG_HEX(binarySize);
        NRF_LOG("\r\n");
#endif

        data[0] = NEW_BINARY;
    }

    tmpAddr.address = addr + currAddress;

#ifdef DEBUG
    NRF_LOG("Current address: ");
    NRF_LOG_HEX(tmpAddr.address);
    NRF_LOG("\r\n");
#endif

    splitAndStoreData(data, data_length, tmpAddr);
    currAddress += data_length;

    if(binarySize + BIN_OFFEST == currAddress)
    {
#ifdef DEBUG
        NRF_LOG("Success\r\n");
#endif

        addr = 0;
        currAddress = 0;
        binarySize = 0;
        return SUCCESS;
    }

    return WRITE_IN_PROGRESS;
}

void writeBinaryStatus(uint8_t status, uint32_t binaryAddr)
{
    union MemoryAddress binAddr;
    binAddr.address = binaryAddr;

    pageProgram(&status, 1, binAddr);
}
/* ########################## */
/* END OF GLOBAL FUNCTIONS    */
/* ########################## */
