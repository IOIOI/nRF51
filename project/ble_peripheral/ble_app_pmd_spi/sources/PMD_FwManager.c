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


/*************************************************************************************************************************//**
* \file PMD_FwManager.c                                             <!-- Has to be changed according to the real file name -->
*
* \brief Firmware manager for the PMD project.                                                      <!-- Brief description -->
*
* This module handles binaries for the STM32 boards of the PMD.                                  <!-- Detailed description -->
* It handles Bluetooth data, stores data in a flash memory and sends the stored binary to the PMD.
*
******************************************************************************************************************************
* <!-- Authors -->
******************************************************************************************************************************
*
* \author Daniel Tatzel                                                                      <!-- One line for each author -->
*
*****************************************************************************************************************************/

/*===========================================================================================================================|
|  INCLUDES                                                                                                                  |
============================================================================================================================*/
#include "PMD_FwManager.h"
#include "PMD_SpiFlash.h"

/*===========================================================================================================================|
|  LOCAL CONSTANT / FUNCTION MACROS                                                                                          |
============================================================================================================================*/
#define BIN_1_ADDR ((uint32_t) 0x0)                                              /**< Memory address of the first binary. */
#define BIN_2_ADDR ((uint32_t) 0x80000)                                          /**< Memory address of the second binary. */

#define BIN_INFO_OFFEST ((uint32_t) 0)                                           /**< Address offset of the binary information block. */
#define INFO_STATUS_OFFEST ((uint8_t) BIN_INFO_OFFEST)                           /**< Address offset of the binary status. */
#define INFO_STATUS_LENGTH ((uint8_t) 1)                                         /**< Length of the binary status in bytes. */
#define INFO_SIZE_OFFEST ((uint8_t) INFO_STATUS_OFFEST + INFO_STATUS_LENGTH)     /**< Address offset of the binary size. */
#define INFO_SIZE_LENGTH ((uint8_t) 4)                                           /**< Length of the binary size in bytes. */
#define INFO_VERSION_OFFEST ((uint8_t) INFO_SIZE_OFFEST + INFO_SIZE_LENGTH)      /**< Address offset of the binary version. */
#define INFO_VERSION_LENGTH ((uint8_t) 4)                                        /**< Length of the binary version in bytes. */
#define INFO_HASH_OFFEST ((uint8_t) INFO_VERSION_OFFEST + INFO_VERSION_LENGTH)   /**< Address offset of the binary hash. */
#define INFO_HASH_LENGTH ((uint8_t) 2)                                           /**< Length of the binary hash in bytes. */
#define INFO_CRC_OFFEST ((uint8_t) INFO_HASH_OFFEST + INFO_HASH_LENGTH)          /**< Address offset of the binary crc. */
#define INFO_CRC_LENGTH ((uint8_t) 2)                                            /**< Length of the binary crc in bytes. */

#define SIG_OFFEST ((uint32_t) 0x80)                                             /**< Address offset of the signature for the binary. */
#define SIG_LENGTH ((uint32_t) 0x80)                                             /**< Length of the signature of the binary in bytes. */

#define MASTER_KEY_OFFEST ((uint32_t) 0x100)                                     /**< Address offset of the master key for the binary. */
#define MASTER_KEY_LENGTH ((uint32_t) 0x80)                                      /**< Length of the master key for the binary in bytes. */

#define SLAVE_KEY_OFFEST ((uint32_t) 0x180)                                      /**< Address offset of the slave key for the binary. */
#define SLAVE_KEY_LENGTH ((uint32_t) 0x80)                                       /**< Length of the slave key for the binary in bytes. */

#define BIN_OFFEST ((uint32_t) 0x200)                                            /**< Address offset of the binary. */

#define CURRENT_BINARY ((uint8_t) 0x80)                                          /**< Binary status value for the current active binary. */
#define OLD_BINARY ((uint8_t) 0xFF)                                              /**< Binary status value of the old (inactive) binary. */
#define NEW_BINARY ((uint8_t) 0x00)                                              /**< Binary status value of a newly transmitted and inactive binary. */

#define MEMORY_SIZE ((uint32_t) 0x100000)                                        /**< Maximum size of the flash memory. */
#define MAX_BIN_SIZE ((uint32_t) 0x80000)                                        /**< Maximum allowed size of a binary. */

/*===========================================================================================================================|
|  LOCAL DATA TYPES AND STRUCTURES                                                                                           |
============================================================================================================================*/


/*===========================================================================================================================|
|  LOCAL DATA PROTOTYPES                                                                                                     |
============================================================================================================================*/


/*===========================================================================================================================|
| LOCAL FUNCTION PROTOTYPES                                                                                                  |
|===========================================================================================================================*/
/**
 * @brief Function for erasing the memory occupied by the specified binary.
 *
 * @param[in] binaryStartAddress    Address (without offset) of the memory which must be erased.
 */
static void eraseBinary(uint32_t binaryStartAddress);

/**
 * @brief Function for getting the address of the current inactive binary.
 *
 * This function returns the address of the inactive binary which can be overwritten with a new one.
 *
 * @retval BIN_1_ADDR                Address of the first binary (withoug offset) if the first binary is currently not in use.
 * @retval BIN_2_ADDR                Address of the second binary (withoug offset) if the second binary is currently not in use.
 */
static uint32_t getAddrOfInactiveBinary();

/*===========================================================================================================================|
|  LOCAL FUNCTIONS                                                                                                           |
|===========================================================================================================================*/
static void eraseBinary(uint32_t binaryStartAddress)
{
    int index;
    union MemoryAddress tmpAddr;

    tmpAddr.address = binaryStartAddress;

    for(index = 0; BYTE_SIZE > index; index++)
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

/*===========================================================================================================================|
|  GLOBAL FUNCTIONS                                                                                                          |
|===========================================================================================================================*/
void initFwManager()
{
    initSPI();
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

    info->status = data.rx_data[INFO_STATUS_OFFEST];

    for(index = INFO_SIZE_OFFEST; INFO_SIZE_OFFEST + INFO_SIZE_LENGTH > index; index++)
    {
        info->size = info->size << BYTE_SIZE;
        info->size = info->size | data.rx_data[index];
    }

    for(index = INFO_VERSION_OFFEST; INFO_VERSION_OFFEST + INFO_VERSION_LENGTH > index; index++)
    {
        info->version = info->version << BYTE_SIZE;
        info->version = info->version | data.rx_data[index];
    }

    for(index = INFO_HASH_OFFEST; INFO_HASH_OFFEST + INFO_HASH_LENGTH > index; index++)
    {
        info->hash = info->hash << BYTE_SIZE;
        info->hash = info->hash | data.rx_data[index];
    }

    for(index = INFO_CRC_OFFEST; INFO_CRC_OFFEST + INFO_CRC_LENGTH > index; index++)
    {
        info->crc = info->crc << BYTE_SIZE;
        info->crc = info->crc | data.rx_data[index];
    }
}

uint8_t writeBinary(uint8_t* data, uint32_t dataLength)
{
    static volatile uint32_t addr = 0;
    static volatile uint32_t currAddress = 0;
    static volatile uint32_t binarySize = 0;
    union MemoryAddress tmpAddr;
    uint8_t index;

    if(NULL == (data) || (0 == dataLength))
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

        for(index = INFO_SIZE_OFFEST; INFO_SIZE_LENGTH >= index; index++)
        {
            binarySize = binarySize << BYTE_SIZE;
            binarySize = binarySize | data[index];
        }

#ifdef DEBUG
        NRF_LOG("binarySize: ");
        NRF_LOG_HEX(binarySize);
        NRF_LOG("\r\n");
#endif

        data[INFO_STATUS_OFFEST] = NEW_BINARY;
    }

    tmpAddr.address = addr + currAddress;

#ifdef DEBUG
    NRF_LOG("Current address: ");
    NRF_LOG_HEX(tmpAddr.address);
    NRF_LOG("\r\n");
#endif

    splitAndStoreData(data, dataLength, tmpAddr);
    currAddress += dataLength;

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
