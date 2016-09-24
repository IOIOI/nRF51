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
* \file PMD_SpiFlash.h                                              <!-- Has to be changed according to the real file name -->
*
* \brief This module uses SPI to store / erase / read data grom the flash memory.                   <!-- Brief description -->
*
* This module is specific to the used flash memory chip.                                         <!-- Detailed description -->
* It uses SPI to communicate with the memory.
*
******************************************************************************************************************************
* <!-- Authors -->
******************************************************************************************************************************
*
* \author Daniel Tatzel                                                                      <!-- One line for each author -->
*
*****************************************************************************************************************************/

#ifndef PMD_SPIFLASH_H
#define PMD_SPIFLASH_H

/*===========================================================================================================================|
|  INCLUDES                                                                                                                  |
============================================================================================================================*/
#include <stdbool.h>
#include <stdint.h>
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "app_error.h"
#include "boards.h"
#include "sdk_common.h"
#include "app_util_platform.h"
#include "nrf_delay.h"

/*===========================================================================================================================|
|  GLOBAL CONSTANT / FUNCTION MACROS                                                                                         |
============================================================================================================================*/
#define BUFFER_SIZE ((uint8_t) 0x80)    /**< Buffer size for spi transmissions. */
#define DUMMY_BYTE ((uint8_t) 0x00)     /**< Defining the value of a dummy byte. */
#define BYTE_SIZE ((uint8_t) 8)         /**< Number of bits in a byte. */

/*===========================================================================================================================|
|  GLOBAL DATA TYPES AND STRUCTURES                                                                                          |
============================================================================================================================*/
/**@brief Union for the memory address. */
union MemoryAddress {
 uint32_t address;
 uint8_t addressBytes[3];
};

/**@brief Enum for accessing the bytes of the MemoryAdress union. */
enum AddressBytes {
    LOWER_BYTE,
    MIDDLE_BYTE,
    UPPER_BYTE
};

/**@brief Struct containing the data of the spi transmit and receive buffer. */
struct transmissionData {
    uint8_t tx_data[BUFFER_SIZE];
    uint8_t tx_length;
    uint8_t rx_data[BUFFER_SIZE];
    uint8_t rx_length;
};


/*===========================================================================================================================|
|  GLOBAL DATA PROTOTYPES                                                                                                    |
============================================================================================================================*/


/*===========================================================================================================================|
|  GLOBAL FUNCTION PROTOTYPES                                                                                                |
============================================================================================================================*/
/**
 * @brief SPI user event handler.
 *
 * @param event
 */
void spiEventHandler(nrf_drv_spi_evt_t const * p_event);

/**
 * @brief SPI initialisation.
 */
void initSPI();

/**
 * @brief SPI Test function.
 */
void spiTestFnc();

/**
 * @brief Set the write protection for specific memory areas.
 *
 * @param Protection area (see datasheet for details)
 */
void setMemWriteProtection(uint8_t blockProtection);

/**
 * @brief Get the current memory protection value.
 *
 * @retval curr_block_protection    Value of the current block protection.
 */
uint8_t getMemWriteProtection();

/**
 * @brief Function for writing data bigger than BUFFER_SIZE.
 *
 * @param[in] data          Data buffer to be written.
 * @param[in] dataLength    Length of the data buffer.
 * @param[in] addr          Start of the memory address.
 */
void splitAndStoreData(uint8_t* data, uint32_t dataLength, union MemoryAddress addr);

#ifndef USE_FAST_READ
/**
 * @brief Function for reading data from the flash memory.
 *
 * @param[in] addr          Start of the memory address.
 * @param[in] retData       Struct in which the read data is stored (set Null if the spi handler is processing the data).
 */
void readData(union MemoryAddress addr, struct transmissionData* retData);

#else
/**
 * @brief Function for reading data fast from the flash memory.
 *
 * @param[in] addr          Start of the memory address.
 * @param[in] retData       Struct in which the read data is stored (set Null if the spi handler is processing the data).
 */
void fastReadData(union MemoryAddress addr, struct transmissionData* retData);

#endif /* USE_FAST_READ */

/**
 * @brief Function for writing data smaller than or equal BUFFER_SIZE.
 *
 * @param[in] data          Data buffer to be written.
 * @param[in] dataLength    Length of the data buffer.
 * @param[in] addr          Start of the memory address.
 */
void pageProgram(uint8_t* data, uint8_t dataLength, union MemoryAddress addr);

/**
 * @brief Function for erasing a whole sector (4KB).
 *
 * @param[in] addr          Start of the memory address.
 */
void sectorErase(union MemoryAddress addr);

/**
 * @brief Function for erasing a whole block (64KB).
 *
 * @param[in] addr          Start of the memory address.
 */
void blockErase(union MemoryAddress addr);

/**
 * @brief Function for erasing the whole flash memory.
 */
void chipErase();

/**
 * @brief Function for activating the deep sleep mode of the flash memory.
 */
void powerDown();

/**
 * @brief Function for waking up the flash memory.
 */
void releasePowerDown();

#ifdef DEBUG
/**
 * @brief Function for reading the device ID
 *
 * @retval Device_ID
 */
uint8_t getDeviceID();

/**
 * @brief Function for reading the manufacturer and device ID
 *
 * The returned value contains the manufactruer ID in the upper byte and
 * the device ID in the lower byte. Manufacturer id = (manufacturer_device_id & 0xFF00) >> BYTE_SIZE.
 * Device id = manufacturer_device_id & 0x00FF.
 *
 * @retval manufacturer_device_id
 */
uint16_t getManufacturerAndDeviceId();
#endif /* DEBUG */

#endif /* PMD_SPIFLASH_H */
