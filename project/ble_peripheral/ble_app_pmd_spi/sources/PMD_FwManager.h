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
* \file PMD_FwManager.h                                             <!-- Has to be changed according to the real file name -->
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

#ifndef PMD_FWMANAGER_H
#define PMD_FWMANAGER_H

/*===========================================================================================================================|
|  INCLUDES                                                                                                                  |
============================================================================================================================*/
#include <stdint.h>

/*===========================================================================================================================|
|  GLOBAL CONSTANT / FUNCTION MACROS                                                                                         |
============================================================================================================================*/
#define RESET_PROGRESS ((uint8_t) 0x40)         /**< Error Code if the progress has been reseted. */
#define WRITE_IN_PROGRESS ((uint8_t) 0x80)      /**< Error code if binary has not been fully transmitted yet. */
#define SUCCESS ((uint8_t) 0x00)                /**< Error code if writing of the binary was successful. */
// #define ERROR_WRONG_ADDRESS ((uint8_t) 0xE)     /**< Error code if wrong address was used. */
#define ERROR_BIN_SIZE ((uint8_t) 0xF0)         /**< Error code if size of the binary is to big. */

/*===========================================================================================================================|
|  GLOBAL DATA TYPES AND STRUCTURES                                                                                          |
============================================================================================================================*/
/**@brief Struct containing information of a binary. */
struct binaryInfo {
    uint8_t  status;
    uint32_t size;
    uint32_t version;
    uint16_t hash;
    uint16_t crc;
};

/*===========================================================================================================================|
|  GLOBAL DATA PROTOTYPES                                                                                                    |
============================================================================================================================*/


/*===========================================================================================================================|
|  GLOBAL FUNCTION PROTOTYPES                                                                                                |
============================================================================================================================*/
/**
 * @brief Function for the initialisation of the firmware manager.
 */
void initFwManager();

/**
 * @brief Function for getting information about a bianry.
 *
 * @param[in] addr    Address (without offset) of the binary of which the info must be retreived .
 * @param[in] info    Struct which contains the retreived information.
 */
void getBinaryInfo(uint32_t addr, struct binaryInfo* info);

/**
 * @brief Function for writing a new binary into the flash memory.
 *
 * @param[in] data          Data buffer to be written.
 * @param[in] dataLength    Length of the data buffer.
 *
 * @retval RESET_PROGRESS         If the progress of writing a binary has been reseted.
 * @retval WRITE_IN_PROGRESS      If the binary has not been completly transmitted yet.
 * @retval SUCCESS                If the binary has been completly transmitted.
 * @retval ERROR_BIN_SIZE         If the size of the binary is to big.
 */
uint8_t writeBinary(uint8_t* data, uint32_t dataLength);

/**
 * @brief Function for overwriting the status byte of the binary.
 *
 * @param[in] status        Value of the status to be written (CURRENT_BINARY, OLD_BINARY or NEW_BINARY).
 * @param[in] binaryAddr    Address (without offset) of the binary.
 */
void writeBinaryStatus(uint8_t status, uint32_t binaryAddr);

#endif /* PMD_FWMANAGER_H */
