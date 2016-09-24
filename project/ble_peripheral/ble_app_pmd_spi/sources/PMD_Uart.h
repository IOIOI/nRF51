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
* \file PMD_Uart.h                                                  <!-- Has to be changed according to the real file name -->
*
* \brief Uart API for the firmware Manager.                                                         <!-- Brief description -->
*
* This module handles data which must be sent to the STM32 boards of the PMD.                    <!-- Detailed description -->
*
******************************************************************************************************************************
* <!-- Authors -->
******************************************************************************************************************************
*
* \author Daniel Tatzel                                                                      <!-- One line for each author -->
*
*****************************************************************************************************************************/

#ifndef PMD_UART_H
#define PMD_UART_H

/*===========================================================================================================================|
|  INCLUDES                                                                                                                  |
============================================================================================================================*/
#include <stdbool.h>
#include <stdint.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "nrf_drv_config.h"

/*===========================================================================================================================|
|  GLOBAL CONSTANT / FUNCTION MACROS                                                                                         |
============================================================================================================================*/


/*===========================================================================================================================|
|  GLOBAL DATA TYPES AND STRUCTURES                                                                                          |
============================================================================================================================*/


/*===========================================================================================================================|
|  GLOBAL DATA PROTOTYPES                                                                                                    |
============================================================================================================================*/


/*===========================================================================================================================|
|  GLOBAL FUNCTION PROTOTYPES                                                                                                |
============================================================================================================================*/
/**
 * @brief Function for handling errors from the uart module.
 *
 * @param[in] p_event    The event that caused the error.
 */
void uartErrorHandler(app_uart_evt_t * p_event);

/**
 * @brief Function for initialising the uart module.

 */
void initUart();

/**
 * @brief Uart test function.
 */
void testUart();

#endif /* PMD_UART_H */
