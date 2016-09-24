/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/*************************************************************************************************************************//**
* \file PMD_Ble.h                                                   <!-- Has to be changed according to the real file name -->
*
* \brief This module is handling the communication via Bluetooth.                                   <!-- Brief description -->
*
* This module handles the communication via Bluetooth.                                           <!-- Detailed description -->
* It is also responsible for parsing the received data to passthrough the data
* to the corresponding module eg. the firmware manager.
*
******************************************************************************************************************************
* <!-- Authors -->
******************************************************************************************************************************
*
* \author Nordic Semiconductor                                                               <!-- One line for each author -->
* \author Daniel Tatzel
*
*****************************************************************************************************************************/

#ifndef PMD_BLE_H
#define PMD_BLE_H

/*===========================================================================================================================|
|  INCLUDES                                                                                                                  |
============================================================================================================================*/
#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "pstorage.h"
#include "app_uart.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"

#include "advertising.h"

/*===========================================================================================================================|
|  GLOBAL CONSTANT / FUNCTION MACROS                                                                                         |
============================================================================================================================*/
#define APP_LOG NRF_LOG /**< Define for the log type to be used. */

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
 * @brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void timers_init(void);

/**
 * @brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void);

/**
 * @brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Glucose, Battery and Device Information services.
 */
void services_init(void);
/**
 * @brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void);

/**
 * @brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void);

/**
 * @brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
void peer_manager_init(bool erase_bonds);

/**
 * @brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
void buttons_leds_init(bool* p_erase_bonds);

/**
 * @brief Function for the Power manager.
 */
void power_manage(void);

#endif /* PMD_BLE_H */
