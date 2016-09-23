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

#define APP_LOG NRF_LOG

void timers_init(void);
void gap_params_init(void);
void services_init(void);
void conn_params_init(void);
void ble_stack_init(void);
void peer_manager_init(bool erase_bonds);
void buttons_leds_init(bool* p_erase_bonds);
void power_manage(void);
