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

/** @file
 *
 * @defgroup ble_sdk_app_gls_main main.c
 * @{
 * @ingroup ble_sdk_app_gls
 * @brief Glucose Meter service Sample Application
 *
 * This file contains the source code for a sample application using the Glucose Meter service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <cstdint>
#include <cstring>
#include <string>

extern "C" {
#include "app_error.h"
#include "boards.h"
#include "nrf_log.h"
#include "nrf_delay.h"

#include "PMD_Ble.h"
#include "PMD_Spi.h"
}

#if BUTTONS_NUMBER < 2
#error "Not enough resources on board to run example"
#endif

#define APP_LOG NRF_LOG

#define DEAD_BEEF                      0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t* p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    APP_ERROR_CHECK(NRF_LOG_INIT());

    timers_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    peer_manager_init(erase_bonds);
    if (erase_bonds == true) {
        APP_LOG("Bonds erased!\r\n");
    }
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    pmd_spi_init();

    LEDS_CONFIGURE(LED_3);
    LEDS_CONFIGURE(LED_4);

    advertising_start();

    APP_LOG("\r\nPMD Start!\r\n");

    for ( ; ; ) {
        power_manage();
        pmd_spi_transceiver();

        nrf_delay_ms(200);
    }
}

/**
 * @}
 */
