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

#include "PMD_Uart.h"

#define UART_TX_BUFFER_SIZE 128                         /**< UART TX buffer size. */
#define UART_RX_BUFFER_SIZE 1                           /**< UART RX buffer size. */

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void initUart()
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
          UART0_CONFIG_PSEL_RXD,
          UART0_CONFIG_PSEL_TXD,
          UART0_CONFIG_PSEL_RTS,
          UART0_CONFIG_PSEL_CTS,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUFFER_SIZE,
                         UART_TX_BUFFER_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
}


uint32_t uartTransfer(uint8_t* data, uint8_t data_length)
{
    uint8_t index;
    uint32_t err_code;

    for (index = 0; index < data_length; index++)
    {
        do
        {
            err_code = app_uart_put(data[i]);

            if(NRF_ERROR_NO_MEM == err_code)
            {
                return NRF_ERROR_NO_MEM;
            }
        }
        while(NRF_SUCCESS != err_code);
    }

    return NRF_SUCCESS;
}

uint32_t uartReceive(uint8_t* data)
{
    uint32_t err_code;

    do
    {
        err_code = app_uart_get(&data);

        if(NRF_ERROR_NOT_FOUND == err_code)
        {
            return NRF_ERROR_NOT_FOUND;
        }
    }
    while(NRF_SUCCESS != err_code);

    return NRF_SUCCESS;
}





/**
 * @brief Function for main application entry.
 */
void testUart(void)
{

    initUart();

}


/** @} */
