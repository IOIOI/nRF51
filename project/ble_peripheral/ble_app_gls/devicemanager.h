/* Copyright (C) 2016  Nils Weiss
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

#ifndef DEVICEMANAGER_H_
#define DEVICEMANAGER_H_

extern "C" {
#include "device_manager.h"
#include "app_timer.h"
#include "pstorage.h"
}

#define APP_TIMER_PRESCALER            0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE        4                                           /**< Size of timer operation queues. */

#define SEC_PARAM_BOND                 1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                 1                                           /**< Man In The Middle protection required (applicable when display module is detected). */
#define SEC_PARAM_IO_CAPABILITIES      BLE_GAP_IO_CAPS_DISPLAY_ONLY                /**< Display I/O capabilities. */
#define SEC_PARAM_OOB                  0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE         7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE         16                                          /**< Maximum encryption key size. */

#define PASSKEY_TXT                    "Passkey:"                                  /**< Message to be displayed together with the pass-key. */
#define PASSKEY_TXT_LENGTH             8                                           /**< Length of message to be displayed together with the pass-key. */
#define PASSKEY_LENGTH                 6                                           /**< Length of pass-key received by the stack for display. */

#define SECURITY_REQUEST_DELAY         APP_TIMER_TICKS(4000, APP_TIMER_PRESCALER)  /**< Delay after connection until Security Request is sent, if necessary (ticks). */

void device_manager_init(const bool erase_bonds);

void device_manager_start_sec_timer(void);
void device_manager_init_sec_timer(void);

void timers_init(void);

#endif /* DEVICEMANAGER_H_ */
