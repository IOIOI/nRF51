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

#ifndef ADVERTISING_H_
#define ADVERTISING_H_

// extern "C" {
#include "ble_advertising.h"
// }
#define DEVICE_NAME                     "Nordic_PMD"                           /**< Name of device. Will be included in the advertising data. */

static const uint32_t APP_ADV_INTERVAL = 40;                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
static const uint32_t APP_ADV_TIMEOUT_IN_SECONDS = 30;                     /**< The advertising timeout in units of seconds. */

void advertising_init(void);
void advertising_start(void);
void advertising_restart(void);

#endif /* ADVERTISING_H_ */
