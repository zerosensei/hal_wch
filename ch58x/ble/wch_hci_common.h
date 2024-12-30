/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_BLUETOOTH_HCI_WCH_HCI_COMMON_H
#define ZEPHYR_DRIVERS_BLUETOOTH_HCI_WCH_HCI_COMMON_H

#include <stdint.h>
#include <zephyr/drivers/bluetooth/hci_driver.h>

#include "wch_bt_hci.h"

struct net_buf *wch_hci_meta_evt_create(uint8_t subevt, uint8_t melen);

void wch_observer_init(void);
void wch_broadcaster_init(void);
void wch_peripheral_init(void);


#endif /* ZEPHYR_DRIVERS_BLUETOOTH_HCI_WCH_HCI_COMMON_H */