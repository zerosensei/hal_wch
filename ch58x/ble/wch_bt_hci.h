/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_BLUETOOTH_HCI_WCH_BT_HCI_H
#define DRIVERS_BLUETOOTH_HCI_WCH_BT_HCI_H

#include <zephyr/net/buf.h>

typedef struct {
    void (*host_send_ready)(void);
    void (*host_rcv_pkt)(struct net_buf *buf);
} wch_bt_host_callback_t;


int wch_ble_stack_init(void);
void wch_bt_host_rcv_pkt(struct net_buf *buf);
int wch_bt_host_send(struct net_buf *buf);
void wch_bt_host_callback_register(wch_bt_host_callback_t *wch_host_cb);


#endif /* DRIVERS_BLUETOOTH_HCI_WCH_BT_HCI_H */