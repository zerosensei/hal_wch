/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRIVERS_BLUETOOTH_HCI_WCH_BT_HCI_H
#define DRIVERS_BLUETOOTH_HCI_WCH_BT_HCI_H

#include <zephyr/net_buf.h>

typedef struct {
    void (*host_send_ready)(void);
    void (*host_rcv_pkt)(struct net_buf *buf);
} wch_bt_host_callback_t;

int wch_ble_stack_init(void);
int wch_ble_stack_deinit(void);
void wch_bt_host_rcv_pkt(struct net_buf *buf);
int wch_bt_host_send(struct net_buf *buf);
void wch_bt_host_callback_register(wch_bt_host_callback_t *wch_host_cb);

void wch_bt_idle_clear(void);

#endif /* DRIVERS_BLUETOOTH_HCI_WCH_BT_HCI_H */