/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "wch_hci_common.h"

struct net_buf *wch_hci_meta_evt_create(uint8_t subevt, uint8_t melen)
{
    struct bt_hci_evt_le_meta_event *meta;
	struct bt_hci_evt_hdr *hdr;
    struct net_buf *buf;

    buf = bt_buf_get_rx(BT_BUF_EVT, K_FOREVER);
	hdr = net_buf_add(buf, sizeof(*hdr));
	hdr->evt = BT_HCI_EVT_LE_META_EVENT;
	hdr->len = sizeof(*meta) + melen;

	meta = net_buf_add(buf, sizeof(*meta));
	meta->subevent = subevt;

    return buf;
}
