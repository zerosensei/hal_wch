/*
 * Copyright (c) 2025 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wch_central, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);
#include <soc.h>

#include "wch_hci_common.h"
#include "wch_bt_hci.h"

static uint8_t central_taskid;

static void scan_event_cb(gapRoleEvent_t *event);

static gapCentralRoleCB_t central_role_cb = {
    .eventCB = scan_event_cb,
};
static gapBondCBs_t central_bond_cb = {NULL};

static void scan_event_cb(gapRoleEvent_t *event)
{
    switch (event->gap.opcode) {
        case GAP_DEVICE_INIT_DONE_EVENT:
            LOG_DBG("init done...");
        break;

        case GAP_DEVICE_INFO_EVENT: {
            LOG_DBG("dev found");
            struct net_buf *buf;
            struct bt_hci_evt_le_advertising_report *sep;
            struct bt_hci_evt_le_advertising_info *adv_info;
            uint8_t info_len;
            int8_t rssi;

            info_len = sizeof(struct bt_hci_evt_le_advertising_info) + 
                event->deviceInfo.dataLen;
            buf = wch_hci_meta_evt_create(BT_HCI_EVT_LE_ADVERTISING_REPORT,
                    sizeof(*sep) + info_len + sizeof(rssi));
            sep = net_buf_add(buf, sizeof(*sep));
            sep->num_reports = 1;

            adv_info = net_buf_add(buf, info_len);
            adv_info->evt_type = event->deviceInfo.eventType;
            adv_info->addr.type = event->deviceInfo.addrType;
            tmos_memcpy(adv_info->addr.a.val, event->deviceInfo.addr, B_ADDR_LEN);
            adv_info->length = event->deviceInfo.dataLen;
            tmos_memcpy(adv_info->data, event->deviceInfo.pEvtData,
                 event->deviceInfo.dataLen);
            rssi = event->deviceInfo.rssi;
            net_buf_add_u8(buf, rssi);

            wch_bt_host_rcv_pkt(buf);
        }
        break;

        case GAP_DEVICE_DISCOVERY_EVENT:
            LOG_DBG("Discovery over...");
        break;

        //TODO: EXT ADV
        // case GAP_EXT_ADV_DEVICE_INFO_EVENT:
        // {
        //     // Display device addr
        //     printf("Recv ext adv \n");
        //     // Add device to list
        //     ObserverAddDeviceInfo(pEvent->deviceExtAdvInfo.addr, pEvent->deviceExtAdvInfo.addrType);
        // }
        // break;

        // case GAP_DIRECT_DEVICE_INFO_EVENT:
        // {
        //     // Display device addr
        //     printf("Recv direct adv \n");
        //     // Add device to list
        //     ObserverAddDeviceInfo(pEvent->deviceDirectInfo.addr, pEvent->deviceDirectInfo.addrType);
        // }
        // break;

        default:
            break;
    }
}

static uint16_t scan_process_event(uint8_t taskid, uint16_t events)
{
    if (events & SYS_EVENT_MSG) {
        uint8_t *pmsg = tmos_msg_receive(central_taskid);

        if (pmsg != NULL) {
            
            tmos_msg_deallocate(pmsg);
        }

        return (events ^ SYS_EVENT_MSG);
    }

    return 0;
}

int wch_bt_central_init(void)
{
    int ret;
    central_taskid = TMOS_ProcessEventRegister(scan_process_event);

    ret = GATT_InitClient();

    if (ret) {
        LOG_ERR("GATT_InitClient failed: %d", ret);
        return ret;
    }

    GATT_RegisterForInd(central_taskid);
    ret = GAPRole_CentralStartDevice(central_taskid, &central_bond_cb, &central_role_cb);

    if (ret) {
        LOG_ERR("Central start failed: %d", ret);
        return ret;
    }

    return 0;
}

