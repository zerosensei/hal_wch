/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <soc.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wch_observer, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);

#include "wch_hci_common.h"
#include "wch_bt_hci.h"

static uint8_t observer_taskid;
static void ObserverEventCB(gapRoleEvent_t *pEvent);

static const gapRoleObserverCB_t ObserverRoleCB = {
    ObserverEventCB // Event callback
};

static void ObserverEventCB(gapRoleEvent_t *pEvent)
{
    switch(pEvent->gap.opcode)
    {
        case GAP_DEVICE_INIT_DONE_EVENT:
        {
            LOG_DBG("init done...");
        }
        break;

        case GAP_DEVICE_INFO_EVENT:
        {
            LOG_DBG("dev found\n");
            printf("dev found\n");
	        struct net_buf *buf;
            struct bt_hci_evt_le_advertising_report *sep;
            struct bt_hci_evt_le_advertising_info *adv_info;
            uint8_t info_len;
            int8_t rssi;

            info_len = sizeof(struct bt_hci_evt_le_advertising_info) +
                pEvent->deviceInfo.dataLen;

            buf = wch_hci_meta_evt_create(BT_HCI_EVT_LE_ADVERTISING_REPORT, 
                    sizeof(*sep) + info_len + sizeof(rssi));

            sep = net_buf_add(buf, sizeof(*sep));
            sep->num_reports = 1;

            adv_info = net_buf_add(buf, info_len);
            adv_info->evt_type = pEvent->deviceInfo.eventType;
            adv_info->addr.type = pEvent->deviceInfo.addrType;
            tmos_memcpy(adv_info->addr.a.val, pEvent->deviceInfo.addr,
                     B_ADDR_LEN);
            adv_info->length = pEvent->deviceInfo.dataLen;
            tmos_memcpy(adv_info->data, pEvent->deviceInfo.pEvtData,
                     pEvent->deviceInfo.dataLen);
            
            rssi = pEvent->deviceInfo.rssi;
            net_buf_add_u8(buf, rssi);

            wch_bt_host_rcv_pkt(buf);
        }
        break;

        case GAP_DEVICE_DISCOVERY_EVENT:
        {
            LOG_DBG("Discovery over...");
            // GAPRole_ObserverStartDiscovery(DEVDISC_MODE_ALL,
            //                     TRUE,
            //                     FALSE);
        }
        break;

        // TODO: EXT ADV
        // case GAP_EXT_ADV_DEVICE_INFO_EVENT:
        // {
        //     // Display device addr
        //     PRINT("Recv ext adv \n");
        //     // Add device to list
        //     ObserverAddDeviceInfo(pEvent->deviceExtAdvInfo.addr, pEvent->deviceExtAdvInfo.addrType);
        // }
        // break;

        // case GAP_DIRECT_DEVICE_INFO_EVENT:
        // {
        //     // Display device addr
        //     PRINT("Recv direct adv \n");
        //     // Add device to list
        //     ObserverAddDeviceInfo(pEvent->deviceDirectInfo.addr, pEvent->deviceDirectInfo.addrType);
        // }
        // break;

        default:
            break;
    }
}

static void Observer_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        case GATT_MSG_EVENT:
            break;

        default:
            break;
    }
}

static uint16_t observer_process_event(uint8_t task_id, uint16_t events)
{
    //  VOID task_id; // TMOS required parameter that isn't used in this function
    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(observer_taskid)) != NULL)
        {
            Observer_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);

            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & (1<<1)) {

        printf("curr: %d\n", RTC_GetCycle32k());

        tmos_start_task(observer_taskid, (1<<1), MS1_TO_SYSTEM_TIME(1000));
        return (events ^ (1<<1));

    }


    // Discard unknown events
    return 0;
}


void wch_observer_init(void)
{
    observer_taskid = TMOS_ProcessEventRegister(observer_process_event);
    GAPRole_ObserverStartDevice((gapRoleObserverCB_t *)&ObserverRoleCB);




    tmos_start_task(observer_taskid, (1<<1), MS1_TO_SYSTEM_TIME(100));
}



