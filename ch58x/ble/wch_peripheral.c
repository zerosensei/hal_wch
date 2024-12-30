/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <soc.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wch_peripheral, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);

#include "wch_hci_common.h"
#include "wch_bt_hci.h"

static void peripheralStateNotificationCB(gapRole_States_t newState,
                             gapRoleEvent_t *pEvent);
static void peripheralRssiCB(uint16_t connHandle, int8_t rssi);
static void peripheralParamUpdateCB(uint16_t connHandle, uint16_t connInterval,
                                    uint16_t connSlaveLatency, uint16_t connTimeout);

static uint8_t perip_task_id = INVALID_TASK_ID; 

static gapBondCBs_t Peripheral_BondMgrCBs = {
    NULL, // Passcode callback (not used by application)
    NULL, // Pairing / Bonding state Callback (not used by application)
    NULL  // oob callback
};

static gapRolesCBs_t Peripheral_PeripheralCBs = {
    peripheralStateNotificationCB, // Profile State Change Callbacks
    peripheralRssiCB,              // When a valid RSSI is read from controller (not used by application)
    peripheralParamUpdateCB
};

static void peripheralParamUpdateCB(uint16_t connHandle, uint16_t connInterval,
                                    uint16_t connSlaveLatency, uint16_t connTimeout)
{
    LOG_INF("Update %x - Int %x \n", connHandle, connInterval);

}

static void peripheralRssiCB(uint16_t connHandle, int8_t rssi)
{
    LOG_INF("RSSI -%d dB Conn  %x \n", -rssi, connHandle);
}

static void peripheralStateNotificationCB(gapRole_States_t newState, gapRoleEvent_t *pEvent)
{
    switch(newState & GAPROLE_STATE_ADV_MASK)
    {
        case GAPROLE_STARTED:
            LOG_INF("Initialized..\n");
            break;

        case GAPROLE_ADVERTISING:
            if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                // Peripheral_LinkTerminated(pEvent);
                LOG_INF("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
                LOG_INF("Advertising..\n");
            }
            else if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                LOG_INF("Advertising..\n");
            }
            break;

        case GAPROLE_CONNECTED:
            if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                // Peripheral_LinkEstablished(pEvent);
                LOG_INF("Connected..\n");
            }
            break;

        case GAPROLE_CONNECTED_ADV:
            if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT)
            {
                LOG_INF("Connected Advertising..\n");
            }
            break;

        case GAPROLE_WAITING:
            if(pEvent->gap.opcode == GAP_END_DISCOVERABLE_DONE_EVENT)
            {
                LOG_INF("Waiting for advertising..\n");
            }
            else if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
            {
                // Peripheral_LinkTerminated(pEvent);
                LOG_INF("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
            }
            else if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
            {
                if(pEvent->gap.hdr.status != SUCCESS)
                {
                    LOG_INF("Waiting for advertising..\n");
                }
                else
                {
                    LOG_ERR("Error..\n");
                }
            }
            else
            {
                LOG_ERR("Error..%x\n", pEvent->gap.opcode);
            }
            break;

        case GAPROLE_ERROR:
            LOG_ERR("Error..\n");
            break;

        default:
            break;
    }
}

static void Peripheral_ProcessGAPMsg(gapRoleEvent_t *pEvent)
{
    switch(pEvent->gap.opcode)
    {
        case GAP_SCAN_REQUEST_EVENT:
        {
            LOG_INF("Receive scan req from %x %x %x %x %x %x  ..\n", pEvent->scanReqEvt.scannerAddr[0],
                  pEvent->scanReqEvt.scannerAddr[1], pEvent->scanReqEvt.scannerAddr[2], pEvent->scanReqEvt.scannerAddr[3],
                  pEvent->scanReqEvt.scannerAddr[4], pEvent->scanReqEvt.scannerAddr[5]);
            break;
        }

        case GAP_PHY_UPDATE_EVENT:
        {
            LOG_INF("Phy update Rx:%x Tx:%x ..\n", pEvent->linkPhyUpdate.connRxPHYS, pEvent->linkPhyUpdate.connTxPHYS);
            break;
        }

        default:
            break;
    }
}

static void Peripheral_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        case GAP_MSG_EVENT:
        {
            Peripheral_ProcessGAPMsg((gapRoleEvent_t *)pMsg);
            break;
        }

        case GATT_MSG_EVENT:
        {
            gattMsgEvent_t *pMsgEvent;

            pMsgEvent = (gattMsgEvent_t *)pMsg;
            if(pMsgEvent->method == ATT_MTU_UPDATED_EVENT)
            {
                LOG_INF("mtu exchange: %d\n", pMsgEvent->msg.exchangeMTUReq.clientRxMTU);
            }
            break;
        }

        default:
            break;
    }
}

uint16_t peripheral_processevent(uint8_t task_id, uint16_t events)
{
    
    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(perip_task_id)) != NULL)
        {
            Peripheral_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);
            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    return 0;

}


void wch_peripheral_init(void)
{
    perip_task_id = TMOS_ProcessEventRegister(peripheral_processevent);
    // Setup the GAP Peripheral Role Profile
    {
        // uint16_t desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        // uint16_t desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

        // GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t), &desired_min_interval);
        // GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t), &desired_max_interval);
    }

    // Setup the GAP Bond Manager
    // {
    //     uint32_t passkey = 0; // passkey "000000"
    //     uint8_t  pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    //     uint8_t  mitm = TRUE;
    //     uint8_t  bonding = TRUE;
    //     uint8_t  ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    //     GAPBondMgr_SetParameter(GAPBOND_PERI_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
    //     GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    //     GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    //     GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    //     GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    // }

    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes

    // Set the GAP Characteristics
    // GGS_SetParameter(GGS_DEVICE_NAME_ATT, sizeof(attDeviceName), attDeviceName);

    GAPRole_PeripheralStartDevice(perip_task_id, &Peripheral_BondMgrCBs, &Peripheral_PeripheralCBs);
}