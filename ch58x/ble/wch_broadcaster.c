/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <soc.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wch_broadcaster, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);

#include "wch_hci_common.h"
#include "wch_bt_hci.h"

static void Broadcaster_StateNotificationCB(gapRole_States_t newState);

static uint8_t Broadcaster_TaskID;
// GAP Role Callbacks
static gapRolesBroadcasterCBs_t Broadcaster_BroadcasterCBs = {
    Broadcaster_StateNotificationCB, // Profile State Change Callbacks
    NULL
};

static void Broadcaster_StateNotificationCB(gapRole_States_t newState)
{
    switch(newState)
    {
        case GAPROLE_STARTED:
            LOG_INF("Initialized..");
            break;

        case GAPROLE_ADVERTISING:
            LOG_INF("Advertising..");
            break;

        case GAPROLE_WAITING:
            LOG_INF("Waiting for advertising..");
            break;

        case GAPROLE_ERROR:
            LOG_INF("Error..");
            break;

        default:
            break;
    }
}

static void Broadcaster_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
    switch(pMsg->event)
    {
        default:
            break;
    }
}

static uint16_t Broadcaster_ProcessEvent(uint8_t task_id, uint16_t events)
{
    //  VOID task_id; // TMOS required parameter that isn't used in this function

    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pMsg;

        if((pMsg = tmos_msg_receive(Broadcaster_TaskID)) != NULL)
        {
            Broadcaster_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);

            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & (1<<0)) {

        printf("test\n");

        tmos_start_task(Broadcaster_TaskID, (1<<0), MS1_TO_SYSTEM_TIME(1000));
        return (events ^ (1<<0));
    }

    // Discard unknown events
    return 0;
}


void wch_broadcaster_init(void)
{
    Broadcaster_TaskID = TMOS_ProcessEventRegister(Broadcaster_ProcessEvent);
    GAPRole_BroadcasterStartDevice(&Broadcaster_BroadcasterCBs);
    // tmos_start_task(Broadcaster_TaskID, (1<<0), MS1_TO_SYSTEM_TIME(1000));
}


