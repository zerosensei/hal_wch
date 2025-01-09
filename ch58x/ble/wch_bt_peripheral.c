/*
 * Copyright (c) 2025 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wch_perip, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);
#include <soc.h>

#include "wch_hci_common.h"
#include "wch_bt_hci.h"

static void perip_state_changed(gapRole_States_t newState, gapRoleEvent_t *pEvent);

static uint8_t perip_taskid;
static gapRolesCBs_t perip_cbs = {
    .pfnStateChange = perip_state_changed,
};
static gapBondCBs_t perip_bond_cbs = {NULL};

static void perip_state_changed(gapRole_States_t newState, gapRoleEvent_t *pEvent)
{
    switch(newState & GAPROLE_STATE_ADV_MASK)
    {
        case GAPROLE_STARTED:
            LOG_DBG("Initialized..\n");
            break;

        case GAPROLE_ADVERTISING:
            if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT) {
                // Peripheral_LinkTerminated(pEvent);
                LOG_DBG("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
                LOG_DBG("Advertising..\n");
            } else if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT) {
                LOG_DBG("Advertising..\n");
            }
            break;

        case GAPROLE_CONNECTED:
            if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT) {
                // Peripheral_LinkEstablished(pEvent);
                LOG_DBG("Connected..\n");
            }
            break;

        case GAPROLE_CONNECTED_ADV:
            if(pEvent->gap.opcode == GAP_MAKE_DISCOVERABLE_DONE_EVENT) {
                LOG_DBG("Connected Advertising..\n");
            }
            break;

        case GAPROLE_WAITING:
            if(pEvent->gap.opcode == GAP_END_DISCOVERABLE_DONE_EVENT) {
                LOG_DBG("Waiting for advertising..\n");
            }
            else if(pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT) {
                // Peripheral_LinkTerminated(pEvent);
                LOG_DBG("Disconnected.. Reason:%x\n",
                     pEvent->linkTerminate.reason);
            } else if(pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT) {
                if(pEvent->gap.hdr.status != SUCCESS) {
                    LOG_DBG("Waiting for advertising..\n");
                } else {
                    LOG_ERR("Error..\n");
                }
            } else {
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

static uint16_t perip_process_event(uint8_t taskid, uint16_t events) 
{
    if(events & SYS_EVENT_MSG)
    {
        uint8_t *pmsg = tmos_msg_receive(perip_taskid);

        if(pmsg != NULL)
        {

            // Release the TMOS message
            tmos_msg_deallocate(pmsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    return 0;
}

int wch_bt_peripheral_init(void)
{
    int ret;
    perip_taskid = TMOS_ProcessEventRegister(perip_process_event);

    uint8_t initial_advertising_enable = false;
    ret = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
         &initial_advertising_enable);

    if (ret) {
        LOG_ERR("Set parameter failed: %d", ret);
        return ret;
    }

    ret = GAPRole_PeripheralStartDevice(perip_taskid, &perip_bond_cbs, &perip_cbs);

    if (ret) {
        LOG_ERR("Peripheral start failed: %d", ret);
        return ret;
    }

    return 0;
}