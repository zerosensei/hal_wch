/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/random/random.h>  //TODO: 
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wch_hci, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);

#include "wch_hci_common.h"


#if defined(CONFIG_BT_CONN)
static uint32_t conn_count;
#endif

#define DEFAULT_EVENT_MASK           0x1fffffffffff
#define DEFAULT_EVENT_MASK_PAGE_2    0x0
#define DEFAULT_LE_EVENT_MASK        0x1f


static uint64_t event_mask = DEFAULT_EVENT_MASK;
static uint64_t event_mask_page_2 = DEFAULT_EVENT_MASK_PAGE_2;
static uint64_t le_event_mask = DEFAULT_LE_EVENT_MASK;
/* opcode of the HCI command currently being processed. The opcode is stored
 * by hci_cmd_handle() and then used during the creation of cmd complete and
 * cmd status events to avoid passing it up the call chain.
 */
static uint16_t _opcode;




void *hci_cmd_complete(struct net_buf **buf, uint8_t plen)
{
	*buf = bt_hci_cmd_complete_create(_opcode, plen);

	return net_buf_add(*buf, plen);
}

static struct net_buf *cmd_status(uint8_t status)
{
	return bt_hci_cmd_status_create(_opcode, status);
}

static struct net_buf *cmd_complete_status(uint8_t status)
{
	struct net_buf *buf;
	struct bt_hci_evt_cc_status *ccst;

	buf = bt_hci_cmd_complete_create(_opcode, sizeof(*ccst));
	ccst = net_buf_add(buf, sizeof(*ccst));
	ccst->status = status;

	return buf;
}

static int wch_link_control_cmd_handle(uint16_t  ocf, struct net_buf *cmd,
				   struct net_buf **evt)
{
	switch (ocf) {
#if defined(CONFIG_BT_CONN)
	case BT_OCF(BT_HCI_OP_DISCONNECT):
		// disconnect(cmd, evt);
		break;
	case BT_OCF(BT_HCI_OP_READ_REMOTE_VERSION_INFO):
		// read_remote_ver_info(cmd, evt);
		break;
#endif /* CONFIG_BT_CONN */
	default:
		return -EINVAL;
	}

	return 0;
}



static void wch_set_event_mask(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_cp_set_event_mask *cmd = (void *)buf->data;

	event_mask = sys_get_le64(cmd->events);

	*evt = cmd_complete_status(0x00);
}

static void wch_set_event_mask_page_2(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_cp_set_event_mask_page_2 *cmd = (void *)buf->data;

	event_mask_page_2 = sys_get_le64(cmd->events_page_2);

	*evt = cmd_complete_status(0x00);
}

static void wch_reset(struct net_buf *buf, struct net_buf **evt)
{
	/* reset event masks */
	event_mask = DEFAULT_EVENT_MASK;
	event_mask_page_2 = DEFAULT_EVENT_MASK_PAGE_2;
	le_event_mask = DEFAULT_LE_EVENT_MASK;

	if (buf) {
		// ll_reset();  //TODO: reset wch lib
		*evt = cmd_complete_status(0x00);
	}

#if defined(CONFIG_BT_CONN)
	conn_count = 0U;
#endif
}

#if defined(CONFIG_BT_CONN)
static void wch_read_tx_power_level(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_cp_read_tx_power_level *cmd = (void *)buf->data;
	struct bt_hci_rp_read_tx_power_level *rp;
	uint16_t handle;
	uint8_t status;
	uint8_t type;

	handle = sys_le16_to_cpu(cmd->handle);
	type = cmd->type;

	rp = hci_cmd_complete(evt, sizeof(*rp));

	status = 0;
    rp->tx_power_level = LL_TX_POWEER_0_DBM; //TODO: get tx power level

	rp->status = status;
	rp->handle = sys_cpu_to_le16(handle);
}
#endif /* CONFIG_BT_CONN */

static int wch_ctrl_bb_cmd_handle(uint16_t  ocf, struct net_buf *cmd,
			      struct net_buf **evt)
{
	switch (ocf) {
	case BT_OCF(BT_HCI_OP_SET_EVENT_MASK):
		wch_set_event_mask(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_RESET):
		wch_reset(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_SET_EVENT_MASK_PAGE_2):
		wch_set_event_mask_page_2(cmd, evt);
		break;

#if defined(CONFIG_BT_CONN)
	case BT_OCF(BT_HCI_OP_READ_TX_POWER_LEVEL):
		wch_read_tx_power_level(cmd, evt);
		break;
#endif /* CONFIG_BT_CONN */

#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
	case BT_OCF(BT_HCI_OP_SET_CTL_TO_HOST_FLOW):
		set_ctl_to_host_flow(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_HOST_BUFFER_SIZE):
		host_buffer_size(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_HOST_NUM_COMPLETED_PACKETS):
		host_num_completed_packets(cmd, evt);
		break;
#endif

#if defined(CONFIG_BT_CTLR_LE_PING)
	case BT_OCF(BT_HCI_OP_READ_AUTH_PAYLOAD_TIMEOUT):
		read_auth_payload_timeout(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_WRITE_AUTH_PAYLOAD_TIMEOUT):
		write_auth_payload_timeout(cmd, evt);
		break;
#endif /* CONFIG_BT_CTLR_LE_PING */

#if defined(CONFIG_BT_CTLR_HCI_CODEC_AND_DELAY_INFO)
	case BT_OCF(BT_HCI_OP_CONFIGURE_DATA_PATH):
		configure_data_path(cmd, evt);
		break;
#endif /* CONFIG_BT_CTLR_HCI_CODEC_AND_DELAY_INFO */

	default:
		return -EINVAL;
	}

	return 0;
}




static void wch_read_local_version_info(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_rp_read_local_version_info *rp;

	rp = hci_cmd_complete(evt, sizeof(*rp));

	rp->status = 0x00;
	rp->hci_version = BT_HCI_VERSION_5_4;
	rp->hci_revision = sys_cpu_to_le16(0);
	rp->lmp_version = BT_HCI_VERSION_5_4;
	rp->manufacturer = sys_cpu_to_le16(0x07D7);
	rp->lmp_subversion = sys_cpu_to_le16(0x0200);  //TODO
}

static void wch_read_supported_commands(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_rp_read_supported_commands *rp;

	rp = hci_cmd_complete(evt, sizeof(*rp));

	rp->status = 0x00;
	(void)memset(&rp->commands[0], 0, sizeof(rp->commands));

#if defined(CONFIG_BT_REMOTE_VERSION)
	/* Read Remote Version Info. */
	rp->commands[2] |= BIT(7);
#endif
	/* Set Event Mask, and Reset. */
	rp->commands[5] |= BIT(6) | BIT(7);

	/* Read TX Power Level. */
	rp->commands[10] |= BIT(2);

	/* Read Local Version Info, Read Local Supported Features. */
	rp->commands[14] |= BIT(3) | BIT(5);
	/* Read BD ADDR. */
	rp->commands[15] |= BIT(1);

	/* Read RSSI. */
	rp->commands[15] |= BIT(5);

	/* Set Event Mask Page 2 */
	rp->commands[22] |= BIT(2);
	/* LE Set Event Mask, LE Read Buffer Size, LE Read Local Supp Feats,
	 * Set Random Addr
	 */
	rp->commands[25] |= BIT(0) | BIT(1) | BIT(2) | BIT(4);

	/* LE Read FAL Size, LE Clear FAL */
	rp->commands[26] |= BIT(6) | BIT(7);
	/* LE Add Dev to FAL, LE Remove Dev from FAL */
	rp->commands[27] |= BIT(0) | BIT(1);

	/* LE Encrypt, LE Rand */
	rp->commands[27] |= BIT(6) | BIT(7);
	/* LE Read Supported States */
	rp->commands[28] |= BIT(3);

#if defined(CONFIG_BT_BROADCASTER)
	/* LE Set Adv Params, LE Read Adv Channel TX Power, LE Set Adv Data */
	rp->commands[25] |= BIT(5) | BIT(6) | BIT(7);
	/* LE Set Scan Response Data, LE Set Adv Enable */
	rp->commands[26] |= BIT(0) | BIT(1);

	/* LE Set Adv Set Random Addr, LE Set Ext Adv Params, LE Set Ext Adv
	 * Data, LE Set Ext Adv Scan Rsp Data, LE Set Ext Adv Enable, LE Read
	 * Max Adv Data Len, LE Read Num Supp Adv Sets
	 */
	rp->commands[36] |= BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) |
			    BIT(6) | BIT(7);
	/* LE Remove Adv Set, LE Clear Adv Sets */
	rp->commands[37] |= BIT(0) | BIT(1);
	/* LE Set PA Params, LE Set PA Data, LE Set PA Enable */
	rp->commands[37] |= BIT(2) | BIT(3) | BIT(4);
#endif /* CONFIG_BT_BROADCASTER */

#if defined(CONFIG_BT_OBSERVER)
	/* LE Set Scan Params, LE Set Scan Enable */
	rp->commands[26] |= BIT(2) | BIT(3);

	/* LE Set Extended Scan Params, LE Set Extended Scan Enable */
	rp->commands[37] |= BIT(5) | BIT(6);
	/* LE PA Create Sync, LE PA Create Sync Cancel, LE PA Terminate Sync */
	rp->commands[38] |= BIT(0) | BIT(1) | BIT(2);
	/* LE PA Add Device to Periodic Advertiser List,
	 * LE PA Remove Device from Periodic Advertiser List,
	 * LE Clear Periodic Advertiser List,
	 * LE Read Periodic Adveritiser List Size
	 */
	rp->commands[38] |= BIT(3) | BIT(4) | BIT(5) | BIT(6);
	/* LE Set PA Receive Enable */
	rp->commands[40] |= BIT(5);
#endif /* CONFIG_BT_OBSERVER */

#if defined(CONFIG_BT_CONN)
#if defined(CONFIG_BT_CENTRAL)
	/* LE Create Connection, LE Create Connection Cancel */
	rp->commands[26] |= BIT(4) | BIT(5);
	/* Set Host Channel Classification */
	rp->commands[27] |= BIT(3);

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	/* LE Extended Create Connection */
	rp->commands[37] |= BIT(7);
#endif /* CONFIG_BT_CTLR_ADV_EXT */

#if defined(CONFIG_BT_CTLR_LE_ENC)
	/* LE Start Encryption */
	rp->commands[28] |= BIT(0);
#endif /* CONFIG_BT_CTLR_LE_ENC */

#if defined(CONFIG_BT_CTLR_CENTRAL_ISO)
	/* LE Set CIG Parameters */
	rp->commands[41] |= BIT(7);
	/* LE Set CIG Parameters Test, LE Create CIS, LE Remove CIS */
	rp->commands[42] |= BIT(0) | BIT(1) | BIT(2);
#endif /* CONFIG_BT_CTLR_CENTRAL_ISO */
#endif /* CONFIG_BT_CENTRAL */

#if defined(CONFIG_BT_PERIPHERAL)
#if defined(CONFIG_BT_CTLR_LE_ENC)
	/* LE LTK Request Reply, LE LTK Request Negative Reply */
	rp->commands[28] |= BIT(1) | BIT(2);
#endif /* CONFIG_BT_CTLR_LE_ENC */
#if defined(CONFIG_BT_CTLR_PERIPHERAL_ISO)
	/* LE Accept CIS Request, LE Reject CIS Request */
	rp->commands[42] |= BIT(3) | BIT(4);
#endif /* CONFIG_BT_CTLR_PERIPHERAL_ISO */
#endif /* CONFIG_BT_PERIPHERAL */

	/* Disconnect. */
	rp->commands[0] |= BIT(5);
	/* LE Connection Update, LE Read Channel Map, LE Read Remote Features */
	rp->commands[27] |= BIT(2) | BIT(4) | BIT(5);

#if defined(CONFIG_BT_CTLR_CONN_PARAM_REQ)
	/* LE Remote Conn Param Req and Neg Reply */
	rp->commands[33] |= BIT(4) | BIT(5);
#endif /* CONFIG_BT_CTLR_CONN_PARAM_REQ */

#if defined(CONFIG_BT_CTLR_LE_PING)
	/* Read and Write authenticated payload timeout */
	rp->commands[32] |= BIT(4) | BIT(5);
#endif /* CONFIG_BT_CTLR_LE_PING */

#if defined(CONFIG_BT_CTLR_DATA_LENGTH)
	/* LE Set Data Length, and LE Read Suggested Data Length. */
	rp->commands[33] |= BIT(6) | BIT(7);
	/* LE Write Suggested Data Length. */
	rp->commands[34] |= BIT(0);
	/* LE Read Maximum Data Length. */
	rp->commands[35] |= BIT(3);
#endif /* CONFIG_BT_CTLR_DATA_LENGTH */

#if defined(CONFIG_BT_CTLR_PHY)
	/* LE Read PHY Command. */
	rp->commands[35] |= BIT(4);
	/* LE Set Default PHY Command. */
	rp->commands[35] |= BIT(5);
	/* LE Set PHY Command. */
	rp->commands[35] |= BIT(6);
#endif /* CONFIG_BT_CTLR_PHY */
#if defined(CONFIG_BT_CTLR_SCA_UPDATE)
	/* LE Request Peer SCA */
	rp->commands[43] |= BIT(2);
#endif /* CONFIG_BT_CTLR_SCA_UPDATE */
#endif /* CONFIG_BT_CONN */

	/* LE RX Test, LE TX Test, LE Test End */
	rp->commands[28] |= BIT(4) | BIT(5) | BIT(6);
	/* LE Enhanced RX Test. */
	rp->commands[35] |= BIT(7);
	/* LE Enhanced TX Test. */
	rp->commands[36] |= BIT(0);
	rp->commands[39] |= BIT(3);

	rp->commands[39] |= BIT(4);

	/* LE resolving list commands, LE Read Peer RPA */
	rp->commands[34] |= BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7);
	/* LE Read Local RPA, LE Set AR Enable, Set RPA Timeout */
	rp->commands[35] |= BIT(0) | BIT(1) | BIT(2);
	/* LE Set Privacy Mode */
	rp->commands[39] |= BIT(2);

	/* LE Read TX Power. */
	rp->commands[38] |= BIT(7);

	/* LE Set Host Feature */
	rp->commands[44] |= BIT(1);
}

static void wch_read_local_features(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_rp_read_local_features *rp;

	rp = hci_cmd_complete(evt, sizeof(*rp));

	rp->status = 0x00;
	(void)memset(&rp->features[0], 0x00, sizeof(rp->features));
	/* BR/EDR not supported and LE supported */
	rp->features[4] = (1 << 5) | (1 << 6);
}

static void wch_read_bd_addr(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_rp_read_bd_addr *rp;

	rp = hci_cmd_complete(evt, sizeof(*rp));

	rp->status = 0x00;

	GAPRole_GetParameter(GAPROLE_BD_ADDR, &rp->bdaddr.val[0]);
}


static int wch_info_cmd_handle(uint16_t  ocf, struct net_buf *cmd,
			   struct net_buf **evt)
{
	switch (ocf) {
	case BT_OCF(BT_HCI_OP_READ_LOCAL_VERSION_INFO):
		wch_read_local_version_info(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_READ_SUPPORTED_COMMANDS):
		wch_read_supported_commands(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_READ_LOCAL_FEATURES):
		wch_read_local_features(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_READ_BD_ADDR):
		wch_read_bd_addr(cmd, evt);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int wch_status_cmd_handle(uint16_t  ocf, struct net_buf *cmd,
			     struct net_buf **evt)
{
	switch (ocf) {
#if defined(CONFIG_BT_CTLR_CONN_RSSI)
	case BT_OCF(BT_HCI_OP_READ_RSSI):
		// read_rssi(cmd, evt);
		break;
#endif /* CONFIG_BT_CTLR_CONN_RSSI */

	default:
		return -EINVAL;
	}

	return 0;
}

static void wch_le_set_event_mask(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_cp_set_event_mask *cmd = (void *)buf->data;

	le_event_mask = sys_get_le64(cmd->events);

	*evt = cmd_complete_status(0x00);
}

static void wch_le_read_buffer_size(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_rp_le_read_buffer_size *rp;

	rp = hci_cmd_complete(evt, sizeof(*rp));

	rp->status = 0x00;

	rp->le_max_len = sys_cpu_to_le16(MIN(CONFIG_BT_BUF_ACL_TX_SIZE, 251U)); //TODO: 读宏还是stack
	rp->le_max_num = CONFIG_BT_BUF_ACL_TX_COUNT;
}


static void wch_le_read_local_features(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_rp_le_read_local_features *rp;

	rp = hci_cmd_complete(evt, sizeof(*rp));

	rp->status = 0x00;

	(void)memset(&rp->features[0], 0x00, sizeof(rp->features));

    
    //TODO: features
	sys_put_le64(0, rp->features);
}

static void wch_le_set_random_address(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_cp_le_set_random_address *cmd = (void *)buf->data;
	uint8_t status;

	// status = ll_addr_set(1, &cmd->bdaddr.val[0]);
	status = 0;  //TODO:

	*evt = cmd_complete_status(status);
}

static void wch_le_rand(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_rp_le_rand *rp;
	uint8_t count = sizeof(rp->rand);

	rp = hci_cmd_complete(evt, sizeof(*rp));
	rp->status = 0x00;

	sys_rand_get(rp->rand, count);
}

static void wch_le_read_supp_states(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_rp_le_read_supp_states *rp;
	uint64_t states = 0U;

	rp = hci_cmd_complete(evt, sizeof(*rp));
	rp->status = 0x00;

#define ST_ADV (BIT64(0)  | BIT64(1)  | BIT64(8)  | BIT64(9)  | BIT64(12) | \
		BIT64(13) | BIT64(16) | BIT64(17) | BIT64(18) | BIT64(19) | \
		BIT64(20) | BIT64(21))

#define ST_SCA (BIT64(4)  | BIT64(5)  | BIT64(8)  | BIT64(9)  | BIT64(10) | \
		BIT64(11) | BIT64(12) | BIT64(13) | BIT64(14) | BIT64(15) | \
		BIT64(22) | BIT64(23) | BIT64(24) | BIT64(25) | BIT64(26) | \
		BIT64(27) | BIT64(30) | BIT64(31))

#define ST_PER (BIT64(2)  | BIT64(3)  | BIT64(7)  | BIT64(10) | BIT64(11) | \
		BIT64(14) | BIT64(15) | BIT64(20) | BIT64(21) | BIT64(26) | \
		BIT64(27) | BIT64(29) | BIT64(30) | BIT64(31) | BIT64(32) | \
		BIT64(33) | BIT64(34) | BIT64(35) | BIT64(36) | BIT64(37) | \
		BIT64(38) | BIT64(39) | BIT64(40) | BIT64(41))

#define ST_CEN (BIT64(6)  | BIT64(16) | BIT64(17) | BIT64(18) | BIT64(19) | \
		BIT64(22) | BIT64(23) | BIT64(24) | BIT64(25) | BIT64(28) | \
		BIT64(32) | BIT64(33) | BIT64(34) | BIT64(35) | BIT64(36) | \
		BIT64(37) | BIT64(41))

#if defined(CONFIG_BT_BROADCASTER)
	states |= ST_ADV;
#else
	states &= ~ST_ADV;
#endif
#if defined(CONFIG_BT_OBSERVER)
	states |= ST_SCA;
#else
	states &= ~ST_SCA;
#endif
#if defined(CONFIG_BT_PERIPHERAL)
	states |= ST_PER;
#else
	states &= ~ST_PER;
#endif
#if defined(CONFIG_BT_CENTRAL)
	states |= ST_CEN;
#else
	states &= ~ST_CEN;
#endif
	/* All states and combinations supported except:
	 * Initiating State + Passive Scanning
	 * Initiating State + Active Scanning
	 */
	states &= ~(BIT64(22) | BIT64(23));  //TODO: 抓包看下
	LOG_DBG("states: 0x%08x%08x", (uint32_t)(states >> 32), (uint32_t)(states & 0xffffffff));
	sys_put_le64(states, rp->le_states);
}

#if defined(CONFIG_BT_BROADCASTER)
static void wch_le_set_adv_param(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_cp_le_set_adv_param *cmd = (void *)buf->data;
	uint16_t min_interval;
	uint8_t status;

	min_interval = sys_le16_to_cpu(cmd->min_interval);

	if ((cmd->type != BT_HCI_ADV_DIRECT_IND)) {
		uint16_t max_interval = sys_le16_to_cpu(cmd->max_interval);

		if ((min_interval > max_interval) ||
		    (min_interval < 0x0020) ||
		    (max_interval > 0x4000)) {
			*evt = cmd_complete_status(BT_HCI_ERR_INVALID_PARAM);
			return;
		}
	}

    status = GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, min_interval);
    status |= GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, min_interval);
    status |= GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, 
				sizeof(cmd->channel_map), &cmd->channel_map);
    status |= GAPRole_SetParameter(GAPROLE_ADV_FILTER_POLICY, 
				sizeof(cmd->filter_policy), &cmd->filter_policy);
    status |= GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, 
				sizeof(uint8_t), &cmd->type);

    if ((cmd->type == BT_HCI_ADV_DIRECT_IND) ||
        (cmd->type == BT_HCI_ADV_DIRECT_IND_LOW_DUTY)) {
        status |= GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR, BT_ADDR_SIZE,
            cmd->direct_addr.a.val);
        status |= GAPRole_SetParameter(GAPROLE_ADV_DIRECT_TYPE, 
            sizeof(cmd->direct_addr.type), &cmd->direct_addr.type);
    }

	*evt = cmd_complete_status(status);
}

static void wch_le_read_adv_chan_tx_power(struct net_buf *buf, 
        struct net_buf **evt)
{
	struct bt_hci_rp_le_read_chan_tx_power *rp;

	rp = hci_cmd_complete(evt, sizeof(*rp));

	rp->status = 0x00;

	rp->tx_power_level = 0;
}

static void wch_le_set_adv_data(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_cp_le_set_adv_data *cmd = (void *)buf->data;
	uint8_t status;

    status = GAPRole_SetParameter(GAPROLE_ADVERT_DATA, cmd->len, &cmd->data[0]);

	*evt = cmd_complete_status(status);
}

static void wch_le_set_scan_rsp_data(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_cp_le_set_scan_rsp_data *cmd = (void *)buf->data;
	uint8_t status;

    status =GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, cmd->len, &cmd->data[0]);

	*evt = cmd_complete_status(status);
}

static void wch_le_set_adv_enable(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_cp_le_set_adv_enable *cmd = (void *)buf->data;
	uint8_t status;

    status = GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, 
        sizeof(uint8_t), &cmd->enable);

	*evt = cmd_complete_status(status);
}

#endif /* CONFIG_BT_BROADCASTER */


#if defined(CONFIG_BT_OBSERVER)
static uint8_t scan_type = 0;
static uint8_t filter_policy = 0;
static void wch_le_set_scan_param(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_cp_le_set_scan_param *cmd = (void *)buf->data;
	uint16_t interval;
	uint16_t window;
	uint8_t status;

	interval = sys_le16_to_cpu(cmd->interval);
	window = sys_le16_to_cpu(cmd->window);

    status = GAP_SetParamValue(TGAP_DISC_SCAN_INT, interval);
    status |= GAP_SetParamValue(TGAP_DISC_SCAN_WIND, window);

    scan_type = cmd->scan_type;
    filter_policy = cmd->filter_policy;

	*evt = cmd_complete_status(status);
}

static void wch_le_set_scan_enable(struct net_buf *buf, struct net_buf **evt)
{
	uint8_t status = 0;

    status = GAP_SetParamValue(TGAP_DISC_SCAN, 0);
    status |= GAPRole_ObserverStartDiscovery(DEVDISC_MODE_ALL, scan_type, 
                (uint8_t) (filter_policy & 0x01));

	*evt = cmd_complete_status(status);
}
#endif /* CONFIG_BT_OBSERVER */

#if defined(CONFIG_BT_CENTRAL)


#endif /* CONFIG_BT_CENTRAL */

#if defined(CONFIG_BT_CONN)


static void wch_le_conn_update(struct net_buf *buf, struct net_buf **evt)
{
	struct hci_cp_le_conn_update *cmd = (void *)buf->data;
	uint16_t supervision_timeout;
	uint16_t conn_interval_min;
	uint16_t conn_interval_max;
	uint16_t conn_latency;
	uint16_t handle;
	uint8_t status;

	handle = sys_le16_to_cpu(cmd->handle);
	conn_interval_min = sys_le16_to_cpu(cmd->conn_interval_min);
	conn_interval_max = sys_le16_to_cpu(cmd->conn_interval_max);
	conn_latency = sys_le16_to_cpu(cmd->conn_latency);
	supervision_timeout = sys_le16_to_cpu(cmd->supervision_timeout);

#if defined(CONFIG_BT_CENTRAL)
	status = GAPRole_UpdateLink(handle, conn_interval_min, conn_interval_max,
                conn_latency, supervision_timeout);
#endif

#if defined(CONFIG_BT_PERIPHERAL)
    status = GAPRole_PeripheralConnParamUpdateReq(handle,conn_interval_min, 
                conn_interval_max, conn_latency, supervision_timeout, 
                0xFF);
#endif

	*evt = cmd_status(status);
}


#endif /* CONFIG_BT_CONN */



static int wch_controller_cmd_handle(uint16_t ocf, struct net_buf *cmd,
                        struct net_buf **evt, void **node_rx)
{
    switch (ocf) {
	case BT_OCF(BT_HCI_OP_LE_SET_EVENT_MASK):
		wch_le_set_event_mask(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_LE_READ_BUFFER_SIZE):
		wch_le_read_buffer_size(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_LE_READ_LOCAL_FEATURES):
		wch_le_read_local_features(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_LE_SET_RANDOM_ADDRESS):
		wch_le_set_random_address(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_LE_RAND):
		wch_le_rand(cmd, evt);
		break;
	case BT_OCF(BT_HCI_OP_LE_READ_SUPP_STATES):
		wch_le_read_supp_states(cmd, evt);
		break;

#if defined(CONFIG_BT_BROADCASTER)
	case BT_OCF(BT_HCI_OP_LE_SET_ADV_PARAM):
		wch_le_set_adv_param(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_LE_READ_ADV_CHAN_TX_POWER):
		wch_le_read_adv_chan_tx_power(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_LE_SET_ADV_DATA):
		wch_le_set_adv_data(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_LE_SET_SCAN_RSP_DATA):
		wch_le_set_scan_rsp_data(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_LE_SET_ADV_ENABLE):
		wch_le_set_adv_enable(cmd, evt);
		break;
#endif /* CONFIG_BT_BROADCASTER */

#if defined(CONFIG_BT_OBSERVER)
	case BT_OCF(BT_HCI_OP_LE_SET_SCAN_PARAM):
		wch_le_set_scan_param(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_LE_SET_SCAN_ENABLE):
		wch_le_set_scan_enable(cmd, evt);
		break;

#endif /* CONFIG_BT_OBSERVER */

#if defined(CONFIG_BT_CENTRAL)
	case BT_OCF(BT_HCI_OP_LE_CREATE_CONNBT_HCI_OP_LE_CREATE_CONN):
		le_create_connection(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_LE_CREATE_CONN_CANCEL):
		le_create_conn_cancel(cmd, evt, node_rx);
		break;

	case BT_OCF(BT_HCI_OP_LE_SET_HOST_CHAN_CLASSIF):
		le_set_host_chan_classif(cmd, evt);
		break;
#endif /* CONFIG_BT_CENTRAL */

#if defined(CONFIG_BT_PERIPHERAL)
    //Nothing
#endif /* CONFIG_BT_PERIPHERAL */

#if defined(CONFIG_BT_CONN)
	// case BT_OCF(BT_HCI_OP_LE_READ_CHAN_MAP):
	// 	le_read_chan_map(cmd, evt);
	// 	break;

#if defined(CONFIG_BT_CENTRAL)
	case BT_OCF(BT_HCI_OP_LE_READ_REMOTE_FEATURES):
		// le_read_remote_features(cmd, evt);
		break;
#endif /* CONFIG_BT_CENTRAL */

	case BT_OCF(BT_HCI_OP_LE_CONN_UPDATE):
		wch_le_conn_update(cmd, evt);
		break;

#endif /* CONFIG_BT_CONN */


    default:
		return -EINVAL;  
    };

    return 0;
}

static struct net_buf *wch_hci_cmd_handle(struct net_buf *cmd, void **node_rx)
{
    struct bt_hci_cmd_hdr *chdr;
	struct net_buf *evt = NULL;
	uint16_t ocf;
	int err = 0;

	if (cmd->len < sizeof(*chdr)) {
		LOG_ERR("No HCI Command header");
		return NULL;
	}

	chdr = net_buf_pull_mem(cmd, sizeof(*chdr));
	if (cmd->len < chdr->param_len) {
		LOG_ERR("Invalid HCI CMD packet length");
		return NULL;
	}

	/* store in a global for later CC/CS event creation */
	_opcode = sys_le16_to_cpu(chdr->opcode);

	ocf = BT_OCF(_opcode);

	switch (BT_OGF(_opcode)) {
	case BT_OGF_LINK_CTRL:
		err = wch_link_control_cmd_handle(ocf, cmd, &evt);
		break;
	case BT_OGF_BASEBAND:
		err = wch_ctrl_bb_cmd_handle(ocf, cmd, &evt);
		break;
	case BT_OGF_INFO:
		err = wch_info_cmd_handle(ocf, cmd, &evt);
		break;
	case BT_OGF_STATUS:
		err = wch_status_cmd_handle(ocf, cmd, &evt);
		break;
	case BT_OGF_LE:
		err = wch_controller_cmd_handle(ocf, cmd, &evt, node_rx);
		break;
#if defined(CONFIG_BT_HCI_VS)
	case BT_OGF_VS:
		err = hci_vendor_cmd_handle(ocf, cmd, &evt);
		break;
#endif
	default:
		err = -EINVAL;
		break;
	}

	if (err == -EINVAL) {
		evt = cmd_status(BT_HCI_ERR_UNKNOWN_CMD);
	}

	return evt;
}

static int wch_cmd_handle(struct net_buf *buf)
{
	struct net_buf *node_rx = NULL;  //controllor fanhui de evt
	struct net_buf *evt;

	evt = wch_hci_cmd_handle(buf, (void **) &node_rx);
	if (evt) {
		LOG_DBG("Replying with event of %u bytes", evt->len);
		
        wch_bt_host_rcv_pkt(evt);
        //TODO: call bt_rcv? note about priority

		if (node_rx) {  //TODO: neccessary?
			LOG_DBG("RX node enqueue");
			
            wch_bt_host_rcv_pkt(evt);  
		}
	}

	return 0;
}
#if defined(CONFIG_BT_CONN)
static int wch_acl_handle(struct net_buf *buf)
{
#if 0
	struct node_tx *node_tx;
	struct bt_hci_acl_hdr *acl;
	struct pdu_data *pdu_data;
	uint16_t handle;
	uint8_t flags;
	uint16_t len;

	if (buf->len < sizeof(*acl)) {
		LOG_ERR("No HCI ACL header");
		return -EINVAL;
	}

	acl = net_buf_pull_mem(buf, sizeof(*acl));
	len = sys_le16_to_cpu(acl->len);
	handle = sys_le16_to_cpu(acl->handle);

	if (buf->len < len) {
		LOG_ERR("Invalid HCI ACL packet length");
		return -EINVAL;
	}

	if (len > LL_LENGTH_OCTETS_TX_MAX) {
		LOG_ERR("Invalid HCI ACL Data length");
		return -EINVAL;
	}

	/* assigning flags first because handle will be overwritten */
	flags = bt_acl_flags(handle);
	handle = bt_acl_handle(handle);

	node_tx = ll_tx_mem_acquire();
	if (!node_tx) {
		LOG_ERR("Tx Buffer Overflow");
		data_buf_overflow(evt, BT_OVERFLOW_LINK_ACL);
		return -ENOBUFS;
	}

	pdu_data = (void *)node_tx->pdu;

	if (bt_acl_flags_bc(flags) != BT_ACL_POINT_TO_POINT) {
		return -EINVAL;
	}

	switch (bt_acl_flags_pb(flags)) {
	case BT_ACL_START_NO_FLUSH:
		pdu_data->ll_id = PDU_DATA_LLID_DATA_START;
		break;
	case BT_ACL_CONT:
		pdu_data->ll_id = PDU_DATA_LLID_DATA_CONTINUE;
		break;
	default:
		/* BT_ACL_START and BT_ACL_COMPLETE not allowed on LE-U
		 * from Host to Controller
		 */
		return -EINVAL;
	}

	pdu_data->len = len;
	memcpy(&pdu_data->lldata[0], buf->data, len);

	if (ll_tx_mem_enqueue(handle, node_tx)) {
		LOG_ERR("Invalid Tx Enqueue");
		ll_tx_mem_release(node_tx);
		return -EINVAL;
	}
#endif
	return 0;
}
#endif /* CONFIG_BT_CONN */

int wch_bt_host_send(struct net_buf *buf)
{
    int ret = 0;
    uint8_t type;

    if (!buf->len) {
		LOG_ERR("Empty HCI packet");
		return -EINVAL;
	}

	type = bt_buf_get_type(buf);
	switch (type) {
#if defined(CONFIG_BT_CONN)
	case BT_BUF_ACL_OUT:
		ret = wch_acl_handle(buf); //TODO: 
		break;
#endif /* CONFIG_BT_CONN */
	case BT_BUF_CMD:
		ret = wch_cmd_handle(buf);
		break;

	default:
		LOG_ERR("Unknown HCI type %u", type);
		return -EINVAL;
	}

	return ret;
}
