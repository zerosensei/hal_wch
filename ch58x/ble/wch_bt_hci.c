/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/random/random.h>
#include <soc.h>

#include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(wch_hci, CONFIG_WCH_HCI_LOG_LEVEL);
LOG_MODULE_REGISTER(wch_hci, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);

#include <wch_hci_common.h>

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

static uint8_t bt_wch_memory_buf[CONFIG_BT_WCH_MEM_POOL_SIZE];
static K_KERNEL_STACK_DEFINE(wch_bt_main_stack, CONFIG_BT_WCH_STACK_SIZE);
static struct k_thread wch_main_thread_data;

static wch_bt_host_callback_t host_callback;

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
		read_tx_power_level(cmd, evt);
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
	rp->lmp_subversion = sys_cpu_to_le16(0x0002);  //TODO
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

#if defined(CONFIG_BT_CTLR_CONN_ISO)
	/* Read/Write Connection Accept Timeout */
	rp->commands[7] |= BIT(2) | BIT(3);
#endif /* CONFIG_BT_CTLR_CONN_ISO */

	/* Read TX Power Level. */
	rp->commands[10] |= BIT(2);

#if defined(CONFIG_BT_HCI_ACL_FLOW_CONTROL)
	/* Set FC, Host Buffer Size and Host Num Completed */
	rp->commands[10] |= BIT(5) | BIT(6) | BIT(7);
#endif /* CONFIG_BT_HCI_ACL_FLOW_CONTROL */

	/* Read Local Version Info, Read Local Supported Features. */
	rp->commands[14] |= BIT(3) | BIT(5);
	/* Read BD ADDR. */
	rp->commands[15] |= BIT(1);

#if defined(CONFIG_BT_CTLR_CONN_RSSI)
	/* Read RSSI. */
	rp->commands[15] |= BIT(5);
#endif /* CONFIG_BT_CTLR_CONN_RSSI */

	/* Set Event Mask Page 2 */
	rp->commands[22] |= BIT(2);
	/* LE Set Event Mask, LE Read Buffer Size, LE Read Local Supp Feats,
	 * Set Random Addr
	 */
	rp->commands[25] |= BIT(0) | BIT(1) | BIT(2) | BIT(4);

#if defined(CONFIG_BT_CTLR_FILTER_ACCEPT_LIST)
	/* LE Read FAL Size, LE Clear FAL */
	rp->commands[26] |= BIT(6) | BIT(7);
	/* LE Add Dev to FAL, LE Remove Dev from FAL */
	rp->commands[27] |= BIT(0) | BIT(1);
#endif /* CONFIG_BT_CTLR_FILTER_ACCEPT_LIST */

	/* LE Encrypt, LE Rand */
	rp->commands[27] |= BIT(6) | BIT(7);
	/* LE Read Supported States */
	rp->commands[28] |= BIT(3);

#if defined(CONFIG_BT_BROADCASTER)
	/* LE Set Adv Params, LE Read Adv Channel TX Power, LE Set Adv Data */
	rp->commands[25] |= BIT(5) | BIT(6) | BIT(7);
	/* LE Set Scan Response Data, LE Set Adv Enable */
	rp->commands[26] |= BIT(0) | BIT(1);

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	/* LE Set Adv Set Random Addr, LE Set Ext Adv Params, LE Set Ext Adv
	 * Data, LE Set Ext Adv Scan Rsp Data, LE Set Ext Adv Enable, LE Read
	 * Max Adv Data Len, LE Read Num Supp Adv Sets
	 */
	rp->commands[36] |= BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) |
			    BIT(6) | BIT(7);
	/* LE Remove Adv Set, LE Clear Adv Sets */
	rp->commands[37] |= BIT(0) | BIT(1);
#if defined(CONFIG_BT_CTLR_ADV_PERIODIC)
	/* LE Set PA Params, LE Set PA Data, LE Set PA Enable */
	rp->commands[37] |= BIT(2) | BIT(3) | BIT(4);
#if defined(CONFIG_BT_CTLR_ADV_ISO)
	/* LE Create BIG, LE Create BIG Test, LE Terminate BIG */
	rp->commands[42] |= BIT(5) | BIT(6) | BIT(7);
#endif /* CONFIG_BT_CTLR_ADV_ISO */
#endif /* CONFIG_BT_CTLR_ADV_PERIODIC */
#endif /* CONFIG_BT_CTLR_ADV_EXT */
#endif /* CONFIG_BT_BROADCASTER */

#if defined(CONFIG_BT_OBSERVER)
	/* LE Set Scan Params, LE Set Scan Enable */
	rp->commands[26] |= BIT(2) | BIT(3);

#if defined(CONFIG_BT_CTLR_ADV_EXT)
	/* LE Set Extended Scan Params, LE Set Extended Scan Enable */
	rp->commands[37] |= BIT(5) | BIT(6);
#if defined(CONFIG_BT_CTLR_SYNC_PERIODIC)
	/* LE PA Create Sync, LE PA Create Sync Cancel, LE PA Terminate Sync */
	rp->commands[38] |= BIT(0) | BIT(1) | BIT(2);
#if defined(CONFIG_BT_CTLR_SYNC_PERIODIC_ADV_LIST)
	/* LE PA Add Device to Periodic Advertiser List,
	 * LE PA Remove Device from Periodic Advertiser List,
	 * LE Clear Periodic Advertiser List,
	 * LE Read Periodic Adveritiser List Size
	 */
	rp->commands[38] |= BIT(3) | BIT(4) | BIT(5) | BIT(6);
#endif /* CONFIG_BT_CTLR_SYNC_PERIODIC_ADV_LIST */
#if defined(CONFIG_BT_CTLR_SYNC_PERIODIC)
	/* LE Set PA Receive Enable */
	rp->commands[40] |= BIT(5);
#endif /* CONFIG_BT_CTLR_SYNC_PERIODIC */
#if defined(CONFIG_BT_CTLR_SYNC_ISO)
	/* LE BIG Create Sync, LE BIG Terminate Sync */
	rp->commands[43] |= BIT(0) | BIT(1);
#endif /* CONFIG_BT_CTLR_SYNC_ISO */
#endif /* CONFIG_BT_CTLR_SYNC_PERIODIC */
#endif /* CONFIG_BT_CTLR_ADV_EXT */

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

#if defined(CONFIG_BT_CTLR_DTM_HCI)
	/* LE RX Test, LE TX Test, LE Test End */
	rp->commands[28] |= BIT(4) | BIT(5) | BIT(6);
	/* LE Enhanced RX Test. */
	rp->commands[35] |= BIT(7);
	/* LE Enhanced TX Test. */
	rp->commands[36] |= BIT(0);
#if defined(CONFIG_BT_CTLR_DTM_HCI_RX_V3)
	rp->commands[39] |= BIT(3);
#endif /* CONFIG_BT_CTLR_DTM_HCI_RX_V3 */

#if defined(CONFIG_BT_CTLR_DTM_HCI_TX_V3)
	rp->commands[39] |= BIT(4);
#endif

#if defined(CONFIG_BT_CTLR_DTM_HCI_TX_V4)
	rp->commands[45] |= BIT(0);
#endif
#endif /* CONFIG_BT_CTLR_DTM_HCI */

#if defined(CONFIG_BT_CTLR_PRIVACY)
	/* LE resolving list commands, LE Read Peer RPA */
	rp->commands[34] |= BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7);
	/* LE Read Local RPA, LE Set AR Enable, Set RPA Timeout */
	rp->commands[35] |= BIT(0) | BIT(1) | BIT(2);
	/* LE Set Privacy Mode */
	rp->commands[39] |= BIT(2);
#endif /* CONFIG_BT_CTLR_PRIVACY */


#if defined(CONFIG_BT_HCI_RAW) && defined(CONFIG_BT_TINYCRYPT_ECC)
	bt_hci_ecc_supported_commands(rp->commands);
#endif /* CONFIG_BT_HCI_RAW && CONFIG_BT_TINYCRYPT_ECC */

	/* LE Read TX Power. */
	rp->commands[38] |= BIT(7);

#if defined(CONFIG_BT_CTLR_ADV_ISO) || defined(CONFIG_BT_CTLR_CONN_ISO)
	/* LE Read Buffer Size v2, LE Read ISO TX Sync */
	rp->commands[41] |= BIT(5) | BIT(6);
	/* LE ISO Transmit Test */
	rp->commands[43] |= BIT(5);
#endif /* CONFIG_BT_CTLR_ADV_ISO || CONFIG_BT_CTLR_CONN_ISO */



#if defined(CONFIG_BT_CTLR_SET_HOST_FEATURE)
	/* LE Set Host Feature */
	rp->commands[44] |= BIT(1);
#endif /* CONFIG_BT_CTLR_SET_HOST_FEATURE */


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

	// case BT_OCF(BT_HCI_OP_LE_ENCRYPT):
	// 	break;

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
	case BT_OCF(BT_HCI_OP_LE_CREATE_CONN):
		le_create_connection(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_LE_CREATE_CONN_CANCEL):
		le_create_conn_cancel(cmd, evt, node_rx);
		break;

	case BT_OCF(BT_HCI_OP_LE_SET_HOST_CHAN_CLASSIF):
		le_set_host_chan_classif(cmd, evt);
		break;
#endif /* CONFIG_BT_CENTRAL */

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

#define SLEEP_RTC_MIN_TIME                  k_us_to_ticks_ceil32(3000)
#define SLEEP_RTC_MAX_TIME                	\
				(	(0xa8c00000U) - \
					k_ms_to_ticks_ceil32(MSEC_PER_SEC * SEC_PER_MIN * MIN_PER_HOUR))

__ramfunc static uint32_t wch_bt_idle(uint32_t time)
{

	return 0;

	uint32_t current_time, sleep_time;
    
	current_time = RTC_GetCycle32k();

    if (time < current_time) {
        sleep_time = time + (0xa8c00000U - current_time);
    } else {
        sleep_time = time - current_time;
    }

    if ((sleep_time < SLEEP_RTC_MIN_TIME) ||
        (sleep_time > SLEEP_RTC_MAX_TIME)) {

		// LOG_DBG("error time______\n");
		// LOG_DBG("sleep time: %#x\n", sleep_time);
		// LOG_DBG("current_time: %d\n", current_time);
		// LOG_DBG("time: %d\n", time);
		// LOG_DBG("PASS: %d", (int) (current_time - time));
		// printf("%d\n", (int) (current_time - time));
        return 2;
    }

	// LOG_DBG("slp %d", sleep_time);
	// printk("slp %d\n", sleep_time);
	// printf("*\n");

    // k_sleep(Z_TIMEOUT_TICKS(sleep_time/*  - 200 */));
    k_sleep(Z_TIMEOUT_TICKS(sleep_time));


    return 0;
}


static int wch_ble_lib_init(void)
{
    bleConfig_t cfg;
    if(tmos_memcmp(VER_LIB, VER_FILE, strlen(VER_FILE)) == FALSE)
    {
        LOG_ERR("head file error...\n");
        return -1;
    }

    tmos_memset(&cfg, 0, sizeof(bleConfig_t));
    cfg.MEMAddr = (uint32_t) bt_wch_memory_buf;
    cfg.MEMLen = (uint32_t) sizeof(bt_wch_memory_buf);
    cfg.BufMaxLen = 
            MAX(CONFIG_BT_BUF_ACL_TX_SIZE, CONFIG_BT_BUF_ACL_RX_SIZE);
    cfg.BufNumber = CONFIG_BT_BUF_ACL_RX_COUNT + CONFIG_BT_BUF_ACL_TX_COUNT;
#if defined (CONFIG_BT_CONN)
    cfg.TxNumEvent = CONFIG_BT_CONN_TX_MAX;
    cfg.ConnectNumber = CONFIG_BT_MAX_CONN & BIT_MASK(2);
#endif /* CONFIG_BT_CONN */
    cfg.TxPower = (uint32_t)LL_TX_POWEER_0_DBM; //TODO: 
#if(defined(BLE_SNV)) && (BLE_SNV == TRUE)
    if((BLE_SNV_ADDR + BLE_SNV_BLOCK * BLE_SNV_NUM) > (0x78000 - FLASH_ROM_MAX_SIZE))
    {
        PRINT("SNV config error...\n");
        while(1);
    }
    cfg.SNVAddr = (uint32_t)BLE_SNV_ADDR;
    cfg.SNVBlock = (uint32_t)BLE_SNV_BLOCK;
    cfg.SNVNum = (uint32_t)BLE_SNV_NUM;
    cfg.readFlashCB = Lib_Read_Flash;
    cfg.writeFlashCB = Lib_Write_Flash;
#endif
// #if(CLK_OSC32K)
    cfg.SelRTCClock = 1; //TODO: 
    cfg.SelRTCClock |= 0x80;
// #endif
    cfg.srandCB = sys_rand32_get;
#if(defined TEM_SAMPLE) && (TEM_SAMPLE == TRUE)
    cfg.tsCB = HAL_GetInterTempValue; // 根据温度变化校准RF和内部RC( 大于7摄氏度 )
  #if(CLK_OSC32K)
    cfg.rcCB = Lib_Calibration_LSI; // 内部32K时钟校准
  #endif
#endif
// #if(defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    cfg.WakeUpTime = 0;//k_us_to_ticks_ceil32(1400);
    cfg.sleepCB = wch_bt_idle;
// #endif
#if(defined(BLE_MAC)) && (BLE_MAC == TRUE)
    for(i = 0; i < 6; i++)
    {
        cfg.MacAddr[i] = MacAddr[5 - i];
    }
#else
    {
        uint8_t MacAddr[6];
        GetMACAddress(MacAddr);
        for(int i = 0; i < 6; i++)
        {
            cfg.MacAddr[i] = MacAddr[i]; // 使用芯片mac地址
        }
    }
#endif
    if(!cfg.MEMAddr || cfg.MEMLen < 4 * 1024)
    {
        return -1;
    }
    uint8_t ret = BLE_LibInit(&cfg);
    if(ret)
    {
        LOG_ERR("LIB init error code: %x ...\n", ret);
        return -1;
    }

    return TMOS_TimerInit(0);
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
		ret = wch_acl_handle(buf);
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


void wch_bt_host_callback_register(wch_bt_host_callback_t *wch_host_cb)
{
    host_callback.host_send_ready = wch_host_cb->host_send_ready;
    host_callback.host_rcv_pkt = wch_host_cb->host_rcv_pkt;
}

void wch_bt_host_rcv_pkt(struct net_buf *buf)
{
    if (host_callback.host_rcv_pkt)
        host_callback.host_rcv_pkt(buf);
}

static void bt_wch_main_thread(void *p1, void *p2, void *p3)
{
 	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);   

    while (true) {
        TMOS_SystemProcess();
    	k_sleep(Z_TIMEOUT_TICKS(1));
    }
}

int wch_ble_stack_init(void)
{
    int err;

    err = wch_ble_lib_init();

    if (err) {
        return err;
    }

	printf("ble stack init success\r\n");

#if defined(CONFIG_BT_OBSERVER)
    GAPRole_ObserverInit();
#endif /* CONFIG_BT_OBSERVER */

#if defined(CONFIG_BT_BROADCASTER)
    GAPRole_BroadcasterInit();
#endif /* CONFIG_BT_BROADCASTER */

#if defined(CONFIG_BT_PERIPHERAL)
    GAPRole_PeripheralInit();
#endif /* CONFIG_BT_PERIPHERAL */

#if defined(CONFIG_BT_CENTRAL)
    GAPRole_CentralInit();
#endif /* CONFIG_BT_CENTRAL */

#if defined(CONFIG_BT_OBSERVER)
    wch_observer_init();
#endif /* CONFIG_BT_OBSERVER */

#if defined(CONFIG_BT_BROADCASTER)
	wch_broadcaster_init();
#endif /* CONFIG_BT_BROADCASTER */

#if defined(CONFIG_BT_PERIPHERAL)

#endif /* CONFIG_BT_PERIPHERAL */

#if defined(CONFIG_BT_CENTRAL)
#endif /* CONFIG_BT_CENTRAL */

	// extern void BB_IRQHandler(void);
	// extern void LLE_IRQHandler(void);


	IRQ_CONNECT(BLEB_IRQn, 7, BB_IRQLibHandler, 0, 0);
	IRQ_CONNECT(BLEL_IRQn, 6, LLE_IRQLibHandler, 0, 0);

    k_thread_create(&wch_main_thread_data, wch_bt_main_stack, 
            K_KERNEL_STACK_SIZEOF(wch_bt_main_stack),
            bt_wch_main_thread, NULL, NULL, NULL,
            K_PRIO_COOP(CONFIG_BT_DRIVER_RX_HIGH_PRIO),
			0, K_NO_WAIT);
	k_thread_name_set(&wch_main_thread_data, "BT WCH mian");

    return 0;
}
