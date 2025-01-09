/*
 * Copyright (c) 2024 zerosensei
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/random/random.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wch_init, CONFIG_BT_HCI_DRIVER_LOG_LEVEL);

#include "wch_hci_common.h"

#if !DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(clk32k))
#error "32K clock is required in ble"
#endif

#if !DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(pll))
#error "PLL is required in ble"
#endif

#define DT_CLK32K_SCR_NODE \
    DT_PHANDLE_BY_IDX(DT_NODELABEL(clk32k), clock_source, 0)

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(lsi)) && \
        DT_SAME_NODE(DT_CLK32K_SCR_NODE, DT_NODELABEL(lsi))
#define CLK_32K_SRC_LSI_ENABLE 1
#elif DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(lse)) && \
        DT_SAME_NODE(DT_CLK32K_SCR_NODE, DT_NODELABEL(lse))
#define CLK_32K_SRC_LSE_ENABLE 1
#else
#error "32K clock source is not supported"
#endif

#define DT_CLK32K_FREQ()        \
    DT_PROP(DT_CLK32K_SCR_NODE, clock_frequency)

#define RTC_TO_US(x)      \
    (DT_CLK32K_FREQ() == 32768)? \
        ((x) * 30) : \
        ((x) * 31)  

#define SLEEP_RTC_MIN_TIME                  (32)
#define SLEEP_RTC_MAX_TIME                	(0xa8bfdb00)

__attribute__((aligned(4))) static uint8_t bt_wch_memory_buf[CONFIG_BT_WCH_MEM_POOL_SIZE];
static K_KERNEL_STACK_DEFINE(wch_bt_main_stack, CONFIG_BT_WCH_STACK_SIZE);
static struct k_thread wch_main_thread_data;
K_SEM_DEFINE(wch_bt_sem, 0, 1);
static wch_bt_host_callback_t host_callback;

#if CLK_32K_SRC_LSI_ENABLE
static uint8_t get_32k_source(void)
{
    uint8_t ret = 1;
#if DT_SAME_NODE(DT_CLK32K_SCR_NODE, DT_NODELABEL(lsi))
    if (DT_CLK32K_FREQ() == 32000) {
        ret = 1;
    } else {
        ret = 2;
    }
#endif

#if DT_SAME_NODE(DT_CLK32K_SCR_NODE, DT_NODELABEL(lse))
    ret = 0;
#endif
    return ret;
}

static void Lib_Calibration_LSI(void)
{
    Calibration_LSI(Level_64);
}

#endif /* CLK_32K_SRC_LSI_ENABLE */

static uint16_t HAL_GetInterTempValue(void)
{
    uint8_t  sensor, channel, config, tkey_cfg;
    uint16_t adc_data;

    tkey_cfg = R8_TKEY_CFG;
    sensor = R8_TEM_SENSOR;
    channel = R8_ADC_CHANNEL;
    config = R8_ADC_CFG;
    ADC_InterTSSampInit();
    R8_ADC_CONVERT |= RB_ADC_START;
    while(R8_ADC_CONVERT & RB_ADC_START);
    adc_data = R16_ADC_DATA;
    R8_TEM_SENSOR = sensor;
    R8_ADC_CHANNEL = channel;
    R8_ADC_CFG = config;
    R8_TKEY_CFG = tkey_cfg;
    return (adc_data);
}

uint32_t Lib_Read_Flash(uint32_t addr, uint32_t num, uint32_t *pBuf)
{
    EEPROM_READ(addr, pBuf, num * 4);
    return 0;
}

uint32_t Lib_Write_Flash(uint32_t addr, uint32_t num, uint32_t *pBuf)
{
    EEPROM_ERASE(addr, num * 4);
    EEPROM_WRITE(addr, pBuf, num * 4);
    return 0;
}


void wch_bt_idle_clear(void)
{
    k_sem_give(&wch_bt_sem);
}

__ramfunc static uint32_t wch_bt_idle(uint32_t time)
{
    static struct k_spinlock idle_lock;
	uint32_t current_time, sleep_time;
	k_spinlock_key_t key = k_spin_lock(&idle_lock);

    // time = time - 1;
	current_time = RTC_GetCycle32k();

    if (time < current_time) {
        sleep_time = time + (0xa8c00000U - current_time);
    } 
    else 
    {
        sleep_time = time - current_time;
    }

    if ((sleep_time < SLEEP_RTC_MIN_TIME) ||
        (sleep_time > SLEEP_RTC_MAX_TIME)) {

        return 2;
    }

    k_spin_unlock(&idle_lock, key);

    k_sem_take(&wch_bt_sem, Z_TIMEOUT_US(RTC_TO_US(sleep_time)));

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
    cfg.BufMaxLen = MAX(CONFIG_BT_BUF_ACL_TX_SIZE, CONFIG_BT_BUF_ACL_RX_SIZE);
    cfg.BufNumber = CONFIG_BT_BUF_ACL_RX_COUNT + CONFIG_BT_BUF_ACL_TX_COUNT;
    cfg.TxNumEvent = 1;  //TODO: 
    cfg.ConnectNumber = 1 | (1 << 2);//TODO: BT_MAX_CONN
    cfg.TxPower = (uint32_t)LL_TX_POWEER_0_DBM; //TODO: 
#if(defined(BLE_SNV)) && (BLE_SNV == TRUE)
    // if((BLE_SNV_ADDR + BLE_SNV_BLOCK * BLE_SNV_NUM) >
    //          (0x78000 - FLASH_ROM_MAX_SIZE))
    // {
    //     LOG_ERR("SNV config error...\n");
    //     while(1);
    // }
    cfg.SNVAddr = (uint32_t)(0x77E00-0x070000);
    cfg.SNVBlock = (uint32_t)(256);
    cfg.SNVNum = (uint32_t)(1);
    cfg.readFlashCB = Lib_Read_Flash;
    cfg.writeFlashCB = Lib_Write_Flash;
#endif
#if CLK_32K_SRC_LSI_ENABLE
    cfg.SelRTCClock = get_32k_source();
    cfg.rcCB = Lib_Calibration_LSI;
#endif
    cfg.srandCB = sys_rand32_get;
    cfg.tsCB = HAL_GetInterTempValue;
    // cfg.WakeUpTime = 1;
    // cfg.sleepCB = wch_bt_idle;
    {
        uint8_t MacAddr[6];
        GetMACAddress(MacAddr);
        for(int i = 0; i < 6; i++)
        {
            cfg.MacAddr[i] = MacAddr[i];
        }
    }
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

	IRQ_CONNECT(BLEB_IRQn, 7, BB_IRQLibHandler, 0, 0);
	IRQ_CONNECT(BLEL_IRQn, 6, LLE_IRQLibHandler, 0, 0);

    RTC_InitTime(2020, 1, 1, 0, 0, 0);
    TMOS_TimerInit(0);

    return 0;
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


static __ramfunc void bt_wch_main_thread(void *p1, void *p2, void *p3)
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
        LOG_ERR("ble stack init error: %d", err);
        return err;
    }
#if defined(CONFIG_BT_BROADCASTER) || defined(CONFIG_BT_PERIPHERAL)
    err = GAPRole_PeripheralInit();
    if (err) {
        LOG_ERR("peripheral role init error: %d", err);
    }
#endif

#if defined(CONFIG_BT_OBSERVER) || defined(CONFIG_BT_CENTRAL)
    err = GAPRole_CentralInit();
    if (err) {
        LOG_ERR("central role init error: %d", err);
    }
#endif 

#if defined(CONFIG_BT_BROADCASTER) || defined(CONFIG_BT_PERIPHERAL)
    err = wch_bt_peripheral_init();
    if (err) {
        LOG_ERR("peripheral init error: %d", err);
        return err;
    }
#endif

#if defined(CONFIG_BT_OBSERVER) || defined(CONFIG_BT_CENTRAL)
    err = wch_bt_central_init();
    if (err) {
        LOG_ERR("central init error: %d", err);
        return err;
    }
#endif 

    k_thread_create(&wch_main_thread_data, wch_bt_main_stack, 
            K_KERNEL_STACK_SIZEOF(wch_bt_main_stack),
            bt_wch_main_thread, NULL, NULL, NULL,
            K_PRIO_COOP(CONFIG_BT_DRIVER_RX_HIGH_PRIO),
			0, K_NO_WAIT);
	k_thread_name_set(&wch_main_thread_data, "BT WCH mian");

	LOG_DBG("ble stack init success");

    return 0;
}

int wch_ble_stack_deinit(void)
{
    host_callback.host_send_ready = NULL;
    host_callback.host_rcv_pkt = NULL;

	/* Abort prio RX thread */
	k_thread_abort(&wch_main_thread_data);

    return 0;
}