
/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include "cmsis.h"
#include "hal_dma.h"
#include "hal_iomux.h"
#include "hal_norflash.h"
#include "hal_sysfreq.h"
#include "hal_timer.h"
#include "hal_trace.h"
#include "hal_sec.h"
#include "hwtimer_list.h"
#include "main_entry.h"
#include "pmu.h"
#ifdef RTOS
#include "rtx_os.h"
#endif
#include "tool_msg.h"
#include "norflash_cfg.h"
#include <arm_cmse.h>
#include "cmse/tz_trace_s.h"

#include "partition_ARMCM33.h"
#include "mpc.h"

#ifdef MBEDTLS_TEST
#include "hwtest.h"
#endif

#ifdef RTOS
extern "C" void rtx_show_all_threads(void);
#endif
#ifdef USER_SECURE_BOOT
extern  "C" int user_secure_boot_check(void);
#endif

#if defined(SE_OTP_DEMO_TEST)
#include "cmse/tz_otp_handler_demo.h"
#endif

#if defined(SPA_AUDIO_SEC)
#include "parser.h"
extern "C"     void rsa_test(void);
#endif

#define NS_CALL                             __attribute__((cmse_nonsecure_call))

/* typedef for non-secure callback functions */
typedef void (*funcptr_void) (void) NS_CALL;

// GDB can set a breakpoint on the main function only if it is
// declared as below, when linking with STD libraries.

int MAIN_ENTRY(void)
{
    int POSSIBLY_UNUSED ret;
    uint32_t ns_app_start_addr = (FLASHX_BASE + NS_APP_START_OFFSET);

    hal_audma_open();
    hal_gpdma_open();
#ifdef DEBUG
    cmse_trace_init();
#if (DEBUG_PORT == 3)
    hal_iomux_set_analog_i2c();
    hal_iomux_set_uart2();
    hal_trace_open(HAL_TRACE_TRANSPORT_UART2);
#elif (DEBUG_PORT == 2)
    hal_iomux_set_analog_i2c();
    hal_iomux_set_uart1();
    hal_trace_open(HAL_TRACE_TRANSPORT_UART1);
#else
    hal_iomux_set_uart0();
    hal_trace_open(HAL_TRACE_TRANSPORT_UART0);
#endif
#endif

#ifdef RTOS
#if !(defined(ROM_BUILD) || defined(PROGRAMMER))
    ret = hal_trace_crash_dump_register(HAL_TRACE_CRASH_DUMP_MODULE_SYS, rtx_show_all_threads);
    ASSERT(ret == 0, "IdleTask: Failed to register crash dump callback");
#endif
#endif

#if !defined(SIMU) && !defined(FPGA)
    hal_norflash_show_id_state(HAL_FLASH_ID_0, true);
#endif

#ifndef NO_PMU
#if 0
    ret = pmu_open();
#endif
    ASSERT(ret == 0, "Failed to open pmu");
#endif

    hal_sysfreq_req(HAL_SYSFREQ_USER_INIT, HAL_CMU_FREQ_208M);
    TR_INFO(TR_MOD(MAIN), "CPU freq: %u", hal_sys_timer_calc_cpu_freq(5, 0));

#ifndef  CMSE_RAM_RAMX_LEND_NSE
    hal_sec_init();

    ret = mpc_init();
    ASSERT(ret==0, "mpc init fail. ret:%d", ret);

    TZ_SAU_Setup();
    __ISB();
#endif


    //hal_page_spy_set(SPY_SRAM0_BASE, 0, 0x20001000, 0x1000, HAL_PAGE_SPY_FLAG_READ);

#ifdef RTOS
    osKernelSuspend();
#endif

#ifdef SE_OTP_DEMO_TEST
    se_otp_demo_test();
#endif

#ifdef MBEDTLS_TEST
    mbedtls_test();
#endif

#ifdef CMSE_RAM_RAMX_LEND_NSE
    TRACE(0,"CMSE_RAM_RAMX_LEND_NSE start");

    cmse_flash_load_into_sram_if((unsigned int )__cmse_ramx_lend_text_section_start_flash__,(unsigned int )__cmse_ramx_lend_text_section_end_flash__,
                                (unsigned int )__cmse_ramx_lend_text_section_ram_start__,(unsigned int )__cmse_ramx_lend_text_section_ram_end__);
    cmse_flash_load_into_sram_if((unsigned int )__cmse_ram_lend_data_section_start_flash__,(unsigned int )__cmse_ram_ramx_data_text_end_flash__,
                                (unsigned int )__cmse_ram_lend_data_section_exec_start__,(unsigned int )__cmse_ram_lend_data_section_exec_end__);

    TRACE(0,"CMSE_RAM_RAMX_LEND_NSE finished");
//    DUMP8("%02x ",__cmse_ramx_lend_text_section_ram_start__,16);
//    DUMP8("%02x ",__cmse_ram_lend_data_section_exec_start__,16);
#endif

#ifdef CMSE_CRYPT_TEST_DEMO
    tz_customer_load_ram_ramx_section_info_t info = {0};
    info.flash_addr = (unsigned int )inc_enc_bin_sec_start;
    info.dst_text_sramx_addr_start = (unsigned int )__customer_load_sram_text_start__;
    info.dst_text_sramx_addr_end = (unsigned int )__customer_load_sram_text_end__;
    info.dst_data_sram_addr_start = (unsigned int )__customer_load_ram_data_start__;
    info.dst_data_sram_addr_end = (unsigned int )__customer_load_ram_data_end__;
    if(sec_decrypted_section_load_init((void *)&info) == 0){
        rsa_test();
    }
#endif

    hal_sysfreq_req(HAL_SYSFREQ_USER_INIT, HAL_CMU_FREQ_32K);
    TR_INFO(TR_MOD(MAIN), "Release CPU freq for sleep");

    funcptr_void NonSecure_ResetHandler;

    /* Add user setup code for secure part here*/

    /* Set non-secure main stack (MSP_NS) */
    //__TZ_set_MSP_NS(*((uint32_t *)(ns_app_start_addr)));

    ///TODO: check SECURE_BOOT
    /* Get non-secure reset handler */
#ifdef USER_SECURE_BOOT
    if(user_secure_boot_check() !=0){
        ASSERT(0,"security check fail");
        return 0;
    }
#endif
#ifdef CMSE_RAM_RAMX_LEND_NSE
    hal_sec_init();

    ret = mpc_init();
    ASSERT(ret==0, "mpc init fail. ret:%d", ret);

    TZ_SAU_Setup();
    __ISB();
#endif

#ifdef SECURE_BOOT
    NonSecure_ResetHandler = (funcptr_void)((uint32_t)ns_app_start_addr + sizeof(struct boot_struct_t) + sizeof(struct code_sig_struct_t) +
                                            sizeof(struct norflash_cfg_struct_t) + sizeof(struct code_ver_struct_t));
#else
    NonSecure_ResetHandler = (funcptr_void)((uint32_t)(&((struct boot_struct_t *)ns_app_start_addr)->hdr + 1));
#endif
    if (*(uint32_t *)ns_app_start_addr != BOOT_MAGIC_NUMBER) {
        TRACE(0, "nonsec image(0x%08x) magic(0x%08x) error", ns_app_start_addr, *(uint32_t *)ns_app_start_addr);
    } else {
        TRACE(0, "Call nonsec App. start addr:0x%08x", ns_app_start_addr);
        /* Start non-secure state software application */
#ifdef DEBUG
        cmse_set_ns_start_flag(true);
#endif
        NonSecure_ResetHandler();
    }
    /* Non-secure software does not return, this code is not executed */
    SAFE_PROGRAM_STOP();
    return 0;
}

/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include "cmsis.h"
#include "analog.h"
#include "hal_bootmode.h"
#include "hal_cmu.h"
#include "hal_dma.h"
#include "hal_iomux.h"
#include "hal_key.h"
#include "hal_norflash.h"
#include "hal_sleep.h"
#include "hal_sysfreq.h"
#include "hal_timer.h"
#include "hal_trace.h"
#include "hwtimer_list.h"
#include "main_entry.h"
#include "mpu_cfg.h"
#include "pmu.h"
#include "hal_evr.h"

#ifdef SENSOR_TEST
#include "sensor_test.h"
#endif

#ifdef RTOS
#include "cmsis_os.h"
#ifdef KERNEL_RTX
#include "rt_Time.h"
#endif
#endif

#ifdef HWTEST
#include "hwtest.h"
#ifdef VD_TEST
#include "voice_detector.h"
#endif
#endif

#ifdef DSP_TEST
#include "dsp_loader.h"
#endif

#ifdef LIS25BA_TEST
#include "sensor_test.h"
#endif

#ifdef I2S_HOST_TEST
#include "i2s_host.h"
#endif

#ifdef CHIP_FUNCTION_TEST
#include "chip_function_test.h"
#endif

#ifdef SLEEP_SIMU
#define TIMER_IRQ_PERIOD_MS             3
#define DELAY_PERIOD_MS                 4
#elif defined(SLEEP_TEST) && (SLEEP_TEST)
#define TIMER_IRQ_PERIOD_MS             3000
#define DELAY_PERIOD_MS                 4000
#else
#define TIMER_IRQ_PERIOD_MS             1000
#define DELAY_PERIOD_MS                 2000
#endif

#ifndef FLASH_FILL
#define FLASH_FILL                      1
#endif

#if defined(KERNEL_RTX) || defined(KERNEL_RTX5)
#define OS_TIME_STR                     "[%2u/%u]"
#define OS_CUR_TIME                     , SysTick->VAL, osKernelGetTickCount()
#else
#define OS_TIME_STR
#define OS_CUR_TIME
#endif

#if defined(MS_TIME)
#define TIME_STR                        "[%u]" OS_TIME_STR
#define CUR_TIME                        TICKS_TO_MS(hal_sys_timer_get())  OS_CUR_TIME
#elif defined(RAW_TIME)
#define TIME_STR                        "[0x%X]" OS_TIME_STR
#define CUR_TIME                        hal_sys_timer_get()  OS_CUR_TIME
#else
#define TIME_STR                        "[%u/0x%X]" OS_TIME_STR
#define CUR_TIME                        TICKS_TO_MS(hal_sys_timer_get()), hal_sys_timer_get() OS_CUR_TIME
#endif

#ifdef DEBUG_TIME
#define TR_INFO_TIME(attr, str, ...)    TRACE(attr, TIME_STR " " str, CUR_TIME, ##__VA_ARGS__)
#else
#define TR_INFO_TIME(attr, str, ...)    TRACE(attr, str, ##__VA_ARGS__)
#endif

const static unsigned char bytes[FLASH_FILL] = { 0x1, };

#if defined(DEBUG) && (DEBUG_PORT == 1)
static bool i2c_flag = false;

static void switch_between_i2c_and_uart(bool to_i2c, bool flush_trace)
{
    uint32_t lock;

    lock = int_lock();

    if (to_i2c) {
        TR_INFO(TR_MOD(MAIN), "Switch to I2C");
        if (flush_trace) {
            TRACE_FLUSH();
        }
        while (hal_trace_busy());
        hal_trace_pause();

        hal_iomux_set_analog_i2c();
    } else {
        hal_iomux_set_uart0();

        hal_trace_continue();
        if (flush_trace) {
            TRACE_FLUSH();
        }
        TR_INFO(TR_MOD(MAIN), "Switch to UART0");
    }

    int_unlock(lock);
}
#endif

static HWTIMER_CALLBACK_T timer_cb[3];

#ifdef __cplusplus
extern "C"
#endif
void sleep_test_register_timer_callback(HWTIMER_CALLBACK_T cb)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(timer_cb); i++) {
        if (!timer_cb[i]) {
            timer_cb[i] = cb;
            return;
        }
    }

    ASSERT(false, "%s: failed", __func__);
}

POSSIBLY_UNUSED
static void sleep_test_exec_timer_callback(void *param)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(timer_cb); i++) {
        if (timer_cb[i]) {
            timer_cb[i](param);
        }
    }
}

static void (*loop_cb[3])(void);

#ifdef __cplusplus
extern "C"
#endif
void sleep_test_register_loop_callback(void (*cb)(void))
{
    for (uint32_t i = 0; i < ARRAY_SIZE(loop_cb); i++) {
        if (!loop_cb[i]) {
            loop_cb[i] = cb;
            return;
        }
    }

    ASSERT(false, "%s: failed", __func__);
}

POSSIBLY_UNUSED
static void sleep_test_exec_loop_callback(void)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(loop_cb); i++) {
        if (loop_cb[i]) {
            loop_cb[i]();
        }
    }
}

#ifdef NO_TIMER
#if defined(SLEEP_TEST) && (SLEEP_TEST)
static bool wakeup_flag = false;
#endif
#else
static HWTIMER_ID hw_timer = NULL;
static void timer_handler(void *param)
{
    TR_INFO_TIME(TR_MOD(MAIN), "Timer handler: %u", (uint32_t)param);

    sleep_test_exec_timer_callback(param);

    hwtimer_start(hw_timer, MS_TO_TICKS(TIMER_IRQ_PERIOD_MS));
    TR_INFO_TIME(TR_MOD(MAIN), "Start timer %u ms", TIMER_IRQ_PERIOD_MS);
}
#endif

static int key_event_process(uint32_t key_code, uint8_t key_event)
{
    TR_INFO(TR_MOD(MAIN), "%s: code=0x%X event=%u", __FUNCTION__, key_code, key_event);

    if (key_code == HAL_KEY_CODE_PWR) {
        if (0) {
#if (defined(SLEEP_TEST) && (SLEEP_TEST)) && defined(NO_TIMER)
        } else if (key_event == HAL_KEY_EVENT_CLICK) {
            enum HAL_CMU_FREQ_T sys_freq;

            wakeup_flag = !wakeup_flag;
            if (wakeup_flag) {
                sys_freq = HAL_CMU_FREQ_26M;
            } else {
                sys_freq = HAL_CMU_FREQ_32K;
            }
            hal_sysfreq_req(HAL_SYSFREQ_USER_INIT, sys_freq);

            TR_INFO(TR_MOD(MAIN), "%s: sleep=%d", __FUNCTION__, !wakeup_flag);
#endif
#if defined(DEBUG) && (DEBUG_PORT == 1)
        } else if (key_event == HAL_KEY_EVENT_LONGPRESS) {
            i2c_flag = !i2c_flag;
            switch_between_i2c_and_uart(i2c_flag, true);
#endif
        }
    }

    return 0;
}

// GDB can set a breakpoint on the main function only if it is
// declared as below, when linking with STD libraries.

int MAIN_ENTRY(void)
{
    int POSSIBLY_UNUSED ret;

    hwtimer_init();
    hal_audma_open();
    hal_gpdma_open();
#ifdef DEBUG
#if (DEBUG_PORT == 3)
    hal_iomux_set_analog_i2c();
    hal_iomux_set_uart2();
    hal_trace_open(HAL_TRACE_TRANSPORT_UART2);
#elif (DEBUG_PORT == 2)
    hal_iomux_set_analog_i2c();
    hal_iomux_set_uart1();
    hal_trace_open(HAL_TRACE_TRANSPORT_UART1);
#else
    hal_iomux_set_uart0();
    hal_trace_open(HAL_TRACE_TRANSPORT_UART0);
#endif
#endif

#ifdef EVR_TRACING
    hal_evr_init(0);
#endif

#if !defined(SIMU) && !defined(FPGA) && !defined(BT_CONN_SIMU)
    hal_norflash_show_id_state(HAL_FLASH_ID_0, true);
#endif

    TR_INFO_TIME(TR_MOD(MAIN), "main started: filled@0x%08x", (uint32_t)bytes);

    mpu_cfg();
#ifndef NO_PMU
    ret = pmu_open();
    ASSERT(ret == 0, "Failed to open pmu");
#endif
    analog_open();

#ifdef REBOOT_TEST
    if (!hal_hw_bootmode_get().global) {
        hal_sw_bootmode_set(HAL_SW_BOOTMODE_FLASH_BOOT);
        pmu_reboot();
        ASSERT(false, "%s: Reboot failed", __func__);
    }
#endif

    hal_cmu_simu_pass();

    enum HAL_CMU_FREQ_T sys_freq;

#if defined(SLEEP_TEST) && (SLEEP_TEST)
    sys_freq = HAL_CMU_FREQ_32K;
#elif defined(ULTRA_LOW_POWER)
    sys_freq = HAL_CMU_FREQ_52M;
#else
    sys_freq = HAL_CMU_FREQ_104M;
#endif

    hal_sysfreq_req(HAL_SYSFREQ_USER_INIT, sys_freq);

    TR_INFO(TR_MOD(MAIN), "CPU freq: %u", hal_sys_timer_calc_cpu_freq(5, 0));

#ifdef CHIP_WAKE_TEST
    hal_chip_wake_lock(HAL_CHIP_WAKE_LOCK_USER_31);
#endif

    hal_sleep_start_stats(10000, 10000);

#ifdef HWTEST

#ifdef CMSIS_TEST
    cmsis_test();
#endif

#ifdef SBC_ROM_TEST
    sbc_rom_test();
#endif
#ifdef LC3_TEST
    lc3_test();
#endif

#ifdef SBC_TEST
    sbc_test();
#endif

#ifdef ACCDEC_TEST
    acc_dec_test();
#endif

#ifdef USB_SERIAL_TEST
    pmu_usb_config(PMU_USB_CONFIG_TYPE_DEVICE);
    usb_serial_test();
#endif

#ifdef USB_SERIAL_DIRECT_XFER_TEST
    pmu_usb_config(PMU_USB_CONFIG_TYPE_DEVICE);
    usb_serial_direct_xfer_test();
#endif

#ifdef USB_AUDIO_TEST
    pmu_usb_config(PMU_USB_CONFIG_TYPE_DEVICE);
    usb_audio_test();
#endif

#ifdef I2C_TEST
    i2c_test();
#endif

#ifdef I2C_DEBUG_SLAVE_TEST
    i2c_debug_slave_test();
#endif

#ifdef I2C_SLAVE_TEST
    i2c_slave_test();
#endif

#ifdef AF_TEST
    af_test();
#endif

#ifdef VD_TEST
    voice_detector_test();
#endif

#ifdef CP_TEST
    cp_test();
#endif

#ifdef SEC_ENG_TEST
    sec_eng_test();
#endif

#ifdef TDM_TEST
    tdm_test();
#endif

#ifdef A7_DSP_TEST
    a7_dsp_test();
#endif

#ifdef TRANSQ_TEST
    transq_test();
#endif

#ifdef FLASH_TEST
    flash_test();
#endif

#ifdef NANDFLASH_TEST
    nandflash_test();
#endif

#ifdef PSRAM_TEST
    psram_test();
#endif

#ifdef PSRAM_WINDOW_TEST
    psram_window_test();
#endif

#ifdef PSRAM_DUAL_PORT_TEST
    psram_dual_port_test();
#endif

#ifdef PSRAMUHS_TEST
    psramuhs_test();
#endif

#ifdef PSRAMUHS_WINDOW_TEST
    psramuhs_window_test();
#endif

#ifdef MBW_TEST
    mbw_test();
#endif

#ifdef SPI_NORFLASH_TEST
    spi_norflash_test();
#endif

#ifdef SENSOR_HUB_TEST
    sensor_hub_test();
#endif

#ifdef BT_HOST_TEST
    bt_host_test();
#endif

#ifdef DSP_HIFI4_TEST
    dsp_hifi4_test();
#ifdef HIFI4_ADDA_LOOP_TEST
    adda_loop_start_demo();
#endif
#endif

#ifdef DSP_M55_TEST
    dsp_m55_test();
#endif

#ifdef CODEC_SIMU
    codec_test();
#endif

#ifdef I2S_TEST
    i2s_test();
#endif

#ifdef SENSOR_TEST
    sensor_test();
#endif

#ifdef BECO_TEST
    beco_test();
#endif

#ifdef TZ_TEST
    tz_test();
#endif

#ifdef BT_CONN_TEST
    bt_conn_test();
#endif

#ifdef BT_CONN_SIMU
    bt_conn_simu();
#endif

#ifdef FT_TEST
    ft_test_main();
#endif

#ifdef LIS25BA_TEST
    lis25ba_test();
#endif

#ifdef TRNG_TEST
    trng_test();
#endif

#ifdef PWM_TEST
    pwm_test();
#endif

#ifdef BREATHING_LED_TEST
    breathing_led_test();
#endif

#ifdef IR_TEST
    ir_test();
#endif

#ifdef RSA_TEST
    test_rsa();
#endif

#ifdef DAC_TEST
    dac_test();
#endif

#ifdef ADC2_TEST
    adc2_test();
#endif

#ifdef SDMMC_TEST
    sdmmc_test();
#endif

#ifdef SDEMMC_TEST
    sdemmc_test();
#endif

#ifdef EMMC_TEST
    emmc_test();
#endif

#ifdef SDIO_HOST_TEST
    sdio_host_test();
#endif

#ifdef SDIO_HOST_SIGNAL_TEST
    sdio_host_signal_test();
#endif

#ifdef SDIO_HOST_FIRMWARE_TEST
    sdio_host_firmware_test();
#endif

#ifdef SDIO_HOST_PROGRAMMER_TEST
    sdio_host_programmer_test();
#endif

#ifdef SDIO_DEVICE_TEST
    sdio_device_test();
#endif

#ifdef DSI_TEST
    dsi_test();
#endif

#ifdef CSI_TEST
    csi_test();
#endif

#ifdef QSPI_1501_TEST
    qspi_1501_test();
#endif

#ifdef QSPI_LCDC_TEST
    qspi_lcdc_test();
#endif

#ifdef I2S_HOST_TEST
    i2s_host_test();
#endif

#ifdef SPI_TEST
    spi_test();
#endif

#ifdef SPI2SDIO_TEST
    spi2sdio_test();
#endif

#ifdef SPI_DEBUG_SLAVE_TEST
    spi_debug_slave_test();
#endif

#ifdef UART_TEST
    uart_test();
#endif

#ifdef MAX_POWER_TEST
    max_power_test();
#endif

#ifdef GPIO_TEST
    gpio_test();
#endif

#ifdef GPU_TEST
    gpu_test();
#endif

#ifdef GRAPHIC_TEST
    run_graphic_test_cases();
#endif

#ifdef GPADC_TEST
    gpadc_test();
#endif

#ifdef WDT_TEST
    wdt_test();
#endif

#ifdef HEAP_TEST
    heap_test();
#endif

#ifdef CHARGER_TEST
    charger_test();
#endif

#ifdef COREMARK_TEST
    coremark_test();
#endif

#ifdef DHRYSTONE_TEST
    dhrystone_test();
#endif

#ifdef MBEDTLS_TEST
    mbedtls_test();
#endif

#if !(defined(SLEEP_TEST) && (SLEEP_TEST))
    SAFE_PROGRAM_STOP();
#endif

#endif // HWTEST

    hal_key_open(false, key_event_process);

#if defined(DEBUG) && (DEBUG_PORT == 1)
    TR_INFO(TR_MOD(MAIN), "TIP: Long press PwrKey to switch between I2C and UART0");
#endif

#ifdef CHIP_FUNCTION_TEST
    chip_function_test();
#endif

#ifdef NO_TIMER
#if defined(SLEEP_TEST) && (SLEEP_TEST)
#if (SLEEP_TEST == 2)
    wakeup_flag = true;
    hal_sysfreq_req(HAL_SYSFREQ_USER_INIT, HAL_CMU_FREQ_26M);
    TR_INFO(TR_MOD(MAIN), "Single click PwrKey to sleep ...");
#else
    TR_INFO(TR_MOD(MAIN), "Enter sleep (and single click PwrKey to wakeup) ...");
#endif
#endif
#else
    hw_timer = hwtimer_alloc(timer_handler, 0);
    hwtimer_start(hw_timer, MS_TO_TICKS(TIMER_IRQ_PERIOD_MS));
    TR_INFO(TR_MOD(MAIN), TIME_STR " Start timer %u ms", CUR_TIME, TIMER_IRQ_PERIOD_MS);
#endif

    while (1) {
        sleep_test_exec_loop_callback();
#if (defined(SLEEP_TEST) && (SLEEP_TEST)) && !defined(RTOS)
        hal_sleep_enter_sleep();
#else
#if defined(NO_TIMER) && defined(RTOS)
        osSignalWait(0x0, osWaitForever);
#elif !defined(NO_TIMER)
        osDelay(DELAY_PERIOD_MS);
        TR_INFO(TR_MOD(MAIN), TIME_STR " Delay %u ms done", CUR_TIME, DELAY_PERIOD_MS);
#endif
#endif
    }

    SAFE_PROGRAM_STOP();
    return 0;
}

/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include <string.h>
#include "app_anc.h"
#include "app_anc_sync.h"
#include "hal_trace.h"

#ifdef IBRT
#include "app_tws_ibrt.h"
#include "app_tws_ibrt_cmd_handler.h"
#include "app_ibrt_customif_cmd.h"
#include "app_tws_ctrl_thread.h"
#endif

int32_t app_anc_sync_init(void)
{
    return 0;
}

int32_t app_anc_sync_send(uint8_t *buf, uint32_t len)
{
#if defined(IBRT)
    tws_ctrl_send_cmd(APP_TWS_CMD_SYNC_ANC_STATUS, buf, len);
#endif

    return 0;
}

int32_t app_anc_sync_mode(uint32_t mode)
{
    uint8_t buf[8];

    *((uint32_t *)&buf[0]) = APP_ANC_SYNC_MODE;
    *((uint32_t *)&buf[4]) = mode;

    app_anc_sync_send(buf, sizeof(buf));

    return 0;
}

int32_t app_anc_sync_psap_bands_gain(float gain_l, float gain_r, uint32_t index_start, uint32_t index_end)
{
    uint8_t buf[20];

    *((uint32_t *)&buf[0]) = APP_ANC_SYNC_PSAP_BANDS_GAIN;
    *((float *)&buf[4]) = gain_l;
    *((float *)&buf[8]) = gain_r;
    *((uint32_t *)&buf[12]) = index_start;
    *((uint32_t *)&buf[16]) = index_end;

    app_anc_sync_send(buf, sizeof(buf));

    return 0;
}

int32_t app_anc_sync_psap_bands_gain_array(float *gain)
{
    uint8_t buf[4 + 17 * 4];

    *((uint32_t *)&buf[0]) = APP_ANC_SYNC_PSAP_BANDS_GAIN_ARRAY;
    memcpy(buf + sizeof(uint32_t), gain, 17 * sizeof(float));

    app_anc_sync_send(buf, sizeof(buf));

    return 0;
}

#if defined(PSAP_APP)
extern void psap_set_bands_same_gain_f32(float gain_l, float gain_r, uint32_t index_start, uint32_t index_end);
extern void psap_set_bands_gain_f32(float *gain_l, float *gain_r);
#endif
int32_t app_anc_tws_sync_change(uint8_t *buf, uint32_t len)
{
    app_anc_sync_t sync_type = *((uint32_t *)buf);
    buf += sizeof(uint32_t);

    TRACE(0, "[%s] sync_type: %d, len: %d", __func__, sync_type, len);

    switch (sync_type) {
        case APP_ANC_SYNC_MODE: {
            uint32_t mode = *((uint32_t *)buf);

            TRACE(0, "[%s] sync mode: %d", __func__, mode);
            app_anc_switch_locally(mode);
            break;
        }
#if defined(PSAP_APP)
        case APP_ANC_SYNC_PSAP_BANDS_GAIN: {
            float gain_l;
            float gain_r;
            uint32_t index_start;
            uint32_t index_end;

            gain_l = *((float *)buf);
            buf += sizeof(float);
            gain_r = *((float *)buf);
            buf += sizeof(float);
            index_start = *((uint32_t *)buf);
            buf += sizeof(uint32_t);
            index_end = *((uint32_t *)buf);
            buf += sizeof(uint32_t);

            psap_set_bands_same_gain_f32(gain_l, gain_r, index_start, index_end);
            break;
        }
        case APP_ANC_SYNC_PSAP_BANDS_GAIN_ARRAY: {
            ASSERT(len == 4 + 17 * 4, "[%s] Invalid len: %d", __func__, len);
            psap_set_bands_gain_f32((float *)buf, NULL);
            break;
        }
#endif
        default:
            break;
    }

    return 0;
}
/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include <string.h>
#include "app_anc.h"
#include "app_anc_sync.h"
#include "hal_trace.h"

#ifdef IBRT
#include "app_tws_ibrt.h"
#include "app_tws_ibrt_cmd_handler.h"
#include "app_ibrt_customif_cmd.h"
#include "app_tws_ctrl_thread.h"
#endif

int32_t app_anc_sync_init(void)
{
    return 0;
}

int32_t app_anc_sync_send(uint8_t *buf, uint32_t len)
{
#if defined(IBRT)
    tws_ctrl_send_cmd(APP_TWS_CMD_SYNC_ANC_STATUS, buf, len);
#endif

    return 0;
}

int32_t app_anc_sync_mode(uint32_t mode)
{
    uint8_t buf[8];

    *((uint32_t *)&buf[0]) = APP_ANC_SYNC_MODE;
    *((uint32_t *)&buf[4]) = mode;

    app_anc_sync_send(buf, sizeof(buf));

    return 0;
}

int32_t app_anc_sync_psap_bands_gain(float gain_l, float gain_r, uint32_t index_start, uint32_t index_end)
{
    uint8_t buf[20];

    *((uint32_t *)&buf[0]) = APP_ANC_SYNC_PSAP_BANDS_GAIN;
    *((float *)&buf[4]) = gain_l;
    *((float *)&buf[8]) = gain_r;
    *((uint32_t *)&buf[12]) = index_start;
    *((uint32_t *)&buf[16]) = index_end;

    app_anc_sync_send(buf, sizeof(buf));

    return 0;
}

int32_t app_anc_sync_psap_bands_gain_array(float *gain)
{
    uint8_t buf[4 + 17 * 4];

    *((uint32_t *)&buf[0]) = APP_ANC_SYNC_PSAP_BANDS_GAIN_ARRAY;
    memcpy(buf + sizeof(uint32_t), gain, 17 * sizeof(float));

    app_anc_sync_send(buf, sizeof(buf));

    return 0;
}

#if defined(PSAP_APP)
extern void psap_set_bands_same_gain_f32(float gain_l, float gain_r, uint32_t index_start, uint32_t index_end);
extern void psap_set_bands_gain_f32(float *gain_l, float *gain_r);
#endif
int32_t app_anc_tws_sync_change(uint8_t *buf, uint32_t len)
{
    app_anc_sync_t sync_type = *((uint32_t *)buf);
    buf += sizeof(uint32_t);

    TRACE(0, "[%s] sync_type: %d, len: %d", __func__, sync_type, len);

    switch (sync_type) {
        case APP_ANC_SYNC_MODE: {
            uint32_t mode = *((uint32_t *)buf);

            TRACE(0, "[%s] sync mode: %d", __func__, mode);
            app_anc_switch_locally(mode);
            break;
        }
#if defined(PSAP_APP)
        case APP_ANC_SYNC_PSAP_BANDS_GAIN: {
            float gain_l;
            float gain_r;
            uint32_t index_start;
            uint32_t index_end;

            gain_l = *((float *)buf);
            buf += sizeof(float);
            gain_r = *((float *)buf);
            buf += sizeof(float);
            index_start = *((uint32_t *)buf);
            buf += sizeof(uint32_t);
            index_end = *((uint32_t *)buf);
            buf += sizeof(uint32_t);

            psap_set_bands_same_gain_f32(gain_l, gain_r, index_start, index_end);
            break;
        }
        case APP_ANC_SYNC_PSAP_BANDS_GAIN_ARRAY: {
            ASSERT(len == 4 + 17 * 4, "[%s] Invalid len: %d", __func__, len);
            psap_set_bands_gain_f32((float *)buf, NULL);
            break;
        }
#endif
        default:
            break;
    }

    return 0;
}
/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include <string.h>
#include "app_anc.h"
#include "app_anc_sync.h"
#include "hal_trace.h"

#ifdef IBRT
#include "app_tws_ibrt.h"
#include "app_tws_ibrt_cmd_handler.h"
#include "app_ibrt_customif_cmd.h"
#include "app_tws_ctrl_thread.h"
#endif

int32_t app_anc_sync_init(void)
{
    return 0;
}

int32_t app_anc_sync_send(uint8_t *buf, uint32_t len)
{
#if defined(IBRT)
    tws_ctrl_send_cmd(APP_TWS_CMD_SYNC_ANC_STATUS, buf, len);
#endif

    return 0;
}

int32_t app_anc_sync_mode(uint32_t mode)
{
    uint8_t buf[8];

    *((uint32_t *)&buf[0]) = APP_ANC_SYNC_MODE;
    *((uint32_t *)&buf[4]) = mode;

    app_anc_sync_send(buf, sizeof(buf));

    return 0;
}

int32_t app_anc_sync_psap_bands_gain(float gain_l, float gain_r, uint32_t index_start, uint32_t index_end)
{
    uint8_t buf[20];

    *((uint32_t *)&buf[0]) = APP_ANC_SYNC_PSAP_BANDS_GAIN;
    *((float *)&buf[4]) = gain_l;
    *((float *)&buf[8]) = gain_r;
    *((uint32_t *)&buf[12]) = index_start;
    *((uint32_t *)&buf[16]) = index_end;

    app_anc_sync_send(buf, sizeof(buf));

    return 0;
}

int32_t app_anc_sync_psap_bands_gain_array(float *gain)
{
    uint8_t buf[4 + 17 * 4];

    *((uint32_t *)&buf[0]) = APP_ANC_SYNC_PSAP_BANDS_GAIN_ARRAY;
    memcpy(buf + sizeof(uint32_t), gain, 17 * sizeof(float));

    app_anc_sync_send(buf, sizeof(buf));

    return 0;
}

#if defined(PSAP_APP)
extern void psap_set_bands_same_gain_f32(float gain_l, float gain_r, uint32_t index_start, uint32_t index_end);
extern void psap_set_bands_gain_f32(float *gain_l, float *gain_r);
#endif
int32_t app_anc_tws_sync_change(uint8_t *buf, uint32_t len)
{
    app_anc_sync_t sync_type = *((uint32_t *)buf);
    buf += sizeof(uint32_t);

    TRACE(0, "[%s] sync_type: %d, len: %d", __func__, sync_type, len);

    switch (sync_type) {
        case APP_ANC_SYNC_MODE: {
            uint32_t mode = *((uint32_t *)buf);

            TRACE(0, "[%s] sync mode: %d", __func__, mode);
            app_anc_switch_locally(mode);
            break;
        }
#if defined(PSAP_APP)
        case APP_ANC_SYNC_PSAP_BANDS_GAIN: {
            float gain_l;
            float gain_r;
            uint32_t index_start;
            uint32_t index_end;

            gain_l = *((float *)buf);
            buf += sizeof(float);
            gain_r = *((float *)buf);
            buf += sizeof(float);
            index_start = *((uint32_t *)buf);
            buf += sizeof(uint32_t);
            index_end = *((uint32_t *)buf);
            buf += sizeof(uint32_t);

            psap_set_bands_same_gain_f32(gain_l, gain_r, index_start, index_end);
            break;
        }
        case APP_ANC_SYNC_PSAP_BANDS_GAIN_ARRAY: {
            ASSERT(len == 4 + 17 * 4, "[%s] Invalid len: %d", __func__, len);
            psap_set_bands_gain_f32((float *)buf, NULL);
            break;
        }
#endif
        default:
            break;
    }

    return 0;
}
/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include <string.h>
#include "app_anc.h"
#include "app_anc_sync.h"
#include "hal_trace.h"

#ifdef IBRT
#include "app_tws_ibrt.h"
#include "app_tws_ibrt_cmd_handler.h"
#include "app_ibrt_customif_cmd.h"
#include "app_tws_ctrl_thread.h"
#endif

int32_t app_anc_sync_init(void)
{
    return 0;
}

int32_t app_anc_sync_send(uint8_t *buf, uint32_t len)
{
#if defined(IBRT)
    tws_ctrl_send_cmd(APP_TWS_CMD_SYNC_ANC_STATUS, buf, len);
#endif

    return 0;
}

int32_t app_anc_sync_mode(uint32_t mode)
{
    uint8_t buf[8];

    *((uint32_t *)&buf[0]) = APP_ANC_SYNC_MODE;
    *((uint32_t *)&buf[4]) = mode;

    app_anc_sync_send(buf, sizeof(buf));

    return 0;
}

int32_t app_anc_sync_psap_bands_gain(float gain_l, float gain_r, uint32_t index_start, uint32_t index_end)
{
    uint8_t buf[20];

    *((uint32_t *)&buf[0]) = APP_ANC_SYNC_PSAP_BANDS_GAIN;
    *((float *)&buf[4]) = gain_l;
    *((float *)&buf[8]) = gain_r;
    *((uint32_t *)&buf[12]) = index_start;
    *((uint32_t *)&buf[16]) = index_end;

    app_anc_sync_send(buf, sizeof(buf));

    return 0;
}

int32_t app_anc_sync_psap_bands_gain_array(float *gain)
{
    uint8_t buf[4 + 17 * 4];

    *((uint32_t *)&buf[0]) = APP_ANC_SYNC_PSAP_BANDS_GAIN_ARRAY;
    memcpy(buf + sizeof(uint32_t), gain, 17 * sizeof(float));

    app_anc_sync_send(buf, sizeof(buf));

    return 0;
}

#if defined(PSAP_APP)
extern void psap_set_bands_same_gain_f32(float gain_l, float gain_r, uint32_t index_start, uint32_t index_end);
extern void psap_set_bands_gain_f32(float *gain_l, float *gain_r);
#endif
int32_t app_anc_tws_sync_change(uint8_t *buf, uint32_t len)
{
    app_anc_sync_t sync_type = *((uint32_t *)buf);
    buf += sizeof(uint32_t);

    TRACE(0, "[%s] sync_type: %d, len: %d", __func__, sync_type, len);

    switch (sync_type) {
        case APP_ANC_SYNC_MODE: {
            uint32_t mode = *((uint32_t *)buf);

            TRACE(0, "[%s] sync mode: %d", __func__, mode);
            app_anc_switch_locally(mode);
            break;
        }
#if defined(PSAP_APP)
        case APP_ANC_SYNC_PSAP_BANDS_GAIN: {
            float gain_l;
            float gain_r;
            uint32_t index_start;
            uint32_t index_end;

            gain_l = *((float *)buf);
            buf += sizeof(float);
            gain_r = *((float *)buf);
            buf += sizeof(float);
            index_start = *((uint32_t *)buf);
            buf += sizeof(uint32_t);
            index_end = *((uint32_t *)buf);
            buf += sizeof(uint32_t);

            psap_set_bands_same_gain_f32(gain_l, gain_r, index_start, index_end);
            break;
        }
        case APP_ANC_SYNC_PSAP_BANDS_GAIN_ARRAY: {
            ASSERT(len == 4 + 17 * 4, "[%s] Invalid len: %d", __func__, len);
            psap_set_bands_gain_f32((float *)buf, NULL);
            break;
        }
#endif
        default:
            break;
    }

    return 0;
}
/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include "cmsis_os.h"
#include "tgt_hardware.h"
#include "pmu.h"
#include "hal_timer.h"
#include "hal_gpadc.h"
#include "hal_trace.h"
#include "hal_gpio.h"
#include "hal_iomux.h"
#include "hal_chipid.h"
#include "app_thread.h"
#include "app_battery.h"
#include "audio_policy.h"
#include "app_bt.h"
#include "apps.h"
#include "app_hfp.h"

#ifdef APP_BATTERY_ENABLE
#include "app_status_ind.h"
#include "bluetooth_bt_api.h"
#include "app_media_player.h"
#ifdef BT_USB_AUDIO_DUAL_MODE
#include "btusb_audio.h"
#endif
#include <stdlib.h>

#ifdef __INTERCONNECTION__
#include "bluetooth_ble_api.h"
#endif

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
extern "C" bool app_usbaudio_mode_on(void);
#endif

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
#include CHIP_SPECIFIC_HDR(charger)
#endif

#define APP_BATTERY_TRACE(s,...)
// TRACE(s, ##__VA_ARGS__)

#ifndef APP_BATTERY_GPADC_CH_NUM
#define APP_BATTERY_GPADC_CH_NUM (HAL_GPADC_CHAN_BATTERY)
#endif

#ifndef APP_BATTERY_ERR_MV
#define APP_BATTERY_ERR_MV (2800)
#endif

#ifndef APP_BATTERY_MIN_MV
#define APP_BATTERY_MIN_MV (3200)
#endif

#ifndef APP_BATTERY_MAX_MV
#define APP_BATTERY_MAX_MV (4200)
#endif

#ifndef APP_BATTERY_PD_MV
#define APP_BATTERY_PD_MV   (3100)
#endif

#ifndef APP_BATTERY_CHARGE_TIMEOUT_MIN
#define APP_BATTERY_CHARGE_TIMEOUT_MIN (90)
#endif

#ifndef APP_BATTERY_CHARGE_OFFSET_MV
#define APP_BATTERY_CHARGE_OFFSET_MV (20)
#endif

#ifndef CHARGER_PLUGINOUT_RESET
#define CHARGER_PLUGINOUT_RESET (1)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

#define APP_BATTERY_CHARGING_PLUGOUT_DEDOUNCE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<500?3:1)

#define APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT (6)

#define APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT (3)

#define APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<20*1000?20*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT (6)


#define APP_BATTERY_REPORT_INTERVAL (5)

#define APP_BATTERY_MV_BASE ((APP_BATTERY_MAX_MV-APP_BATTERY_PD_MV)/(APP_BATTERY_LEVEL_NUM))

#define APP_BATTERY_STABLE_COUNT (5)
#define APP_BATTERY_MEASURE_PERIODIC_FAST_MS (200)
#ifdef BLE_ONLY_ENABLED
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (25000)
#else
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (10000)
#endif
#define APP_BATTERY_CHARGING_PERIODIC_MS (APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS)

#define APP_BATTERY_SET_MESSAGE(appevt, status, volt) (appevt = (((uint32_t)status&0xffff)<<16)|(volt&0xffff))
#define APP_BATTERY_GET_STATUS(appevt, status) (status = (appevt>>16)&0xffff)
#define APP_BATTERY_GET_VOLT(appevt, volt) (volt = appevt&0xffff)
#define APP_BATTERY_GET_PRAMS(appevt, prams) ((prams) = appevt&0xffff)

enum APP_BATTERY_MEASURE_PERIODIC_T
{
    APP_BATTERY_MEASURE_PERIODIC_FAST = 0,
    APP_BATTERY_MEASURE_PERIODIC_NORMAL,
    APP_BATTERY_MEASURE_PERIODIC_CHARGING,

    APP_BATTERY_MEASURE_PERIODIC_QTY,
};

struct APP_BATTERY_MEASURE_CHARGER_STATUS_T
{
    HAL_GPADC_MV_T prevolt;
    int32_t slope_1000[APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT];
    int slope_1000_index;
    int cnt;
};


typedef void (*APP_BATTERY_EVENT_CB_T)(enum APP_BATTERY_STATUS_T, APP_BATTERY_MV_T volt);

struct APP_BATTERY_MEASURE_T
{
    uint32_t start_time;
    enum APP_BATTERY_STATUS_T status;
#ifdef __INTERCONNECTION__
    uint8_t currentBatteryInfo;
    uint8_t lastBatteryInfo;
    uint8_t isMobileSupportSelfDefinedCommand;
#else
    uint8_t currlevel;
#endif
    APP_BATTERY_MV_T currvolt;
    APP_BATTERY_MV_T lowvolt;
    APP_BATTERY_MV_T highvolt;
    APP_BATTERY_MV_T pdvolt;
    uint32_t chargetimeout;
    enum APP_BATTERY_MEASURE_PERIODIC_T periodic;
    HAL_GPADC_MV_T voltage[APP_BATTERY_STABLE_COUNT];
    uint16_t index;
    struct APP_BATTERY_MEASURE_CHARGER_STATUS_T charger_status;
    APP_BATTERY_EVENT_CB_T cb;
    APP_BATTERY_CB_T user_cb;
};

#ifdef IS_BES_BATTERY_MANAGER_ENABLED

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void);

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static enum HAL_GPADC_CHAN_T app_vbat_ch = APP_BATTERY_GPADC_CH_NUM;
#ifdef CHARGER_SPICIAL_CHAN
static uint8_t app_vbat_volt_div = GPADC_VBAT_VOLT_DIV;
#endif
static void app_battery_timer_handler(void const *param);
osTimerDef (APP_BATTERY, app_battery_timer_handler);
static osTimerId app_battery_timer = NULL;
static struct APP_BATTERY_MEASURE_T app_battery_measure;

static int app_battery_charger_handle_process(void);

#ifdef __INTERCONNECTION__
uint8_t* app_battery_get_mobile_support_self_defined_command_p(void)
{
    return &app_battery_measure.isMobileSupportSelfDefinedCommand;
}
#endif


void app_battery_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint8_t i;
    uint32_t meanBattVolt = 0;
    HAL_GPADC_MV_T vbat = volt;
    APP_BATTERY_TRACE(2,"%s %d",__func__, vbat);
 #ifdef CHARGER_SPICIAL_CHAN
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat * app_vbat_volt_div) <= APP_BATTERY_ERR_MV))
#else
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat<<2) <= APP_BATTERY_ERR_MV))
#endif
    {
        app_battery_measure.cb(APP_BATTERY_STATUS_INVALID, vbat);
        return;
    }

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
    if(app_usbaudio_mode_on()) return ;
#endif
#ifdef CHARGER_SPICIAL_CHAN
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat * app_vbat_volt_div;
#else
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat<<2;
#endif 
    if (app_battery_measure.index > APP_BATTERY_STABLE_COUNT)
    {
        for (i=0; i<APP_BATTERY_STABLE_COUNT; i++)
        {
            meanBattVolt += app_battery_measure.voltage[i];
        }
        meanBattVolt /= APP_BATTERY_STABLE_COUNT;
        if (app_battery_measure.cb)
        {
            if (meanBattVolt>app_battery_measure.highvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_OVERVOLT, meanBattVolt);
            }
            else if((meanBattVolt>app_battery_measure.pdvolt) && (meanBattVolt<app_battery_measure.lowvolt))
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_UNDERVOLT, meanBattVolt);
            }
            else if(meanBattVolt<app_battery_measure.pdvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_PDVOLT, meanBattVolt);
            }
            else
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_NORMAL, meanBattVolt);
            }
        }
    }
    else
    {
        int8_t level = 0;
#ifdef CHARGER_SPICIAL_CHAN
         meanBattVolt = vbat * app_vbat_volt_div;
#else
         meanBattVolt = vbat<<2;
#endif
        level = (meanBattVolt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

        if (level<APP_BATTERY_LEVEL_MIN)
            level = APP_BATTERY_LEVEL_MIN;
        if (level>APP_BATTERY_LEVEL_MAX)
            level = APP_BATTERY_LEVEL_MAX;

        app_battery_measure.currvolt = meanBattVolt;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->batteryLevel = level;
#else
        app_battery_measure.currlevel = level;
#endif
    }
}

static void app_battery_timer_start(enum APP_BATTERY_MEASURE_PERIODIC_T periodic)
{
    uint32_t periodic_millisec = 0;

    if (app_battery_measure.periodic != periodic){
        app_battery_measure.periodic = periodic;
        switch (periodic)
        {
            case APP_BATTERY_MEASURE_PERIODIC_FAST:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_FAST_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_CHARGING:
                periodic_millisec = APP_BATTERY_CHARGING_PERIODIC_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_NORMAL:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS;
            default:
                break;
        }
        osTimerStop(app_battery_timer);
        osTimerStart(app_battery_timer, periodic_millisec);
    }
}

static void app_battery_timer_handler(void const *param)
{
#ifdef CHARGER_1802
    charger_vbat_div_adc_enable(true);
    hal_gpadc_open(HAL_GPADC_CHAN_5, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#else
    hal_gpadc_open(app_vbat_ch, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#endif
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);

}

#ifdef BT_BUILD_WITH_CUSTOMER_HOST
int app_status_battery_report(uint8_t level)
{
    return 0;
}
#else
int app_status_battery_report(uint8_t level)
{
#if defined(__BTIF_EARPHONE__)
    app_10_second_timer_check();
#endif

    if (app_is_stack_ready())
    {
// #if (HF_CUSTOM_FEATURE_SUPPORT & HF_CUSTOM_FEATURE_BATTERY_REPORT) || (HF_SDK_FEATURES & HF_FEATURE_HF_INDICATORS)
#if defined(SUPPORT_BATTERY_REPORT) || defined(SUPPORT_HF_INDICATORS)
#if defined(IBRT)
        uint8_t hfp_device = app_bt_audio_get_curr_hfp_device();
        struct BT_DEVICE_T *curr_device = app_bt_get_device(hfp_device);
        if (curr_device->hf_conn_flag)
        {
            app_hfp_set_battery_level(level);
        }
#elif defined(BT_HFP_SUPPORT)
        app_hfp_set_battery_level(level);
#endif
#else
        TRACE(1,"[%s] Can not enable SUPPORT_BATTERY_REPORT", __func__);
#endif
        bes_bt_osapi_notify_evm();
    }
    return 0;
}
#endif

int app_battery_handle_process_normal(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    int8_t level = 0;

    switch (status)
    {
        case APP_BATTERY_STATUS_UNDERVOLT:
            TRACE(1,"UNDERVOLT:%d", prams.volt);
            app_status_indication_set(APP_STATUS_INDICATION_CHARGENEED);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(IBRT)

#else
            media_PlayAudio(AUD_ID_BT_CHARGE_PLEASE, 0);
#endif
#endif
            // FALLTHROUGH
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_OVERVOLT:
            app_battery_measure.currvolt = prams.volt;
            level = (prams.volt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

            if (level<APP_BATTERY_LEVEL_MIN)
                level = APP_BATTERY_LEVEL_MIN;
            if (level>APP_BATTERY_LEVEL_MAX)
                level = APP_BATTERY_LEVEL_MAX;
#ifdef __INTERCONNECTION__
            APP_BATTERY_INFO_T* pBatteryInfo;
            pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
            pBatteryInfo->batteryLevel = level;
            if(level == APP_BATTERY_LEVEL_MAX)
            {
                level = 9;
            }
            else
            {
                level /= 10;
            }
#else
            app_battery_measure.currlevel = level;
#endif
            app_status_battery_report(level);
            break;
        case APP_BATTERY_STATUS_PDVOLT:
#ifndef BT_USB_AUDIO_DUAL_MODE
            TRACE(1,"PDVOLT-->POWEROFF:%d", prams.volt);
            osTimerStop(app_battery_timer);
            app_shutdown();
#endif
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING-->APP_BATTERY_CHARGER :%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#ifdef BT_USB_AUDIO_DUAL_MODE
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#else
#if CHARGER_PLUGINOUT_RESET
                app_reset();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
#endif
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_NORMAL);
    return 0;
}

int app_battery_handle_process_charging(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    switch (status)
    {
        case APP_BATTERY_STATUS_OVERVOLT:
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_UNDERVOLT:
            app_battery_measure.currvolt = prams.volt;
            app_status_battery_report(prams.volt);
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING:%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGOUT)
            {
#ifndef BT_USB_AUDIO_DUAL_MODE
#if CHARGER_PLUGINOUT_RESET
                TRACE(0,"CHARGING-->RESET");
                osTimerStop(app_battery_timer);
                app_shutdown();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#endif
#endif
            }
            else if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#if defined(BT_USB_AUDIO_DUAL_MODE)
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    if (app_battery_charger_handle_process()<=0)
    {
        if (app_status_indication_get() != APP_STATUS_INDICATION_FULLCHARGE)
        {
            TRACE(1,"FULL_CHARGING:%d", app_battery_measure.currvolt);
            app_status_indication_set(APP_STATUS_INDICATION_FULLCHARGE);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(BT_USB_AUDIO_DUAL_MODE) || defined(IBRT)
#else
            media_PlayAudio(AUD_ID_BT_CHARGE_FINISH, 0);
#endif
#endif
        }
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_CHARGING);

    return 0;
}

extern "C"     bool pmu_ana_volt_is_high(void);
static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;
    union APP_BATTERY_MSG_PRAMS msg_prams;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);
    APP_BATTERY_GET_PRAMS(msg_body->message_id, msg_prams.prams);

    uint32_t generatedSeed = hal_sys_timer_get();
    for (uint8_t index = 0; index < sizeof(bt_global_addr); index++)
    {
        generatedSeed ^= (((uint32_t)(bt_global_addr[index])) << (hal_sys_timer_get()&0xF));
    }
    srand(generatedSeed);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }
    else
    {
        switch (app_battery_measure.status)
        {
            case APP_BATTERY_STATUS_NORMAL:
                app_battery_handle_process_normal((uint32_t)status, msg_prams);
#if defined(CHIP_BEST1501P)
                if(pmu_ana_volt_is_high() == false){
                    pmu_ntc_capture_start(NULL);
                }else{
                    TRACE(1,"%s stop ana check",__func__);
                }
#endif
                break;

            case APP_BATTERY_STATUS_CHARGING:
                app_battery_handle_process_charging((uint32_t)status, msg_prams);
                break;

            default:
                break;
        }
    }
    if (NULL != app_battery_measure.user_cb)
    {
        uint8_t batteryLevel;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo;
        pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->chargingStatus = ((app_battery_measure.status == APP_BATTERY_STATUS_CHARGING)? 1:0);
        batteryLevel = pBatteryInfo->batteryLevel;

#else
        batteryLevel = app_battery_measure.currlevel;
#endif
        app_battery_measure.user_cb(app_battery_measure.currvolt,
                                    batteryLevel, app_battery_measure.status,status,msg_prams);
    }

    return 0;
}

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    if(NULL == app_battery_measure.user_cb)
    {
        app_battery_measure.user_cb = user_cb;
        return 0;
    }
    return 1;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    if (currvolt)
    {
        *currvolt = app_battery_measure.currvolt;
    }

    if (currlevel)
    {
#ifdef __INTERCONNECTION__
        *currlevel = app_battery_measure.currentBatteryInfo;
#else
        *currlevel = app_battery_measure.currlevel;
#endif
    }

    if (status)
    {
        *status = app_battery_measure.status;
    }

    return 0;
}

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
static void app_battery_gpadc_configuration_init(void)
{
    if (charger_package_type_get() == CHARGER_PACKAGE_TYPE_SIP_1620)
    {
        app_vbat_ch = HAL_GPADC_CHAN_BATTERY;
        app_vbat_volt_div = 4;
    }
}
#endif

int app_battery_open(void)
{
    APP_BATTERY_TRACE(3,"%s batt range:%d~%d",__func__, APP_BATTERY_MIN_MV, APP_BATTERY_MAX_MV);
    int nRet = APP_BATTERY_OPEN_MODE_INVALID;

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
    app_battery_gpadc_configuration_init();
#endif

    if (app_battery_timer == NULL)
        app_battery_timer = osTimerCreate (osTimer(APP_BATTERY), osTimerPeriodic, NULL);

    if (app_battery_pluginout_debounce_timer == NULL)
        app_battery_pluginout_debounce_timer = osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE), osTimerOnce, &app_battery_pluginout_debounce_ctx);

    app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#ifdef __INTERCONNECTION__
    app_battery_measure.currentBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.lastBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.isMobileSupportSelfDefinedCommand = 0;
#else
    app_battery_measure.currlevel = APP_BATTERY_LEVEL_MAX;
#endif
    app_battery_measure.currvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.lowvolt = APP_BATTERY_MIN_MV;
    app_battery_measure.highvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.pdvolt = APP_BATTERY_PD_MV;
    app_battery_measure.chargetimeout = APP_BATTERY_CHARGE_TIMEOUT_MIN;

    app_battery_measure.periodic = APP_BATTERY_MEASURE_PERIODIC_QTY;
    app_battery_measure.cb = app_battery_event_process;
    app_battery_measure.user_cb = NULL;

    app_battery_measure.charger_status.prevolt = 0;
    app_battery_measure.charger_status.slope_1000_index = 0;
    app_battery_measure.charger_status.cnt = 0;

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_IN, 1);
    }

    if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    }

    if (app_battery_charger_indication_open() == APP_BATTERY_CHARGER_PLUGIN)
    {
        app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
        app_battery_measure.start_time = hal_sys_timer_get();
        //pmu_charger_plugin_config();
        if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
        }

#if (CHARGER_PLUGINOUT_RESET == 0)
        nRet = APP_BATTERY_OPEN_MODE_CHARGING_PWRON;
#else
        nRet = APP_BATTERY_OPEN_MODE_CHARGING;
#endif
    }
    else
    {
        app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
        //pmu_charger_plugout_config();
        nRet = APP_BATTERY_OPEN_MODE_NORMAL;
    }
    return nRet;
}

int app_battery_start(void)
{
    APP_BATTERY_TRACE(2,"%s %d",__func__, APP_BATTERY_MEASURE_PERIODIC_FAST_MS);

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_FAST);

    return 0;
}

int app_battery_stop(void)
{
    osTimerStop(app_battery_timer);

    return 0;
}

int app_battery_close(void)
{
    hal_gpadc_close(HAL_GPADC_CHAN_BATTERY);

    return 0;
}


static int32_t app_battery_charger_slope_calc(int32_t t1, int32_t v1, int32_t t2, int32_t v2)
{
    int32_t slope_1000;
    slope_1000 = (v2-v1)*1000/(t2-t1);
    return slope_1000;
}

static int app_battery_charger_handle_process(void)
{
    int nRet = 1;
    int8_t i=0,cnt=0;
    uint32_t slope_1000 = 0;
    uint32_t charging_min;
    static uint8_t overvolt_full_charge_cnt = 0;
    static uint8_t ext_pin_full_charge_cnt = 0;

    charging_min = hal_sys_timer_get() - app_battery_measure.start_time;
    charging_min = TICKS_TO_MS(charging_min)/1000/60;
    if (charging_min >= app_battery_measure.chargetimeout)
    {
        // TRACE(0,"TIMEROUT-->FULL_CHARGING");
        nRet = -1;
        goto exit;
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT) == 0)
    {
        if (app_battery_measure.currvolt>=(app_battery_measure.highvolt+APP_BATTERY_CHARGE_OFFSET_MV))
        {
            overvolt_full_charge_cnt++;
        }
        else
        {
            overvolt_full_charge_cnt = 0;
        }
        if (overvolt_full_charge_cnt>=APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT)
        {
            //TRACE(0,"OVERVOLT-->FULL_CHARGING");
            nRet = -1;
            goto exit;
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT) == 0)
    {
        if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            if (hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
            {
                ext_pin_full_charge_cnt++;
            }
            else
            {
                ext_pin_full_charge_cnt = 0;
            }
            if (ext_pin_full_charge_cnt>=APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT)
            {
                TRACE(0,"EXT PIN-->FULL_CHARGING");
                nRet = -1;
                goto exit;
            }
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT) == 0)
    {
        if (!app_battery_measure.charger_status.prevolt)
        {
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                app_battery_measure.charger_status.slope_1000[i]=100;
            }
        }
        else
        {
            slope_1000 = app_battery_charger_slope_calc(0, app_battery_measure.charger_status.prevolt,
                         APP_BATTERY_CHARGING_PERIODIC_MS*APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT/1000, app_battery_measure.currvolt);
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                if (app_battery_measure.charger_status.slope_1000[i]>0)
                    cnt++;
                else
                    cnt--;
                TRACE(3,"slope_1000[%d]=%d cnt:%d", i,app_battery_measure.charger_status.slope_1000[i], cnt);
            }
            TRACE(3,"app_battery_charger_slope_proc slope*1000=%d cnt:%d nRet:%d", slope_1000, cnt, nRet);
            if (cnt>1)
            {
                nRet = 1;
            }/*else (3>=cnt && cnt>=-3){
                nRet = 0;
            }*/else
            {
                if (app_battery_measure.currvolt>=(app_battery_measure.highvolt-APP_BATTERY_CHARGE_OFFSET_MV))
                {
                    TRACE(0,"SLOPE-->FULL_CHARGING");
                    nRet = -1;
                }
            }
        }
        app_battery_measure.charger_status.slope_1000_index++;
    }
exit:
    return nRet;
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGIN");
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGOUT");
    }

    return status;
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);
#if defined(BT_USB_AUDIO_DUAL_MODE)
    btusb_switch(BTUSB_MODE_BT);
#endif
    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");
        if (status_charger == APP_BATTERY_CHARGER_PLUGIN)
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
            }
            app_battery_measure.start_time = hal_sys_timer_get();
        }
        else
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
            }
        }
        app_battery_event_process(APP_BATTERY_STATUS_CHARGING, status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    uint8_t cnt = 0;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    do
    {
        status = app_battery_charger_forcegetstatus();
        if (status == APP_BATTERY_CHARGER_PLUGIN)
            break;
        osDelay(20);
    }
    while(cnt++<5);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        if (!hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
        {
            status = APP_BATTERY_CHARGER_PLUGIN;
        }
    }

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}

int8_t app_battery_current_level(void)
{
#ifdef __INTERCONNECTION__
    return app_battery_measure.currentBatteryInfo & 0x7f;
#else
    return app_battery_measure.currlevel;
#endif
}

int8_t app_battery_is_charging(void)
{
    return (APP_BATTERY_STATUS_CHARGING == app_battery_measure.status);
}
typedef uint16_t NTP_VOLTAGE_MV_T;
typedef uint16_t NTP_TEMPERATURE_C_T;

#define NTC_CAPTURE_STABLE_COUNT (5)
#define NTC_CAPTURE_TEMPERATURE_STEP (4)
#define NTC_CAPTURE_TEMPERATURE_REF (15)
#define NTC_CAPTURE_VOLTAGE_REF (1100)

typedef void (*NTC_CAPTURE_MEASURE_CB_T)(NTP_TEMPERATURE_C_T);

struct NTC_CAPTURE_MEASURE_T
{
    NTP_TEMPERATURE_C_T temperature;
    NTP_VOLTAGE_MV_T currvolt;
    NTP_VOLTAGE_MV_T voltage[NTC_CAPTURE_STABLE_COUNT];
    uint16_t index;
    NTC_CAPTURE_MEASURE_CB_T cb;
};

static struct NTC_CAPTURE_MEASURE_T ntc_capture_measure;

void ntc_capture_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint32_t meanVolt = 0;
    TRACE(3,"%s %d irq:0x%04x",__func__, volt, irq_val);

    if (volt == HAL_GPADC_BAD_VALUE)
    {
        return;
    }

    ntc_capture_measure.voltage[ntc_capture_measure.index++%NTC_CAPTURE_STABLE_COUNT] = volt;

    if (ntc_capture_measure.index > NTC_CAPTURE_STABLE_COUNT)
    {
        for (uint8_t i=0; i<NTC_CAPTURE_STABLE_COUNT; i++)
        {
            meanVolt += ntc_capture_measure.voltage[i];
        }
        meanVolt /= NTC_CAPTURE_STABLE_COUNT;
        ntc_capture_measure.currvolt = meanVolt;
    }
    else if (!ntc_capture_measure.currvolt)
    {
        ntc_capture_measure.currvolt = volt;
    }
    ntc_capture_measure.temperature = ((int32_t)ntc_capture_measure.currvolt - NTC_CAPTURE_VOLTAGE_REF)/NTC_CAPTURE_TEMPERATURE_STEP + NTC_CAPTURE_TEMPERATURE_REF;
    pmu_ntc_capture_disable();
    TRACE(3,"%s ad:%d temperature:%d",__func__, ntc_capture_measure.currvolt, ntc_capture_measure.temperature);
}

int ntc_capture_open(void)
{

    ntc_capture_measure.currvolt = 0;
    ntc_capture_measure.index = 0;
    ntc_capture_measure.temperature = 0;
    ntc_capture_measure.cb = NULL;

    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}

int ntc_capture_start(void)
{
    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}
#else

#define IS_USE_SOC_PMU_PLUGINOUT

#ifdef IS_USE_SOC_PMU_PLUGINOUT

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }

    return 0;
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
    }

    return status;
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);

    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event);

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");

        app_battery_pluginout_event_callback(status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}
#endif

int app_battery_open(void)
{
    app_battery_opened_callback();
#ifdef IS_USE_SOC_PMU_PLUGINOUT
    if (app_battery_pluginout_debounce_timer == NULL)
    {
        app_battery_pluginout_debounce_timer =
            osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE),
            osTimerOnce, &app_battery_pluginout_debounce_ctx);
    }

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    app_battery_charger_indication_open();
#endif

    // initialize the custom battery manager here
    // returned value could be:
    // #define APP_BATTERY_OPEN_MODE_INVALID        (-1)
    // #define APP_BATTERY_OPEN_MODE_NORMAL         (0)
    // #define APP_BATTERY_OPEN_MODE_CHARGING       (1)
    // #define APP_BATTERY_OPEN_MODE_CHARGING_PWRON (2)
    return APP_BATTERY_OPEN_MODE_NORMAL;
}

int app_battery_start(void)
{
    // start battery measurement timer here
    return 0;
}

int app_battery_stop(void)
{
    // stop battery measurement timer here
    return 0;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    // should just return battery level via currlevel for hfp battery level indication
    *currlevel = APP_BATTERY_LEVEL_MAX;
    return 0;
}

#ifdef IS_USE_SOC_PMU_PLUGINOUT
static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event)
{
    if (APP_BATTERY_CHARGER_PLUGOUT == event)
    {
        TRACE(0, "Charger plug out.");
    }
    else if (APP_BATTERY_CHARGER_PLUGIN == event)
    {
        TRACE(0, "Charger plug in.");
    }
}
#endif

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    // register the battery level update event callback
    return 0;
}

#endif

WEAK void app_battery_opened_callback(void)
{

}

#else
int app_battery_open(void)
{
    return 0;
}

int app_battery_start(void)
{
    return 0;
}
#endif
/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include "cmsis_os.h"
#include "tgt_hardware.h"
#include "pmu.h"
#include "hal_timer.h"
#include "hal_gpadc.h"
#include "hal_trace.h"
#include "hal_gpio.h"
#include "hal_iomux.h"
#include "hal_chipid.h"
#include "app_thread.h"
#include "app_battery.h"
#include "audio_policy.h"
#include "app_bt.h"
#include "apps.h"
#include "app_hfp.h"

#ifdef APP_BATTERY_ENABLE
#include "app_status_ind.h"
#include "bluetooth_bt_api.h"
#include "app_media_player.h"
#ifdef BT_USB_AUDIO_DUAL_MODE
#include "btusb_audio.h"
#endif
#include <stdlib.h>

#ifdef __INTERCONNECTION__
#include "bluetooth_ble_api.h"
#endif

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
extern "C" bool app_usbaudio_mode_on(void);
#endif

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
#include CHIP_SPECIFIC_HDR(charger)
#endif

#define APP_BATTERY_TRACE(s,...)
// TRACE(s, ##__VA_ARGS__)

#ifndef APP_BATTERY_GPADC_CH_NUM
#define APP_BATTERY_GPADC_CH_NUM (HAL_GPADC_CHAN_BATTERY)
#endif

#ifndef APP_BATTERY_ERR_MV
#define APP_BATTERY_ERR_MV (2800)
#endif

#ifndef APP_BATTERY_MIN_MV
#define APP_BATTERY_MIN_MV (3200)
#endif

#ifndef APP_BATTERY_MAX_MV
#define APP_BATTERY_MAX_MV (4200)
#endif

#ifndef APP_BATTERY_PD_MV
#define APP_BATTERY_PD_MV   (3100)
#endif

#ifndef APP_BATTERY_CHARGE_TIMEOUT_MIN
#define APP_BATTERY_CHARGE_TIMEOUT_MIN (90)
#endif

#ifndef APP_BATTERY_CHARGE_OFFSET_MV
#define APP_BATTERY_CHARGE_OFFSET_MV (20)
#endif

#ifndef CHARGER_PLUGINOUT_RESET
#define CHARGER_PLUGINOUT_RESET (1)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

#define APP_BATTERY_CHARGING_PLUGOUT_DEDOUNCE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<500?3:1)

#define APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT (6)

#define APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT (3)

#define APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<20*1000?20*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT (6)


#define APP_BATTERY_REPORT_INTERVAL (5)

#define APP_BATTERY_MV_BASE ((APP_BATTERY_MAX_MV-APP_BATTERY_PD_MV)/(APP_BATTERY_LEVEL_NUM))

#define APP_BATTERY_STABLE_COUNT (5)
#define APP_BATTERY_MEASURE_PERIODIC_FAST_MS (200)
#ifdef BLE_ONLY_ENABLED
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (25000)
#else
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (10000)
#endif
#define APP_BATTERY_CHARGING_PERIODIC_MS (APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS)

#define APP_BATTERY_SET_MESSAGE(appevt, status, volt) (appevt = (((uint32_t)status&0xffff)<<16)|(volt&0xffff))
#define APP_BATTERY_GET_STATUS(appevt, status) (status = (appevt>>16)&0xffff)
#define APP_BATTERY_GET_VOLT(appevt, volt) (volt = appevt&0xffff)
#define APP_BATTERY_GET_PRAMS(appevt, prams) ((prams) = appevt&0xffff)

enum APP_BATTERY_MEASURE_PERIODIC_T
{
    APP_BATTERY_MEASURE_PERIODIC_FAST = 0,
    APP_BATTERY_MEASURE_PERIODIC_NORMAL,
    APP_BATTERY_MEASURE_PERIODIC_CHARGING,

    APP_BATTERY_MEASURE_PERIODIC_QTY,
};

struct APP_BATTERY_MEASURE_CHARGER_STATUS_T
{
    HAL_GPADC_MV_T prevolt;
    int32_t slope_1000[APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT];
    int slope_1000_index;
    int cnt;
};


typedef void (*APP_BATTERY_EVENT_CB_T)(enum APP_BATTERY_STATUS_T, APP_BATTERY_MV_T volt);

struct APP_BATTERY_MEASURE_T
{
    uint32_t start_time;
    enum APP_BATTERY_STATUS_T status;
#ifdef __INTERCONNECTION__
    uint8_t currentBatteryInfo;
    uint8_t lastBatteryInfo;
    uint8_t isMobileSupportSelfDefinedCommand;
#else
    uint8_t currlevel;
#endif
    APP_BATTERY_MV_T currvolt;
    APP_BATTERY_MV_T lowvolt;
    APP_BATTERY_MV_T highvolt;
    APP_BATTERY_MV_T pdvolt;
    uint32_t chargetimeout;
    enum APP_BATTERY_MEASURE_PERIODIC_T periodic;
    HAL_GPADC_MV_T voltage[APP_BATTERY_STABLE_COUNT];
    uint16_t index;
    struct APP_BATTERY_MEASURE_CHARGER_STATUS_T charger_status;
    APP_BATTERY_EVENT_CB_T cb;
    APP_BATTERY_CB_T user_cb;
};

#ifdef IS_BES_BATTERY_MANAGER_ENABLED

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void);

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static enum HAL_GPADC_CHAN_T app_vbat_ch = APP_BATTERY_GPADC_CH_NUM;
#ifdef CHARGER_SPICIAL_CHAN
static uint8_t app_vbat_volt_div = GPADC_VBAT_VOLT_DIV;
#endif
static void app_battery_timer_handler(void const *param);
osTimerDef (APP_BATTERY, app_battery_timer_handler);
static osTimerId app_battery_timer = NULL;
static struct APP_BATTERY_MEASURE_T app_battery_measure;

static int app_battery_charger_handle_process(void);

#ifdef __INTERCONNECTION__
uint8_t* app_battery_get_mobile_support_self_defined_command_p(void)
{
    return &app_battery_measure.isMobileSupportSelfDefinedCommand;
}
#endif


void app_battery_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint8_t i;
    uint32_t meanBattVolt = 0;
    HAL_GPADC_MV_T vbat = volt;
    APP_BATTERY_TRACE(2,"%s %d",__func__, vbat);
 #ifdef CHARGER_SPICIAL_CHAN
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat * app_vbat_volt_div) <= APP_BATTERY_ERR_MV))
#else
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat<<2) <= APP_BATTERY_ERR_MV))
#endif
    {
        app_battery_measure.cb(APP_BATTERY_STATUS_INVALID, vbat);
        return;
    }

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
    if(app_usbaudio_mode_on()) return ;
#endif
#ifdef CHARGER_SPICIAL_CHAN
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat * app_vbat_volt_div;
#else
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat<<2;
#endif 
    if (app_battery_measure.index > APP_BATTERY_STABLE_COUNT)
    {
        for (i=0; i<APP_BATTERY_STABLE_COUNT; i++)
        {
            meanBattVolt += app_battery_measure.voltage[i];
        }
        meanBattVolt /= APP_BATTERY_STABLE_COUNT;
        if (app_battery_measure.cb)
        {
            if (meanBattVolt>app_battery_measure.highvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_OVERVOLT, meanBattVolt);
            }
            else if((meanBattVolt>app_battery_measure.pdvolt) && (meanBattVolt<app_battery_measure.lowvolt))
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_UNDERVOLT, meanBattVolt);
            }
            else if(meanBattVolt<app_battery_measure.pdvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_PDVOLT, meanBattVolt);
            }
            else
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_NORMAL, meanBattVolt);
            }
        }
    }
    else
    {
        int8_t level = 0;
#ifdef CHARGER_SPICIAL_CHAN
         meanBattVolt = vbat * app_vbat_volt_div;
#else
         meanBattVolt = vbat<<2;
#endif
        level = (meanBattVolt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

        if (level<APP_BATTERY_LEVEL_MIN)
            level = APP_BATTERY_LEVEL_MIN;
        if (level>APP_BATTERY_LEVEL_MAX)
            level = APP_BATTERY_LEVEL_MAX;

        app_battery_measure.currvolt = meanBattVolt;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->batteryLevel = level;
#else
        app_battery_measure.currlevel = level;
#endif
    }
}

static void app_battery_timer_start(enum APP_BATTERY_MEASURE_PERIODIC_T periodic)
{
    uint32_t periodic_millisec = 0;

    if (app_battery_measure.periodic != periodic){
        app_battery_measure.periodic = periodic;
        switch (periodic)
        {
            case APP_BATTERY_MEASURE_PERIODIC_FAST:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_FAST_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_CHARGING:
                periodic_millisec = APP_BATTERY_CHARGING_PERIODIC_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_NORMAL:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS;
            default:
                break;
        }
        osTimerStop(app_battery_timer);
        osTimerStart(app_battery_timer, periodic_millisec);
    }
}

static void app_battery_timer_handler(void const *param)
{
#ifdef CHARGER_1802
    charger_vbat_div_adc_enable(true);
    hal_gpadc_open(HAL_GPADC_CHAN_5, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#else
    hal_gpadc_open(app_vbat_ch, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#endif
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);

}

#ifdef BT_BUILD_WITH_CUSTOMER_HOST
int app_status_battery_report(uint8_t level)
{
    return 0;
}
#else
int app_status_battery_report(uint8_t level)
{
#if defined(__BTIF_EARPHONE__)
    app_10_second_timer_check();
#endif

    if (app_is_stack_ready())
    {
// #if (HF_CUSTOM_FEATURE_SUPPORT & HF_CUSTOM_FEATURE_BATTERY_REPORT) || (HF_SDK_FEATURES & HF_FEATURE_HF_INDICATORS)
#if defined(SUPPORT_BATTERY_REPORT) || defined(SUPPORT_HF_INDICATORS)
#if defined(IBRT)
        uint8_t hfp_device = app_bt_audio_get_curr_hfp_device();
        struct BT_DEVICE_T *curr_device = app_bt_get_device(hfp_device);
        if (curr_device->hf_conn_flag)
        {
            app_hfp_set_battery_level(level);
        }
#elif defined(BT_HFP_SUPPORT)
        app_hfp_set_battery_level(level);
#endif
#else
        TRACE(1,"[%s] Can not enable SUPPORT_BATTERY_REPORT", __func__);
#endif
        bes_bt_osapi_notify_evm();
    }
    return 0;
}
#endif

int app_battery_handle_process_normal(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    int8_t level = 0;

    switch (status)
    {
        case APP_BATTERY_STATUS_UNDERVOLT:
            TRACE(1,"UNDERVOLT:%d", prams.volt);
            app_status_indication_set(APP_STATUS_INDICATION_CHARGENEED);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(IBRT)

#else
            media_PlayAudio(AUD_ID_BT_CHARGE_PLEASE, 0);
#endif
#endif
            // FALLTHROUGH
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_OVERVOLT:
            app_battery_measure.currvolt = prams.volt;
            level = (prams.volt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

            if (level<APP_BATTERY_LEVEL_MIN)
                level = APP_BATTERY_LEVEL_MIN;
            if (level>APP_BATTERY_LEVEL_MAX)
                level = APP_BATTERY_LEVEL_MAX;
#ifdef __INTERCONNECTION__
            APP_BATTERY_INFO_T* pBatteryInfo;
            pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
            pBatteryInfo->batteryLevel = level;
            if(level == APP_BATTERY_LEVEL_MAX)
            {
                level = 9;
            }
            else
            {
                level /= 10;
            }
#else
            app_battery_measure.currlevel = level;
#endif
            app_status_battery_report(level);
            break;
        case APP_BATTERY_STATUS_PDVOLT:
#ifndef BT_USB_AUDIO_DUAL_MODE
            TRACE(1,"PDVOLT-->POWEROFF:%d", prams.volt);
            osTimerStop(app_battery_timer);
            app_shutdown();
#endif
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING-->APP_BATTERY_CHARGER :%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#ifdef BT_USB_AUDIO_DUAL_MODE
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#else
#if CHARGER_PLUGINOUT_RESET
                app_reset();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
#endif
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_NORMAL);
    return 0;
}

int app_battery_handle_process_charging(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    switch (status)
    {
        case APP_BATTERY_STATUS_OVERVOLT:
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_UNDERVOLT:
            app_battery_measure.currvolt = prams.volt;
            app_status_battery_report(prams.volt);
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING:%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGOUT)
            {
#ifndef BT_USB_AUDIO_DUAL_MODE
#if CHARGER_PLUGINOUT_RESET
                TRACE(0,"CHARGING-->RESET");
                osTimerStop(app_battery_timer);
                app_shutdown();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#endif
#endif
            }
            else if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#if defined(BT_USB_AUDIO_DUAL_MODE)
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    if (app_battery_charger_handle_process()<=0)
    {
        if (app_status_indication_get() != APP_STATUS_INDICATION_FULLCHARGE)
        {
            TRACE(1,"FULL_CHARGING:%d", app_battery_measure.currvolt);
            app_status_indication_set(APP_STATUS_INDICATION_FULLCHARGE);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(BT_USB_AUDIO_DUAL_MODE) || defined(IBRT)
#else
            media_PlayAudio(AUD_ID_BT_CHARGE_FINISH, 0);
#endif
#endif
        }
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_CHARGING);

    return 0;
}

extern "C"     bool pmu_ana_volt_is_high(void);
static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;
    union APP_BATTERY_MSG_PRAMS msg_prams;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);
    APP_BATTERY_GET_PRAMS(msg_body->message_id, msg_prams.prams);

    uint32_t generatedSeed = hal_sys_timer_get();
    for (uint8_t index = 0; index < sizeof(bt_global_addr); index++)
    {
        generatedSeed ^= (((uint32_t)(bt_global_addr[index])) << (hal_sys_timer_get()&0xF));
    }
    srand(generatedSeed);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }
    else
    {
        switch (app_battery_measure.status)
        {
            case APP_BATTERY_STATUS_NORMAL:
                app_battery_handle_process_normal((uint32_t)status, msg_prams);
#if defined(CHIP_BEST1501P)
                if(pmu_ana_volt_is_high() == false){
                    pmu_ntc_capture_start(NULL);
                }else{
                    TRACE(1,"%s stop ana check",__func__);
                }
#endif
                break;

            case APP_BATTERY_STATUS_CHARGING:
                app_battery_handle_process_charging((uint32_t)status, msg_prams);
                break;

            default:
                break;
        }
    }
    if (NULL != app_battery_measure.user_cb)
    {
        uint8_t batteryLevel;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo;
        pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->chargingStatus = ((app_battery_measure.status == APP_BATTERY_STATUS_CHARGING)? 1:0);
        batteryLevel = pBatteryInfo->batteryLevel;

#else
        batteryLevel = app_battery_measure.currlevel;
#endif
        app_battery_measure.user_cb(app_battery_measure.currvolt,
                                    batteryLevel, app_battery_measure.status,status,msg_prams);
    }

    return 0;
}

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    if(NULL == app_battery_measure.user_cb)
    {
        app_battery_measure.user_cb = user_cb;
        return 0;
    }
    return 1;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    if (currvolt)
    {
        *currvolt = app_battery_measure.currvolt;
    }

    if (currlevel)
    {
#ifdef __INTERCONNECTION__
        *currlevel = app_battery_measure.currentBatteryInfo;
#else
        *currlevel = app_battery_measure.currlevel;
#endif
    }

    if (status)
    {
        *status = app_battery_measure.status;
    }

    return 0;
}

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
static void app_battery_gpadc_configuration_init(void)
{
    if (charger_package_type_get() == CHARGER_PACKAGE_TYPE_SIP_1620)
    {
        app_vbat_ch = HAL_GPADC_CHAN_BATTERY;
        app_vbat_volt_div = 4;
    }
}
#endif

int app_battery_open(void)
{
    APP_BATTERY_TRACE(3,"%s batt range:%d~%d",__func__, APP_BATTERY_MIN_MV, APP_BATTERY_MAX_MV);
    int nRet = APP_BATTERY_OPEN_MODE_INVALID;

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
    app_battery_gpadc_configuration_init();
#endif

    if (app_battery_timer == NULL)
        app_battery_timer = osTimerCreate (osTimer(APP_BATTERY), osTimerPeriodic, NULL);

    if (app_battery_pluginout_debounce_timer == NULL)
        app_battery_pluginout_debounce_timer = osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE), osTimerOnce, &app_battery_pluginout_debounce_ctx);

    app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#ifdef __INTERCONNECTION__
    app_battery_measure.currentBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.lastBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.isMobileSupportSelfDefinedCommand = 0;
#else
    app_battery_measure.currlevel = APP_BATTERY_LEVEL_MAX;
#endif
    app_battery_measure.currvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.lowvolt = APP_BATTERY_MIN_MV;
    app_battery_measure.highvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.pdvolt = APP_BATTERY_PD_MV;
    app_battery_measure.chargetimeout = APP_BATTERY_CHARGE_TIMEOUT_MIN;

    app_battery_measure.periodic = APP_BATTERY_MEASURE_PERIODIC_QTY;
    app_battery_measure.cb = app_battery_event_process;
    app_battery_measure.user_cb = NULL;

    app_battery_measure.charger_status.prevolt = 0;
    app_battery_measure.charger_status.slope_1000_index = 0;
    app_battery_measure.charger_status.cnt = 0;

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_IN, 1);
    }

    if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    }

    if (app_battery_charger_indication_open() == APP_BATTERY_CHARGER_PLUGIN)
    {
        app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
        app_battery_measure.start_time = hal_sys_timer_get();
        //pmu_charger_plugin_config();
        if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
        }

#if (CHARGER_PLUGINOUT_RESET == 0)
        nRet = APP_BATTERY_OPEN_MODE_CHARGING_PWRON;
#else
        nRet = APP_BATTERY_OPEN_MODE_CHARGING;
#endif
    }
    else
    {
        app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
        //pmu_charger_plugout_config();
        nRet = APP_BATTERY_OPEN_MODE_NORMAL;
    }
    return nRet;
}

int app_battery_start(void)
{
    APP_BATTERY_TRACE(2,"%s %d",__func__, APP_BATTERY_MEASURE_PERIODIC_FAST_MS);

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_FAST);

    return 0;
}

int app_battery_stop(void)
{
    osTimerStop(app_battery_timer);

    return 0;
}

int app_battery_close(void)
{
    hal_gpadc_close(HAL_GPADC_CHAN_BATTERY);

    return 0;
}


static int32_t app_battery_charger_slope_calc(int32_t t1, int32_t v1, int32_t t2, int32_t v2)
{
    int32_t slope_1000;
    slope_1000 = (v2-v1)*1000/(t2-t1);
    return slope_1000;
}

static int app_battery_charger_handle_process(void)
{
    int nRet = 1;
    int8_t i=0,cnt=0;
    uint32_t slope_1000 = 0;
    uint32_t charging_min;
    static uint8_t overvolt_full_charge_cnt = 0;
    static uint8_t ext_pin_full_charge_cnt = 0;

    charging_min = hal_sys_timer_get() - app_battery_measure.start_time;
    charging_min = TICKS_TO_MS(charging_min)/1000/60;
    if (charging_min >= app_battery_measure.chargetimeout)
    {
        // TRACE(0,"TIMEROUT-->FULL_CHARGING");
        nRet = -1;
        goto exit;
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT) == 0)
    {
        if (app_battery_measure.currvolt>=(app_battery_measure.highvolt+APP_BATTERY_CHARGE_OFFSET_MV))
        {
            overvolt_full_charge_cnt++;
        }
        else
        {
            overvolt_full_charge_cnt = 0;
        }
        if (overvolt_full_charge_cnt>=APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT)
        {
            //TRACE(0,"OVERVOLT-->FULL_CHARGING");
            nRet = -1;
            goto exit;
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT) == 0)
    {
        if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            if (hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
            {
                ext_pin_full_charge_cnt++;
            }
            else
            {
                ext_pin_full_charge_cnt = 0;
            }
            if (ext_pin_full_charge_cnt>=APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT)
            {
                TRACE(0,"EXT PIN-->FULL_CHARGING");
                nRet = -1;
                goto exit;
            }
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT) == 0)
    {
        if (!app_battery_measure.charger_status.prevolt)
        {
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                app_battery_measure.charger_status.slope_1000[i]=100;
            }
        }
        else
        {
            slope_1000 = app_battery_charger_slope_calc(0, app_battery_measure.charger_status.prevolt,
                         APP_BATTERY_CHARGING_PERIODIC_MS*APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT/1000, app_battery_measure.currvolt);
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                if (app_battery_measure.charger_status.slope_1000[i]>0)
                    cnt++;
                else
                    cnt--;
                TRACE(3,"slope_1000[%d]=%d cnt:%d", i,app_battery_measure.charger_status.slope_1000[i], cnt);
            }
            TRACE(3,"app_battery_charger_slope_proc slope*1000=%d cnt:%d nRet:%d", slope_1000, cnt, nRet);
            if (cnt>1)
            {
                nRet = 1;
            }/*else (3>=cnt && cnt>=-3){
                nRet = 0;
            }*/else
            {
                if (app_battery_measure.currvolt>=(app_battery_measure.highvolt-APP_BATTERY_CHARGE_OFFSET_MV))
                {
                    TRACE(0,"SLOPE-->FULL_CHARGING");
                    nRet = -1;
                }
            }
        }
        app_battery_measure.charger_status.slope_1000_index++;
    }
exit:
    return nRet;
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGIN");
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGOUT");
    }

    return status;
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);
#if defined(BT_USB_AUDIO_DUAL_MODE)
    btusb_switch(BTUSB_MODE_BT);
#endif
    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");
        if (status_charger == APP_BATTERY_CHARGER_PLUGIN)
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
            }
            app_battery_measure.start_time = hal_sys_timer_get();
        }
        else
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
            }
        }
        app_battery_event_process(APP_BATTERY_STATUS_CHARGING, status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    uint8_t cnt = 0;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    do
    {
        status = app_battery_charger_forcegetstatus();
        if (status == APP_BATTERY_CHARGER_PLUGIN)
            break;
        osDelay(20);
    }
    while(cnt++<5);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        if (!hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
        {
            status = APP_BATTERY_CHARGER_PLUGIN;
        }
    }

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}

int8_t app_battery_current_level(void)
{
#ifdef __INTERCONNECTION__
    return app_battery_measure.currentBatteryInfo & 0x7f;
#else
    return app_battery_measure.currlevel;
#endif
}

int8_t app_battery_is_charging(void)
{
    return (APP_BATTERY_STATUS_CHARGING == app_battery_measure.status);
}
typedef uint16_t NTP_VOLTAGE_MV_T;
typedef uint16_t NTP_TEMPERATURE_C_T;

#define NTC_CAPTURE_STABLE_COUNT (5)
#define NTC_CAPTURE_TEMPERATURE_STEP (4)
#define NTC_CAPTURE_TEMPERATURE_REF (15)
#define NTC_CAPTURE_VOLTAGE_REF (1100)

typedef void (*NTC_CAPTURE_MEASURE_CB_T)(NTP_TEMPERATURE_C_T);

struct NTC_CAPTURE_MEASURE_T
{
    NTP_TEMPERATURE_C_T temperature;
    NTP_VOLTAGE_MV_T currvolt;
    NTP_VOLTAGE_MV_T voltage[NTC_CAPTURE_STABLE_COUNT];
    uint16_t index;
    NTC_CAPTURE_MEASURE_CB_T cb;
};

static struct NTC_CAPTURE_MEASURE_T ntc_capture_measure;

void ntc_capture_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint32_t meanVolt = 0;
    TRACE(3,"%s %d irq:0x%04x",__func__, volt, irq_val);

    if (volt == HAL_GPADC_BAD_VALUE)
    {
        return;
    }

    ntc_capture_measure.voltage[ntc_capture_measure.index++%NTC_CAPTURE_STABLE_COUNT] = volt;

    if (ntc_capture_measure.index > NTC_CAPTURE_STABLE_COUNT)
    {
        for (uint8_t i=0; i<NTC_CAPTURE_STABLE_COUNT; i++)
        {
            meanVolt += ntc_capture_measure.voltage[i];
        }
        meanVolt /= NTC_CAPTURE_STABLE_COUNT;
        ntc_capture_measure.currvolt = meanVolt;
    }
    else if (!ntc_capture_measure.currvolt)
    {
        ntc_capture_measure.currvolt = volt;
    }
    ntc_capture_measure.temperature = ((int32_t)ntc_capture_measure.currvolt - NTC_CAPTURE_VOLTAGE_REF)/NTC_CAPTURE_TEMPERATURE_STEP + NTC_CAPTURE_TEMPERATURE_REF;
    pmu_ntc_capture_disable();
    TRACE(3,"%s ad:%d temperature:%d",__func__, ntc_capture_measure.currvolt, ntc_capture_measure.temperature);
}

int ntc_capture_open(void)
{

    ntc_capture_measure.currvolt = 0;
    ntc_capture_measure.index = 0;
    ntc_capture_measure.temperature = 0;
    ntc_capture_measure.cb = NULL;

    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}

int ntc_capture_start(void)
{
    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}
#else

#define IS_USE_SOC_PMU_PLUGINOUT

#ifdef IS_USE_SOC_PMU_PLUGINOUT

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }

    return 0;
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
    }

    return status;
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);

    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event);

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");

        app_battery_pluginout_event_callback(status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}
#endif

int app_battery_open(void)
{
    app_battery_opened_callback();
#ifdef IS_USE_SOC_PMU_PLUGINOUT
    if (app_battery_pluginout_debounce_timer == NULL)
    {
        app_battery_pluginout_debounce_timer =
            osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE),
            osTimerOnce, &app_battery_pluginout_debounce_ctx);
    }

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    app_battery_charger_indication_open();
#endif

    // initialize the custom battery manager here
    // returned value could be:
    // #define APP_BATTERY_OPEN_MODE_INVALID        (-1)
    // #define APP_BATTERY_OPEN_MODE_NORMAL         (0)
    // #define APP_BATTERY_OPEN_MODE_CHARGING       (1)
    // #define APP_BATTERY_OPEN_MODE_CHARGING_PWRON (2)
    return APP_BATTERY_OPEN_MODE_NORMAL;
}

int app_battery_start(void)
{
    // start battery measurement timer here
    return 0;
}

int app_battery_stop(void)
{
    // stop battery measurement timer here
    return 0;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    // should just return battery level via currlevel for hfp battery level indication
    *currlevel = APP_BATTERY_LEVEL_MAX;
    return 0;
}

#ifdef IS_USE_SOC_PMU_PLUGINOUT
static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event)
{
    if (APP_BATTERY_CHARGER_PLUGOUT == event)
    {
        TRACE(0, "Charger plug out.");
    }
    else if (APP_BATTERY_CHARGER_PLUGIN == event)
    {
        TRACE(0, "Charger plug in.");
    }
}
#endif

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    // register the battery level update event callback
    return 0;
}

#endif

WEAK void app_battery_opened_callback(void)
{

}

#else
int app_battery_open(void)
{
    return 0;
}

int app_battery_start(void)
{
    return 0;
}
#endif
/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include "cmsis_os.h"
#include "tgt_hardware.h"
#include "pmu.h"
#include "hal_timer.h"
#include "hal_gpadc.h"
#include "hal_trace.h"
#include "hal_gpio.h"
#include "hal_iomux.h"
#include "hal_chipid.h"
#include "app_thread.h"
#include "app_battery.h"
#include "audio_policy.h"
#include "app_bt.h"
#include "apps.h"
#include "app_hfp.h"

#ifdef APP_BATTERY_ENABLE
#include "app_status_ind.h"
#include "bluetooth_bt_api.h"
#include "app_media_player.h"
#ifdef BT_USB_AUDIO_DUAL_MODE
#include "btusb_audio.h"
#endif
#include <stdlib.h>

#ifdef __INTERCONNECTION__
#include "bluetooth_ble_api.h"
#endif

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
extern "C" bool app_usbaudio_mode_on(void);
#endif

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
#include CHIP_SPECIFIC_HDR(charger)
#endif

#define APP_BATTERY_TRACE(s,...)
// TRACE(s, ##__VA_ARGS__)

#ifndef APP_BATTERY_GPADC_CH_NUM
#define APP_BATTERY_GPADC_CH_NUM (HAL_GPADC_CHAN_BATTERY)
#endif

#ifndef APP_BATTERY_ERR_MV
#define APP_BATTERY_ERR_MV (2800)
#endif

#ifndef APP_BATTERY_MIN_MV
#define APP_BATTERY_MIN_MV (3200)
#endif

#ifndef APP_BATTERY_MAX_MV
#define APP_BATTERY_MAX_MV (4200)
#endif

#ifndef APP_BATTERY_PD_MV
#define APP_BATTERY_PD_MV   (3100)
#endif

#ifndef APP_BATTERY_CHARGE_TIMEOUT_MIN
#define APP_BATTERY_CHARGE_TIMEOUT_MIN (90)
#endif

#ifndef APP_BATTERY_CHARGE_OFFSET_MV
#define APP_BATTERY_CHARGE_OFFSET_MV (20)
#endif

#ifndef CHARGER_PLUGINOUT_RESET
#define CHARGER_PLUGINOUT_RESET (1)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

#define APP_BATTERY_CHARGING_PLUGOUT_DEDOUNCE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<500?3:1)

#define APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT (6)

#define APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT (3)

#define APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<20*1000?20*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT (6)


#define APP_BATTERY_REPORT_INTERVAL (5)

#define APP_BATTERY_MV_BASE ((APP_BATTERY_MAX_MV-APP_BATTERY_PD_MV)/(APP_BATTERY_LEVEL_NUM))

#define APP_BATTERY_STABLE_COUNT (5)
#define APP_BATTERY_MEASURE_PERIODIC_FAST_MS (200)
#ifdef BLE_ONLY_ENABLED
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (25000)
#else
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (10000)
#endif
#define APP_BATTERY_CHARGING_PERIODIC_MS (APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS)

#define APP_BATTERY_SET_MESSAGE(appevt, status, volt) (appevt = (((uint32_t)status&0xffff)<<16)|(volt&0xffff))
#define APP_BATTERY_GET_STATUS(appevt, status) (status = (appevt>>16)&0xffff)
#define APP_BATTERY_GET_VOLT(appevt, volt) (volt = appevt&0xffff)
#define APP_BATTERY_GET_PRAMS(appevt, prams) ((prams) = appevt&0xffff)

enum APP_BATTERY_MEASURE_PERIODIC_T
{
    APP_BATTERY_MEASURE_PERIODIC_FAST = 0,
    APP_BATTERY_MEASURE_PERIODIC_NORMAL,
    APP_BATTERY_MEASURE_PERIODIC_CHARGING,

    APP_BATTERY_MEASURE_PERIODIC_QTY,
};

struct APP_BATTERY_MEASURE_CHARGER_STATUS_T
{
    HAL_GPADC_MV_T prevolt;
    int32_t slope_1000[APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT];
    int slope_1000_index;
    int cnt;
};


typedef void (*APP_BATTERY_EVENT_CB_T)(enum APP_BATTERY_STATUS_T, APP_BATTERY_MV_T volt);

struct APP_BATTERY_MEASURE_T
{
    uint32_t start_time;
    enum APP_BATTERY_STATUS_T status;
#ifdef __INTERCONNECTION__
    uint8_t currentBatteryInfo;
    uint8_t lastBatteryInfo;
    uint8_t isMobileSupportSelfDefinedCommand;
#else
    uint8_t currlevel;
#endif
    APP_BATTERY_MV_T currvolt;
    APP_BATTERY_MV_T lowvolt;
    APP_BATTERY_MV_T highvolt;
    APP_BATTERY_MV_T pdvolt;
    uint32_t chargetimeout;
    enum APP_BATTERY_MEASURE_PERIODIC_T periodic;
    HAL_GPADC_MV_T voltage[APP_BATTERY_STABLE_COUNT];
    uint16_t index;
    struct APP_BATTERY_MEASURE_CHARGER_STATUS_T charger_status;
    APP_BATTERY_EVENT_CB_T cb;
    APP_BATTERY_CB_T user_cb;
};

#ifdef IS_BES_BATTERY_MANAGER_ENABLED

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void);

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static enum HAL_GPADC_CHAN_T app_vbat_ch = APP_BATTERY_GPADC_CH_NUM;
#ifdef CHARGER_SPICIAL_CHAN
static uint8_t app_vbat_volt_div = GPADC_VBAT_VOLT_DIV;
#endif
static void app_battery_timer_handler(void const *param);
osTimerDef (APP_BATTERY, app_battery_timer_handler);
static osTimerId app_battery_timer = NULL;
static struct APP_BATTERY_MEASURE_T app_battery_measure;

static int app_battery_charger_handle_process(void);

#ifdef __INTERCONNECTION__
uint8_t* app_battery_get_mobile_support_self_defined_command_p(void)
{
    return &app_battery_measure.isMobileSupportSelfDefinedCommand;
}
#endif


void app_battery_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint8_t i;
    uint32_t meanBattVolt = 0;
    HAL_GPADC_MV_T vbat = volt;
    APP_BATTERY_TRACE(2,"%s %d",__func__, vbat);
 #ifdef CHARGER_SPICIAL_CHAN
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat * app_vbat_volt_div) <= APP_BATTERY_ERR_MV))
#else
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat<<2) <= APP_BATTERY_ERR_MV))
#endif
    {
        app_battery_measure.cb(APP_BATTERY_STATUS_INVALID, vbat);
        return;
    }

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
    if(app_usbaudio_mode_on()) return ;
#endif
#ifdef CHARGER_SPICIAL_CHAN
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat * app_vbat_volt_div;
#else
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat<<2;
#endif 
    if (app_battery_measure.index > APP_BATTERY_STABLE_COUNT)
    {
        for (i=0; i<APP_BATTERY_STABLE_COUNT; i++)
        {
            meanBattVolt += app_battery_measure.voltage[i];
        }
        meanBattVolt /= APP_BATTERY_STABLE_COUNT;
        if (app_battery_measure.cb)
        {
            if (meanBattVolt>app_battery_measure.highvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_OVERVOLT, meanBattVolt);
            }
            else if((meanBattVolt>app_battery_measure.pdvolt) && (meanBattVolt<app_battery_measure.lowvolt))
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_UNDERVOLT, meanBattVolt);
            }
            else if(meanBattVolt<app_battery_measure.pdvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_PDVOLT, meanBattVolt);
            }
            else
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_NORMAL, meanBattVolt);
            }
        }
    }
    else
    {
        int8_t level = 0;
#ifdef CHARGER_SPICIAL_CHAN
         meanBattVolt = vbat * app_vbat_volt_div;
#else
         meanBattVolt = vbat<<2;
#endif
        level = (meanBattVolt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

        if (level<APP_BATTERY_LEVEL_MIN)
            level = APP_BATTERY_LEVEL_MIN;
        if (level>APP_BATTERY_LEVEL_MAX)
            level = APP_BATTERY_LEVEL_MAX;

        app_battery_measure.currvolt = meanBattVolt;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->batteryLevel = level;
#else
        app_battery_measure.currlevel = level;
#endif
    }
}

static void app_battery_timer_start(enum APP_BATTERY_MEASURE_PERIODIC_T periodic)
{
    uint32_t periodic_millisec = 0;

    if (app_battery_measure.periodic != periodic){
        app_battery_measure.periodic = periodic;
        switch (periodic)
        {
            case APP_BATTERY_MEASURE_PERIODIC_FAST:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_FAST_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_CHARGING:
                periodic_millisec = APP_BATTERY_CHARGING_PERIODIC_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_NORMAL:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS;
            default:
                break;
        }
        osTimerStop(app_battery_timer);
        osTimerStart(app_battery_timer, periodic_millisec);
    }
}

static void app_battery_timer_handler(void const *param)
{
#ifdef CHARGER_1802
    charger_vbat_div_adc_enable(true);
    hal_gpadc_open(HAL_GPADC_CHAN_5, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#else
    hal_gpadc_open(app_vbat_ch, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#endif
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);

}

#ifdef BT_BUILD_WITH_CUSTOMER_HOST
int app_status_battery_report(uint8_t level)
{
    return 0;
}
#else
int app_status_battery_report(uint8_t level)
{
#if defined(__BTIF_EARPHONE__)
    app_10_second_timer_check();
#endif

    if (app_is_stack_ready())
    {
// #if (HF_CUSTOM_FEATURE_SUPPORT & HF_CUSTOM_FEATURE_BATTERY_REPORT) || (HF_SDK_FEATURES & HF_FEATURE_HF_INDICATORS)
#if defined(SUPPORT_BATTERY_REPORT) || defined(SUPPORT_HF_INDICATORS)
#if defined(IBRT)
        uint8_t hfp_device = app_bt_audio_get_curr_hfp_device();
        struct BT_DEVICE_T *curr_device = app_bt_get_device(hfp_device);
        if (curr_device->hf_conn_flag)
        {
            app_hfp_set_battery_level(level);
        }
#elif defined(BT_HFP_SUPPORT)
        app_hfp_set_battery_level(level);
#endif
#else
        TRACE(1,"[%s] Can not enable SUPPORT_BATTERY_REPORT", __func__);
#endif
        bes_bt_osapi_notify_evm();
    }
    return 0;
}
#endif

int app_battery_handle_process_normal(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    int8_t level = 0;

    switch (status)
    {
        case APP_BATTERY_STATUS_UNDERVOLT:
            TRACE(1,"UNDERVOLT:%d", prams.volt);
            app_status_indication_set(APP_STATUS_INDICATION_CHARGENEED);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(IBRT)

#else
            media_PlayAudio(AUD_ID_BT_CHARGE_PLEASE, 0);
#endif
#endif
            // FALLTHROUGH
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_OVERVOLT:
            app_battery_measure.currvolt = prams.volt;
            level = (prams.volt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

            if (level<APP_BATTERY_LEVEL_MIN)
                level = APP_BATTERY_LEVEL_MIN;
            if (level>APP_BATTERY_LEVEL_MAX)
                level = APP_BATTERY_LEVEL_MAX;
#ifdef __INTERCONNECTION__
            APP_BATTERY_INFO_T* pBatteryInfo;
            pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
            pBatteryInfo->batteryLevel = level;
            if(level == APP_BATTERY_LEVEL_MAX)
            {
                level = 9;
            }
            else
            {
                level /= 10;
            }
#else
            app_battery_measure.currlevel = level;
#endif
            app_status_battery_report(level);
            break;
        case APP_BATTERY_STATUS_PDVOLT:
#ifndef BT_USB_AUDIO_DUAL_MODE
            TRACE(1,"PDVOLT-->POWEROFF:%d", prams.volt);
            osTimerStop(app_battery_timer);
            app_shutdown();
#endif
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING-->APP_BATTERY_CHARGER :%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#ifdef BT_USB_AUDIO_DUAL_MODE
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#else
#if CHARGER_PLUGINOUT_RESET
                app_reset();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
#endif
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_NORMAL);
    return 0;
}

int app_battery_handle_process_charging(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    switch (status)
    {
        case APP_BATTERY_STATUS_OVERVOLT:
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_UNDERVOLT:
            app_battery_measure.currvolt = prams.volt;
            app_status_battery_report(prams.volt);
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING:%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGOUT)
            {
#ifndef BT_USB_AUDIO_DUAL_MODE
#if CHARGER_PLUGINOUT_RESET
                TRACE(0,"CHARGING-->RESET");
                osTimerStop(app_battery_timer);
                app_shutdown();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#endif
#endif
            }
            else if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#if defined(BT_USB_AUDIO_DUAL_MODE)
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    if (app_battery_charger_handle_process()<=0)
    {
        if (app_status_indication_get() != APP_STATUS_INDICATION_FULLCHARGE)
        {
            TRACE(1,"FULL_CHARGING:%d", app_battery_measure.currvolt);
            app_status_indication_set(APP_STATUS_INDICATION_FULLCHARGE);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(BT_USB_AUDIO_DUAL_MODE) || defined(IBRT)
#else
            media_PlayAudio(AUD_ID_BT_CHARGE_FINISH, 0);
#endif
#endif
        }
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_CHARGING);

    return 0;
}

extern "C"     bool pmu_ana_volt_is_high(void);
static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;
    union APP_BATTERY_MSG_PRAMS msg_prams;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);
    APP_BATTERY_GET_PRAMS(msg_body->message_id, msg_prams.prams);

    uint32_t generatedSeed = hal_sys_timer_get();
    for (uint8_t index = 0; index < sizeof(bt_global_addr); index++)
    {
        generatedSeed ^= (((uint32_t)(bt_global_addr[index])) << (hal_sys_timer_get()&0xF));
    }
    srand(generatedSeed);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }
    else
    {
        switch (app_battery_measure.status)
        {
            case APP_BATTERY_STATUS_NORMAL:
                app_battery_handle_process_normal((uint32_t)status, msg_prams);
#if defined(CHIP_BEST1501P)
                if(pmu_ana_volt_is_high() == false){
                    pmu_ntc_capture_start(NULL);
                }else{
                    TRACE(1,"%s stop ana check",__func__);
                }
#endif
                break;

            case APP_BATTERY_STATUS_CHARGING:
                app_battery_handle_process_charging((uint32_t)status, msg_prams);
                break;

            default:
                break;
        }
    }
    if (NULL != app_battery_measure.user_cb)
    {
        uint8_t batteryLevel;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo;
        pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->chargingStatus = ((app_battery_measure.status == APP_BATTERY_STATUS_CHARGING)? 1:0);
        batteryLevel = pBatteryInfo->batteryLevel;

#else
        batteryLevel = app_battery_measure.currlevel;
#endif
        app_battery_measure.user_cb(app_battery_measure.currvolt,
                                    batteryLevel, app_battery_measure.status,status,msg_prams);
    }

    return 0;
}

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    if(NULL == app_battery_measure.user_cb)
    {
        app_battery_measure.user_cb = user_cb;
        return 0;
    }
    return 1;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    if (currvolt)
    {
        *currvolt = app_battery_measure.currvolt;
    }

    if (currlevel)
    {
#ifdef __INTERCONNECTION__
        *currlevel = app_battery_measure.currentBatteryInfo;
#else
        *currlevel = app_battery_measure.currlevel;
#endif
    }

    if (status)
    {
        *status = app_battery_measure.status;
    }

    return 0;
}

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
static void app_battery_gpadc_configuration_init(void)
{
    if (charger_package_type_get() == CHARGER_PACKAGE_TYPE_SIP_1620)
    {
        app_vbat_ch = HAL_GPADC_CHAN_BATTERY;
        app_vbat_volt_div = 4;
    }
}
#endif

int app_battery_open(void)
{
    APP_BATTERY_TRACE(3,"%s batt range:%d~%d",__func__, APP_BATTERY_MIN_MV, APP_BATTERY_MAX_MV);
    int nRet = APP_BATTERY_OPEN_MODE_INVALID;

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
    app_battery_gpadc_configuration_init();
#endif

    if (app_battery_timer == NULL)
        app_battery_timer = osTimerCreate (osTimer(APP_BATTERY), osTimerPeriodic, NULL);

    if (app_battery_pluginout_debounce_timer == NULL)
        app_battery_pluginout_debounce_timer = osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE), osTimerOnce, &app_battery_pluginout_debounce_ctx);

    app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#ifdef __INTERCONNECTION__
    app_battery_measure.currentBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.lastBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.isMobileSupportSelfDefinedCommand = 0;
#else
    app_battery_measure.currlevel = APP_BATTERY_LEVEL_MAX;
#endif
    app_battery_measure.currvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.lowvolt = APP_BATTERY_MIN_MV;
    app_battery_measure.highvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.pdvolt = APP_BATTERY_PD_MV;
    app_battery_measure.chargetimeout = APP_BATTERY_CHARGE_TIMEOUT_MIN;

    app_battery_measure.periodic = APP_BATTERY_MEASURE_PERIODIC_QTY;
    app_battery_measure.cb = app_battery_event_process;
    app_battery_measure.user_cb = NULL;

    app_battery_measure.charger_status.prevolt = 0;
    app_battery_measure.charger_status.slope_1000_index = 0;
    app_battery_measure.charger_status.cnt = 0;

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_IN, 1);
    }

    if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    }

    if (app_battery_charger_indication_open() == APP_BATTERY_CHARGER_PLUGIN)
    {
        app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
        app_battery_measure.start_time = hal_sys_timer_get();
        //pmu_charger_plugin_config();
        if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
        }

#if (CHARGER_PLUGINOUT_RESET == 0)
        nRet = APP_BATTERY_OPEN_MODE_CHARGING_PWRON;
#else
        nRet = APP_BATTERY_OPEN_MODE_CHARGING;
#endif
    }
    else
    {
        app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
        //pmu_charger_plugout_config();
        nRet = APP_BATTERY_OPEN_MODE_NORMAL;
    }
    return nRet;
}

int app_battery_start(void)
{
    APP_BATTERY_TRACE(2,"%s %d",__func__, APP_BATTERY_MEASURE_PERIODIC_FAST_MS);

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_FAST);

    return 0;
}

int app_battery_stop(void)
{
    osTimerStop(app_battery_timer);

    return 0;
}

int app_battery_close(void)
{
    hal_gpadc_close(HAL_GPADC_CHAN_BATTERY);

    return 0;
}


static int32_t app_battery_charger_slope_calc(int32_t t1, int32_t v1, int32_t t2, int32_t v2)
{
    int32_t slope_1000;
    slope_1000 = (v2-v1)*1000/(t2-t1);
    return slope_1000;
}

static int app_battery_charger_handle_process(void)
{
    int nRet = 1;
    int8_t i=0,cnt=0;
    uint32_t slope_1000 = 0;
    uint32_t charging_min;
    static uint8_t overvolt_full_charge_cnt = 0;
    static uint8_t ext_pin_full_charge_cnt = 0;

    charging_min = hal_sys_timer_get() - app_battery_measure.start_time;
    charging_min = TICKS_TO_MS(charging_min)/1000/60;
    if (charging_min >= app_battery_measure.chargetimeout)
    {
        // TRACE(0,"TIMEROUT-->FULL_CHARGING");
        nRet = -1;
        goto exit;
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT) == 0)
    {
        if (app_battery_measure.currvolt>=(app_battery_measure.highvolt+APP_BATTERY_CHARGE_OFFSET_MV))
        {
            overvolt_full_charge_cnt++;
        }
        else
        {
            overvolt_full_charge_cnt = 0;
        }
        if (overvolt_full_charge_cnt>=APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT)
        {
            //TRACE(0,"OVERVOLT-->FULL_CHARGING");
            nRet = -1;
            goto exit;
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT) == 0)
    {
        if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            if (hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
            {
                ext_pin_full_charge_cnt++;
            }
            else
            {
                ext_pin_full_charge_cnt = 0;
            }
            if (ext_pin_full_charge_cnt>=APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT)
            {
                TRACE(0,"EXT PIN-->FULL_CHARGING");
                nRet = -1;
                goto exit;
            }
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT) == 0)
    {
        if (!app_battery_measure.charger_status.prevolt)
        {
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                app_battery_measure.charger_status.slope_1000[i]=100;
            }
        }
        else
        {
            slope_1000 = app_battery_charger_slope_calc(0, app_battery_measure.charger_status.prevolt,
                         APP_BATTERY_CHARGING_PERIODIC_MS*APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT/1000, app_battery_measure.currvolt);
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                if (app_battery_measure.charger_status.slope_1000[i]>0)
                    cnt++;
                else
                    cnt--;
                TRACE(3,"slope_1000[%d]=%d cnt:%d", i,app_battery_measure.charger_status.slope_1000[i], cnt);
            }
            TRACE(3,"app_battery_charger_slope_proc slope*1000=%d cnt:%d nRet:%d", slope_1000, cnt, nRet);
            if (cnt>1)
            {
                nRet = 1;
            }/*else (3>=cnt && cnt>=-3){
                nRet = 0;
            }*/else
            {
                if (app_battery_measure.currvolt>=(app_battery_measure.highvolt-APP_BATTERY_CHARGE_OFFSET_MV))
                {
                    TRACE(0,"SLOPE-->FULL_CHARGING");
                    nRet = -1;
                }
            }
        }
        app_battery_measure.charger_status.slope_1000_index++;
    }
exit:
    return nRet;
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGIN");
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGOUT");
    }

    return status;
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);
#if defined(BT_USB_AUDIO_DUAL_MODE)
    btusb_switch(BTUSB_MODE_BT);
#endif
    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");
        if (status_charger == APP_BATTERY_CHARGER_PLUGIN)
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
            }
            app_battery_measure.start_time = hal_sys_timer_get();
        }
        else
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
            }
        }
        app_battery_event_process(APP_BATTERY_STATUS_CHARGING, status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    uint8_t cnt = 0;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    do
    {
        status = app_battery_charger_forcegetstatus();
        if (status == APP_BATTERY_CHARGER_PLUGIN)
            break;
        osDelay(20);
    }
    while(cnt++<5);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        if (!hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
        {
            status = APP_BATTERY_CHARGER_PLUGIN;
        }
    }

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}

int8_t app_battery_current_level(void)
{
#ifdef __INTERCONNECTION__
    return app_battery_measure.currentBatteryInfo & 0x7f;
#else
    return app_battery_measure.currlevel;
#endif
}

int8_t app_battery_is_charging(void)
{
    return (APP_BATTERY_STATUS_CHARGING == app_battery_measure.status);
}
typedef uint16_t NTP_VOLTAGE_MV_T;
typedef uint16_t NTP_TEMPERATURE_C_T;

#define NTC_CAPTURE_STABLE_COUNT (5)
#define NTC_CAPTURE_TEMPERATURE_STEP (4)
#define NTC_CAPTURE_TEMPERATURE_REF (15)
#define NTC_CAPTURE_VOLTAGE_REF (1100)

typedef void (*NTC_CAPTURE_MEASURE_CB_T)(NTP_TEMPERATURE_C_T);

struct NTC_CAPTURE_MEASURE_T
{
    NTP_TEMPERATURE_C_T temperature;
    NTP_VOLTAGE_MV_T currvolt;
    NTP_VOLTAGE_MV_T voltage[NTC_CAPTURE_STABLE_COUNT];
    uint16_t index;
    NTC_CAPTURE_MEASURE_CB_T cb;
};

static struct NTC_CAPTURE_MEASURE_T ntc_capture_measure;

void ntc_capture_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint32_t meanVolt = 0;
    TRACE(3,"%s %d irq:0x%04x",__func__, volt, irq_val);

    if (volt == HAL_GPADC_BAD_VALUE)
    {
        return;
    }

    ntc_capture_measure.voltage[ntc_capture_measure.index++%NTC_CAPTURE_STABLE_COUNT] = volt;

    if (ntc_capture_measure.index > NTC_CAPTURE_STABLE_COUNT)
    {
        for (uint8_t i=0; i<NTC_CAPTURE_STABLE_COUNT; i++)
        {
            meanVolt += ntc_capture_measure.voltage[i];
        }
        meanVolt /= NTC_CAPTURE_STABLE_COUNT;
        ntc_capture_measure.currvolt = meanVolt;
    }
    else if (!ntc_capture_measure.currvolt)
    {
        ntc_capture_measure.currvolt = volt;
    }
    ntc_capture_measure.temperature = ((int32_t)ntc_capture_measure.currvolt - NTC_CAPTURE_VOLTAGE_REF)/NTC_CAPTURE_TEMPERATURE_STEP + NTC_CAPTURE_TEMPERATURE_REF;
    pmu_ntc_capture_disable();
    TRACE(3,"%s ad:%d temperature:%d",__func__, ntc_capture_measure.currvolt, ntc_capture_measure.temperature);
}

int ntc_capture_open(void)
{

    ntc_capture_measure.currvolt = 0;
    ntc_capture_measure.index = 0;
    ntc_capture_measure.temperature = 0;
    ntc_capture_measure.cb = NULL;

    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}

int ntc_capture_start(void)
{
    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}
#else

#define IS_USE_SOC_PMU_PLUGINOUT

#ifdef IS_USE_SOC_PMU_PLUGINOUT

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }

    return 0;
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
    }

    return status;
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);

    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event);

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");

        app_battery_pluginout_event_callback(status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}
#endif

int app_battery_open(void)
{
    app_battery_opened_callback();
#ifdef IS_USE_SOC_PMU_PLUGINOUT
    if (app_battery_pluginout_debounce_timer == NULL)
    {
        app_battery_pluginout_debounce_timer =
            osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE),
            osTimerOnce, &app_battery_pluginout_debounce_ctx);
    }

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    app_battery_charger_indication_open();
#endif

    // initialize the custom battery manager here
    // returned value could be:
    // #define APP_BATTERY_OPEN_MODE_INVALID        (-1)
    // #define APP_BATTERY_OPEN_MODE_NORMAL         (0)
    // #define APP_BATTERY_OPEN_MODE_CHARGING       (1)
    // #define APP_BATTERY_OPEN_MODE_CHARGING_PWRON (2)
    return APP_BATTERY_OPEN_MODE_NORMAL;
}

int app_battery_start(void)
{
    // start battery measurement timer here
    return 0;
}

int app_battery_stop(void)
{
    // stop battery measurement timer here
    return 0;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    // should just return battery level via currlevel for hfp battery level indication
    *currlevel = APP_BATTERY_LEVEL_MAX;
    return 0;
}

#ifdef IS_USE_SOC_PMU_PLUGINOUT
static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event)
{
    if (APP_BATTERY_CHARGER_PLUGOUT == event)
    {
        TRACE(0, "Charger plug out.");
    }
    else if (APP_BATTERY_CHARGER_PLUGIN == event)
    {
        TRACE(0, "Charger plug in.");
    }
}
#endif

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    // register the battery level update event callback
    return 0;
}

#endif

WEAK void app_battery_opened_callback(void)
{

}

#else
int app_battery_open(void)
{
    return 0;
}

int app_battery_start(void)
{
    return 0;
}
#endif
/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include "cmsis_os.h"
#include "tgt_hardware.h"
#include "pmu.h"
#include "hal_timer.h"
#include "hal_gpadc.h"
#include "hal_trace.h"
#include "hal_gpio.h"
#include "hal_iomux.h"
#include "hal_chipid.h"
#include "app_thread.h"
#include "app_battery.h"
#include "audio_policy.h"
#include "app_bt.h"
#include "apps.h"
#include "app_hfp.h"

#ifdef APP_BATTERY_ENABLE
#include "app_status_ind.h"
#include "bluetooth_bt_api.h"
#include "app_media_player.h"
#ifdef BT_USB_AUDIO_DUAL_MODE
#include "btusb_audio.h"
#endif
#include <stdlib.h>

#ifdef __INTERCONNECTION__
#include "bluetooth_ble_api.h"
#endif

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
extern "C" bool app_usbaudio_mode_on(void);
#endif

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
#include CHIP_SPECIFIC_HDR(charger)
#endif

#define APP_BATTERY_TRACE(s,...)
// TRACE(s, ##__VA_ARGS__)

#ifndef APP_BATTERY_GPADC_CH_NUM
#define APP_BATTERY_GPADC_CH_NUM (HAL_GPADC_CHAN_BATTERY)
#endif

#ifndef APP_BATTERY_ERR_MV
#define APP_BATTERY_ERR_MV (2800)
#endif

#ifndef APP_BATTERY_MIN_MV
#define APP_BATTERY_MIN_MV (3200)
#endif

#ifndef APP_BATTERY_MAX_MV
#define APP_BATTERY_MAX_MV (4200)
#endif

#ifndef APP_BATTERY_PD_MV
#define APP_BATTERY_PD_MV   (3100)
#endif

#ifndef APP_BATTERY_CHARGE_TIMEOUT_MIN
#define APP_BATTERY_CHARGE_TIMEOUT_MIN (90)
#endif

#ifndef APP_BATTERY_CHARGE_OFFSET_MV
#define APP_BATTERY_CHARGE_OFFSET_MV (20)
#endif

#ifndef CHARGER_PLUGINOUT_RESET
#define CHARGER_PLUGINOUT_RESET (1)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

#define APP_BATTERY_CHARGING_PLUGOUT_DEDOUNCE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<500?3:1)

#define APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT (6)

#define APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT (3)

#define APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<20*1000?20*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT (6)


#define APP_BATTERY_REPORT_INTERVAL (5)

#define APP_BATTERY_MV_BASE ((APP_BATTERY_MAX_MV-APP_BATTERY_PD_MV)/(APP_BATTERY_LEVEL_NUM))

#define APP_BATTERY_STABLE_COUNT (5)
#define APP_BATTERY_MEASURE_PERIODIC_FAST_MS (200)
#ifdef BLE_ONLY_ENABLED
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (25000)
#else
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (10000)
#endif
#define APP_BATTERY_CHARGING_PERIODIC_MS (APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS)

#define APP_BATTERY_SET_MESSAGE(appevt, status, volt) (appevt = (((uint32_t)status&0xffff)<<16)|(volt&0xffff))
#define APP_BATTERY_GET_STATUS(appevt, status) (status = (appevt>>16)&0xffff)
#define APP_BATTERY_GET_VOLT(appevt, volt) (volt = appevt&0xffff)
#define APP_BATTERY_GET_PRAMS(appevt, prams) ((prams) = appevt&0xffff)

enum APP_BATTERY_MEASURE_PERIODIC_T
{
    APP_BATTERY_MEASURE_PERIODIC_FAST = 0,
    APP_BATTERY_MEASURE_PERIODIC_NORMAL,
    APP_BATTERY_MEASURE_PERIODIC_CHARGING,

    APP_BATTERY_MEASURE_PERIODIC_QTY,
};

struct APP_BATTERY_MEASURE_CHARGER_STATUS_T
{
    HAL_GPADC_MV_T prevolt;
    int32_t slope_1000[APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT];
    int slope_1000_index;
    int cnt;
};


typedef void (*APP_BATTERY_EVENT_CB_T)(enum APP_BATTERY_STATUS_T, APP_BATTERY_MV_T volt);

struct APP_BATTERY_MEASURE_T
{
    uint32_t start_time;
    enum APP_BATTERY_STATUS_T status;
#ifdef __INTERCONNECTION__
    uint8_t currentBatteryInfo;
    uint8_t lastBatteryInfo;
    uint8_t isMobileSupportSelfDefinedCommand;
#else
    uint8_t currlevel;
#endif
    APP_BATTERY_MV_T currvolt;
    APP_BATTERY_MV_T lowvolt;
    APP_BATTERY_MV_T highvolt;
    APP_BATTERY_MV_T pdvolt;
    uint32_t chargetimeout;
    enum APP_BATTERY_MEASURE_PERIODIC_T periodic;
    HAL_GPADC_MV_T voltage[APP_BATTERY_STABLE_COUNT];
    uint16_t index;
    struct APP_BATTERY_MEASURE_CHARGER_STATUS_T charger_status;
    APP_BATTERY_EVENT_CB_T cb;
    APP_BATTERY_CB_T user_cb;
};

#ifdef IS_BES_BATTERY_MANAGER_ENABLED

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void);

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static enum HAL_GPADC_CHAN_T app_vbat_ch = APP_BATTERY_GPADC_CH_NUM;
#ifdef CHARGER_SPICIAL_CHAN
static uint8_t app_vbat_volt_div = GPADC_VBAT_VOLT_DIV;
#endif
static void app_battery_timer_handler(void const *param);
osTimerDef (APP_BATTERY, app_battery_timer_handler);
static osTimerId app_battery_timer = NULL;
static struct APP_BATTERY_MEASURE_T app_battery_measure;

static int app_battery_charger_handle_process(void);

#ifdef __INTERCONNECTION__
uint8_t* app_battery_get_mobile_support_self_defined_command_p(void)
{
    return &app_battery_measure.isMobileSupportSelfDefinedCommand;
}
#endif


void app_battery_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint8_t i;
    uint32_t meanBattVolt = 0;
    HAL_GPADC_MV_T vbat = volt;
    APP_BATTERY_TRACE(2,"%s %d",__func__, vbat);
 #ifdef CHARGER_SPICIAL_CHAN
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat * app_vbat_volt_div) <= APP_BATTERY_ERR_MV))
#else
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat<<2) <= APP_BATTERY_ERR_MV))
#endif
    {
        app_battery_measure.cb(APP_BATTERY_STATUS_INVALID, vbat);
        return;
    }

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
    if(app_usbaudio_mode_on()) return ;
#endif
#ifdef CHARGER_SPICIAL_CHAN
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat * app_vbat_volt_div;
#else
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat<<2;
#endif 
    if (app_battery_measure.index > APP_BATTERY_STABLE_COUNT)
    {
        for (i=0; i<APP_BATTERY_STABLE_COUNT; i++)
        {
            meanBattVolt += app_battery_measure.voltage[i];
        }
        meanBattVolt /= APP_BATTERY_STABLE_COUNT;
        if (app_battery_measure.cb)
        {
            if (meanBattVolt>app_battery_measure.highvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_OVERVOLT, meanBattVolt);
            }
            else if((meanBattVolt>app_battery_measure.pdvolt) && (meanBattVolt<app_battery_measure.lowvolt))
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_UNDERVOLT, meanBattVolt);
            }
            else if(meanBattVolt<app_battery_measure.pdvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_PDVOLT, meanBattVolt);
            }
            else
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_NORMAL, meanBattVolt);
            }
        }
    }
    else
    {
        int8_t level = 0;
#ifdef CHARGER_SPICIAL_CHAN
         meanBattVolt = vbat * app_vbat_volt_div;
#else
         meanBattVolt = vbat<<2;
#endif
        level = (meanBattVolt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

        if (level<APP_BATTERY_LEVEL_MIN)
            level = APP_BATTERY_LEVEL_MIN;
        if (level>APP_BATTERY_LEVEL_MAX)
            level = APP_BATTERY_LEVEL_MAX;

        app_battery_measure.currvolt = meanBattVolt;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->batteryLevel = level;
#else
        app_battery_measure.currlevel = level;
#endif
    }
}

static void app_battery_timer_start(enum APP_BATTERY_MEASURE_PERIODIC_T periodic)
{
    uint32_t periodic_millisec = 0;

    if (app_battery_measure.periodic != periodic){
        app_battery_measure.periodic = periodic;
        switch (periodic)
        {
            case APP_BATTERY_MEASURE_PERIODIC_FAST:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_FAST_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_CHARGING:
                periodic_millisec = APP_BATTERY_CHARGING_PERIODIC_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_NORMAL:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS;
            default:
                break;
        }
        osTimerStop(app_battery_timer);
        osTimerStart(app_battery_timer, periodic_millisec);
    }
}

static void app_battery_timer_handler(void const *param)
{
#ifdef CHARGER_1802
    charger_vbat_div_adc_enable(true);
    hal_gpadc_open(HAL_GPADC_CHAN_5, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#else
    hal_gpadc_open(app_vbat_ch, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#endif
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);

}

#ifdef BT_BUILD_WITH_CUSTOMER_HOST
int app_status_battery_report(uint8_t level)
{
    return 0;
}
#else
int app_status_battery_report(uint8_t level)
{
#if defined(__BTIF_EARPHONE__)
    app_10_second_timer_check();
#endif

    if (app_is_stack_ready())
    {
// #if (HF_CUSTOM_FEATURE_SUPPORT & HF_CUSTOM_FEATURE_BATTERY_REPORT) || (HF_SDK_FEATURES & HF_FEATURE_HF_INDICATORS)
#if defined(SUPPORT_BATTERY_REPORT) || defined(SUPPORT_HF_INDICATORS)
#if defined(IBRT)
        uint8_t hfp_device = app_bt_audio_get_curr_hfp_device();
        struct BT_DEVICE_T *curr_device = app_bt_get_device(hfp_device);
        if (curr_device->hf_conn_flag)
        {
            app_hfp_set_battery_level(level);
        }
#elif defined(BT_HFP_SUPPORT)
        app_hfp_set_battery_level(level);
#endif
#else
        TRACE(1,"[%s] Can not enable SUPPORT_BATTERY_REPORT", __func__);
#endif
        bes_bt_osapi_notify_evm();
    }
    return 0;
}
#endif

int app_battery_handle_process_normal(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    int8_t level = 0;

    switch (status)
    {
        case APP_BATTERY_STATUS_UNDERVOLT:
            TRACE(1,"UNDERVOLT:%d", prams.volt);
            app_status_indication_set(APP_STATUS_INDICATION_CHARGENEED);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(IBRT)

#else
            media_PlayAudio(AUD_ID_BT_CHARGE_PLEASE, 0);
#endif
#endif
            // FALLTHROUGH
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_OVERVOLT:
            app_battery_measure.currvolt = prams.volt;
            level = (prams.volt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

            if (level<APP_BATTERY_LEVEL_MIN)
                level = APP_BATTERY_LEVEL_MIN;
            if (level>APP_BATTERY_LEVEL_MAX)
                level = APP_BATTERY_LEVEL_MAX;
#ifdef __INTERCONNECTION__
            APP_BATTERY_INFO_T* pBatteryInfo;
            pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
            pBatteryInfo->batteryLevel = level;
            if(level == APP_BATTERY_LEVEL_MAX)
            {
                level = 9;
            }
            else
            {
                level /= 10;
            }
#else
            app_battery_measure.currlevel = level;
#endif
            app_status_battery_report(level);
            break;
        case APP_BATTERY_STATUS_PDVOLT:
#ifndef BT_USB_AUDIO_DUAL_MODE
            TRACE(1,"PDVOLT-->POWEROFF:%d", prams.volt);
            osTimerStop(app_battery_timer);
            app_shutdown();
#endif
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING-->APP_BATTERY_CHARGER :%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#ifdef BT_USB_AUDIO_DUAL_MODE
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#else
#if CHARGER_PLUGINOUT_RESET
                app_reset();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
#endif
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_NORMAL);
    return 0;
}

int app_battery_handle_process_charging(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    switch (status)
    {
        case APP_BATTERY_STATUS_OVERVOLT:
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_UNDERVOLT:
            app_battery_measure.currvolt = prams.volt;
            app_status_battery_report(prams.volt);
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING:%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGOUT)
            {
#ifndef BT_USB_AUDIO_DUAL_MODE
#if CHARGER_PLUGINOUT_RESET
                TRACE(0,"CHARGING-->RESET");
                osTimerStop(app_battery_timer);
                app_shutdown();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#endif
#endif
            }
            else if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#if defined(BT_USB_AUDIO_DUAL_MODE)
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    if (app_battery_charger_handle_process()<=0)
    {
        if (app_status_indication_get() != APP_STATUS_INDICATION_FULLCHARGE)
        {
            TRACE(1,"FULL_CHARGING:%d", app_battery_measure.currvolt);
            app_status_indication_set(APP_STATUS_INDICATION_FULLCHARGE);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(BT_USB_AUDIO_DUAL_MODE) || defined(IBRT)
#else
            media_PlayAudio(AUD_ID_BT_CHARGE_FINISH, 0);
#endif
#endif
        }
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_CHARGING);

    return 0;
}

extern "C"     bool pmu_ana_volt_is_high(void);
static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;
    union APP_BATTERY_MSG_PRAMS msg_prams;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);
    APP_BATTERY_GET_PRAMS(msg_body->message_id, msg_prams.prams);

    uint32_t generatedSeed = hal_sys_timer_get();
    for (uint8_t index = 0; index < sizeof(bt_global_addr); index++)
    {
        generatedSeed ^= (((uint32_t)(bt_global_addr[index])) << (hal_sys_timer_get()&0xF));
    }
    srand(generatedSeed);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }
    else
    {
        switch (app_battery_measure.status)
        {
            case APP_BATTERY_STATUS_NORMAL:
                app_battery_handle_process_normal((uint32_t)status, msg_prams);
#if defined(CHIP_BEST1501P)
                if(pmu_ana_volt_is_high() == false){
                    pmu_ntc_capture_start(NULL);
                }else{
                    TRACE(1,"%s stop ana check",__func__);
                }
#endif
                break;

            case APP_BATTERY_STATUS_CHARGING:
                app_battery_handle_process_charging((uint32_t)status, msg_prams);
                break;

            default:
                break;
        }
    }
    if (NULL != app_battery_measure.user_cb)
    {
        uint8_t batteryLevel;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo;
        pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->chargingStatus = ((app_battery_measure.status == APP_BATTERY_STATUS_CHARGING)? 1:0);
        batteryLevel = pBatteryInfo->batteryLevel;

#else
        batteryLevel = app_battery_measure.currlevel;
#endif
        app_battery_measure.user_cb(app_battery_measure.currvolt,
                                    batteryLevel, app_battery_measure.status,status,msg_prams);
    }

    return 0;
}

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    if(NULL == app_battery_measure.user_cb)
    {
        app_battery_measure.user_cb = user_cb;
        return 0;
    }
    return 1;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    if (currvolt)
    {
        *currvolt = app_battery_measure.currvolt;
    }

    if (currlevel)
    {
#ifdef __INTERCONNECTION__
        *currlevel = app_battery_measure.currentBatteryInfo;
#else
        *currlevel = app_battery_measure.currlevel;
#endif
    }

    if (status)
    {
        *status = app_battery_measure.status;
    }

    return 0;
}

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
static void app_battery_gpadc_configuration_init(void)
{
    if (charger_package_type_get() == CHARGER_PACKAGE_TYPE_SIP_1620)
    {
        app_vbat_ch = HAL_GPADC_CHAN_BATTERY;
        app_vbat_volt_div = 4;
    }
}
#endif

int app_battery_open(void)
{
    APP_BATTERY_TRACE(3,"%s batt range:%d~%d",__func__, APP_BATTERY_MIN_MV, APP_BATTERY_MAX_MV);
    int nRet = APP_BATTERY_OPEN_MODE_INVALID;

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
    app_battery_gpadc_configuration_init();
#endif

    if (app_battery_timer == NULL)
        app_battery_timer = osTimerCreate (osTimer(APP_BATTERY), osTimerPeriodic, NULL);

    if (app_battery_pluginout_debounce_timer == NULL)
        app_battery_pluginout_debounce_timer = osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE), osTimerOnce, &app_battery_pluginout_debounce_ctx);

    app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#ifdef __INTERCONNECTION__
    app_battery_measure.currentBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.lastBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.isMobileSupportSelfDefinedCommand = 0;
#else
    app_battery_measure.currlevel = APP_BATTERY_LEVEL_MAX;
#endif
    app_battery_measure.currvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.lowvolt = APP_BATTERY_MIN_MV;
    app_battery_measure.highvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.pdvolt = APP_BATTERY_PD_MV;
    app_battery_measure.chargetimeout = APP_BATTERY_CHARGE_TIMEOUT_MIN;

    app_battery_measure.periodic = APP_BATTERY_MEASURE_PERIODIC_QTY;
    app_battery_measure.cb = app_battery_event_process;
    app_battery_measure.user_cb = NULL;

    app_battery_measure.charger_status.prevolt = 0;
    app_battery_measure.charger_status.slope_1000_index = 0;
    app_battery_measure.charger_status.cnt = 0;

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_IN, 1);
    }

    if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    }

    if (app_battery_charger_indication_open() == APP_BATTERY_CHARGER_PLUGIN)
    {
        app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
        app_battery_measure.start_time = hal_sys_timer_get();
        //pmu_charger_plugin_config();
        if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
        }

#if (CHARGER_PLUGINOUT_RESET == 0)
        nRet = APP_BATTERY_OPEN_MODE_CHARGING_PWRON;
#else
        nRet = APP_BATTERY_OPEN_MODE_CHARGING;
#endif
    }
    else
    {
        app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
        //pmu_charger_plugout_config();
        nRet = APP_BATTERY_OPEN_MODE_NORMAL;
    }
    return nRet;
}

int app_battery_start(void)
{
    APP_BATTERY_TRACE(2,"%s %d",__func__, APP_BATTERY_MEASURE_PERIODIC_FAST_MS);

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_FAST);

    return 0;
}

int app_battery_stop(void)
{
    osTimerStop(app_battery_timer);

    return 0;
}

int app_battery_close(void)
{
    hal_gpadc_close(HAL_GPADC_CHAN_BATTERY);

    return 0;
}


static int32_t app_battery_charger_slope_calc(int32_t t1, int32_t v1, int32_t t2, int32_t v2)
{
    int32_t slope_1000;
    slope_1000 = (v2-v1)*1000/(t2-t1);
    return slope_1000;
}

static int app_battery_charger_handle_process(void)
{
    int nRet = 1;
    int8_t i=0,cnt=0;
    uint32_t slope_1000 = 0;
    uint32_t charging_min;
    static uint8_t overvolt_full_charge_cnt = 0;
    static uint8_t ext_pin_full_charge_cnt = 0;

    charging_min = hal_sys_timer_get() - app_battery_measure.start_time;
    charging_min = TICKS_TO_MS(charging_min)/1000/60;
    if (charging_min >= app_battery_measure.chargetimeout)
    {
        // TRACE(0,"TIMEROUT-->FULL_CHARGING");
        nRet = -1;
        goto exit;
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT) == 0)
    {
        if (app_battery_measure.currvolt>=(app_battery_measure.highvolt+APP_BATTERY_CHARGE_OFFSET_MV))
        {
            overvolt_full_charge_cnt++;
        }
        else
        {
            overvolt_full_charge_cnt = 0;
        }
        if (overvolt_full_charge_cnt>=APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT)
        {
            //TRACE(0,"OVERVOLT-->FULL_CHARGING");
            nRet = -1;
            goto exit;
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT) == 0)
    {
        if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            if (hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
            {
                ext_pin_full_charge_cnt++;
            }
            else
            {
                ext_pin_full_charge_cnt = 0;
            }
            if (ext_pin_full_charge_cnt>=APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT)
            {
                TRACE(0,"EXT PIN-->FULL_CHARGING");
                nRet = -1;
                goto exit;
            }
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT) == 0)
    {
        if (!app_battery_measure.charger_status.prevolt)
        {
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                app_battery_measure.charger_status.slope_1000[i]=100;
            }
        }
        else
        {
            slope_1000 = app_battery_charger_slope_calc(0, app_battery_measure.charger_status.prevolt,
                         APP_BATTERY_CHARGING_PERIODIC_MS*APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT/1000, app_battery_measure.currvolt);
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                if (app_battery_measure.charger_status.slope_1000[i]>0)
                    cnt++;
                else
                    cnt--;
                TRACE(3,"slope_1000[%d]=%d cnt:%d", i,app_battery_measure.charger_status.slope_1000[i], cnt);
            }
            TRACE(3,"app_battery_charger_slope_proc slope*1000=%d cnt:%d nRet:%d", slope_1000, cnt, nRet);
            if (cnt>1)
            {
                nRet = 1;
            }/*else (3>=cnt && cnt>=-3){
                nRet = 0;
            }*/else
            {
                if (app_battery_measure.currvolt>=(app_battery_measure.highvolt-APP_BATTERY_CHARGE_OFFSET_MV))
                {
                    TRACE(0,"SLOPE-->FULL_CHARGING");
                    nRet = -1;
                }
            }
        }
        app_battery_measure.charger_status.slope_1000_index++;
    }
exit:
    return nRet;
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGIN");
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGOUT");
    }

    return status;
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);
#if defined(BT_USB_AUDIO_DUAL_MODE)
    btusb_switch(BTUSB_MODE_BT);
#endif
    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");
        if (status_charger == APP_BATTERY_CHARGER_PLUGIN)
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
            }
            app_battery_measure.start_time = hal_sys_timer_get();
        }
        else
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
            }
        }
        app_battery_event_process(APP_BATTERY_STATUS_CHARGING, status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    uint8_t cnt = 0;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    do
    {
        status = app_battery_charger_forcegetstatus();
        if (status == APP_BATTERY_CHARGER_PLUGIN)
            break;
        osDelay(20);
    }
    while(cnt++<5);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        if (!hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
        {
            status = APP_BATTERY_CHARGER_PLUGIN;
        }
    }

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}

int8_t app_battery_current_level(void)
{
#ifdef __INTERCONNECTION__
    return app_battery_measure.currentBatteryInfo & 0x7f;
#else
    return app_battery_measure.currlevel;
#endif
}

int8_t app_battery_is_charging(void)
{
    return (APP_BATTERY_STATUS_CHARGING == app_battery_measure.status);
}
typedef uint16_t NTP_VOLTAGE_MV_T;
typedef uint16_t NTP_TEMPERATURE_C_T;

#define NTC_CAPTURE_STABLE_COUNT (5)
#define NTC_CAPTURE_TEMPERATURE_STEP (4)
#define NTC_CAPTURE_TEMPERATURE_REF (15)
#define NTC_CAPTURE_VOLTAGE_REF (1100)

typedef void (*NTC_CAPTURE_MEASURE_CB_T)(NTP_TEMPERATURE_C_T);

struct NTC_CAPTURE_MEASURE_T
{
    NTP_TEMPERATURE_C_T temperature;
    NTP_VOLTAGE_MV_T currvolt;
    NTP_VOLTAGE_MV_T voltage[NTC_CAPTURE_STABLE_COUNT];
    uint16_t index;
    NTC_CAPTURE_MEASURE_CB_T cb;
};

static struct NTC_CAPTURE_MEASURE_T ntc_capture_measure;

void ntc_capture_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint32_t meanVolt = 0;
    TRACE(3,"%s %d irq:0x%04x",__func__, volt, irq_val);

    if (volt == HAL_GPADC_BAD_VALUE)
    {
        return;
    }

    ntc_capture_measure.voltage[ntc_capture_measure.index++%NTC_CAPTURE_STABLE_COUNT] = volt;

    if (ntc_capture_measure.index > NTC_CAPTURE_STABLE_COUNT)
    {
        for (uint8_t i=0; i<NTC_CAPTURE_STABLE_COUNT; i++)
        {
            meanVolt += ntc_capture_measure.voltage[i];
        }
        meanVolt /= NTC_CAPTURE_STABLE_COUNT;
        ntc_capture_measure.currvolt = meanVolt;
    }
    else if (!ntc_capture_measure.currvolt)
    {
        ntc_capture_measure.currvolt = volt;
    }
    ntc_capture_measure.temperature = ((int32_t)ntc_capture_measure.currvolt - NTC_CAPTURE_VOLTAGE_REF)/NTC_CAPTURE_TEMPERATURE_STEP + NTC_CAPTURE_TEMPERATURE_REF;
    pmu_ntc_capture_disable();
    TRACE(3,"%s ad:%d temperature:%d",__func__, ntc_capture_measure.currvolt, ntc_capture_measure.temperature);
}

int ntc_capture_open(void)
{

    ntc_capture_measure.currvolt = 0;
    ntc_capture_measure.index = 0;
    ntc_capture_measure.temperature = 0;
    ntc_capture_measure.cb = NULL;

    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}

int ntc_capture_start(void)
{
    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}
#else

#define IS_USE_SOC_PMU_PLUGINOUT

#ifdef IS_USE_SOC_PMU_PLUGINOUT

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }

    return 0;
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
    }

    return status;
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);

    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event);

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");

        app_battery_pluginout_event_callback(status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}
#endif

int app_battery_open(void)
{
    app_battery_opened_callback();
#ifdef IS_USE_SOC_PMU_PLUGINOUT
    if (app_battery_pluginout_debounce_timer == NULL)
    {
        app_battery_pluginout_debounce_timer =
            osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE),
            osTimerOnce, &app_battery_pluginout_debounce_ctx);
    }

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    app_battery_charger_indication_open();
#endif

    // initialize the custom battery manager here
    // returned value could be:
    // #define APP_BATTERY_OPEN_MODE_INVALID        (-1)
    // #define APP_BATTERY_OPEN_MODE_NORMAL         (0)
    // #define APP_BATTERY_OPEN_MODE_CHARGING       (1)
    // #define APP_BATTERY_OPEN_MODE_CHARGING_PWRON (2)
    return APP_BATTERY_OPEN_MODE_NORMAL;
}

int app_battery_start(void)
{
    // start battery measurement timer here
    return 0;
}

int app_battery_stop(void)
{
    // stop battery measurement timer here
    return 0;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    // should just return battery level via currlevel for hfp battery level indication
    *currlevel = APP_BATTERY_LEVEL_MAX;
    return 0;
}

#ifdef IS_USE_SOC_PMU_PLUGINOUT
static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event)
{
    if (APP_BATTERY_CHARGER_PLUGOUT == event)
    {
        TRACE(0, "Charger plug out.");
    }
    else if (APP_BATTERY_CHARGER_PLUGIN == event)
    {
        TRACE(0, "Charger plug in.");
    }
}
#endif

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    // register the battery level update event callback
    return 0;
}

#endif

WEAK void app_battery_opened_callback(void)
{

}

#else
int app_battery_open(void)
{
    return 0;
}

int app_battery_start(void)
{
    return 0;
}
#endif
/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include "cmsis_os.h"
#include "tgt_hardware.h"
#include "pmu.h"
#include "hal_timer.h"
#include "hal_gpadc.h"
#include "hal_trace.h"
#include "hal_gpio.h"
#include "hal_iomux.h"
#include "hal_chipid.h"
#include "app_thread.h"
#include "app_battery.h"
#include "audio_policy.h"
#include "app_bt.h"
#include "apps.h"
#include "app_hfp.h"

#ifdef APP_BATTERY_ENABLE
#include "app_status_ind.h"
#include "bluetooth_bt_api.h"
#include "app_media_player.h"
#ifdef BT_USB_AUDIO_DUAL_MODE
#include "btusb_audio.h"
#endif
#include <stdlib.h>

#ifdef __INTERCONNECTION__
#include "bluetooth_ble_api.h"
#endif

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
extern "C" bool app_usbaudio_mode_on(void);
#endif

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
#include CHIP_SPECIFIC_HDR(charger)
#endif

#define APP_BATTERY_TRACE(s,...)
// TRACE(s, ##__VA_ARGS__)

#ifndef APP_BATTERY_GPADC_CH_NUM
#define APP_BATTERY_GPADC_CH_NUM (HAL_GPADC_CHAN_BATTERY)
#endif

#ifndef APP_BATTERY_ERR_MV
#define APP_BATTERY_ERR_MV (2800)
#endif

#ifndef APP_BATTERY_MIN_MV
#define APP_BATTERY_MIN_MV (3200)
#endif

#ifndef APP_BATTERY_MAX_MV
#define APP_BATTERY_MAX_MV (4200)
#endif

#ifndef APP_BATTERY_PD_MV
#define APP_BATTERY_PD_MV   (3100)
#endif

#ifndef APP_BATTERY_CHARGE_TIMEOUT_MIN
#define APP_BATTERY_CHARGE_TIMEOUT_MIN (90)
#endif

#ifndef APP_BATTERY_CHARGE_OFFSET_MV
#define APP_BATTERY_CHARGE_OFFSET_MV (20)
#endif

#ifndef CHARGER_PLUGINOUT_RESET
#define CHARGER_PLUGINOUT_RESET (1)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

#define APP_BATTERY_CHARGING_PLUGOUT_DEDOUNCE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<500?3:1)

#define APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT (6)

#define APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT (3)

#define APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<20*1000?20*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT (6)


#define APP_BATTERY_REPORT_INTERVAL (5)

#define APP_BATTERY_MV_BASE ((APP_BATTERY_MAX_MV-APP_BATTERY_PD_MV)/(APP_BATTERY_LEVEL_NUM))

#define APP_BATTERY_STABLE_COUNT (5)
#define APP_BATTERY_MEASURE_PERIODIC_FAST_MS (200)
#ifdef BLE_ONLY_ENABLED
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (25000)
#else
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (10000)
#endif
#define APP_BATTERY_CHARGING_PERIODIC_MS (APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS)

#define APP_BATTERY_SET_MESSAGE(appevt, status, volt) (appevt = (((uint32_t)status&0xffff)<<16)|(volt&0xffff))
#define APP_BATTERY_GET_STATUS(appevt, status) (status = (appevt>>16)&0xffff)
#define APP_BATTERY_GET_VOLT(appevt, volt) (volt = appevt&0xffff)
#define APP_BATTERY_GET_PRAMS(appevt, prams) ((prams) = appevt&0xffff)

enum APP_BATTERY_MEASURE_PERIODIC_T
{
    APP_BATTERY_MEASURE_PERIODIC_FAST = 0,
    APP_BATTERY_MEASURE_PERIODIC_NORMAL,
    APP_BATTERY_MEASURE_PERIODIC_CHARGING,

    APP_BATTERY_MEASURE_PERIODIC_QTY,
};

struct APP_BATTERY_MEASURE_CHARGER_STATUS_T
{
    HAL_GPADC_MV_T prevolt;
    int32_t slope_1000[APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT];
    int slope_1000_index;
    int cnt;
};


typedef void (*APP_BATTERY_EVENT_CB_T)(enum APP_BATTERY_STATUS_T, APP_BATTERY_MV_T volt);

struct APP_BATTERY_MEASURE_T
{
    uint32_t start_time;
    enum APP_BATTERY_STATUS_T status;
#ifdef __INTERCONNECTION__
    uint8_t currentBatteryInfo;
    uint8_t lastBatteryInfo;
    uint8_t isMobileSupportSelfDefinedCommand;
#else
    uint8_t currlevel;
#endif
    APP_BATTERY_MV_T currvolt;
    APP_BATTERY_MV_T lowvolt;
    APP_BATTERY_MV_T highvolt;
    APP_BATTERY_MV_T pdvolt;
    uint32_t chargetimeout;
    enum APP_BATTERY_MEASURE_PERIODIC_T periodic;
    HAL_GPADC_MV_T voltage[APP_BATTERY_STABLE_COUNT];
    uint16_t index;
    struct APP_BATTERY_MEASURE_CHARGER_STATUS_T charger_status;
    APP_BATTERY_EVENT_CB_T cb;
    APP_BATTERY_CB_T user_cb;
};

#ifdef IS_BES_BATTERY_MANAGER_ENABLED

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void);

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static enum HAL_GPADC_CHAN_T app_vbat_ch = APP_BATTERY_GPADC_CH_NUM;
#ifdef CHARGER_SPICIAL_CHAN
static uint8_t app_vbat_volt_div = GPADC_VBAT_VOLT_DIV;
#endif
static void app_battery_timer_handler(void const *param);
osTimerDef (APP_BATTERY, app_battery_timer_handler);
static osTimerId app_battery_timer = NULL;
static struct APP_BATTERY_MEASURE_T app_battery_measure;

static int app_battery_charger_handle_process(void);

#ifdef __INTERCONNECTION__
uint8_t* app_battery_get_mobile_support_self_defined_command_p(void)
{
    return &app_battery_measure.isMobileSupportSelfDefinedCommand;
}
#endif


void app_battery_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint8_t i;
    uint32_t meanBattVolt = 0;
    HAL_GPADC_MV_T vbat = volt;
    APP_BATTERY_TRACE(2,"%s %d",__func__, vbat);
 #ifdef CHARGER_SPICIAL_CHAN
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat * app_vbat_volt_div) <= APP_BATTERY_ERR_MV))
#else
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat<<2) <= APP_BATTERY_ERR_MV))
#endif
    {
        app_battery_measure.cb(APP_BATTERY_STATUS_INVALID, vbat);
        return;
    }

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
    if(app_usbaudio_mode_on()) return ;
#endif
#ifdef CHARGER_SPICIAL_CHAN
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat * app_vbat_volt_div;
#else
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat<<2;
#endif 
    if (app_battery_measure.index > APP_BATTERY_STABLE_COUNT)
    {
        for (i=0; i<APP_BATTERY_STABLE_COUNT; i++)
        {
            meanBattVolt += app_battery_measure.voltage[i];
        }
        meanBattVolt /= APP_BATTERY_STABLE_COUNT;
        if (app_battery_measure.cb)
        {
            if (meanBattVolt>app_battery_measure.highvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_OVERVOLT, meanBattVolt);
            }
            else if((meanBattVolt>app_battery_measure.pdvolt) && (meanBattVolt<app_battery_measure.lowvolt))
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_UNDERVOLT, meanBattVolt);
            }
            else if(meanBattVolt<app_battery_measure.pdvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_PDVOLT, meanBattVolt);
            }
            else
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_NORMAL, meanBattVolt);
            }
        }
    }
    else
    {
        int8_t level = 0;
#ifdef CHARGER_SPICIAL_CHAN
         meanBattVolt = vbat * app_vbat_volt_div;
#else
         meanBattVolt = vbat<<2;
#endif
        level = (meanBattVolt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

        if (level<APP_BATTERY_LEVEL_MIN)
            level = APP_BATTERY_LEVEL_MIN;
        if (level>APP_BATTERY_LEVEL_MAX)
            level = APP_BATTERY_LEVEL_MAX;

        app_battery_measure.currvolt = meanBattVolt;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->batteryLevel = level;
#else
        app_battery_measure.currlevel = level;
#endif
    }
}

static void app_battery_timer_start(enum APP_BATTERY_MEASURE_PERIODIC_T periodic)
{
    uint32_t periodic_millisec = 0;

    if (app_battery_measure.periodic != periodic){
        app_battery_measure.periodic = periodic;
        switch (periodic)
        {
            case APP_BATTERY_MEASURE_PERIODIC_FAST:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_FAST_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_CHARGING:
                periodic_millisec = APP_BATTERY_CHARGING_PERIODIC_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_NORMAL:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS;
            default:
                break;
        }
        osTimerStop(app_battery_timer);
        osTimerStart(app_battery_timer, periodic_millisec);
    }
}

static void app_battery_timer_handler(void const *param)
{
#ifdef CHARGER_1802
    charger_vbat_div_adc_enable(true);
    hal_gpadc_open(HAL_GPADC_CHAN_5, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#else
    hal_gpadc_open(app_vbat_ch, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#endif
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);

}

#ifdef BT_BUILD_WITH_CUSTOMER_HOST
int app_status_battery_report(uint8_t level)
{
    return 0;
}
#else
int app_status_battery_report(uint8_t level)
{
#if defined(__BTIF_EARPHONE__)
    app_10_second_timer_check();
#endif

    if (app_is_stack_ready())
    {
// #if (HF_CUSTOM_FEATURE_SUPPORT & HF_CUSTOM_FEATURE_BATTERY_REPORT) || (HF_SDK_FEATURES & HF_FEATURE_HF_INDICATORS)
#if defined(SUPPORT_BATTERY_REPORT) || defined(SUPPORT_HF_INDICATORS)
#if defined(IBRT)
        uint8_t hfp_device = app_bt_audio_get_curr_hfp_device();
        struct BT_DEVICE_T *curr_device = app_bt_get_device(hfp_device);
        if (curr_device->hf_conn_flag)
        {
            app_hfp_set_battery_level(level);
        }
#elif defined(BT_HFP_SUPPORT)
        app_hfp_set_battery_level(level);
#endif
#else
        TRACE(1,"[%s] Can not enable SUPPORT_BATTERY_REPORT", __func__);
#endif
        bes_bt_osapi_notify_evm();
    }
    return 0;
}
#endif

int app_battery_handle_process_normal(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    int8_t level = 0;

    switch (status)
    {
        case APP_BATTERY_STATUS_UNDERVOLT:
            TRACE(1,"UNDERVOLT:%d", prams.volt);
            app_status_indication_set(APP_STATUS_INDICATION_CHARGENEED);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(IBRT)

#else
            media_PlayAudio(AUD_ID_BT_CHARGE_PLEASE, 0);
#endif
#endif
            // FALLTHROUGH
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_OVERVOLT:
            app_battery_measure.currvolt = prams.volt;
            level = (prams.volt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

            if (level<APP_BATTERY_LEVEL_MIN)
                level = APP_BATTERY_LEVEL_MIN;
            if (level>APP_BATTERY_LEVEL_MAX)
                level = APP_BATTERY_LEVEL_MAX;
#ifdef __INTERCONNECTION__
            APP_BATTERY_INFO_T* pBatteryInfo;
            pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
            pBatteryInfo->batteryLevel = level;
            if(level == APP_BATTERY_LEVEL_MAX)
            {
                level = 9;
            }
            else
            {
                level /= 10;
            }
#else
            app_battery_measure.currlevel = level;
#endif
            app_status_battery_report(level);
            break;
        case APP_BATTERY_STATUS_PDVOLT:
#ifndef BT_USB_AUDIO_DUAL_MODE
            TRACE(1,"PDVOLT-->POWEROFF:%d", prams.volt);
            osTimerStop(app_battery_timer);
            app_shutdown();
#endif
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING-->APP_BATTERY_CHARGER :%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#ifdef BT_USB_AUDIO_DUAL_MODE
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#else
#if CHARGER_PLUGINOUT_RESET
                app_reset();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
#endif
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_NORMAL);
    return 0;
}

int app_battery_handle_process_charging(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    switch (status)
    {
        case APP_BATTERY_STATUS_OVERVOLT:
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_UNDERVOLT:
            app_battery_measure.currvolt = prams.volt;
            app_status_battery_report(prams.volt);
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING:%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGOUT)
            {
#ifndef BT_USB_AUDIO_DUAL_MODE
#if CHARGER_PLUGINOUT_RESET
                TRACE(0,"CHARGING-->RESET");
                osTimerStop(app_battery_timer);
                app_shutdown();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#endif
#endif
            }
            else if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#if defined(BT_USB_AUDIO_DUAL_MODE)
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    if (app_battery_charger_handle_process()<=0)
    {
        if (app_status_indication_get() != APP_STATUS_INDICATION_FULLCHARGE)
        {
            TRACE(1,"FULL_CHARGING:%d", app_battery_measure.currvolt);
            app_status_indication_set(APP_STATUS_INDICATION_FULLCHARGE);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(BT_USB_AUDIO_DUAL_MODE) || defined(IBRT)
#else
            media_PlayAudio(AUD_ID_BT_CHARGE_FINISH, 0);
#endif
#endif
        }
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_CHARGING);

    return 0;
}

extern "C"     bool pmu_ana_volt_is_high(void);
static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;
    union APP_BATTERY_MSG_PRAMS msg_prams;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);
    APP_BATTERY_GET_PRAMS(msg_body->message_id, msg_prams.prams);

    uint32_t generatedSeed = hal_sys_timer_get();
    for (uint8_t index = 0; index < sizeof(bt_global_addr); index++)
    {
        generatedSeed ^= (((uint32_t)(bt_global_addr[index])) << (hal_sys_timer_get()&0xF));
    }
    srand(generatedSeed);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }
    else
    {
        switch (app_battery_measure.status)
        {
            case APP_BATTERY_STATUS_NORMAL:
                app_battery_handle_process_normal((uint32_t)status, msg_prams);
#if defined(CHIP_BEST1501P)
                if(pmu_ana_volt_is_high() == false){
                    pmu_ntc_capture_start(NULL);
                }else{
                    TRACE(1,"%s stop ana check",__func__);
                }
#endif
                break;

            case APP_BATTERY_STATUS_CHARGING:
                app_battery_handle_process_charging((uint32_t)status, msg_prams);
                break;

            default:
                break;
        }
    }
    if (NULL != app_battery_measure.user_cb)
    {
        uint8_t batteryLevel;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo;
        pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->chargingStatus = ((app_battery_measure.status == APP_BATTERY_STATUS_CHARGING)? 1:0);
        batteryLevel = pBatteryInfo->batteryLevel;

#else
        batteryLevel = app_battery_measure.currlevel;
#endif
        app_battery_measure.user_cb(app_battery_measure.currvolt,
                                    batteryLevel, app_battery_measure.status,status,msg_prams);
    }

    return 0;
}

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    if(NULL == app_battery_measure.user_cb)
    {
        app_battery_measure.user_cb = user_cb;
        return 0;
    }
    return 1;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    if (currvolt)
    {
        *currvolt = app_battery_measure.currvolt;
    }

    if (currlevel)
    {
#ifdef __INTERCONNECTION__
        *currlevel = app_battery_measure.currentBatteryInfo;
#else
        *currlevel = app_battery_measure.currlevel;
#endif
    }

    if (status)
    {
        *status = app_battery_measure.status;
    }

    return 0;
}

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
static void app_battery_gpadc_configuration_init(void)
{
    if (charger_package_type_get() == CHARGER_PACKAGE_TYPE_SIP_1620)
    {
        app_vbat_ch = HAL_GPADC_CHAN_BATTERY;
        app_vbat_volt_div = 4;
    }
}
#endif

int app_battery_open(void)
{
    APP_BATTERY_TRACE(3,"%s batt range:%d~%d",__func__, APP_BATTERY_MIN_MV, APP_BATTERY_MAX_MV);
    int nRet = APP_BATTERY_OPEN_MODE_INVALID;

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
    app_battery_gpadc_configuration_init();
#endif

    if (app_battery_timer == NULL)
        app_battery_timer = osTimerCreate (osTimer(APP_BATTERY), osTimerPeriodic, NULL);

    if (app_battery_pluginout_debounce_timer == NULL)
        app_battery_pluginout_debounce_timer = osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE), osTimerOnce, &app_battery_pluginout_debounce_ctx);

    app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#ifdef __INTERCONNECTION__
    app_battery_measure.currentBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.lastBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.isMobileSupportSelfDefinedCommand = 0;
#else
    app_battery_measure.currlevel = APP_BATTERY_LEVEL_MAX;
#endif
    app_battery_measure.currvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.lowvolt = APP_BATTERY_MIN_MV;
    app_battery_measure.highvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.pdvolt = APP_BATTERY_PD_MV;
    app_battery_measure.chargetimeout = APP_BATTERY_CHARGE_TIMEOUT_MIN;

    app_battery_measure.periodic = APP_BATTERY_MEASURE_PERIODIC_QTY;
    app_battery_measure.cb = app_battery_event_process;
    app_battery_measure.user_cb = NULL;

    app_battery_measure.charger_status.prevolt = 0;
    app_battery_measure.charger_status.slope_1000_index = 0;
    app_battery_measure.charger_status.cnt = 0;

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_IN, 1);
    }

    if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    }

    if (app_battery_charger_indication_open() == APP_BATTERY_CHARGER_PLUGIN)
    {
        app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
        app_battery_measure.start_time = hal_sys_timer_get();
        //pmu_charger_plugin_config();
        if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
        }

#if (CHARGER_PLUGINOUT_RESET == 0)
        nRet = APP_BATTERY_OPEN_MODE_CHARGING_PWRON;
#else
        nRet = APP_BATTERY_OPEN_MODE_CHARGING;
#endif
    }
    else
    {
        app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
        //pmu_charger_plugout_config();
        nRet = APP_BATTERY_OPEN_MODE_NORMAL;
    }
    return nRet;
}

int app_battery_start(void)
{
    APP_BATTERY_TRACE(2,"%s %d",__func__, APP_BATTERY_MEASURE_PERIODIC_FAST_MS);

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_FAST);

    return 0;
}

int app_battery_stop(void)
{
    osTimerStop(app_battery_timer);

    return 0;
}

int app_battery_close(void)
{
    hal_gpadc_close(HAL_GPADC_CHAN_BATTERY);

    return 0;
}


static int32_t app_battery_charger_slope_calc(int32_t t1, int32_t v1, int32_t t2, int32_t v2)
{
    int32_t slope_1000;
    slope_1000 = (v2-v1)*1000/(t2-t1);
    return slope_1000;
}

static int app_battery_charger_handle_process(void)
{
    int nRet = 1;
    int8_t i=0,cnt=0;
    uint32_t slope_1000 = 0;
    uint32_t charging_min;
    static uint8_t overvolt_full_charge_cnt = 0;
    static uint8_t ext_pin_full_charge_cnt = 0;

    charging_min = hal_sys_timer_get() - app_battery_measure.start_time;
    charging_min = TICKS_TO_MS(charging_min)/1000/60;
    if (charging_min >= app_battery_measure.chargetimeout)
    {
        // TRACE(0,"TIMEROUT-->FULL_CHARGING");
        nRet = -1;
        goto exit;
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT) == 0)
    {
        if (app_battery_measure.currvolt>=(app_battery_measure.highvolt+APP_BATTERY_CHARGE_OFFSET_MV))
        {
            overvolt_full_charge_cnt++;
        }
        else
        {
            overvolt_full_charge_cnt = 0;
        }
        if (overvolt_full_charge_cnt>=APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT)
        {
            //TRACE(0,"OVERVOLT-->FULL_CHARGING");
            nRet = -1;
            goto exit;
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT) == 0)
    {
        if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            if (hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
            {
                ext_pin_full_charge_cnt++;
            }
            else
            {
                ext_pin_full_charge_cnt = 0;
            }
            if (ext_pin_full_charge_cnt>=APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT)
            {
                TRACE(0,"EXT PIN-->FULL_CHARGING");
                nRet = -1;
                goto exit;
            }
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT) == 0)
    {
        if (!app_battery_measure.charger_status.prevolt)
        {
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                app_battery_measure.charger_status.slope_1000[i]=100;
            }
        }
        else
        {
            slope_1000 = app_battery_charger_slope_calc(0, app_battery_measure.charger_status.prevolt,
                         APP_BATTERY_CHARGING_PERIODIC_MS*APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT/1000, app_battery_measure.currvolt);
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                if (app_battery_measure.charger_status.slope_1000[i]>0)
                    cnt++;
                else
                    cnt--;
                TRACE(3,"slope_1000[%d]=%d cnt:%d", i,app_battery_measure.charger_status.slope_1000[i], cnt);
            }
            TRACE(3,"app_battery_charger_slope_proc slope*1000=%d cnt:%d nRet:%d", slope_1000, cnt, nRet);
            if (cnt>1)
            {
                nRet = 1;
            }/*else (3>=cnt && cnt>=-3){
                nRet = 0;
            }*/else
            {
                if (app_battery_measure.currvolt>=(app_battery_measure.highvolt-APP_BATTERY_CHARGE_OFFSET_MV))
                {
                    TRACE(0,"SLOPE-->FULL_CHARGING");
                    nRet = -1;
                }
            }
        }
        app_battery_measure.charger_status.slope_1000_index++;
    }
exit:
    return nRet;
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGIN");
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGOUT");
    }

    return status;
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);
#if defined(BT_USB_AUDIO_DUAL_MODE)
    btusb_switch(BTUSB_MODE_BT);
#endif
    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");
        if (status_charger == APP_BATTERY_CHARGER_PLUGIN)
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
            }
            app_battery_measure.start_time = hal_sys_timer_get();
        }
        else
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
            }
        }
        app_battery_event_process(APP_BATTERY_STATUS_CHARGING, status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    uint8_t cnt = 0;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    do
    {
        status = app_battery_charger_forcegetstatus();
        if (status == APP_BATTERY_CHARGER_PLUGIN)
            break;
        osDelay(20);
    }
    while(cnt++<5);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        if (!hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
        {
            status = APP_BATTERY_CHARGER_PLUGIN;
        }
    }

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}

int8_t app_battery_current_level(void)
{
#ifdef __INTERCONNECTION__
    return app_battery_measure.currentBatteryInfo & 0x7f;
#else
    return app_battery_measure.currlevel;
#endif
}

int8_t app_battery_is_charging(void)
{
    return (APP_BATTERY_STATUS_CHARGING == app_battery_measure.status);
}
typedef uint16_t NTP_VOLTAGE_MV_T;
typedef uint16_t NTP_TEMPERATURE_C_T;

#define NTC_CAPTURE_STABLE_COUNT (5)
#define NTC_CAPTURE_TEMPERATURE_STEP (4)
#define NTC_CAPTURE_TEMPERATURE_REF (15)
#define NTC_CAPTURE_VOLTAGE_REF (1100)

typedef void (*NTC_CAPTURE_MEASURE_CB_T)(NTP_TEMPERATURE_C_T);

struct NTC_CAPTURE_MEASURE_T
{
    NTP_TEMPERATURE_C_T temperature;
    NTP_VOLTAGE_MV_T currvolt;
    NTP_VOLTAGE_MV_T voltage[NTC_CAPTURE_STABLE_COUNT];
    uint16_t index;
    NTC_CAPTURE_MEASURE_CB_T cb;
};

static struct NTC_CAPTURE_MEASURE_T ntc_capture_measure;

void ntc_capture_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint32_t meanVolt = 0;
    TRACE(3,"%s %d irq:0x%04x",__func__, volt, irq_val);

    if (volt == HAL_GPADC_BAD_VALUE)
    {
        return;
    }

    ntc_capture_measure.voltage[ntc_capture_measure.index++%NTC_CAPTURE_STABLE_COUNT] = volt;

    if (ntc_capture_measure.index > NTC_CAPTURE_STABLE_COUNT)
    {
        for (uint8_t i=0; i<NTC_CAPTURE_STABLE_COUNT; i++)
        {
            meanVolt += ntc_capture_measure.voltage[i];
        }
        meanVolt /= NTC_CAPTURE_STABLE_COUNT;
        ntc_capture_measure.currvolt = meanVolt;
    }
    else if (!ntc_capture_measure.currvolt)
    {
        ntc_capture_measure.currvolt = volt;
    }
    ntc_capture_measure.temperature = ((int32_t)ntc_capture_measure.currvolt - NTC_CAPTURE_VOLTAGE_REF)/NTC_CAPTURE_TEMPERATURE_STEP + NTC_CAPTURE_TEMPERATURE_REF;
    pmu_ntc_capture_disable();
    TRACE(3,"%s ad:%d temperature:%d",__func__, ntc_capture_measure.currvolt, ntc_capture_measure.temperature);
}

int ntc_capture_open(void)
{

    ntc_capture_measure.currvolt = 0;
    ntc_capture_measure.index = 0;
    ntc_capture_measure.temperature = 0;
    ntc_capture_measure.cb = NULL;

    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}

int ntc_capture_start(void)
{
    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}
#else

#define IS_USE_SOC_PMU_PLUGINOUT

#ifdef IS_USE_SOC_PMU_PLUGINOUT

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }

    return 0;
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
    }

    return status;
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);

    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event);

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");

        app_battery_pluginout_event_callback(status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}
#endif

int app_battery_open(void)
{
    app_battery_opened_callback();
#ifdef IS_USE_SOC_PMU_PLUGINOUT
    if (app_battery_pluginout_debounce_timer == NULL)
    {
        app_battery_pluginout_debounce_timer =
            osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE),
            osTimerOnce, &app_battery_pluginout_debounce_ctx);
    }

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    app_battery_charger_indication_open();
#endif

    // initialize the custom battery manager here
    // returned value could be:
    // #define APP_BATTERY_OPEN_MODE_INVALID        (-1)
    // #define APP_BATTERY_OPEN_MODE_NORMAL         (0)
    // #define APP_BATTERY_OPEN_MODE_CHARGING       (1)
    // #define APP_BATTERY_OPEN_MODE_CHARGING_PWRON (2)
    return APP_BATTERY_OPEN_MODE_NORMAL;
}

int app_battery_start(void)
{
    // start battery measurement timer here
    return 0;
}

int app_battery_stop(void)
{
    // stop battery measurement timer here
    return 0;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    // should just return battery level via currlevel for hfp battery level indication
    *currlevel = APP_BATTERY_LEVEL_MAX;
    return 0;
}

#ifdef IS_USE_SOC_PMU_PLUGINOUT
static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event)
{
    if (APP_BATTERY_CHARGER_PLUGOUT == event)
    {
        TRACE(0, "Charger plug out.");
    }
    else if (APP_BATTERY_CHARGER_PLUGIN == event)
    {
        TRACE(0, "Charger plug in.");
    }
}
#endif

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    // register the battery level update event callback
    return 0;
}

#endif

WEAK void app_battery_opened_callback(void)
{

}

#else
int app_battery_open(void)
{
    return 0;
}

int app_battery_start(void)
{
    return 0;
}
#endif
/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include "cmsis_os.h"
#include "tgt_hardware.h"
#include "pmu.h"
#include "hal_timer.h"
#include "hal_gpadc.h"
#include "hal_trace.h"
#include "hal_gpio.h"
#include "hal_iomux.h"
#include "hal_chipid.h"
#include "app_thread.h"
#include "app_battery.h"
#include "audio_policy.h"
#include "app_bt.h"
#include "apps.h"
#include "app_hfp.h"

#ifdef APP_BATTERY_ENABLE
#include "app_status_ind.h"
#include "bluetooth_bt_api.h"
#include "app_media_player.h"
#ifdef BT_USB_AUDIO_DUAL_MODE
#include "btusb_audio.h"
#endif
#include <stdlib.h>

#ifdef __INTERCONNECTION__
#include "bluetooth_ble_api.h"
#endif

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
extern "C" bool app_usbaudio_mode_on(void);
#endif

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
#include CHIP_SPECIFIC_HDR(charger)
#endif

#define APP_BATTERY_TRACE(s,...)
// TRACE(s, ##__VA_ARGS__)

#ifndef APP_BATTERY_GPADC_CH_NUM
#define APP_BATTERY_GPADC_CH_NUM (HAL_GPADC_CHAN_BATTERY)
#endif

#ifndef APP_BATTERY_ERR_MV
#define APP_BATTERY_ERR_MV (2800)
#endif

#ifndef APP_BATTERY_MIN_MV
#define APP_BATTERY_MIN_MV (3200)
#endif

#ifndef APP_BATTERY_MAX_MV
#define APP_BATTERY_MAX_MV (4200)
#endif

#ifndef APP_BATTERY_PD_MV
#define APP_BATTERY_PD_MV   (3100)
#endif

#ifndef APP_BATTERY_CHARGE_TIMEOUT_MIN
#define APP_BATTERY_CHARGE_TIMEOUT_MIN (90)
#endif

#ifndef APP_BATTERY_CHARGE_OFFSET_MV
#define APP_BATTERY_CHARGE_OFFSET_MV (20)
#endif

#ifndef CHARGER_PLUGINOUT_RESET
#define CHARGER_PLUGINOUT_RESET (1)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

#define APP_BATTERY_CHARGING_PLUGOUT_DEDOUNCE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<500?3:1)

#define APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT (6)

#define APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT (3)

#define APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<20*1000?20*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT (6)


#define APP_BATTERY_REPORT_INTERVAL (5)

#define APP_BATTERY_MV_BASE ((APP_BATTERY_MAX_MV-APP_BATTERY_PD_MV)/(APP_BATTERY_LEVEL_NUM))

#define APP_BATTERY_STABLE_COUNT (5)
#define APP_BATTERY_MEASURE_PERIODIC_FAST_MS (200)
#ifdef BLE_ONLY_ENABLED
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (25000)
#else
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (10000)
#endif
#define APP_BATTERY_CHARGING_PERIODIC_MS (APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS)

#define APP_BATTERY_SET_MESSAGE(appevt, status, volt) (appevt = (((uint32_t)status&0xffff)<<16)|(volt&0xffff))
#define APP_BATTERY_GET_STATUS(appevt, status) (status = (appevt>>16)&0xffff)
#define APP_BATTERY_GET_VOLT(appevt, volt) (volt = appevt&0xffff)
#define APP_BATTERY_GET_PRAMS(appevt, prams) ((prams) = appevt&0xffff)

enum APP_BATTERY_MEASURE_PERIODIC_T
{
    APP_BATTERY_MEASURE_PERIODIC_FAST = 0,
    APP_BATTERY_MEASURE_PERIODIC_NORMAL,
    APP_BATTERY_MEASURE_PERIODIC_CHARGING,

    APP_BATTERY_MEASURE_PERIODIC_QTY,
};

struct APP_BATTERY_MEASURE_CHARGER_STATUS_T
{
    HAL_GPADC_MV_T prevolt;
    int32_t slope_1000[APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT];
    int slope_1000_index;
    int cnt;
};


typedef void (*APP_BATTERY_EVENT_CB_T)(enum APP_BATTERY_STATUS_T, APP_BATTERY_MV_T volt);

struct APP_BATTERY_MEASURE_T
{
    uint32_t start_time;
    enum APP_BATTERY_STATUS_T status;
#ifdef __INTERCONNECTION__
    uint8_t currentBatteryInfo;
    uint8_t lastBatteryInfo;
    uint8_t isMobileSupportSelfDefinedCommand;
#else
    uint8_t currlevel;
#endif
    APP_BATTERY_MV_T currvolt;
    APP_BATTERY_MV_T lowvolt;
    APP_BATTERY_MV_T highvolt;
    APP_BATTERY_MV_T pdvolt;
    uint32_t chargetimeout;
    enum APP_BATTERY_MEASURE_PERIODIC_T periodic;
    HAL_GPADC_MV_T voltage[APP_BATTERY_STABLE_COUNT];
    uint16_t index;
    struct APP_BATTERY_MEASURE_CHARGER_STATUS_T charger_status;
    APP_BATTERY_EVENT_CB_T cb;
    APP_BATTERY_CB_T user_cb;
};

#ifdef IS_BES_BATTERY_MANAGER_ENABLED

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void);

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static enum HAL_GPADC_CHAN_T app_vbat_ch = APP_BATTERY_GPADC_CH_NUM;
#ifdef CHARGER_SPICIAL_CHAN
static uint8_t app_vbat_volt_div = GPADC_VBAT_VOLT_DIV;
#endif
static void app_battery_timer_handler(void const *param);
osTimerDef (APP_BATTERY, app_battery_timer_handler);
static osTimerId app_battery_timer = NULL;
static struct APP_BATTERY_MEASURE_T app_battery_measure;

static int app_battery_charger_handle_process(void);

#ifdef __INTERCONNECTION__
uint8_t* app_battery_get_mobile_support_self_defined_command_p(void)
{
    return &app_battery_measure.isMobileSupportSelfDefinedCommand;
}
#endif


void app_battery_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint8_t i;
    uint32_t meanBattVolt = 0;
    HAL_GPADC_MV_T vbat = volt;
    APP_BATTERY_TRACE(2,"%s %d",__func__, vbat);
 #ifdef CHARGER_SPICIAL_CHAN
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat * app_vbat_volt_div) <= APP_BATTERY_ERR_MV))
#else
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat<<2) <= APP_BATTERY_ERR_MV))
#endif
    {
        app_battery_measure.cb(APP_BATTERY_STATUS_INVALID, vbat);
        return;
    }

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
    if(app_usbaudio_mode_on()) return ;
#endif
#ifdef CHARGER_SPICIAL_CHAN
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat * app_vbat_volt_div;
#else
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat<<2;
#endif 
    if (app_battery_measure.index > APP_BATTERY_STABLE_COUNT)
    {
        for (i=0; i<APP_BATTERY_STABLE_COUNT; i++)
        {
            meanBattVolt += app_battery_measure.voltage[i];
        }
        meanBattVolt /= APP_BATTERY_STABLE_COUNT;
        if (app_battery_measure.cb)
        {
            if (meanBattVolt>app_battery_measure.highvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_OVERVOLT, meanBattVolt);
            }
            else if((meanBattVolt>app_battery_measure.pdvolt) && (meanBattVolt<app_battery_measure.lowvolt))
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_UNDERVOLT, meanBattVolt);
            }
            else if(meanBattVolt<app_battery_measure.pdvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_PDVOLT, meanBattVolt);
            }
            else
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_NORMAL, meanBattVolt);
            }
        }
    }
    else
    {
        int8_t level = 0;
#ifdef CHARGER_SPICIAL_CHAN
         meanBattVolt = vbat * app_vbat_volt_div;
#else
         meanBattVolt = vbat<<2;
#endif
        level = (meanBattVolt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

        if (level<APP_BATTERY_LEVEL_MIN)
            level = APP_BATTERY_LEVEL_MIN;
        if (level>APP_BATTERY_LEVEL_MAX)
            level = APP_BATTERY_LEVEL_MAX;

        app_battery_measure.currvolt = meanBattVolt;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->batteryLevel = level;
#else
        app_battery_measure.currlevel = level;
#endif
    }
}

static void app_battery_timer_start(enum APP_BATTERY_MEASURE_PERIODIC_T periodic)
{
    uint32_t periodic_millisec = 0;

    if (app_battery_measure.periodic != periodic){
        app_battery_measure.periodic = periodic;
        switch (periodic)
        {
            case APP_BATTERY_MEASURE_PERIODIC_FAST:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_FAST_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_CHARGING:
                periodic_millisec = APP_BATTERY_CHARGING_PERIODIC_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_NORMAL:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS;
            default:
                break;
        }
        osTimerStop(app_battery_timer);
        osTimerStart(app_battery_timer, periodic_millisec);
    }
}

static void app_battery_timer_handler(void const *param)
{
#ifdef CHARGER_1802
    charger_vbat_div_adc_enable(true);
    hal_gpadc_open(HAL_GPADC_CHAN_5, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#else
    hal_gpadc_open(app_vbat_ch, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#endif
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);

}

#ifdef BT_BUILD_WITH_CUSTOMER_HOST
int app_status_battery_report(uint8_t level)
{
    return 0;
}
#else
int app_status_battery_report(uint8_t level)
{
#if defined(__BTIF_EARPHONE__)
    app_10_second_timer_check();
#endif

    if (app_is_stack_ready())
    {
// #if (HF_CUSTOM_FEATURE_SUPPORT & HF_CUSTOM_FEATURE_BATTERY_REPORT) || (HF_SDK_FEATURES & HF_FEATURE_HF_INDICATORS)
#if defined(SUPPORT_BATTERY_REPORT) || defined(SUPPORT_HF_INDICATORS)
#if defined(IBRT)
        uint8_t hfp_device = app_bt_audio_get_curr_hfp_device();
        struct BT_DEVICE_T *curr_device = app_bt_get_device(hfp_device);
        if (curr_device->hf_conn_flag)
        {
            app_hfp_set_battery_level(level);
        }
#elif defined(BT_HFP_SUPPORT)
        app_hfp_set_battery_level(level);
#endif
#else
        TRACE(1,"[%s] Can not enable SUPPORT_BATTERY_REPORT", __func__);
#endif
        bes_bt_osapi_notify_evm();
    }
    return 0;
}
#endif

int app_battery_handle_process_normal(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    int8_t level = 0;

    switch (status)
    {
        case APP_BATTERY_STATUS_UNDERVOLT:
            TRACE(1,"UNDERVOLT:%d", prams.volt);
            app_status_indication_set(APP_STATUS_INDICATION_CHARGENEED);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(IBRT)

#else
            media_PlayAudio(AUD_ID_BT_CHARGE_PLEASE, 0);
#endif
#endif
            // FALLTHROUGH
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_OVERVOLT:
            app_battery_measure.currvolt = prams.volt;
            level = (prams.volt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

            if (level<APP_BATTERY_LEVEL_MIN)
                level = APP_BATTERY_LEVEL_MIN;
            if (level>APP_BATTERY_LEVEL_MAX)
                level = APP_BATTERY_LEVEL_MAX;
#ifdef __INTERCONNECTION__
            APP_BATTERY_INFO_T* pBatteryInfo;
            pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
            pBatteryInfo->batteryLevel = level;
            if(level == APP_BATTERY_LEVEL_MAX)
            {
                level = 9;
            }
            else
            {
                level /= 10;
            }
#else
            app_battery_measure.currlevel = level;
#endif
            app_status_battery_report(level);
            break;
        case APP_BATTERY_STATUS_PDVOLT:
#ifndef BT_USB_AUDIO_DUAL_MODE
            TRACE(1,"PDVOLT-->POWEROFF:%d", prams.volt);
            osTimerStop(app_battery_timer);
            app_shutdown();
#endif
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING-->APP_BATTERY_CHARGER :%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#ifdef BT_USB_AUDIO_DUAL_MODE
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#else
#if CHARGER_PLUGINOUT_RESET
                app_reset();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
#endif
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_NORMAL);
    return 0;
}

int app_battery_handle_process_charging(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    switch (status)
    {
        case APP_BATTERY_STATUS_OVERVOLT:
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_UNDERVOLT:
            app_battery_measure.currvolt = prams.volt;
            app_status_battery_report(prams.volt);
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING:%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGOUT)
            {
#ifndef BT_USB_AUDIO_DUAL_MODE
#if CHARGER_PLUGINOUT_RESET
                TRACE(0,"CHARGING-->RESET");
                osTimerStop(app_battery_timer);
                app_shutdown();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#endif
#endif
            }
            else if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#if defined(BT_USB_AUDIO_DUAL_MODE)
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    if (app_battery_charger_handle_process()<=0)
    {
        if (app_status_indication_get() != APP_STATUS_INDICATION_FULLCHARGE)
        {
            TRACE(1,"FULL_CHARGING:%d", app_battery_measure.currvolt);
            app_status_indication_set(APP_STATUS_INDICATION_FULLCHARGE);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(BT_USB_AUDIO_DUAL_MODE) || defined(IBRT)
#else
            media_PlayAudio(AUD_ID_BT_CHARGE_FINISH, 0);
#endif
#endif
        }
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_CHARGING);

    return 0;
}

extern "C"     bool pmu_ana_volt_is_high(void);
static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;
    union APP_BATTERY_MSG_PRAMS msg_prams;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);
    APP_BATTERY_GET_PRAMS(msg_body->message_id, msg_prams.prams);

    uint32_t generatedSeed = hal_sys_timer_get();
    for (uint8_t index = 0; index < sizeof(bt_global_addr); index++)
    {
        generatedSeed ^= (((uint32_t)(bt_global_addr[index])) << (hal_sys_timer_get()&0xF));
    }
    srand(generatedSeed);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }
    else
    {
        switch (app_battery_measure.status)
        {
            case APP_BATTERY_STATUS_NORMAL:
                app_battery_handle_process_normal((uint32_t)status, msg_prams);
#if defined(CHIP_BEST1501P)
                if(pmu_ana_volt_is_high() == false){
                    pmu_ntc_capture_start(NULL);
                }else{
                    TRACE(1,"%s stop ana check",__func__);
                }
#endif
                break;

            case APP_BATTERY_STATUS_CHARGING:
                app_battery_handle_process_charging((uint32_t)status, msg_prams);
                break;

            default:
                break;
        }
    }
    if (NULL != app_battery_measure.user_cb)
    {
        uint8_t batteryLevel;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo;
        pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->chargingStatus = ((app_battery_measure.status == APP_BATTERY_STATUS_CHARGING)? 1:0);
        batteryLevel = pBatteryInfo->batteryLevel;

#else
        batteryLevel = app_battery_measure.currlevel;
#endif
        app_battery_measure.user_cb(app_battery_measure.currvolt,
                                    batteryLevel, app_battery_measure.status,status,msg_prams);
    }

    return 0;
}

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    if(NULL == app_battery_measure.user_cb)
    {
        app_battery_measure.user_cb = user_cb;
        return 0;
    }
    return 1;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    if (currvolt)
    {
        *currvolt = app_battery_measure.currvolt;
    }

    if (currlevel)
    {
#ifdef __INTERCONNECTION__
        *currlevel = app_battery_measure.currentBatteryInfo;
#else
        *currlevel = app_battery_measure.currlevel;
#endif
    }

    if (status)
    {
        *status = app_battery_measure.status;
    }

    return 0;
}

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
static void app_battery_gpadc_configuration_init(void)
{
    if (charger_package_type_get() == CHARGER_PACKAGE_TYPE_SIP_1620)
    {
        app_vbat_ch = HAL_GPADC_CHAN_BATTERY;
        app_vbat_volt_div = 4;
    }
}
#endif

int app_battery_open(void)
{
    APP_BATTERY_TRACE(3,"%s batt range:%d~%d",__func__, APP_BATTERY_MIN_MV, APP_BATTERY_MAX_MV);
    int nRet = APP_BATTERY_OPEN_MODE_INVALID;

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
    app_battery_gpadc_configuration_init();
#endif

    if (app_battery_timer == NULL)
        app_battery_timer = osTimerCreate (osTimer(APP_BATTERY), osTimerPeriodic, NULL);

    if (app_battery_pluginout_debounce_timer == NULL)
        app_battery_pluginout_debounce_timer = osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE), osTimerOnce, &app_battery_pluginout_debounce_ctx);

    app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#ifdef __INTERCONNECTION__
    app_battery_measure.currentBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.lastBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.isMobileSupportSelfDefinedCommand = 0;
#else
    app_battery_measure.currlevel = APP_BATTERY_LEVEL_MAX;
#endif
    app_battery_measure.currvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.lowvolt = APP_BATTERY_MIN_MV;
    app_battery_measure.highvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.pdvolt = APP_BATTERY_PD_MV;
    app_battery_measure.chargetimeout = APP_BATTERY_CHARGE_TIMEOUT_MIN;

    app_battery_measure.periodic = APP_BATTERY_MEASURE_PERIODIC_QTY;
    app_battery_measure.cb = app_battery_event_process;
    app_battery_measure.user_cb = NULL;

    app_battery_measure.charger_status.prevolt = 0;
    app_battery_measure.charger_status.slope_1000_index = 0;
    app_battery_measure.charger_status.cnt = 0;

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_IN, 1);
    }

    if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    }

    if (app_battery_charger_indication_open() == APP_BATTERY_CHARGER_PLUGIN)
    {
        app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
        app_battery_measure.start_time = hal_sys_timer_get();
        //pmu_charger_plugin_config();
        if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
        }

#if (CHARGER_PLUGINOUT_RESET == 0)
        nRet = APP_BATTERY_OPEN_MODE_CHARGING_PWRON;
#else
        nRet = APP_BATTERY_OPEN_MODE_CHARGING;
#endif
    }
    else
    {
        app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
        //pmu_charger_plugout_config();
        nRet = APP_BATTERY_OPEN_MODE_NORMAL;
    }
    return nRet;
}

int app_battery_start(void)
{
    APP_BATTERY_TRACE(2,"%s %d",__func__, APP_BATTERY_MEASURE_PERIODIC_FAST_MS);

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_FAST);

    return 0;
}

int app_battery_stop(void)
{
    osTimerStop(app_battery_timer);

    return 0;
}

int app_battery_close(void)
{
    hal_gpadc_close(HAL_GPADC_CHAN_BATTERY);

    return 0;
}


static int32_t app_battery_charger_slope_calc(int32_t t1, int32_t v1, int32_t t2, int32_t v2)
{
    int32_t slope_1000;
    slope_1000 = (v2-v1)*1000/(t2-t1);
    return slope_1000;
}

static int app_battery_charger_handle_process(void)
{
    int nRet = 1;
    int8_t i=0,cnt=0;
    uint32_t slope_1000 = 0;
    uint32_t charging_min;
    static uint8_t overvolt_full_charge_cnt = 0;
    static uint8_t ext_pin_full_charge_cnt = 0;

    charging_min = hal_sys_timer_get() - app_battery_measure.start_time;
    charging_min = TICKS_TO_MS(charging_min)/1000/60;
    if (charging_min >= app_battery_measure.chargetimeout)
    {
        // TRACE(0,"TIMEROUT-->FULL_CHARGING");
        nRet = -1;
        goto exit;
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT) == 0)
    {
        if (app_battery_measure.currvolt>=(app_battery_measure.highvolt+APP_BATTERY_CHARGE_OFFSET_MV))
        {
            overvolt_full_charge_cnt++;
        }
        else
        {
            overvolt_full_charge_cnt = 0;
        }
        if (overvolt_full_charge_cnt>=APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT)
        {
            //TRACE(0,"OVERVOLT-->FULL_CHARGING");
            nRet = -1;
            goto exit;
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT) == 0)
    {
        if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            if (hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
            {
                ext_pin_full_charge_cnt++;
            }
            else
            {
                ext_pin_full_charge_cnt = 0;
            }
            if (ext_pin_full_charge_cnt>=APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT)
            {
                TRACE(0,"EXT PIN-->FULL_CHARGING");
                nRet = -1;
                goto exit;
            }
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT) == 0)
    {
        if (!app_battery_measure.charger_status.prevolt)
        {
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                app_battery_measure.charger_status.slope_1000[i]=100;
            }
        }
        else
        {
            slope_1000 = app_battery_charger_slope_calc(0, app_battery_measure.charger_status.prevolt,
                         APP_BATTERY_CHARGING_PERIODIC_MS*APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT/1000, app_battery_measure.currvolt);
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                if (app_battery_measure.charger_status.slope_1000[i]>0)
                    cnt++;
                else
                    cnt--;
                TRACE(3,"slope_1000[%d]=%d cnt:%d", i,app_battery_measure.charger_status.slope_1000[i], cnt);
            }
            TRACE(3,"app_battery_charger_slope_proc slope*1000=%d cnt:%d nRet:%d", slope_1000, cnt, nRet);
            if (cnt>1)
            {
                nRet = 1;
            }/*else (3>=cnt && cnt>=-3){
                nRet = 0;
            }*/else
            {
                if (app_battery_measure.currvolt>=(app_battery_measure.highvolt-APP_BATTERY_CHARGE_OFFSET_MV))
                {
                    TRACE(0,"SLOPE-->FULL_CHARGING");
                    nRet = -1;
                }
            }
        }
        app_battery_measure.charger_status.slope_1000_index++;
    }
exit:
    return nRet;
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGIN");
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGOUT");
    }

    return status;
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);
#if defined(BT_USB_AUDIO_DUAL_MODE)
    btusb_switch(BTUSB_MODE_BT);
#endif
    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");
        if (status_charger == APP_BATTERY_CHARGER_PLUGIN)
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
            }
            app_battery_measure.start_time = hal_sys_timer_get();
        }
        else
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
            }
        }
        app_battery_event_process(APP_BATTERY_STATUS_CHARGING, status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    uint8_t cnt = 0;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    do
    {
        status = app_battery_charger_forcegetstatus();
        if (status == APP_BATTERY_CHARGER_PLUGIN)
            break;
        osDelay(20);
    }
    while(cnt++<5);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        if (!hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
        {
            status = APP_BATTERY_CHARGER_PLUGIN;
        }
    }

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}

int8_t app_battery_current_level(void)
{
#ifdef __INTERCONNECTION__
    return app_battery_measure.currentBatteryInfo & 0x7f;
#else
    return app_battery_measure.currlevel;
#endif
}

int8_t app_battery_is_charging(void)
{
    return (APP_BATTERY_STATUS_CHARGING == app_battery_measure.status);
}
typedef uint16_t NTP_VOLTAGE_MV_T;
typedef uint16_t NTP_TEMPERATURE_C_T;

#define NTC_CAPTURE_STABLE_COUNT (5)
#define NTC_CAPTURE_TEMPERATURE_STEP (4)
#define NTC_CAPTURE_TEMPERATURE_REF (15)
#define NTC_CAPTURE_VOLTAGE_REF (1100)

typedef void (*NTC_CAPTURE_MEASURE_CB_T)(NTP_TEMPERATURE_C_T);

struct NTC_CAPTURE_MEASURE_T
{
    NTP_TEMPERATURE_C_T temperature;
    NTP_VOLTAGE_MV_T currvolt;
    NTP_VOLTAGE_MV_T voltage[NTC_CAPTURE_STABLE_COUNT];
    uint16_t index;
    NTC_CAPTURE_MEASURE_CB_T cb;
};

static struct NTC_CAPTURE_MEASURE_T ntc_capture_measure;

void ntc_capture_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint32_t meanVolt = 0;
    TRACE(3,"%s %d irq:0x%04x",__func__, volt, irq_val);

    if (volt == HAL_GPADC_BAD_VALUE)
    {
        return;
    }

    ntc_capture_measure.voltage[ntc_capture_measure.index++%NTC_CAPTURE_STABLE_COUNT] = volt;

    if (ntc_capture_measure.index > NTC_CAPTURE_STABLE_COUNT)
    {
        for (uint8_t i=0; i<NTC_CAPTURE_STABLE_COUNT; i++)
        {
            meanVolt += ntc_capture_measure.voltage[i];
        }
        meanVolt /= NTC_CAPTURE_STABLE_COUNT;
        ntc_capture_measure.currvolt = meanVolt;
    }
    else if (!ntc_capture_measure.currvolt)
    {
        ntc_capture_measure.currvolt = volt;
    }
    ntc_capture_measure.temperature = ((int32_t)ntc_capture_measure.currvolt - NTC_CAPTURE_VOLTAGE_REF)/NTC_CAPTURE_TEMPERATURE_STEP + NTC_CAPTURE_TEMPERATURE_REF;
    pmu_ntc_capture_disable();
    TRACE(3,"%s ad:%d temperature:%d",__func__, ntc_capture_measure.currvolt, ntc_capture_measure.temperature);
}

int ntc_capture_open(void)
{

    ntc_capture_measure.currvolt = 0;
    ntc_capture_measure.index = 0;
    ntc_capture_measure.temperature = 0;
    ntc_capture_measure.cb = NULL;

    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}

int ntc_capture_start(void)
{
    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}
#else

#define IS_USE_SOC_PMU_PLUGINOUT

#ifdef IS_USE_SOC_PMU_PLUGINOUT

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }

    return 0;
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
    }

    return status;
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);

    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event);

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");

        app_battery_pluginout_event_callback(status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}
#endif

int app_battery_open(void)
{
    app_battery_opened_callback();
#ifdef IS_USE_SOC_PMU_PLUGINOUT
    if (app_battery_pluginout_debounce_timer == NULL)
    {
        app_battery_pluginout_debounce_timer =
            osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE),
            osTimerOnce, &app_battery_pluginout_debounce_ctx);
    }

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    app_battery_charger_indication_open();
#endif

    // initialize the custom battery manager here
    // returned value could be:
    // #define APP_BATTERY_OPEN_MODE_INVALID        (-1)
    // #define APP_BATTERY_OPEN_MODE_NORMAL         (0)
    // #define APP_BATTERY_OPEN_MODE_CHARGING       (1)
    // #define APP_BATTERY_OPEN_MODE_CHARGING_PWRON (2)
    return APP_BATTERY_OPEN_MODE_NORMAL;
}

int app_battery_start(void)
{
    // start battery measurement timer here
    return 0;
}

int app_battery_stop(void)
{
    // stop battery measurement timer here
    return 0;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    // should just return battery level via currlevel for hfp battery level indication
    *currlevel = APP_BATTERY_LEVEL_MAX;
    return 0;
}

#ifdef IS_USE_SOC_PMU_PLUGINOUT
static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event)
{
    if (APP_BATTERY_CHARGER_PLUGOUT == event)
    {
        TRACE(0, "Charger plug out.");
    }
    else if (APP_BATTERY_CHARGER_PLUGIN == event)
    {
        TRACE(0, "Charger plug in.");
    }
}
#endif

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    // register the battery level update event callback
    return 0;
}

#endif

WEAK void app_battery_opened_callback(void)
{

}

#else
int app_battery_open(void)
{
    return 0;
}

int app_battery_start(void)
{
    return 0;
}
#endif
/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include "cmsis_os.h"
#include "tgt_hardware.h"
#include "pmu.h"
#include "hal_timer.h"
#include "hal_gpadc.h"
#include "hal_trace.h"
#include "hal_gpio.h"
#include "hal_iomux.h"
#include "hal_chipid.h"
#include "app_thread.h"
#include "app_battery.h"
#include "audio_policy.h"
#include "app_bt.h"
#include "apps.h"
#include "app_hfp.h"

#ifdef APP_BATTERY_ENABLE
#include "app_status_ind.h"
#include "bluetooth_bt_api.h"
#include "app_media_player.h"
#ifdef BT_USB_AUDIO_DUAL_MODE
#include "btusb_audio.h"
#endif
#include <stdlib.h>

#ifdef __INTERCONNECTION__
#include "bluetooth_ble_api.h"
#endif

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
extern "C" bool app_usbaudio_mode_on(void);
#endif

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
#include CHIP_SPECIFIC_HDR(charger)
#endif

#define APP_BATTERY_TRACE(s,...)
// TRACE(s, ##__VA_ARGS__)

#ifndef APP_BATTERY_GPADC_CH_NUM
#define APP_BATTERY_GPADC_CH_NUM (HAL_GPADC_CHAN_BATTERY)
#endif

#ifndef APP_BATTERY_ERR_MV
#define APP_BATTERY_ERR_MV (2800)
#endif

#ifndef APP_BATTERY_MIN_MV
#define APP_BATTERY_MIN_MV (3200)
#endif

#ifndef APP_BATTERY_MAX_MV
#define APP_BATTERY_MAX_MV (4200)
#endif

#ifndef APP_BATTERY_PD_MV
#define APP_BATTERY_PD_MV   (3100)
#endif

#ifndef APP_BATTERY_CHARGE_TIMEOUT_MIN
#define APP_BATTERY_CHARGE_TIMEOUT_MIN (90)
#endif

#ifndef APP_BATTERY_CHARGE_OFFSET_MV
#define APP_BATTERY_CHARGE_OFFSET_MV (20)
#endif

#ifndef CHARGER_PLUGINOUT_RESET
#define CHARGER_PLUGINOUT_RESET (1)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

#define APP_BATTERY_CHARGING_PLUGOUT_DEDOUNCE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<500?3:1)

#define APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT (6)

#define APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT (3)

#define APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<20*1000?20*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT (6)


#define APP_BATTERY_REPORT_INTERVAL (5)

#define APP_BATTERY_MV_BASE ((APP_BATTERY_MAX_MV-APP_BATTERY_PD_MV)/(APP_BATTERY_LEVEL_NUM))

#define APP_BATTERY_STABLE_COUNT (5)
#define APP_BATTERY_MEASURE_PERIODIC_FAST_MS (200)
#ifdef BLE_ONLY_ENABLED
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (25000)
#else
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (10000)
#endif
#define APP_BATTERY_CHARGING_PERIODIC_MS (APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS)

#define APP_BATTERY_SET_MESSAGE(appevt, status, volt) (appevt = (((uint32_t)status&0xffff)<<16)|(volt&0xffff))
#define APP_BATTERY_GET_STATUS(appevt, status) (status = (appevt>>16)&0xffff)
#define APP_BATTERY_GET_VOLT(appevt, volt) (volt = appevt&0xffff)
#define APP_BATTERY_GET_PRAMS(appevt, prams) ((prams) = appevt&0xffff)

enum APP_BATTERY_MEASURE_PERIODIC_T
{
    APP_BATTERY_MEASURE_PERIODIC_FAST = 0,
    APP_BATTERY_MEASURE_PERIODIC_NORMAL,
    APP_BATTERY_MEASURE_PERIODIC_CHARGING,

    APP_BATTERY_MEASURE_PERIODIC_QTY,
};

struct APP_BATTERY_MEASURE_CHARGER_STATUS_T
{
    HAL_GPADC_MV_T prevolt;
    int32_t slope_1000[APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT];
    int slope_1000_index;
    int cnt;
};


typedef void (*APP_BATTERY_EVENT_CB_T)(enum APP_BATTERY_STATUS_T, APP_BATTERY_MV_T volt);

struct APP_BATTERY_MEASURE_T
{
    uint32_t start_time;
    enum APP_BATTERY_STATUS_T status;
#ifdef __INTERCONNECTION__
    uint8_t currentBatteryInfo;
    uint8_t lastBatteryInfo;
    uint8_t isMobileSupportSelfDefinedCommand;
#else
    uint8_t currlevel;
#endif
    APP_BATTERY_MV_T currvolt;
    APP_BATTERY_MV_T lowvolt;
    APP_BATTERY_MV_T highvolt;
    APP_BATTERY_MV_T pdvolt;
    uint32_t chargetimeout;
    enum APP_BATTERY_MEASURE_PERIODIC_T periodic;
    HAL_GPADC_MV_T voltage[APP_BATTERY_STABLE_COUNT];
    uint16_t index;
    struct APP_BATTERY_MEASURE_CHARGER_STATUS_T charger_status;
    APP_BATTERY_EVENT_CB_T cb;
    APP_BATTERY_CB_T user_cb;
};

#ifdef IS_BES_BATTERY_MANAGER_ENABLED

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void);

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static enum HAL_GPADC_CHAN_T app_vbat_ch = APP_BATTERY_GPADC_CH_NUM;
#ifdef CHARGER_SPICIAL_CHAN
static uint8_t app_vbat_volt_div = GPADC_VBAT_VOLT_DIV;
#endif
static void app_battery_timer_handler(void const *param);
osTimerDef (APP_BATTERY, app_battery_timer_handler);
static osTimerId app_battery_timer = NULL;
static struct APP_BATTERY_MEASURE_T app_battery_measure;

static int app_battery_charger_handle_process(void);

#ifdef __INTERCONNECTION__
uint8_t* app_battery_get_mobile_support_self_defined_command_p(void)
{
    return &app_battery_measure.isMobileSupportSelfDefinedCommand;
}
#endif


void app_battery_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint8_t i;
    uint32_t meanBattVolt = 0;
    HAL_GPADC_MV_T vbat = volt;
    APP_BATTERY_TRACE(2,"%s %d",__func__, vbat);
 #ifdef CHARGER_SPICIAL_CHAN
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat * app_vbat_volt_div) <= APP_BATTERY_ERR_MV))
#else
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat<<2) <= APP_BATTERY_ERR_MV))
#endif
    {
        app_battery_measure.cb(APP_BATTERY_STATUS_INVALID, vbat);
        return;
    }

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
    if(app_usbaudio_mode_on()) return ;
#endif
#ifdef CHARGER_SPICIAL_CHAN
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat * app_vbat_volt_div;
#else
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat<<2;
#endif 
    if (app_battery_measure.index > APP_BATTERY_STABLE_COUNT)
    {
        for (i=0; i<APP_BATTERY_STABLE_COUNT; i++)
        {
            meanBattVolt += app_battery_measure.voltage[i];
        }
        meanBattVolt /= APP_BATTERY_STABLE_COUNT;
        if (app_battery_measure.cb)
        {
            if (meanBattVolt>app_battery_measure.highvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_OVERVOLT, meanBattVolt);
            }
            else if((meanBattVolt>app_battery_measure.pdvolt) && (meanBattVolt<app_battery_measure.lowvolt))
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_UNDERVOLT, meanBattVolt);
            }
            else if(meanBattVolt<app_battery_measure.pdvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_PDVOLT, meanBattVolt);
            }
            else
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_NORMAL, meanBattVolt);
            }
        }
    }
    else
    {
        int8_t level = 0;
#ifdef CHARGER_SPICIAL_CHAN
         meanBattVolt = vbat * app_vbat_volt_div;
#else
         meanBattVolt = vbat<<2;
#endif
        level = (meanBattVolt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

        if (level<APP_BATTERY_LEVEL_MIN)
            level = APP_BATTERY_LEVEL_MIN;
        if (level>APP_BATTERY_LEVEL_MAX)
            level = APP_BATTERY_LEVEL_MAX;

        app_battery_measure.currvolt = meanBattVolt;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->batteryLevel = level;
#else
        app_battery_measure.currlevel = level;
#endif
    }
}

static void app_battery_timer_start(enum APP_BATTERY_MEASURE_PERIODIC_T periodic)
{
    uint32_t periodic_millisec = 0;

    if (app_battery_measure.periodic != periodic){
        app_battery_measure.periodic = periodic;
        switch (periodic)
        {
            case APP_BATTERY_MEASURE_PERIODIC_FAST:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_FAST_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_CHARGING:
                periodic_millisec = APP_BATTERY_CHARGING_PERIODIC_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_NORMAL:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS;
            default:
                break;
        }
        osTimerStop(app_battery_timer);
        osTimerStart(app_battery_timer, periodic_millisec);
    }
}

static void app_battery_timer_handler(void const *param)
{
#ifdef CHARGER_1802
    charger_vbat_div_adc_enable(true);
    hal_gpadc_open(HAL_GPADC_CHAN_5, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#else
    hal_gpadc_open(app_vbat_ch, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#endif
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);

}

#ifdef BT_BUILD_WITH_CUSTOMER_HOST
int app_status_battery_report(uint8_t level)
{
    return 0;
}
#else
int app_status_battery_report(uint8_t level)
{
#if defined(__BTIF_EARPHONE__)
    app_10_second_timer_check();
#endif

    if (app_is_stack_ready())
    {
// #if (HF_CUSTOM_FEATURE_SUPPORT & HF_CUSTOM_FEATURE_BATTERY_REPORT) || (HF_SDK_FEATURES & HF_FEATURE_HF_INDICATORS)
#if defined(SUPPORT_BATTERY_REPORT) || defined(SUPPORT_HF_INDICATORS)
#if defined(IBRT)
        uint8_t hfp_device = app_bt_audio_get_curr_hfp_device();
        struct BT_DEVICE_T *curr_device = app_bt_get_device(hfp_device);
        if (curr_device->hf_conn_flag)
        {
            app_hfp_set_battery_level(level);
        }
#elif defined(BT_HFP_SUPPORT)
        app_hfp_set_battery_level(level);
#endif
#else
        TRACE(1,"[%s] Can not enable SUPPORT_BATTERY_REPORT", __func__);
#endif
        bes_bt_osapi_notify_evm();
    }
    return 0;
}
#endif

int app_battery_handle_process_normal(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    int8_t level = 0;

    switch (status)
    {
        case APP_BATTERY_STATUS_UNDERVOLT:
            TRACE(1,"UNDERVOLT:%d", prams.volt);
            app_status_indication_set(APP_STATUS_INDICATION_CHARGENEED);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(IBRT)

#else
            media_PlayAudio(AUD_ID_BT_CHARGE_PLEASE, 0);
#endif
#endif
            // FALLTHROUGH
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_OVERVOLT:
            app_battery_measure.currvolt = prams.volt;
            level = (prams.volt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

            if (level<APP_BATTERY_LEVEL_MIN)
                level = APP_BATTERY_LEVEL_MIN;
            if (level>APP_BATTERY_LEVEL_MAX)
                level = APP_BATTERY_LEVEL_MAX;
#ifdef __INTERCONNECTION__
            APP_BATTERY_INFO_T* pBatteryInfo;
            pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
            pBatteryInfo->batteryLevel = level;
            if(level == APP_BATTERY_LEVEL_MAX)
            {
                level = 9;
            }
            else
            {
                level /= 10;
            }
#else
            app_battery_measure.currlevel = level;
#endif
            app_status_battery_report(level);
            break;
        case APP_BATTERY_STATUS_PDVOLT:
#ifndef BT_USB_AUDIO_DUAL_MODE
            TRACE(1,"PDVOLT-->POWEROFF:%d", prams.volt);
            osTimerStop(app_battery_timer);
            app_shutdown();
#endif
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING-->APP_BATTERY_CHARGER :%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#ifdef BT_USB_AUDIO_DUAL_MODE
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#else
#if CHARGER_PLUGINOUT_RESET
                app_reset();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
#endif
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_NORMAL);
    return 0;
}

int app_battery_handle_process_charging(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    switch (status)
    {
        case APP_BATTERY_STATUS_OVERVOLT:
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_UNDERVOLT:
            app_battery_measure.currvolt = prams.volt;
            app_status_battery_report(prams.volt);
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING:%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGOUT)
            {
#ifndef BT_USB_AUDIO_DUAL_MODE
#if CHARGER_PLUGINOUT_RESET
                TRACE(0,"CHARGING-->RESET");
                osTimerStop(app_battery_timer);
                app_shutdown();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#endif
#endif
            }
            else if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#if defined(BT_USB_AUDIO_DUAL_MODE)
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    if (app_battery_charger_handle_process()<=0)
    {
        if (app_status_indication_get() != APP_STATUS_INDICATION_FULLCHARGE)
        {
            TRACE(1,"FULL_CHARGING:%d", app_battery_measure.currvolt);
            app_status_indication_set(APP_STATUS_INDICATION_FULLCHARGE);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(BT_USB_AUDIO_DUAL_MODE) || defined(IBRT)
#else
            media_PlayAudio(AUD_ID_BT_CHARGE_FINISH, 0);
#endif
#endif
        }
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_CHARGING);

    return 0;
}

extern "C"     bool pmu_ana_volt_is_high(void);
static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;
    union APP_BATTERY_MSG_PRAMS msg_prams;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);
    APP_BATTERY_GET_PRAMS(msg_body->message_id, msg_prams.prams);

    uint32_t generatedSeed = hal_sys_timer_get();
    for (uint8_t index = 0; index < sizeof(bt_global_addr); index++)
    {
        generatedSeed ^= (((uint32_t)(bt_global_addr[index])) << (hal_sys_timer_get()&0xF));
    }
    srand(generatedSeed);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }
    else
    {
        switch (app_battery_measure.status)
        {
            case APP_BATTERY_STATUS_NORMAL:
                app_battery_handle_process_normal((uint32_t)status, msg_prams);
#if defined(CHIP_BEST1501P)
                if(pmu_ana_volt_is_high() == false){
                    pmu_ntc_capture_start(NULL);
                }else{
                    TRACE(1,"%s stop ana check",__func__);
                }
#endif
                break;

            case APP_BATTERY_STATUS_CHARGING:
                app_battery_handle_process_charging((uint32_t)status, msg_prams);
                break;

            default:
                break;
        }
    }
    if (NULL != app_battery_measure.user_cb)
    {
        uint8_t batteryLevel;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo;
        pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->chargingStatus = ((app_battery_measure.status == APP_BATTERY_STATUS_CHARGING)? 1:0);
        batteryLevel = pBatteryInfo->batteryLevel;

#else
        batteryLevel = app_battery_measure.currlevel;
#endif
        app_battery_measure.user_cb(app_battery_measure.currvolt,
                                    batteryLevel, app_battery_measure.status,status,msg_prams);
    }

    return 0;
}

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    if(NULL == app_battery_measure.user_cb)
    {
        app_battery_measure.user_cb = user_cb;
        return 0;
    }
    return 1;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    if (currvolt)
    {
        *currvolt = app_battery_measure.currvolt;
    }

    if (currlevel)
    {
#ifdef __INTERCONNECTION__
        *currlevel = app_battery_measure.currentBatteryInfo;
#else
        *currlevel = app_battery_measure.currlevel;
#endif
    }

    if (status)
    {
        *status = app_battery_measure.status;
    }

    return 0;
}

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
static void app_battery_gpadc_configuration_init(void)
{
    if (charger_package_type_get() == CHARGER_PACKAGE_TYPE_SIP_1620)
    {
        app_vbat_ch = HAL_GPADC_CHAN_BATTERY;
        app_vbat_volt_div = 4;
    }
}
#endif

int app_battery_open(void)
{
    APP_BATTERY_TRACE(3,"%s batt range:%d~%d",__func__, APP_BATTERY_MIN_MV, APP_BATTERY_MAX_MV);
    int nRet = APP_BATTERY_OPEN_MODE_INVALID;

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
    app_battery_gpadc_configuration_init();
#endif

    if (app_battery_timer == NULL)
        app_battery_timer = osTimerCreate (osTimer(APP_BATTERY), osTimerPeriodic, NULL);

    if (app_battery_pluginout_debounce_timer == NULL)
        app_battery_pluginout_debounce_timer = osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE), osTimerOnce, &app_battery_pluginout_debounce_ctx);

    app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#ifdef __INTERCONNECTION__
    app_battery_measure.currentBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.lastBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.isMobileSupportSelfDefinedCommand = 0;
#else
    app_battery_measure.currlevel = APP_BATTERY_LEVEL_MAX;
#endif
    app_battery_measure.currvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.lowvolt = APP_BATTERY_MIN_MV;
    app_battery_measure.highvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.pdvolt = APP_BATTERY_PD_MV;
    app_battery_measure.chargetimeout = APP_BATTERY_CHARGE_TIMEOUT_MIN;

    app_battery_measure.periodic = APP_BATTERY_MEASURE_PERIODIC_QTY;
    app_battery_measure.cb = app_battery_event_process;
    app_battery_measure.user_cb = NULL;

    app_battery_measure.charger_status.prevolt = 0;
    app_battery_measure.charger_status.slope_1000_index = 0;
    app_battery_measure.charger_status.cnt = 0;

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_IN, 1);
    }

    if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    }

    if (app_battery_charger_indication_open() == APP_BATTERY_CHARGER_PLUGIN)
    {
        app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
        app_battery_measure.start_time = hal_sys_timer_get();
        //pmu_charger_plugin_config();
        if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
        }

#if (CHARGER_PLUGINOUT_RESET == 0)
        nRet = APP_BATTERY_OPEN_MODE_CHARGING_PWRON;
#else
        nRet = APP_BATTERY_OPEN_MODE_CHARGING;
#endif
    }
    else
    {
        app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
        //pmu_charger_plugout_config();
        nRet = APP_BATTERY_OPEN_MODE_NORMAL;
    }
    return nRet;
}

int app_battery_start(void)
{
    APP_BATTERY_TRACE(2,"%s %d",__func__, APP_BATTERY_MEASURE_PERIODIC_FAST_MS);

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_FAST);

    return 0;
}

int app_battery_stop(void)
{
    osTimerStop(app_battery_timer);

    return 0;
}

int app_battery_close(void)
{
    hal_gpadc_close(HAL_GPADC_CHAN_BATTERY);

    return 0;
}


static int32_t app_battery_charger_slope_calc(int32_t t1, int32_t v1, int32_t t2, int32_t v2)
{
    int32_t slope_1000;
    slope_1000 = (v2-v1)*1000/(t2-t1);
    return slope_1000;
}

static int app_battery_charger_handle_process(void)
{
    int nRet = 1;
    int8_t i=0,cnt=0;
    uint32_t slope_1000 = 0;
    uint32_t charging_min;
    static uint8_t overvolt_full_charge_cnt = 0;
    static uint8_t ext_pin_full_charge_cnt = 0;

    charging_min = hal_sys_timer_get() - app_battery_measure.start_time;
    charging_min = TICKS_TO_MS(charging_min)/1000/60;
    if (charging_min >= app_battery_measure.chargetimeout)
    {
        // TRACE(0,"TIMEROUT-->FULL_CHARGING");
        nRet = -1;
        goto exit;
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT) == 0)
    {
        if (app_battery_measure.currvolt>=(app_battery_measure.highvolt+APP_BATTERY_CHARGE_OFFSET_MV))
        {
            overvolt_full_charge_cnt++;
        }
        else
        {
            overvolt_full_charge_cnt = 0;
        }
        if (overvolt_full_charge_cnt>=APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT)
        {
            //TRACE(0,"OVERVOLT-->FULL_CHARGING");
            nRet = -1;
            goto exit;
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT) == 0)
    {
        if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            if (hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
            {
                ext_pin_full_charge_cnt++;
            }
            else
            {
                ext_pin_full_charge_cnt = 0;
            }
            if (ext_pin_full_charge_cnt>=APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT)
            {
                TRACE(0,"EXT PIN-->FULL_CHARGING");
                nRet = -1;
                goto exit;
            }
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT) == 0)
    {
        if (!app_battery_measure.charger_status.prevolt)
        {
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                app_battery_measure.charger_status.slope_1000[i]=100;
            }
        }
        else
        {
            slope_1000 = app_battery_charger_slope_calc(0, app_battery_measure.charger_status.prevolt,
                         APP_BATTERY_CHARGING_PERIODIC_MS*APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT/1000, app_battery_measure.currvolt);
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                if (app_battery_measure.charger_status.slope_1000[i]>0)
                    cnt++;
                else
                    cnt--;
                TRACE(3,"slope_1000[%d]=%d cnt:%d", i,app_battery_measure.charger_status.slope_1000[i], cnt);
            }
            TRACE(3,"app_battery_charger_slope_proc slope*1000=%d cnt:%d nRet:%d", slope_1000, cnt, nRet);
            if (cnt>1)
            {
                nRet = 1;
            }/*else (3>=cnt && cnt>=-3){
                nRet = 0;
            }*/else
            {
                if (app_battery_measure.currvolt>=(app_battery_measure.highvolt-APP_BATTERY_CHARGE_OFFSET_MV))
                {
                    TRACE(0,"SLOPE-->FULL_CHARGING");
                    nRet = -1;
                }
            }
        }
        app_battery_measure.charger_status.slope_1000_index++;
    }
exit:
    return nRet;
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGIN");
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGOUT");
    }

    return status;
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);
#if defined(BT_USB_AUDIO_DUAL_MODE)
    btusb_switch(BTUSB_MODE_BT);
#endif
    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");
        if (status_charger == APP_BATTERY_CHARGER_PLUGIN)
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
            }
            app_battery_measure.start_time = hal_sys_timer_get();
        }
        else
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
            }
        }
        app_battery_event_process(APP_BATTERY_STATUS_CHARGING, status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    uint8_t cnt = 0;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    do
    {
        status = app_battery_charger_forcegetstatus();
        if (status == APP_BATTERY_CHARGER_PLUGIN)
            break;
        osDelay(20);
    }
    while(cnt++<5);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        if (!hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
        {
            status = APP_BATTERY_CHARGER_PLUGIN;
        }
    }

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}

int8_t app_battery_current_level(void)
{
#ifdef __INTERCONNECTION__
    return app_battery_measure.currentBatteryInfo & 0x7f;
#else
    return app_battery_measure.currlevel;
#endif
}

int8_t app_battery_is_charging(void)
{
    return (APP_BATTERY_STATUS_CHARGING == app_battery_measure.status);
}
typedef uint16_t NTP_VOLTAGE_MV_T;
typedef uint16_t NTP_TEMPERATURE_C_T;

#define NTC_CAPTURE_STABLE_COUNT (5)
#define NTC_CAPTURE_TEMPERATURE_STEP (4)
#define NTC_CAPTURE_TEMPERATURE_REF (15)
#define NTC_CAPTURE_VOLTAGE_REF (1100)

typedef void (*NTC_CAPTURE_MEASURE_CB_T)(NTP_TEMPERATURE_C_T);

struct NTC_CAPTURE_MEASURE_T
{
    NTP_TEMPERATURE_C_T temperature;
    NTP_VOLTAGE_MV_T currvolt;
    NTP_VOLTAGE_MV_T voltage[NTC_CAPTURE_STABLE_COUNT];
    uint16_t index;
    NTC_CAPTURE_MEASURE_CB_T cb;
};

static struct NTC_CAPTURE_MEASURE_T ntc_capture_measure;

void ntc_capture_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint32_t meanVolt = 0;
    TRACE(3,"%s %d irq:0x%04x",__func__, volt, irq_val);

    if (volt == HAL_GPADC_BAD_VALUE)
    {
        return;
    }

    ntc_capture_measure.voltage[ntc_capture_measure.index++%NTC_CAPTURE_STABLE_COUNT] = volt;

    if (ntc_capture_measure.index > NTC_CAPTURE_STABLE_COUNT)
    {
        for (uint8_t i=0; i<NTC_CAPTURE_STABLE_COUNT; i++)
        {
            meanVolt += ntc_capture_measure.voltage[i];
        }
        meanVolt /= NTC_CAPTURE_STABLE_COUNT;
        ntc_capture_measure.currvolt = meanVolt;
    }
    else if (!ntc_capture_measure.currvolt)
    {
        ntc_capture_measure.currvolt = volt;
    }
    ntc_capture_measure.temperature = ((int32_t)ntc_capture_measure.currvolt - NTC_CAPTURE_VOLTAGE_REF)/NTC_CAPTURE_TEMPERATURE_STEP + NTC_CAPTURE_TEMPERATURE_REF;
    pmu_ntc_capture_disable();
    TRACE(3,"%s ad:%d temperature:%d",__func__, ntc_capture_measure.currvolt, ntc_capture_measure.temperature);
}

int ntc_capture_open(void)
{

    ntc_capture_measure.currvolt = 0;
    ntc_capture_measure.index = 0;
    ntc_capture_measure.temperature = 0;
    ntc_capture_measure.cb = NULL;

    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}

int ntc_capture_start(void)
{
    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}
#else

#define IS_USE_SOC_PMU_PLUGINOUT

#ifdef IS_USE_SOC_PMU_PLUGINOUT

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }

    return 0;
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
    }

    return status;
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);

    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event);

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");

        app_battery_pluginout_event_callback(status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}
#endif

int app_battery_open(void)
{
    app_battery_opened_callback();
#ifdef IS_USE_SOC_PMU_PLUGINOUT
    if (app_battery_pluginout_debounce_timer == NULL)
    {
        app_battery_pluginout_debounce_timer =
            osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE),
            osTimerOnce, &app_battery_pluginout_debounce_ctx);
    }

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    app_battery_charger_indication_open();
#endif

    // initialize the custom battery manager here
    // returned value could be:
    // #define APP_BATTERY_OPEN_MODE_INVALID        (-1)
    // #define APP_BATTERY_OPEN_MODE_NORMAL         (0)
    // #define APP_BATTERY_OPEN_MODE_CHARGING       (1)
    // #define APP_BATTERY_OPEN_MODE_CHARGING_PWRON (2)
    return APP_BATTERY_OPEN_MODE_NORMAL;
}

int app_battery_start(void)
{
    // start battery measurement timer here
    return 0;
}

int app_battery_stop(void)
{
    // stop battery measurement timer here
    return 0;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    // should just return battery level via currlevel for hfp battery level indication
    *currlevel = APP_BATTERY_LEVEL_MAX;
    return 0;
}

#ifdef IS_USE_SOC_PMU_PLUGINOUT
static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event)
{
    if (APP_BATTERY_CHARGER_PLUGOUT == event)
    {
        TRACE(0, "Charger plug out.");
    }
    else if (APP_BATTERY_CHARGER_PLUGIN == event)
    {
        TRACE(0, "Charger plug in.");
    }
}
#endif

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    // register the battery level update event callback
    return 0;
}

#endif

WEAK void app_battery_opened_callback(void)
{

}

#else
int app_battery_open(void)
{
    return 0;
}

int app_battery_start(void)
{
    return 0;
}
#endif
/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include "cmsis_os.h"
#include "tgt_hardware.h"
#include "pmu.h"
#include "hal_timer.h"
#include "hal_gpadc.h"
#include "hal_trace.h"
#include "hal_gpio.h"
#include "hal_iomux.h"
#include "hal_chipid.h"
#include "app_thread.h"
#include "app_battery.h"
#include "audio_policy.h"
#include "app_bt.h"
#include "apps.h"
#include "app_hfp.h"

#ifdef APP_BATTERY_ENABLE
#include "app_status_ind.h"
#include "bluetooth_bt_api.h"
#include "app_media_player.h"
#ifdef BT_USB_AUDIO_DUAL_MODE
#include "btusb_audio.h"
#endif
#include <stdlib.h>

#ifdef __INTERCONNECTION__
#include "bluetooth_ble_api.h"
#endif

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
extern "C" bool app_usbaudio_mode_on(void);
#endif

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
#include CHIP_SPECIFIC_HDR(charger)
#endif

#define APP_BATTERY_TRACE(s,...)
// TRACE(s, ##__VA_ARGS__)

#ifndef APP_BATTERY_GPADC_CH_NUM
#define APP_BATTERY_GPADC_CH_NUM (HAL_GPADC_CHAN_BATTERY)
#endif

#ifndef APP_BATTERY_ERR_MV
#define APP_BATTERY_ERR_MV (2800)
#endif

#ifndef APP_BATTERY_MIN_MV
#define APP_BATTERY_MIN_MV (3200)
#endif

#ifndef APP_BATTERY_MAX_MV
#define APP_BATTERY_MAX_MV (4200)
#endif

#ifndef APP_BATTERY_PD_MV
#define APP_BATTERY_PD_MV   (3100)
#endif

#ifndef APP_BATTERY_CHARGE_TIMEOUT_MIN
#define APP_BATTERY_CHARGE_TIMEOUT_MIN (90)
#endif

#ifndef APP_BATTERY_CHARGE_OFFSET_MV
#define APP_BATTERY_CHARGE_OFFSET_MV (20)
#endif

#ifndef CHARGER_PLUGINOUT_RESET
#define CHARGER_PLUGINOUT_RESET (1)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

#define APP_BATTERY_CHARGING_PLUGOUT_DEDOUNCE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<500?3:1)

#define APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT (6)

#define APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<2*1000?2*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT (3)

#define APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT (APP_BATTERY_CHARGING_PERIODIC_MS<20*1000?20*1000/APP_BATTERY_CHARGING_PERIODIC_MS:1)
#define APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT (6)


#define APP_BATTERY_REPORT_INTERVAL (5)

#define APP_BATTERY_MV_BASE ((APP_BATTERY_MAX_MV-APP_BATTERY_PD_MV)/(APP_BATTERY_LEVEL_NUM))

#define APP_BATTERY_STABLE_COUNT (5)
#define APP_BATTERY_MEASURE_PERIODIC_FAST_MS (200)
#ifdef BLE_ONLY_ENABLED
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (25000)
#else
#define APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS (10000)
#endif
#define APP_BATTERY_CHARGING_PERIODIC_MS (APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS)

#define APP_BATTERY_SET_MESSAGE(appevt, status, volt) (appevt = (((uint32_t)status&0xffff)<<16)|(volt&0xffff))
#define APP_BATTERY_GET_STATUS(appevt, status) (status = (appevt>>16)&0xffff)
#define APP_BATTERY_GET_VOLT(appevt, volt) (volt = appevt&0xffff)
#define APP_BATTERY_GET_PRAMS(appevt, prams) ((prams) = appevt&0xffff)

enum APP_BATTERY_MEASURE_PERIODIC_T
{
    APP_BATTERY_MEASURE_PERIODIC_FAST = 0,
    APP_BATTERY_MEASURE_PERIODIC_NORMAL,
    APP_BATTERY_MEASURE_PERIODIC_CHARGING,

    APP_BATTERY_MEASURE_PERIODIC_QTY,
};

struct APP_BATTERY_MEASURE_CHARGER_STATUS_T
{
    HAL_GPADC_MV_T prevolt;
    int32_t slope_1000[APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT];
    int slope_1000_index;
    int cnt;
};


typedef void (*APP_BATTERY_EVENT_CB_T)(enum APP_BATTERY_STATUS_T, APP_BATTERY_MV_T volt);

struct APP_BATTERY_MEASURE_T
{
    uint32_t start_time;
    enum APP_BATTERY_STATUS_T status;
#ifdef __INTERCONNECTION__
    uint8_t currentBatteryInfo;
    uint8_t lastBatteryInfo;
    uint8_t isMobileSupportSelfDefinedCommand;
#else
    uint8_t currlevel;
#endif
    APP_BATTERY_MV_T currvolt;
    APP_BATTERY_MV_T lowvolt;
    APP_BATTERY_MV_T highvolt;
    APP_BATTERY_MV_T pdvolt;
    uint32_t chargetimeout;
    enum APP_BATTERY_MEASURE_PERIODIC_T periodic;
    HAL_GPADC_MV_T voltage[APP_BATTERY_STABLE_COUNT];
    uint16_t index;
    struct APP_BATTERY_MEASURE_CHARGER_STATUS_T charger_status;
    APP_BATTERY_EVENT_CB_T cb;
    APP_BATTERY_CB_T user_cb;
};

#ifdef IS_BES_BATTERY_MANAGER_ENABLED

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void);

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static enum HAL_GPADC_CHAN_T app_vbat_ch = APP_BATTERY_GPADC_CH_NUM;
#ifdef CHARGER_SPICIAL_CHAN
static uint8_t app_vbat_volt_div = GPADC_VBAT_VOLT_DIV;
#endif
static void app_battery_timer_handler(void const *param);
osTimerDef (APP_BATTERY, app_battery_timer_handler);
static osTimerId app_battery_timer = NULL;
static struct APP_BATTERY_MEASURE_T app_battery_measure;

static int app_battery_charger_handle_process(void);

#ifdef __INTERCONNECTION__
uint8_t* app_battery_get_mobile_support_self_defined_command_p(void)
{
    return &app_battery_measure.isMobileSupportSelfDefinedCommand;
}
#endif


void app_battery_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint8_t i;
    uint32_t meanBattVolt = 0;
    HAL_GPADC_MV_T vbat = volt;
    APP_BATTERY_TRACE(2,"%s %d",__func__, vbat);
 #ifdef CHARGER_SPICIAL_CHAN
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat * app_vbat_volt_div) <= APP_BATTERY_ERR_MV))
#else
    if ((vbat == HAL_GPADC_BAD_VALUE) || ((vbat<<2) <= APP_BATTERY_ERR_MV))
#endif
    {
        app_battery_measure.cb(APP_BATTERY_STATUS_INVALID, vbat);
        return;
    }

#if (defined(BTUSB_AUDIO_MODE) || defined(BTUSB_AUDIO_MODE))
    if(app_usbaudio_mode_on()) return ;
#endif
#ifdef CHARGER_SPICIAL_CHAN
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat * app_vbat_volt_div;
#else
    app_battery_measure.voltage[app_battery_measure.index++%APP_BATTERY_STABLE_COUNT] = vbat<<2;
#endif 
    if (app_battery_measure.index > APP_BATTERY_STABLE_COUNT)
    {
        for (i=0; i<APP_BATTERY_STABLE_COUNT; i++)
        {
            meanBattVolt += app_battery_measure.voltage[i];
        }
        meanBattVolt /= APP_BATTERY_STABLE_COUNT;
        if (app_battery_measure.cb)
        {
            if (meanBattVolt>app_battery_measure.highvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_OVERVOLT, meanBattVolt);
            }
            else if((meanBattVolt>app_battery_measure.pdvolt) && (meanBattVolt<app_battery_measure.lowvolt))
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_UNDERVOLT, meanBattVolt);
            }
            else if(meanBattVolt<app_battery_measure.pdvolt)
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_PDVOLT, meanBattVolt);
            }
            else
            {
                app_battery_measure.cb(APP_BATTERY_STATUS_NORMAL, meanBattVolt);
            }
        }
    }
    else
    {
        int8_t level = 0;
#ifdef CHARGER_SPICIAL_CHAN
         meanBattVolt = vbat * app_vbat_volt_div;
#else
         meanBattVolt = vbat<<2;
#endif
        level = (meanBattVolt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

        if (level<APP_BATTERY_LEVEL_MIN)
            level = APP_BATTERY_LEVEL_MIN;
        if (level>APP_BATTERY_LEVEL_MAX)
            level = APP_BATTERY_LEVEL_MAX;

        app_battery_measure.currvolt = meanBattVolt;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->batteryLevel = level;
#else
        app_battery_measure.currlevel = level;
#endif
    }
}

static void app_battery_timer_start(enum APP_BATTERY_MEASURE_PERIODIC_T periodic)
{
    uint32_t periodic_millisec = 0;

    if (app_battery_measure.periodic != periodic){
        app_battery_measure.periodic = periodic;
        switch (periodic)
        {
            case APP_BATTERY_MEASURE_PERIODIC_FAST:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_FAST_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_CHARGING:
                periodic_millisec = APP_BATTERY_CHARGING_PERIODIC_MS;
                break;
            case APP_BATTERY_MEASURE_PERIODIC_NORMAL:
                periodic_millisec = APP_BATTERY_MEASURE_PERIODIC_NORMAL_MS;
            default:
                break;
        }
        osTimerStop(app_battery_timer);
        osTimerStart(app_battery_timer, periodic_millisec);
    }
}

static void app_battery_timer_handler(void const *param)
{
#ifdef CHARGER_1802
    charger_vbat_div_adc_enable(true);
    hal_gpadc_open(HAL_GPADC_CHAN_5, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#else
    hal_gpadc_open(app_vbat_ch, HAL_GPADC_ATP_ONESHOT, app_battery_irqhandler);
#endif
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);

}

#ifdef BT_BUILD_WITH_CUSTOMER_HOST
int app_status_battery_report(uint8_t level)
{
    return 0;
}
#else
int app_status_battery_report(uint8_t level)
{
#if defined(__BTIF_EARPHONE__)
    app_10_second_timer_check();
#endif

    if (app_is_stack_ready())
    {
// #if (HF_CUSTOM_FEATURE_SUPPORT & HF_CUSTOM_FEATURE_BATTERY_REPORT) || (HF_SDK_FEATURES & HF_FEATURE_HF_INDICATORS)
#if defined(SUPPORT_BATTERY_REPORT) || defined(SUPPORT_HF_INDICATORS)
#if defined(IBRT)
        uint8_t hfp_device = app_bt_audio_get_curr_hfp_device();
        struct BT_DEVICE_T *curr_device = app_bt_get_device(hfp_device);
        if (curr_device->hf_conn_flag)
        {
            app_hfp_set_battery_level(level);
        }
#elif defined(BT_HFP_SUPPORT)
        app_hfp_set_battery_level(level);
#endif
#else
        TRACE(1,"[%s] Can not enable SUPPORT_BATTERY_REPORT", __func__);
#endif
        bes_bt_osapi_notify_evm();
    }
    return 0;
}
#endif

int app_battery_handle_process_normal(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    int8_t level = 0;

    switch (status)
    {
        case APP_BATTERY_STATUS_UNDERVOLT:
            TRACE(1,"UNDERVOLT:%d", prams.volt);
            app_status_indication_set(APP_STATUS_INDICATION_CHARGENEED);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(IBRT)

#else
            media_PlayAudio(AUD_ID_BT_CHARGE_PLEASE, 0);
#endif
#endif
            // FALLTHROUGH
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_OVERVOLT:
            app_battery_measure.currvolt = prams.volt;
            level = (prams.volt-APP_BATTERY_PD_MV)/APP_BATTERY_MV_BASE;

            if (level<APP_BATTERY_LEVEL_MIN)
                level = APP_BATTERY_LEVEL_MIN;
            if (level>APP_BATTERY_LEVEL_MAX)
                level = APP_BATTERY_LEVEL_MAX;
#ifdef __INTERCONNECTION__
            APP_BATTERY_INFO_T* pBatteryInfo;
            pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
            pBatteryInfo->batteryLevel = level;
            if(level == APP_BATTERY_LEVEL_MAX)
            {
                level = 9;
            }
            else
            {
                level /= 10;
            }
#else
            app_battery_measure.currlevel = level;
#endif
            app_status_battery_report(level);
            break;
        case APP_BATTERY_STATUS_PDVOLT:
#ifndef BT_USB_AUDIO_DUAL_MODE
            TRACE(1,"PDVOLT-->POWEROFF:%d", prams.volt);
            osTimerStop(app_battery_timer);
            app_shutdown();
#endif
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING-->APP_BATTERY_CHARGER :%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#ifdef BT_USB_AUDIO_DUAL_MODE
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#else
#if CHARGER_PLUGINOUT_RESET
                app_reset();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
#endif
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_NORMAL);
    return 0;
}

int app_battery_handle_process_charging(uint32_t status,  union APP_BATTERY_MSG_PRAMS prams)
{
    switch (status)
    {
        case APP_BATTERY_STATUS_OVERVOLT:
        case APP_BATTERY_STATUS_NORMAL:
        case APP_BATTERY_STATUS_UNDERVOLT:
            app_battery_measure.currvolt = prams.volt;
            app_status_battery_report(prams.volt);
            break;
        case APP_BATTERY_STATUS_CHARGING:
            TRACE(1,"CHARGING:%d", prams.charger);
            if (prams.charger == APP_BATTERY_CHARGER_PLUGOUT)
            {
#ifndef BT_USB_AUDIO_DUAL_MODE
#if CHARGER_PLUGINOUT_RESET
                TRACE(0,"CHARGING-->RESET");
                osTimerStop(app_battery_timer);
                app_shutdown();
#else
                app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#endif
#endif
            }
            else if (prams.charger == APP_BATTERY_CHARGER_PLUGIN)
            {
#if defined(BT_USB_AUDIO_DUAL_MODE)
                TRACE(1,"%s:PLUGIN.", __func__);
                btusb_switch(BTUSB_MODE_USB);
#endif
            }
            break;
        case APP_BATTERY_STATUS_INVALID:
        default:
            break;
    }

    if (app_battery_charger_handle_process()<=0)
    {
        if (app_status_indication_get() != APP_STATUS_INDICATION_FULLCHARGE)
        {
            TRACE(1,"FULL_CHARGING:%d", app_battery_measure.currvolt);
            app_status_indication_set(APP_STATUS_INDICATION_FULLCHARGE);
#ifdef MEDIA_PLAYER_SUPPORT
#if defined(BT_USB_AUDIO_DUAL_MODE) || defined(IBRT)
#else
            media_PlayAudio(AUD_ID_BT_CHARGE_FINISH, 0);
#endif
#endif
        }
    }

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_CHARGING);

    return 0;
}

extern "C"     bool pmu_ana_volt_is_high(void);
static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;
    union APP_BATTERY_MSG_PRAMS msg_prams;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);
    APP_BATTERY_GET_PRAMS(msg_body->message_id, msg_prams.prams);

    uint32_t generatedSeed = hal_sys_timer_get();
    for (uint8_t index = 0; index < sizeof(bt_global_addr); index++)
    {
        generatedSeed ^= (((uint32_t)(bt_global_addr[index])) << (hal_sys_timer_get()&0xF));
    }
    srand(generatedSeed);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }
    else
    {
        switch (app_battery_measure.status)
        {
            case APP_BATTERY_STATUS_NORMAL:
                app_battery_handle_process_normal((uint32_t)status, msg_prams);
#if defined(CHIP_BEST1501P)
                if(pmu_ana_volt_is_high() == false){
                    pmu_ntc_capture_start(NULL);
                }else{
                    TRACE(1,"%s stop ana check",__func__);
                }
#endif
                break;

            case APP_BATTERY_STATUS_CHARGING:
                app_battery_handle_process_charging((uint32_t)status, msg_prams);
                break;

            default:
                break;
        }
    }
    if (NULL != app_battery_measure.user_cb)
    {
        uint8_t batteryLevel;
#ifdef __INTERCONNECTION__
        APP_BATTERY_INFO_T* pBatteryInfo;
        pBatteryInfo = (APP_BATTERY_INFO_T*)&app_battery_measure.currentBatteryInfo;
        pBatteryInfo->chargingStatus = ((app_battery_measure.status == APP_BATTERY_STATUS_CHARGING)? 1:0);
        batteryLevel = pBatteryInfo->batteryLevel;

#else
        batteryLevel = app_battery_measure.currlevel;
#endif
        app_battery_measure.user_cb(app_battery_measure.currvolt,
                                    batteryLevel, app_battery_measure.status,status,msg_prams);
    }

    return 0;
}

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    if(NULL == app_battery_measure.user_cb)
    {
        app_battery_measure.user_cb = user_cb;
        return 0;
    }
    return 1;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    if (currvolt)
    {
        *currvolt = app_battery_measure.currvolt;
    }

    if (currlevel)
    {
#ifdef __INTERCONNECTION__
        *currlevel = app_battery_measure.currentBatteryInfo;
#else
        *currlevel = app_battery_measure.currlevel;
#endif
    }

    if (status)
    {
        *status = app_battery_measure.status;
    }

    return 0;
}

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
static void app_battery_gpadc_configuration_init(void)
{
    if (charger_package_type_get() == CHARGER_PACKAGE_TYPE_SIP_1620)
    {
        app_vbat_ch = HAL_GPADC_CHAN_BATTERY;
        app_vbat_volt_div = 4;
    }
}
#endif

int app_battery_open(void)
{
    APP_BATTERY_TRACE(3,"%s batt range:%d~%d",__func__, APP_BATTERY_MIN_MV, APP_BATTERY_MAX_MV);
    int nRet = APP_BATTERY_OPEN_MODE_INVALID;

#ifdef MORE_THAN_ONE_TYPE_OF_CHARGER
    app_battery_gpadc_configuration_init();
#endif

    if (app_battery_timer == NULL)
        app_battery_timer = osTimerCreate (osTimer(APP_BATTERY), osTimerPeriodic, NULL);

    if (app_battery_pluginout_debounce_timer == NULL)
        app_battery_pluginout_debounce_timer = osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE), osTimerOnce, &app_battery_pluginout_debounce_ctx);

    app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
#ifdef __INTERCONNECTION__
    app_battery_measure.currentBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.lastBatteryInfo = APP_BATTERY_DEFAULT_INFO;
    app_battery_measure.isMobileSupportSelfDefinedCommand = 0;
#else
    app_battery_measure.currlevel = APP_BATTERY_LEVEL_MAX;
#endif
    app_battery_measure.currvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.lowvolt = APP_BATTERY_MIN_MV;
    app_battery_measure.highvolt = APP_BATTERY_MAX_MV;
    app_battery_measure.pdvolt = APP_BATTERY_PD_MV;
    app_battery_measure.chargetimeout = APP_BATTERY_CHARGE_TIMEOUT_MIN;

    app_battery_measure.periodic = APP_BATTERY_MEASURE_PERIODIC_QTY;
    app_battery_measure.cb = app_battery_event_process;
    app_battery_measure.user_cb = NULL;

    app_battery_measure.charger_status.prevolt = 0;
    app_battery_measure.charger_status.slope_1000_index = 0;
    app_battery_measure.charger_status.cnt = 0;

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_IN, 1);
    }

    if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_battery_ext_charger_detecter_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    }

    if (app_battery_charger_indication_open() == APP_BATTERY_CHARGER_PLUGIN)
    {
        app_battery_measure.status = APP_BATTERY_STATUS_CHARGING;
        app_battery_measure.start_time = hal_sys_timer_get();
        //pmu_charger_plugin_config();
        if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
        }

#if (CHARGER_PLUGINOUT_RESET == 0)
        nRet = APP_BATTERY_OPEN_MODE_CHARGING_PWRON;
#else
        nRet = APP_BATTERY_OPEN_MODE_CHARGING;
#endif
    }
    else
    {
        app_battery_measure.status = APP_BATTERY_STATUS_NORMAL;
        //pmu_charger_plugout_config();
        nRet = APP_BATTERY_OPEN_MODE_NORMAL;
    }
    return nRet;
}

int app_battery_start(void)
{
    APP_BATTERY_TRACE(2,"%s %d",__func__, APP_BATTERY_MEASURE_PERIODIC_FAST_MS);

    app_battery_timer_start(APP_BATTERY_MEASURE_PERIODIC_FAST);

    return 0;
}

int app_battery_stop(void)
{
    osTimerStop(app_battery_timer);

    return 0;
}

int app_battery_close(void)
{
    hal_gpadc_close(HAL_GPADC_CHAN_BATTERY);

    return 0;
}


static int32_t app_battery_charger_slope_calc(int32_t t1, int32_t v1, int32_t t2, int32_t v2)
{
    int32_t slope_1000;
    slope_1000 = (v2-v1)*1000/(t2-t1);
    return slope_1000;
}

static int app_battery_charger_handle_process(void)
{
    int nRet = 1;
    int8_t i=0,cnt=0;
    uint32_t slope_1000 = 0;
    uint32_t charging_min;
    static uint8_t overvolt_full_charge_cnt = 0;
    static uint8_t ext_pin_full_charge_cnt = 0;

    charging_min = hal_sys_timer_get() - app_battery_measure.start_time;
    charging_min = TICKS_TO_MS(charging_min)/1000/60;
    if (charging_min >= app_battery_measure.chargetimeout)
    {
        // TRACE(0,"TIMEROUT-->FULL_CHARGING");
        nRet = -1;
        goto exit;
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_OVERVOLT_MEASURE_CNT) == 0)
    {
        if (app_battery_measure.currvolt>=(app_battery_measure.highvolt+APP_BATTERY_CHARGE_OFFSET_MV))
        {
            overvolt_full_charge_cnt++;
        }
        else
        {
            overvolt_full_charge_cnt = 0;
        }
        if (overvolt_full_charge_cnt>=APP_BATTERY_CHARGING_OVERVOLT_DEDOUNCE_CNT)
        {
            //TRACE(0,"OVERVOLT-->FULL_CHARGING");
            nRet = -1;
            goto exit;
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_EXTPIN_MEASURE_CNT) == 0)
    {
        if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
        {
            if (hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
            {
                ext_pin_full_charge_cnt++;
            }
            else
            {
                ext_pin_full_charge_cnt = 0;
            }
            if (ext_pin_full_charge_cnt>=APP_BATTERY_CHARGING_EXTPIN_DEDOUNCE_CNT)
            {
                TRACE(0,"EXT PIN-->FULL_CHARGING");
                nRet = -1;
                goto exit;
            }
        }
    }

    if ((app_battery_measure.charger_status.cnt++%APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT) == 0)
    {
        if (!app_battery_measure.charger_status.prevolt)
        {
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                app_battery_measure.charger_status.slope_1000[i]=100;
            }
        }
        else
        {
            slope_1000 = app_battery_charger_slope_calc(0, app_battery_measure.charger_status.prevolt,
                         APP_BATTERY_CHARGING_PERIODIC_MS*APP_BATTERY_CHARGING_SLOPE_MEASURE_CNT/1000, app_battery_measure.currvolt);
            app_battery_measure.charger_status.slope_1000[app_battery_measure.charger_status.slope_1000_index%APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT] = slope_1000;
            app_battery_measure.charger_status.prevolt = app_battery_measure.currvolt;
            for (i=0; i<APP_BATTERY_CHARGING_SLOPE_TABLE_COUNT; i++)
            {
                if (app_battery_measure.charger_status.slope_1000[i]>0)
                    cnt++;
                else
                    cnt--;
                TRACE(3,"slope_1000[%d]=%d cnt:%d", i,app_battery_measure.charger_status.slope_1000[i], cnt);
            }
            TRACE(3,"app_battery_charger_slope_proc slope*1000=%d cnt:%d nRet:%d", slope_1000, cnt, nRet);
            if (cnt>1)
            {
                nRet = 1;
            }/*else (3>=cnt && cnt>=-3){
                nRet = 0;
            }*/else
            {
                if (app_battery_measure.currvolt>=(app_battery_measure.highvolt-APP_BATTERY_CHARGE_OFFSET_MV))
                {
                    TRACE(0,"SLOPE-->FULL_CHARGING");
                    nRet = -1;
                }
            }
        }
        app_battery_measure.charger_status.slope_1000_index++;
    }
exit:
    return nRet;
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGIN");
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
        // TRACE(0,"force APP_BATTERY_CHARGER_PLUGOUT");
    }

    return status;
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);
#if defined(BT_USB_AUDIO_DUAL_MODE)
    btusb_switch(BTUSB_MODE_BT);
#endif
    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");
        if (status_charger == APP_BATTERY_CHARGER_PLUGIN)
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 0);
            }
            app_battery_measure.start_time = hal_sys_timer_get();
        }
        else
        {
            if (app_battery_ext_charger_enable_cfg.pin != HAL_IOMUX_PIN_NUM)
            {
                hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin, HAL_GPIO_DIR_OUT, 1);
            }
        }
        app_battery_event_process(APP_BATTERY_STATUS_CHARGING, status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    uint8_t cnt = 0;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    do
    {
        status = app_battery_charger_forcegetstatus();
        if (status == APP_BATTERY_CHARGER_PLUGIN)
            break;
        osDelay(20);
    }
    while(cnt++<5);

    if (app_battery_ext_charger_detecter_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        if (!hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_battery_ext_charger_detecter_cfg.pin))
        {
            status = APP_BATTERY_CHARGER_PLUGIN;
        }
    }

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}

int8_t app_battery_current_level(void)
{
#ifdef __INTERCONNECTION__
    return app_battery_measure.currentBatteryInfo & 0x7f;
#else
    return app_battery_measure.currlevel;
#endif
}

int8_t app_battery_is_charging(void)
{
    return (APP_BATTERY_STATUS_CHARGING == app_battery_measure.status);
}
typedef uint16_t NTP_VOLTAGE_MV_T;
typedef uint16_t NTP_TEMPERATURE_C_T;

#define NTC_CAPTURE_STABLE_COUNT (5)
#define NTC_CAPTURE_TEMPERATURE_STEP (4)
#define NTC_CAPTURE_TEMPERATURE_REF (15)
#define NTC_CAPTURE_VOLTAGE_REF (1100)

typedef void (*NTC_CAPTURE_MEASURE_CB_T)(NTP_TEMPERATURE_C_T);

struct NTC_CAPTURE_MEASURE_T
{
    NTP_TEMPERATURE_C_T temperature;
    NTP_VOLTAGE_MV_T currvolt;
    NTP_VOLTAGE_MV_T voltage[NTC_CAPTURE_STABLE_COUNT];
    uint16_t index;
    NTC_CAPTURE_MEASURE_CB_T cb;
};

static struct NTC_CAPTURE_MEASURE_T ntc_capture_measure;

void ntc_capture_irqhandler(uint16_t irq_val, HAL_GPADC_MV_T volt)
{
    uint32_t meanVolt = 0;
    TRACE(3,"%s %d irq:0x%04x",__func__, volt, irq_val);

    if (volt == HAL_GPADC_BAD_VALUE)
    {
        return;
    }

    ntc_capture_measure.voltage[ntc_capture_measure.index++%NTC_CAPTURE_STABLE_COUNT] = volt;

    if (ntc_capture_measure.index > NTC_CAPTURE_STABLE_COUNT)
    {
        for (uint8_t i=0; i<NTC_CAPTURE_STABLE_COUNT; i++)
        {
            meanVolt += ntc_capture_measure.voltage[i];
        }
        meanVolt /= NTC_CAPTURE_STABLE_COUNT;
        ntc_capture_measure.currvolt = meanVolt;
    }
    else if (!ntc_capture_measure.currvolt)
    {
        ntc_capture_measure.currvolt = volt;
    }
    ntc_capture_measure.temperature = ((int32_t)ntc_capture_measure.currvolt - NTC_CAPTURE_VOLTAGE_REF)/NTC_CAPTURE_TEMPERATURE_STEP + NTC_CAPTURE_TEMPERATURE_REF;
    pmu_ntc_capture_disable();
    TRACE(3,"%s ad:%d temperature:%d",__func__, ntc_capture_measure.currvolt, ntc_capture_measure.temperature);
}

int ntc_capture_open(void)
{

    ntc_capture_measure.currvolt = 0;
    ntc_capture_measure.index = 0;
    ntc_capture_measure.temperature = 0;
    ntc_capture_measure.cb = NULL;

    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}

int ntc_capture_start(void)
{
    pmu_ntc_capture_enable();
    hal_gpadc_open(HAL_GPADC_CHAN_0, HAL_GPADC_ATP_ONESHOT, ntc_capture_irqhandler);
    return 0;
}
#else

#define IS_USE_SOC_PMU_PLUGINOUT

#ifdef IS_USE_SOC_PMU_PLUGINOUT

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_MS
#define CHARGER_PLUGINOUT_DEBOUNCE_MS (50)
#endif

#ifndef CHARGER_PLUGINOUT_DEBOUNCE_CNT
#define CHARGER_PLUGINOUT_DEBOUNCE_CNT (3)
#endif

static void app_battery_pluginout_debounce_start(void);
static void app_battery_pluginout_debounce_handler(void const *param);
osTimerDef (APP_BATTERY_PLUGINOUT_DEBOUNCE, app_battery_pluginout_debounce_handler);
static osTimerId app_battery_pluginout_debounce_timer = NULL;
static uint32_t app_battery_pluginout_debounce_ctx = 0;
static uint32_t app_battery_pluginout_debounce_cnt = 0;

static int app_battery_handle_process(APP_MESSAGE_BODY *msg_body)
{
    uint8_t status;

    APP_BATTERY_GET_STATUS(msg_body->message_id, status);

    if (status == APP_BATTERY_STATUS_PLUGINOUT){
        app_battery_pluginout_debounce_start();
    }

    return 0;
}

static void app_battery_event_process(enum APP_BATTERY_STATUS_T status, APP_BATTERY_MV_T volt)
{
    uint32_t app_battevt;
    APP_MESSAGE_BLOCK msg;

    APP_BATTERY_TRACE(3,"%s %d,%d",__func__, status, volt);
    msg.mod_id = APP_MODUAL_BATTERY;
#if defined(USE_BASIC_THREADS)
    msg.mod_level = APP_MOD_LEVEL_2;
#endif
    APP_BATTERY_SET_MESSAGE(app_battevt, status, volt);
    msg.msg_body.message_id = app_battevt;
    msg.msg_body.message_ptr = (uint32_t)NULL;
    app_mailbox_put(&msg);
}

static void app_battery_charger_handler(enum PMU_CHARGER_STATUS_T status)
{
    TRACE(2,"%s: status=%d", __func__, status);
    pmu_charger_set_irq_handler(NULL);
    app_battery_event_process(APP_BATTERY_STATUS_PLUGINOUT,
                              (status == PMU_CHARGER_PLUGIN) ? APP_BATTERY_CHARGER_PLUGIN : APP_BATTERY_CHARGER_PLUGOUT);
}

static enum APP_BATTERY_CHARGER_T app_battery_charger_forcegetstatus(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;
    enum PMU_CHARGER_STATUS_T charger;

    charger = pmu_charger_get_status();

    if (charger == PMU_CHARGER_PLUGIN)
    {
        status = APP_BATTERY_CHARGER_PLUGIN;
    }
    else
    {
        status = APP_BATTERY_CHARGER_PLUGOUT;
    }

    return status;
}

static void app_battery_pluginout_debounce_start(void)
{
    TRACE(1,"%s", __func__);

    app_battery_pluginout_debounce_ctx = (uint32_t)app_battery_charger_forcegetstatus();
    app_battery_pluginout_debounce_cnt = 1;
    osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
}

static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event);

static void app_battery_pluginout_debounce_handler(void const *param)
{
    enum APP_BATTERY_CHARGER_T status_charger = app_battery_charger_forcegetstatus();

    if(app_battery_pluginout_debounce_ctx == (uint32_t) status_charger){
        app_battery_pluginout_debounce_cnt++;
    }
    else
    {
        TRACE(2,"%s dithering cnt %u", __func__, app_battery_pluginout_debounce_cnt);
        app_battery_pluginout_debounce_cnt = 0;
        app_battery_pluginout_debounce_ctx = (uint32_t)status_charger;
    }

    if (app_battery_pluginout_debounce_cnt >= CHARGER_PLUGINOUT_DEBOUNCE_CNT){
        TRACE(2,"%s %s", __func__, status_charger == APP_BATTERY_CHARGER_PLUGOUT ? "PLUGOUT" : "PLUGIN");

        app_battery_pluginout_event_callback(status_charger);
        pmu_charger_set_irq_handler(app_battery_charger_handler);
        osTimerStop(app_battery_pluginout_debounce_timer);
    }else{
        osTimerStart(app_battery_pluginout_debounce_timer, CHARGER_PLUGINOUT_DEBOUNCE_MS);
    }
}

int app_battery_charger_indication_open(void)
{
    enum APP_BATTERY_CHARGER_T status = APP_BATTERY_CHARGER_QTY;

    APP_BATTERY_TRACE(1,"%s",__func__);

    pmu_charger_init();

    pmu_charger_set_irq_handler(app_battery_charger_handler);

    return status;
}
#endif

int app_battery_open(void)
{
    app_battery_opened_callback();
#ifdef IS_USE_SOC_PMU_PLUGINOUT
    if (app_battery_pluginout_debounce_timer == NULL)
    {
        app_battery_pluginout_debounce_timer =
            osTimerCreate (osTimer(APP_BATTERY_PLUGINOUT_DEBOUNCE),
            osTimerOnce, &app_battery_pluginout_debounce_ctx);
    }

    app_set_threadhandle(APP_MODUAL_BATTERY, app_battery_handle_process);

    app_battery_charger_indication_open();
#endif

    // initialize the custom battery manager here
    // returned value could be:
    // #define APP_BATTERY_OPEN_MODE_INVALID        (-1)
    // #define APP_BATTERY_OPEN_MODE_NORMAL         (0)
    // #define APP_BATTERY_OPEN_MODE_CHARGING       (1)
    // #define APP_BATTERY_OPEN_MODE_CHARGING_PWRON (2)
    return APP_BATTERY_OPEN_MODE_NORMAL;
}

int app_battery_start(void)
{
    // start battery measurement timer here
    return 0;
}

int app_battery_stop(void)
{
    // stop battery measurement timer here
    return 0;
}

int app_battery_get_info(APP_BATTERY_MV_T *currvolt, uint8_t *currlevel, enum APP_BATTERY_STATUS_T *status)
{
    // should just return battery level via currlevel for hfp battery level indication
    *currlevel = APP_BATTERY_LEVEL_MAX;
    return 0;
}

#ifdef IS_USE_SOC_PMU_PLUGINOUT
static void app_battery_pluginout_event_callback(enum APP_BATTERY_CHARGER_T event)
{
    if (APP_BATTERY_CHARGER_PLUGOUT == event)
    {
        TRACE(0, "Charger plug out.");
    }
    else if (APP_BATTERY_CHARGER_PLUGIN == event)
    {
        TRACE(0, "Charger plug in.");
    }
}
#endif

int app_battery_register(APP_BATTERY_CB_T user_cb)
{
    // register the battery level update event callback
    return 0;
}

#endif

WEAK void app_battery_opened_callback(void)
{

}

#else
int app_battery_open(void)
{
    return 0;
}

int app_battery_start(void)
{
    return 0;
}
#endif

