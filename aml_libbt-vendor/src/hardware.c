/******************************************************************************
*
*  Copyright (C) 2019-2021 Amlogic Corporation
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at:
*
*  http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.
*
******************************************************************************/

/******************************************************************************
*
*  Filename:      hardware.c
*
*  Description:   Contains controller-specific functions, like
*                      firmware patch download
*                      low power mode operations
*
******************************************************************************/

#define LOG_TAG "bt_hwcfg"

#include <unistd.h>
#include <utils/Log.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <dirent.h>
#include <ctype.h>
#include <cutils/properties.h>
#include <stdlib.h>
#include <string.h>

#include "bt_hci_bdroid.h"
#include "bt_vendor_aml.h"
#include "userial.h"
#include "userial_vendor.h"
#include "upio.h"
#include "vendor_common.h"

/**********bt print**********/
#ifndef BTHW_DBG
    #define BTHW_DBG TRUE
#endif

#if (BTHW_DBG == TRUE)
    #define BTHWDBG(param, ...) { ALOGD(param, ## __VA_ARGS__); }
#else
    #define BTHWDBG(param, ...) {}
#endif

/**********bt related path in the file system**********/
#ifndef AML_BT_FS_PATH
    //#define AML_BT_FS_PATH "/vendor/etc/bluetooth/amlbt"
    #define AML_BT_FS_PATH "/vendor/lib/firmware"
#endif

/**********bt config**********/
#define AML_BT_CONFIG_RF_FILE AML_BT_FS_PATH"/aml_bt.conf"

/**********bt rom check************/
//#define AML_BT_ROM_CHECK_ENABLE

const char *amlbt_file_path[] = {
    AML_BT_FS_PATH,
    NULL
};

char *amlbt_fw_bin[AML_BT_CHIP_TYPE][AML_BT_INTF_TYPE] =
{
    {
        NULL,
        NULL,
        NULL,
    },
    {
        "w1_bt_fw_uart.bin",     //w1 sdio
        NULL,                                               //w1 usb
        NULL,                                               //w1 pcie
    },
    {
        "w1u_bt_fw_uart.bin",    //w1u sdio
        "w1u_bt_fw_usb.bin",     //w1u usb
        "w1u_bt_fw_uart.bin",    //w1u pcie
    },
    {
        "w2_bt_fw_uart.bin",     //w2 sdio
        "w2_bt_fw_usb.bin",      //w2 usb
        "w2_bt_fw_uart.bin",     //w2 pcie
    },
    {
        "w2l_bt_15p4_fw_uart.bin",     //w2l sdio
        "w2l_bt_15p4_fw_usb.bin",      //w2l usb
        "w2l_bt_15p4_fw_uart.bin",     //w2l pcie
    }
};

#define BTM_SCO_CODEC_CVSD 0x0001
#define AML_DOWNLOADFW_UART
#define AML_FW_BIN

#ifdef AML_DOWNLOADFW_UART
    #ifdef AML_FW_FILE
        #include "bt_fucode.h"
    #endif
#endif
/******************************************************************************
**  Constants & Macros
******************************************************************************/

#define FW_PATCHFILE_EXTENSION      ".hcd"
#define FW_PATCHFILE_EXTENSION_LEN  4
#define FW_PATCHFILE_PATH_MAXLEN    248

static const char VENDOR_AMLBTVER_PROPERTY[] = "vendor.sys.amlbtversion";
static const char AML_ROM_CHECK_PROPERTY[] = "vendor.sys.amlbt_rom_check";

#ifdef AML_DOWNLOADFW_UART
int len_iccm = 0, offset_iccm = 0;
int add_iccm = 0;
unsigned int cmd_len_iccm = 0;
unsigned int data_len_iccm = 0;
int len_dccm = 0, offset_dccm = 0;
unsigned int cmd_len_dccm = 0;
unsigned int data_len_dccm = 0;
int cnt = 1;
int iccm_read_off = 0;
unsigned int j = 0;
int cmp_data = 0;
int reg_data = 0;

#ifdef AML_FW_BIN
unsigned char *p_iccm_buf = NULL;
unsigned char *p_dccm_buf = NULL;
unsigned int iccm_size = 0;
unsigned int dccm_size = 0;
unsigned int rom_size = 256 * 1024;
unsigned int check_size = 4 * 1024;
unsigned int check_start = 0;
int fw_fd = -1;
#endif
#endif

/* one byte is for enable/disable
 *    next 2 bytes are for codec type */
#define SCO_CODEC_PARAM_SIZE                    3


/* Hardware Configuration State */
enum
{
    /*       hardware download config       */
    HW_CFG_AML_DOWNLOAD_FIRMWARE_TEST_WRITE_UART,
    HW_CFG_AML_DOWNLOAD_FIRMWARE_TEST_READ_UART,
    HW_CFG_AML_DOWNLOAD_FIRMWARE_ICCM_UART,
    HW_CFG_AML_DOWNLOAD_FIRMWARE_DCCM_UART,
    HW_CFG_AML_DOWNLOAD_FIRMWARE_CLOSE_EVENT,
    HW_CFG_AML_CONFIG_RF_CALIBRATION,
    HW_CFG_AML_CONFIG_TX_POWER,   //w2 used
    HW_CFG_AML_CONFIG_FWLOG_OUTPUT,   //w2 used
    HW_CFG_AML_DOWNLOAD_FIRMWARE_START_CPU_UART_BEFORE,
    HW_CFG_AML_DOWNLOAD_FIRMWARE_START_CPU_UART,
    HW_CFG_SET_PARAMS,
    HW_CFG_SET_WAVEFORM_DATA,
    HW_CFG_SET_BD_ADDR,
    /*       hardware undownload config       */
    HW_CFG_SET_WAKEUP_PARAMS,
    HW_CFG_GET_REG, //not used
    /*       common end       */
    HW_CFG_SET_MANU_DATA, //end
};

/*       hardware download config       */
uint8_t hw_cfg_download_firmware_test_write_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_test_read_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_iccm_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_dccm_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_close_event(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_rf_calibration(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_tx_power(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_fwlog_output(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_start_cpu_uart_before(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_download_firmware_start_cpu_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_set_params(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_set_waveform_data(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_set_bd_addr(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
uint8_t hw_cfg_get_reg(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
/*       hardware undownload config       */
uint8_t hw_cfg_set_wakeup_params(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);
/*       common end       */
uint8_t wole_config_write_manufacture(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p);

uint8_t (*hw_config_func[])(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p) =
{
    /*       hardware download config       */
    hw_cfg_download_firmware_test_write_uart,
    hw_cfg_download_firmware_test_read_uart,
    hw_cfg_download_firmware_iccm_uart,
    hw_cfg_download_firmware_dccm_uart,
    hw_cfg_download_firmware_close_event,
    hw_cfg_rf_calibration,
    hw_cfg_tx_power,   //w2 used
    hw_cfg_fwlog_output,   //w2 used
    hw_cfg_download_firmware_start_cpu_uart_before,
    hw_cfg_download_firmware_start_cpu_uart,
    hw_cfg_set_params,
    hw_cfg_set_waveform_data, //w1u
    hw_cfg_set_bd_addr,
    /*       hardware undownload config       */
    hw_cfg_set_wakeup_params,
    hw_cfg_get_reg, //not used
    /*       common end       */
    wole_config_write_manufacture,    //end
};

/******************************************************************************
**  Externs
******************************************************************************/
extern uint8_t vnd_local_bd_addr[BD_ADDR_LEN];
extern unsigned int amlbt_rftype;
extern unsigned int amlbt_btsink;
extern unsigned int amlbt_fw_mode;
extern unsigned int amlbt_pin_mux;
extern unsigned int amlbt_br_digit_gain;
extern unsigned int amlbt_edr_digit_gain;
extern unsigned int amlbt_fwlog_config;
extern unsigned char APCF_config_manf_data[256];
extern unsigned int amlbt_manf_cnt;
extern unsigned int amlbt_factory;
extern unsigned int amlbt_system;
extern unsigned int amlbt_manf_para;
extern unsigned char w1u_manf_data[MANF_ROW][MANF_COLUMN];

/******************************************************************************
**  Static variables
******************************************************************************/

static char fw_patchfile_path[256] = FW_PATCHFILE_LOCATION;
static char fw_patchfile_name[128] = { 0 };
#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
    static int fw_patch_settlement_delay = -1;
#endif
static int wbs_sample_rate = SCO_WBS_SAMPLE_RATE;

void hw_config_cback(void *p_evt_buf);
void hw_read_type(HC_BT_HDR *p_buf);
void hw_read_type_cback(void *p_mem);

/* low power mode parameters */
typedef struct
{
    uint8_t sleep_mode;                     /* 0(disable),1(UART),9(H5) */
    uint8_t host_stack_idle_threshold;      /* Unit scale 300ms/25ms */
    uint8_t host_controller_idle_threshold; /* Unit scale 300ms/25ms */
    uint8_t bt_wake_polarity;               /* 0=Active Low, 1= Active High */
    uint8_t host_wake_polarity;             /* 0=Active Low, 1= Active High */
    uint8_t allow_host_sleep_during_sco;
    uint8_t combine_sleep_mode_and_lpm;
    uint8_t enable_uart_txd_tri_state;      /* UART_TXD Tri-State */
    uint8_t sleep_guard_time;               /* sleep guard time in 12.5ms */
    uint8_t wakeup_guard_time;              /* wakeup guard time in 12.5ms */
    uint8_t txd_config;                     /* TXD is high in sleep state */
    uint8_t pulsed_host_wake;               /* pulsed host wake if mode = 1 */
} bt_lpm_param_t;

/* Firmware re-launch settlement time */
typedef struct
{
    const char 	*chipset_name;
    const uint32_t	delay_time;
} fw_settlement_entry_t;

static bt_lpm_param_t lpm_param =
{
    LPM_SLEEP_MODE,
    LPM_IDLE_THRESHOLD,
    LPM_HC_IDLE_THRESHOLD,
    LPM_BT_WAKE_POLARITY,
    LPM_HOST_WAKE_POLARITY,
    LPM_ALLOW_HOST_SLEEP_DURING_SCO,
    LPM_COMBINE_SLEEP_MODE_AND_LPM,
    LPM_ENABLE_UART_TXD_TRI_STATE,
    0,      /* not applicable */
    0,      /* not applicable */
    0,      /* not applicable */
    LPM_PULSED_HOST_WAKE
};

#if (SCO_CFG_INCLUDED == TRUE)

/* need to update the bt_sco_i2spcm_param as well
 * bt_sco_i2spcm_param will be used for WBS setting
 * update the bt_sco_param and bt_sco_i2spcm_param */
static uint8_t bt_sco_param[SCO_PCM_PARAM_SIZE] =
{
    SCO_PCM_ROUTING,
    SCO_PCM_IF_CLOCK_RATE,
    SCO_PCM_IF_FRAME_TYPE,
    SCO_PCM_IF_SYNC_MODE,
    SCO_PCM_IF_CLOCK_MODE
};

static uint8_t bt_pcm_data_fmt_param[PCM_DATA_FORMAT_PARAM_SIZE] =
{
    PCM_DATA_FMT_SHIFT_MODE,
    PCM_DATA_FMT_FILL_BITS,
    PCM_DATA_FMT_FILL_METHOD,
    PCM_DATA_FMT_FILL_NUM,
    PCM_DATA_FMT_JUSTIFY_MODE
};

static uint8_t bt_sco_i2spcm_param[SCO_I2SPCM_PARAM_SIZE] =
{
    SCO_I2SPCM_IF_MODE,
    SCO_I2SPCM_IF_ROLE,
    SCO_I2SPCM_IF_SAMPLE_RATE,
    SCO_I2SPCM_IF_CLOCK_RATE
};
#endif
/*
 * The look-up table of recommended firmware settlement delay (milliseconds) on
 * known chipsets.
 */
static const fw_settlement_entry_t fw_settlement_table[] =
{
    { "BCM43241",	      200 },
    { "BCM43341",	      100 },
    { (const char *)NULL, 100 }// Giving the generic fw settlement delay setting.
};

/*
 * NOTICE:
 *     If the platform plans to run I2S interface bus over I2S/PCM port of the
 *     BT Controller with the Host AP, explicitly set "SCO_USE_I2S_INTERFACE = TRUE"
 *     in the corresponding include/vnd_<target>.txt file.
 *     Otherwise, leave SCO_USE_I2S_INTERFACE undefined in the vnd_<target>.txt file.
 *     And, PCM interface will be set as the default bus format running over I2S/PCM
 *     port.
 */
#if (defined(SCO_USE_I2S_INTERFACE) && SCO_USE_I2S_INTERFACE == TRUE)
    static uint8_t sco_bus_interface = SCO_INTERFACE_I2S;
#else
    static uint8_t sco_bus_interface = SCO_INTERFACE_PCM;
#endif

#define INVALID_SCO_CLOCK_RATE  0xFF
static uint8_t sco_bus_clock_rate = INVALID_SCO_CLOCK_RATE;
static uint8_t sco_bus_wbs_clock_rate = INVALID_SCO_CLOCK_RATE;

/******************************************************************************
**  Static functions
******************************************************************************/
static void hw_sco_i2spcm_config(void *p_mem, uint16_t codec);

static int open_file(const char *filename, int flags)
{
    int fd = -1;
    char debug_path[PROPERTY_VALUE_MAX] = {'\0'};
    char buf[256];
    const char *f = NULL;
    int i = 0;

    memset(debug_path, 0, sizeof(debug_path));
    f = strrchr(filename, '/');

    /* Note: you need disable selinux and gives chmod permission for
    ** driver files when specify the path of driver.
    ** e.g. setprop persist.vendor.wifibt_drv_path "/data/vendor"
    */
    if (property_get("persist.vendor.wifibt_drv_path", debug_path, NULL)) {
        memset(buf, 0, sizeof(buf));
        if (f)
            snprintf(buf, sizeof(buf), "%s%s", debug_path, f);
        else
            snprintf(buf, sizeof(buf), "%s/%s", debug_path, filename);
        ALOGD("open file: %s\n", buf);
        if ((fd = open(buf, flags)) < 0) {
            ALOGD("open: %s failed!\n", buf);
        } else {
            ALOGD("open: %s successful!\n", buf);
            return fd;
        }
    }

    if (!f) {
        for (i = 0; amlbt_file_path[i] != NULL; i++) {
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "%s/%s", amlbt_file_path[i], filename);
            ALOGD("open file: %s\n", buf);
            if ((fd = open(buf, flags)) < 0) {
                ALOGD("open file: %s failed!\n", buf);
            } else {
                ALOGD("open file: %s successful!\n", buf);
                return fd;
            }
        }
    } else {
        ALOGD("open file: %s\n", filename);
        if ((fd = open(filename, flags)) < 0) {
            ALOGD("open: %s failed!\n", filename);
        } else {
            ALOGD("open: %s successful!\n", filename);
            return fd;
        }
    }

    return fd;
}

/******************************************************************************
**  Controller Initialization Static Functions
******************************************************************************/

/*******************************************************************************
**
** Function        look_up_fw_settlement_delay
**
** Description     If FW_PATCH_SETTLEMENT_DELAY_MS has not been explicitly
**                 re-defined in the platform specific build-time configuration
**                 file, we will search into the look-up table for a
**                 recommended firmware settlement delay value.
**
**                 Although the settlement time might be also related to board
**                 configurations such as the crystal clocking speed.
**
** Returns         Firmware settlement delay
**
*******************************************************************************/
uint32_t look_up_fw_settlement_delay(void)
{
    uint32_t ret_value;
    fw_settlement_entry_t *p_entry;

    if (FW_PATCH_SETTLEMENT_DELAY_MS > 0)
    {
        ret_value = FW_PATCH_SETTLEMENT_DELAY_MS;
    }
#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
    else if (fw_patch_settlement_delay >= 0)
    {
        ret_value = fw_patch_settlement_delay;
    }
#endif
    else
    {
        p_entry = (fw_settlement_entry_t *)fw_settlement_table;

        while (p_entry->chipset_name != NULL)
        {
            if (strstr(hw_cfg_cb.local_chip_name, p_entry->chipset_name) != NULL)
            {
                break;
            }

            p_entry++;
        }

        ret_value = p_entry->delay_time;
    }

    BTHWDBG("Settlement delay -- %d ms", ret_value);

    return (ret_value);
}

/*******************************************************************************
**
** Function        ms_delay
**
** Description     sleep unconditionally for timeout milliseconds
**
** Returns         None
**
*******************************************************************************/
void ms_delay(uint32_t timeout)
{
    struct timespec delay;
    int err;

    if (timeout == 0)
        return;

    delay.tv_sec = timeout / 1000;
    delay.tv_nsec = 1000 * 1000 * (timeout % 1000);

    /* [u]sleep can't be used because it uses SIGALRM */
    do
    {
        err = nanosleep(&delay, &delay);
    }
    while (err < 0 && errno == EINTR);
}

/*******************************************************************************
**
** Function        line_speed_to_userial_baud
**
** Description     helper function converts line speed number into USERIAL baud
**                 rate symbol
**
** Returns         unit8_t (USERIAL baud symbol)
**
*******************************************************************************/
uint8_t line_speed_to_userial_baud(uint32_t line_speed)
{
    uint8_t baud;

    if (line_speed == 4000000)
        baud = USERIAL_BAUD_4M;
    else if (line_speed == 3000000)
        baud = USERIAL_BAUD_3M;
    else if (line_speed == 2000000)
        baud = USERIAL_BAUD_2M;
    else if (line_speed == 1000000)
        baud = USERIAL_BAUD_1M;
    else if (line_speed == 921600)
        baud = USERIAL_BAUD_921600;
    else if (line_speed == 460800)
        baud = USERIAL_BAUD_460800;
    else if (line_speed == 230400)
        baud = USERIAL_BAUD_230400;
    else if (line_speed == 115200)
        baud = USERIAL_BAUD_115200;
    else if (line_speed == 57600)
        baud = USERIAL_BAUD_57600;
    else if (line_speed == 19200)
        baud = USERIAL_BAUD_19200;
    else if (line_speed == 9600)
        baud = USERIAL_BAUD_9600;
    else if (line_speed == 1200)
        baud = USERIAL_BAUD_1200;
    else if (line_speed == 600)
        baud = USERIAL_BAUD_600;
    else
    {
        ALOGE("userial vendor: unsupported baud speed %d", line_speed);
        baud = USERIAL_BAUD_115200;
    }
    ALOGE("userial vendor: set baud speed %d", line_speed);
    return baud;
}


/*******************************************************************************
**
** Function         hw_strncmp
**
** Description      Used to compare two strings
**
** Returns          0: match, otherwise: not match
**
*******************************************************************************/
static int hw_strncmp(const char *p_str1, const char *p_str2, const int len)
{
    int i;

    if (!p_str1 || !p_str2)
        return (1);

    for (i = 0; i < len; i++)
    {
        if (toupper(p_str1[i]) != toupper(p_str2[i]))
            return (i + 1);
    }

    return 0;
}

/*******************************************************************************
**
** Function         hw_config_set_bdaddr
**
** Description      Program controller's Bluetooth Device Address
**
** Returns          TRUE, if valid address is sent
**                  FALSE, otherwise
**
*******************************************************************************/
static uint8_t hw_config_set_bdaddr(HC_BT_HDR *p_buf)
{
    uint8_t retval = FALSE;
    uint8_t *p = (uint8_t *)(p_buf + 1);

    BTHWDBG("Setting local bd addr to %02X:%02X:%02X:%02X:%02X:%02X",
            vnd_local_bd_addr[0], vnd_local_bd_addr[1], vnd_local_bd_addr[2],
            vnd_local_bd_addr[3], vnd_local_bd_addr[4], vnd_local_bd_addr[5]);

    UINT16_TO_STREAM(p, HCI_VSC_WRITE_BD_ADDR);
    *p++ = BD_ADDR_LEN; /* parameter length */
    *p++ = vnd_local_bd_addr[5];
    *p++ = vnd_local_bd_addr[4];
    *p++ = vnd_local_bd_addr[3];
    *p++ = vnd_local_bd_addr[2];
    *p++ = vnd_local_bd_addr[1];
    *p = vnd_local_bd_addr[0];

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + BD_ADDR_LEN;
    if (amlbt_transtype.family_id == AML_W1U)
    {
        hw_cfg_cb.state = HW_CFG_SET_WAVEFORM_DATA;
    }
    else
    {
        hw_cfg_cb.state = HW_CFG_SET_BD_ADDR;
    }
    retval = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_BD_ADDR, p_buf, \
                                       hw_config_cback);

    return (retval);
}

#if (USE_CONTROLLER_BDADDR == TRUE)
/*******************************************************************************
**
** Function         hw_config_read_bdaddr
**
** Description      Read controller's Bluetooth Device Address
**
** Returns          TRUE, if valid address is sent
**                  FALSE, otherwise
**
*******************************************************************************/
static uint8_t hw_config_read_bdaddr(HC_BT_HDR *p_buf)
{
    uint8_t retval = FALSE;
    uint8_t *p = (uint8_t *)(p_buf + 1);

    UINT16_TO_STREAM(p, HCI_READ_LOCAL_BDADDR);
    *p = 0; /* parameter length */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE;
    hw_cfg_cb.state = HW_CFG_READ_BD_ADDR;

    retval = bt_vendor_cbacks->xmit_cb(HCI_READ_LOCAL_BDADDR, p_buf, \
                                       hw_config_cback);

    return (retval);
}
#endif // (USE_CONTROLLER_BDADDR == TRUE)


/*******************************************************************************
**
** Function         hw_config_set_rf_params
**
** Description      Config rf parameters to controller
**
** Returns
**
**
*******************************************************************************/
static uint8_t hw_config_set_rf_params(HC_BT_HDR *p_buf)
{
    uint8_t retval = FALSE;
    uint8_t *p = (uint8_t *)(p_buf + 1);
    //uint8_t set_rf[8] = { 0 };
    //int size = 0;
    //uint8_t *q;
    uint32_t reg_val = 0;

    //antenna_num = amlbt_rftype;

    BTHWDBG("antenna number=%d sink mode=%d", amlbt_rftype, amlbt_btsink);

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;                       /* parameter length */
    UINT32_TO_STREAM(p, REG_PMU_POWER_CFG);  /* addr */
    if (amlbt_rftype == AML_SINGLE_ANTENNA)
    {
        UINT32_TO_STREAM(p, (unsigned int)((0x1 << BIT_RF_NUM) | (amlbt_btsink << BT_SINK_MODE)));
    }
    else if (amlbt_rftype == AML_DOUBLE_ANTENNA)
    {
        UINT32_TO_STREAM(p, (unsigned int)((0x2 << BIT_RF_NUM) | (amlbt_btsink << BT_SINK_MODE)));
    }

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + 8;

    if (amlbt_transtype.family_id == AML_W2)
    {
        hw_cfg_cb.state = HW_CFG_AML_CONFIG_TX_POWER;
    }
    else
    {
        hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_START_CPU_UART_BEFORE;
    }

    retval = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                                       p_buf, hw_config_cback);
#if 0
    if ((amlbt_transtype.family_id >= AML_W1U) &&
            (amlbt_transtype.interface != AML_INTF_USB))
    {
        q = set_rf;
        UINT32_TO_STREAM(q, REG_PMU_POWER_CFG);  /* addr */
        if (amlbt_rftype == AML_SINGLE_ANTENNA)
        {
            UINT32_TO_STREAM(q, (unsigned int)((0x1 << BIT_RF_NUM) | (amlbt_btsink << BT_SINK_MODE)));
        }
        else if (amlbt_rftype == AML_DOUBLE_ANTENNA)
        {
            UINT32_TO_STREAM(q, (unsigned int)((0x2 << BIT_RF_NUM) | (amlbt_btsink << BT_SINK_MODE)));
        }
        size = write(bt_sdio_fd, set_rf, sizeof(set_rf));
        if (size < 0)
        {
            BTHWDBG("write failed!");
        }
    }
#endif
    return (retval);
}

static int hw_config_get_iccm_size(void)
{
    int fd = 0;
    unsigned int iccm_size = 0;
    int size = 0;
    char *file = NULL;

    BTHWDBG("hw_config_get_iccm_size chip(%d:%d:%d)\n", amlbt_transtype.family_id,
            amlbt_transtype.family_rev, amlbt_transtype.interface);

    file = amlbt_fw_bin[amlbt_transtype.family_id][amlbt_transtype.interface];
    if (file == NULL)
    {
        BTHWDBG("hw_config_get_iccm_size get fw bin error!\n");
        return 0;
    }

    if ((fd = open_file(file, O_RDONLY)) < 0)
        return 0;
    size = read(fd, &iccm_size, 4);
    if (size < 0)
    {
        BTHWDBG("---------hw_config_get_iccm_size read error!---------");
        close(fd);
        return 0;
    }
    close(fd);

    BTHWDBG("---------hw_config_get_iccm_size iccm_size %#x---------\n", iccm_size);
    return iccm_size;
}

static int hw_config_get_dccm_size(void)
{
    int fd = 0;
    unsigned int dccm_size = 0;
    int size = 0;
    char *file = NULL;

    BTHWDBG("hw_config_get_dccm_size chip(%d:%d:%d)\n", amlbt_transtype.family_id,
            amlbt_transtype.family_rev, amlbt_transtype.interface);

    file = amlbt_fw_bin[amlbt_transtype.family_id][amlbt_transtype.interface];
    if (file == NULL)
    {
        BTHWDBG("hw_config_get_dccm_size get fw bin error!\n");
        return 0;
    }

    if ((fd = open_file(file, O_RDONLY)) < 0)
        return 0;
    /*skip 4 bytes iccm len*/
    size = read(fd, &dccm_size, 4);
    if (size < 0)
    {
        BTHWDBG("---------hw_config_get_dccm_size read error!---------");
        close(fd);
        return 0;
    }

    //W1U skip 15KB additional iccm
    if (amlbt_transtype.family_id == AML_W1U && amlbt_transtype.interface != AML_INTF_USB)
    {
        size = read(fd, &add_iccm, 4);
        if (size < 0)
        {
            BTHWDBG("---------hw_config_get_dccm_size read error!---------");
            close(fd);
            return 0;
        }
        BTHWDBG("---------hw_config_get_add_iccm_size %#x---------\n", add_iccm);
    }

    size = read(fd, &dccm_size, 4);
    if (size < 0)
    {
        BTHWDBG("---------hw_config_get_dccm_size read error!---------");
        close(fd);
        return 0;
    }
    close(fd);

    BTHWDBG("---------hw_config_get_dccm_size dccm_size %#x---------\n", dccm_size);

    if (amlbt_transtype.family_id == AML_W1U && amlbt_transtype.interface != AML_INTF_USB)
    {
        if (dccm_size == W1U_ROM_START_CODE)
        {
            BTHWDBG("---------w1u sram code size 0---------\n");
            dccm_size = add_iccm;
            add_iccm = 0;
        }
    }

    return dccm_size;
}

static inline bool is_valid_size(unsigned int size)
{
    return (size >= MIN_ALLOC_SIZE && size <= MAX_ALLOC_SIZE);
}

#ifdef AML_DOWNLOADFW_UART
static void hw_get_bin_size(void)
{
#ifdef AML_FW_FILE
    len_iccm = sizeof(BT_fwICCM);
#endif
#ifdef AML_FW_BIN
    len_iccm = hw_config_get_iccm_size();
    iccm_size = len_iccm;
#endif
    if (amlbt_transtype.family_id == AML_W2L)
    {
        len_iccm -= 384 * 1024;
        offset_iccm = 384 * 1024;
    }
    else
    {
        len_iccm -= 256 * 1024;
        offset_iccm = 256 * 1024;
    }
#ifdef AML_FW_FILE
    len_dccm = sizeof(BT_fwDCCM);
#endif
#ifdef AML_FW_BIN
    len_dccm = hw_config_get_dccm_size();
    dccm_size = len_dccm;
#endif
    offset_dccm = 0;
#ifdef AML_FW_BIN
    if (!is_valid_size(iccm_size) || !is_valid_size(dccm_size))
    {
        BTHWDBG("is_valid_size error! iccm_size %#x dccm_size %#x\n", iccm_size, dccm_size);
        return ;
    }
    p_iccm_buf = malloc(iccm_size);
    if (!p_iccm_buf)
    {
        BTHWDBG("malloc(iccm_size) error!\n");
        return ;
    }
    p_dccm_buf = malloc(dccm_size);
    if (!p_dccm_buf)
    {
        BTHWDBG("malloc(dccm_size) error!\n");
        free(p_iccm_buf);
        return ;
    }
#endif
}
#endif

uint8_t hw_cfg_download_firmware_test_write_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;						/* parameter length */
    UINT32_TO_STREAM(p, REG_RAM_PD_SHUTDWONW_SW);	/* addr */
    UINT32_TO_STREAM(p, 0x0);		/* data 4M */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_ICCM_UART;

    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);

    return is_proceeding;
}

uint8_t hw_cfg_download_firmware_test_read_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, TCI_READ_REG);
    *p++ = 4;                       /* parameter length */
    UINT32_TO_STREAM(p, REG_RAM_PD_SHUTDWONW_SW);  /* addr */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 4;
    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_ICCM_UART;

    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_READ_REG, \
                    p_buf, hw_config_cback);

    return is_proceeding;
}

uint8_t hw_cfg_download_firmware_iccm_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;
    int i;

    if (len_iccm <= 0)
    {
        ALOGW(" iccm write over ");
        hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_DCCM_UART;
        cnt = 0;
        return is_proceeding;
    }
    data_len_iccm = (len_iccm > RW_OPERATION_SIZE) ? RW_OPERATION_SIZE : len_iccm;
    cmd_len_iccm = data_len_iccm + 4;		  // addr


    UINT16_TO_STREAM(p, TCI_DOWNLOAD_BT_FW);
    *p++ = cmd_len_iccm; /* parameter length */
    UINT32_TO_STREAM(p, ICCM_RAM_BASE + offset_iccm);
    for (i = 0; i < (int)data_len_iccm; i += 1)
    {
        p[i] = *(p_iccm_buf + offset_iccm + i);/* data */
    }

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 cmd_len_iccm;

    cnt++;
    offset_iccm += data_len_iccm;
    len_iccm -= data_len_iccm;

    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_DOWNLOAD_BT_FW, \
                    p_buf, hw_config_cback);

    if (len_iccm <= 0)
    {
        BTHWDBG("iccm write over successfully ");
        hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_DCCM_UART;
        cnt = 0;
    }

    return is_proceeding;
}


uint8_t hw_cfg_download_firmware_dccm_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;
    int i;

    if (len_dccm <= 0)
    {
        ALOGW("dccm write over ");
        hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_CLOSE_EVENT;
        return is_proceeding;
    }
    data_len_dccm = (len_dccm > RW_OPERATION_SIZE) ? RW_OPERATION_SIZE : len_dccm;
    cmd_len_dccm = data_len_dccm + 4;


    UINT16_TO_STREAM(p, TCI_DOWNLOAD_BT_FW);
    *p++ = cmd_len_dccm;
    UINT32_TO_STREAM(p, DCCM_RAM_BASE + offset_dccm);
    for (i = 0; i < (int)data_len_dccm; i += 1)
    {
        p[i] = *(p_dccm_buf + offset_dccm + i);
    }

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 cmd_len_dccm;

    cnt++;
    offset_dccm += data_len_dccm;
    len_dccm -= data_len_dccm;

    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_DOWNLOAD_BT_FW, \
                    p_buf, hw_config_cback);

    if (len_dccm <= 0)
    {
        ALOGI("dccm write over successfully. ");
        hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_CLOSE_EVENT;
        cnt = 0;
         if (amlbt_transtype.family_id == AML_W1U && amlbt_transtype.interface != AML_INTF_USB)
       {
           bt_sdio_fd = userial_vendor_devchar_open();
           if (bt_sdio_fd < 0)
           {
             ALOGD("bluetooth node open failed!");
             return -1;
           }
       }
    }

    return is_proceeding;
}

uint8_t hw_cfg_download_firmware_close_event(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;
    UINT32_TO_STREAM(p, 0xa70014);
    UINT32_TO_STREAM(p, 0x0000000);

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_AML_CONFIG_RF_CALIBRATION;
    free(p_iccm_buf);
    free(p_dccm_buf);
    if (fw_fd != -1)
    {
        close(fw_fd);
        fw_fd = -1;
    }
    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);
    BTHWDBG("HW_CFG_AML_DOWNLOAD_FIRMWARE_CLOSE_EVENT");

    return is_proceeding;
}

uint8_t hw_cfg_rf_calibration(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    is_proceeding = hw_config_set_rf_params(p_buf);

    if (is_proceeding == FALSE)
    {
        if (hw_cfg_cb.fw_fd != -1)
        {
            close(hw_cfg_cb.fw_fd);
            hw_cfg_cb.fw_fd = -1;
        }
        ALOGE("config rf parameters failed!!!");
    }

    return is_proceeding;
}

uint8_t hw_cfg_tx_power(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;
    uint32_t reg_val = 0;


    BTHWDBG("amlbt_pin_mux=%d amlbt_br_digit_gain=%#x amlbt_edr_digit_gain=%#x amlbt_factory=%#x amlbt_system=%#x",
            amlbt_pin_mux, amlbt_br_digit_gain, amlbt_edr_digit_gain, amlbt_factory, amlbt_system);

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;
    UINT32_TO_STREAM(p, RG_AON_A53);
    reg_val |= ((amlbt_pin_mux << 20) | (amlbt_factory << 21) | (amlbt_system << 23));
    reg_val |= (((amlbt_edr_digit_gain & 0xff) << 8) | (amlbt_br_digit_gain & 0xff));
    //BTHWDBG("reg_val=%#x", reg_val);
    UINT32_TO_STREAM(p, reg_val);

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_AML_CONFIG_FWLOG_OUTPUT;
    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);

    return is_proceeding;
}

uint8_t hw_cfg_fwlog_output(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;
    uint32_t reg_val = 0;

    BTHWDBG("amlbt_fwlog_config=%d", amlbt_fwlog_config);

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;
    UINT32_TO_STREAM(p, RG_AON_A59);
    reg_val |= (amlbt_fwlog_config & 0x3);
    //BTHWDBG("reg_val=%#x", reg_val);
    UINT32_TO_STREAM(p, reg_val);

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_START_CPU_UART_BEFORE;
    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);

    return is_proceeding;
}

uint8_t hw_cfg_download_firmware_start_cpu_uart_before(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;
    UINT32_TO_STREAM(p, 0xa7000c);
    UINT32_TO_STREAM(p, 0x8000000);

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_START_CPU_UART;
    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);
    BTHWDBG("HW_CFG_AML_DOWNLOAD_FIRMWARE_START_CPU_UART_BEFORE");

    return is_proceeding;
}

uint8_t hw_cfg_download_firmware_start_cpu_uart(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;
    UINT32_TO_STREAM(p, REG_DEV_RESET);
    if (amlbt_transtype.family_id == AML_W1 || amlbt_transtype.family_id == AML_W1U)
    {
        UINT32_TO_STREAM(p, (unsigned int)((BIT_CPU | BIT_MAC | BIT_PHY) << 8));
        BTHWDBG("%#x", ((BIT_CPU | BIT_MAC | BIT_PHY) << 8));
    }
    else
    {
        UINT32_TO_STREAM(p, (unsigned int)((BIT_CPU | BIT_MAC | BIT_PHY) << DEV_RESET_SW));
        BTHWDBG("%#x", ((BIT_CPU | BIT_MAC | BIT_PHY) << DEV_RESET_SW));
    }
    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_SET_PARAMS;

    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);
    BTHWDBG("HW_CFG_AML_DOWNLOAD_FIRMWARE_START_CPU_UART");

    return is_proceeding;
}

uint8_t hw_cfg_set_params(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;

    ms_delay(300);  //need 300ms delay!!

    is_proceeding = hw_config_set_bdaddr(p_buf);

    if (is_proceeding == FALSE)
    {
        BTHWDBG("HW_CFG_SET_PARAMS ERROR");
        if (hw_cfg_cb.fw_fd != -1)
        {
            close(hw_cfg_cb.fw_fd);
            hw_cfg_cb.fw_fd = -1;
        }
    }
    return is_proceeding;
}

uint8_t hw_cfg_set_waveform_data(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    int i = 0;
    int total_len = 0;
    unsigned int cnt_data = 0;
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    char local_ver[128];
    char chip_name[128] = {0};
    uint8_t default_manf_data[] = {0x05, 0x19, 0xff, 0x01, 0x0a, 0xb}; //public version
    uint8_t *total_manf_data = NULL;

    char *p_tmp = (char *)(p_evt_buf + 1) + \
                  HCI_EVT_CMD_CMPL_LOCAL_FW_VERSION;
    uint8_t is_proceeding = FALSE;

    property_get("persist.vendor.bt_name", chip_name, "unknown");
    int year = 2020 + (*(p_tmp + 1) >> 4)%16;
    int month= (*(p_tmp + 1) & 0x0F)%16;

    BTHWDBG("BT Controller model=%s,version = %04d.%02d.%02x,number = 0x%02x%02x", chip_name,year,month, *p_tmp, *(p_tmp + 3), *(p_tmp + 2));
    snprintf(local_ver, sizeof(local_ver), "model=%s,version = %04d.%02d.%02x,number = 0x%02x%02x",chip_name, year,month, *p_tmp, *(p_tmp + 3), *(p_tmp + 2));

    if (property_set(VENDOR_AMLBTVER_PROPERTY, (char *)local_ver) < 0)
    {
        ALOGE("%s:Failed to set amlbt version in %s", __func__, VENDOR_AMLBTVER_PROPERTY);
    }
    if (amlbt_transtype.interface == AML_INTF_USB && amlbt_transtype.family_id == AML_W2)
    {
        bt_vendor_cbacks->dealloc(p_buf);
        bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);

        hw_cfg_cb.state = 0;
        if (hw_cfg_cb.fw_fd != -1)
        {
            close(hw_cfg_cb.fw_fd);
            hw_cfg_cb.fw_fd = -1;
        }
        return TRUE;
    }
    if (amlbt_manf_para == 0)
    {
        UINT16_TO_STREAM(p, HCI_VSC_WAKE_WRITE_DATA);
        for (i = 0; i < sizeof(default_manf_data); i++)
            *p++ = default_manf_data[i];
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + default_manf_data[0];
        hw_cfg_cb.state = HW_CFG_SET_MANU_DATA;
        //Set ADV parameter of remote controller using to wake up device when suspend
        is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_VSC_WAKE_WRITE_DATA, p_buf, hw_config_cback);
        ALOGE("%s aml_bt config manf error use default", __func__);

        return is_proceeding;
    }
    total_manf_data = malloc(LOCAL_BDADDR_PATH_BUFFER_LEN);
    if (!total_manf_data)
    {
        ALOGE("total_manf_data Memory allocation failed!");
        return FALSE;
    }
    memset(total_manf_data, 0, LOCAL_BDADDR_PATH_BUFFER_LEN);
    while (cnt < MANF_ROW)
    {
        if (BIT(cnt) & amlbt_manf_para)
        {
            if ((total_len + w1u_manf_data[cnt][0] + 1) > LOCAL_BDADDR_PATH_BUFFER_LEN)
            {
                ALOGE("Buffer overflow!");
                break;
            }
            else
            {
                memcpy(&total_manf_data[total_len], w1u_manf_data[cnt], w1u_manf_data[cnt][0] + 1);
                /*ALOGD("%#x %#x %#x %#x %#x %#x %#x %#x", total_manf_data[total_len], total_manf_data[total_len+1],
                        total_manf_data[total_len+2], total_manf_data[total_len+3],
                        total_manf_data[total_len+4], total_manf_data[total_len+5],
                        total_manf_data[total_len+6], total_manf_data[total_len+7]);*/
                total_len += w1u_manf_data[cnt][0] + 1;
                ALOGD("%#x %#x %#x %#x %#x %#x %#x %#x", w1u_manf_data[cnt][0], w1u_manf_data[cnt][1],
                        w1u_manf_data[cnt][2], w1u_manf_data[cnt][3],
                        w1u_manf_data[cnt][4], w1u_manf_data[cnt][5],
                        w1u_manf_data[cnt][6], w1u_manf_data[cnt][7]);
                cnt_data++;
            }
        }
        cnt++;
    }
    //ALOGD("%#x %#x", cnt_data, total_len);
    UINT16_TO_STREAM(p, HCI_VSC_WAKE_WRITE_DATA);
    UINT8_TO_STREAM(p, total_len + 1);
    UINT8_TO_STREAM(p, cnt_data);
    for (i = 0; i < total_len; i++)
    {
        *p++ = total_manf_data[i];
    }

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + 1 + total_len;

    hw_cfg_cb.state = HW_CFG_SET_MANU_DATA;
    //Set ADV parameter of remote controller using to wake up device when suspend
    is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_VSC_WAKE_WRITE_DATA, p_buf, hw_config_cback);
    if (total_manf_data)
    {
        free(total_manf_data);
    }

    return is_proceeding;
}

uint8_t hw_cfg_set_bd_addr(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    int i = 0;
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    char local_ver[128];
    char chip_name[128] = {0};
    uint8_t default_manf_data[] = {0x05, 0x19, 0xff, 0x01, 0x0a, 0xb}; //public version
    char *p_tmp = (char *)(p_evt_buf + 1) + \
                  HCI_EVT_CMD_CMPL_LOCAL_FW_VERSION;
    uint8_t is_proceeding = FALSE;
    if (cnt == 0)
    {
        property_get("persist.vendor.bt_name", chip_name, "unknown");
        int year = 2020 + (*(p_tmp + 1) >> 4)%16;
        int month= (*(p_tmp + 1) & 0x0F)%16;

        BTHWDBG("BT Controller model=%s,version = %04d.%02d.%02x,number = 0x%02x%02x", chip_name,year,month, *p_tmp, *(p_tmp + 3), *(p_tmp + 2));
        snprintf(local_ver, sizeof(local_ver), "model=%s,version = %04d.%02d.%02x,number = 0x%02x%02x",chip_name, year,month, *p_tmp, *(p_tmp + 3), *(p_tmp + 2));

        if (property_set(VENDOR_AMLBTVER_PROPERTY, (char *)local_ver) < 0)
        {
            ALOGE("%s:Failed to set amlbt version in %s", __func__, VENDOR_AMLBTVER_PROPERTY);
        }
    }
    if (amlbt_manf_cnt == 0)
    {
        UINT16_TO_STREAM(p, HCI_VSC_WAKE_WRITE_DATA);
        for (i = 0; i < sizeof(default_manf_data); i++)
            *p++ = default_manf_data[i];
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + default_manf_data[0];
        hw_cfg_cb.state = HW_CFG_SET_MANU_DATA;
        //Set ADV parameter of remote controller using to wake up device when suspend
        is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_VSC_WAKE_WRITE_DATA, p_buf, hw_config_cback);
        ALOGE("%s aml_bt config manf error use default", __func__);

        return is_proceeding;
    }
    UINT16_TO_STREAM(p, HCI_VSC_WAKE_WRITE_DATA);
    if (amlbt_manf_cnt > 0)
    {
        for (i = 0; i < sizeof(default_manf_data); cnt++, i++)
        {
            *p++ = APCF_config_manf_data[cnt];
            amlbt_manf_cnt--;
        }
    }
    p_buf->len = HCI_CMD_PREAMBLE_SIZE + default_manf_data[0];

    if (amlbt_transtype.family_id == AML_W1)
    {
        cnt = 0;
        hw_cfg_cb.state = HW_CFG_SET_MANU_DATA;
        ALOGD("vendor lib fwcfg completed");
        ALOGD("vendor lib config manf data");
    }
    else
    {
        if (amlbt_manf_cnt == 0)
        {
            cnt = 0;
            hw_cfg_cb.state = HW_CFG_SET_MANU_DATA;
            ALOGD("vendor lib fwcfg completed");
            ALOGD("vendor lib config manf data");
        }
    }
    //Set ADV parameter of remote controller using to wake up device when suspend
    is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_VSC_WAKE_WRITE_DATA, p_buf, hw_config_cback);

    return is_proceeding;
}

uint8_t hw_cfg_set_wakeup_params(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    int i = 0;
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    char local_ver[128];
    char chip_name[128] = {0};
    uint8_t default_manf_data[] = {0x05, 0x19, 0xff, 0x01, 0x0a, 0xb}; //public version
    char *p_tmp = (char *)(p_evt_buf + 1) + \
                  HCI_EVT_CMD_CMPL_LOCAL_FW_VERSION;
    uint8_t is_proceeding = FALSE;
    if (cnt == 0)
    {
        property_get("persist.vendor.bt_name", chip_name, "unknown");
        int year = 2020 + (*(p_tmp + 1) >> 4)%16;
        int month= (*(p_tmp + 1) & 0x0F)%16;

        BTHWDBG("BT Controller model=%s,version = %04d.%02d.%02x,number = 0x%02x%02x", chip_name,year,month, *p_tmp, *(p_tmp + 3), *(p_tmp + 2));
        snprintf(local_ver, sizeof(local_ver), "model=%s,version = %04d.%02d.%02x,number = 0x%02x%02x",chip_name, year,month, *p_tmp, *(p_tmp + 3), *(p_tmp + 2));

        if (property_set(VENDOR_AMLBTVER_PROPERTY, (char *)local_ver) < 0)
        {
            ALOGE("%s:Failed to set amlbt version in %s", __func__, VENDOR_AMLBTVER_PROPERTY);
        }
    }

    if (amlbt_manf_cnt == 0)
    {
        UINT16_TO_STREAM(p, HCI_VSC_WAKE_WRITE_DATA);
        for (i = 0; i < sizeof(default_manf_data); i++)
            *p++ = default_manf_data[i];
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + default_manf_data[0];
        hw_cfg_cb.state = HW_CFG_SET_MANU_DATA;
        //Set ADV parameter of remote controller using to wake up device when suspend
        is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_VSC_WAKE_WRITE_DATA, p_buf, hw_config_cback);
        ALOGE("%s aml_bt config manf error use default", __func__);

        return is_proceeding;
    }
    UINT16_TO_STREAM(p, HCI_VSC_WAKE_WRITE_DATA);
    if (amlbt_manf_cnt > 0)
    {
        for (i = 0; i < sizeof(default_manf_data); cnt++, i++)
        {
            *p++ = APCF_config_manf_data[cnt];
            amlbt_manf_cnt--;
        }
    }
    p_buf->len = HCI_CMD_PREAMBLE_SIZE + default_manf_data[0];

    if (amlbt_manf_cnt == 0)
    {
        cnt = 0;
        hw_cfg_cb.state = HW_CFG_SET_MANU_DATA;
        ALOGD("vendor lib fwcfg completed");
        ALOGD("vendor lib config manf data");
    }

    //Set ADV parameter of remote controller using to wake up device when suspend
    is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_VSC_WAKE_WRITE_DATA, p_buf, hw_config_cback);

    return is_proceeding;
}

uint8_t wole_config_write_manufacture(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    bt_vendor_cbacks->dealloc(p_buf);
    bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);

    hw_cfg_cb.state = 0;
    if (hw_cfg_cb.fw_fd != -1)
    {
        close(hw_cfg_cb.fw_fd);
        hw_cfg_cb.fw_fd = -1;
    }
    return TRUE;
}

/*******************************************************************************
**
** Function         hw_config_cback
**
** Description      Callback function for controller configuration
**
** Returns          None
**
*******************************************************************************/
void hw_config_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    char *p_name, *p_tmp;
    uint8_t *p, status;
    uint16_t opcode;
    HC_BT_HDR *p_buf = NULL;
    uint8_t is_proceeding = FALSE;
    int i;
    int delay = 100;
    char tempBuf[8];
    char *file = NULL;

#if (USE_CONTROLLER_BDADDR == TRUE)
    const uint8_t null_bdaddr[BD_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };
#endif
    char local_ver[kverLength + 1];

    status = *((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE);
    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode, p);

    /* Ask a new buffer big enough to hold any HCI commands sent in here */
    if ((status == 0) && bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_MAX_LEN);

    //BTHWDBG("-------------hw_config_cback %#x", hw_cfg_cb.state);
    if (p_buf != NULL)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->len = 0;
        p_buf->layer_specific = 0;

        p = (uint8_t *)(p_buf + 1);

        is_proceeding = hw_config_func[hw_cfg_cb.state](p_mem, p_buf, p);
    }

    /* Free the RX event buffer */
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);

    if (is_proceeding == FALSE)
    {
        ALOGE("vendor lib fwcfg aborted!!!");
        if (bt_vendor_cbacks)
        {
            if (p_buf != NULL)
                bt_vendor_cbacks->dealloc(p_buf);

            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
        }

        if (hw_cfg_cb.fw_fd != -1)
        {
            close(hw_cfg_cb.fw_fd);
            hw_cfg_cb.fw_fd = -1;
        }

        hw_cfg_cb.state = 0;
    }
}

/******************************************************************************
**   LPM Static Functions
******************************************************************************/

/*******************************************************************************
**
** Function         hw_lpm_ctrl_cback
**
** Description      Callback function for lpm enable/disable request
**
** Returns          None
**
*******************************************************************************/

#if (SCO_CFG_INCLUDED == TRUE)
/*****************************************************************************
**   SCO Configuration Static Functions
*****************************************************************************/

/*******************************************************************************
**
** Function         hw_sco_i2spcm_cfg_cback
**
** Description      Callback function for SCO I2S/PCM configuration request
**
** Returns          None
**
*******************************************************************************/
static void hw_sco_i2spcm_cfg_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *p;
    uint16_t opcode;
    HC_BT_HDR *p_buf = NULL;
    bt_vendor_op_result_t status = BT_VND_OP_RESULT_FAIL;

    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode, p);

    if (*((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE) == 0)
    {
        status = BT_VND_OP_RESULT_SUCCESS;
    }

    /* Free the RX event buffer */
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);

    if (status == BT_VND_OP_RESULT_SUCCESS)
    {
        if ((opcode == HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM) &&
                (SCO_INTERFACE_PCM == sco_bus_interface))
        {
            uint8_t ret = FALSE;

            /* Ask a new buffer to hold WRITE_SCO_PCM_INT_PARAM command */
            if (bt_vendor_cbacks)
                p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(
                            BT_HC_HDR_SIZE + HCI_CMD_PREAMBLE_SIZE + SCO_PCM_PARAM_SIZE);
            if (p_buf)
            {
                p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
                p_buf->offset = 0;
                p_buf->layer_specific = 0;
                p_buf->len = HCI_CMD_PREAMBLE_SIZE + SCO_PCM_PARAM_SIZE;
                p = (uint8_t *)(p_buf + 1);

                /* do we need this VSC for I2S??? */
                UINT16_TO_STREAM(p, HCI_VSC_WRITE_SCO_PCM_INT_PARAM);
                *p++ = SCO_PCM_PARAM_SIZE;
                memcpy(p, &bt_sco_param, SCO_PCM_PARAM_SIZE);
                ALOGI("SCO PCM configure {0x%x, 0x%x, 0x%x, 0x%x, 0x%x}",
                      bt_sco_param[0], bt_sco_param[1], bt_sco_param[2], bt_sco_param[3],
                      bt_sco_param[4]);
                if ((ret = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_SCO_PCM_INT_PARAM, p_buf,
                                                     hw_sco_i2spcm_cfg_cback)) == FALSE)
                {
                    bt_vendor_cbacks->dealloc(p_buf);
                }
                else
                    return;
            }
            status = BT_VND_OP_RESULT_FAIL;
        }
        else if ((opcode == HCI_VSC_WRITE_SCO_PCM_INT_PARAM) &&
                 (SCO_INTERFACE_PCM == sco_bus_interface))
        {
            uint8_t ret = FALSE;

            /* Ask a new buffer to hold WRITE_PCM_DATA_FORMAT_PARAM command */
            if (bt_vendor_cbacks)
                p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(
                            BT_HC_HDR_SIZE + HCI_CMD_PREAMBLE_SIZE + PCM_DATA_FORMAT_PARAM_SIZE);
            if (p_buf)
            {
                p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
                p_buf->offset = 0;
                p_buf->layer_specific = 0;
                p_buf->len = HCI_CMD_PREAMBLE_SIZE + PCM_DATA_FORMAT_PARAM_SIZE;

                p = (uint8_t *)(p_buf + 1);
                UINT16_TO_STREAM(p, HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM);
                *p++ = PCM_DATA_FORMAT_PARAM_SIZE;
                memcpy(p, &bt_pcm_data_fmt_param, PCM_DATA_FORMAT_PARAM_SIZE);

                ALOGI("SCO PCM data format {0x%x, 0x%x, 0x%x, 0x%x, 0x%x}",
                      bt_pcm_data_fmt_param[0], bt_pcm_data_fmt_param[1],
                      bt_pcm_data_fmt_param[2], bt_pcm_data_fmt_param[3],
                      bt_pcm_data_fmt_param[4]);

                if ((ret = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM,
                                                     p_buf, hw_sco_i2spcm_cfg_cback)) == FALSE)
                {
                    bt_vendor_cbacks->dealloc(p_buf);
                }
                else
                    return;
            }
            status = BT_VND_OP_RESULT_FAIL;
        }
    }

    ALOGI("sco I2S/PCM config result %d [0-Success, 1-Fail]", status);
    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->audio_state_cb(status);
    }
}

/*******************************************************************************
**
** Function         hw_set_MSBC_codec_cback
**
** Description      Callback function for setting WBS codec
**
** Returns          None
**
*******************************************************************************/
static void hw_set_MSBC_codec_cback(void *p_mem)
{
    /* whenever update the codec enable/disable, need to update I2SPCM */
    ALOGI("SCO I2S interface change the sample rate to 16K");
    hw_sco_i2spcm_config(p_mem, SCO_CODEC_MSBC);
}

/*******************************************************************************
**
** Function         hw_set_CVSD_codec_cback
**
** Description      Callback function for setting NBS codec
**
** Returns          None
**
*******************************************************************************/
static void hw_set_CVSD_codec_cback(void *p_mem)
{
    /* whenever update the codec enable/disable, need to update I2SPCM */
    ALOGI("SCO I2S interface change the sample rate to 8K");
    hw_sco_i2spcm_config(p_mem, SCO_CODEC_CVSD);
}

#endif // SCO_CFG_INCLUDED
void Insert32_Uint32(uint8_t *p_buffer, uint32_t data_32_bit)
{
    p_buffer[0] = data_32_bit & 0xFF;
    p_buffer[1] = ((data_32_bit >> 8) & 0xFF);
    p_buffer[2] = ((data_32_bit >> 16) & 0xFF);
    p_buffer[3] = ((data_32_bit >> 24) & 0xFF);
}

/*****************************************************************************
**   Hardware Configuration Interface Functions
*****************************************************************************/
static int hci_send_cmd(int fd, unsigned char *cmd, int cmdsize)
{
    int err = 0;

    //ALOGD("%s [abner test]: ", __FUNCTION__);
    err = do_write(fd, cmd, cmdsize);
    if (err != cmdsize)
    {
        ALOGE("%s: Send failed with ret value: %d", __FUNCTION__, err);
        err = -1;
    }
    return err;
}

void hw_config_quick_start(void)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    uint8_t is_proceeding = FALSE;

    hw_cfg_cb.state = 0;
    hw_cfg_cb.fw_fd = -1;
    hw_cfg_cb.f_set_baud_2 = FALSE;

    ALOGD("hw_config_quick_start-------------\n");

    if (bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE + 8);
    }

    if (p_buf)
    {
        cnt = 0;
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p = (uint8_t *)(p_buf + 1);
        BTHWDBG("Setting local bd addr to %02X:%02X:%02X:%02X:%02X:%02X",
            vnd_local_bd_addr[0], vnd_local_bd_addr[1], vnd_local_bd_addr[2],
            vnd_local_bd_addr[3], vnd_local_bd_addr[4], vnd_local_bd_addr[5]);

        UINT16_TO_STREAM(p, HCI_VSC_WRITE_BD_ADDR);
        *p++ = BD_ADDR_LEN; /* parameter length */
        *p++ = vnd_local_bd_addr[5];
        *p++ = vnd_local_bd_addr[4];
        *p++ = vnd_local_bd_addr[3];
        *p++ = vnd_local_bd_addr[2];
        *p++ = vnd_local_bd_addr[1];
        *p = vnd_local_bd_addr[0];

        p_buf->len = HCI_CMD_PREAMBLE_SIZE + BD_ADDR_LEN;
        if (amlbt_transtype.family_id == AML_W2)
        {
            hw_cfg_cb.state = HW_CFG_SET_WAVEFORM_DATA;
        }
        else
        {
            hw_cfg_cb.state = HW_CFG_SET_WAKEUP_PARAMS;
        }
        bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_BD_ADDR, p_buf, hw_config_cback);
    }
    else
    {
        if (bt_vendor_cbacks)
        {
            ALOGE("vendor lib fw conf aborted [no buffer]");
            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
        }
    }
}


/*******************************************************************************
**
** Function        hw_config_start
**
** Description     Kick off controller initialization process
**
** Returns         None
**
*******************************************************************************/
void hw_config_start(void)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    uint8_t is_proceeding = FALSE;
    static int num;
    char *file = NULL;
    char tempBuf[12];
    uint8_t uart_cmd[12] = {0x01};
    uint8_t uart_cmd_rsp[HCI_MAX_EVENT_SIZE] = {0};
    num = 50;
    hw_cfg_cb.state = 0;
    hw_cfg_cb.fw_fd = -1;
    hw_cfg_cb.f_set_baud_2 = FALSE;
    int size = 0;
    int ret = 0;
    int retry = 0;

    ALOGD("hw_config_start-------------\n");

    if (bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE + 8);
    }

    if (p_buf)
    {
        ALOGD("hw_config_start uart-------------\n");


        if (w1_bt_power)
        {
            p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
            p_buf->offset = 0;
            p_buf->layer_specific = 0;
            p_buf->len = HCI_CMD_PREAMBLE_SIZE;

            p = (uint8_t *)(p_buf + 1);
            UINT16_TO_STREAM(p, HCI_RESET);
            *p = 0;

            hw_cfg_cb.state = HW_CFG_SET_MANU_DATA;

            BTHWDBG(" hw_config_start reset\n");
            bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_config_cback);
        }
        else
        {
            int size = 0;
            p = (uint8_t *)(p_buf + 1);
            if (amlbt_transtype.family_id == AML_UNKNOWN)
            {
                hw_read_type(p_buf);
                is_proceeding = TRUE;
            }
            else
            {
#ifdef AML_DOWNLOADFW_UART
                hw_get_bin_size();
#endif
                UINT16_TO_STREAM(p, TCI_WRITE_REG);
                *p++ = 8;

                UINT32_TO_STREAM(p, 0xa70014);
                UINT32_TO_STREAM(p, 0x1000000); /*enable download event */

                p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                             8;
                hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_TEST_WRITE_UART;

                BTHWDBG("uart_target_baud_rate = %d", UART_TARGET_BAUD_RATE);
                file = amlbt_fw_bin[amlbt_transtype.family_id][amlbt_transtype.interface];

                if ((fw_fd = open_file(file, O_RDONLY)) > 0)
                {
                    if (add_iccm != 0 && amlbt_transtype.family_id == AML_W1U && amlbt_transtype.interface != AML_INTF_USB)
                    {
                        size = read(fw_fd, tempBuf, 12);
                        if (size < 0)
                        {
                            ALOGE("In %s, Read head failed:%s", __FUNCTION__, strerror(errno));
                            close(fw_fd);
                            return ;
                        }
                    }
                    else
                    {
                        size = read(fw_fd, tempBuf, 8);
                        if (size < 0)
                        {
                            ALOGE("In %s, Read head failed:%s", __FUNCTION__, strerror(errno));
                            close(fw_fd);
                            return ;
                        }
                    }
                    size = read(fw_fd, p_iccm_buf, iccm_size);
                    if (size < 0)
                    {
                        ALOGE("In %s, Read iccm failed:%s", __FUNCTION__, strerror(errno));
                        close(fw_fd);
                        return ;
                    }
                    //W1U skip 15KB additional iccm
                    if (amlbt_transtype.family_id == AML_W1U && amlbt_transtype.interface != AML_INTF_USB)
                    {
                        BTHWDBG("w1u skip %d additional iccm", add_iccm);
                        size = lseek(fw_fd, add_iccm, SEEK_CUR);
                        if (size < 0)
                        {
                            ALOGE("In %s, lseek failed:%s", __FUNCTION__, strerror(errno));
                            close(fw_fd);
                            return ;
                        }
                        BTHWDBG("w1u skip additional iccm result %d", size);
                    }
                    size = read(fw_fd, p_dccm_buf, dccm_size);
                    BTHWDBG("p_dccm_buf [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]",
                        p_dccm_buf[0],p_dccm_buf[1],p_dccm_buf[2],p_dccm_buf[3],
                        p_dccm_buf[4],p_dccm_buf[5],p_dccm_buf[6],p_dccm_buf[7]);
                    if (size < 0)
                    {
                        ALOGE("In %s, Read dccm failed:%s", __FUNCTION__, strerror(errno));
                        close(fw_fd);
                        return ;
                    }
                }
                is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                                p_buf, hw_config_cback);
            }
        }
    }
    else
    {
        if (bt_vendor_cbacks)
        {
            ALOGE("vendor lib fw conf aborted [no buffer]");
            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
        }
    }
}

/*******************************************************************************
**
** Function        hw_lpm_enable
**
** Description     Enalbe/Disable LPM
**
** Returns         TRUE/FALSE
**
*******************************************************************************/
uint8_t hw_lpm_enable(uint8_t turn_on)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    uint8_t ret = TRUE;

    if (bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE + \
                LPM_CMD_PARAM_SIZE);

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + LPM_CMD_PARAM_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, HCI_VSC_WRITE_SLEEP_MODE);
        *p++ = LPM_CMD_PARAM_SIZE; /* parameter length */

        if (turn_on)
        {
            memcpy(p, &lpm_param, LPM_CMD_PARAM_SIZE);
            upio_set(UPIO_LPM_MODE, UPIO_ASSERT, 0);
        }
        else
        {
            memset(p, 0, LPM_CMD_PARAM_SIZE);
            upio_set(UPIO_LPM_MODE, UPIO_DEASSERT, 0);
        }

        if (0 == *p)
        {
            ALOGE("LPM disabled!!");
        }
        else
        {
            ALOGD("LPM enabled!!");
        }

        if (amlbt_transtype.interface == AML_INTF_PCIE || amlbt_transtype.interface == AML_INTF_SDIO)
        {
            bt_vendor_cbacks->dealloc(p_buf);
        }
    }
/*
    if ((ret == FALSE) && bt_vendor_cbacks)
        bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_FAIL);
*/
    return ret;
}


/*******************************************************************************
**
** Function        hw_lpm_get_idle_timeout
**
** Description     Calculate idle time based on host stack idle threshold
**
** Returns         idle timeout value
**
*******************************************************************************/
uint32_t hw_lpm_get_idle_timeout(void)
{
    return 3000;
#if 0
    uint32_t timeout_ms;

    /* set idle time to be LPM_IDLE_TIMEOUT_MULTIPLE times of
     * host stack idle threshold (in 300ms/25ms)
     */
    timeout_ms = (uint32_t)lpm_param.host_stack_idle_threshold \
                 * LPM_IDLE_TIMEOUT_MULTIPLE;

    if (strstr(hw_cfg_cb.local_chip_name, "BCM4325") != NULL)
        timeout_ms *= 25; // 12.5 or 25 ?
    else
        timeout_ms *= 300;

    return timeout_ms;
#endif
}

/*******************************************************************************
**
** Function        hw_lpm_set_wake_state
**
** Description     Assert/Deassert BT_WAKE
**
** Returns         None
**
*******************************************************************************/
void hw_lpm_set_wake_state(uint8_t wake_assert)
{
    uint8_t state = (wake_assert) ? UPIO_ASSERT : UPIO_DEASSERT;

    upio_set(UPIO_BT_WAKE, state, lpm_param.bt_wake_polarity);
}

#if (SCO_CFG_INCLUDED == TRUE)
/*******************************************************************************
**
** Function         hw_sco_config
**
** Description      Configure SCO related hardware settings
**
** Returns          None
**
*******************************************************************************/
static int hw_set_SCO_codec(uint16_t codec);
void hw_sco_config(void)
{
    if (SCO_INTERFACE_I2S == sco_bus_interface)
    {
        /* 'Enable' I2S mode */
        bt_sco_i2spcm_param[0] = 1;

        /* set nbs clock rate as the value in SCO_I2SPCM_IF_CLOCK_RATE field */
        sco_bus_clock_rate = bt_sco_i2spcm_param[3];
    }
    else
    {
        /* 'Disable' I2S mode */
        bt_sco_i2spcm_param[0] = 0;

        /* set nbs clock rate as the value in SCO_PCM_IF_CLOCK_RATE field */
        sco_bus_clock_rate = bt_sco_param[1];

        /* sync up clock mode setting */
        bt_sco_i2spcm_param[1] = bt_sco_param[4];
    }

    if (sco_bus_wbs_clock_rate == INVALID_SCO_CLOCK_RATE)
    {
        /* set default wbs clock rate */
        sco_bus_wbs_clock_rate = SCO_I2SPCM_IF_CLOCK_RATE4WBS;

        if (sco_bus_wbs_clock_rate < sco_bus_clock_rate)
            sco_bus_wbs_clock_rate = sco_bus_clock_rate;
    }

    /*
     *  To support I2S/PCM port multiplexing signals for sharing Bluetooth audio
     *  and FM on the same PCM pins, we defer Bluetooth audio (SCO/eSCO)
     *  configuration till SCO/eSCO is being established;
     *  i.e. in hw_set_audio_state() call.
     */

    hw_set_SCO_codec(BTM_SCO_CODEC_CVSD);

    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->scocfg_cb(BT_VND_OP_RESULT_SUCCESS);
    }
}

/*******************************************************************************
**
** Function         hw_sco_i2spcm_config
**
** Description      Configure SCO over I2S or PCM
**
** Returns          None
**
*******************************************************************************/
static void hw_sco_i2spcm_config(void *p_mem, uint16_t codec)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    bt_vendor_op_result_t status = BT_VND_OP_RESULT_FAIL;

    if (*((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE) == 0)
    {
        status = BT_VND_OP_RESULT_SUCCESS;
    }

    /* Free the RX event buffer */
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);

    if (status == BT_VND_OP_RESULT_SUCCESS)
    {
        HC_BT_HDR *p_buf = NULL;
        uint8_t *p, ret;
        uint16_t cmd_u16 = HCI_CMD_PREAMBLE_SIZE + SCO_I2SPCM_PARAM_SIZE;

        if (bt_vendor_cbacks)
            p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + cmd_u16);

        if (p_buf)
        {
            p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
            p_buf->offset = 0;
            p_buf->layer_specific = 0;
            p_buf->len = cmd_u16;

            p = (uint8_t *)(p_buf + 1);

            UINT16_TO_STREAM(p, HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM);
            *p++ = SCO_I2SPCM_PARAM_SIZE;
            if (codec == SCO_CODEC_CVSD)
            {
                bt_sco_i2spcm_param[2] = 0; /* SCO_I2SPCM_IF_SAMPLE_RATE  8k */
                bt_sco_i2spcm_param[3] = bt_sco_param[1] = sco_bus_clock_rate;
            }
            else if (codec == SCO_CODEC_MSBC)
            {
                bt_sco_i2spcm_param[2] = wbs_sample_rate; /* SCO_I2SPCM_IF_SAMPLE_RATE 16K */
                bt_sco_i2spcm_param[3] = bt_sco_param[1] = sco_bus_wbs_clock_rate;
            }
            else
            {
                bt_sco_i2spcm_param[2] = 0; /* SCO_I2SPCM_IF_SAMPLE_RATE  8k */
                bt_sco_i2spcm_param[3] = bt_sco_param[1] = sco_bus_clock_rate;
                ALOGE("wrong codec is use in hw_sco_i2spcm_config, goes default NBS");
            }
            memcpy(p, &bt_sco_i2spcm_param, SCO_I2SPCM_PARAM_SIZE);
            cmd_u16 = HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM;
            ALOGI("I2SPCM config {0x%x, 0x%x, 0x%x, 0x%x}",
                  bt_sco_i2spcm_param[0], bt_sco_i2spcm_param[1],
                  bt_sco_i2spcm_param[2], bt_sco_i2spcm_param[3]);

            if ((ret = bt_vendor_cbacks->xmit_cb(cmd_u16, p_buf, hw_sco_i2spcm_cfg_cback)) == FALSE)
            {
                bt_vendor_cbacks->dealloc(p_buf);
            }
            else
                return;
        }
        status = BT_VND_OP_RESULT_FAIL;
    }

    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->audio_state_cb(status);
    }
}

/*******************************************************************************
**
** Function         hw_set_SCO_codec
**
** Description      This functgion sends command to the controller to setup
**                              WBS/NBS codec for the upcoming eSCO connection.
**
** Returns          -1 : Failed to send VSC
**                   0 : Success
**
*******************************************************************************/
static int hw_set_SCO_codec(uint16_t codec)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    uint8_t ret;
    int ret_val = 0;
    tINT_CMD_CBACK p_set_SCO_codec_cback;

    BTHWDBG("hw_set_SCO_codec 0x%x", codec);

    if (bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(
                    BT_HC_HDR_SIZE + HCI_CMD_PREAMBLE_SIZE + SCO_CODEC_PARAM_SIZE);

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p = (uint8_t *)(p_buf + 1);

        UINT16_TO_STREAM(p, HCI_VSC_ENABLE_WBS);

        if (codec == SCO_CODEC_MSBC)
        {
            /* Enable mSBC */
            *p++ = SCO_CODEC_PARAM_SIZE;    /* set the parameter size */
            UINT8_TO_STREAM(p, 1);          /* enable */
            UINT16_TO_STREAM(p, codec);

            /* set the totall size of this packet */
            p_buf->len = HCI_CMD_PREAMBLE_SIZE + SCO_CODEC_PARAM_SIZE;

            p_set_SCO_codec_cback = hw_set_MSBC_codec_cback;
        }
        else
        {
            /* Disable mSBC */
            *p++ = (SCO_CODEC_PARAM_SIZE);  /* set the parameter size */
            UINT8_TO_STREAM(p, 0);          /* disable */
            UINT16_TO_STREAM(p, codec);

            /* set the totall size of this packet */
            p_buf->len = HCI_CMD_PREAMBLE_SIZE + SCO_CODEC_PARAM_SIZE;

            p_set_SCO_codec_cback = hw_set_CVSD_codec_cback;
            if ((codec != SCO_CODEC_CVSD) && (codec != SCO_CODEC_NONE))
            {
                ALOGW("SCO codec setting is wrong: codec: 0x%x", codec);
            }
        }

        if ((ret = bt_vendor_cbacks->xmit_cb(HCI_VSC_ENABLE_WBS, p_buf, p_set_SCO_codec_cback)) \
                == FALSE)
        {
            bt_vendor_cbacks->dealloc(p_buf);
            ret_val = -1;
        }
    }
    else
    {
        ret_val = -1;
    }

    return ret_val;
}

/*******************************************************************************
**
** Function         hw_set_audio_state
**
** Description      This function configures audio base on provided audio state
**
** Parameters        pointer to audio state structure
**
** Returns          0: ok, -1: error
**
*******************************************************************************/
int hw_set_audio_state(bt_vendor_op_audio_state_t *p_state)
{
    int ret_val = -1;

    if (!bt_vendor_cbacks)
        return ret_val;

    ret_val = hw_set_SCO_codec(p_state->peer_codec);
    return ret_val;
}

#else  // SCO_CFG_INCLUDED
int hw_set_audio_state(bt_vendor_op_audio_state_t *p_state)
{
    int ret_val = 0;
    ret_val = p_state->state;
    return -256;
}
#endif

#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
/*******************************************************************************
**
** Function        hw_set_patch_settlement_delay
**
** Description     Give the specific firmware patch settlement time in milliseconds
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_settlement_delay(char *p_conf_name, char *p_conf_value, int param)
{
    fw_patch_settlement_delay = atoi(p_conf_value);

    return 0;
}
#endif  //VENDOR_LIB_RUNTIME_TUNING_ENABLED

/*****************************************************************************
**   Sample Codes Section
*****************************************************************************/

#if (HW_END_WITH_HCI_RESET == TRUE)
/*******************************************************************************
**
** Function         hw_epilog_cback
**
** Description      Callback function for Command Complete Events from HCI
**                  commands sent in epilog process.
**
** Returns          None
**
*******************************************************************************/
void hw_epilog_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    uint8_t *p, status;
    uint16_t opcode;

    status = *((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE);
    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode, p);

    BTHWDBG("%s Opcode:0x%04X Status: %d", __FUNCTION__, opcode, status);

    if (bt_vendor_cbacks)
    {
        /* Must free the RX event buffer */
        bt_vendor_cbacks->dealloc(p_evt_buf);

        /* Once epilog process is done, must call epilog_cb callback
         * to notify caller */
        bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_SUCCESS);
    }
}

int hw_set_patch_file_path(char *p_conf_name __unused, char *p_conf_value __unused, int param __unused)
{
#ifdef AML_DOWNLOADFW_UART
    snprintf(fw_patchfile_path, sizeof(fw_patchfile_path), "/etc/bluetooth/");
#else
    if (p_conf_value == NULL) return -EINVAL;
    snprintf(fw_patchfile_path, sizeof(fw_patchfile_path), "%s", p_conf_value);
#endif

    return 0;
}

/*******************************************************************************
**
** Function        hw_set_patch_file_name
**
** Description     Give the specific firmware patch filename
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_file_name(char *p_conf_name __unused, char *p_conf_value __unused, int param __unused)
{
#ifdef AML_DOWNLOADFW_UART
    snprintf(fw_patchfile_name, sizeof(fw_patchfile_name), "bt_fucode.h");
#else
    if (p_conf_value == NULL) return -EINVAL;
    snprintf(fw_patchfile_name, sizeof(fw_patchfile_name), "%s", p_conf_value);
#endif

    return 0;
}

/*******************************************************************************
**
** Function         hw_epilog_process
**
** Description      Sample implementation of epilog process
**
** Returns          None
**
*******************************************************************************/
void hw_epilog_process(void)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;

    BTHWDBG("hw_epilog_process");

    /* Sending a HCI_RESET */
    if (bt_vendor_cbacks)
    {
        /* Must allocate command buffer via HC's alloc API */
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                HCI_CMD_PREAMBLE_SIZE);
    }

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE;

        p = (uint8_t *)(p_buf + 1);
        UINT16_TO_STREAM(p, HCI_RESET);
        *p = 0; /* parameter length */

        /* Send command via HC's xmit_cb API */
        bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_epilog_cback);
    }
    else
    {
        if (bt_vendor_cbacks)
        {
            ALOGE("vendor lib epilog process aborted [no buffer]");
            bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_FAIL);
        }
    }
}
#endif // (HW_END_WITH_HCI_RESET == TRUE)


void hw_read_type_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *)p_mem;
    unsigned int reg_value = 0;

    char *p_tmp;
    p_tmp = (char *)(p_evt_buf + 1) + \
            HCI_EVT_CMD_CMPL_LOCAL_NAME_STRING;
    reg_value = *p_tmp + ((*(p_tmp + 1)) << 8) + ((*(p_tmp + 2)) << 16) + ((*(p_tmp + 3)) << 24);

    ALOGD("[AML_USB] %s, %#x", __FUNCTION__, reg_value);

    if (reg_value == W1_RG_AON_A)
    {
        amlbt_transtype.family_id = AML_W1;
        amlbt_transtype.interface = AML_INTF_SDIO;
    }
    else if (reg_value == W1U_RG_AON_A)
    {
        amlbt_transtype.family_id = AML_W1U;
        amlbt_transtype.interface = AML_INTF_SDIO;
    }

    ALOGD("[AML_USB] %s amlbt_transtype(%d:%d:%d)\n", __FUNCTION__, amlbt_transtype.family_id,
          amlbt_transtype.family_rev, amlbt_transtype.interface);
}
uint8_t hw_cfg_get_reg(void *p_mem, HC_BT_HDR *p_buf, uint8_t *p)
{
    uint8_t is_proceeding = FALSE;
    char *file = NULL;
    char tempBuf[8];
    int size = 0;

    hw_read_type_cback(p_mem);
#ifdef AML_DOWNLOADFW_UART
    hw_get_bin_size();
#endif
    UINT16_TO_STREAM(p, TCI_WRITE_REG);
    *p++ = 8;

    UINT32_TO_STREAM(p, 0xa70014);
    UINT32_TO_STREAM(p, 0x1000000); /*enable download event */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 8;
    hw_cfg_cb.state = HW_CFG_AML_DOWNLOAD_FIRMWARE_TEST_WRITE_UART;

    BTHWDBG("uart_target_baud_rate = %d", UART_TARGET_BAUD_RATE);
    file = amlbt_fw_bin[amlbt_transtype.family_id][amlbt_transtype.interface];

    if ((fw_fd = open_file(file, O_RDONLY)) > 0)
    {
        size = read(fw_fd, tempBuf, 8);
        if (size < 0)
        {
            ALOGE("In %s, Read head failed:%s", __FUNCTION__, strerror(errno));
            close(fw_fd);
            return FALSE;
        }
        size = read(fw_fd, p_iccm_buf, iccm_size);
        if (size < 0)
        {
            ALOGE("In %s, Read iccm failed:%s", __FUNCTION__, strerror(errno));
            close(fw_fd);
            return FALSE;
        }
        size = read(fw_fd, p_dccm_buf, dccm_size);
        if (size < 0)
        {
            ALOGE("In %s, Read dccm failed:%s", __FUNCTION__, strerror(errno));
            close(fw_fd);
            return FALSE;
        }
    }
    is_proceeding = bt_vendor_cbacks->xmit_cb(TCI_WRITE_REG, \
                    p_buf, hw_config_cback);

    return is_proceeding;
}

void hw_read_type(HC_BT_HDR *p_buf)
{
    uint8_t *p;

    ALOGD("[AML_USB] %s,%p 1", __FUNCTION__, p_buf);
    ALOGD("[AML_USB] %s 2", __FUNCTION__);
    p = (uint8_t *)(p_buf + 1);

    UINT16_TO_STREAM(p, TCI_READ_REG);
    *p++ = 4;                               /* parameter length */
    UINT32_TO_STREAM(p, RG_AON_A);     /* addr */
    ALOGD("[AML_USB] %s 3", __FUNCTION__);
    p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
                 4;
    bt_vendor_cbacks->xmit_cb(TCI_READ_REG, \
                              p_buf, hw_config_cback);
    ALOGD("[AML_USB] %s end", __FUNCTION__);
    hw_cfg_cb.state = HW_CFG_GET_REG;
}

void aml_15p4_tx(unsigned char *data, unsigned short len)
{
    HC_BT_HDR *p_buf = NULL;
    uint8_t *p;
    uint8_t is_proceeding = FALSE;
    static int num;
    char *file = NULL;
    hw_cfg_cb.state = 0;
    hw_cfg_cb.fw_fd = -1;
    hw_cfg_cb.f_set_baud_2 = FALSE;

    if (bt_vendor_cbacks)
    {
        p_buf = (HC_BT_HDR *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + AML_15P4_CMD_BUF_SIZE);
        if (p_buf)
        {
            p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
            p_buf->offset = 0;
            p_buf->layer_specific = 0;
            p = (uint8_t *)(p_buf + 1);
            memcpy(p, data, len);
            p_buf->len = len;
            bt_vendor_cbacks->xmit_cb(HCI_AML_15P4_CMD, p_buf, aml_15p4_data_cb);
        }
    }
}

