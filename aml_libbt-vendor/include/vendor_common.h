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

#ifndef VENDOR_COMMON_H
#define VENDOR_COMMON_H

struct amlbt_diag_entry {
    unsigned char  type;
    unsigned int w;           // write pointer
    unsigned int r;           // read pointer

    unsigned char  mon;
    unsigned char  day;
    unsigned char  hour;
    unsigned char  min;
    unsigned char  sec;
    unsigned short ms;          // [0-999]

    unsigned char  opcode;      //
    unsigned char  info[7];     //
    unsigned int fw_log_cnt;  //
} __packed;

struct amlbt_diag_buf
{
    unsigned int count;
    unsigned int max;
    unsigned char fw_log[516];
    struct amlbt_diag_entry entries[];
} __packed;

struct amlbt_diag_remain_buf
{
    unsigned int count;
    unsigned int max;
    struct amlbt_diag_entry entries[0];
};

//ioctl
#define BTUSB_IOC_MAGIC 'x'
#define IOCTL_GET_BT_RECOVERY                   _IOR(BTUSB_IOC_MAGIC, 0, int)
#define IOCTL_GET_DEVICE_PID                    _IOR(BTUSB_IOC_MAGIC, 1, int)
//debug dev 2-6
#define IOCTL_SET_BT_SHUTDOWN                   _IOW(BTUSB_IOC_MAGIC, 7, int)
#define IOCTL_GET_COEX_STATUS                   _IOR(BTUSB_IOC_MAGIC, 8, int)
#define IOCTL_REGISTER_SDIO                     _IOW(BTUSB_IOC_MAGIC, 9, int)
#define IOCTL_UNREGISTER_SDIO                   _IOW(BTUSB_IOC_MAGIC, 10, int)
#define IOCTL_GET_SDIO_PROBE_STATUS             _IOR(BTUSB_IOC_MAGIC, 11, int)
#define IOCTL_SET_BT_RECOVERY                   _IOW(BTUSB_IOC_MAGIC, 12, int)
#define IOCTL_SET_BT_UART_RESET                 _IO(BTUSB_IOC_MAGIC, 13)
#define IOCTL_GET_DEVICE_CID                    _IOR(BTUSB_IOC_MAGIC, 14, int)
#define IOCTL_SET_BT_EN_ENABLE                  _IO(BTUSB_IOC_MAGIC, 16)

#define IOCTL_GET_DRIVER_VERSION                _IOR(BTUSB_IOC_MAGIC, 20, int)
#define IOCTL_GET_DIAG_COUNT                    _IOR(BTUSB_IOC_MAGIC, 21, int)
#define IOCTL_GET_DIAG_BUFF                     _IOR(BTUSB_IOC_MAGIC, 22, struct amlbt_diag_buf)
#define IOCTL_GET_DIAG_REMAIN_COUNT             _IOR(BTUSB_IOC_MAGIC, 23, int)
#define IOCTL_GET_DIAG_REMAIN_BUFF              _IOR(BTUSB_IOC_MAGIC, 24, struct amlbt_diag_remain_buf)

#define W1U_ROM_START_CODE                      0x0cc0006f

//vendor cmd
#define HCI_READ_LOCAL_INFO                     0x1001
#define HCI_RESET                               0x0C03
#define HCI_VSC_WRITE_UART_CLOCK_SETTING        0xFC45
#define HCI_VSC_UPDATE_BAUDRATE                 0xFC18
#define HCI_READ_LOCAL_NAME                     0x0C14
#define HCI_VSC_DOWNLOAD_MINIDRV                0xFC2E
#define HCI_VSC_WRITE_BD_ADDR                   0xFC1A
#define HCI_VSC_WRITE_PARAMS                    0xFC20
#define HCI_VSC_WRITE_SLEEP_MODE                0xFC27
#define HCI_VSC_WRITE_SCO_PCM_INT_PARAM         0xFC1C
#define HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM     0xFC1E
#define HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM    0xFC6D
#define HCI_VSC_ENABLE_WBS                      0xFC7E
#define HCI_VSC_LAUNCH_RAM                      0xFC4E
#define HCI_READ_LOCAL_BDADDR                   0x1009
#define HCI_RECOVERY_CMD                        0xFF98
#define HCI_CHECK_CMD                           0xfC51
#define HCI_VSC_WAKE_WRITE_DATA                 0xFC22
#define HCI_HOST_SLEEP_VSC                      0xfc21
#define HCI_FW_CLEAR_LIST                       0xfc55
#define HCI_AML_15P4_CMD                        0xFF9A

//vendor quantification
#define HCI_EVT_CMD_CMPL_STATUS_RET_BYTE        5
#define HCI_EVT_CMD_CMPL_LOCAL_NAME_STRING      6
#define HCI_EVT_CMD_CMPL_LOCAL_BDADDR_ARRAY     6
#define HCI_EVT_CMD_CMPL_LOCAL_FW_VERSION       6
#define HCI_EVT_CMD_CMPL_OPCODE                 3
#define LPM_CMD_PARAM_SIZE                      12
#define UPDATE_BAUDRATE_CMD_PARAM_SIZE          6
#define HCI_CMD_PREAMBLE_SIZE                   3
#define HCD_REC_PAYLOAD_LEN_BYTE                2
#define BD_ADDR_LEN                             6
#define LOCAL_BDADDR_PATH_BUFFER_LEN            256
#define HCI_CMD_MAX_LEN                         258
#define BIT_PHY                                 1
#define BIT_MAC                                 (1 << 1)
#define BIT_CPU                                 (1 << 2)
#define DEV_RESET_SW                            16
#define BIT_RF_NUM                              28
#define BT_SINK_MODE                            25
#define SCO_INTERFACE_PCM                       0
#define SCO_INTERFACE_I2S                       1
#define RG_AON_A                                (0x00f00094)
#define W1U_RG_AON_A                            (0x1)
#define W1_RG_AON_A                             (0x0)
#define MIN_ALLOC_SIZE                          1
#define MAX_ALLOC_SIZE                          (1024 * 1024) // max 1MB
#define MAX_READ_EVENT_CNT                      40

//bt used reg
#define REG_DEV_RESET                           0xf03058
#define REG_PMU_POWER_CFG                       0xf03040
#define REG_RAM_PD_SHUTDWONW_SW                 0xf03050
#define REG_FW_MODE                             0xf000e0
#define RG_AON_A53                              0xf000d4
#define RG_AON_A59                              0xf000ec

//debug dev
#define AML_BT_CHAR_DEBUG_DEVICE_ADDR "/dev/aml_bt_debug"

//15.4 socket
#define rev_15p4_cmd_fifo "/data/vendor/bluetooth/fifo_cmd_out"
#define rsp_15p4_rst_fifo "/data/vendor/bluetooth/fifo_cmd_in"

//Local type definitions
#define HCI_MAX_EVENT_SIZE      260

//coxe bit
#define ZIGBEE_ALIVE            (1 << 1)
#define THREAD_ALIVE            (1 << 2)
#define BIT(_n)                 (1 << (_n))

typedef int (*libbt_func_t)(int state, int (*fd_array)[]);

//extern variable
extern unsigned int amlbt_fw_mode;
extern unsigned int amlbt_poweron;
extern int g_userial_fd;
extern int bt_sdio_fd;
extern libbt_func_t *libbt_interface;
extern pthread_t aml_15p4_handle_thread;
extern bool exit_thread;

extern const char PWR_PROP_NAME[];
extern char shutdwon_status[PROPERTY_VALUE_MAX];
extern char driver_pram[PROPERTY_VALUE_MAX];
extern int w1_bt_power;

//extern interface
extern void aml_15p4_tx(unsigned char *data, unsigned short len);
extern void ms_delay(uint32_t timeout);
extern uint8_t line_speed_to_userial_baud(uint32_t line_speed);
extern int (*libbtw2l_uart_func[])(int state, int (*fd_array)[]);
extern int (*libbtw2l_usb_func[])(int state, int (*fd_array)[]);
extern int (*libbtw2_uart_func[])(int state, int (*fd_array)[]);
extern int (*libbtw2_usb_func[])(int state, int (*fd_array)[]);
extern int (*libbtw1u_uart_func[])(int state, int (*fd_array)[]);
extern int (*libbtw1u_usb_func[])(int state, int (*fd_array)[]);
extern int (*libbtw1_func[])(int state, int (*fd_array)[]);
extern int amlbt_interface_init(void);
extern void amlbt_interface_exit(void);
extern void hw_config_start(void);
extern void hw_config_quick_start(void);
extern void load_aml_stack_conf(void);
extern void vnd_load_conf(const char *p_path);
extern uint32_t hw_lpm_get_idle_timeout(void);
extern uint8_t hw_lpm_enable(uint8_t turn_on);
extern void hw_lpm_set_wake_state(uint8_t wake_assert);

unsigned int amlbt_get_reg(unsigned int addr);
void libbt_save_diag_buff(int fd);
void save_regs_to_file(unsigned char *buf, size_t len, const char *filepath);
void save_regs_with_time_str(unsigned char *buf, size_t len, const char *filepath);

//common interface
int driver_check(const char *modname, int timeout_ms);
int insmod_check(const char *modname);
int insmod(const char *filename, const char *args, const char *modname, int timeout_ms);
int rmmod(const char *modname, int timeout_ms);
int do_write(int fd, unsigned char *buf, int len);
int read_hci_event(int fd, unsigned char *buf, int size);
int aml_hci_send_cmd(int fd, unsigned char *cmd, int cmdsize, unsigned char *rsp);
int hci_write_cmd(int fd, unsigned char *buf, int len);
int hci_read_event(int fd, unsigned char *buf, int size);
int aml_woble_configure(int fd);
void aml_reset_bt(int fd);
void aml_reset_bt_poll(int fd);
unsigned char aml_get_w2l_coex_status(int fd);
void aml_get_w2l_chip_function(int fd);
int aml_uart_init(void);
int aml_uart_get_pmu(void);
int aml_uart_rtl_dbg(unsigned int addr);
void aml_15p4_data_cb(void *p);
void aml_15p4_deinit(void);
void* aml_15p4_socket(void* arg);
int amlbt_chardev_open(char *addr);

enum
{
    FW_MODE_BT_ONLY      = 1,
    FW_MODE_15P4_ONLY    = 2,
    FW_MODE_COEX         = 3,
};

#endif /* VENDOR_COMMON_H */
