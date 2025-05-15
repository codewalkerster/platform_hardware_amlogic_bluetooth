#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/usb.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/io.h>
#include <linux/compat.h>
#include <linux/firmware.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/amlogic/pm.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/sched/clock.h>
#endif

#include "common.h"
#include "w2l_bt_entry.h"
#include "w2l_usb_bt_new.h"
#include "rc_list.h"
#include "debug_dev.h"
#include "amlbt_utils.h"

/*------------------------------------------------------------------------------
Event FIFO,   start:0x00514000, end:0x005151fc, length:4604 bytes
    Rx Type FIFO Read Pointer     : 0x00514000  [0,1,2,3]
    HCI Event FIFO Read Pointer   : 0x00514004  [4,5,6,7]
    15p4 rx data fifo r           : 0x00514008  [8,9,10,11]
    rx data fifo r reg            : 0x0051400c  [12,13,14,15]
    rx data fifo w reg            : 0x00514010  [16,17,18,19]
    sink mode status              : 0x00514014  [20,21,22,23]
    dummy                         : 0x00514018  [24,25,26,27]
    15p4 rx data fifo w           : 0x0051401c  [28,29,30,31]
    Rx Type FIFO Write Pointer    : 0x00514020  [32,33,34,35]
    HCI Event FIFO Write Pointer  : 0x00514024  [36,37,38,39]
    15p4 data tx fifo r           : 0x00514028  [40,41,42,43]
    15p4 data tx fifo w           : 0x0051402c  [44,45,46,47]
    dummy                         : 0x00514030  [48,49,50,51]
    dummy                         : 0x00514034  [52,53,54,55]
    dummy                         : 0x00514038  [56,57,58,59]
    Rx Type FIFO                  : 0x0051403c  [256 bytes]
    HCI Event FIFO                : 0x0051413c  [2048 bytes]

Tx Queue,     start:0x00508000, end:0x0050a3fc, length:9212 bytes

Register RAM, start:0x00510000, end:0x00510200, length:512 bytes
    HCI Command FIFO Read Pointer : 0x00510000
    HCI Command FIFO Write Pointer: 0x00510004

    Tx Queue Prio Pointer         : 0x00510018
    Tx Queue Acl Handle Pointer   : 0x0051001c
    Tx Queue Status Pointer       : 0x00510020
    Dummy                         : 0x00510024

    Driver Firmware Status Pointer: 0x005101fc

Rx Queue,     start:0x00500000, end:0x00501000, length:4096 bytes

Command FIFO, start:0x00518000, end:0x00519000, length:4096 bytes
---------------------------------------------------------------------------------*/

#define GDSL_TX_Q_MAX           (8)
#define GDSL_TX_Q_USED          (1)
#define GDSL_TX_Q_COMPLETE      (2)
#define GDSL_TX_Q_UNUSED        (0)

#define USB_TX_Q_NUM            (8)
#define USB_TX_Q_LEN            (1032)
#define USB_RX_Q_LEN            (1032)
#define POLL_TOTAL_LEN          (2364)  //2048 + 316 + 16 bytes
#define RC_MANFDATA_LEN         (6*8)

#define HI_USB_RX_Q_ADDR        0x00500000  //length:4096 bytes
#define HI_USB_TX_Q_ADDR        0x00508000  //length:4096 bytes
#define HI_USB_MEM_ADDR         0x00510000  //length:512 bytes
#define HI_USB_EVENT_Q_ADDR     0x00514000  //length:2364 bytes
#define HI_USB_CMD_Q_ADDR       0x00518000  //length:4096 bytes

#define FIFO_FW_RX_TYPE_ADDR    HI_USB_EVENT_Q_ADDR + 0x3c
#define FIFO_FW_RX_TYPE_LEN     (256)
#define FIFO_FW_RX_TYPE_R       HI_USB_EVENT_Q_ADDR
#define FIFO_FW_RX_TYPE_W       HI_USB_EVENT_Q_ADDR + 0x20

#define FIFO_FW_EVT_ADDR        HI_USB_EVENT_Q_ADDR + 0x13c
#define FIFO_FW_EVT_LEN         (2048)
#define FIFO_FW_EVT_R           HI_USB_EVENT_Q_ADDR + 0x04
#define FIFO_FW_EVT_W           HI_USB_EVENT_Q_ADDR + 0x24

#define FIFO_FW_DATA_ADDR       HI_USB_RX_Q_ADDR
#define FIFO_FW_DATA_LEN        (USB_RX_Q_LEN * 4)
#define FIFO_FW_DATA_R          HI_USB_EVENT_Q_ADDR + 0x0c
#define FIFO_FW_DATA_W          HI_USB_EVENT_Q_ADDR + 0x10


#define FIFO_FW_CMD_ADDR        HI_USB_CMD_Q_ADDR
#define FIFO_FW_CMD_LEN         (4096)
#define FIFO_FW_CMD_R           HI_USB_MEM_ADDR
#define FIFO_FW_CMD_W           HI_USB_MEM_ADDR + 0x04

#define TX_Q_ADDR               HI_USB_TX_Q_ADDR
#define TX_Q_PRIO_ADDR          HI_USB_MEM_ADDR + 0x18
#define TX_Q_MAX_PRIO           0xFFFFFFFF

#define HI_USB_15P4_Q_ADDR      0x00700000  //length:2048 bytes

#define FIFO_FW_15P4_RX_ADDR    HI_USB_15P4_Q_ADDR + 0x2000
#define FIFO_FW_15P4_RX_LEN     0x800
#define FIFO_FW_15P4_RX_R       HI_USB_EVENT_Q_ADDR + 0x08
#define FIFO_FW_15P4_RX_W       HI_USB_EVENT_Q_ADDR + 0x1c

#define FIFO_FW_15P4_TX_ADDR    HI_USB_15P4_Q_ADDR + 0x3000
#define FIFO_FW_15P4_TX_LEN     0x800
#define FIFO_FW_15P4_TX_R       HI_USB_EVENT_Q_ADDR + 0x28
#define FIFO_FW_15P4_TX_W       HI_USB_EVENT_Q_ADDR + 0x2c

#define DRIVER_FW_STATUS        0x005101fc
#define SRAM_FD_INIT_FLAG       (1 << 1)
#define FIFO_FW_MANFDATA_ADDR   (HI_USB_EVENT_Q_ADDR + POLL_TOTAL_LEN + RC_MANFDATA_LEN + 4) //addr 0x514970 usb bulk 16 bit alignment
#define FIFO_FW_MAC_ADDR        (HI_USB_EVENT_Q_ADDR + POLL_TOTAL_LEN + 2*RC_MANFDATA_LEN + 4) //addr 0x5149a0 usb bulk 16 bit alignment

enum bt_polling_interval{
    POLLING_LEVEL_1 = 1000000,
    POLLING_LEVEL_2 = 5000000,
    POLLING_LEVEL_3 = 8000000,
};

enum bt_rx_state{
    HCI_RX_TYPE,
    HCI_RX_HEADER,
    HCI_RX_PAYLOAD,
    HCI_RX_FATAL,
};

#define AML_BT_CHAR_DEVICE_NAME     "aml_btusb"
#define AML_ZIGBEE_NOTE                 "aml_zigbee"
#define AML_THREAD_NOTE                 "aml_thread"
#define AML_COEX_NOTE                   "aml_coex"

#define AML_BT_FIRMWARE_NAME        "w2l_bt_15p4_fw_usb.bin"
#define AML_BT_FIRMWARE_TXT_NAME    "w2l_bt_15p4_fw_usb.txt"
#define AML_BT_FIRMWARE_FT_NAME     "w2l_bt_15p4_fw_usb_test.bin"

#define AML_BT_CONFIG_NAME          "aml_bt.conf"

#define ICCM_SIZE   0x38000
#define DCCM_SIZE   0x20000
#define ICCM_ROM_SIZE 384*1024
#define DOWNLOAD_SIZE 4096
#define ICCM_RAM_BASE           (0x000000)
#define DCCM_RAM_BASE           (0xd00000)
#define BT_ICCM_AHB_BASE        0x00300000
#define BT_DCCM_AHB_BASE        0x00400000

#ifndef BIT
#define BIT(_n)  (1 << (_n))
#endif

#define BT_DRV_STATE_SUSPEND_ENTRY     BIT(0)
#define BT_DRV_STATE_SUSPEND           BIT(1)
#define BT_DRV_STATE_RESUME            BIT(2)
#define BT_DRV_STATE_RECOVERY          BIT(3)
//#define BT_DRV_STATE_SEND              BIT(4)
#define BT_DRV_STATE_WAIT_RECOVERY     BIT(5)

enum bt_drv_state
{
    BT_DRV_NONE,
    BT_DRV_CLOSED,
    BT_DRV_SUSPEND_ENTRY,
    BT_DRV_SUSPEND,
    BT_DRV_RESUME_ENTRY,
    BT_DRV_RESUME,
    BT_DRV_WAIT_RECOVERY
};

#define REG_DEV_RESET           0xf03058
#define REG_FW_MODE             0xf000e0
#define REG_PMU_POWER_CFG       0xf03040
#define REG_RAM_PD_SHUTDWONW_SW 0xf03050
#define REG_FW_PC               0x200034

#define BIT_PHY                 1
#define BIT_MAC                 (1 << 1)
#define BIT_CPU                 (1 << 2)
#define BIT_RF_NUM              28
#define BT_SINK_MODE            25

#define CHIP_BT_PMU_REG_BASE               (0xf03000)
#define RG_BT_PMU_A11                             (CHIP_BT_PMU_REG_BASE + 0x2c)
#define RG_BT_PMU_A12                             (CHIP_BT_PMU_REG_BASE + 0x30)
#define RG_BT_PMU_A13                             (CHIP_BT_PMU_REG_BASE + 0x34)
#define RG_BT_PMU_A14                             (CHIP_BT_PMU_REG_BASE + 0x38)
#define RG_BT_PMU_A15                             (CHIP_BT_PMU_REG_BASE + 0x3c)
#define RG_BT_PMU_A16                             (CHIP_BT_PMU_REG_BASE + 0x40)
#define RG_BT_PMU_A17                             (CHIP_BT_PMU_REG_BASE + 0x44)
#define RG_BT_PMU_A18                             (CHIP_BT_PMU_REG_BASE + 0x48)
#define RG_BT_PMU_A20                             (CHIP_BT_PMU_REG_BASE + 0x50)
#define RG_BT_PMU_A22                             (CHIP_BT_PMU_REG_BASE + 0x58)

#define CHIP_INTF_REG_BASE               (0xf00000)
#define RG_AON_A15                                (CHIP_INTF_REG_BASE + 0x3c)
#define RG_AON_A16                                (CHIP_INTF_REG_BASE + 0x40)
#define RG_AON_A17                                (CHIP_INTF_REG_BASE + 0x44)
#define RG_AON_A24                                (CHIP_INTF_REG_BASE + 0x60)
#define RG_AON_A30                                (CHIP_INTF_REG_BASE + 0x78)
#define RG_AON_A52                                (CHIP_INTF_REG_BASE + 0xd0)
#define RG_AON_A53                                (CHIP_INTF_REG_BASE + 0xd4)
/*********amlbt_tool*********/
#define RG_AON_A55                                (CHIP_INTF_REG_BASE + 0xdc)
#define RG_AON_A56                                (CHIP_INTF_REG_BASE + 0xe0)
#define RG_AON_A57                                (CHIP_INTF_REG_BASE + 0xe4)
#define RG_AON_A58                                (CHIP_INTF_REG_BASE + 0xe8)
#define RG_AON_A59                                (CHIP_INTF_REG_BASE + 0xec)
#define RG_AON_A60                                (CHIP_INTF_REG_BASE + 0xf0)
#define RG_AON_A61                                (CHIP_INTF_REG_BASE + 0xf4)
#define RG_AON_A62                                (CHIP_INTF_REG_BASE + 0xf8)

// pmu status
#define PMU_PWR_OFF       0x0
#define PMU_PWR_XOSC      0x1
#define PMU_XOSC_WAIT     0x2
#define PMU_XOSC_DPLL     0x3
#define PMU_DPLL_WAIT     0x4
#define PMU_DPLL_ACT      0x5
#define PMU_ACT_MODE      0x6
#define PMU_ACT_SLEEP     0x7
#define PMU_SLEEP_MODE    0x8
#define PMU_SLEEP_WAKE    0x9
#define PMU_WAKE_WAIT     0xa
#define PMU_WAKE_XOSC     0xb

enum wifi_cmd {
    CMD_DOWNLOAD_WIFI = 0xC1,
    CMD_START_WIFI,
    CMD_STOP_WIFI,
    CMD_READ_REG,
    CMD_WRITE_REG,
    CMD_READ_PACKET,
    CMD_WRITE_PACKET,
    CMD_WRITE_SRAM,
    CMD_READ_SRAM,
    CMD_DOWNLOAD_BT,
    CMD_GET_TX_CFM,
    CMD_OTHER_CMD,
    CMD_USB_IRQ
};

#define AML_SIG_CBW             0x43425355
#define AML_XFER_TO_DEVICE      0
#define AML_XFER_TO_HOST        0x80
#define AML_USB_CONTROL_MSG_TIMEOUT 3000
//wait usb recovery time 10s
#define MAX_TIMEOUT             10000000000

#define WRITE_SRAM_DATA_LEN 477

//static struct mutex bt_usb_mutex;
extern struct mutex auc_usb_mutex;

#ifdef CONFIG_AMLOGIC_GX_SUSPEND
extern unsigned int get_resume_method(void);
#endif

//wake source
#define REMOTE_WAKEUP           2
#define BT_WAKEUP               4
#define REMOTE_CUS_WAKEUP       9

#define USB_BEGIN_LOCK() do {\
    mutex_lock(&auc_usb_mutex);\
} while (0)

#define USB_END_LOCK() do {\
    mutex_unlock(&auc_usb_mutex);\
} while (0)


struct crg_msc_cbw {
    unsigned int sig;
    unsigned int tag;
    unsigned int data_len;
    unsigned char flag;
    unsigned char lun;
    unsigned char len;
    unsigned int cdb[4];
    unsigned char resv[481];
}__attribute__ ((packed));

extern struct usb_device *g_udev;
static struct crg_msc_cbw *g_cmd_buf;

extern struct aml_bus_state_detect bus_state_detect;
extern struct aml_pm_type g_wifi_pm;
extern unsigned char g_chip_function_ctrl;

//extern unsigned char aml_wifi_detect_bt_status __attribute__((weak));

static w2l_usb_bt_new_t amlbt_dev = {0};
//static void amlbt_shutdown_func(void);

static int amlbt_probe(struct platform_device *dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int amlbt_remove(struct platform_device *dev);
#else
static void amlbt_remove(struct platform_device *dev);
#endif
static int amlbt_suspend(struct platform_device *dev, pm_message_t state);
static int amlbt_resume(struct platform_device *dev);
static void amlbt_shutdown(struct platform_device *dev);

static int amlbt_open(struct inode *inode, struct file *file);
static int amlbt_close(struct inode *inode, struct file *file);
static ssize_t amlbt_write(struct file *file_p,
                                     const char __user *buf_p,
                                     size_t count,
                                     loff_t *pos_p);
static ssize_t amlbt_read(struct file *file_p,
                                   char __user *buf_p,
                                   size_t count,
                                   loff_t *pos_p);
static unsigned int amlbt_poll(struct file *file, poll_table *wait);
//static int amlbt_submit_poll_urb(w2l_usb_bt_new_t *p_bt);

static void amlbt_lateresume(struct early_suspend *h);
static void amlbt_earlysuspend(struct early_suspend *h);
static int amlbt_download_firmware(w2l_usb_bt_new_t *p_bt);
static int amlbt_task_start(w2l_usb_bt_new_t *p_bt);
static int amlbt_load_firmware(w2l_usb_bt_new_t *p_bt);
static int amlbt_load_conf(w2l_usb_bt_new_t *p_bt);
static int amlbt_res_init(w2l_usb_bt_new_t *p_bt);
static void amlbt_res_deinit(w2l_usb_bt_new_t *p_bt);
static int amlbt_sw_reset(void);
static unsigned int amlbt_w2lu_coex_is_running(w2l_usb_bt_new_t *p_bt);

static int amlbt_coex_open(struct inode *inode, struct file *file);
static int amlbt_coex_close(struct inode *inode, struct file *file);
static int amlbt_zigbee_open(struct inode *inode, struct file *file);
static int amlbt_zigbee_close(struct inode *inode, struct file *file);
static ssize_t amlbt_zigbee_write(struct file *file_p, const char __user *buf_p, size_t count, loff_t *pos_p);
static ssize_t amlbt_zigbee_read(struct file *file_p, char __user *buf_p, size_t count, loff_t *pos_p);
static unsigned int amlbt_zigbee_poll(struct file *file, poll_table *wait);

static int amlbt_thread_open(struct inode *inode, struct file *file);
static int amlbt_thread_close(struct inode *inode, struct file *file);
static ssize_t amlbt_thread_write(struct file *file_p, const char __user *buf_p, size_t count, loff_t *pos_p);
static ssize_t amlbt_thread_read(struct file *file_p, char __user *buf_p, size_t count, loff_t *pos_p);
static unsigned int amlbt_thread_poll(struct file *file, poll_table *wait);

static void amlbt_write_work(struct work_struct *work);


static long amlbt_ioctl(struct file* filp, unsigned int cmd, unsigned long arg);
#ifdef CONFIG_COMPAT
static long amlbt_compat_ioctl(struct file* filp, unsigned int cmd, unsigned long arg);
#endif
static void amlbt_usb_fw_rx_work(struct work_struct *work);
static enum hrtimer_restart amlbt_hrtime(struct hrtimer *timer);

static void amlbt_release(struct device *dev)
{
    return;
}

static struct platform_device amlbt_device =
{
    .name    = "sdio_bt",
    .id      = -1,
    .dev     = {
        .release = &amlbt_release,
    }
};

static struct platform_driver amlbt_driver =
{
    .probe = amlbt_probe,
    .remove = amlbt_remove,
    .suspend = amlbt_suspend,
    .resume = amlbt_resume,
    .shutdown = amlbt_shutdown,

    .driver = {
        .name = "sdio_bt",
        .owner = THIS_MODULE,
    },
};

static const struct file_operations amlbt_fops =
{
    .open       = amlbt_open,
    .release    = amlbt_close,
    .write      = amlbt_write,
    .read      = amlbt_read,
    .unlocked_ioctl = amlbt_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = amlbt_compat_ioctl,
#endif
    .poll       = amlbt_poll,
    .fasync     = NULL
};

static const struct file_operations amlbt_zigbee_fops =
{
    .open       = amlbt_zigbee_open,
    .release    = amlbt_zigbee_close,
    .write      = amlbt_zigbee_write,
    .read      = amlbt_zigbee_read,
    .unlocked_ioctl = amlbt_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = amlbt_compat_ioctl,
#endif
    .poll       = amlbt_zigbee_poll,
    .fasync     = NULL
};

static const struct file_operations amlbt_thread_fops =
{
    .open       = amlbt_thread_open,
    .release    = amlbt_thread_close,
    .write      = amlbt_thread_write,
    .read      = amlbt_thread_read,
    .unlocked_ioctl = amlbt_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = amlbt_compat_ioctl,
#endif
    .poll       = amlbt_thread_poll,
    .fasync     = NULL
};

static const struct file_operations amlbt_coex_fops =
{
    .open       = amlbt_coex_open,
    .release    = amlbt_coex_close,
    .write      = NULL,
    .read      = NULL,
    .unlocked_ioctl = amlbt_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = amlbt_compat_ioctl,
#endif
    .poll       = NULL,
    .fasync     = NULL
};

static void auc_build_cbw(struct crg_msc_cbw *cbw_buf,
                               unsigned char dir,
                               unsigned int len,
                               unsigned char cdb1,
                               unsigned int cdb2,
                               unsigned long cdb3,
                               unsigned long cdb4)
{
    cbw_buf->sig = AML_SIG_CBW;
    cbw_buf->tag = 0x5da729a0;
    cbw_buf->data_len = len;
    cbw_buf->flag = dir; //direction
    cbw_buf->len = 16; //command length
    cbw_buf->lun = 0;

    cbw_buf->cdb[0] = cdb1;
    cbw_buf->cdb[1] = cdb2; // read or write addr
    cbw_buf->cdb[2] = (unsigned int)(unsigned long)cdb3;
    cbw_buf->cdb[3] = cdb4; //read or write data length
}

static void auc_build_cbw_add_data(struct crg_msc_cbw *cbw_buf,
                               unsigned char dir,
                               unsigned int len,
                               unsigned char cdb1,
                               unsigned int cdb2,
                               unsigned long cdb3,
                               SYS_TYPE cdb4,unsigned char *data)
{
    cbw_buf->sig = AML_SIG_CBW;
    cbw_buf->tag = 0x5da729a0;
    cbw_buf->data_len = len;
    cbw_buf->flag = dir; //direction
    cbw_buf->len = 16; //command length
    cbw_buf->lun = 0;

    cbw_buf->cdb[0] = cdb1;
    cbw_buf->cdb[1] = cdb2; // read or write addr
    cbw_buf->cdb[2] = (unsigned int)(unsigned long)cdb3;
    cbw_buf->cdb[3] = cdb4; //read or write data length
    memcpy(cbw_buf->resv + 1, (unsigned char *) data, len);
    /*in case call cmd and data mode but fw call cmd+data stage*/
    cbw_buf->resv[479] = cbw_buf->resv[480] = 0xFF;
}

static void get_btwakeup_work(unsigned int key)
{
    switch (key)
    {
        case 0:
        {
            BTI("default value \n");
        }
        break;
        case 1:
        {
            input_event(amlbt_dev.amlbt_input_dev, EV_KEY, KEY_POWER, 1);
            input_sync(amlbt_dev.amlbt_input_dev);
            input_event(amlbt_dev.amlbt_input_dev, EV_KEY, KEY_POWER, 0);
            input_sync(amlbt_dev.amlbt_input_dev);
            amlbt_dev.input_key = 0;
            BTI("%s input power key\n", __func__);
        }
        break;
        case 2:
        {
            input_event(amlbt_dev.amlbt_input_dev, EV_KEY, KEY_NETFLIX, 1);
            input_sync(amlbt_dev.amlbt_input_dev);
            input_event(amlbt_dev.amlbt_input_dev, EV_KEY, KEY_NETFLIX, 0);
            input_sync(amlbt_dev.amlbt_input_dev);
            amlbt_dev.input_key = 0;
            BTI("%s input Netflix key\n", __func__);
        }
        break;
        default:
            BTF("No identification key\n");
        break;
    }
}

static void amlbt_wakeup_mutex(unsigned int flag)
{
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;
    if (flag)
    {
        // Wake up the system and prevent it from entering
        if (p_bt->amlbt_wakeup_source && (!p_bt->amlbt_wakeup_source->active))
        {
            __pm_stay_awake(p_bt->amlbt_wakeup_source);
        }
        else
        {
            BTF("amlbt_wakeup_source is not initialized or active already\n");
        }
    }
    else
    {
        if (p_bt->amlbt_wakeup_source && p_bt->amlbt_wakeup_source->active)
        {
            __pm_relax(p_bt->amlbt_wakeup_source);
        }
        else
        {
            BTF("amlbt_wakeup_source is not initialized or not active\n");
        }
    }
    p_bt->wake_mux = flag;

    BTI("system state updated: %d\n", flag);
}

static void amlbt_wakeup_lock(void)
{
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;
    if (!p_bt->wake_mux)
    {
        amlbt_wakeup_mutex(1);
    }
}

static void amlbt_wakeup_unlock(void)
{
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;
    if (p_bt->wake_mux)
    {
        amlbt_wakeup_mutex(0);
    }
}

static void amlbt_drv_state_set(unsigned int bit)
{
    unsigned int reg_value = 0;
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;

    reg_value = p_bt->dr_state;
    BTD("amlbt_drv_state_set %#x: %#x\n", reg_value, bit);
    reg_value |= bit;
    BTI("amlbt_drv_state_set end %#x: %#x", reg_value, bit);
    p_bt->dr_state = reg_value;
    if (bit == BT_DRV_STATE_RECOVERY)
    {
        if (!p_bt->recovery_value)
        {
            p_bt->recovery_value = BT_DRV_STATE_RECOVERY;
        }
    }
}

static void amlbt_drv_state_clr(unsigned int bit)
{
    unsigned int reg_value = 0;
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;

    reg_value = p_bt->dr_state;
    BTD("amlbt_drv_state_clr %#x: %#x\n", reg_value, bit);
    reg_value &= ~bit;
    BTD("amlbt_drv_state_clr end %#x: %#x", reg_value, bit);
    p_bt->dr_state = reg_value;
}

static int amlbt_check_usb(void)
{
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;

    if (g_udev == NULL)
    {
        BTE("interface NULL");
        return -1;
    }

    if (bus_state_detect.bus_err || bus_state_detect.bus_reset_ongoing || bus_state_detect.is_recy_ongoing || (enum usb_udev_state)g_udev->state != USB_CONFIGURED)
    {
        if (!(p_bt->dr_state & BT_DRV_STATE_WAIT_RECOVERY))
        {
            p_bt->wait_start = sched_clock();
            amlbt_drv_state_set(BT_DRV_STATE_WAIT_RECOVERY);
        }
        BTE("bus_err %d reset %d recy %d udev %d", bus_state_detect.bus_err, bus_state_detect.bus_reset_ongoing,
                    bus_state_detect.is_recy_ongoing, g_udev->state);
        return -1;
    }
    return 0;
}

static int auc_write_reg_by_ep(unsigned int addr, unsigned int value, unsigned int len, unsigned int ep)
{
    int ret = 0;
    int actual_length = 0;

    USB_BEGIN_LOCK();
    memset(g_cmd_buf, 0, sizeof(*g_cmd_buf));
    auc_build_cbw(g_cmd_buf, AML_XFER_TO_DEVICE, 0, CMD_WRITE_REG, addr, value, len);
    /* cmd stage */
    ret = usb_bulk_msg(g_udev, (unsigned int)usb_sndbulkpipe(g_udev, ep), (void *)g_cmd_buf, sizeof(*g_cmd_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        BTE("auc_write_reg_by_ep Failed to usb_bulk_msg, ret %d, ep: %d, addr: 0x%x, len: %d, value: 0x%x\n", ret, ep, addr, len, value);
        USB_END_LOCK();
        return ret;
    }
    USB_END_LOCK();

    return 0; //bt write maybe use the value
}

static int amlbt_write_word(unsigned int addr,unsigned int data, unsigned int ep)
{
    int len = 4;
    int ret = 0;

    if (amlbt_dev.dr_state & BT_DRV_STATE_SUSPEND)
    {
        BTE("%d:suspend state %d\n", __LINE__, amlbt_dev.dr_state);
        return -2;
    }

    ret = amlbt_check_usb();

    if (ret != 0)
    {
        BTE("%s:%d, error!\n", __func__, __LINE__);
        return -1;
    }


    switch (ep) {
        case USB_EP2:
            ret = auc_write_reg_by_ep(addr, data, len, ep);
            if (ret != 0)
            {
                return ret;
            }
            break;
        default:
            BTE("EP-%d unsupported!\n", ep);
            break;
    }
    return 0;
}

static int auc_read_reg_by_ep(unsigned int addr, unsigned int len, unsigned int ep, unsigned int *value)
{
    int ret = 0;
    int actual_length = 0;
    unsigned char *data = NULL;

    USB_BEGIN_LOCK();

    data = (unsigned char *)kzalloc(len, GFP_KERNEL);

    if (!data) {
        BTE("auc_read_reg_by_ep data malloc fail, ep: %d, addr: 0x%x, len: %d\n", ep, addr, len);
        goto err_unlock;
    }
    memset(g_cmd_buf, 0, sizeof(*g_cmd_buf));
    auc_build_cbw(g_cmd_buf, AML_XFER_TO_HOST, len, CMD_READ_REG, addr, 0, len);

    /* cmd stage */
    ret = usb_bulk_msg(g_udev, usb_sndbulkpipe(g_udev, ep), (void *)g_cmd_buf, sizeof(*g_cmd_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        BTE("auc_read_reg_by_ep cmd Failed to usb_bulk_msg, ret %d, ep: %d, addr: 0x%x, len: %d\n", ret, ep, addr, len);
        goto err_kfree;
    }

    /* data stage */
    ret = usb_bulk_msg(g_udev, usb_rcvbulkpipe(g_udev, ep), (void *)data, len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        BTE("auc_read_reg_by_ep data Failed to usb_bulk_msg, ret %d, ep: %d, addr: 0x%x, len: %d\n", ret ,ep, addr, len);
        goto err_kfree;
    }

    memcpy(value, data, actual_length);
    kfree(data);
    USB_END_LOCK();

    return ret;
err_kfree:
    kfree(data);
err_unlock:
    USB_END_LOCK();
    return ret;
}

static int amlbt_read_word(unsigned int addr, unsigned int ep, unsigned int *value)
{
    int len = 4;
    int ret = 0;

    if (amlbt_dev.dr_state & BT_DRV_STATE_SUSPEND)
    {
        BTE("%d:suspend state %d\n", __LINE__, amlbt_dev.dr_state);
        return -2;
    }

    ret = amlbt_check_usb();

    if (ret != 0)
    {
        BTE("%s:%d, error!\n", __func__, __LINE__);
        return -1;
    }

    switch (ep) {
        case USB_EP2:
            ret = auc_read_reg_by_ep(addr, len, ep, value);
            if (ret != 0)
            {
                return ret;
            }
            break;
        default:
            BTE("EP-%d unsupported!\n", ep);
            break;
    }
    return ret;
}

static int auc_write_sram_by_ep(unsigned char *pdata, unsigned int addr, unsigned int len, unsigned int ep)
{
    int ret = 0;
    int actual_length = 0;
    unsigned char *kmalloc_buf = NULL;

    USB_BEGIN_LOCK();

    //if (len < WRITE_SRAM_DATA_LEN)
    if (0)
    {
        memset(g_cmd_buf, 0, sizeof(*g_cmd_buf));
        auc_build_cbw_add_data(g_cmd_buf, AML_XFER_TO_DEVICE, len, CMD_WRITE_SRAM, addr, 0, len,pdata);
        /* cmd stage */
        ret = usb_bulk_msg(g_udev, usb_sndbulkpipe(g_udev, ep), (void *)g_cmd_buf, sizeof(*g_cmd_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
        if (ret) {
            BTE("auc_write_sram_by_ep 1 Failed to usb_bulk_msg, ret %d, ep: %d, addr: 0x%x, len: %d\n", ret, ep, addr, len);
            BTE("usb command transmit fail,g_cmd_buf->add is %d,len is %d\n", addr, len);
            goto err_unlock;
        }
        g_cmd_buf->resv[479] = g_cmd_buf->resv[480] = 0;
    }
    else
    {
        memset(g_cmd_buf, 0, sizeof(*g_cmd_buf));
        auc_build_cbw(g_cmd_buf, AML_XFER_TO_DEVICE, len, CMD_WRITE_SRAM, addr, 0, len);
        /* cmd stage */
        ret = usb_bulk_msg(g_udev, usb_sndbulkpipe(g_udev, ep), (void *)g_cmd_buf, sizeof(*g_cmd_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
        if (ret) {
            BTE("auc_write_sram_by_ep 2 Failed to usb_bulk_msg, ret %d, ep: %d, addr: 0x%x, len: %d\n", ret, ep, addr, len);
            BTE("usb command transmit fail,g_cmd_buf->add is %d,len is %d\n", addr, len);
            goto err_unlock;
        }

        kmalloc_buf = (unsigned char *)kzalloc(len,  GFP_KERNEL);
        if (kmalloc_buf == NULL)
        {
            BTE("kmalloc buf fail, ep: %d, addr: 0x%x, len: %d\n", ep, addr, len);
            goto err_unlock;
        }

        memcpy(kmalloc_buf, pdata, len);
        /* data stage */
        ret = usb_bulk_msg(g_udev, usb_sndbulkpipe(g_udev, ep), (void *)kmalloc_buf, len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
        if (ret) {
            BTE("auc_write_sram_by_ep data Failed to usb_bulk_msg, ret %d, ep: %d,  addr: 0x%x, len: %d\n", ret, ep, addr, len);
            goto err_kfree;
        }
        kfree(kmalloc_buf);
    }
    USB_END_LOCK();

    return ret;
err_kfree:
    kfree(kmalloc_buf);
err_unlock:
    USB_END_LOCK();
    return ret;
}

static int amlbt_write_sram(unsigned char *buf, unsigned char *sram_addr, unsigned int len, unsigned int ep)
{
    int ret = 0;

    if (amlbt_dev.dr_state & BT_DRV_STATE_SUSPEND)
    {
        BTE("%d:suspend state %d\n", __LINE__, amlbt_dev.dr_state);
        return -2;
    }

    if (len == 0)
    {
        BTE("EP-%d write len err!\n", ep);
        return -1;
    }

    ret = amlbt_check_usb();

    if (ret != 0)
    {
        BTE("%s:%d, error!\n", __func__, __LINE__);
        return -1;
    }

    switch (ep) {
        case USB_EP2:
            ret = auc_write_sram_by_ep(buf, (unsigned int)(unsigned long)sram_addr, len, ep);
            if (ret != 0)
            {
                return ret;
            }
            break;
        default:
            BTE("EP-%d unsupported!\n", ep);
            break;
    }
    return ret;
}

static int auc_read_sram_by_ep(unsigned char *pdata, unsigned int addr, unsigned int len, unsigned int ep)
{
    int ret = 0;
    int actual_length = 0;
    unsigned char *kmalloc_buf = NULL;

    USB_BEGIN_LOCK();
    memset(g_cmd_buf, 0, sizeof(*g_cmd_buf));
    auc_build_cbw(g_cmd_buf,  AML_XFER_TO_HOST, len, CMD_READ_SRAM, addr, 0, len);
    /* cmd stage */
    ret = usb_bulk_msg(g_udev, usb_sndbulkpipe(g_udev, ep), (void *)g_cmd_buf, sizeof(*g_cmd_buf), &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        BTE("auc_read_sram_by_ep cmd Failed to usb_bulk_msg, ret %d, ep: %d, addr: 0x%x, len: %d\n", ret, ep, addr, len);
        BTE("usb command transmit fail,g_cmd_buf->add is %d,len is %d\n",addr,len);
        goto err_unlock;
    }

    kmalloc_buf = (unsigned char *)kzalloc(len, GFP_KERNEL);
    if (kmalloc_buf == NULL)
    {
        BTE("kmalloc buf fail, ep: %d, len: %d\n", ep, len);
        goto err_unlock;
    }

    /* data stage */
    ret = usb_bulk_msg(g_udev, usb_rcvbulkpipe(g_udev, ep),(void *)kmalloc_buf, len, &actual_length, AML_USB_CONTROL_MSG_TIMEOUT);
    if (ret) {
        BTE("auc_read_sram_by_ep cmd Failed to usb_bulk_msg, ret %d, ep: %d, addr: 0x%x, len: %d\n", ret, ep, addr, len);
        goto err_kfree;
    }

    memcpy(pdata, kmalloc_buf, actual_length);
    kfree(kmalloc_buf);

    USB_END_LOCK();
    return ret;
err_kfree:
    kfree(kmalloc_buf);
err_unlock:
    USB_END_LOCK();
    return ret;
}

static int amlbt_read_sram(unsigned char *buf,unsigned char *sram_addr, unsigned int len, unsigned int ep)
{
    int ret = 0;

    if (amlbt_dev.dr_state & BT_DRV_STATE_SUSPEND)
    {
        BTE("%d:suspend state %d\n", __LINE__, amlbt_dev.dr_state);
        return -2;
    }

    if (len == 0)
    {
        BTE("EP-%d read len err!\n", ep);
        return -1;
    }

    ret = amlbt_check_usb();

    if (ret != 0)
    {
        BTE("%s:%d, error!\n", __func__, __LINE__);
        return -1;
    }

    switch (ep) {
        case USB_EP2:
            ret = auc_read_sram_by_ep(buf, (unsigned int)(unsigned long)sram_addr, len, ep);
            if (ret != 0)
            {
                return ret;
            }
            break;
        default:
            BTE("EP-%d unsupported!\n", ep);
            break;
    }
    return ret;
}

static int amlbt_aon_addr_bit_set(unsigned int addr, unsigned int bit)
{
    int ret = 0;
    unsigned int reg_value = 0;

    ret = amlbt_read_word(addr, USB_EP2, &reg_value);
    if (ret != 0)
    {
        goto err_exit;
    }
    BTI("%#x: %#x\n", addr, reg_value);
    reg_value |= BIT(bit);
    ret = amlbt_write_word(addr, reg_value, USB_EP2);
    if (ret != 0)
    {
        goto err_exit;
    }
    ret = amlbt_read_word(addr, USB_EP2, &reg_value);
    if (ret != 0)
    {
        goto err_exit;
    }
    BTI("%#x: %#x", addr, reg_value);
    return ret;
err_exit:
    return ret;
}

static int amlbt_aon_addr_bit_clr(unsigned int addr, unsigned int bit)
{
    unsigned int reg_value = 0;
    int ret = 0;

    ret = amlbt_read_word(addr, USB_EP2, &reg_value);
    if (ret != 0)
    {
       goto err_exit;
    }
    BTI("%#x: %#x\n", addr, reg_value);
    reg_value &= ~BIT(bit);
    ret = amlbt_write_word(addr, reg_value, USB_EP2);
    if (ret != 0)
    {
       goto err_exit;
    }
    ret = amlbt_read_word(addr, USB_EP2, &reg_value);
    if (ret != 0)
    {
       goto err_exit;
    }
    BTI("%#x: %#x", addr, reg_value);

    return 0;
err_exit:
    return ret;
}

static int amlbt_aon_addr_bit_get(unsigned int addr, unsigned int bit)
{
    unsigned int reg_value = 0;
    int bit_value = 0;
    int ret = -1;

    ret = amlbt_read_word(addr, USB_EP2, &reg_value);
    if (ret == -1)
    {
        return ret;
    }
    bit_value = (reg_value >> bit) & 0x1;
    BTI("get %#x bit%#d: %#x\n", addr, bit, bit_value);

    return bit_value;
}

static unsigned int amlbt_fw_pmu_sleep_get(void)
{
    unsigned int reg_value = 0;

    amlbt_read_word(RG_BT_PMU_A15, USB_EP2, &reg_value);
    BTI("%s PMU FSM %#x\n", __func__, (reg_value & 0xF));

    if (((reg_value & 0xF) == PMU_SLEEP_MODE) || ((reg_value & 0xF) == PMU_ACT_SLEEP))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

static int amlbt_wake_fw(void)
{
    unsigned int reg_value = 0;
    int ret = 0;

    ret = amlbt_read_word(RG_BT_PMU_A16, USB_EP2, &reg_value);
    if (ret != 0)
    {
        goto err_exit;
    }
    reg_value &= ~BIT(0);
    reg_value |= BIT(1);
    ret = amlbt_write_word(RG_BT_PMU_A16, reg_value, USB_EP2);
    if (ret != 0)
    {
        goto err_exit;
    }
    ret = amlbt_read_word(RG_BT_PMU_A16, USB_EP2, &reg_value);
    if (ret != 0)
    {
        goto err_exit;
    }

    BTI("%s RG_BT_PMU_A16 %#x\n", __func__, reg_value);
    return ret;
err_exit:
    return ret;
}

static int amlbt_powersave_clear(void)
{
    int wait_cnt = 0;
    int ret = 0;
    // set bt open flag
    ret = amlbt_aon_addr_bit_set(RG_AON_A24, 24);
    if (ret != 0)
    {
        goto exit;
    }
    // clear shutdown bit
    ret = amlbt_aon_addr_bit_clr(RG_AON_A24, 27);
    if (ret != 0)
    {
        goto exit;
    }
    // clear suspend bit
    ret = amlbt_aon_addr_bit_clr(RG_AON_A24, 26);
    if (ret != 0)
    {
        goto exit;
    }
    // wake bt fw
    if (amlbt_fw_pmu_sleep_get())
    {
        usleep_range(1000, 1000);
        ret = amlbt_wake_fw();
        if (ret != 0)
        {
            goto exit;
        }
    }
    // wait bt  wake done 1s
     do
     {
         ret = amlbt_aon_addr_bit_get(RG_AON_A17, 29);
         if (ret == -1)
         {
             goto exit;
         }
         usleep_range(20000, 20000);
         if (wait_cnt++ > 50)
             break;
     } while (ret);
    return 0;
exit:
    return ret;
}

static int amlbt_resume_fw(w2l_usb_bt_new_t *p_bt)
{
    int wait_cnt = 0;
    int retry_cnt = 0;
    int ret = -1;
    //wait usb bus ready
    //if ((atomic_read(&g_wifi_pm.bus_suspend_cnt) == 0 && (enum usb_udev_state)g_udev->state == USB_CONFIGURED)
    //                            || bus_state_detect.is_recy_ongoing)
    {
        //BTI("g_wifi_pm.bus_suspend_cnt 0\n");
        //forbid fw sleep
        ret = amlbt_aon_addr_bit_set(RG_AON_A24, 25);
        if (ret != 0)
        {
            goto error;
        }
        usleep_range(1000, 1000);
        // wake bt fw
wake_retry:
        if (amlbt_fw_pmu_sleep_get() == TRUE)
        {
            usleep_range(1000, 1000);
            ret = amlbt_wake_fw();
            if (ret != 0)
            {
                goto error;
            }
        }
        wait_cnt = 0;
        //fw will clear bit after wake done
        do
        {
            ret = amlbt_aon_addr_bit_get(RG_AON_A17, 29);
            if (ret == -1)
            {
                goto error;
            }
            usleep_range(10000, 10000);
            if (wait_cnt++ > 5)//wait 50ms
            {
                BTE("%s wake fw failed\n", __func__);
                if (retry_cnt++ < 3)
                    goto wake_retry;
                break;
            }
        } while (ret);
        amlbt_clear_rclist_from_firmware();
    }
    return ret;
error:
    return ret;
}

static int amlbt_suspend_fw(w2l_usb_bt_new_t *p_bt)
{
    int ret = 0;

    hrtimer_cancel(&p_bt->poll_timer);

    ret = amlbt_write_rclist_to_firmware();
    if (ret != 0)
    {
        goto err_exit;
    }
    //set suspend bit
    ret = amlbt_aon_addr_bit_set(RG_AON_A24, 26);
    if (ret != 0)
    {
        goto err_exit;
    }
    //allow fw sleep
    ret = amlbt_aon_addr_bit_clr(RG_AON_A24, 25);
    if (ret != 0)
    {
        goto err_exit;
    }
    return ret;
 err_exit:
    return ret;
}
#if 0

static int amlbt_write_manfdata_to_firmware(w2l_usb_bt_new_t *p_bt)
{
    int ret = 0;
    int i = 0;
    printk(KERN_CONT "manfdata:[ ");
    for (; i < amlbt_dev.manfdata_len; i++)
    {
        printk(KERN_CONT "%#x ", p_bt->rc_manfdata[i]);
    }
    printk(KERN_CONT "]");
    ret = amlbt_write_sram(p_bt->rc_manfdata, (unsigned char *)FIFO_FW_MANFDATA_ADDR, RC_MANFDATA_LEN, USB_EP2);
    BTI("rc manfdata end");
    return ret;
}

static int amlbt_write_macaddr_to_firmware(w2l_usb_bt_new_t *p_bt)
{
    int ret = 0;
    ret = amlbt_write_sram(p_bt->mac_addr, (unsigned char *)FIFO_FW_MAC_ADDR, 6, USB_EP2);
    BTI("macaddr:[%#x,%#x,%#x,%#x,%#x,%#x]\n", p_bt->mac_addr[0], p_bt->mac_addr[1], p_bt->mac_addr[2],
                                                p_bt->mac_addr[3], p_bt->mac_addr[4], p_bt->mac_addr[5]);
    BTI("mac addr end");
    return ret;
}

static int amlbt_recy_shutdown_download(void)
{
    int ret = 0;
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;
    //stop cpu
    ret = amlbt_sw_reset();
    if (ret != 0)
    {
        goto exit;
    }
    //init fifo
    if (amlbt_res_init(p_bt) != 0)
    {
        BTI("amlbt_res_init failed!\n");
        goto err_buf;
    }
    //read config
    amlbt_load_conf(p_bt);
    //download fw
    ret = amlbt_load_firmware(p_bt);
    if (ret != 0)
    {
        BTI("amlbt_load_firmware failed!\n");
        goto err_buf;
    }
    //set mac addr
    ret = amlbt_write_macaddr_to_firmware(p_bt);
    if (ret != 0)
    {
        goto err_buf;
    }
    //set rc list
    ret = amlbt_write_rclist_to_firmware();
    if (ret != 0)
    {
        goto err_buf;
    }
    ret = amlbt_write_word(REG_DEV_RESET, 0, USB_EP2);
    if (ret != 0)
    {
        goto err_buf;
    }
    amlbt_res_deinit(p_bt);
    BTI("%s end\n", __func__);
    return 0;
err_buf:
    amlbt_res_deinit(p_bt);
exit:
    return ret;
}
#endif
#if 0   //wifi already write A16 28bit when shutdown
static void amlbt_shutdown_func(void)
{
    int ret = 0;
    unsigned int retry = 2;
    amlbt_drv_state_clr(BT_DRV_STATE_SUSPEND);
    BTI("driver state %d amlbt_dev.firmware %d", amlbt_dev.dr_state, amlbt_dev.bt_start);
    while (retry)
    {
        ret = amlbt_recy_shutdown_download();
        if (ret == 0)
        {
            ret = amlbt_aon_addr_bit_set(RG_AON_A24, 27);
            if (ret == 0)
            {
                break;
            }
        }
        retry--;
    }
    //ret = amlbt_aon_addr_bit_set(RG_AON_A55, 27);
    ret = amlbt_aon_addr_bit_set(RG_AON_A16, 28);
    BTI("%s end\n", __func__);
}
#endif
#if 0
static int gdsl_write_data_by_ep(gdsl_fifo_t *p_fifo, unsigned char *data, unsigned int len, unsigned int ep)
{
    unsigned int index = 0;
    unsigned char *w = p_fifo->w;
    unsigned int i = 0;
    int ret = 0;

    BTA("%s len:%d\n", __func__, len);

    len = ((len + 3) & 0xFFFFFFFC);
    if (gdsl_fifo_remain(p_fifo) < len)
    {
        BTE("write data no space!!\n");
        return 0;
    }

    if (w == 0)
    {
        BTA("w ep %#x\n", (unsigned long)p_fifo->base_addr);
        ret = amlbt_write_sram(data, p_fifo->base_addr, len, ep);
        if (ret != 0)
        {
            BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
            return ret;
        }
        p_fifo->w = (unsigned char *)(unsigned long)(len % p_fifo->size);
        return len;
    }

    while (i < len)
    {
        w = (unsigned char *)(((unsigned long)w + 1) % p_fifo->size);
        i++;
        if (w == 0)
        {
            BTA("w ep2 %#x\n", (unsigned long)p_fifo->w);
            ret = amlbt_write_sram(data,
                (unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr), i, ep);
            if (ret != 0)
            {
                BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
                return ret;
            }
            p_fifo->w = 0;
            index = i;
        }
    }
    if (index < len)
    {
        BTA("w ep3 %#x\n", (unsigned long)p_fifo->w);
        ret = amlbt_write_sram(&data[index],
            (unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr), len - index, ep);
        if (ret != 0)
        {
            BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
            return ret;
        }
    }
    p_fifo->w = w;
    return len;
}
#else
static unsigned int gdsl_write_data_by_ep(gdsl_fifo_t *p_fifo, unsigned char *data, unsigned int len, unsigned int ep)
{
    int ret = 0;
    unsigned long offset = (unsigned long)p_fifo->w;
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;

    BTA("%s len:%d\n", __func__, len);

    len = ((len + 3) & 0xFFFFFFFC);
    if (gdsl_fifo_remain(p_fifo) < len)
    {
        BTE("write data no space!!\n");
        if (!(p_bt->dr_state & BT_DRV_STATE_WAIT_RECOVERY))
        {
            amlbt_drv_state_set(BT_DRV_STATE_RECOVERY);
        }
        return -1;
    }

    if (len < (p_fifo->size - offset))
    {
        ret = amlbt_write_sram(data, (unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr), len, ep);
        if (ret != 0)
        {
            BTE("%s:%d Failed : %d len %d\n", __func__, __LINE__, ret, len);
            return -1;
        }
        p_fifo->w = (unsigned char *)(((unsigned long)p_fifo->w + len) % p_fifo->size);
    }
    else
    {
        ret = amlbt_write_sram(data, (unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr),
            p_fifo->size - offset, ep);
        if (ret != 0)
        {
            BTE("%s:%d Failed : %d len %d\n", __func__, __LINE__, ret, len);
            return -1;
        }
        if ((len - (p_fifo->size - offset)) != 0)
        {
            ret = amlbt_write_sram(&data[p_fifo->size - offset], p_fifo->base_addr, (len - (p_fifo->size - offset)), ep);
            if (ret != 0)
            {
                BTE("%s:%d Failed : %d len %d\n", __func__, __LINE__, ret, len);
                return -1;
            }
        }
        p_fifo->w = (unsigned char *)((len - (p_fifo->size - offset)) % p_fifo->size);
    }

    return len;
}

#endif
static gdsl_fifo_t *amlbt_fifo_init(gdsl_fifo_t **p_fifo, unsigned int len, unsigned char *base_addr,
    unsigned int r_point_addr, unsigned int w_point_addr)
{
    int ret = 0;
    unsigned int w = 0;
    unsigned int r = 0;

    if (*p_fifo == NULL)
    {
        *p_fifo = gdsl_fifo_init(len, base_addr);
        if (*p_fifo != NULL)
        {
            amlbt_write_word(r_point_addr, (unsigned int)(unsigned long)(*p_fifo)->r, USB_EP2);
            amlbt_write_word(w_point_addr, (unsigned int)(unsigned long)(*p_fifo)->w, USB_EP2);
            ret = amlbt_read_word(r_point_addr, USB_EP2, &r);
            BTI("%s, %d, %#x r:%#lx", __func__, ret, r_point_addr, r);
            ret = amlbt_read_word(w_point_addr, USB_EP2, &w);
            BTI("%s, %d, %#x w:%#lx", __func__, ret, w_point_addr, w);
        }
    }
    return *p_fifo;
}

static void amlbt_fifo_deinit(gdsl_fifo_t **p_fifo, w2l_usb_bt_new_t *p_bt, unsigned int r_point_addr, unsigned int w_point_addr)
{
    if (!(p_bt->dr_state & BT_DRV_STATE_RECOVERY))
    {
        amlbt_write_word(r_point_addr, 0, USB_EP2);
        amlbt_write_word(w_point_addr, 0, USB_EP2);
    }
    if (*p_fifo != NULL)
    {
        gdsl_fifo_deinit(*p_fifo);
        *p_fifo = NULL;
    }
}

static void amlbt_res_deinit(w2l_usb_bt_new_t *p_bt)
{
    unsigned int st_reg = 0;
    BTI("%s \n", __func__);

    hrtimer_cancel(&p_bt->poll_timer);
    if (p_bt->check_fw_wq != NULL)
    {
        flush_workqueue(p_bt->check_fw_wq);
        destroy_workqueue(p_bt->check_fw_wq);
        p_bt->check_fw_wq = NULL;
    }

    if (p_bt->usb_rx_buf != NULL)
    {
        kfree(p_bt->usb_rx_buf);
        p_bt->usb_rx_buf = NULL;
    }
    if (!(p_bt->dr_state & BT_DRV_STATE_RECOVERY))
    {
        amlbt_read_word(DRIVER_FW_STATUS, USB_EP2, &st_reg);
        st_reg |= SRAM_FD_INIT_FLAG;
        amlbt_write_word(DRIVER_FW_STATUS, st_reg, USB_EP2);
    }
    amlbt_fifo_deinit(&p_bt->fw_type_fifo, p_bt, FIFO_FW_RX_TYPE_R, FIFO_FW_RX_TYPE_W);
    amlbt_fifo_deinit(&p_bt->fw_evt_fifo, p_bt, FIFO_FW_EVT_R, FIFO_FW_EVT_W);
    amlbt_fifo_deinit(&p_bt->fw_data_fifo, p_bt, FIFO_FW_DATA_R, FIFO_FW_DATA_W);
    amlbt_fifo_deinit(&p_bt->_15p4_tx_fifo, p_bt, FIFO_FW_15P4_TX_R, FIFO_FW_15P4_TX_W);
    amlbt_fifo_deinit(&p_bt->_15p4_rx_fifo, p_bt, FIFO_FW_15P4_RX_R, FIFO_FW_15P4_RX_W);

    amlbt_fifo_deinit(&p_bt->tx_cmd_fifo, p_bt, FIFO_FW_CMD_R, FIFO_FW_CMD_W);

    if (!(p_bt->dr_state & BT_DRV_STATE_RECOVERY))
    {
        st_reg &= ~(SRAM_FD_INIT_FLAG);
        amlbt_write_word(DRIVER_FW_STATUS, st_reg, USB_EP2);
    }
    if (!p_bt->shutdown_value)
    {
        g_bt_shutdown_func = NULL;
    }
/*
    if (p_bt->bt_urb != NULL)
    {
        usb_free_urb(p_bt->bt_urb);
        p_bt->bt_urb = NULL;
    }
*/
    skb_queue_purge(&p_bt->tx_queue);
    skb_queue_purge(&p_bt->bt_rx_queue);
    skb_queue_purge(&p_bt->zigbee_rx_queue);
    skb_queue_purge(&p_bt->thread_rx_queue);
    cancel_work_sync(&p_bt->write_work);
    mutex_destroy(&p_bt->bt_debug_mutex);
    p_bt->dr_state = 0;
    BTI("%s finished \n", __func__);
}

static int amlbt_res_init(w2l_usb_bt_new_t *p_bt)
{
    unsigned int i = 0;
    unsigned int st_reg = 0;
    unsigned int tx_info[USB_TX_Q_NUM * 4] = {0};

    BTI("%s \n", __func__);

    //mutex_init(&bt_usb_mutex);

    p_bt->antenna = 2;
    p_bt->fw_mode = 1;
    p_bt->bt_sink = 0;
    p_bt->pin_mux = 0;
    p_bt->br_digit_gain = 66;
    p_bt->edr_digit_gain = 98;
    p_bt->fw_log = 0;
    p_bt->driver_log = 3;
    p_bt->factory = 0;
    p_bt->dr_state = 0;
    p_bt->rd_state = 0;
    p_bt->zigbee_rd_state = 0;
    p_bt->thread_rd_state = 0;
    p_bt->sink_mode = 0;
    p_bt->recovery_value= 0;
    p_bt->input_key = 0;
    p_bt->shutdown_value = 0;
    p_bt->usb_irq_task_quit = 0;
    p_bt->wait_start = 0;

    amlbt_read_word(DRIVER_FW_STATUS, USB_EP2, &st_reg);
    st_reg |= SRAM_FD_INIT_FLAG;
    amlbt_write_word(DRIVER_FW_STATUS, st_reg, USB_EP2);

    p_bt->usb_rx_buf = kzalloc(POLL_TOTAL_LEN, GFP_DMA|GFP_ATOMIC);
    if (!p_bt->usb_rx_buf)
    {
        BTE("%s:%d usb_rx_buf failed!\n", __func__, __LINE__);
        goto error;
    }

    //fw type fifo init
    if (NULL == amlbt_fifo_init(&p_bt->fw_type_fifo, FIFO_FW_RX_TYPE_LEN, (unsigned char *)FIFO_FW_RX_TYPE_ADDR,
            FIFO_FW_RX_TYPE_R, FIFO_FW_RX_TYPE_W))
    {
        BTE("%s:%d fw type fifo init failed!\n", __func__, __LINE__);
        goto error;
    }
    //fw event fifo init
    if (NULL == amlbt_fifo_init(&p_bt->fw_evt_fifo, FIFO_FW_EVT_LEN, (unsigned char *)FIFO_FW_EVT_ADDR,
            FIFO_FW_EVT_R, FIFO_FW_EVT_W))
    {
        BTE("%s:%d fw event fifo init failed!\n", __func__, __LINE__);
        goto error;
    }
    //fw data fifo init
    if (NULL == amlbt_fifo_init(&p_bt->fw_data_fifo, FIFO_FW_DATA_LEN, (unsigned char *)FIFO_FW_DATA_ADDR,
            FIFO_FW_DATA_R, FIFO_FW_DATA_W))
    {
        BTE("%s:%d fw data fifo init failed!\n", __func__, __LINE__);
        goto error;
    }

    //tx hci cmd fifo init
    if (NULL == amlbt_fifo_init(&p_bt->tx_cmd_fifo, FIFO_FW_CMD_LEN, (unsigned char *)FIFO_FW_CMD_ADDR,
            FIFO_FW_CMD_R, FIFO_FW_CMD_W))
    {
        BTE("%s:%d tx hci cmd fifo init failed!\n", __func__, __LINE__);
        goto error;
    }

    for (i = 0; i < USB_TX_Q_NUM; i++)
    {
        p_bt->tx_q[i].tx_q_addr = (unsigned char *)(unsigned long)(HI_USB_TX_Q_ADDR + i * USB_TX_Q_LEN);

        p_bt->tx_q[i].tx_q_prio_addr = (unsigned int *)(unsigned long)(TX_Q_PRIO_ADDR + i * 16);
        p_bt->tx_q[i].tx_q_dev_index_addr = (unsigned int *)((unsigned long)p_bt->tx_q[i].tx_q_prio_addr + 4);
        p_bt->tx_q[i].tx_q_status_addr = (unsigned int *)((unsigned long)p_bt->tx_q[i].tx_q_dev_index_addr + 4);

        p_bt->tx_q[i].tx_q_dev_index = 0;
        p_bt->tx_q[i].tx_q_prio = TX_Q_MAX_PRIO;
        p_bt->tx_q[i].tx_q_status = GDSL_TX_Q_UNUSED;
        tx_info[i*4] = p_bt->tx_q[i].tx_q_prio;
        tx_info[i*4+1] = p_bt->tx_q[i].tx_q_dev_index;
        tx_info[i*4+2] = p_bt->tx_q[i].tx_q_status;
        BTP("tx_addr:%#x,%#x,%#x\n", (unsigned long)p_bt->tx_q[i].tx_q_prio_addr,
            (unsigned long)p_bt->tx_q[i].tx_q_dev_index_addr,
            (unsigned long)p_bt->tx_q[i].tx_q_status_addr);
    }
    amlbt_write_sram((unsigned char *)tx_info, (unsigned char *)TX_Q_PRIO_ADDR, sizeof(tx_info), USB_EP2);

    //15.4 fifo init
    if (NULL == amlbt_fifo_init(&p_bt->_15p4_tx_fifo, FIFO_FW_15P4_TX_LEN, (unsigned char *)FIFO_FW_15P4_TX_ADDR,
            FIFO_FW_15P4_TX_R, FIFO_FW_15P4_TX_W))
    {
        BTE("%s:%d fw 15.4 tx fifo init failed!\n", __func__, __LINE__);
        goto error;
    }
    BTI("15p4 init tx r %#x\n", (unsigned long)p_bt->_15p4_tx_fifo->r);
    BTI("15p4 init tx w %#x\n", (unsigned long)p_bt->_15p4_tx_fifo->w);
    if (NULL == amlbt_fifo_init(&p_bt->_15p4_rx_fifo, FIFO_FW_15P4_RX_LEN, (unsigned char *)FIFO_FW_15P4_RX_ADDR,
            FIFO_FW_15P4_RX_R, FIFO_FW_15P4_RX_W))
    {
        BTE("%s:%d fw 15.4 rx fifo init failed!\n", __func__, __LINE__);
        goto error;
    }

    st_reg &= ~(SRAM_FD_INIT_FLAG);
    amlbt_write_word(DRIVER_FW_STATUS, st_reg, USB_EP2);
/*
    p_bt->bt_urb = usb_alloc_urb(0, GFP_ATOMIC);
    if (p_bt->bt_urb == NULL)
    {
        BTE("%s:%d p_bt->bt_urb == NULL!!!\n", __func__, __LINE__);
        goto error;
    }
*/
    skb_queue_head_init(&p_bt->tx_queue);
    skb_queue_head_init(&p_bt->bt_rx_queue);
    skb_queue_head_init(&p_bt->zigbee_rx_queue);
    skb_queue_head_init(&p_bt->thread_rx_queue);
    init_waitqueue_head(&p_bt->zigbee_wait_queue);
    init_waitqueue_head(&p_bt->thread_wait_queue);
    INIT_WORK(&p_bt->write_work, amlbt_write_work);

    mutex_init(&p_bt->bt_debug_mutex);
    init_completion(&p_bt->comp);
    init_waitqueue_head(&p_bt->rd_wait_queue);

    p_bt->check_fw_wq = create_singlethread_workqueue("check_fw_wq");
    INIT_WORK(&p_bt->check_fw, amlbt_usb_fw_rx_work);
    hrtimer_init(&p_bt->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    p_bt->poll_timer.function = amlbt_hrtime;

    return 0;
error:
    amlbt_res_deinit(p_bt);

    return -1;
}

static int amlbt_create_device(w2l_usb_bt_new_t *p_bt)
{
    int ret = 0;
    int i = 0, j = 0;
    int cdevErr = 0;
    dev_t dev = 0;
    const char *device_names[AML_W2LU_MAX_COEX_DEVICES] =
        { AML_BT_CHAR_DEVICE_NAME, AML_ZIGBEE_NOTE, AML_THREAD_NOTE, AML_COEX_NOTE };
    BTI("%s \n", __func__);

    ret = alloc_chrdev_region(&dev, 0, AML_W2LU_MAX_COEX_DEVICES, AML_BT_CHAR_DEVICE_NAME);
    if (ret)
    {
        BTE("fail to allocate chrdev\n");
        return ret;
    }

    p_bt->dev_major = MAJOR(dev);
    BTI("major number:%d\n", p_bt->dev_major);

    i = 0;
    //bt node
    cdev_init(&p_bt->dev_cdev[i], &amlbt_fops);
    p_bt->dev_cdev[i].owner = THIS_MODULE;

    cdevErr = cdev_add(&p_bt->dev_cdev[i], MKDEV(p_bt->dev_major, i), 1);
    if (cdevErr)
    {
        goto error_cdev;
    }

    i++;
    //zigbee node
    cdev_init(&p_bt->dev_cdev[i], &amlbt_zigbee_fops);
    p_bt->dev_cdev[i].owner = THIS_MODULE;

    ret = cdev_add(&p_bt->dev_cdev[i], MKDEV(p_bt->dev_major, i), 1);
    if (ret)
    {
        BTE("cdev_add failed for minor %d\n", i);
        goto error_cdev;
    }

    i++;
    //thread node
    cdev_init(&p_bt->dev_cdev[i], &amlbt_thread_fops);
    p_bt->dev_cdev[i].owner = THIS_MODULE;

    ret = cdev_add(&p_bt->dev_cdev[i], MKDEV(p_bt->dev_major, i), 1);
    if (ret)
    {
        BTE("cdev_add failed for minor %d\n", i);
        goto error_cdev;
    }

    i++;
    //coex node
    cdev_init(&p_bt->dev_cdev[i], &amlbt_coex_fops);
    p_bt->dev_cdev[i].owner = THIS_MODULE;

    ret = cdev_add(&p_bt->dev_cdev[i], MKDEV(p_bt->dev_major, i), 1);
    if (ret)
    {
        BTE("cdev_add failed for minor %d\n", i);
        goto error_cdev;
    }

    BTI("driver(major %d) installed.\n", p_bt->dev_major);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    p_bt->dev_class = class_create(THIS_MODULE, AML_BT_CHAR_DEVICE_NAME);
#else
    p_bt->dev_class = class_create(AML_BT_CHAR_DEVICE_NAME);
#endif

    if (IS_ERR(p_bt->dev_class))
    {
        BTE("class create fail, error code(%ld)\n", PTR_ERR(p_bt->dev_class));
        goto error_class;
    }

    for (i = 0; i < AML_W2LU_MAX_COEX_DEVICES; i++)
    {
        p_bt->dev_device[i] = device_create(p_bt->dev_class, NULL, MKDEV(p_bt->dev_major, i), NULL, device_names[i]);
        if (IS_ERR(p_bt->dev_device[i]))
        {
            BTE("device create fail for %s, error code(%ld)\n", device_names[i], PTR_ERR(p_bt->dev_device[i]));
            goto error_device;
        }
    }

    BTI("%s: BT_major %d\n", __func__, p_bt->dev_major);
    BTI("%s: dev id %d\n", __func__, dev);

    return 0;

error_device:
    j = i;
    while (--j >= 0)
    {
        device_destroy(p_bt->dev_class, MKDEV(p_bt->dev_major, j));
    }
    class_destroy(p_bt->dev_class);

error_class:
    j = i;
error_cdev:
    while (--j >= 0)
    {
        cdev_del(&p_bt->dev_cdev[j]);
    }
    unregister_chrdev_region(dev, AML_W2LU_MAX_COEX_DEVICES);

    return -1;
}

static int amlbt_destroy_device(w2l_usb_bt_new_t *p_bt)
{
    dev_t dev;
    int i;

    BTI("%s: destroying devices\n", __func__);

    for (i = 0; i < AML_W2LU_MAX_COEX_DEVICES; i++)
    {
        dev = MKDEV(p_bt->dev_major, i);
        if (p_bt->dev_device[i])
        {
            device_destroy(p_bt->dev_class, dev);
            p_bt->dev_device[i] = NULL;
        }
    }

    if (p_bt->dev_class)
    {
        class_destroy(p_bt->dev_class);
        p_bt->dev_class = NULL;
    }

    for (i = 0; i < AML_W2LU_MAX_COEX_DEVICES; i++)
    {
        cdev_del(&p_bt->dev_cdev[i]);
    }

    unregister_chrdev_region(MKDEV(p_bt->dev_major, 0), AML_W2LU_MAX_COEX_DEVICES);

    BTI("%s driver removed.\n", AML_BT_CHAR_DEVICE_NAME);
    return 0;
}

static void amlbt_register_early_suspend(struct platform_device *dev)
{
    BTI("%s \n", __func__);

    amlbt_dev.early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
    amlbt_dev.early_suspend.suspend = amlbt_earlysuspend;
    amlbt_dev.early_suspend.resume = amlbt_lateresume;
    amlbt_dev.early_suspend.param = dev;
    register_early_suspend(&amlbt_dev.early_suspend);
}

static void amlbt_unregister_early_suspend(struct platform_device *dev)
{
    BTI("%s \n", __func__);

    unregister_early_suspend(&amlbt_dev.early_suspend);
}

static void amlbt_register_wakeupsource(w2l_usb_bt_new_t *p_bt)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
    amlbt_dev.amlbt_wakeup_source = wakeup_source_register("amlbt_wakeup_source");
#else
    amlbt_dev.amlbt_wakeup_source = wakeup_source_register(NULL, "amlbt_wakeup_source");
#endif

    if (!amlbt_dev.amlbt_wakeup_source)
    {
        BTE("Failed to create wakeup source\n");
        return ;
    }
}

static void amlbt_unregister_wakeupsource(w2l_usb_bt_new_t *p_bt)
{
    if (p_bt->amlbt_wakeup_source)
    {
        wakeup_source_unregister(p_bt->amlbt_wakeup_source);
        p_bt->amlbt_wakeup_source = NULL;
    }
    else
    {
        BTE("amlbt_wakeup_source is not initialized, unregistering is not required.\n");
    }
}

static int amlbt_input_device_init(struct platform_device *pdev)
{
    int err;
    amlbt_dev.amlbt_input_dev = input_allocate_device();
    if (!amlbt_dev.amlbt_input_dev)
    {
        BTF("[abner test]input_allocate_device failed:");
        return -EINVAL;
    }
    set_bit(EV_KEY,  amlbt_dev.amlbt_input_dev->evbit);
    set_bit(KEY_POWER, amlbt_dev.amlbt_input_dev->keybit);
    set_bit(KEY_NETFLIX, amlbt_dev.amlbt_input_dev->keybit);

    amlbt_dev.amlbt_input_dev->name = INPUT_NAME;
    amlbt_dev.amlbt_input_dev->phys = INPUT_PHYS;
    amlbt_dev.amlbt_input_dev->dev.parent = &pdev->dev;
    amlbt_dev.amlbt_input_dev->id.bustype = BUS_ISA;
    amlbt_dev.amlbt_input_dev->id.vendor = 0x0001;
    amlbt_dev.amlbt_input_dev->id.product = 0x0001;
    amlbt_dev.amlbt_input_dev->id.version = 0x0100;
    amlbt_dev.amlbt_input_dev->rep[REP_DELAY] = 0xffffffff;
    amlbt_dev.amlbt_input_dev->rep[REP_PERIOD] = 0xffffffff;
    amlbt_dev.amlbt_input_dev->keycodesize = sizeof(unsigned short);
    amlbt_dev.amlbt_input_dev->keycodemax = 0x1ff;
    err = input_register_device(amlbt_dev.amlbt_input_dev);
    if (err < 0)
    {
        pr_err("[abner test]input_register_device failed: %d\n", err);
        input_free_device(amlbt_dev.amlbt_input_dev);
        return -EINVAL;
    }

    return err;
}

static int amlbt_probe(struct platform_device *dev)
{
    BTI("%s \n", __func__);

    amlbt_create_device(&amlbt_dev);
    amlbt_debug_dev_init(amlbt_write_sram, amlbt_read_sram, amlbt_write_word, amlbt_read_word, &amlbt_dev);
    amlbt_rc_list_init(amlbt_dev.dev_device[0], amlbt_write_sram, amlbt_read_sram);
    amlbt_register_early_suspend(dev);
    amlbt_register_wakeupsource(&amlbt_dev);
    //amlbt_dev->wake_mux = 0;
    g_cmd_buf = kzalloc(sizeof(*g_cmd_buf), GFP_DMA | GFP_ATOMIC);
    if (!g_cmd_buf) {
        BTE("%s:%d g_cmd_buf kzalloc failed!\n", __func__, __LINE__);
        return -EINVAL;;
    }
    //input devices
    amlbt_input_device_init(dev);
    return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int amlbt_remove(struct platform_device *dev)
#else
static void amlbt_remove(struct platform_device *dev)
#endif
{
    BTI("%s \n", __func__);

    input_unregister_device(amlbt_dev.amlbt_input_dev);
    amlbt_dev.amlbt_input_dev = NULL;
    amlbt_unregister_wakeupsource(&amlbt_dev);
    amlbt_unregister_early_suspend(dev);
    amlbt_debug_dev_deinit();
    amlbt_rc_list_deinit(amlbt_dev.dev_device[0]);
    amlbt_destroy_device(&amlbt_dev);
    if (g_cmd_buf != NULL)
    {
        kfree(g_cmd_buf);
        g_cmd_buf = NULL;
    }
    if (g_bt_shutdown_func != NULL)
    {
        g_bt_shutdown_func = NULL;
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
    return 0;
#endif
}

static int amlbt_suspend(struct platform_device *dev, pm_message_t state)
{
    int wait_cnt = 0;
    int ret = 0;
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;
    p_bt->input_key = 0;

    BTI("%s %#x,%#x\n", __func__, p_bt->bt_start, p_bt->dr_state);

    if (p_bt->bt_start && !(p_bt->dr_state & BT_DRV_STATE_RECOVERY))
    {
#if 1
        while (!skb_queue_empty(&p_bt->tx_queue))
        {
            usleep_range(10000, 10000); //wait SENDING 1s
            wait_cnt++;
            if (wait_cnt > 100)
            {
                BTE("%s:%d suspend failed wait send!\n", __func__, __LINE__);
                return -EBUSY;
            }
        }
#endif
        hrtimer_cancel(&p_bt->poll_timer);
        if (p_bt->check_fw_wq != NULL)
        {
            flush_workqueue(p_bt->check_fw_wq);
        }
        amlbt_drv_state_set(BT_DRV_STATE_SUSPEND_ENTRY);
        ret = amlbt_suspend_fw(p_bt);
        if (ret != 0)
        {
            BTE("%s:%d entery lowpower failed \n", __func__, __LINE__);
            return -EBUSY;
        }
        amlbt_drv_state_set(BT_DRV_STATE_SUSPEND);
        amlbt_drv_state_clr(BT_DRV_STATE_SUSPEND_ENTRY);
    }
    BTI("%s end\n", __func__);
    return 0;
}

static int amlbt_resume(struct platform_device *dev)
{
    //unsigned long timeout = msecs_to_jiffies(2000);
    int ret = 0;
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;

    BTI("%s %#x,%#x\n", __func__, p_bt->bt_start, p_bt->dr_state);

    if (p_bt->bt_start && !(p_bt->dr_state & BT_DRV_STATE_RECOVERY))
    {
        amlbt_drv_state_set(BT_DRV_STATE_RESUME);
        amlbt_drv_state_clr(BT_DRV_STATE_SUSPEND);
#ifdef  CONFIG_AMLOGIC_GX_SUSPEND
            if (((get_resume_method() != REMOTE_WAKEUP) && (get_resume_method() != BT_WAKEUP))
                                && (get_resume_method() != REMOTE_CUS_WAKEUP))
            {
                p_bt->input_key = 1;
            }
#endif
        ret = amlbt_resume_fw(p_bt);
        if (ret == -1)
        {
            BTE("%s:%d resume fw failed! \n", __func__, __LINE__);
        }
        amlbt_drv_state_clr(BT_DRV_STATE_RESUME);
        p_bt->ktime = ktime_set(0, POLLING_LEVEL_3);
        p_bt->poll_timer.function = amlbt_hrtime;
        hrtimer_start(&p_bt->poll_timer, p_bt->ktime, HRTIMER_MODE_REL);
    }
    BTI("WAKE REASON %d\n", get_resume_method());
    BTI("%s end\n", __func__);
    return ret;
}

static void amlbt_shutdown(struct platform_device *dev)
{
    BTI("%s \n", __func__);
    amlbt_write_word(RG_BT_PMU_A16, 0, USB_EP2);
}

static void amlbt_earlysuspend(struct early_suspend *h)
{
    BTI("%s \n", __func__);
}

static void amlbt_lateresume(struct early_suspend *h)
{
    int wait_cnt = 0;
    unsigned int reg_value = 0;
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;
    p_bt->input_key = 0;

    BTI("%s %#x,%#x\n", __func__, p_bt->bt_start, p_bt->dr_state);

    if (p_bt->bt_start && !(p_bt->dr_state & BT_DRV_STATE_RECOVERY))
    {
        while (p_bt->dr_state & BT_DRV_RESUME)
        {
            usleep_range(10000, 10000); //wait resume 1s
            wait_cnt++;
            if (wait_cnt > 100)
            {
                BTW("%s:%d amlbt_lateresume timeout!\n", __func__, __LINE__);
                break;
            }
        }

        amlbt_read_word(RG_AON_A24, USB_EP2, &reg_value);
        BTI("%s RG_AON_A24:%#x\n", __func__, reg_value);
        reg_value &= ~(1 << 26);
        amlbt_write_word(RG_AON_A24, reg_value, USB_EP2);
        amlbt_read_word(RG_AON_A24, USB_EP2, &reg_value);
        BTI("RG_AON_A24:%#x\n", reg_value);
    }
    BTI("%s end\n", __func__);
}
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
static int amlbt_bind_bus(void)
{
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;

    if (g_udev == NULL)
    {
        BTE("g_udev is NULL");
        return -ENOMEM;
    }
    else
    {
        if (p_bt->link == NULL)
        {
            p_bt->link = device_link_add(&amlbt_device.dev, &g_udev->dev, DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
            if (p_bt->link == NULL)
            {
                BTE("Failed to create device link");
                return -ENOMEM;
            }
            else
            {
                BTI("Success to create device link");
            }
        }
    }
    return 0;
}

static struct device_link *find_device_link(struct device *consumer, struct device *supplier)
{
    struct device_link *link;

    list_for_each_entry(link, &consumer->links.suppliers, c_node)
    {
        if (link->supplier == supplier)
        {
            return link;
        }
    }

    return NULL;
}

static void amlbt_unbind_bus(void)
{
    struct device_link *link = NULL;

    w2l_usb_bt_new_t *p_bt = &amlbt_dev;

    if (p_bt->link != NULL)
    {
        if (g_udev && device_is_registered(&g_udev->dev))
        {
            link = find_device_link(&amlbt_device.dev, &g_udev->dev);
            BTI("find_device_link : %#x", (unsigned long)link);
        }
        if (link != NULL && link == p_bt->link)
        {
            device_link_del(p_bt->link);
            BTI("Success to del device link");
        }
        p_bt->link = NULL;
    }
}
#endif

static int amlbt_sw_reset(void)
{
    int ret = 0;
    ret = amlbt_write_word(REG_DEV_RESET, ((BIT_PHY|BIT_MAC|BIT_CPU)<<16)|(BIT_PHY|BIT_MAC|BIT_CPU), USB_EP2);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        goto error;
    }
    usleep_range(1000, 1000);
    ret = amlbt_write_word(REG_DEV_RESET, ((BIT_CPU)<<16)|(BIT_CPU), USB_EP2);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        goto error;
    }
    return ret;
error:
    return ret;
}

static int parse_int_value(char *start, const char *key, int *value, unsigned char *str)
{
    size_t key_len = strlen(key);
    size_t manflen = 0;
    int i = 0;
    int j = 0;
    char *ptr = NULL;
    char sub_str[3] = {0};
    if (strncmp(start, key, key_len) == 0 && start[key_len] == '=')
    {
        ptr = (start + key_len + 1);
        if (strcmp(key, "ManfData") == 0)
        {
            manflen = strlen(ptr);
            BTI("PTR %s len %d", ptr, manflen);
            for (; i < manflen; i+=2,j++)
            {
                sub_str[0] = ptr[i];
                sub_str[1] = ptr[i+1];
                str[j] = simple_strtoul(sub_str, NULL, 16);
                if (*(ptr+i+2) == ' ')
                {
                    i += 1;
                }
            }
            amlbt_dev.manfdata_len = j;
        }
        else
        {
            *value = simple_strtol(ptr, NULL, 10);
        }
        return 1;
    }
    return 0;
}

static int amlbt_load_conf(w2l_usb_bt_new_t *p_bt)
{
    int ret = 0;
    const struct firmware *fw_entry = NULL;
    char *data;
    size_t len, pos = 0;

    BTI("Firmware load:%s\n", AML_BT_CONFIG_NAME);
    ret = request_firmware(&fw_entry, AML_BT_CONFIG_NAME, p_bt->dev_device[0]);
    if (ret)
    {
        BTE("%s:%d Failed to load config file: %d\n", __func__, __LINE__, ret);
        return -EINVAL;
    }

    if (!fw_entry || !fw_entry->data)
    {
        BTE("Failed to load conf or data is empty\n");
        release_firmware(fw_entry);
        return -EINVAL;
    }

    data = (char *)fw_entry->data;
    len = fw_entry->size;

    // Manual parsing loop
    while (pos < len)
    {
        char *line_start = data + pos;
        char *line_end = strchr(line_start, '\n');  // Find end of line
        if (!line_end)
        {
            line_end = data + len;  // If no newline, this is the last line
        }

        *line_end = '\0';  // Null-terminate the current line

        // Parse known keys
        if (parse_int_value(line_start, "BtAntenna", &p_bt->antenna, NULL))
        {
            BTI("Parsed BtAntenna: %d\n", p_bt->antenna);
        }
        else if (parse_int_value(line_start, "FirmwareMode", &p_bt->fw_mode, NULL))
        {
            BTI("Parsed FirmwareMode: %d\n", p_bt->fw_mode);
        }
        else if (parse_int_value(line_start, "BtSink", &p_bt->bt_sink, NULL)) {
            BTI("Parsed BtSink: %d\n", p_bt->bt_sink);
        }
        else if (parse_int_value(line_start, "ChangePinMux", &p_bt->pin_mux, NULL))
        {
            BTI("Parsed ChangePinMux: %d\n", p_bt->pin_mux);
        }
        else if (parse_int_value(line_start, "BrDigitGain", &p_bt->br_digit_gain, NULL))
        {
            BTI("Parsed BrDigitGain: %d\n", p_bt->br_digit_gain);
        }
        else if (parse_int_value(line_start, "EdrDigitGain", &p_bt->edr_digit_gain, NULL))
        {
            BTI("Parsed EdrDigitGain: %d\n", p_bt->edr_digit_gain);
        }
        else if (parse_int_value(line_start, "Btfwlog", &p_bt->fw_log, NULL))
        {
            BTI("Parsed Btfwlog: %d\n", p_bt->fw_log);
        }
        else if (parse_int_value(line_start, "Btlog", &p_bt->driver_log, NULL))
        {
            BTI("Parsed Btlog: %d\n", p_bt->driver_log);
        }
        else if (parse_int_value(line_start, "Btfactory", &p_bt->factory, NULL))
        {
            BTI("Parsed Btfactory: %d\n", p_bt->factory);
        }
        else if (parse_int_value(line_start, "ManfData", NULL, p_bt->rc_manfdata))
        {
            BTI("Parsed ManfData len: %d\n", p_bt->manfdata_len);
        }

        // Move to the next line
        pos = (line_end - data) + 1;
    }

    release_firmware(fw_entry);
    return 0;
}

#if 0
static int amlbt_load_firmware(w2l_usb_bt_new_t *p_bt)
{
    int ret = 0;
    size_t i = 0;
    unsigned int reg = 0;
    const struct firmware *fw_entry = NULL;
    unsigned int iccm_size;
    unsigned int dccm_size;
    unsigned char *firmware_data;
    unsigned int byte = 0;
    unsigned int byte_count = 0;

    BTI("Firmware load:%s\n", AML_BT_FIRMWARE_TXT_NAME);
    ret = request_firmware(&fw_entry, AML_BT_FIRMWARE_TXT_NAME, p_bt->dev_device);
    if (ret)
    {
        BTE("%s:%d Failed to load firmware: %d\n", __func__, __LINE__, ret);
        return ret;
    }

    if (!fw_entry || !fw_entry->data)
    {
        BTE("Failed to load firmware or data is empty\n");
        release_firmware(fw_entry);
        return -EINVAL;
    }

    firmware_data = kmalloc(fw_entry->size / 3, GFP_KERNEL);

    if (!firmware_data) {
        BTE("Failed to allocate memory for firmware data\n");
        release_firmware(fw_entry);
        return -EINVAL;
    }

    while (i < fw_entry->size) {
        if (fw_entry->data[i] == ' ' || fw_entry->data[i] == '\n' || fw_entry->data[i] == '\r') {
            i++;
            continue;
        }

        if (sscanf(&fw_entry->data[i], "%2X", &byte) == 1) {
            firmware_data[byte_count++] = (unsigned char)byte;
            i += 2;
        } else {
            i++;
        }
    }
    BTI("Firmware [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
        firmware_data[0], firmware_data[1], firmware_data[2], firmware_data[3],
        firmware_data[4], firmware_data[5], firmware_data[6], firmware_data[7]);
    iccm_size = ((firmware_data[3]<<24)|(firmware_data[2]<<16)|(firmware_data[1]<<8)|(firmware_data[0]));
    dccm_size = ((firmware_data[7]<<24)|(firmware_data[6]<<16)|(firmware_data[5]<<8)|(firmware_data[4]));

    BTI("Firmware loaded successfully, iccm_size: %#x, dccm_size:%#x\n", iccm_size - ICCM_ROM_SIZE, dccm_size);

    p_bt->iccm_buf = &firmware_data[ICCM_ROM_SIZE + 8];
    p_bt->dccm_buf = &firmware_data[iccm_size + 8];

    amlbt_write_word(REG_RAM_PD_SHUTDWONW_SW, 0, USB_EP2);
    ret = amlbt_download_firmware(p_bt);
    release_firmware(fw_entry);
    if (ret != 0)
    {
        BTE("Download firmware failed!!\n");
        kfree(firmware_data);
        return ret;
    }
    amlbt_write_word(REG_FW_MODE, p_bt->fw_mode, USB_EP2);
    amlbt_write_word(REG_PMU_POWER_CFG, (p_bt->antenna << BIT_RF_NUM)|(p_bt->bt_sink << BT_SINK_MODE), USB_EP2);
    reg |= ((p_bt->pin_mux << 20) | (p_bt->factory << 21));
    reg |= (((p_bt->edr_digit_gain & 0xff) << 8) | (p_bt->br_digit_gain & 0xff));
    amlbt_write_word(RG_AON_A53, reg, USB_EP2);
    reg = (p_bt->fw_log & 0x3);
    amlbt_write_word(RG_AON_A59, reg, USB_EP2);
    amlbt_write_word(REG_DEV_RESET, 0, USB_EP2);
    p_bt->bt_start = 1;
    p_bt->iccm_buf = NULL;
    p_bt->dccm_buf = NULL;
    kfree(firmware_data);
    return 0;
}
#else
static int amlbt_load_firmware(w2l_usb_bt_new_t *p_bt)
{
    int ret = 0;
    unsigned int reg = 0;
    const struct firmware *fw_entry = NULL;
    unsigned int iccm_size;
    unsigned int dccm_size;

    if (!amlbt_ft_mode)
    {
        BTI("Firmware load:%s\n", AML_BT_FIRMWARE_NAME);
        ret = request_firmware(&fw_entry, AML_BT_FIRMWARE_NAME, p_bt->dev_device[0]);
    }
    else
    {
        BTI("Firmware load:%s\n", AML_BT_FIRMWARE_FT_NAME);
        ret = request_firmware(&fw_entry, AML_BT_FIRMWARE_FT_NAME, p_bt->dev_device[0]);
    }
    if (ret)
    {
        BTE("%s:%d Failed to load firmware: %d\n", __func__, __LINE__, ret);
        return ret;
    }
    BTI("Firmware [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
        fw_entry->data[0], fw_entry->data[1], fw_entry->data[2], fw_entry->data[3],
        fw_entry->data[4], fw_entry->data[5], fw_entry->data[6], fw_entry->data[7]);
    iccm_size = ((fw_entry->data[3]<<24)|(fw_entry->data[2]<<16)|(fw_entry->data[1]<<8)|(fw_entry->data[0]));
    dccm_size = ((fw_entry->data[7]<<24)|(fw_entry->data[6]<<16)|(fw_entry->data[5]<<8)|(fw_entry->data[4]));

    BTI("Firmware loaded successfully, iccm_size: %#x, dccm_size:%#x\n", iccm_size - ICCM_ROM_SIZE, dccm_size);

    p_bt->iccm_buf = &fw_entry->data[ICCM_ROM_SIZE + 8];
    p_bt->dccm_buf = &fw_entry->data[iccm_size + 8];

    amlbt_write_word(REG_RAM_PD_SHUTDWONW_SW, 0, USB_EP2);
    ret = amlbt_download_firmware(p_bt);
    release_firmware(fw_entry);
    if (ret != 0)
    {
        BTE("Download firmware failed!!\n");
        return ret;
    }
    amlbt_read_word(REG_FW_MODE, USB_EP2, &reg);
    reg |= (p_bt->fw_mode & 0x3);
    amlbt_write_word(REG_FW_MODE, reg, USB_EP2);

    amlbt_read_word(REG_PMU_POWER_CFG, USB_EP2, &reg);
    reg |= ((p_bt->antenna << BIT_RF_NUM)|(p_bt->bt_sink << BT_SINK_MODE));
    amlbt_write_word(REG_PMU_POWER_CFG, reg, USB_EP2);

    amlbt_read_word(RG_AON_A53, USB_EP2, &reg);
    reg |= ((p_bt->pin_mux << 20) | (p_bt->factory << 21));
    reg |= (((p_bt->edr_digit_gain & 0xff) << 8) | (p_bt->br_digit_gain & 0xff));
    amlbt_write_word(RG_AON_A53, reg, USB_EP2);

    amlbt_read_word(RG_AON_A59, USB_EP2, &reg);
    reg |= (p_bt->fw_log & 0x3);
    amlbt_write_word(RG_AON_A59, reg, USB_EP2);

    //amlbt_write_manfdata_to_firmware(p_bt);
    //p_bt->bt_start = 1;
    p_bt->iccm_buf = NULL;
    p_bt->dccm_buf = NULL;
    amlbt_wakeup_unlock();

    return 0;
}
#endif

static int amlbt_show_fw_debug_info(void)
{
    unsigned int value = 0;
    int ret = 0;

    ret = amlbt_read_word(REG_PMU_POWER_CFG, USB_EP2, &value);
    BTI("PMU 0x00f03040:%#x \n", value);
    if (ret != 0)
    {
        goto exit;
    }
    usleep_range(10000, 10000);
    ret = amlbt_read_word(REG_FW_PC, USB_EP2, &value);
    value = (value >> 6);
    BTI("pc1 0x200034:%#x\n", value);
    if (ret != 0)
    {
        goto exit;
    }
    usleep_range(10000, 10000);
    ret = amlbt_read_word(REG_FW_PC, USB_EP2, &value);
    value = (value >> 6);
    BTI("pc2 0x200034:%#x\n", value);
    if (ret != 0)
    {
        goto exit;
    }
    usleep_range(10000, 10000);
    ret = amlbt_read_word(REG_FW_PC, USB_EP2, &value);
    value = (value >> 6);
    BTI("pc3 0x200034:%#x\n", value);
    if (ret != 0)
    {
        goto exit;
    }
    return ret;
exit:
    return ret;
}

static int amlbt_open(struct inode *inode, struct file *file)
{
    int ret = 0;
    BTI("%s,%d, version:%s\n", __func__, amlbt_ft_mode, AML_W2LU_VERSION);

    file->private_data = &amlbt_dev;

    ret = amlbt_check_usb();
    if (ret != 0)
    {
        goto err_exit;
    }
    if (amlbt_ft_mode && amlbt_dev.bt_start)
    {
        file->private_data = &amlbt_dev;
        BTI("%s FT MODE", __func__);
        return nonseekable_open(inode, file);
    }
    else
    {
        if (!amlbt_w2lu_coex_is_running(&amlbt_dev))
        {
            ret = amlbt_powersave_clear();
            if (ret != 0)
            {
                goto err_exit;
            }

            if (amlbt_res_init(&amlbt_dev) != 0)
            {
                BTI("amlbt_res_init failed!\n");
                goto err_buf;
            }
            //stop bt cpu
            ret = amlbt_sw_reset();
            if (ret != 0)
            {
                goto err_exit;
            }

            //amlbt_utils_w2l_usb_init(amlbt_read_word, amlbt_write_word, amlbt_read_sram, amlbt_write_sram);
            amlbt_load_conf(&amlbt_dev);
            ret = amlbt_load_firmware(&amlbt_dev);
            if (ret != 0)
            {
                BTI("amlbt_load_firmware failed!\n");
                goto err_buf;
            }
            ret = amlbt_write_word(REG_DEV_RESET, 0, USB_EP2);
            if (ret != 0)
            {
                BTI("start cpu failed!\n");
                goto err_buf;
            }
            ret = amlbt_show_fw_debug_info();
            if (ret != 0)
            {
                BTI("show fw failed!\n");
                goto err_buf;
            }
            amlbt_task_start(&amlbt_dev);
        }

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
        ret = amlbt_bind_bus();
        if (ret != 0)
        {
            goto err_buf;
        }
#endif
        amlbt_dev.bt_start = 1;
        return nonseekable_open(inode, file);
err_buf:
        amlbt_res_deinit(&amlbt_dev);
err_exit:
        return -EIO;
    }
}

static int amlbt_close(struct inode *inode, struct file *file)
{
    int ret = 0;
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)file->private_data;

    BTI("%s, %d, version:%s\n", __func__, amlbt_ft_mode, AML_W2LU_VERSION);

    if (!amlbt_ft_mode)
    {
        amlbt_show_debug();
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
        amlbt_unbind_bus();
#endif
        if (!p_bt->zigbee_start && !p_bt->thread_start)
        {
            amlbt_show_fw_debug_info();
            p_bt->usb_irq_task_quit = 1;
            //wait_for_completion(&p_bt->comp);

            //amlbt_utils_w2l_usb_deinit();
            ret = amlbt_aon_addr_bit_clr(RG_AON_A24, 26);//wake up firmware, make sure firmware running
            if (ret != 0)
            {
                amlbt_res_deinit(&amlbt_dev);
                return 0;
            }

            amlbt_res_deinit(&amlbt_dev);
            amlbt_write_word(RG_AON_A15, 0, USB_EP2); //set bt_en auto mode
        }
        p_bt->bt_start = 0;
    }

    BTI("BT closed\n");
    return 0;
}

#if 0
static void amlbt_submit_read_urb(struct urb *urb)
{
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)urb->context;
    int status = urb->status;
    unsigned int actual_length = urb->actual_length;

    if (urb != p_bt->bt_urb)
    {
        BTE("%s:%d p_bt->usb_rx_buf != urb!!!\n", __func__, __LINE__);
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        USB_END_LOCK();
        return ;
    }

    if (urb->actual_length != POLL_TOTAL_LEN)
    {
        BTE("%s:%d urb->actual_length = %d!!!\n", __func__, __LINE__, urb->actual_length);
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        USB_END_LOCK();
        return ;
    }

    if (status == 0)
    {
        p_bt->usb_rx_len = actual_length;
        up(&p_bt->usb_irq_sem);
    }
    else
    {
        BTE("%s:%d URB read failed with status: %d\n", __func__, __LINE__, status);
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
    }
    USB_END_LOCK();
}

static void amlbt_submit_poll_urb_cb(struct urb *urb)
{
    int ret = 0;
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)urb->context;
    int status = urb->status;

    if (urb != p_bt->bt_urb)
    {
        BTE("%s:%d p_bt->usb_rx_buf != urb!!!\n", __func__, __LINE__);
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        USB_END_LOCK();
        return ;
    }
    if (urb->actual_length != sizeof(*g_cmd_buf))
    {
        BTE("%s:%d urb->actual_length = %d!!!\n", __func__, __LINE__, urb->actual_length);
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        USB_END_LOCK();
        return ;
    }

    if (g_udev == NULL)
    {
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        USB_END_LOCK();
        BTE("%s:%d g_udev == NULL!!!\n", __func__, __LINE__);
        return ;
    }

    if (p_bt->usb_rx_buf == NULL)
    {
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        USB_END_LOCK();
        BTE("%s:%d p_bt->usb_rx_buf == NULL!!!\n", __func__, __LINE__);
        return ;
    }

    if (status == 0)
    {
        usb_fill_bulk_urb(p_bt->bt_urb, g_udev, usb_rcvbulkpipe(g_udev, USB_EP2),
            (void *)p_bt->usb_rx_buf, POLL_TOTAL_LEN, amlbt_submit_read_urb, p_bt);
        ret = usb_submit_urb(p_bt->bt_urb, GFP_ATOMIC);
        if (ret < 0)
        {
            p_bt->usb_rx_len = 0;
            up(&p_bt->usb_irq_sem);
            USB_END_LOCK();
            BTE("%s:%d usb_submit_urb failed!\n", __func__, __LINE__);
        }
    }
    else
    {
        BTE("%s:%d URB read failed with status: %d\n", __func__, __LINE__, status);
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        USB_END_LOCK();
    }
}

static int amlbt_submit_poll_urb(w2l_usb_bt_new_t *p_bt)
{
    int ret = 0;

    USB_BEGIN_LOCK();

    if (g_udev == NULL)
    {
        BTE("%s:%d g_udev == NULL!!!\n", __func__, __LINE__);
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        USB_END_LOCK();
        return -1;
    }
    if (g_cmd_buf == NULL)
    {
        BTE("%s:%d g_cmd_buf == NULL!!!\n", __func__, __LINE__);
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        USB_END_LOCK();
        return -1;
    }

    if (bus_state_detect.bus_err || bus_state_detect.bus_reset_ongoing) {
        BTE("amlbt_submit_poll_urb bus reset is ongoing(bus err:%d, reset on going: %d:), do not read/write now!\n",
           bus_state_detect.bus_err, bus_state_detect.bus_reset_ongoing);
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        USB_END_LOCK();
        return -1;
    }

    if (p_bt->bt_urb == NULL)
    {
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        USB_END_LOCK();
        BTE("%s:%d p_bt->bt_urb == NULL!!!\n", __func__, __LINE__);
        return -1;
    }
    memset(g_cmd_buf, 0, sizeof(*g_cmd_buf));
    auc_build_cbw(g_cmd_buf,  AML_XFER_TO_HOST, POLL_TOTAL_LEN, CMD_READ_SRAM, HI_USB_EVENT_Q_ADDR, 0, POLL_TOTAL_LEN);
    usb_fill_bulk_urb(p_bt->bt_urb, g_udev, usb_sndbulkpipe(g_udev, USB_EP2),
        (void *)g_cmd_buf, sizeof(*g_cmd_buf), amlbt_submit_poll_urb_cb, p_bt);
    ret = usb_submit_urb(p_bt->bt_urb, GFP_ATOMIC);
    if (ret < 0)
    {
        BTE("%s:%d usb_submit_urb failed!\n", __func__, __LINE__);
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        USB_END_LOCK();
    }
    return ret;
}
#else
/*
static int amlbt_submit_poll_urb(w2l_usb_bt_new_t *p_bt)
{
    int ret = 0;

    ret = amlbt_read_sram(p_bt->usb_rx_buf, (unsigned char *)(unsigned long)HI_USB_EVENT_Q_ADDR, POLL_TOTAL_LEN, USB_EP2);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        p_bt->usb_rx_len = 0;
        up(&p_bt->usb_irq_sem);
        return ret;
    }
    p_bt->usb_rx_len = POLL_TOTAL_LEN;
    up(&p_bt->usb_irq_sem);

    return ret;
}*/
#endif

/*
type 0x04: 1 byte
head[evt code, length]: 2 bytes
payload
*/
static int amlbt_process_evt_to_skb(w2l_usb_bt_new_t *p_bt, unsigned char *evt_buf, unsigned int len)
{
    struct sk_buff *skb = NULL;
    unsigned int read_len = 0;
    unsigned int i = 0;
    unsigned char *p = evt_buf;

    //BTI("process evt %d\n", len);

    while (i < len)
    {
        read_len = 3 + p[2];
        read_len = (read_len + 3) & ~3;
        skb = alloc_skb(read_len, GFP_KERNEL);
        if (!skb)
        {
            BTE("%s:%d alloc_skb Failed\n", __func__, __LINE__);
            return -ENOMEM;
        }
        //BTI("push evt %d[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n", read_len,
        //    p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
        skb_put_data(skb, p, read_len);
        skb_queue_tail(&p_bt->bt_rx_queue, skb);
        p += read_len;
        i += read_len;
    }
    //BTI("process evt end\n");
    return 0;
}

/*
type 0x02: 4 byte
head[handle 2bytes, length 2bytes]: 4 bytes
payload
*/
static int amlbt_process_data_to_skb(w2l_usb_bt_new_t *p_bt, unsigned char *data_buf, unsigned int len)
{
    struct sk_buff *skb = NULL;
    unsigned int read_len = 0;
    unsigned int i = 0;
    unsigned char *p = data_buf;

    //BTI("process data %d\n", len);

    while (i < len)
    {
        read_len = 8 + ((p[7] << 8) | (p[6]));
        read_len = (read_len + 3) & ~3;
        skb = alloc_skb(read_len, GFP_KERNEL);
        if (!skb)
        {
            BTE("%s:%d alloc_skb Failed\n", __func__, __LINE__);
            return -ENOMEM;
        }
        //BTI("push data %d[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n", read_len,
        //    p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
        skb_put_data(skb, p, read_len);
        skb_queue_tail(&p_bt->bt_rx_queue, skb);
        p += read_len;
        i += read_len;
    }
    //BTI("process data end\n");
    return 0;
}

/*
type 0x10: 1 byte
head[MHDL(zigbee 0xF5, thread 0xFA), MID, length 2bytes]: 4 bytes
payload + 2 bytes crc
*/
static int amlbt_process_15p4_data_to_skb(w2l_usb_bt_new_t *p_bt, unsigned char *data_buf, unsigned int len)
{
    struct sk_buff *skb = NULL;
    unsigned int read_len = 0;
    unsigned int i = 0;
    unsigned char *p = data_buf;

    //BTI("process iot %d\n", len);

    while (i < len)
    {
        read_len = 7 + ((p[4] << 8) | (p[3]));
        read_len = (read_len + 3) & ~3;
        skb = alloc_skb(read_len, GFP_KERNEL);
        if (!skb)
        {
            BTE("%s:%d alloc_skb Failed\n", __func__, __LINE__);
            return -ENOMEM;
        }
        //BTI("push data %d[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n", read_len,
        //    p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
        skb_put_data(skb, p, read_len);
        if (p[1] == HCI_TYPE_ZIGBEE)
        {
#ifdef _15P4_SEPARATION
            skb_queue_tail(&p_bt->zigbee_rx_queue, skb);
#else
            skb_queue_tail(&p_bt->bt_rx_queue, skb);
#endif
        }
        else if (p[1] == HCI_TYPE_THREAD)
        {
#ifdef _15P4_SEPARATION
            skb_queue_tail(&p_bt->thread_rx_queue, skb);
#else
            skb_queue_tail(&p_bt->bt_rx_queue, skb);
#endif
        }
        p += read_len;
        i += read_len;
    }
    //BTI("process iot end\n");
    return 0;
}

static int amlbt_firmware_data_process(w2l_usb_bt_new_t *p_bt)
{
    unsigned char *p_buf = p_bt->usb_rx_buf;
    gdsl_fifo_t read_fifo = {0};
    unsigned int reg = 0;
    unsigned int type_size = 0;
    unsigned int evt_size = 0;
    unsigned int data_size = 0;
    unsigned int _15p4_size = 0;
    unsigned char read_reg[16] = {0};
    unsigned int key_value = 0;
    static unsigned char type_buff[FIFO_FW_RX_TYPE_LEN+4] = {0};
    static unsigned char fw_read_buff[USB_RX_Q_LEN*4] = {0};
    static unsigned char fw_data_buff[USB_RX_Q_LEN*4] = {0};

    int ret = 0;

    gdsl_fifo_t ro_fw_evt_fifo = *p_bt->fw_evt_fifo;
    gdsl_fifo_t ro_fw_type_fifo = *p_bt->fw_type_fifo;
    gdsl_fifo_t ro_fw_data_fifo = *p_bt->fw_data_fifo;
    gdsl_fifo_t ro_fw_15p4_fifo = *p_bt->_15p4_rx_fifo;


    //check fw gpio
    if (p_bt->input_key)
    {
        ret = amlbt_read_word(RG_AON_A17, USB_EP2, &key_value);
        if (ret != 0)
        {
            BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
            return -EFAULT;
        }
        //BTI("RG_AON_A17 %#x", key_value);
        key_value = (key_value >> 5) & 0x3;
        //BTI("key_value %#x", key_value);
        if (key_value != 0)
        {
            get_btwakeup_work(key_value);
        }
    }

    p_bt->fw_type_fifo->w = (unsigned char *)(unsigned long)((p_buf[35]<<24)|(p_buf[34]<<16)|(p_buf[33]<<8)|p_buf[32]);
    if (p_bt->fw_type_fifo->w == p_bt->fw_type_fifo->r) // no data
    {
        return 0;
    }
    memset(type_buff, 0, sizeof(type_buff));
    p_bt->fw_evt_fifo->w = (unsigned char *)(unsigned long)((p_buf[39]<<24)|(p_buf[38]<<16)|(p_buf[37]<<8)|p_buf[36]);
    p_bt->fw_data_fifo->w = (unsigned char *)(unsigned long)((p_buf[19]<<24)|(p_buf[18]<<16)|(p_buf[17]<<8)|p_buf[16]);
    p_bt->sink_mode = ((p_buf[23]<<24)|(p_buf[22]<<16)|(p_buf[21]<<8)|p_buf[20]);
    p_bt->_15p4_rx_fifo->w =(unsigned char *)(unsigned long)((p_buf[31]<<24)|(p_buf[30]<<16)|(p_buf[29]<<8)|p_buf[28]);

    BTD("dp1 type:w %#lx, r %#lx\n", (unsigned long)p_bt->fw_type_fifo->w, (unsigned long)p_bt->fw_type_fifo->r);
    BTD("dp1 evt:w %#lx, r %#lx\n", (unsigned long)p_bt->fw_evt_fifo->w, (unsigned long)p_bt->fw_evt_fifo->r);
    BTD("dp1 data:w %#lx, r %#lx\n", (unsigned long)p_bt->fw_data_fifo->w, (unsigned long)p_bt->fw_data_fifo->r);

    //copy type fifo
    read_fifo.base_addr = &p_buf[FIFO_FW_RX_TYPE_ADDR - HI_USB_EVENT_Q_ADDR];
    read_fifo.r = p_bt->fw_type_fifo->r;
    read_fifo.w = p_bt->fw_type_fifo->w;
    read_fifo.size = FIFO_FW_RX_TYPE_LEN;

    type_size = gdsl_fifo_get_data(&read_fifo, type_buff, sizeof(type_buff));
    if (type_buff[0] != HCI_ACLDATA_PKT && type_buff[0] != HCI_EVENT_PKT && type_buff[0] != HCI_15P4_PKT)
    {
        BTE("%s:%d type error!\n", __func__, __LINE__);
        BTE("fw_type_fifo->w:%#x fw_type_fifo->r:%#x", p_bt->fw_type_fifo->w, p_bt->fw_type_fifo->r);
        BTE("fw_evt_fifo->w:%#x fw_evt_fifo->r:%#x", p_bt->fw_evt_fifo->w, p_bt->fw_evt_fifo->r);
        BTE("fw_data_index_fifo->w:%#x fw_data_index_fifo->r:%#x", p_bt->fw_data_fifo->w, p_bt->fw_data_fifo->r);
        return -1;
    }

    BTP("type fifo:w %#lx, r %#lx\n", (unsigned long)p_bt->fw_type_fifo->w, (unsigned long)p_bt->fw_type_fifo->r);
    BTP("[%#x,%#x,%#x,%#x]\n", type_buff[0], type_buff[4], type_buff[8], type_buff[12]);
    p_bt->fw_type_fifo->r = read_fifo.r;

    reg = (((unsigned int)(unsigned long)read_fifo.r) & 0xff);
    read_reg[0] = (reg & 0xff);
    read_reg[1] = ((reg >> 8) & 0xff);
    read_reg[2] = ((reg >> 16) & 0xff);
    read_reg[3] = ((reg >> 24) & 0xff);


    //copy event fifo
    if (p_bt->fw_evt_fifo->w != p_bt->fw_evt_fifo->r)
    {
        read_fifo.base_addr = &p_buf[FIFO_FW_EVT_ADDR - HI_USB_EVENT_Q_ADDR];
        read_fifo.r = p_bt->fw_evt_fifo->r;
        read_fifo.w = p_bt->fw_evt_fifo->w;
        read_fifo.size = FIFO_FW_EVT_LEN;
        evt_size = gdsl_fifo_get_data(&read_fifo, fw_read_buff, sizeof(fw_read_buff));
    }
    BTP("evt fifo:w %#lx, r %#lx\n", (unsigned long)p_bt->fw_evt_fifo->w, (unsigned long)p_bt->fw_evt_fifo->r);
    if (evt_size)
    {
        p_bt->fw_evt_fifo->r = read_fifo.r;
        if (0 != amlbt_process_evt_to_skb(p_bt, fw_read_buff, evt_size))
        {
            return -1;
        }
    }
    reg = (((unsigned int)(unsigned long)p_bt->fw_evt_fifo->r) & 0x1fff);
    read_reg[4] = (reg & 0xff);
    read_reg[5] = ((reg >> 8) & 0xff);
    read_reg[6] = ((reg >> 16) & 0xff);
    read_reg[7] = ((reg >> 24) & 0xff);

    //copy data fifo
    if (p_bt->fw_data_fifo->w != p_bt->fw_data_fifo->r)
    {
        ret = amlbt_read_sram(&fw_data_buff[0],
                                              (unsigned char *)(unsigned long)(FIFO_FW_DATA_ADDR), FIFO_FW_DATA_LEN,
                                              USB_EP2);
        if (ret == -1)
        {
            BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
            goto err_exit;
        }
        else if (ret == -2)
        {
            BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
            goto err_buf;
        }
        read_fifo.base_addr = fw_data_buff;
        read_fifo.r = p_bt->fw_data_fifo->r;
        read_fifo.w = p_bt->fw_data_fifo->w;
        read_fifo.size = FIFO_FW_DATA_LEN;
        data_size = gdsl_fifo_get_data(&read_fifo, fw_read_buff, sizeof(fw_read_buff));
    }

    if (data_size)
    {
        p_bt->fw_data_fifo->r = read_fifo.r;

        if (0 != amlbt_process_data_to_skb(p_bt, fw_read_buff, data_size))
        {
            return -1;
        }
    }

    reg = (((unsigned int)(unsigned long)p_bt->fw_data_fifo->r) & 0x1fff);
    read_reg[12] = (reg & 0xff);
    read_reg[13] = ((reg >> 8) & 0xff);
    read_reg[14] = ((reg >> 16) & 0xff);
    read_reg[15] = ((reg >> 24) & 0xff);

    //copy 15.4 fifo
    if (p_bt->_15p4_rx_fifo->w != p_bt->_15p4_rx_fifo->r)
    {
        memset(fw_read_buff, 0, sizeof(fw_read_buff));
        _15p4_size = 0;
        ret = amlbt_read_sram(fw_read_buff, (unsigned char *)FIFO_FW_15P4_RX_ADDR, FIFO_FW_15P4_RX_LEN, USB_EP2);
        if (ret == -1)
        {
            BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
            goto err_exit;
        }
        else if (ret == -2)
        {
            BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
            goto err_buf;
        }
        read_fifo.base_addr = fw_read_buff;
        read_fifo.r = p_bt->_15p4_rx_fifo->r;
        read_fifo.w = p_bt->_15p4_rx_fifo->w;
        read_fifo.size = FIFO_FW_15P4_RX_LEN;
        _15p4_size = gdsl_fifo_get_data(&read_fifo, &fw_read_buff[FIFO_FW_15P4_RX_LEN], FIFO_FW_15P4_RX_LEN);
        BTP("15.4 get size %d\n", _15p4_size);
        BTP("15.4 rx:w %#lx, r %#lx\n", (unsigned long)p_bt->_15p4_rx_fifo->w, (unsigned long)p_bt->_15p4_rx_fifo->r);
        if (_15p4_size)
        {
            p_bt->_15p4_rx_fifo->r = read_fifo.r;
            //BTI("15.4 update rx rd %#x\n", amlbt_read_word(FIFO_FW_15P4_RX_R, USB_EP2));
            amlbt_process_15p4_data_to_skb(p_bt, &fw_read_buff[FIFO_FW_15P4_RX_LEN], _15p4_size);
        }
    }

    reg = (((unsigned int)(unsigned long)p_bt->_15p4_rx_fifo->r) & 0x7ff);
    read_reg[8] = (reg & 0xff);
    read_reg[9] = ((reg >> 8) & 0xff);
    read_reg[10] = ((reg >> 16) & 0xff);
    read_reg[11] = ((reg >> 24) & 0xff);

    ret = amlbt_write_sram(&read_reg[0], (unsigned char *)(unsigned long)(FIFO_FW_RX_TYPE_R), sizeof(read_reg), USB_EP2);
    if (ret == -1)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        goto err_exit;
    }
    else if (ret == -2)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        goto err_buf;
    }

    if (!skb_queue_empty(&p_bt->bt_rx_queue))
    {
        wake_up_interruptible(&p_bt->rd_wait_queue);
    }
    if (!skb_queue_empty(&p_bt->zigbee_rx_queue))
    {
        wake_up_interruptible(&p_bt->zigbee_wait_queue);
    }
    if (!skb_queue_empty(&p_bt->thread_rx_queue))
    {
        wake_up_interruptible(&p_bt->thread_wait_queue);
    }

    return 0;
err_buf:
    *p_bt->fw_evt_fifo = ro_fw_evt_fifo;
    *p_bt->fw_type_fifo =  ro_fw_type_fifo;
    *p_bt->fw_data_fifo = ro_fw_data_fifo;
    *p_bt->_15p4_rx_fifo = ro_fw_15p4_fifo;
    return -2;
err_exit:
    return -1;
}

static int amlbt_drv_state_process(w2l_usb_bt_new_t *p_bt)
{
    int ret = BT_DRV_NONE;

    if (p_bt->dr_state & BT_DRV_STATE_WAIT_RECOVERY)
    {
        if (bus_state_detect.bus_err || \
                bus_state_detect.bus_reset_ongoing || \
                    bus_state_detect.is_recy_ongoing || \
                        (enum usb_udev_state)g_udev->state != USB_CONFIGURED)
        {
            if ((sched_clock() - p_bt->wait_start) >= MAX_TIMEOUT)
            {
                BTE("%s bus_err %d reset %d recy %d udev %d", __func__, bus_state_detect.bus_err, bus_state_detect.bus_reset_ongoing,
                    bus_state_detect.is_recy_ongoing, g_udev->state);
            }
        }
        else
        {
            amlbt_wakeup_lock();
            amlbt_drv_state_set(BT_DRV_STATE_RECOVERY);
            amlbt_drv_state_clr(BT_DRV_STATE_WAIT_RECOVERY);
        }
        ret = BT_DRV_WAIT_RECOVERY;
        goto exit;
    }

    if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
    {
        p_bt->usb_irq_task_quit = 1;
        if (p_bt->bt_start)
        {
            wake_up_interruptible(&p_bt->rd_wait_queue);
        }
        if (p_bt->zigbee_start)
        {
            wake_up_interruptible(&p_bt->zigbee_wait_queue);
        }
        if (p_bt->thread_start)
        {
            wake_up_interruptible(&p_bt->thread_wait_queue);
        }
    }

    if (p_bt->usb_irq_task_quit)
    {
        BTW("%s:%d usb_irq_task_quit == 1!\n", __func__, __LINE__);
        //complete(&p_bt->comp);
        ret = BT_DRV_CLOSED;
        goto exit;
    }

    if ((p_bt->dr_state & BT_DRV_STATE_SUSPEND_ENTRY) || \
           (p_bt->dr_state & BT_DRV_STATE_SUSPEND))
    {
        ret = BT_DRV_SUSPEND;
        goto exit;
    }
    if (p_bt->dr_state & BT_DRV_STATE_RESUME)
    {
        ret = BT_DRV_RESUME;
        goto exit;
    }
exit:
    return ret;
}

#if 0
static void amlbt_polling_time_check(w2l_usb_bt_new_t *p_bt)
{
    if (p_bt->sink_mode)
    {
        polling_time = 1000;
    }
    else
    {
        polling_time = 8000;
    }
    return ;
}

static int amlbt_irq_task(void *data)
{
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)data;
    //process data
    int actual_length = 0;
    int ret = 0;
    int err = 0;
    while (1)
    {
        ret = amlbt_drv_state_process(p_bt);

        if (ret == BT_DRV_NONE)
        {
            if (down_interruptible(&p_bt->usb_irq_sem) != 0)
            {
                /* interrupted, exit */
                BTE("%s:%d wait usb_irq_sem fail!\n", __func__, __LINE__);
                break;
            }
            data = p_bt->usb_rx_buf;
            actual_length = p_bt->usb_rx_len;
            if (actual_length == POLL_TOTAL_LEN)
            {
                mutex_lock(&p_bt->bt_debug_mutex);
                err = amlbt_firmware_data_process(p_bt);
                mutex_unlock(&p_bt->bt_debug_mutex);
                if (err == -1)
                {
                    BTE("%s:%d Failed : %d\n", __func__, __LINE__, err);
                    amlbt_drv_state_set(BT_DRV_STATE_RECOVERY);
                    complete(&p_bt->comp);
                    wake_up_interruptible(&p_bt->rd_wait_queue);
                    break;
                }
                else if (err == -2)
                {
                    continue;
                }
                amlbt_polling_time_check(p_bt);
            }
            else
            {
                BTE("%s:%d usb rx data length not match!!, %d\n", __func__, __LINE__, actual_length);
            }
            usleep_range(polling_time, polling_time);
            p_bt->usb_rx_len = 0;
            mutex_lock(&p_bt->bt_debug_mutex);
            err = amlbt_submit_poll_urb(p_bt);
            mutex_unlock(&p_bt->bt_debug_mutex);
            if (err == -1)
            {
                BTE("%s:%d Failed : %d\n", __func__, __LINE__, err);
                amlbt_drv_state_set(BT_DRV_STATE_RECOVERY);
                complete(&p_bt->comp);
                wake_up_interruptible(&p_bt->rd_wait_queue);
                break;
            }
            else if (err == -2)
            {
                continue;
            }
        }
        else if (ret == BT_DRV_CLOSED)
        {
            BTW("%s:%d BT_DRV_CLOSED!\n", __func__, __LINE__);
            break;
        }
        else if (ret == BT_DRV_SUSPEND || ret == BT_DRV_RESUME)
        {
            BTA("%s:%d BT_DRV_SUSPEND or BT_DRV_RESUME!\n", __func__, __LINE__);
            usleep_range(polling_time, polling_time);
        }
    }

    BTW("%s:%d amlbt_irq_task quit!\n", __func__, __LINE__);
    return 0;
}

static int amlbt_task_start(w2l_usb_bt_new_t *p_bt)
{
    sema_init(&p_bt->usb_irq_sem, 0);
    p_bt->usb_irq_task_quit = 0;
    p_bt->usb_irq_task = kthread_run(amlbt_irq_task, &amlbt_dev, "aml_irq_task");
    BTI("amlbt_task_start:%#lx\n", (unsigned long)&amlbt_dev);
    if (IS_ERR(p_bt->usb_irq_task)) {
        p_bt->usb_irq_task = NULL;
        BTE("create usb task error!!!!\n");
        return -1;
    }
    reinit_completion(&p_bt->comp);
    return amlbt_submit_poll_urb(p_bt);
}
#else

static enum hrtimer_restart amlbt_hrtime(struct hrtimer *timer)
{
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;

    if (p_bt->sink_mode)
    {
        p_bt->ktime = ktime_set(0, POLLING_LEVEL_1);
    }
    else
    {
        p_bt->ktime = ktime_set(0, POLLING_LEVEL_3);
    }

    queue_work(p_bt->check_fw_wq, &p_bt->check_fw);
    hrtimer_start(timer, p_bt->ktime, HRTIMER_MODE_REL);

    return HRTIMER_RESTART;
}


static void amlbt_usb_fw_rx_work(struct work_struct *work)
{
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;
    //process data
    int actual_length = 0;
    int ret = 0;
    int err = 0;
    unsigned char *data;

    if (p_bt->dr_state == 0)
    {
        mutex_lock(&p_bt->bt_debug_mutex);
        ret = amlbt_read_sram(p_bt->usb_rx_buf, (unsigned char *)(unsigned long)HI_USB_EVENT_Q_ADDR, POLL_TOTAL_LEN, USB_EP2);
        if (ret != 0)
        {
            BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
            p_bt->usb_rx_len = 0;
            mutex_unlock(&p_bt->bt_debug_mutex);
            if (err == -1)
            {
                BTE("%s:%d Failed : %d\n", __func__, __LINE__, err);
                if (!(p_bt->dr_state & BT_DRV_STATE_WAIT_RECOVERY))
                {
                    amlbt_drv_state_set(BT_DRV_STATE_RECOVERY);
                    //complete(&p_bt->comp);
                    wake_up_interruptible(&p_bt->rd_wait_queue);
                }
                return ;
            }
        }
        p_bt->usb_rx_len = POLL_TOTAL_LEN;
        mutex_unlock(&p_bt->bt_debug_mutex);
    }
    else
    {
        p_bt->usb_rx_len = 0;
    }

    ret = amlbt_drv_state_process(p_bt);

    if (ret == BT_DRV_NONE)
    {
        data = p_bt->usb_rx_buf;
        actual_length = p_bt->usb_rx_len;
        if (actual_length == POLL_TOTAL_LEN)
        {
            mutex_lock(&p_bt->bt_debug_mutex);
            err = amlbt_firmware_data_process(p_bt);
            mutex_unlock(&p_bt->bt_debug_mutex);
            if (err == -1)
            {
                BTE("%s:%d Failed : %d\n", __func__, __LINE__, err);
                if (!(p_bt->dr_state & BT_DRV_STATE_WAIT_RECOVERY))
                {
                    amlbt_drv_state_set(BT_DRV_STATE_RECOVERY);
                    //complete(&p_bt->comp);
                    wake_up_interruptible(&p_bt->rd_wait_queue);
                }
                return ;
            }
        }
        else
        {
            BTE("%s:%d usb rx data length not match!!, %d\n", __func__, __LINE__, actual_length);
        }
        p_bt->usb_rx_len = 0;
    }
    else if (ret == BT_DRV_CLOSED)
    {
        BTW("%s:%d BT_DRV_CLOSED!\n", __func__, __LINE__);
    }
    else if (ret == BT_DRV_SUSPEND || ret == BT_DRV_RESUME || ret == BT_DRV_WAIT_RECOVERY)
    {
        BTA("%s:%d BT_DRV_SUSPEND or BT_DRV_RESUME or BT_DRV_WAIT_RECOVERY!\n", __func__, __LINE__);
    }
}

static int amlbt_task_start(w2l_usb_bt_new_t *p_bt)
{
    p_bt->ktime = ktime_set(0, POLLING_LEVEL_3); // 8 millisecond
    hrtimer_start(&p_bt->poll_timer, p_bt->ktime, HRTIMER_MODE_REL);
    return 0;
}
#endif

static int amlbt_download_firmware(w2l_usb_bt_new_t *p_bt)
{
    int ret = 0;
    unsigned int offset = 0;
    unsigned int remain_len = 0;
    unsigned int iccm_base_addr = BT_ICCM_AHB_BASE + ICCM_ROM_SIZE;
    unsigned int dccm_base_addr = BT_DCCM_AHB_BASE;
    uint8_t *check_buf = kzalloc(DOWNLOAD_SIZE, GFP_DMA|GFP_ATOMIC);

    if (check_buf == NULL)
    {
        BTF("amlbt_download_firmware check_buf alloc failed!!!\n");
        return -1;
    }

    memset(check_buf, 0, DOWNLOAD_SIZE);
    remain_len = ICCM_SIZE;

    //to do download bt fw
    BTI("amlbt_download_firmware:iccm size %#x, remain_len %#x\n", ICCM_SIZE, remain_len);

    while (offset < ICCM_SIZE)
    {
        if (remain_len < DOWNLOAD_SIZE)
        {
            BTD("amlbt_download_firmware iccm1 offset %#x, addr %#x\n", offset, iccm_base_addr);
            amlbt_write_sram((unsigned char *)&p_bt->iccm_buf[offset], (unsigned char *)(unsigned long)iccm_base_addr, remain_len, USB_EP2);
            amlbt_read_sram(check_buf, (unsigned char *)(unsigned long)iccm_base_addr, remain_len, USB_EP2);
            if (memcmp(check_buf, &p_bt->iccm_buf[offset], remain_len))
            {
                BTE("Firmware iccm check2 error! offset %#x\n", offset);
                ret = -1;
                goto error;
            }
            offset += remain_len;
            iccm_base_addr += remain_len;
            BTD("amlbt_download_firmware iccm1 offset %#x, write_len %#x\n", offset, remain_len);
        }
        else
        {
            BTD("amlbt_download_firmware iccm2 offset %#x, write_len %#x, addr %#x\n", offset, DOWNLOAD_SIZE, iccm_base_addr);
            amlbt_write_sram((unsigned char *)&p_bt->iccm_buf[offset], (unsigned char *)(unsigned long)iccm_base_addr, DOWNLOAD_SIZE, USB_EP2);
            amlbt_read_sram(check_buf, (unsigned char *)(unsigned long)iccm_base_addr, DOWNLOAD_SIZE, USB_EP2);
            if (memcmp(check_buf, &p_bt->iccm_buf[offset], DOWNLOAD_SIZE))
            {
                BTE("Firmware iccm check error! offset %#x\n", offset);
                ret = -1;
                goto error;
            }
            offset += DOWNLOAD_SIZE;
            remain_len -= DOWNLOAD_SIZE;
            iccm_base_addr += DOWNLOAD_SIZE;
        }
        BTD("amlbt_download_firmware iccm remain_len %#x\n", remain_len);
    }

    BTI("Firmware iccm check pass, offset %#x\n", offset);
    offset = 0;
    remain_len = DCCM_SIZE;
    //to do download bt fw
    BTI("amlbt_download_firmware:dccm size %#x, remain_len %#x\n", DCCM_SIZE, remain_len);
    while (offset < DCCM_SIZE)
    {
        if (remain_len < DOWNLOAD_SIZE)
        {
            BTD("amlbt_download_firmware dccm1 offset %#x, addr %#x\n", offset, dccm_base_addr);
            amlbt_write_sram((unsigned char *)&p_bt->dccm_buf[offset], (unsigned char *)(unsigned long)dccm_base_addr, remain_len, USB_EP2);
            amlbt_read_sram(check_buf, (unsigned char *)(unsigned long)dccm_base_addr, remain_len, USB_EP2);
            if (memcmp(check_buf, &p_bt->dccm_buf[offset], remain_len))
            {
                BTE("Firmware dccm check2 error! offset %#x\n", offset);
                ret = -1;
                goto error;
            }
            offset += remain_len;
            dccm_base_addr += remain_len;
            BTD("amlbt_download_firmware dccm1 offset %#x, write_len %#x\n", offset, remain_len);
        }
        else
        {
            BTD("amlbt_download_firmware dccm2 offset %#x, write_len %#x, addr%#x\n", offset, DOWNLOAD_SIZE, dccm_base_addr);
            amlbt_write_sram((unsigned char *)&p_bt->dccm_buf[offset], (unsigned char *)(unsigned long)dccm_base_addr, DOWNLOAD_SIZE, USB_EP2);
            amlbt_read_sram(check_buf, (unsigned char *)(unsigned long)dccm_base_addr, DOWNLOAD_SIZE, USB_EP2);
            if (memcmp(check_buf, &p_bt->dccm_buf[offset], DOWNLOAD_SIZE))
            {
                BTE("Firmware dccm check error! offset %#x\n", offset);
                ret = -1;
                goto error;
            }
            offset += DOWNLOAD_SIZE;
            remain_len -= DOWNLOAD_SIZE;
            dccm_base_addr += DOWNLOAD_SIZE;
        }
        BTD("amlbt_download_firmware dccm remain_len %#x \n", remain_len);
    }
    BTI("Firmware dccm check pass, offset %#x\n", offset);
error:
    kfree(check_buf);
    return ret;
}

static unsigned int amlbt_w2lu_coex_is_running(w2l_usb_bt_new_t *p_bt)
{
    BTI("%s, %#x, %#x, %#x\n", __func__, p_bt->bt_start, p_bt->zigbee_start, p_bt->thread_start);

    if (p_bt->bt_start || p_bt->zigbee_start || p_bt->thread_start)
    {
        return 1;
    }

    return 0;
}


static int amlbt_send_hci_cmd(w2l_usb_bt_new_t *p_bt, unsigned char *data, unsigned int len)
{
    int ret = 0;
    unsigned int val = 0;
    gdsl_fifo_t t_fifo;
    BTA("%s, len %d \n", __func__, len);

    if (p_bt->tx_cmd_fifo == NULL)
    {
        BTE("%s: p_bt->tx_cmd_fifo NULL!!!!\n", __func__);
        goto err_exit;
    }
    t_fifo = *p_bt->tx_cmd_fifo;

    len = ((len + 3) & 0xFFFFFFFC);//Keep 4 bytes aligned
    BTA("%s, Actual length %d \n", __func__, len);
    //step 1: Update the command FIFO read pointer
    ret = amlbt_read_word(FIFO_FW_CMD_R, USB_EP2, &val);
    BTA("cmd r %#x\n", val);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        goto err_exit;
    }
    t_fifo.r = (unsigned char *)(unsigned long)val;
    //step 3: Write HCI commands to WiFi SRAM
    ret = gdsl_write_data_by_ep(&t_fifo, data, len, USB_EP2);
    //step 4: Update the write pointer and write to WiFi SRAM
    if (ret < 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        goto err_exit;
    }

    BTA("before write:r:%#lx, w:%#lx\n", (unsigned long)p_bt->tx_cmd_fifo->r, (unsigned long)p_bt->tx_cmd_fifo->w);

    ret = amlbt_write_word(FIFO_FW_CMD_W, (unsigned long)t_fifo.w & 0xfff, USB_EP2);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        goto err_exit;
    }
    *p_bt->tx_cmd_fifo = t_fifo;
    BTP("len %#x:w %#lx, r %#lx\n", len, (unsigned long)p_bt->tx_cmd_fifo->w, (unsigned long)p_bt->tx_cmd_fifo->r);
    amlbt_debug_get_cmd((unsigned int)(unsigned long)p_bt->tx_cmd_fifo->w, (unsigned int)(unsigned long)p_bt->tx_cmd_fifo->r, data);

    return ret;
err_exit:
    return ret;
}

static unsigned int amlbt_get_tx_prio(gdsl_tx_q_t *p_fifo, unsigned int acl_handle)
{
    unsigned int prio = 0;
    unsigned int i = 0;
    unsigned int find = 0;

    for (i = 0; i < USB_TX_Q_NUM; i++)
    {
        if (p_fifo[i].tx_q_dev_index == acl_handle/* && p_fifo[i].tx_q_status == GDSL_TX_Q_USED*/)
        {
            if (p_fifo[i].tx_q_prio >= prio)
            {
                prio = p_fifo[i].tx_q_prio;
                find = 1;
            }
        }
    }

    if (!find)
    {
        prio = TX_Q_MAX_PRIO;
    }

    return prio;
}


static int amlbt_send_hci_data(w2l_usb_bt_new_t *p_bt, unsigned char *data, unsigned int len)
{
    int ret = 0;
    unsigned int i = 0;
    unsigned int acl_handle = (((data[1] << 8) | data[0]) & 0xfff);
    unsigned int prio = 0;
    unsigned int tx_q_prio[USB_TX_Q_NUM] = {0};
    unsigned int tx_q_index[USB_TX_Q_NUM] = {0};
    unsigned int tx_q_status[USB_TX_Q_NUM] = {0};
    unsigned int tx_buff[USB_TX_Q_NUM * 4] = {0};//prio, index, status
    gdsl_tx_q_t ro_tx_q = *p_bt->tx_q;
    BTA("%s, len:%d\n", __func__, len);

    ret = amlbt_read_sram((unsigned char *)tx_buff, (unsigned char *)p_bt->tx_q[0].tx_q_prio_addr,
                                          sizeof(tx_buff), USB_EP2);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        USB_END_LOCK();
        goto err_exit;
    }
    for (i = 0; i < USB_TX_Q_NUM; i++)
    {
        tx_q_prio[i] = tx_buff[i*4];
        tx_q_index[i]  = tx_buff[i*4+1];
        tx_q_status[i]   = tx_buff[i*4+2];
    }
    BTA("P %#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x\n", tx_q_prio[0],tx_q_prio[1],tx_q_prio[2],tx_q_prio[3],
        tx_q_prio[4],tx_q_prio[5],tx_q_prio[6],tx_q_prio[7]);
    BTA("A %#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x\n", tx_q_index[0],tx_q_index[1],tx_q_index[2],tx_q_index[3],
        tx_q_index[4],tx_q_index[5],tx_q_index[6],tx_q_index[7]);
    BTA("S %#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x\n", tx_q_status[0],tx_q_status[1],tx_q_status[2],tx_q_status[3],
        tx_q_status[4],tx_q_status[5],tx_q_status[6],tx_q_status[7]);
    for (i = 0; i < USB_TX_Q_NUM; i++)
    {
        if (tx_q_status[i] == GDSL_TX_Q_COMPLETE)
        {
            tx_q_index[i] = 0;
            tx_q_status[i] = GDSL_TX_Q_UNUSED;
            tx_q_prio[i] = TX_Q_MAX_PRIO;
        }
    }

    for (i = 0; i < USB_TX_Q_NUM; i++)
    {
        p_bt->tx_q[i].tx_q_dev_index = tx_q_index[i];
        p_bt->tx_q[i].tx_q_status = tx_q_status[i];
        p_bt->tx_q[i].tx_q_prio = tx_q_prio[i];
    }

    for (i = 0; i < USB_TX_Q_NUM; i++)
    {
        if (p_bt->tx_q[i].tx_q_status == GDSL_TX_Q_UNUSED)
        {
            break;
        }
    }

    if (i == USB_TX_Q_NUM)
    {
        BTE("%s: hci data space invalid!!!!\n", __func__);
        for (i = 0; i < USB_TX_Q_NUM; i++)
        {
            BTI("[%#x,%#x,%#x]", (unsigned int)p_bt->tx_q[i].tx_q_prio,
                    (unsigned int)p_bt->tx_q[i].tx_q_dev_index,
                    (unsigned int)p_bt->tx_q[i].tx_q_status);
            BTI("{%#x,%#x,%#x}", tx_q_prio[i], tx_q_index[i],tx_q_status[i]);
        }
        return -1;
    }
    ro_tx_q = *p_bt->tx_q;

    prio = amlbt_get_tx_prio(p_bt->tx_q, acl_handle);

    p_bt->tx_q[i].tx_q_prio = (++prio & TX_Q_MAX_PRIO);
    p_bt->tx_q[i].tx_q_dev_index = acl_handle;
    p_bt->tx_q[i].tx_q_status = GDSL_TX_Q_USED;


    len = (len + 3) & ~3;
    ret = amlbt_write_sram(data, p_bt->tx_q[i].tx_q_addr, len, USB_EP2);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        USB_END_LOCK();
        *p_bt->tx_q = ro_tx_q;
        goto err_exit;
    }

    tx_buff[0] = p_bt->tx_q[i].tx_q_prio;
    tx_buff[1] = p_bt->tx_q[i].tx_q_dev_index;
    tx_buff[2] = p_bt->tx_q[i].tx_q_status;
    tx_buff[3] = 0;

    BTA("TX:%d,%d,%#x,%#x\n", i, len, acl_handle, p_bt->tx_q[i].tx_q_prio);
    ret = amlbt_write_sram((unsigned char *)&tx_buff[0], (unsigned char *)p_bt->tx_q[i].tx_q_prio_addr,
                                      sizeof(unsigned int)*4, USB_EP2);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        *p_bt->tx_q = ro_tx_q;
        goto err_exit;
    }
    BTA("%s, Actual length:%d\n", __func__, len);
    return 0;
err_exit:
    return ret;
}

static int amlbt_send_15p4_data(w2l_usb_bt_new_t *p_bt,unsigned char *data, unsigned int len)
{
    int ret = 0;
    unsigned int val = 0;
    gdsl_fifo_t p_fifo;
    BTP("%s, len %d \n", __func__, len);

    if (p_bt->_15p4_tx_fifo == NULL)
    {
        BTE("%s: p_bt->_15p4_tx_fifo NULL!!!!\n", __func__);
        return -1;
    }
    p_fifo = *p_bt->_15p4_tx_fifo;

    len = ((len + 3) & 0xFFFFFFFC);//Keep 4 bytes aligned

    BTA("%s, Actual length %d \n", __func__, len);
    //step 1: Update the tx FIFO read pointer
    ret = amlbt_read_word(FIFO_FW_15P4_TX_R, USB_EP2, &val);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        goto err_exit;
    }
    p_fifo.r = (unsigned char *)(unsigned long)val;
    BTP("15p4 tx r %#x\n", (unsigned long)p_fifo.r);
    BTP("15p4 tx w %#x\n", (unsigned long)p_fifo.w);
    //step 2: Check the command FIFO space

    //step 3: Write HCI commands to WiFi SRAM
    ret = gdsl_write_data_by_ep(&p_fifo, data, len, USB_EP2);
    if (ret < 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        goto err_exit;
    }
    //step 4: Update the write pointer and write to WiFi SRAM
    //BTI("15p4 before write:r:%#lx, w:%#lx\n", (unsigned long)p_bt->_15p4_tx_fifo->r, (unsigned long)p_bt->_15p4_tx_fifo->w);
    ret = amlbt_write_word(FIFO_FW_15P4_TX_W, (unsigned long)p_fifo.w & 0x7ff, USB_EP2);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        goto err_exit;
    }
    *p_bt->_15p4_tx_fifo = p_fifo;
    BTP("15p4 len %#x:w %#lx, r %#lx\n", len, (unsigned long)p_bt->_15p4_tx_fifo->w, (unsigned long)p_bt->_15p4_tx_fifo->r);
    return ret;
err_exit:
    return ret;
}


static ssize_t amlbt_write(struct file *file_p,
                                     const char __user *buf_p,
                                     size_t count,
                                     loff_t *pos_p)
{
    int i = 0;
    static unsigned int w_type = 0;
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)file_p->private_data;
    struct sk_buff *skb;
    unsigned char *p;

    BTA("%s, count:%ld\n", __func__, count);

    if (count > HCI_MAX_FRAME_SIZE)
    {
        BTE("%s:%d count > HCI_MAX_FRAME_SIZE %d, %d!\n", __func__, __LINE__, count, HCI_MAX_FRAME_SIZE);
        return -EINVAL;
    }

    if (count == 1) //host write hci type
    {
        get_user(w_type, buf_p);
        return count;
    }

    if (!p_bt->bt_start)
    {
        BTE("%s:%d p_bt->bt_start == 0!\n", __func__, __LINE__);
        return -EINVAL;
    }

    skb = alloc_skb(count+1, GFP_KERNEL);
    if (!skb)
    {
        return -ENOMEM;
    }

    *(unsigned char *)skb_put(skb, 1) = w_type;

    if (copy_from_user(skb_put(skb, count), buf_p, count))
    {
        kfree_skb(skb);
        BTE("%s: Failed to get data from user space\n", __func__);
        return -EFAULT;
    }
#ifndef _15P4_SEPARATION
    //p = &skb->data[0];
    //if (w_type == HCI_15P4_PKT)
    //{
    //    BTI("write 15.4 %d, %d:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n", skb->len, count,
    //            p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
    //}
#endif
    p = &skb->data[1];

    if (w_type == HCI_COMMAND_PKT)
    {
        if (count == 0x0f && p[0] == 0x27 && p[1] == 0xfc)   //close
        {
            BTP("close cmd:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]",
                p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
            BTW("bluetooth close!\n");
            kfree_skb(skb);
            return count;
        }
        if (p[0] == 0x05 && p[1] == 0x14)    // read rssi
        {
            BTI("rs\n");
        }
        if (p[0] == 0x1a && p[1] == 0xfc)
        {
            for (; i < sizeof(p_bt->mac_addr); i++)
            {
                p_bt->mac_addr[i] = p[i+3];
            }
        }
    }

    skb_queue_tail(&p_bt->tx_queue, skb);
    schedule_work(&p_bt->write_work);

    return count;
}

static ssize_t amlbt_read(struct file *file_p,
                                   char __user *buf_p,
                                   size_t count,
                                   loff_t *pos_p)
{
    unsigned char hw_error_evt[5] = {0x04, 0x10, 0x01, 0x00, 0x00};
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)file_p->private_data;
    static struct sk_buff *skb;
    unsigned char *p = NULL;

    if (!p_bt->bt_start)
    {
        BTE("%s:%d p_bt->bt_start == 0!\n", __func__, __LINE__);
        return -EFAULT;
    }

    BTD("rd %#x, count %d\n", p_bt->rd_state, count);

    switch (p_bt->rd_state)
    {
        case HCI_RX_TYPE:   //read type
            if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
            {
                BTW("%s:%d BT_DRV_STATE_RECOVERY!\n", __func__, __LINE__);
                if (copy_to_user(buf_p, &hw_error_evt[0], count))
                {
                    BTE("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
                p_bt->rd_state = HCI_RX_HEADER;
                return count;
            }
            skb = skb_dequeue(&p_bt->bt_rx_queue);
            if (!skb)
            {
                BTE("zb HCI_RX_TYPE no data!\n");
                return 0;
            }
            if (skb->len < count)
            {
                BTE("%s:%d Failed to copy data: %d, %d\n", __func__, __LINE__, skb->len, count);
                return -EFAULT;
            }
            if (copy_to_user(buf_p, skb->data, count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                kfree_skb(skb);
                p_bt->rd_state = HCI_RX_TYPE;
                return -EFAULT;
            }
            p = skb->data;
#ifdef _15P4_SEPARATION
            if (p[0] == HCI_EVENT_PKT)
#else
            if (p[0] == HCI_EVENT_PKT || p[0] == HCI_15P4_PKT)
#endif
            {
                if (amlbt_debug_filter_event(p) == 0)
                {
                    amlbt_debug_get_event((unsigned int)p_bt->fw_evt_fifo->w, (unsigned int)p_bt->fw_evt_fifo->r, p);
                }
                skb_pull(skb, count);
            }
            else if (p[0] == HCI_ACLDATA_PKT)
            {
                skb_pull(skb, 4);
            }
            else
            {
                BTE("%s, bt type error! \n", __func__);
                return -EFAULT;
            }
            p_bt->rd_state = HCI_RX_HEADER;
        break;
        case HCI_RX_HEADER: // read header
            if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
            {
                if (copy_to_user(buf_p, &hw_error_evt[1], count))
                {
                    BTE("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
                p_bt->rd_state = HCI_RX_PAYLOAD;
                return count;
            }

            if (skb->len < count)
            {
                BTE("%s:%d Failed to copy data: %d, %d\n", __func__, __LINE__, skb->len, count);
                return -EFAULT;
            }
            if (copy_to_user(buf_p, skb->data, count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                kfree_skb(skb);
                p_bt->rd_state = HCI_RX_TYPE;
                return -EFAULT;
            }
            skb_pull(skb, count);
            if (skb->len == 0)
            {
                kfree_skb(skb);
                p_bt->rd_state = HCI_RX_TYPE;
            }
            else
            {
                p_bt->rd_state = HCI_RX_PAYLOAD;
            }
        break;
        case HCI_RX_PAYLOAD:    //read payload

            if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
            {
                if (copy_to_user(buf_p, &hw_error_evt[3], count))
                {
                    BTE("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
                p_bt->rd_state = HCI_RX_FATAL;
                p_bt->bt_start = 0;
                return count;
            }

            if (skb->len < count)
            {
                BTE("%s:%d Failed to copy data: %d, %d\n", __func__, __LINE__, skb->len, count);
                return -EFAULT;
            }
            if (copy_to_user(buf_p, skb->data, count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                kfree_skb(skb);
                p_bt->rd_state = HCI_RX_TYPE;
                return -EFAULT;
            }
            kfree_skb(skb);
            p_bt->rd_state = HCI_RX_TYPE;
            break;
        default:
            BTE("%s, evt_state error!!\n", __func__);
            break;
    }
    return count;
}

static unsigned int amlbt_poll(struct file *file, poll_table *wait)
{
    int mask = 0;
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)file->private_data;

    poll_wait(file, &p_bt->rd_wait_queue, wait);

    if (!p_bt->bt_start)
    {
        mask |= POLLIN | POLLRDNORM;
        goto exit;
    }

    if ((p_bt->dr_state & BT_DRV_STATE_RECOVERY) && p_bt->rd_state == HCI_RX_FATAL)
    {
        goto exit;
    }

    if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
    {
        mask |= POLLIN | POLLRDNORM;
        goto exit;
    }

    if ((g_udev == NULL) || bus_state_detect.bus_err || bus_state_detect.bus_reset_ongoing)
    {
        BTF("%s:%d usb error!, %#x,%#x\n", __func__, __LINE__, bus_state_detect.bus_err, bus_state_detect.bus_reset_ongoing);
        goto exit;
    }

    if (p_bt->rd_state > HCI_RX_TYPE || skb_queue_len(&p_bt->bt_rx_queue) > 0)
    {
        mask |= POLLIN | POLLRDNORM;
        goto exit;
    }

exit:
    return mask;
}

static long amlbt_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    unsigned char coex_running = 0;
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)filp->private_data;

    switch (cmd)
    {
        case IOCTL_GET_BT_RECOVERY:
        {
            if (copy_to_user((unsigned char __user *)arg, &amlbt_dev.recovery_value, sizeof(unsigned long)) != 0)
            {
                BTE("IOCTL_GET_BT_RECOVERY copy error\n");
                return -EFAULT;
            }
            BTI("IOCTL_GET_BT_RECOVERY %#x\n", amlbt_dev.recovery_value);
        }
        break;
        case IOCTL_GET_DEVICE_PID:
        {
            if (copy_to_user((unsigned char __user *)arg, &g_chip_function_ctrl, sizeof(unsigned char)) != 0)
            {
                BTE("IOCTL_GET_DEVICE_PID copy error\n");
                return -EFAULT;
            }
            BTI("IOCTL_GET_DEVICE_PID %#x\n", g_chip_function_ctrl);
        }
        break;
        case IOCTL_SET_BT_SHUTDOWN:
        {
            if (copy_from_user(&amlbt_dev.shutdown_value, (unsigned char __user *)arg, sizeof(unsigned long)) != 0)
            {
                BTE("IOCTL_SET_BT_SHUTDOWN copy error\n");
                return -EFAULT;
            }
            BTI("IOCTL_SET_BT_SHUTDOWN %#x\n", amlbt_dev.shutdown_value);
        }
        break;
        case IOCTL_GET_COEX_STATUS:
        {
            coex_running = ((p_bt->thread_start << 2) | (p_bt->zigbee_start << 1) | p_bt->bt_start);
            if (copy_to_user((unsigned char __user *)arg, &coex_running, sizeof(unsigned char)) != 0)
            {
                BTE("IOCTL_GET_COEX_STATUS copy error\n");
                return -EFAULT;
            }
            BTI("IOCTL_GET_COEX_STATUS %#x\n", coex_running);
        }
        break;
    }
    return 0;
}

#ifdef CONFIG_COMPAT
static long amlbt_compat_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    ret = amlbt_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
    return ret;
}
#endif

static int amlbt_coex_open(struct inode *inode, struct file *file)
{
    BTI("%s \n", __func__);
    file->private_data = &amlbt_dev;
    return nonseekable_open(inode, file);
}

static int amlbt_coex_close(struct inode *inode, struct file *file)
{
    BTI("%s \n", __func__);

    return 0;
}

static int amlbt_zigbee_open(struct inode *inode, struct file *file)
{
    int ret = 0;
    BTI("%s,%d, version:%s\n", __func__, AML_W2LU_VERSION);

    file->private_data = &amlbt_dev;

    ret = amlbt_check_usb();
    if (ret != 0)
    {
        goto err_exit;
    }

    if (!amlbt_w2lu_coex_is_running(&amlbt_dev))
    {
        ret = amlbt_powersave_clear();
        if (ret != 0)
        {
            goto err_exit;
        }

        if (amlbt_res_init(&amlbt_dev) != 0)
        {
            BTI("amlbt_res_init failed!\n");
            goto err_buf;
        }
        //stop bt cpu
        ret = amlbt_sw_reset();
        if (ret != 0)
        {
            goto err_exit;
        }

        //amlbt_utils_w2l_usb_init(amlbt_read_word, amlbt_write_word, amlbt_read_sram, amlbt_write_sram);
        amlbt_load_conf(&amlbt_dev);
        ret = amlbt_load_firmware(&amlbt_dev);
        if (ret != 0)
        {
            BTI("amlbt_load_firmware failed!\n");
            goto err_buf;
        }
        ret = amlbt_write_word(REG_DEV_RESET, 0, USB_EP2);
        if (ret != 0)
        {
            BTI("start cpu failed!\n");
            goto err_buf;
        }
        ret = amlbt_show_fw_debug_info();
        if (ret != 0)
        {
            BTI("show fw failed!\n");
            goto err_buf;
        }
        amlbt_task_start(&amlbt_dev);
    }

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
    ret = amlbt_bind_bus();
    if (ret != 0)
    {
        goto err_buf;
    }
#endif
    amlbt_dev.zigbee_start = 1;
    return nonseekable_open(inode, file);
err_buf:
    amlbt_res_deinit(&amlbt_dev);
err_exit:
    return nonseekable_open(inode, file);
}

static int amlbt_zigbee_close(struct inode *inode, struct file *file)
{
    int ret = 0;
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)file->private_data;

    BTI("%s, version:%s, [%#x,%#x]\n", __func__, AML_W2LU_VERSION, p_bt->bt_start, p_bt->thread_start);

    if (!p_bt->bt_start && !p_bt->thread_start)
    {
        amlbt_show_fw_debug_info();
        p_bt->usb_irq_task_quit = 1;
        //wait_for_completion(&p_bt->comp);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
        amlbt_unbind_bus();
#endif
        //amlbt_utils_w2l_usb_deinit();
        ret = amlbt_aon_addr_bit_clr(RG_AON_A24, 26);//wake up firmware, make sure firmware running
        if (ret != 0)
        {
            amlbt_res_deinit(&amlbt_dev);
            return 0;
        }

        amlbt_res_deinit(&amlbt_dev);
        amlbt_write_word(RG_AON_A15, 0, USB_EP2); //set bt_en auto mode
    }
    p_bt->zigbee_start = 0;

    BTI("zigbee closed\n");
    return 0;
}

static ssize_t amlbt_zigbee_write(struct file *file_p,
                                     const char __user *buf_p,
                                     size_t count,
                                     loff_t *pos_p)
{
    static unsigned int w_type = 0;
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)file_p->private_data;
    struct sk_buff *skb;
    //unsigned char *p;

    BTA("%s, count:%ld\n", __func__, count);

    if (count > HCI_MAX_FRAME_SIZE)
    {
        BTE("%s:%d count > HCI_MAX_FRAME_SIZE %d, %d!\n", __func__, __LINE__, count, HCI_MAX_FRAME_SIZE);
        return -EINVAL;
    }

    if (count == 1) //host write hci type
    {
        get_user(w_type, buf_p);
        return count;
    }

    if (!p_bt->zigbee_start)
    {
        BTE("%s:%d p_bt->zigbee_start == 0!\n", __func__, __LINE__);
        return -EINVAL;
    }

    skb = alloc_skb(count+1, GFP_KERNEL);
    if (!skb)
    {
        return -ENOMEM;
    }

    *(unsigned char *)skb_put(skb, 1) = w_type;

    if (copy_from_user(skb_put(skb, count), buf_p, count))
    {
        kfree_skb(skb);
        BTE("%s: Failed to get data from user space\n", __func__);
        return -EFAULT;
    }
    //p = &skb->data[0];
    //if (w_type == HCI_15P4_PKT)
    //{
    //    BTI("write 15.4 %d, %d:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n", skb->len, count,
    //            p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
    //}

    skb_queue_tail(&p_bt->tx_queue, skb);
    schedule_work(&p_bt->write_work);

    return count;
}


static ssize_t amlbt_zigbee_read(struct file *file_p,
                                   char __user *buf_p,
                                   size_t count,
                                   loff_t *pos_p)
{
    unsigned char zigbee_hw_error[8] = {0x10, 0xfa, 0x42, 0x01, 0x00, 0x00, 0x00, 0x00};
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)file_p->private_data;
    static struct sk_buff *skb;

    if (!p_bt->zigbee_start)
    {
        BTE("%s:%d p_bt->zigbee_start == 0!\n", __func__, __LINE__);
        return -EFAULT;
    }

    switch (p_bt->zigbee_rd_state)
    {
        case HCI_RX_TYPE:   //read type
            if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
            {
                BTW("%s:%d BT_DRV_STATE_RECOVERY!\n", __func__, __LINE__);
                if (copy_to_user(buf_p, &zigbee_hw_error[0], count))
                {
                    BTE("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
                p_bt->zigbee_rd_state = HCI_RX_HEADER;
                return count;
            }

            skb = skb_dequeue(&p_bt->zigbee_rx_queue);
            if (!skb)
            {
                BTE("zb HCI_RX_TYPE no data!\n");
                return 0;
            }
            if (skb->len < count)
            {
                BTE("%s:%d Failed to copy data: %d, %d\n", __func__, __LINE__, skb->len, count);
                return -EFAULT;
            }
            if (copy_to_user(buf_p, skb->data, count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                kfree_skb(skb);
                p_bt->zigbee_rd_state = HCI_RX_TYPE;
                return -EFAULT;
            }
            skb_pull(skb, count);
            p_bt->zigbee_rd_state = HCI_RX_HEADER;
        break;
        case HCI_RX_HEADER: // read header
            if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
            {
                if (copy_to_user(buf_p, &zigbee_hw_error[1], count))
                {
                    BTE("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
                p_bt->zigbee_rd_state = HCI_RX_PAYLOAD;
                return count;
            }

            if (skb->len < count)
            {
                BTE("%s:%d Failed to copy data: %d, %d\n", __func__, __LINE__, skb->len, count);
                return -EFAULT;
            }
            if (copy_to_user(buf_p, skb->data, count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                kfree_skb(skb);
                p_bt->zigbee_rd_state = HCI_RX_TYPE;
                return -EFAULT;
            }
            skb_pull(skb, count);
            if (skb->len == 0)
            {
                kfree_skb(skb);
                p_bt->zigbee_rd_state = HCI_RX_TYPE;
            }
            else
            {
                p_bt->zigbee_rd_state = HCI_RX_PAYLOAD;
            }
        break;
        case HCI_RX_PAYLOAD:    //read payload
            if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
            {
                if (copy_to_user(buf_p, &zigbee_hw_error[5], count))
                {
                    BTE("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
                p_bt->zigbee_rd_state = HCI_RX_FATAL;
                p_bt->zigbee_start = 0;
                return count;
            }

            if (skb->len < count)
            {
                BTE("%s:%d Failed to copy data: %d, %d\n", __func__, __LINE__, skb->len, count);
                return -EFAULT;
            }
            if (copy_to_user(buf_p, skb->data, count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                kfree_skb(skb);
                p_bt->zigbee_rd_state = HCI_RX_TYPE;
                return -EFAULT;
            }
            kfree_skb(skb);
            p_bt->zigbee_rd_state = HCI_RX_TYPE;
            break;
        default:
            BTE("%s, evt_state error!!\n", __func__);
            break;
    }
    return count;
}

static unsigned int amlbt_zigbee_poll(struct file *file, poll_table *wait)
{
    int mask = 0;
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)file->private_data;

    poll_wait(file, &p_bt->zigbee_wait_queue, wait);

    if (!p_bt->zigbee_start)
    {
        mask |= POLLIN | POLLRDNORM;
        goto exit;
    }

    if ((p_bt->dr_state & BT_DRV_STATE_RECOVERY) && p_bt->zigbee_rd_state == HCI_RX_FATAL)
    {
        goto exit;
    }

    if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
    {
        mask |= POLLIN | POLLRDNORM;
        goto exit;
    }

    if ((g_udev == NULL) || bus_state_detect.bus_err || bus_state_detect.bus_reset_ongoing)
    {
        BTF("%s:%d usb error!, %#x,%#x\n", __func__, __LINE__, bus_state_detect.bus_err, bus_state_detect.bus_reset_ongoing);
        goto exit;
    }

    if (p_bt->zigbee_rd_state > HCI_RX_TYPE || skb_queue_len(&p_bt->zigbee_rx_queue) > 0)
    {
        mask |= POLLIN | POLLRDNORM;
        goto exit;
    }

exit:
    return mask;
}

static int amlbt_thread_open(struct inode *inode, struct file *file)
{
    int ret = 0;
    BTI("%s,%d, version:%s\n", __func__, AML_W2LU_VERSION);

    file->private_data = &amlbt_dev;

    ret = amlbt_check_usb();
    if (ret != 0)
    {
        goto err_exit;
    }

    if (!amlbt_w2lu_coex_is_running(&amlbt_dev))
    {
        ret = amlbt_powersave_clear();
        if (ret != 0)
        {
            goto err_exit;
        }

        if (amlbt_res_init(&amlbt_dev) != 0)
        {
            BTI("amlbt_res_init failed!\n");
            goto err_buf;
        }
        //stop bt cpu
        ret = amlbt_sw_reset();
        if (ret != 0)
        {
            goto err_exit;
        }

        //amlbt_utils_w2l_usb_init(amlbt_read_word, amlbt_write_word, amlbt_read_sram, amlbt_write_sram);
        amlbt_load_conf(&amlbt_dev);
        ret = amlbt_load_firmware(&amlbt_dev);
        if (ret != 0)
        {
            BTI("amlbt_load_firmware failed!\n");
            goto err_buf;
        }
        ret = amlbt_write_word(REG_DEV_RESET, 0, USB_EP2);
        if (ret != 0)
        {
            BTI("start cpu failed!\n");
            goto err_buf;
        }
        ret = amlbt_show_fw_debug_info();
        if (ret != 0)
        {
            BTI("show fw failed!\n");
            goto err_buf;
        }
        amlbt_task_start(&amlbt_dev);
    }

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
    ret = amlbt_bind_bus();
    if (ret != 0)
    {
        goto err_buf;
    }
#endif
    amlbt_dev.thread_start = 1;
    return nonseekable_open(inode, file);
err_buf:
    amlbt_res_deinit(&amlbt_dev);
err_exit:
    return nonseekable_open(inode, file);
}

static int amlbt_thread_close(struct inode *inode, struct file *file)
{
    int ret = 0;
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)file->private_data;

    BTI("%s, version:%s\n", __func__, AML_W2LU_VERSION);

    if (!p_bt->bt_start && !p_bt->zigbee_start)
    {
        amlbt_show_fw_debug_info();
        p_bt->usb_irq_task_quit = 1;
        //wait_for_completion(&p_bt->comp);
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 10, 0)
        amlbt_unbind_bus();
#endif
        //amlbt_utils_w2l_usb_deinit();
        ret = amlbt_aon_addr_bit_clr(RG_AON_A24, 26);//wake up firmware, make sure firmware running
        if (ret != 0)
        {
            amlbt_res_deinit(&amlbt_dev);
            return 0;
        }

        amlbt_res_deinit(&amlbt_dev);
        amlbt_write_word(RG_AON_A15, 0, USB_EP2); //set bt_en auto mode
    }
    p_bt->thread_start = 0;

    BTI("zigbee closed\n");
    return 0;
}

static ssize_t amlbt_thread_write(struct file *file_p,
                                     const char __user *buf_p,
                                     size_t count,
                                     loff_t *pos_p)
{
    static unsigned int w_type = 0;
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)file_p->private_data;
    struct sk_buff *skb;
    //unsigned char *p;

    BTA("%s, count:%ld\n", __func__, count);

    if (count > HCI_MAX_FRAME_SIZE)
    {
        BTE("%s:%d count > HCI_MAX_FRAME_SIZE %d, %d!\n", __func__, __LINE__, count, HCI_MAX_FRAME_SIZE);
        return -EINVAL;
    }

    if (count == 1) //host write hci type
    {
        get_user(w_type, buf_p);
        return count;
    }

    if (!p_bt->thread_start)
    {
        BTE("%s:%d p_bt->thread_start == 0!\n", __func__, __LINE__);
        return -EINVAL;
    }

    skb = alloc_skb(count+1, GFP_KERNEL);
    if (!skb)
    {
        return -ENOMEM;
    }

    *(unsigned char *)skb_put(skb, 1) = w_type;

    if (copy_from_user(skb_put(skb, count), buf_p, count))
    {
        kfree_skb(skb);
        BTE("%s: Failed to get data from user space\n", __func__);
        return -EFAULT;
    }
    //p = &skb->data[0];
    //if (w_type == HCI_15P4_PKT)
    //{
    //    BTI("write 15.4 %d, %d:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n", skb->len, count,
    //           p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
    //}

    skb_queue_tail(&p_bt->tx_queue, skb);
    schedule_work(&p_bt->write_work);

    return count;
}


static ssize_t amlbt_thread_read(struct file *file_p,
                                   char __user *buf_p,
                                   size_t count,
                                   loff_t *pos_p)
{
    unsigned char thread_hw_error[8] = {0x10, 0xfa, 0x42, 0x01, 0x00, 0x00, 0x00, 0x00};
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)file_p->private_data;

    static struct sk_buff *skb;

    if (!p_bt->thread_start)
    {
        BTE("%s:%d p_bt->zigbee_start == 0!\n", __func__, __LINE__);
        return -EFAULT;
    }

    BTD("rd %#x\n", p_bt->thread_rd_state);
    switch (p_bt->thread_rd_state)
    {
        case HCI_RX_TYPE:   //read type
            if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
            {
                BTW("%s:%d BT_DRV_STATE_RECOVERY!\n", __func__, __LINE__);
                if (copy_to_user(buf_p, &thread_hw_error[0], count))
                {
                    BTE("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
                p_bt->thread_rd_state = HCI_RX_HEADER;
                return count;
            }

            skb = skb_dequeue(&p_bt->thread_rx_queue);
            if (!skb)
            {
                BTE("zb HCI_RX_TYPE no data!\n");
                return 0;
            }
            if (skb->len < count)
            {
                BTE("%s:%d Failed to copy data: %d, %d\n", __func__, __LINE__, skb->len, count);
                return -EFAULT;
            }
            if (copy_to_user(buf_p, skb->data, count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                kfree_skb(skb);
                p_bt->thread_rd_state = HCI_RX_TYPE;
                return -EFAULT;
            }
            skb_pull(skb, count);
            p_bt->thread_rd_state = HCI_RX_HEADER;
        break;
        case HCI_RX_HEADER: // read header
            if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
            {
                if (copy_to_user(buf_p, &thread_hw_error[1], count))
                {
                    BTE("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
                p_bt->thread_rd_state = HCI_RX_PAYLOAD;
                return count;
            }

            if (skb->len < count)
            {
                BTE("%s:%d Failed to copy data: %d, %d\n", __func__, __LINE__, skb->len, count);
                return -EFAULT;
            }
            if (copy_to_user(buf_p, skb->data, count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                kfree_skb(skb);
                p_bt->thread_rd_state = HCI_RX_TYPE;
                return -EFAULT;
            }
            skb_pull(skb, count);
            if (skb->len == 0)
            {
                kfree_skb(skb);
                p_bt->thread_rd_state = HCI_RX_TYPE;
            }
            else
            {
                p_bt->thread_rd_state = HCI_RX_PAYLOAD;
            }
        break;
        case HCI_RX_PAYLOAD:    //read payload
            if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
            {
                if (copy_to_user(buf_p, &thread_hw_error[5], count))
                {
                    BTE("%s, copy_to_user error \n", __func__);
                    return -EFAULT;
                }
                p_bt->thread_rd_state = HCI_RX_FATAL;
                p_bt->thread_start = 0;
                return count;
            }
            if (skb->len < count)
            {
                BTE("%s:%d Failed to copy data: %d, %d\n", __func__, __LINE__, skb->len, count);
                return -EFAULT;
            }
            if (copy_to_user(buf_p, skb->data, count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                kfree_skb(skb);
                p_bt->thread_rd_state = HCI_RX_TYPE;
                return -EFAULT;
            }
            kfree_skb(skb);
            p_bt->thread_rd_state = HCI_RX_TYPE;
            break;
        default:
            BTE("%s, evt_state error!!\n", __func__);
            break;
    }
    return count;
}

static unsigned int amlbt_thread_poll(struct file *file, poll_table *wait)
{
    int mask = 0;
    w2l_usb_bt_new_t *p_bt = (w2l_usb_bt_new_t *)file->private_data;

    poll_wait(file, &p_bt->thread_wait_queue, wait);

    if (!p_bt->thread_start)
    {
        mask |= POLLIN | POLLRDNORM;
        goto exit;
    }

    if ((p_bt->dr_state & BT_DRV_STATE_RECOVERY) && p_bt->thread_rd_state == HCI_RX_FATAL)
    {
        goto exit;
    }

    if (p_bt->dr_state & BT_DRV_STATE_RECOVERY)
    {
        mask |= POLLIN | POLLRDNORM;
        goto exit;
    }

    if ((g_udev == NULL) || bus_state_detect.bus_err || bus_state_detect.bus_reset_ongoing)
    {
        BTF("%s:%d usb error!, %#x,%#x\n", __func__, __LINE__, bus_state_detect.bus_err, bus_state_detect.bus_reset_ongoing);
        goto exit;
    }
    if (p_bt->thread_rd_state > HCI_RX_TYPE || skb_queue_len(&p_bt->thread_rx_queue) > 0)
    {
        mask |= POLLIN | POLLRDNORM;
        goto exit;
    }

exit:
    return mask;
}

static void amlbt_write_work(struct work_struct *work)
{
    struct sk_buff *skb;
    w2l_usb_bt_new_t *p_bt = &amlbt_dev;
    unsigned char *p;
    int ret;
    unsigned int len = 0;


    if (amlbt_dev.dr_state != 0)
    {
        BTE("amlbt_write_work dr_state:%#x\n", amlbt_dev.dr_state);
        return ;
    }

restart:
    //BTI("amlbt_w2ls_uart_write_work \n");

    while ((skb = skb_dequeue(&p_bt->tx_queue))) {
        p = skb->data;
        skb_pull(skb, 1);
        if (*p == HCI_COMMAND_PKT)
        {
            p = skb->data;
            len = 3 + p[2];
            //BTI("hci cmd %d, %d:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]", skb->len, len,
            //    p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
            ret = amlbt_send_hci_cmd(p_bt, skb->data, len);
        }
        else if (*p == HCI_ACLDATA_PKT)
        {
            p = skb->data;
            //BTI("hci data %d, %d:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n", skb->len, len,
            //        p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
            ret = amlbt_send_hci_data(p_bt, skb->data, skb->len);
        }
        else if (*p == HCI_15P4_PKT)
        {
            p = skb->data;
            //BTI("hci iot %d, %d:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n", skb->len, len,
            //        p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
            ret = amlbt_send_15p4_data(p_bt, skb->data, skb->len);
        }
        else
        {
            BTE("type error!\n");
            BTE("raw %d, %d:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n", skb->len, len,
                    p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]);
        }
        //BTI("amlbt_w2ls_uart_dequeue skb->len %d \n", skb->len);
        kfree_skb(skb);
        //BTI("amlbt_w2ls_uart_write_work complete! \n");
        if (ret != 0)
        {
            BTF("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        }
    }

    if (!skb_queue_empty(&p_bt->tx_queue))
    {
        goto restart;
    }
}


int amlbt_w2lu_new_init(void)
{
    int ret = 0;
    struct platform_device *p_device = &amlbt_device;
    struct platform_driver *p_driver = &amlbt_driver;

    BTI("%s, version:%s", __func__, AML_W2LU_VERSION);

    ret = platform_device_register(p_device);
    if (ret)
    {
        dev_err(&p_device->dev, "platform_device_register failed!\n");
        return ret;
    }

    ret = platform_driver_register(p_driver);
    if (ret)
    {
        dev_err(&p_device->dev, "platform_driver_register failed!\n");
        return ret;
    }

    return ret;
}

void amlbt_w2lu_new_exit(void)
{
    struct platform_device *p_device = &amlbt_device;
    struct platform_driver *p_driver = &amlbt_driver;

    BTI("%s, log level:%d \n", __func__, g_dbg_level);

    platform_device_unregister(p_device);
    platform_driver_unregister(p_driver);
}

