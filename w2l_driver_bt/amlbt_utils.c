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
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/input.h>

#include "common.h"
#include "w2l_bt_entry.h"
#include "amlbt_utils.h"

#define TIMER_INTERVAL_MS  10000  // 定时器触发间隔 (10s)
#define W2L_USB_UTILS_SENSOR_PRINT_INTERVAL         30  //5 min
#define W2L_DF_REG_A188                             (0x00f062f0)
#define W2L_TS_CFG_REG1                             (0x00a04904)
#define W2L_TS_STAT0                                (0x00a04940)
#define CHIP_INTF_REG_BASE                          (0xf00000)
#define RG_AON_A61                                  (CHIP_INTF_REG_BASE + 0xf4)

#define W2L_USB_TEMP_SENSOR_THRES_HOLE  (0x1c50)

typedef union TS_CFG_REG1_FIELD
{
    unsigned int data;
    struct
    {
        unsigned int    REG_CH_SEL : 3;
        unsigned int    REG_DEM_EN : 1;
        unsigned int    REG_EN_IPTAT_GPIO : 1;
        unsigned int    REG_ENABLE : 1;
        unsigned int    REG_TS_OUT_CTRL : 1;
        unsigned int    REG_HCIC_MODE : 2;
        unsigned int    REG_TS_EN_VBG : 1;
        unsigned int    REG_TS_EN_VCM : 1;
        unsigned int    REG_TS_RESET_SD : 1;
        unsigned int    REG_TS_RESET_VBG : 1;
        unsigned int    CLR_HI_TEMP_STAT : 1;
        unsigned int    FAST_MODE : 1;
        unsigned int    TS_IRQ_EN : 1;
        unsigned int    CLR_IRQ : 8;
        unsigned int    irq_mask : 8;
    }b;
}TS_CFG_REG1_FIELD_T;

typedef union TS_STAT0_FIELD
{
    unsigned int data;
    struct
    {
        unsigned int    yout_d2 : 16;
        unsigned int    yvalid_d2 : 1;
        unsigned int    detected_hi_temp_r : 1;
        unsigned int    detect_hi_temp_cnt : 14;
    }b;
}TS_STAT0_FIELD_T;


static w2l_usb_utils_bt_t w2l_utils = {0};

static unsigned int amlbt_utils_w2l_usb_is_wifi_alive(w2l_usb_utils_bt_t *p_utils)
{
    int ret = 0;
    unsigned int reg = 0;

    ret = p_utils->read_word(W2L_DF_REG_A188, USB_EP2, &reg);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        return 1;
    }

    if (p_utils->print_cnt >= W2L_USB_UTILS_SENSOR_PRINT_INTERVAL)
    {
        BTI("wifi status:%#x\n", reg);
    }

    if ((reg & BIT(30) || reg & BIT(31)))
    {
        //BTI("wifi alive\n");
        return 1;
    }
    else
    {
        //BTI("wifi not alive:%#x\n", reg);
        return 0;
    }
}

static enum hrtimer_restart amlbt_utils_w2l_usb_timer_callback(struct hrtimer *timer)
{
    w2l_usb_utils_bt_t *p_utils = &w2l_utils;

    queue_work(p_utils->wq, &p_utils->work);

    hrtimer_forward_now(timer, ms_to_ktime(TIMER_INTERVAL_MS));
    return HRTIMER_RESTART;
}

static unsigned int amlbt_utils_w2l_usb_sensor_get_temperature(w2l_usb_utils_bt_t *p_utils)
{
    int ret = 0;
    unsigned int reg = 0;

    ret = p_utils->read_word(W2L_TS_STAT0, USB_EP2, &reg);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        return 0;
    }

    if (p_utils->print_cnt >= W2L_USB_UTILS_SENSOR_PRINT_INTERVAL)
    {
        BTI("W2L_TS_STAT0:%#x\n", reg);
    }

    if ((reg & 0xffff) < W2L_USB_TEMP_SENSOR_THRES_HOLE)
    {
        return 0;
    }

    return 1;
}

static void amlbt_utils_w2l_usb_work_handler(struct work_struct *work)
{
    int ret = 0;
    unsigned int reg = 0;
    w2l_usb_utils_bt_t *p_utils = &w2l_utils;

    p_utils->print_cnt++;

    if (!amlbt_utils_w2l_usb_is_wifi_alive(p_utils))
    {
        ret = p_utils->read_word(RG_AON_A61, USB_EP2, &reg);
        if (ret != 0)
        {
            BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
            p_utils->print_cnt = 0;
            return ;
        }
        if (amlbt_utils_w2l_usb_sensor_get_temperature(p_utils))
        {
            reg |=  (1 << 0);
        }
        else
        {
            reg &= ~(1 << 0);
        }

        ret = p_utils->write_word(RG_AON_A61, reg, USB_EP2);
        if (ret != 0)
        {
            BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
            p_utils->print_cnt = 0;
            return ;
        }
    }

    if (p_utils->print_cnt >= W2L_USB_UTILS_SENSOR_PRINT_INTERVAL)
    {
        p_utils->print_cnt = 0;
    }
}

static int amlbt_utils_w2l_usb_temp_sensor_cfg(w2l_usb_utils_bt_t *p_utils)
{
    int ret = 0;
    TS_CFG_REG1_FIELD_T ts_cfg_reg1;

    ret = p_utils->read_word(W2L_TS_CFG_REG1, USB_EP2, &ts_cfg_reg1.data);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        return -1;
    }

    ts_cfg_reg1.b.REG_CH_SEL = 3;
    ts_cfg_reg1.b.REG_DEM_EN = 1;
    ts_cfg_reg1.b.REG_ENABLE = 1;
    ts_cfg_reg1.b.REG_TS_OUT_CTRL = 1;
    ts_cfg_reg1.b.REG_HCIC_MODE = 1;
    ts_cfg_reg1.b.REG_TS_EN_VBG = 1;
    ts_cfg_reg1.b.REG_TS_EN_VCM = 1;

    ret = p_utils->write_word(W2L_TS_CFG_REG1, ts_cfg_reg1.data, USB_EP2);
    if (ret != 0)
    {
        BTE("%s:%d Failed : %d\n", __func__, __LINE__, ret);
        return -1;
    }
    return 0;
}

void amlbt_utils_w2l_usb_init(
    int (*read_word)(unsigned int addr, unsigned int ep, unsigned int *value),
    int (*write_word)(unsigned int addr,unsigned int data, unsigned int ep),
    int (*write_sram)(unsigned char *buf, unsigned char *sram_addr, unsigned int len, unsigned int ep),
    int (*read_sram)(unsigned char *buf,unsigned char *sram_addr, unsigned int len, unsigned int ep))
{
    w2l_usb_utils_bt_t *p_utils = &w2l_utils;

    BTI("%s\n", __func__);

    p_utils->read_word = read_word;
    p_utils->write_word = write_word;
    p_utils->write_sram = write_sram;
    p_utils->read_sram = read_sram;
    p_utils->print_cnt = 0;
    if (0 != amlbt_utils_w2l_usb_temp_sensor_cfg(p_utils))
    {
        BTE("%s failed!!\n", __func__);
        return ;
    }

    p_utils->wq = create_singlethread_workqueue("w2l_usb_utils_workqueue");
    if (!p_utils->wq)
    {
        BTE("%s:%d Failed\n", __func__, __LINE__);
        return ;
    }

    INIT_WORK(&p_utils->work, amlbt_utils_w2l_usb_work_handler);

    hrtimer_init(&p_utils->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    p_utils->hrtimer.function = amlbt_utils_w2l_usb_timer_callback;

    hrtimer_start(&p_utils->hrtimer, ms_to_ktime(TIMER_INTERVAL_MS), HRTIMER_MODE_REL);
    p_utils->utils_start = 1;
}

void amlbt_utils_w2l_usb_deinit(void)
{
    w2l_usb_utils_bt_t *p_utils = &w2l_utils;
    BTI("%s, utils_start:%d\n", __func__, p_utils->utils_start);

    if (p_utils->utils_start)
    {
        hrtimer_cancel(&p_utils->hrtimer);
        flush_workqueue(p_utils->wq);
        destroy_workqueue(p_utils->wq);
        p_utils->wq = NULL;
        p_utils->utils_start = 0;
    }
    p_utils->print_cnt = 0;
}

