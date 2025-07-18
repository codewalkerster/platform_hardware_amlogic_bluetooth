/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
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
#include <linux/compat.h>
#include <linux/io.h>
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
#include <linux/pm_wakeup.h>
#include <linux/amlogic/pm.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "amlbt.h"
#include "common.h"
#include "debug_dev.h"
#include "w2_usb_bt_new.h"


#define TOTAL_LEN               (2364)  //2048 + 316 bytes

#define DBG_USB_MEM_ADDR         0x00510000  //length:512 bytes
#define DBG_USB_EVENT_Q_ADDR     0x00514000  //length:2364 bytes
#define DBG_USB_CMD_Q_ADDR       0x00518000  //length:4096 bytes

#define DBG_RX_TYPE_ADDR         DBG_USB_EVENT_Q_ADDR + 0x3c
#define DBG_RX_TYPE_LEN          (256)
#define DBG_RX_TYPE_R            DBG_USB_EVENT_Q_ADDR
#define DBG_RX_TYPE_W            DBG_USB_EVENT_Q_ADDR + 0x20

#define DBG_EVT_ADDR             DBG_USB_EVENT_Q_ADDR + 0x13c
#define DBG_EVT_LEN              (2048)
#define DBG_EVT_R                DBG_USB_EVENT_Q_ADDR + 0x04
#define DBG_EVT_W                DBG_USB_EVENT_Q_ADDR + 0x24

#define DBG_CMD_ADDR             DBG_USB_CMD_Q_ADDR
#define DBG_CMD_LEN              (4096)
#define DBG_CMD_R                DBG_USB_MEM_ADDR
#define DBG_CMD_W                DBG_USB_MEM_ADDR + 0x04
//recovery dbg use
#define MAX_DBG_BUF              64
#define BT_DRV_STATE_RECOVERY    BIT(3)

static ws_inf p_ws_inf = NULL;
static rs_inf p_rs_inf = NULL;
static ww_inf p_ww_inf = NULL;
static rw_inf p_rw_inf = NULL;

typedef struct
{
    long long int regvalue;
    long long int regaddr;
    unsigned char ver_inf[100];
    unsigned char manf_inf[48];
    unsigned int manf_len;
    char hci_cmd[8];
    unsigned int hci_len;
    unsigned char hci_evt[8];
    unsigned char dbg_cmd_inf[240];
    unsigned int cmd_index;
    unsigned char dbg_evt_inf[320];
    unsigned int evt_index;
} debug_cmd_t;

typedef struct
{
    struct cdev cdev;
    int         major;
    struct class *class;
    struct device *device;
    debug_cmd_t _cmd;
    w2_usb_bt_new_t *w2_dev;
    struct dentry *debug_dir;
} debug_dev_t;

static debug_dev_t debug_dev = {0};

static char recy_dbg_buf[MAX_DBG_BUF] = {0};
static ssize_t recy_dbg_read(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t recy_dbg_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static unsigned int dbg_write_data_by_ep(gdsl_fifo_t *p_fifo, unsigned char *data, unsigned int len, unsigned int ep)
{
    int ret = 0;
    unsigned long offset = (unsigned long)p_fifo->w;

    BTA("%s len:%d\n", __func__, len);

    len = ((len + 3) & 0xFFFFFFFC);
    if (gdsl_fifo_remain(p_fifo) < len)
    {
        BTE("write data no space!!\n");
        return -1;
    }

    if (len < (p_fifo->size - offset))
    {
        ret = p_ws_inf(data, (unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr), len, ep);
        if (ret != 0)
        {
            BTE("%s:%d Failed : %d len %d\n", __func__, __LINE__, ret, len);
            return -1;
        }
        p_fifo->w = (unsigned char *)(((unsigned long)p_fifo->w + len) % p_fifo->size);
    }
    else
    {
        ret = p_ws_inf(data, (unsigned char *)((unsigned long)p_fifo->w + (unsigned long)p_fifo->base_addr),
            p_fifo->size - offset, ep);
        if (ret != 0)
        {
            BTE("%s:%d Failed : %d len %d\n", __func__, __LINE__, ret, len);
            return -1;
        }
        if ((len - (p_fifo->size - offset)) != 0)
        {
            ret = p_ws_inf(&data[p_fifo->size - offset], p_fifo->base_addr, (len - (p_fifo->size - offset)), ep);
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

static int amlbt_debug_open(struct inode *inode, struct file *filp)
{
    BTI("%s\n", __func__);
    filp->private_data = &debug_dev;
    return 0;
}


static int amlbt_debug_close(struct inode *inode, struct file *filp)
{
    BTI("%s\n", __func__);
    return 0;
}

int amlbt_debug_filter_event(unsigned char *evt_buf)
{
    if (evt_buf[1] == 0x3e && evt_buf[3] == 0x02)   //adv report
    {
        return 1;
    }
    if (evt_buf[1] == 0x3e && evt_buf[3] == 0x0d)   //extend adv report
    {
        return 1;
    }
    if (evt_buf[1] == 0x13 && evt_buf[3] == 0x1)   //number of complete
    {
        return 1;
    }
    if (evt_buf[1] == 0x05 && evt_buf[3] == 0x00) //disconnect complete
    {
        return 1;
    }
    if (evt_buf[1] == 0x62) //fw log
    {
        return 1;
    }
    if (evt_buf[1] == 0x02) //inquiry result
    {
        return 1;
    }
    if (evt_buf[1] == 0x22) //inquiry result with rssi
    {
        return 1;
    }
    if (evt_buf[1] == 0x2f) //enhanced inquiry result
    {
        return 1;
    }
    if (evt_buf[1] == 0xf && evt_buf[3] == 0x0)   //event status
    {
        return 1;
    }

    return 0;
}

void amlbt_debug_send_cmd(debug_dev_t *d_bt)
{
    gdsl_fifo_t read_fifo = {0};
    unsigned int reg = 0;
    unsigned int type_size = 0;
    unsigned int evt_size = 0;
    unsigned char read_reg[8] = {0};
    unsigned int read_len = 0;
    static unsigned char type_buff[DBG_RX_TYPE_LEN] = {0};
    static unsigned char fw_read_buff[DBG_EVT_LEN] = {0};
    static unsigned char sram_buf[TOTAL_LEN];
    unsigned int wait_cnt = 0;
    unsigned int val = 0;
    gdsl_fifo_t t_fifo;
    gdsl_fifo_t ro_fw_evt_fifo;
    gdsl_fifo_t ro_fw_type_fifo;

    if (FAMILY_TYPE_IS_W2(amlbt_if_type))
    {
        if (d_bt->w2_dev->usb_irq_task_quit)
        {
            BTE("bt close \n");
            return ;
        }
        mutex_lock(&d_bt->w2_dev->bt_debug_mutex);
        t_fifo = *d_bt->w2_dev->tx_cmd_fifo;
        ro_fw_evt_fifo = *d_bt->w2_dev->fw_evt_fifo;
        ro_fw_type_fifo = *d_bt->w2_dev->fw_type_fifo;
        //send cmd
        if (d_bt->w2_dev->tx_cmd_fifo == NULL)
        {
            BTE("%s: tx_cmd_fifo NULL!!!!\n", __func__);
            mutex_unlock(&d_bt->w2_dev->bt_debug_mutex);
            return ;
        }
        d_bt->_cmd.hci_len = ((d_bt->_cmd.hci_len + 3) & 0xFFFFFFFC);//Keep 4 bytes aligned
        if (d_bt->_cmd.hci_len > 8)
        {
            d_bt->_cmd.hci_len = 8;
        }

        memset(sram_buf, 0, sizeof(sram_buf));
        p_rw_inf(DBG_CMD_R, USB_EP2, &val);
        t_fifo.r = (unsigned char *)(unsigned long)val;

        dbg_write_data_by_ep(&t_fifo, d_bt->_cmd.hci_cmd, d_bt->_cmd.hci_len, USB_EP2);

        p_ww_inf(DBG_CMD_W, (unsigned long)t_fifo.w & 0xfff, USB_EP2);
        *d_bt->w2_dev->tx_cmd_fifo = t_fifo;
        BTP("len %#x:w %#lx, r %#lx\n", d_bt->_cmd.hci_len, (unsigned long)d_bt->w2_dev->tx_cmd_fifo->w, (unsigned long)d_bt->w2_dev->tx_cmd_fifo->r);

        //get data
        p_rs_inf(sram_buf, (unsigned char *)(unsigned long)DBG_USB_EVENT_Q_ADDR, TOTAL_LEN, USB_EP2);
        d_bt->w2_dev->fw_type_fifo->w = (unsigned char *)(unsigned long)((sram_buf[35]<<24)|(sram_buf[34]<<16)|(sram_buf[33]<<8)|sram_buf[32]);
        d_bt->w2_dev->fw_evt_fifo->w = (unsigned char *)(unsigned long)((sram_buf[39]<<24)|(sram_buf[38]<<16)|(sram_buf[37]<<8)|sram_buf[36]);

        while (wait_cnt < 20)
        {
            if (d_bt->w2_dev->fw_type_fifo->w != d_bt->w2_dev->fw_type_fifo->r
                                            && d_bt->w2_dev->fw_evt_fifo->w != d_bt->w2_dev->fw_evt_fifo->r)
            {
                //get type
                memset(type_buff, 0, sizeof(type_buff));
                read_fifo.base_addr = &sram_buf[DBG_RX_TYPE_ADDR - DBG_USB_EVENT_Q_ADDR];
                read_fifo.r = d_bt->w2_dev->fw_type_fifo->r;
                read_fifo.w = d_bt->w2_dev->fw_type_fifo->w;
                read_fifo.size = DBG_RX_TYPE_LEN;

                type_size = gdsl_fifo_get_data(&read_fifo, type_buff, sizeof(type_buff));
                if (type_buff[0] != HCI_ACLDATA_PKT && type_buff[0] != HCI_EVENT_PKT)
                {
                    BTE("%s:%d type error!\n", __func__, __LINE__);
                    BTE("fw_type_fifo->w:%#x fw_type_fifo->r:%#x", d_bt->w2_dev->fw_type_fifo->w, d_bt->w2_dev->fw_type_fifo->r);
                    BTE("fw_evt_fifo->w:%#x fw_evt_fifo->r:%#x", d_bt->w2_dev->fw_evt_fifo->w, d_bt->w2_dev->fw_evt_fifo->r);
                    mutex_unlock(&d_bt->w2_dev->bt_debug_mutex);
                    return ;
                }

                d_bt->w2_dev->fw_type_fifo->r = read_fifo.r;
                reg = (((unsigned int)(unsigned long)read_fifo.r) & 0xff);
                read_reg[0] = (reg & 0xff);
                read_reg[1] = ((reg >> 8) & 0xff);
                read_reg[2] = ((reg >> 16) & 0xff);
                read_reg[3] = ((reg >> 24) & 0xff);

                //get event
                read_fifo.base_addr = &sram_buf[DBG_EVT_ADDR - DBG_USB_EVENT_Q_ADDR];
                read_fifo.r = d_bt->w2_dev->fw_evt_fifo->r;
                read_fifo.w = d_bt->w2_dev->fw_evt_fifo->w;
                read_fifo.size = DBG_EVT_LEN;
                evt_size = gdsl_fifo_get_data(&read_fifo, fw_read_buff, sizeof(fw_read_buff));
                BTP("read buff %#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x", fw_read_buff[0],fw_read_buff[1],fw_read_buff[2],
                                                     fw_read_buff[3],fw_read_buff[4],fw_read_buff[5],fw_read_buff[6],fw_read_buff[7]);
                BTP("evt fifo:w %#lx, r %#lx\n", (unsigned long)d_bt->w2_dev->fw_evt_fifo->w, (unsigned long)d_bt->w2_dev->fw_evt_fifo->r);
                if (evt_size)
                {
                    d_bt->w2_dev->fw_evt_fifo->r = read_fifo.r;
                    memcpy(d_bt->_cmd.hci_evt, fw_read_buff, 4);
                    read_len = fw_read_buff[2];
                    read_len -= 1;
                    read_len = ((read_len + 3) & 0xFFFFFFFC);
                    if (read_len != 0)
                    {
                        memcpy(&d_bt->_cmd.hci_evt[4], &fw_read_buff[4], 4);
                    }
                    if (d_bt->_cmd.hci_evt[4] == d_bt->_cmd.hci_cmd[0] &&
                                            d_bt->_cmd.hci_evt[5] == d_bt->_cmd.hci_cmd[1])
                    {
                        reg = (((unsigned int)(unsigned long)d_bt->w2_dev->fw_evt_fifo->r) & 0x1fff);
                        read_reg[4] = (reg & 0xff);
                        read_reg[5] = ((reg >> 8) & 0xff);
                        read_reg[6] = ((reg >> 16) & 0xff);
                        read_reg[7] = ((reg >> 24) & 0xff);
                        p_ws_inf(&read_reg[0], (unsigned char *)(unsigned long)(DBG_RX_TYPE_R), sizeof(read_reg), USB_EP2);
                        BTI("receive event succeed \n");
                        mutex_unlock(&d_bt->w2_dev->bt_debug_mutex);
                        return ;
                    }
                    else
                    {
                        BTE("event not match \n");
                        *d_bt->w2_dev->fw_evt_fifo = ro_fw_evt_fifo;
                        *d_bt->w2_dev->fw_type_fifo = ro_fw_type_fifo;
                    }
                }
            }

            usleep_range(30000, 30000);
            p_rs_inf(sram_buf, (unsigned char *)(unsigned long)DBG_USB_EVENT_Q_ADDR, TOTAL_LEN, USB_EP2);
            d_bt->w2_dev->fw_type_fifo->w = (unsigned char *)(unsigned long)((sram_buf[35]<<24)|(sram_buf[34]<<16)|(sram_buf[33]<<8)|sram_buf[32]);
            d_bt->w2_dev->fw_evt_fifo->w = (unsigned char *)(unsigned long)((sram_buf[39]<<24)|(sram_buf[38]<<16)|(sram_buf[37]<<8)|sram_buf[36]);
            wait_cnt++;
        }
        mutex_unlock(&d_bt->w2_dev->bt_debug_mutex);
        BTE("timeout no data\n");
        BTE("evt: w:%#lx, r:%#lx\n", (unsigned long)d_bt->w2_dev->fw_evt_fifo->w, (unsigned long)d_bt->w2_dev->fw_evt_fifo->r);
        BTE("type: w:%#lx, r:%#lx\n", (unsigned long)d_bt->w2_dev->fw_evt_fifo->w, (unsigned long)d_bt->w2_dev->fw_evt_fifo->r);
    }

}

void amlbt_debug_get_cmd(unsigned int w, unsigned int r, unsigned char *data)
{
    unsigned char buf[12] = {0};
    int len = sizeof(buf);

    buf[0] = (w & 0xff);
    buf[1] = ((w >> 8) & 0xff);
    buf[2] = ((w >> 16) & 0xff);
    buf[3] = ((w >> 24) & 0xff);
    buf[4] = (r & 0xff);
    buf[5] = ((r >> 8) & 0xff);
    buf[6] = ((r >> 16) & 0xff);
    buf[7] = ((r >> 24) & 0xff);

    memcpy(&buf[8], data, len-8);
    memcpy(&debug_dev._cmd.dbg_cmd_inf[debug_dev._cmd.cmd_index*cmd_len], buf, len);

    debug_dev._cmd.cmd_index++;
    if (debug_dev._cmd.cmd_index % 20 == 0)
    {
        debug_dev._cmd.cmd_index = 0;
    }
}

void amlbt_debug_get_event(unsigned int w, unsigned int r, unsigned char *data)
{
    unsigned char buf[16] = {0};
    int len = sizeof(buf);

    buf[0] = (w & 0xff);
    buf[1] = ((w >> 8) & 0xff);
    buf[2] = ((w >> 16) & 0xff);
    buf[3] = ((w >> 24) & 0xff);
    buf[4] = (r & 0xff);
    buf[5] = ((r >> 8) & 0xff);
    buf[6] = ((r >> 16) & 0xff);
    buf[7] = ((r >> 24) & 0xff);

    memcpy(&buf[8], data, len/2);
    memcpy(&debug_dev._cmd.dbg_evt_inf[debug_dev._cmd.evt_index*evt_len], buf, len);

    debug_dev._cmd.evt_index++;

    if (debug_dev._cmd.evt_index % 20 == 0)
    {
        debug_dev._cmd.evt_index = 0;
    }
}

void amlbt_show_debug(void)
{
    int i = 1;
    unsigned int r = 0;
    unsigned int w = 0;
    unsigned int opcode = 0;
    unsigned int playload = 0;
    unsigned char *p = debug_dev._cmd.dbg_cmd_inf;
    unsigned int line = sizeof(debug_dev._cmd.dbg_cmd_inf)/cmd_len;

    printk(KERN_CONT "cmd debug:[\n");
    for (; i <= line; i++)
    {
        w = (unsigned int)((p[3]<<24)|(p[2]<<16)|(p[1]<<8)|p[0]);
        r = (unsigned int)((p[7]<<24)|(p[6]<<16)|(p[5]<<8)|p[4]);
        opcode = (unsigned int)((p[9]<<8)|p[8]);
        if (i == debug_dev._cmd.cmd_index)
        {
            printk(KERN_CONT "W:%#x R:%#x op_end:%#x \n", w, r, opcode);
        }
        else
        {
            printk(KERN_CONT "W:%#x R:%#x op:%#x \n", w, r, opcode);
        }
        p += cmd_len;
    }
    printk(KERN_CONT "]\n");
    BTI("cmd_index %d \n", (debug_dev._cmd.cmd_index-1));

    p = debug_dev._cmd.dbg_evt_inf;
    printk(KERN_CONT "event debug:[ \n");
    for (i = 1; i <= line; i++)
    {
        w = (unsigned int)((p[3]<<24)|(p[2]<<16)|(p[1]<<8)|p[0]);
        r = (unsigned int)((p[7]<<24)|(p[6]<<16)|(p[5]<<8)|p[4]);
        playload = (unsigned int)((p[13]<<8)|p[12]);
        if (i == debug_dev._cmd.evt_index)
        {
            printk(KERN_CONT "W:%#x R:%#x evt_end:%#x \n", w, r, playload);
        }
        else
        {
            printk(KERN_CONT "W:%#x R:%#x evt:%#x \n", w, r, playload);
        }
        p += evt_len;
    }
    printk(KERN_CONT "]\n");
    BTI("evt_index %d \n", (debug_dev._cmd.evt_index-1));
}

static long amlbt_debug_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    unsigned int reg_value = 0;
    debug_dev_t *d_bt = (debug_dev_t *)filp->private_data;

    switch (cmd)
    {
        case IOCTL_GET_BT_REG:
        {
            if (copy_from_user(&d_bt->_cmd , (void __user *)arg, sizeof(debug_cmd_t)) != 0)
            {
                BTE("IOCTL_GET_BT_REG copy error\n");
                return -EFAULT;
            }
            BTI("IOCTL_GET_BT_REG\n");
            BTI("reg addr %#x\n", d_bt->_cmd.regaddr);
            BTI("reg value %#x\n", d_bt->_cmd.regvalue);
            p_rw_inf(d_bt->_cmd.regaddr, USB_EP2, &reg_value);
            BTI("%#x %#x\n", d_bt->_cmd.regaddr, reg_value);
            d_bt->_cmd.regvalue = reg_value;
            if (copy_to_user((void __user *)arg, &d_bt->_cmd, sizeof(d_bt->_cmd)) != 0)
            {
                BTI("IOCTL_GET_BT_REG copy error\n");
                return -EFAULT;
            }
        }
        break;
        case IOCTL_SET_BT_REG:
        {
            if (copy_from_user(&d_bt->_cmd , (void __user *)arg, sizeof(debug_cmd_t)) != 0)
            {
                BTE("IOCTL_SET_BT_REG copy error\n");
                return -EFAULT;
            }
            BTI("IOCTL_SET_BT_REG\n");
            BTI("reg addr %#x\n", d_bt->_cmd.regaddr);
            BTI("reg value %#x\n", d_bt->_cmd.regvalue);
            p_ww_inf(d_bt->_cmd.regaddr, d_bt->_cmd.regvalue, USB_EP2);
            p_rw_inf(d_bt->_cmd.regaddr, USB_EP2, &reg_value);
            d_bt->_cmd.regvalue = reg_value;
            if (copy_to_user((void __user *)arg, &d_bt->_cmd, sizeof(d_bt->_cmd)) != 0)
            {
                BTI("IOCTL_SET_BT_REG copy error\n");
                return -EFAULT;
            }
        }
        break;
        case IOCTL_SET_HCI_CMD:
        {
            if (copy_from_user(&d_bt->_cmd , (void __user *)arg, sizeof(debug_cmd_t)) != 0)
            {
                BTE("IOCTL_SET_HCI_CMD copy error\n");
                return -EFAULT;
            }
            BTI("IOCTL_SET_HCI_CMD\n");

            amlbt_debug_send_cmd(d_bt);
            if (copy_to_user((void __user *)arg, &d_bt->_cmd, sizeof(d_bt->_cmd)) != 0)
            {
                BTI("IOCTL_SET_BT_REG copy error\n");
                return -EFAULT;
            }
        }
        break;
        case IOCTL_GET_BT_BUF:
        {
            BTI("IOCTL_GET_BT_BUF\n");
            amlbt_show_debug();
            if (copy_to_user((void __user *)arg, &d_bt->_cmd, sizeof(d_bt->_cmd)) != 0)
            {
                BTI("IOCTL_GET_BT_BUF copy error\n");
                return -EFAULT;
            }
        }
        break;
        case IOCTL_GET_BT_VERSION:
        {
            BTI("IOCTL_GET_BT_VERSION\n");

            if (FAMILY_TYPE_IS_W2(amlbt_if_type))
            {
                memcpy(d_bt->_cmd.ver_inf, AML_W2U_VERSION, strlen(AML_W2U_VERSION));
                memcpy(d_bt->_cmd.manf_inf, d_bt->w2_dev->rc_manfdata, d_bt->w2_dev->manfdata_len);
                d_bt->_cmd.manf_len = d_bt->w2_dev->manfdata_len;
            }

            //amlbt_show_debug();
            if (copy_to_user((void __user *)arg, &d_bt->_cmd, sizeof(d_bt->_cmd)) != 0)
            {
                BTI("IOCTL_GET_BT_VERSION copy error\n");
                return -EFAULT;
            }
        }
        break;
    }
    return 0;
}

static struct device_attribute recy_attr_dbg = {
    .attr = { .name = AML_BT_CHAR_RECYDBG_NAME, .mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH },
    .show = recy_dbg_read,
    .store = recy_dbg_write,
};

static ssize_t recy_dbg_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%s\n", recy_dbg_buf);
}

static ssize_t recy_dbg_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    BTI("recy_dbg_write count: %zu\n", count);

    if (count > sizeof(recy_dbg_buf) - 1)
    {
        count = sizeof(recy_dbg_buf) - 1;
    }
    memset(recy_dbg_buf, 0, sizeof(recy_dbg_buf));
    memcpy(recy_dbg_buf, buf, count);
    recy_dbg_buf[count] = '\0';

    if (strncmp(buf, "over", 4) == 0 || strncmp(buf, "over\n", 5) == 0)
    {
        BTI("%s bt recovery!\n", __func__);
        debug_dev.w2_dev->dr_state = BT_DRV_STATE_RECOVERY;
        debug_dev.w2_dev->recovery_value = BT_DRV_STATE_RECOVERY;
    }

    return count;
}

static int amlbt_recy_dbg_init(void)
{
    int res = 0;

    res = device_create_file(debug_dev.w2_dev->dev_device, &recy_attr_dbg);
    if (res)
    {
        BTE("%s:Failed to create device attribute\n", __func__);
    }
    return res;
}

static ssize_t amlbt_debug_level_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char temp[10] = {0};
    int len = snprintf(temp, sizeof(temp), "%d\n", g_dbg_level);
    return simple_read_from_buffer(buf, count, ppos, temp, len);
}

static ssize_t amlbt_debug_level_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char temp[10] = {0};
    int ret = 0, val = 0;

    if (count > sizeof(temp) - 1)
        return -EINVAL;

    if (copy_from_user(temp, buf, count))
        return -EFAULT;

    temp[count] = '\0';

    ret = kstrtoint(temp, 10, &val);
    if (ret)
    {
        pr_err("Invalid input for debug_level\n");
        return ret;
    }

    g_dbg_level = val;
    pr_info("Debug level set to %d\n", g_dbg_level);

    return count;
}

static const struct file_operations debug_level_fops =
{
    .read = amlbt_debug_level_read,
    .write = amlbt_debug_level_write,
};

static int amlbt_debug_level_init(void)
{
    debug_dev.debug_dir = debugfs_create_dir("aml_btz", NULL);
    if (debug_dev.debug_dir == NULL)
        return -ENOMEM;

    debugfs_create_file("aml_btz_dbg_lvl", 0644, debug_dev.debug_dir, NULL, &debug_level_fops);
    return 0;
}

static void amlbt_debug_level_deinit(void)
{
    if (debug_dev.debug_dir != NULL)
    {
        debugfs_remove_recursive(debug_dev.debug_dir);
        debug_dev.debug_dir = NULL;
    }
}

#ifdef CONFIG_COMPAT
static long amlbt_debug_compat_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    ret = amlbt_debug_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
    return ret;
}
#endif

static const struct file_operations amlbt_debug_fops =
{
    .open       = amlbt_debug_open,
    .release    = amlbt_debug_close,
    .write      = NULL,
    .read       = NULL,
    .unlocked_ioctl = amlbt_debug_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = amlbt_debug_compat_ioctl,
#endif
    .poll       = NULL,
    .fasync     = NULL
};

static int amlbt_create_debug_device(debug_dev_t *d_bt)
{
    int ret = 0;
    dev_t dev = 0;

    ret = alloc_chrdev_region(&dev, 0, 1,  AML_BT_CHAR_DEBUG_DEVICE);
    if (ret)
    {
        BTE("fail to allocate debug chrdev\n");
        return ret;
    }

    d_bt->major = MAJOR(dev);
    cdev_init(&d_bt->cdev, &amlbt_debug_fops);
    d_bt->cdev.owner = THIS_MODULE;
    ret = cdev_add(&d_bt->cdev, dev, 1);
    if (ret < 0)
    {
        BTE("Failed to add debug device\n");
        goto err_add;
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    d_bt->class = class_create(THIS_MODULE, AML_BT_CHAR_DEBUG_DEVICE);
#else
    d_bt->class = class_create(AML_BT_CHAR_DEBUG_DEVICE);
#endif

    if (IS_ERR(d_bt->class))
    {
        ret = PTR_ERR(d_bt->class);
        BTE("Failed to create debug device class,  error code(%ld)\n", ret);
        goto err_class;
    }

    d_bt->device = device_create(d_bt->class, NULL, dev, NULL, AML_BT_CHAR_DEBUG_DEVICE);
    if (IS_ERR(d_bt->device))
    {
        BTE("debug device create fail, error code(%ld)\n", PTR_ERR(d_bt->device));
        goto err_dev;
    }

    BTI("%s: BT_major %d\n", __func__, d_bt->major);
    BTI("%s: dev id %d\n", __func__, dev);

    return 0;

err_dev:
    class_destroy(d_bt->class);
    d_bt->class = NULL;
err_class:
    cdev_del(&d_bt->cdev);
err_add:
    unregister_chrdev_region(dev, 1);
    return -1;
}

static int amlbt_destroy_debug_device(debug_dev_t *d_bt)
{
    dev_t dev = MKDEV(d_bt->major, 0);

    BTI("%s dev id %d\n", __func__, dev);

    if (d_bt->device)
    {
        device_destroy(d_bt->class, dev);
        d_bt->device = NULL;
    }
    if (d_bt->class)
    {
        class_destroy(d_bt->class);
        d_bt->class = NULL;
    }
    cdev_del(&d_bt->cdev);

    unregister_chrdev_region(dev, 1);

    BTI("%s driver removed.\n", AML_BT_CHAR_DEBUG_DEVICE);
    return 0;
}

int amlbt_debug_dev_init(ws_inf p_ws_func, rs_inf p_rs_func, ww_inf p_ww_func, rw_inf p_rw_func, void *bt_dev)
{
    int res = 0;

    res = amlbt_create_debug_device(&debug_dev);
    if (res)
    {
        BTE("%s:Failed to create debug device\n", __func__);
    }
    else
    {
        p_ws_inf = p_ws_func;
        p_rs_inf = p_rs_func;
        p_ww_inf = p_ww_func;
        p_rw_inf = p_rw_func;

        if (FAMILY_TYPE_IS_W2(amlbt_if_type))
        {
            if (bt_dev != NULL)
            {
                debug_dev.w2_dev = (w2_usb_bt_new_t *)bt_dev;
            }
            else
            {
                BTE("%s: bt_dev is NULL\n", __func__);
                res = -ENOMEM;
            }
        }
    }
    res = amlbt_debug_level_init();
    if (res != 0)
    {
        BTE("%s:Failed to create debugfs_create_dir\n", __func__);
    }
    res = amlbt_recy_dbg_init();
    if (res)
    {
        BTE("%s:Failed to create recy debug device attribute\n", __func__);
    }
    return res;
}

void amlbt_debug_dev_deinit(void)
{
    device_remove_file(debug_dev.w2_dev->dev_device, &recy_attr_dbg);
    debug_dev.w2_dev = NULL;
    amlbt_debug_level_deinit();
    amlbt_destroy_debug_device(&debug_dev);
}

