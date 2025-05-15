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
#include <linux/interrupt.h>
#include <linux/pm_wakeup.h>
#include <linux/pm_wakeirq.h>
#include <linux/amlogic/pm.h>
#include <linux/completion.h>

#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/version.h>
#include <linux/tty.h>
#include <linux/skbuff.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)
#include <linux/unaligned/packed_struct.h>
#else
#include <asm/unaligned.h>
#endif

#include "common.h"
#include "w2l_bt_entry.h"
#include "w2l_sdio_bt.h"

#define AML_BT_NOTE "stpbt"
#define AML_ZIGBEE_NOTE                 "aml_zigbee"
#define AML_THREAD_NOTE                 "aml_thread"
#define AML_COEX_NOTE                   "aml_coex"

#define AML_BT_FIRMWARE_NAME        "w2l_bt_15p4_fw_uart.bin"
#define AML_BT_FIRMWARE_TXT_NAME    "w2l_bt_15p4_fw_uart.txt"

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

#define REG_DEV_RESET           0xf03058
#define REG_PMU_POWER_CFG       0xf03040
#define REG_RAM_PD_SHUTDWONW_SW 0xf03050
#define REG_FW_MODE             0xf000e0
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

#define W2L_DF_REG_A188                           (0x00f062f0)
#define W2L_RG_PMU_A16                            (0x00f02040)

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

extern struct aml_hif_sdio_ops g_hif_sdio_ops;
extern struct aml_pm_type g_wifi_pm;
extern unsigned char g_chip_function_ctrl;
extern struct aml_bus_state_detect bus_state_detect;

extern void aml_sdio_exit(void);
extern int  aml_sdio_init(void);
extern void aml_bus_state_detect_deinit(void);
extern void extern_wifi_set_enable(int is_on);
extern int register_bt_event_notifier(struct notifier_block *nb);
extern int unregister_bt_event_notifier(struct notifier_block *nb);
//extern unsigned char aml_wifi_detect_bt_status __attribute__((weak));

static w2l_sdio_bt_t sdio_bt = {0};
//static void amlbt_shutdown_func(void);

static int amlbt_sdio_probe(struct platform_device *dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int amlbt_sdio_remove(struct platform_device *dev);
#else
static void amlbt_sdio_remove(struct platform_device *dev);
#endif
static int amlbt_sdio_suspend(struct platform_device *dev, pm_message_t state);
static int amlbt_sdio_resume(struct platform_device *dev);
static void amlbt_sdio_shutdown(struct platform_device *dev);

#ifdef CONFIG_COMPAT
static long amlbt_sdio_compat_ioctl(struct file* filp, unsigned int cmd, unsigned long arg);
#endif
static long amlbt_sdio_ioctl(struct file* filp, unsigned int cmd, unsigned long arg);
static int amlbt_sdio_download_firmware(w2l_sdio_bt_t *p_sdio);
static void amlbt_wake_func(struct work_struct *work);

static unsigned int amlbt_w2ls_bt_fops_poll(struct file *file, poll_table *wait);
static int amlbt_w2ls_bt_fops_open(struct inode *inode, struct file *file);
static int amlbt_w2ls_bt_fops_close(struct inode *inode, struct file *file);
static ssize_t amlbt_w2ls_bt_fops_write(struct file *file_p, const char __user *buf_p, size_t count, loff_t *pos_p);
static ssize_t amlbt_w2ls_bt_fops_read(struct file *file_p, char __user *buf_p, size_t count, loff_t *pos_p);

static unsigned int amlbt_w2ls_zigbee_fops_poll(struct file *file, poll_table *wait);
static int amlbt_w2ls_zigbee_fops_open(struct inode *inode, struct file *file);
static int amlbt_w2ls_zigbee_fops_close(struct inode *inode, struct file *file);
static ssize_t amlbt_w2ls_zigbee_fops_write(struct file *file_p, const char __user *buf_p, size_t count, loff_t *pos_p);
static ssize_t amlbt_w2ls_zigbee_fops_read(struct file *file_p, char __user *buf_p, size_t count, loff_t *pos_p);

static unsigned int amlbt_w2ls_thread_fops_poll(struct file *file, poll_table *wait);
static int amlbt_w2ls_thread_fops_open(struct inode *inode, struct file *file);
static int amlbt_w2ls_thread_fops_close(struct inode *inode, struct file *file);
static ssize_t amlbt_w2ls_thread_fops_write(struct file *file_p, const char __user *buf_p, size_t count, loff_t *pos_p);
static ssize_t amlbt_w2ls_thread_fops_read(struct file *file_p, char __user *buf_p, size_t count, loff_t *pos_p);

static int amlbt_w2ls_coex_fops_open(struct inode *inode, struct file *file);
static int amlbt_w2ls_coex_fops_close(struct inode *inode, struct file *file);
static int amlbt_w2ls_uart_tx_wakeup(struct hci_uart *hu);
static struct tty_ldisc_ops amlbt_w2ls_uart_ldisc;
static void amlbt_w2ls_exception_func(struct work_struct *work);

static void amlbt_dev_release(struct device *dev)
{
    return;
}

static struct platform_device amlbt_sdio_device =
{
    .name    = "sdio_bt",
    .id      = -1,
    .dev     = {
        .release = &amlbt_dev_release,
    }
};

static struct platform_driver amlbt_sdio_driver =
{
    .probe = amlbt_sdio_probe,
    .remove = amlbt_sdio_remove,
    .suspend = amlbt_sdio_suspend,
    .resume = amlbt_sdio_resume,
    .shutdown = amlbt_sdio_shutdown,

    .driver = {
        .name = "sdio_bt",
        .owner = THIS_MODULE,
    },
};

static const struct file_operations amlbt_w2ls_coex_bt_fops =
{
    .open       = amlbt_w2ls_bt_fops_open,
    .release    = amlbt_w2ls_bt_fops_close,
    .write      = amlbt_w2ls_bt_fops_write,
    .read      = amlbt_w2ls_bt_fops_read,
    .unlocked_ioctl = amlbt_sdio_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = amlbt_sdio_compat_ioctl,
#endif
    .poll       = amlbt_w2ls_bt_fops_poll,
    .fasync     = NULL
};

static const struct file_operations amlbt_w2ls_coex_zigbee_fops =
{
    .open       = amlbt_w2ls_zigbee_fops_open,
    .release    = amlbt_w2ls_zigbee_fops_close,
    .write      = amlbt_w2ls_zigbee_fops_write,
    .read      = amlbt_w2ls_zigbee_fops_read,
    .unlocked_ioctl = amlbt_sdio_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = amlbt_sdio_compat_ioctl,
#endif
    .poll       = amlbt_w2ls_zigbee_fops_poll,
    .fasync     = NULL
};

static const struct file_operations amlbt_w2ls_coex_thread_fops =
{
    .open       = amlbt_w2ls_thread_fops_open,
    .release    = amlbt_w2ls_thread_fops_close,
    .write      = amlbt_w2ls_thread_fops_write,
    .read      = amlbt_w2ls_thread_fops_read,
    .unlocked_ioctl = amlbt_sdio_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = amlbt_sdio_compat_ioctl,
#endif
    .poll       = amlbt_w2ls_thread_fops_poll,
    .fasync     = NULL
};

static const struct file_operations amlbt_w2ls_coex_fops =
{
    .open       = amlbt_w2ls_coex_fops_open,
    .release    = amlbt_w2ls_coex_fops_close,
    .write      = NULL,
    .read      = NULL,
    .unlocked_ioctl = amlbt_sdio_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = amlbt_sdio_compat_ioctl,
#endif
    .poll       = NULL,
    .fasync     = NULL
};


static void amlbt_sdio_res_deinit(w2l_sdio_bt_t *p_sdio)
{
    BTI("%s p_sdio->irq %d\n", __func__, p_sdio->irq);
    p_sdio->irq = -1;
    p_sdio->irq_handle = 0;
    p_sdio->bt_start = 0;
    skb_queue_purge(&p_sdio->bt_tx_queue);
    skb_queue_purge(&p_sdio->bt_rx_queue);
    BTI("%s finished \n", __func__);
}

static int amlbt_sdio_res_init(w2l_sdio_bt_t *p_sdio)
{
    BTI("%s p_sdio->irq %d\n", __func__, p_sdio->irq);

    init_completion(&p_sdio->notify_comp);
    p_sdio->notify_trig = 0;
    p_sdio->antenna = 2;
    p_sdio->fw_mode = 1;
    p_sdio->bt_sink = 0;
    p_sdio->pin_mux = 0;
    p_sdio->br_digit_gain = 66;
    p_sdio->edr_digit_gain = 98;
    p_sdio->fw_log = 0;
    p_sdio->driver_log = 3;
    p_sdio->factory = 0;
    p_sdio->irq = -1;
    p_sdio->irq_handle = 0;
    init_waitqueue_head(&p_sdio->bt_wait_queue);
    skb_queue_head_init(&p_sdio->bt_tx_queue);
    skb_queue_head_init(&p_sdio->bt_rx_queue);
    return 0;
}

static void amlbt_w2ls_coex_zigbee_res_deinit(w2l_sdio_bt_t *p_sdio)
{
    BTI("%s \n", __func__);
    p_sdio->zigbee_start = 0;
    skb_queue_purge(&p_sdio->zigbee_tx_queue);
    skb_queue_purge(&p_sdio->zigbee_rx_queue);
}

static int amlbt_w2ls_coex_zigbee_res_init(w2l_sdio_bt_t *p_sdio)
{
    BTI("%s \n", __func__);
    p_sdio->zigbee_start = 0;
    init_waitqueue_head(&p_sdio->zigbee_wait_queue);
    skb_queue_head_init(&p_sdio->zigbee_tx_queue);
    skb_queue_head_init(&p_sdio->zigbee_rx_queue);
    return 0;
}

static void amlbt_w2ls_coex_thread_res_deinit(w2l_sdio_bt_t *p_sdio)
{
    BTI("%s \n", __func__);
    p_sdio->thread_start = 0;
    skb_queue_purge(&p_sdio->thread_tx_queue);
    skb_queue_purge(&p_sdio->thread_rx_queue);
}

static int amlbt_w2ls_coex_thread_res_init(w2l_sdio_bt_t *p_sdio)
{
    BTI("%s \n", __func__);
    p_sdio->thread_start = 0;
    init_waitqueue_head(&p_sdio->thread_wait_queue);
    skb_queue_head_init(&p_sdio->thread_tx_queue);
    skb_queue_head_init(&p_sdio->thread_rx_queue);
    return 0;
}


#if 0
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    unsigned long current_jiffies = 0;
    unsigned long time_diff_jiffies = 0;
    unsigned long time_diff_ms = 0;

    if (sdio_bt.irq_handle)
    {
        current_jiffies = jiffies;

        if (sdio_bt.last_jiffies != 0)
        {
            time_diff_jiffies = current_jiffies - sdio_bt.last_jiffies;
            time_diff_ms = jiffies_to_msecs(time_diff_jiffies);
        }

        BTI("irq %#x,%#x,%#x\n", time_diff_ms, current_jiffies, sdio_bt.last_jiffies);
        sdio_bt.last_jiffies = current_jiffies;

        if (time_diff_ms > 350 && time_diff_ms < 450)
        {
            schedule_work(&sdio_bt.wake_work);
            sdio_bt.last_jiffies = 0;
            sdio_bt.irq_handle = 0;
        }
    }
    return IRQ_HANDLED;
}
#else
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    if (sdio_bt.irq_handle)
    {
        //BTI("irq\n");
        schedule_work(&sdio_bt.wake_work);
    }
    return IRQ_HANDLED;
}
#endif

static int amlbt_register_interrupt_gpio(w2l_sdio_bt_t *p_bt)
{
    struct device_node *node;
    int gpio_num;
    int irq;
    int ret;

    BTI("%s %d\n", __func__, p_bt->irq);

    node = of_find_node_by_name(NULL, "aml_bt");
    if (!node)
    {
        BTE("Failed to find device node\n");
        return -1;
    }
    //BTI("%s find node success!\n", __func__);
    gpio_num = of_get_named_gpio(node, "btwakeup-gpios", 0);
    if (gpio_num < 0)
    {
        BTE("Failed to get GPIO\n");
        return -1;
    }
    //BTI("%s find gpio %d success!\n", __func__, gpio_num);
    irq = gpio_to_irq(gpio_num);
    if (irq < 0)
    {
        BTE("Failed to get IRQ for GPIO\n");
        gpio_free(gpio_num);
        return -1;
    }
    //BTI("%s find irq %d success!\n", __func__, irq);

    ret = request_irq(irq, gpio_irq_handler, IRQF_TRIGGER_FALLING, "usb_gpio_irq", p_bt->dev_device);
    if (ret)
    {
        pr_err("Failed to request IRQ %d: %d\n", irq, ret);
        gpio_free(gpio_num);
        return -1;
    }
    BTI("%s request irq %d success!\n", __func__, irq);
    p_bt->irq = irq;
    INIT_WORK(&sdio_bt.wake_work, amlbt_wake_func);
    return 0;
}

static void amlbt_unregister_interrupt_gpio(w2l_sdio_bt_t *p_bt)
{
    BTI("%s %d\n", __func__, p_bt->irq);
    if (p_bt->irq > 0)
    {
        cancel_work_sync(&p_bt->wake_work);
        free_irq(p_bt->irq, p_bt->dev_device);
        p_bt->irq = -1;
    }
}

static int amlbt_input_device_init(struct platform_device *pdev)
{
    int err;
    w2l_sdio_bt_t *p_sdio = &sdio_bt;
    p_sdio->input_dev = input_allocate_device();
    if (!p_sdio->input_dev)
    {
        BTF("input_allocate_device failed:");
        return -EINVAL;
    }
    set_bit(EV_KEY,  p_sdio->input_dev->evbit);
    set_bit(KEY_POWER, p_sdio->input_dev->keybit);

    p_sdio->input_dev->name = INPUT_NAME;
    p_sdio->input_dev->phys = INPUT_PHYS;
    p_sdio->input_dev->dev.parent = &pdev->dev;
    p_sdio->input_dev->id.bustype = BUS_ISA;
    p_sdio->input_dev->id.vendor = 0x0001;
    p_sdio->input_dev->id.product = 0x0001;
    p_sdio->input_dev->id.version = 0x0100;
    p_sdio->input_dev->rep[REP_DELAY] = 0xffffffff;
    p_sdio->input_dev->rep[REP_PERIOD] = 0xffffffff;
    p_sdio->input_dev->keycodesize = sizeof(unsigned short);
    p_sdio->input_dev->keycodemax = 0x1ff;
    err = input_register_device(p_sdio->input_dev);
    if (err < 0)
    {
        pr_err("input_register_device failed: %d\n", err);
        input_free_device(p_sdio->input_dev);
        return -EINVAL;
    }

    return err;
}

static void amlbt_input_device_deinit(void)
{
    w2l_sdio_bt_t *p_sdio = &sdio_bt;
    input_unregister_device(p_sdio->input_dev);
    p_sdio->input_dev = NULL;
}


static void amlbt_sdio_write_word(unsigned int addr, unsigned int data)
{
    if (g_hif_sdio_ops.bt_hi_write_word == NULL)
    {
        BTE("amlbt_sdio_write_word NULL");
        return ;
    }
    g_hif_sdio_ops.bt_hi_write_word(addr, data);
}

static unsigned int amlbt_sdio_read_word(unsigned int addr)
{
    unsigned int value = 0;
    if (g_hif_sdio_ops.bt_hi_read_word == NULL)
    {
        BTE("amlbt_sdio_read_word NULL");
        return 0;
    }
    value = g_hif_sdio_ops.bt_hi_read_word(addr);
    return value;
}

static void amlbt_sdio_read_sram(unsigned char* buf, unsigned char* addr, unsigned int len)
{
    if (g_hif_sdio_ops.hi_random_ram_read == NULL)
    {
        BTE("amlbt_sdio_read_sram NULL");
        return ;
    }
    g_hif_sdio_ops.hi_random_ram_read(buf, addr, len);
}

static void amlbt_sdio_write_sram(unsigned char* buf, unsigned char* addr, unsigned int len)
{
    if (g_hif_sdio_ops.hi_random_ram_write == NULL)
    {
        BTE("amlbt_sdio_write_sram NULL");
        return ;
    }
    g_hif_sdio_ops.hi_random_ram_write(buf, addr, len);
}

static void amlbt_wake_func(struct work_struct *work)
{
    w2l_sdio_bt_t *p_sdio = &sdio_bt;
    //unsigned int key = amlbt_sdio_read_word(RG_AON_A52);
    unsigned int key = amlbt_sdio_read_word(RG_AON_A17);

    //BTI("key %#x\n", key);
    if (key & BIT(5))   //bit 5 power key, bit 6 netfix
    {
        key &= ~BIT(5);
        amlbt_sdio_write_word(RG_AON_A17, key);
        input_event(p_sdio->input_dev, EV_KEY, KEY_POWER, 1);
        input_sync(p_sdio->input_dev);
        input_event(p_sdio->input_dev, EV_KEY, KEY_POWER, 0);
        input_sync(p_sdio->input_dev);
        sdio_bt.irq_handle = 0;
        BTI("%s input power key\n", __func__);
    }
}

static void amlbt_aon_addr_bit_set(unsigned int addr, unsigned int bit)
{
    unsigned int reg_value = 0;

    reg_value = amlbt_sdio_read_word(addr);
    BTI("%#x: %#x\n", addr, reg_value);
    reg_value |= BIT(bit);
    amlbt_sdio_write_word(addr, reg_value);
    BTI("%#x: %#x", addr, amlbt_sdio_read_word(addr));
}

static void amlbt_aon_addr_bit_clr(unsigned int addr, unsigned int bit)
{
    unsigned int reg_value = 0;

    reg_value = amlbt_sdio_read_word(addr);
    BTI("%#x: %#x\n", addr, reg_value);
    reg_value &= ~BIT(bit);
    amlbt_sdio_write_word(addr, reg_value);
    BTI("%#x: %#x", addr, amlbt_sdio_read_word(addr));
}

static unsigned int amlbt_aon_addr_bit_get(unsigned int addr, unsigned int bit)
{
    unsigned int reg_value = 0;
    unsigned int bit_value = 0;

    reg_value = amlbt_sdio_read_word(addr);

    bit_value = (reg_value >> bit) & 0x1;
    BTI("get %#x bit%#d: %#x\n", addr, bit, bit_value);

    return bit_value;
}

#if 0
static void amlbt_shutdown_func(void)
{
    BTI("%s \n", __func__);
    //amlbt_aon_addr_bit_set(RG_AON_A52, 27);
    //amlbt_aon_addr_bit_set(RG_AON_A16, 28);
    BTI("%s finished\n", __func__);
}
#endif

static int amlbt_sdio_create_device(w2l_sdio_bt_t *p_sdio)
{
    int ret = 0;
    int i = 0, j = 0;
    int cdevErr = 0;
    dev_t dev = 0;
    const char *device_names[AML_W2LS_MAX_COEX_DEVICES] =
        { AML_BT_NOTE, AML_ZIGBEE_NOTE, AML_THREAD_NOTE, AML_COEX_NOTE };

    BTI("%s \n", __func__);

    ret = alloc_chrdev_region(&dev, 0, AML_W2LS_MAX_COEX_DEVICES, AML_BT_NOTE);
    if (ret)
    {
        BTE("fail to allocate chrdev\n");
        return ret;
    }

    p_sdio->dev_major = MAJOR(dev);
    BTI("major number:%d\n", p_sdio->dev_major);

    i = 0;
    //bt node
    cdev_init(&p_sdio->dev_cdev[i], &amlbt_w2ls_coex_bt_fops);
    p_sdio->dev_cdev[i].owner = THIS_MODULE;

    cdevErr = cdev_add(&p_sdio->dev_cdev[i], MKDEV(p_sdio->dev_major, i), 1);
    if (cdevErr)
    {
        goto error_cdev;
    }

    i++;
    //zigbee node
    cdev_init(&p_sdio->dev_cdev[i], &amlbt_w2ls_coex_zigbee_fops);
    p_sdio->dev_cdev[i].owner = THIS_MODULE;

    ret = cdev_add(&p_sdio->dev_cdev[i], MKDEV(p_sdio->dev_major, i), 1);
    if (ret)
    {
        BTE("cdev_add failed for minor %d\n", i);
        goto error_cdev;
    }

    i++;
    //thread node
    cdev_init(&p_sdio->dev_cdev[i], &amlbt_w2ls_coex_thread_fops);
    p_sdio->dev_cdev[i].owner = THIS_MODULE;

    ret = cdev_add(&p_sdio->dev_cdev[i], MKDEV(p_sdio->dev_major, i), 1);
    if (ret)
    {
        BTE("cdev_add failed for minor %d\n", i);
        goto error_cdev;
    }

    i++;
    //coex node
    cdev_init(&p_sdio->dev_cdev[i], &amlbt_w2ls_coex_fops);
    p_sdio->dev_cdev[i].owner = THIS_MODULE;

    ret = cdev_add(&p_sdio->dev_cdev[i], MKDEV(p_sdio->dev_major, i), 1);
    if (ret)
    {
        BTE("cdev_add failed for minor %d\n", i);
        goto error_cdev;
    }

    BTI("driver (major %d) installed.\n", p_sdio->dev_major);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    p_sdio->dev_class = class_create(THIS_MODULE, AML_BT_NOTE);
#else
    p_sdio->dev_class = class_create(AML_BT_NOTE);
#endif

    if (IS_ERR(p_sdio->dev_class))
    {
        BTE("class create fail, error code(%ld)\n", PTR_ERR(p_sdio->dev_class));
        goto error_class;
    }

    for (i = 0; i < AML_W2LS_MAX_COEX_DEVICES; i++)
    {
        p_sdio->dev_device[i] = device_create(p_sdio->dev_class, NULL, MKDEV(p_sdio->dev_major, i), NULL, device_names[i]);
        if (IS_ERR(p_sdio->dev_device[i]))
        {
            BTE("device create fail for %s, error code(%ld)\n", device_names[i], PTR_ERR(p_sdio->dev_device[i]));
            goto error_device;
        }
    }

    BTI("Devices created success!\n");

    return 0;

error_device:
    j = i;
    while (--j >= 0)
    {
        device_destroy(p_sdio->dev_class, MKDEV(p_sdio->dev_major, j));
    }
    class_destroy(p_sdio->dev_class);

error_class:
    j = i;
error_cdev:
    while (--j >= 0)
    {
        cdev_del(&p_sdio->dev_cdev[j]);
    }
    unregister_chrdev_region(dev, AML_W2LS_MAX_COEX_DEVICES);

    return -1;
}

static int amlbt_sdio_destroy_device(w2l_sdio_bt_t *p_sdio)
{
    dev_t dev;
    int i;

    BTI("%s: Destroying devices\n", __func__);

    for (i = 0; i < AML_W2LS_MAX_COEX_DEVICES; i++)
    {
        dev = MKDEV(p_sdio->dev_major, i);
        if (p_sdio->dev_device[i])
        {
            device_destroy(p_sdio->dev_class, dev);
            p_sdio->dev_device[i] = NULL;
        }
    }

    if (p_sdio->dev_class)
    {
        class_destroy(p_sdio->dev_class);
        p_sdio->dev_class = NULL;
    }

    for (i = 0; i < AML_W2LS_MAX_COEX_DEVICES; i++)
    {
        cdev_del(&p_sdio->dev_cdev[i]);
    }

    unregister_chrdev_region(MKDEV(p_sdio->dev_major, 0), AML_W2LS_MAX_COEX_DEVICES);

    BTI("%s: Driver removed.\n", AML_BT_NOTE);
    return 0;
}

static void bt_earlysuspend(struct early_suspend *h)
{
    BTI("%s \n", __func__);
}

static void bt_lateresume(struct early_suspend *h)
{
    BTI("%s \n", __func__);

    //clear suspend bit
    //amlbt_aon_addr_bit_clr(RG_AON_A52, 26);
    amlbt_aon_addr_bit_clr(RG_AON_A24, 26);
    sdio_bt.irq_handle = 0;
}

static void amlbt_register_early_suspend(struct platform_device *dev)
{
    BTI("%s \n", __func__);

    sdio_bt.early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
    sdio_bt.early_suspend.suspend = bt_earlysuspend;
    sdio_bt.early_suspend.resume = bt_lateresume;
    sdio_bt.early_suspend.param = dev;
    register_early_suspend(&sdio_bt.early_suspend);
}

static void amlbt_unregister_early_suspend(struct platform_device *dev)
{
    BTI("%s \n", __func__);

    unregister_early_suspend(&sdio_bt.early_suspend);
}


static unsigned int amlbt_w2l_sdio_is_wifi_alive(void)
{
    unsigned int reg = 0;

    if (bus_state_detect.is_recy_ongoing)
    {
        BTI("wifi recovery ongoing! \n");
        return 1;
    }

    //reg = amlbt_sdio_read_word(W2L_DF_REG_A188);
    reg = amlbt_sdio_read_word(W2L_RG_PMU_A16);

    if ((reg & BIT(30) || reg & BIT(31)))
    {
        //wifi alive
        return 1;
    }
    else
    {
        //wifi not alive
        return 0;
    }
}

static int amlbt_sdio_probe(struct platform_device *dev)
{
    unsigned int alive = amlbt_w2l_sdio_is_wifi_alive();

    if (alive)
    {
        BTI("wifi alive\n");
    }
    else
    {
        BTI("wifi not alive\n");
        aml_sdio_init();
    }

    return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int amlbt_sdio_remove(struct platform_device *dev)
#else
static void amlbt_sdio_remove(struct platform_device *dev)
#endif
{
    unsigned int alive = amlbt_w2l_sdio_is_wifi_alive();

    if (alive)
    {
        BTI("wifi alive\n");
    }
    else
    {
        BTI("wifi not alive\n");
        BTI("aml_bus_state_detect_deinit\n");
        aml_bus_state_detect_deinit();
        BTI("remove wifi sdio device\n");
        aml_sdio_exit();
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
    return 0;
#endif
}

static int amlbt_sdio_suspend(struct platform_device *dev, pm_message_t state)
{
//    unsigned int reg = 0;
    w2l_sdio_bt_t *p_sdio = &sdio_bt;
    if (p_sdio->bt_start)
    {
#if 0
        if (p_sdio->fw_log)
        {
            //close fw log
            reg = amlbt_sdio_read_word(RG_AON_A59);
            BTI("close fw log before %#x\n", reg);
            reg &= ~(0x3);
            BTI("close fw log write %#x\n", reg);
            amlbt_sdio_write_word(RG_AON_A59, reg);
        }
#endif
        amlbt_aon_addr_bit_set(RG_AON_A24, 26);//set suspend bit
        amlbt_aon_addr_bit_clr(RG_AON_A24, 25);//allow fw sleep
        p_sdio->irq_handle = 0;
    }
    BTI("%s \n", __func__);
    return 0;
}

static int amlbt_sdio_resume(struct platform_device *dev)
{
    int wait_cnt = 0;
    unsigned int reg = 0;
    w2l_sdio_bt_t *p_sdio = &sdio_bt;

    if (p_sdio->bt_start)
    {
        //wait usb bus ready
        BTI("g_wifi_pm.bus_suspend_cnt:%#x\n", g_wifi_pm.bus_suspend_cnt);
#if 0
        if (p_sdio->fw_log)
        {
            //open fw log
            reg = amlbt_sdio_read_word(RG_AON_A59);
            BTI("open fw log before %#x\n", reg);
            reg |= (p_sdio->fw_log & 0x3);
            BTI("open fw log write %#x\n", reg);
            amlbt_sdio_write_word(RG_AON_A59, reg);
        }
#endif
        while (atomic_read(&g_wifi_pm.bus_suspend_cnt) != 0)
        {
            usleep_range(20000, 20000);
            if (wait_cnt++ > 100)
            {
                BTE("%s bus err\n", __func__);
                break;
            }
            BTI("g_wifi_pm.bus_suspend_cnt:%#x\n", g_wifi_pm.bus_suspend_cnt);
        }
        wait_cnt = 0;
        //forbid fw sleep
        amlbt_aon_addr_bit_set(RG_AON_A24, 25);

        // wake bt fw
        reg = amlbt_sdio_read_word(RG_BT_PMU_A15);
        BTI("bt pmu:%#x\n", reg);
        if (((reg & 0xF) == PMU_SLEEP_MODE) || ((reg & 0xF) == PMU_ACT_SLEEP))
        {
            reg = amlbt_sdio_read_word(RG_BT_PMU_A16);
            BTI("RG_BT_PMU_A16:%#x\n", reg);
            reg &= ~BIT(0);
            reg |= BIT(1);
            BTI("Write RG_BT_PMU_A16:%#x\n", reg);
            amlbt_sdio_write_word(RG_BT_PMU_A16, reg);
            reg = amlbt_sdio_read_word(RG_BT_PMU_A16);
            BTI("Read RG_BT_PMU_A16:%#x\n", reg);
        }
        // wait bt fw wake done
        while (amlbt_aon_addr_bit_get(RG_AON_A17, 29))
        {
            usleep_range(20000, 20000);
            if (wait_cnt++ > 100)
            {
                BTE("%s wake fw failed\n", __func__);
                break;
            }
        }
        BTI("get_resume_method %d, %d\n", get_resume_method(), BT_WAKEUP);
        if (get_resume_method() != BT_WAKEUP)
        {
            p_sdio->irq_handle = 1;
        }
    }
    BTI("%s\n", __func__);
    return 0;
}

static void amlbt_sdio_shutdown(struct platform_device *dev)
{
//    w2l_sdio_bt_t *p_sdio = &sdio_bt;
//    unsigned int reg = 0;

    BTI("%s \n", __func__);
#if 0
    if (p_sdio->fw_log)
    {
        //close fw log
        reg = amlbt_sdio_read_word(RG_AON_A59);
        BTI("close fw log before %#x\n", reg);
        reg &= ~(0x3);
        BTI("close fw log write %#x\n", reg);
        amlbt_sdio_write_word(RG_AON_A59, reg);
    }
#endif
    amlbt_sdio_write_word(RG_BT_PMU_A16, 0);
}

static int parse_int_value(char *start, const char *key, int *value)
{
    size_t key_len = strlen(key);
    if (strncmp(start, key, key_len) == 0 && start[key_len] == '=')
    {
        *value = simple_strtol(start + key_len + 1, NULL, 10);
        return 1;
    }
    return 0;
}

static int amlbt_load_conf(w2l_sdio_bt_t *p_bt)
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
    while (pos < len) {
        char *line_start = data + pos;
        char *line_end = strchr(line_start, '\n');  // Find end of line
        if (!line_end) {
            line_end = data + len;  // If no newline, this is the last line
        }

        *line_end = '\0';  // Null-terminate the current line

        // Parse known keys
        if (parse_int_value(line_start, "BtAntenna", &p_bt->antenna)) {
            BTI("Parsed BtAntenna: %d\n", p_bt->antenna);
        } else if (parse_int_value(line_start, "FirmwareMode", &p_bt->fw_mode)) {
            BTI("Parsed FirmwareMode: %d\n", p_bt->fw_mode);
        } else if (parse_int_value(line_start, "BtSink", &p_bt->bt_sink)) {
            BTI("Parsed BtSink: %d\n", p_bt->bt_sink);
        } else if (parse_int_value(line_start, "ChangePinMux", &p_bt->pin_mux)) {
            BTI("Parsed ChangePinMux: %d\n", p_bt->pin_mux);
        } else if (parse_int_value(line_start, "BrDigitGain", &p_bt->br_digit_gain)) {
            BTI("Parsed BrDigitGain: %d\n", p_bt->br_digit_gain);
        } else if (parse_int_value(line_start, "EdrDigitGain", &p_bt->edr_digit_gain)) {
            BTI("Parsed EdrDigitGain: %d\n", p_bt->edr_digit_gain);
        } else if (parse_int_value(line_start, "Btfwlog", &p_bt->fw_log)) {
            BTI("Parsed Btfwlog: %d\n", p_bt->fw_log);
        } else if (parse_int_value(line_start, "Btlog", &p_bt->driver_log)) {
            BTI("Parsed Btlog: %d\n", p_bt->driver_log);
        } else if (parse_int_value(line_start, "Btfactory", &p_bt->factory)) {
            BTI("Parsed Btfactory: %d\n", p_bt->factory);
        }/* else {
            BTI("Unknown key in configuration file: %s\n", line_start);
        }*/

        // Move to the next line
        pos = (line_end - data) + 1;
    }

    release_firmware(fw_entry);
    return 0;
}

#if 0
static int amlbt_load_firmware(w2l_sdio_bt_t *p_bt)
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

    BTI("Firmware loaded successfully, iccm_size: %#x, dccm_size:%#x\n",iccm_size - ICCM_ROM_SIZE, dccm_size);

    p_bt->iccm_buf = &firmware_data[ICCM_ROM_SIZE + 8];
    p_bt->dccm_buf = &firmware_data[iccm_size + 8];

    ret = amlbt_sdio_download_firmware(p_bt);

    release_firmware(fw_entry);
    if (ret != 0)
    {
        BTE("Download firmware failed!!\n");
        kfree(firmware_data);
        return ret;
    }
    amlbt_sdio_write_word(REG_FW_MODE, p_bt->fw_mode);
    amlbt_sdio_write_word(REG_PMU_POWER_CFG, (p_bt->antenna << BIT_RF_NUM)|(p_bt->bt_sink << BT_SINK_MODE));
    reg |= ((p_bt->pin_mux << 20) | (p_bt->factory << 21));
    reg |= (((p_bt->edr_digit_gain & 0xff) << 8) | (p_bt->br_digit_gain & 0xff));
    amlbt_sdio_write_word(RG_AON_A53, reg);
    reg = (p_bt->fw_log & 0x3);
    amlbt_sdio_write_word(RG_AON_A59, reg);
    amlbt_sdio_write_word(REG_DEV_RESET, 0);
    p_bt->bt_start = 1;
    p_bt->iccm_buf = NULL;
    p_bt->dccm_buf = NULL;
    kfree(firmware_data);
    return 0;
}
#else
static int amlbt_load_firmware(w2l_sdio_bt_t *p_bt)
{
    int ret = 0;
    unsigned int reg = 0;
    const struct firmware *fw_entry = NULL;
    unsigned int iccm_size;
    unsigned int dccm_size;

    BTI("Firmware load:%s\n", AML_BT_FIRMWARE_NAME);
    ret = request_firmware(&fw_entry, AML_BT_FIRMWARE_NAME, p_bt->dev_device[0]);
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

    BTI("pmu %#x\n",amlbt_sdio_read_word(0xf02078));
    amlbt_sdio_write_word(REG_DEV_RESET, ((BIT_PHY|BIT_MAC|BIT_CPU)<<16)|(BIT_PHY|BIT_MAC|BIT_CPU));
    usleep_range(1000, 1000);
    amlbt_sdio_write_word(REG_DEV_RESET, (BIT_CPU<<16)|BIT_CPU);
    BTI("hold bt cpu\n");
    ret = amlbt_sdio_download_firmware(p_bt);
    release_firmware(fw_entry);
    if (ret != 0)
    {
        BTE("Download firmware failed!!\n");
        return ret;
    }
    reg = amlbt_sdio_read_word(REG_FW_MODE);
    reg |= (p_bt->fw_mode & 0x3);
    amlbt_sdio_write_word(REG_FW_MODE, reg);

    reg = amlbt_sdio_read_word(REG_PMU_POWER_CFG);
    reg |= ((p_bt->antenna << BIT_RF_NUM)|(p_bt->bt_sink << BT_SINK_MODE));
    amlbt_sdio_write_word(REG_PMU_POWER_CFG, reg);

    reg = amlbt_sdio_read_word(RG_AON_A53);
    reg |= ((p_bt->pin_mux << 20) | (p_bt->factory << 21));
    reg |= (((p_bt->edr_digit_gain & 0xff) << 8) | (p_bt->br_digit_gain & 0xff));
    amlbt_sdio_write_word(RG_AON_A53, reg);

    reg = amlbt_sdio_read_word(RG_AON_A59);
    reg |= (p_bt->fw_log & 0x3);
    amlbt_sdio_write_word(RG_AON_A59, reg);
    amlbt_sdio_write_word(REG_DEV_RESET, 0);
    BTI("start bt cpu ok!\n");
    usleep_range(50000, 50000);
    BTI("pc1:%#x\n", amlbt_sdio_read_word(0x200034));
    usleep_range(10000, 10000);
    BTI("pc2:%#x\n", amlbt_sdio_read_word(0x200034));
    usleep_range(10000, 10000);
    BTI("pc3:%#x\n", amlbt_sdio_read_word(0x200034));
    //p_bt->bt_start = 1;
    p_bt->iccm_buf = NULL;
    p_bt->dccm_buf = NULL;
    return 0;
}
#endif

#if 0
static int bt_event_handler(struct notifier_block *nb, unsigned long event, void *data)
{
    reinit_completion(&sdio_bt.notify_comp);
    sdio_bt.notify_trig = 1;
    BTI("Consumer received BT event: %lu\n", event);

    amlbt_aon_addr_bit_set(RG_AON_A55, 29);

    amlbt_load_firmware(&sdio_bt);
    complete(&sdio_bt.notify_comp);
    return NOTIFY_OK;
}
#else
static int bt_event_handler(struct notifier_block *nb, unsigned long event, void *data)
{
    w2l_sdio_bt_t *p_sdio = &sdio_bt;
    struct sk_buff      *skb;
    unsigned char bt_hw_error[5] = {0x04, 0x10, 0x01, 0x00, 0x00};
    unsigned char zigbee_hw_error[8] = {0x10, 0xfa, 0x42, 0x01, 0x00, 0x00, 0x00, 0x00};
    unsigned char thread_hw_error[8] = {0x10, 0xfa, 0x42, 0x01, 0x00, 0x00, 0x00, 0x00};

    BTF("Consumer received BT event: %lu, [%#x,%#x,%#x]\n", event,
        p_sdio->bt_start, p_sdio->zigbee_start, p_sdio->thread_start);
    if (p_sdio->bt_start || p_sdio->zigbee_start || p_sdio->thread_start)
    {
        if (p_sdio->bt_start)
        {
            reinit_completion(&p_sdio->notify_comp);
            p_sdio->notify_trig = 1;
            skb_queue_purge(&p_sdio->bt_tx_queue);
            skb_queue_purge(&p_sdio->bt_rx_queue);
        }
        if (p_sdio->zigbee_start)
        {
            skb_queue_purge(&p_sdio->zigbee_tx_queue);
            skb_queue_purge(&p_sdio->zigbee_rx_queue);
        }
        if (p_sdio->thread_start)
        {
            skb_queue_purge(&p_sdio->thread_tx_queue);
            skb_queue_purge(&p_sdio->thread_rx_queue);
        }

        //amlbt_aon_addr_bit_set(RG_AON_A55, 29);
        amlbt_load_firmware(&sdio_bt);

        if (p_sdio->bt_start)
        {
            skb = alloc_skb(sizeof(bt_hw_error), GFP_ATOMIC);
            if (!skb)
            {
                BTF("bt skb error!!\n");
                return  NOTIFY_OK;
            }
            skb_put_data(skb, bt_hw_error, sizeof(bt_hw_error));
            BTF("Report bt hw error!\n");
            skb_queue_tail(&p_sdio->bt_rx_queue, skb);
            wake_up_interruptible(&p_sdio->bt_wait_queue);
        }

        if (p_sdio->zigbee_start)
        {
            skb = alloc_skb(sizeof(zigbee_hw_error), GFP_ATOMIC);
            if (!skb)
            {
                BTF("zigbee skb error!!\n");
                return NOTIFY_OK;
            }
            skb_put_data(skb, zigbee_hw_error, sizeof(zigbee_hw_error));
            BTF("Report zigbee hw error!\n");
            skb_queue_tail(&p_sdio->zigbee_rx_queue, skb);
            wake_up_interruptible(&p_sdio->zigbee_wait_queue);
        }

        if (p_sdio->thread_start)
        {
            skb = alloc_skb(sizeof(thread_hw_error), GFP_ATOMIC);
            if (!skb)
            {
                BTF("thread skb error!!\n");
                return NOTIFY_OK;
            }
            skb_put_data(skb, thread_hw_error, sizeof(thread_hw_error));
            BTF("Report thread hw error!\n");
            skb_queue_tail(&p_sdio->thread_rx_queue, skb);
            wake_up_interruptible(&p_sdio->thread_wait_queue);
        }
    }
    BTF("Coex driver excepion finish!\n");
    if (p_sdio->bt_start)
    {
        complete(&p_sdio->notify_comp);
    }
    return NOTIFY_OK;
}
#endif

static struct notifier_block bt_nb = {
    .notifier_call = bt_event_handler,
};

static int amlbt_sdio_download_firmware(w2l_sdio_bt_t *p_sdio)
{
    unsigned int offset = 0;
    unsigned int remain_len = 0;
    unsigned int iccm_base_addr = BT_ICCM_AHB_BASE + ICCM_ROM_SIZE;
    unsigned int dccm_base_addr = BT_DCCM_AHB_BASE;
    uint8_t *check_buf = kzalloc(DOWNLOAD_SIZE, GFP_DMA|GFP_ATOMIC);
    int ret = 0;

    if (check_buf == NULL)
    {
        BTF("amlbt_sdio_download_firmware check_buf alloc failed!!!\n");
        return -1;
    }

    memset(check_buf, 0, DOWNLOAD_SIZE);
    remain_len = ICCM_SIZE;

    //to do download bt fw
    BTI("amlbt_sdio_download_firmware:iccm size %#x, remain_len %#x\n", ICCM_SIZE, remain_len);

    BTI("iccm start [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
        p_sdio->iccm_buf[0], p_sdio->iccm_buf[1], p_sdio->iccm_buf[2], p_sdio->iccm_buf[3],
        p_sdio->iccm_buf[4], p_sdio->iccm_buf[5], p_sdio->iccm_buf[6], p_sdio->iccm_buf[7]);
    BTI("dccm start [%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]\n",
        p_sdio->dccm_buf[0], p_sdio->dccm_buf[1], p_sdio->dccm_buf[2], p_sdio->dccm_buf[3],
        p_sdio->dccm_buf[4], p_sdio->dccm_buf[5], p_sdio->dccm_buf[6], p_sdio->dccm_buf[7]);

    while (offset < ICCM_SIZE)
    {
        if (remain_len < DOWNLOAD_SIZE)
        {
            BTD("bt_usb_download_firmware iccm1 offset %#x, addr %#x\n", offset, iccm_base_addr);
            amlbt_sdio_write_sram((unsigned char *)&p_sdio->iccm_buf[offset], (unsigned char *)(unsigned long)iccm_base_addr, remain_len);
            amlbt_sdio_read_sram(check_buf, (unsigned char *)(unsigned long)iccm_base_addr, remain_len);
            if (memcmp(check_buf, &p_sdio->iccm_buf[offset], remain_len))
            {
                BTE("Firmware iccm check2 error! offset %#x\n", offset);
                ret = -1;
                goto error;
            }
            offset += remain_len;
            iccm_base_addr += remain_len;
            BTD("amlbt_sdio_download_firmware iccm1 offset %#x, write_len %#x\n", offset, remain_len);
        }
        else
        {
            BTD("amlbt_sdio_download_firmware iccm2 offset %#x, write_len %#x, addr %#x\n", offset, DOWNLOAD_SIZE, iccm_base_addr);
            amlbt_sdio_write_sram((unsigned char *)&p_sdio->iccm_buf[offset], (unsigned char *)(unsigned long)iccm_base_addr, DOWNLOAD_SIZE);
            amlbt_sdio_read_sram(check_buf, (unsigned char *)(unsigned long)iccm_base_addr, DOWNLOAD_SIZE);
            if (memcmp(check_buf, &p_sdio->iccm_buf[offset], DOWNLOAD_SIZE))
            {
                BTE("Firmware iccm check error! offset %#x\n", offset);
                ret = -1;
                goto error;
            }
            offset += DOWNLOAD_SIZE;
            remain_len -= DOWNLOAD_SIZE;
            iccm_base_addr += DOWNLOAD_SIZE;
        }
        BTD("amlbt_sdio_download_firmware iccm remain_len %#x\n", remain_len);
    }

    BTI("Firmware iccm check pass, offset %#x\n", offset);
    offset = 0;
    remain_len = DCCM_SIZE;
    //to do download bt fw
    BTI("amlbt_sdio_download_firmware:dccm size %#x, remain_len %#x\n", DCCM_SIZE, remain_len);
    while (offset < DCCM_SIZE)
    {
        if (remain_len < DOWNLOAD_SIZE)
        {
            BTD("bt_usb_download_firmware dccm1 offset %#x, addr %#x\n", offset, dccm_base_addr);
            amlbt_sdio_write_sram((unsigned char *)&p_sdio->dccm_buf[offset], (unsigned char *)(unsigned long)dccm_base_addr, remain_len);
            amlbt_sdio_read_sram(check_buf, (unsigned char *)(unsigned long)dccm_base_addr, remain_len);
            if (memcmp(check_buf, &p_sdio->dccm_buf[offset], remain_len))
            {
                BTE("Firmware dccm check2 error! offset %#x\n", offset);
                ret = -1;
                goto error;
            }
            offset += remain_len;
            dccm_base_addr += remain_len;
            BTD("amlbt_sdio_download_firmware dccm1 offset %#x, write_len %#x\n", offset, remain_len);
        }
        else
        {
            BTD("amlbt_sdio_download_firmware dccm2 offset %#x, write_len %#x, addr%#x\n", offset, DOWNLOAD_SIZE, dccm_base_addr);
            amlbt_sdio_write_sram((unsigned char *)&p_sdio->dccm_buf[offset], (unsigned char *)(unsigned long)dccm_base_addr, DOWNLOAD_SIZE);
            amlbt_sdio_read_sram(check_buf, (unsigned char *)(unsigned long)dccm_base_addr, DOWNLOAD_SIZE);
            if (memcmp(check_buf, &p_sdio->dccm_buf[offset], DOWNLOAD_SIZE))
            {
                BTE("Firmware dccm check error! offset %#x\n", offset);
                ret = -1;
                goto error;
            }
            offset += DOWNLOAD_SIZE;
            remain_len -= DOWNLOAD_SIZE;
            dccm_base_addr += DOWNLOAD_SIZE;
        }
        BTD("amlbt_sdio_download_firmware dccm remain_len %#x \n", remain_len);
    }
    BTI("Firmware dccm check pass, offset %#x\n", offset);
error:
    kfree(check_buf);
    return ret;
}

static long amlbt_sdio_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    unsigned char coex_running = 0;
    w2l_sdio_bt_t *p_bt = (w2l_sdio_bt_t *)filp->private_data;

    BTI("arg value %ld", arg);
    BTI("cmd type=%c\t nr=%d\t dir=%d\t size=%d\n", _IOC_TYPE(cmd), _IOC_NR(cmd), _IOC_DIR(cmd), _IOC_SIZE(cmd));
    BTI("cmd value %ld", cmd);
    switch (cmd)
    {
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
static long amlbt_sdio_compat_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    ret = amlbt_sdio_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
    return ret;
}
#endif


int amlbt_w2ls_init(void)
{
    int ret = 0;
    struct platform_device *p_device = &amlbt_sdio_device;
    struct platform_driver *p_driver = &amlbt_sdio_driver;

    BTI("%s, version:%s", __func__, AML_W2LS_VERSION);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 14, 0)
    ret = tty_register_ldisc(&amlbt_w2ls_uart_ldisc);
#else
    ret = tty_register_ldisc(N_HCI, &amlbt_w2ls_uart_ldisc);

#endif
    if (ret) {
        dev_err(&p_device->dev, "platform_driver_register failed!\n");
        return ret;
    }

    ret = platform_driver_register(p_driver);
    if (ret) {
        dev_err(&p_device->dev, "platform_driver_register failed!\n");
        return ret;
    }

    ret = platform_device_register(p_device);
    if (ret) {
        dev_err(&p_device->dev, "platform_device_register failed!\n");
        platform_driver_unregister(p_driver);
        return ret;
    }

    amlbt_sdio_create_device(&sdio_bt);
    amlbt_register_early_suspend(p_device);
    amlbt_input_device_init(p_device);

    register_bt_event_notifier(&bt_nb);
    INIT_WORK(&sdio_bt.exception_work, amlbt_w2ls_exception_func);

    return 0;
}


void amlbt_w2ls_exit(void)
{
    struct platform_device *p_device = &amlbt_sdio_device;
    struct platform_driver *p_driver = &amlbt_sdio_driver;

    BTI("%s, log level:%d \n", __func__, g_dbg_level);

    g_bt_shutdown_func = NULL;

    unregister_bt_event_notifier(&bt_nb);

    amlbt_input_device_deinit();
    amlbt_unregister_early_suspend(p_device);
    amlbt_sdio_destroy_device(&sdio_bt);

    platform_device_unregister(p_device);
    platform_driver_unregister(p_driver);
}

static unsigned int amlbt_w2ls_coex_is_running(w2l_sdio_bt_t *p_bt)
{
    BTI("%s, %#x, %#x, %#x\n", __func__, sdio_bt.bt_start, sdio_bt.zigbee_start, sdio_bt.thread_start);

    if (p_bt->bt_start || p_bt->zigbee_start || p_bt->thread_start)
    {
        return 1;
    }

    return 0;
}

/*--------------------------------------------------bt node------------------------------------------------------*/

static unsigned int amlbt_w2ls_bt_fops_poll(struct file *file, poll_table *wait)
{
    int mask = 0;
    w2l_sdio_bt_t *p_bt = (w2l_sdio_bt_t *)file->private_data;

    poll_wait(file, &p_bt->bt_wait_queue, wait);

    if (!p_bt->bt_start)
    {
        goto exit;
    }

    if (p_bt->bt_rd_state || skb_queue_len(&p_bt->bt_rx_queue) > 0)
    {
        mask |= POLLIN | POLLRDNORM;
    }

exit:
    return mask;
}

static void amlbt_show_fw_debug_info(void)
{
    unsigned int value = 0;
    BTI("PMU 0x00f03040:%#x \n", amlbt_sdio_read_word(REG_PMU_POWER_CFG));
    usleep_range(10000, 10000);
    value = amlbt_sdio_read_word(REG_FW_PC);
    value = (value >> 6);
    BTI("pc1 0x200034:%#x\n", value);
    usleep_range(10000, 10000);
    value = amlbt_sdio_read_word(REG_FW_PC);
    value = (value >> 6);
    BTI("pc2 0x200034:%#x\n", value);
    usleep_range(10000, 10000);
    value = amlbt_sdio_read_word(REG_FW_PC);
    value = (value >> 6);
    BTI("pc3 0x200034:%#x\n", value);
}

static int amlbt_w2ls_bt_fops_open(struct inode *inode, struct file *file)
{
    int ret = 0;

    BTI("%s, %d, version:%s \n", __func__, sdio_bt.bt_start, AML_W2LS_VERSION);

    if (amlbt_sdio_res_init(&sdio_bt) != 0)
    {
        BTI("amlbt_sdio_res_init failed!\n");
        goto exit;
    }
    file->private_data = &sdio_bt;

    if (!amlbt_w2ls_coex_is_running(&sdio_bt))
    {
        amlbt_load_conf(&sdio_bt);
        ret = amlbt_load_firmware(&sdio_bt);
        if (ret != 0)
        {
            BTI("amlbt_load_firmware failed!\n");
            amlbt_sdio_res_deinit(&sdio_bt);
            goto exit;
        }
    }
    sdio_bt.bt_start = 1;
    amlbt_register_interrupt_gpio(&sdio_bt);
    //register_bt_event_notifier(&bt_nb);
exit:
    return nonseekable_open(inode, file);
}

static ssize_t amlbt_w2ls_bt_fops_write(struct file *file_p, const char __user *buf_p, size_t count, loff_t *pos_p)
{
    struct sk_buff *skb;
    static unsigned char w_type = 0;
    w2l_sdio_bt_t *p_bt = (w2l_sdio_bt_t *)file_p->private_data;
    unsigned char *p;

    if (!p_bt->bt_start)
    {
        BTE("%s:%d p_bt->bt_start == 0!\n", __func__, __LINE__);
        return -EFAULT;
    }

    if (p_bt->hu == NULL)
    {
        BTE("%s:%d p_bt->hu == 0!\n", __func__, __LINE__);
        return -EFAULT;
    }

    if (count == 1) //host write hci type
    {
        get_user(w_type, buf_p);
        //BTI("bt write type %#x \n", w_type);
    }
    else
    {
        skb = alloc_skb(count+1, GFP_KERNEL);
        if (!skb)
        {
            return -ENOMEM;
        }
        p = skb->data;
        *(unsigned char *)skb_put(skb, 1) = w_type;

        if (copy_from_user(skb_put(skb, count), buf_p, count))
        {
            kfree_skb(skb);
            return -EFAULT;
        }
        skb_queue_tail(&p_bt->hu->tx_queue, skb);
        //BTI("bt write %d, [%#x,%#x,%#x,%#x] \n", count, p[0], p[1], p[2], p[3]);
        amlbt_w2ls_uart_tx_wakeup(p_bt->hu);
    }

    return count;
}

static ssize_t amlbt_w2ls_bt_fops_read(struct file *file_p, char __user *buf_p, size_t count, loff_t *pos_p)
{
    w2l_sdio_bt_t *p_bt = (w2l_sdio_bt_t *)file_p->private_data;
    static struct sk_buff *skb;

    if (!p_bt->bt_start)
    {
        BTE("%s:%d p_bt->bt_start == 0!\n", __func__, __LINE__);
        return -EFAULT;
    }

    //BTI("bt read %#x\n", p_bt->bt_rd_state);

    switch (p_bt->bt_rd_state)
    {
        case HCI_RX_TYPE:
        {
            skb = skb_dequeue(&p_bt->bt_rx_queue);
            if (!skb)
            {
                BTE("bt HCI_RX_TYPE no data!\n");
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
                p_bt->bt_rd_state = HCI_RX_TYPE;
                return -EFAULT;
            }
            //BTI("bt read type %#x, %d\n", skb->data[0], skb->len);
            skb_pull(skb, count);
            p_bt->bt_rd_state = HCI_RX_HEADER;
        }
        break;
        case HCI_RX_HEADER:
        {
            if (skb->len < count)
            {
                BTE("%s:%d Failed to copy data: %d, %d\n", __func__, __LINE__, skb->len, count);
                return -EFAULT;
            }
            if (copy_to_user(buf_p, skb->data, count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                kfree_skb(skb);
                p_bt->bt_rd_state = HCI_RX_TYPE;
                return -EFAULT;
            }
            //BTI("bt read header %d, %#x, %#x\n", count, skb->data[0], skb->data[1]);
            skb_pull(skb, count);
            if (skb->len == 0)
            {
                kfree_skb(skb);
                p_bt->bt_rd_state = HCI_RX_TYPE;
            }
            else
            {
                p_bt->bt_rd_state = HCI_RX_PAYLOAD;
            }
        }
        break;
        case HCI_RX_PAYLOAD:
        {
            if (skb->len < count)
            {
                BTE("%s:%d Failed to copy data: %d, %d\n", __func__, __LINE__, skb->len, count);
                return -EFAULT;
            }
            if (copy_to_user(buf_p, skb->data, count))
            {
                BTE("%s, copy_to_user error \n", __func__);
                kfree_skb(skb);
                p_bt->bt_rd_state = HCI_RX_TYPE;
                return -EFAULT;
            }
            //BTI("bt read payload %d, %#x, %#x\n", count, skb->data[0], skb->data[1]);
            kfree_skb(skb);
            p_bt->bt_rd_state = HCI_RX_TYPE;
        }
        break;

    }
    return count;
}

static int amlbt_w2ls_bt_fops_close(struct inode *inode, struct file *file)
{
    BTI("%s, %#x version:%s \n", __func__, sdio_bt.bt_start, AML_W2LS_VERSION);

    if (sdio_bt.bt_start)
    {
        amlbt_show_fw_debug_info();
        //unregister_bt_event_notifier(&bt_nb);
        if (sdio_bt.notify_trig)
        {
            if (!completion_done(&sdio_bt.notify_comp))
            {
                BTI("Waiting for exception task to finish...\n");
                wait_for_completion(&sdio_bt.notify_comp);
            }
        }
        //amlbt_aon_addr_bit_clr(RG_AON_A52, 26);
        amlbt_aon_addr_bit_clr(RG_AON_A24, 26);//bug fix, WIRELESS-10963, Solve the problem that fw cannot run after downloading
        usleep_range(50000, 50000);
        amlbt_unregister_interrupt_gpio(&sdio_bt);
        amlbt_sdio_res_deinit(&sdio_bt);
        amlbt_sdio_write_word(RG_AON_A15, 0);
    }

    return 0;
}

/*--------------------------------------------------zigbee node------------------------------------------------------*/

static unsigned int amlbt_w2ls_zigbee_fops_poll(struct file *file, poll_table *wait)
{
    int mask = 0;
    w2l_sdio_bt_t *p_bt = (w2l_sdio_bt_t *)file->private_data;

    poll_wait(file, &p_bt->zigbee_wait_queue, wait);

    if (!p_bt->zigbee_start)
    {
        goto exit;
    }

    if (p_bt->zigbee_rd_state || skb_queue_len(&p_bt->zigbee_rx_queue) > 0)
    {
        mask |= POLLIN | POLLRDNORM;
    }

exit:
    return mask;
}


static int amlbt_w2ls_zigbee_fops_open(struct inode *inode, struct file *file)
{
    int ret = 0;

    BTI("%s, %d, version:%s \n", __func__, sdio_bt.zigbee_start, AML_W2LS_VERSION);

    if (sdio_bt.zigbee_start)
    {
        BTE("zigbee open status error!\n");
        return -EFAULT;
    }

    if (amlbt_w2ls_coex_zigbee_res_init(&sdio_bt) != 0)
    {
        BTI("amlbt_sdio_res_init failed!\n");
        goto exit;
    }
    file->private_data = &sdio_bt;
    if (!amlbt_w2ls_coex_is_running(&sdio_bt))
    {
        amlbt_load_conf(&sdio_bt);
        ret = amlbt_load_firmware(&sdio_bt);
        if (ret != 0)
        {
            BTI("amlbt_load_firmware failed!\n");
            amlbt_w2ls_coex_zigbee_res_deinit(&sdio_bt);
            goto exit;
        }
    }
    sdio_bt.zigbee_start = 1;
exit:
    return nonseekable_open(inode, file);
}

static int amlbt_w2ls_zigbee_fops_close(struct inode *inode, struct file *file)
{
    BTI("%s, %d version:%s \n", __func__, sdio_bt.zigbee_start, AML_W2LS_VERSION);

    if (sdio_bt.zigbee_start)
    {
        amlbt_show_fw_debug_info();
        amlbt_w2ls_coex_zigbee_res_deinit(&sdio_bt);
        sdio_bt.zigbee_start = 0;
    }
    return 0;
}

static ssize_t amlbt_w2ls_zigbee_fops_write(struct file *file_p, const char __user *buf_p, size_t count, loff_t *pos_p)
{
    struct sk_buff *skb;
    static unsigned char w_type = 0;
    w2l_sdio_bt_t *p_bt = (w2l_sdio_bt_t *)file_p->private_data;
    unsigned char *p;

    if (!p_bt->zigbee_start)
    {
        BTE("%s:%d p_bt->zigbee_start == 0!\n", __func__, __LINE__);
        return -EFAULT;
    }

    if (p_bt->hu == NULL)
    {
        BTE("%s:%d p_bt->hu == 0!\n", __func__, __LINE__);
        return -EFAULT;
    }

    if (count == 1) //host write hci type
    {
        get_user(w_type, buf_p);
    }
    else
    {
        skb = alloc_skb(count+1, GFP_KERNEL);
        if (!skb)
        {
            return -ENOMEM;
        }
        p = skb->data;
        *(unsigned char *)skb_put(skb, 1) = w_type;

        if (copy_from_user(skb_put(skb, count), buf_p, count))
        {
            kfree_skb(skb);
            return -EFAULT;
        }
        skb_queue_tail(&p_bt->hu->tx_queue, skb);
        //BTI("zigbee write %d, [%#x,%#x,%#x,%#x] \n", count, p[0], p[1], p[2], p[3]);
        amlbt_w2ls_uart_tx_wakeup(p_bt->hu);
    }

    return count;
}

static ssize_t amlbt_w2ls_zigbee_fops_read(struct file *file_p, char __user *buf_p, size_t count, loff_t *pos_p)
{
    w2l_sdio_bt_t *p_bt = (w2l_sdio_bt_t *)file_p->private_data;
    static struct sk_buff *skb;

    if (!p_bt->zigbee_start)
    {
        BTE("%s:%d p_bt->zigbee_start == 0!\n", __func__, __LINE__);
        return -EFAULT;
    }

    switch (p_bt->zigbee_rd_state)
    {
        case HCI_RX_TYPE:
        {
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
        }
        break;
        case HCI_RX_HEADER:
        {
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
        }
        break;
        case HCI_RX_PAYLOAD:
        {
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
        }
        break;

    }
    return count;
}

/*----------------------------------------------------thread node----------------------------------------------------------*/

static unsigned int amlbt_w2ls_thread_fops_poll(struct file *file, poll_table *wait)
{
    int mask = 0;
    w2l_sdio_bt_t *p_bt = (w2l_sdio_bt_t *)file->private_data;

    poll_wait(file, &p_bt->thread_wait_queue, wait);

    if (!p_bt->thread_start)
    {
        goto exit;
    }

    if (p_bt->thread_rd_state || skb_queue_len(&p_bt->thread_rx_queue) > 0)
    {
        mask |= POLLIN | POLLRDNORM;
    }

exit:
    return mask;
}


static int amlbt_w2ls_thread_fops_open(struct inode *inode, struct file *file)
{
    int ret = 0;

    BTI("%s, %d, version:%s \n", __func__, sdio_bt.thread_start, AML_W2LS_VERSION);

    if (sdio_bt.thread_start)
    {
        BTE("thread open status error!\n");
        return -EFAULT;
    }

    if (amlbt_w2ls_coex_thread_res_init(&sdio_bt) != 0)
    {
        BTI("amlbt_sdio_res_init failed!\n");
        goto exit;
    }
    file->private_data = &sdio_bt;
    if (!amlbt_w2ls_coex_is_running(&sdio_bt))
    {
        amlbt_load_conf(&sdio_bt);
        ret = amlbt_load_firmware(&sdio_bt);
        if (ret != 0)
        {
            BTI("amlbt_load_firmware failed!\n");
            amlbt_w2ls_coex_thread_res_deinit(&sdio_bt);
            goto exit;
        }
    }
    sdio_bt.thread_start = 1;
exit:
    return nonseekable_open(inode, file);
}

static int amlbt_w2ls_thread_fops_close(struct inode *inode, struct file *file)
{
    BTI("%s, %d version:%s \n", __func__, sdio_bt.thread_start, AML_W2LS_VERSION);

    if (sdio_bt.thread_start)
    {
        amlbt_show_fw_debug_info();
        amlbt_w2ls_coex_thread_res_deinit(&sdio_bt);
        sdio_bt.thread_start = 0;
    }
    return 0;
}

static ssize_t amlbt_w2ls_thread_fops_write(struct file *file_p, const char __user *buf_p, size_t count, loff_t *pos_p)
{
    struct sk_buff *skb;
    static unsigned char w_type = 0;
    w2l_sdio_bt_t *p_bt = (w2l_sdio_bt_t *)file_p->private_data;
    unsigned char *p;

    if (!p_bt->thread_start)
    {
        BTE("%s:%d p_bt->thread_start == 0!\n", __func__, __LINE__);
        return -EFAULT;
    }

    if (count == 1) //host write hci type
    {
        get_user(w_type, buf_p);
    }
    else
    {
        skb = alloc_skb(count+1, GFP_KERNEL);
        if (!skb)
        {
            return -ENOMEM;
        }

        p = skb->data;

        *(unsigned char *)skb_put(skb, 1) = w_type;

        if (copy_from_user(skb_put(skb, count), buf_p, count))
        {
            kfree_skb(skb);
            return -EFAULT;
        }
        skb_queue_tail(&p_bt->hu->tx_queue, skb);
        //BTI("thread write %d, [%#x,%#x,%#x,%#x] \n", count, p[0], p[1], p[2], p[3]);
        amlbt_w2ls_uart_tx_wakeup(p_bt->hu);
    }

    return count;
}

static ssize_t amlbt_w2ls_thread_fops_read(struct file *file_p, char __user *buf_p, size_t count, loff_t *pos_p)
{
    w2l_sdio_bt_t *p_bt = (w2l_sdio_bt_t *)file_p->private_data;
    static struct sk_buff *skb;

    if (!p_bt->thread_start)
    {
        BTE("%s:%d p_bt->thread_start == 0!\n", __func__, __LINE__);
        return -EFAULT;
    }

    switch (p_bt->thread_rd_state)
    {
        case HCI_RX_TYPE:
        {
            skb = skb_dequeue(&p_bt->thread_rx_queue);
            if (!skb)
            {
                BTE("td HCI_RX_TYPE no data!\n");
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
        }
        break;
        case HCI_RX_HEADER:
        {
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
        }
        break;
        case HCI_RX_PAYLOAD:
        {
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
        }
        break;

    }
    return count;
}

static int amlbt_w2ls_coex_fops_open(struct inode *inode, struct file *file)
{
    BTI("%s \n", __func__);
    file->private_data = &sdio_bt;
    return nonseekable_open(inode, file);
}

static int amlbt_w2ls_coex_fops_close(struct inode *inode, struct file *file)
{
    BTI("%s \n", __func__);

    return 0;
}

/*-------------------------------------------------------------------------------------------------------------------------*/


/**-------------------------------------------tty discipline----------------------------------**/

static int amlbt_w2ls_uart_tty_open(struct tty_struct *tty);
static void amlbt_w2ls_uart_tty_close(struct tty_struct *tty);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 20)
static ssize_t amlbt_w2ls_uart_tty_read(struct tty_struct *tty, struct file *file,
                 unsigned char *buf, size_t nr,
                 void **cookie, unsigned long offset);
#else
static ssize_t amlbt_w2ls_uart_tty_read(struct tty_struct *tty, struct file *file,
                             unsigned char __user *buf, size_t nr);
#endif
static ssize_t amlbt_w2ls_uart_tty_write(struct tty_struct *tty, struct file *file,
                  const unsigned char *data, size_t count);
static __poll_t amlbt_w2ls_uart_tty_poll(struct tty_struct *tty,
                  struct file *filp, poll_table *wait);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
static void amlbt_w2ls_uart_tty_receive(struct tty_struct *tty, const unsigned char *data, char *flags, int count);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
static void amlbt_w2ls_uart_tty_receive(struct tty_struct *tty, const uint8_t *data, const char *flags, int count);
#else
static void amlbt_w2ls_uart_tty_receive(struct tty_struct *tty, const uint8_t *data, const uint8_t *flags, size_t count);
#endif
static struct tty_ldisc_ops amlbt_w2ls_uart_ldisc = {
    .owner      = THIS_MODULE,
    .num        = N_HCI,
    .name       = "n_hci",
    .open       = amlbt_w2ls_uart_tty_open,
    .close      = amlbt_w2ls_uart_tty_close,
    .read       = amlbt_w2ls_uart_tty_read,
    .write      = amlbt_w2ls_uart_tty_write,
    .ioctl      = NULL,
    .compat_ioctl   = NULL,
    .poll       = amlbt_w2ls_uart_tty_poll,
    .receive_buf    = amlbt_w2ls_uart_tty_receive,
    //.write_wakeup = hci_uart_tty_wakeup,
    .write_wakeup   = NULL,
};

static struct sk_buff *amlbt_w2ls_uart_dequeue(struct hci_uart *hu, struct sk_buff_head *p_head)
{
    struct sk_buff *skb = skb_dequeue(p_head);

    return skb;
}

static void amlbt_w2ls_uart_write_work(struct work_struct *work)
{
    struct hci_uart *hu = container_of(work, struct hci_uart, write_work);
    struct tty_struct *tty = hu->tty;
    struct sk_buff *skb;

    /* REVISIT: should we cope with bad skbs or ->write() returning
     * and error value ?
     */

restart:
    clear_bit(HCI_UART_TX_WAKEUP, &hu->tx_state);

    //BTI("amlbt_w2ls_uart_write_work \n");

    while ((skb = amlbt_w2ls_uart_dequeue(hu, &hu->tx_queue))) {
        int len;


        //BTI("amlbt_w2ls_uart_dequeue skb->len %d \n", skb->len);
        set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
        len = tty->ops->write(tty, skb->data, skb->len);

        skb_pull(skb, len);
        if (skb->len) {
            hu->tx_skb = skb;
            break;
        }

        //hci_uart_tx_complete(hu, skb->data[0]);
        kfree_skb(skb);
        //BTI("amlbt_w2ls_uart_write_work complete! \n");
    }

    clear_bit(HCI_UART_SENDING, &hu->tx_state);
    if (test_bit(HCI_UART_TX_WAKEUP, &hu->tx_state))
        goto restart;

    wake_up_bit(&hu->tx_state, HCI_UART_SENDING);
}

static int amlbt_w2ls_uart_tty_open(struct tty_struct *tty)
{
    struct hci_uart *hu;

    BTI("amlbt_w2ls_uart_tty_open tty %p \n", tty);

    /* Error if the tty has no write op instead of leaving an exploitable
     * hole
     */
    if (tty->ops->write == NULL)
    {
        BTE("amlbt_w2ls_uart_tty_open EOPNOTSUPP \n");
        return -EOPNOTSUPP;
    }
    hu = kzalloc(sizeof(struct hci_uart), GFP_KERNEL);
    if (!hu) {
        BTE("Can't allocate control structure \n");
        return -ENFILE;
    }
    tty->disc_data = hu;
    hu->tty = tty;
    tty->receive_room = 65536;

    /* disable alignment support by default */
    hu->alignment = 1;
    hu->padding = 0;
    hu->rx_skb = NULL;
    hu->rx_state = HCI_RX_TYPE;
    memset(hu->uart_buf, 0, sizeof(hu->uart_buf));
    hu->p_ub = &hu->uart_buf[0];
    skb_queue_head_init(&hu->tx_queue);
    skb_queue_head_init(&hu->rx_queue);
    init_waitqueue_head(&hu->wait_queue);
    //INIT_WORK(&hu->init_ready, hci_uart_init_work);
    INIT_WORK(&hu->write_work, amlbt_w2ls_uart_write_work);
    percpu_init_rwsem(&hu->proto_lock);
    /* Flush any pending characters in the driver */
    tty_driver_flush_buffer(tty);
    BTI("amlbt_w2ls_uart_tty_open success!\n");
    set_bit(HCI_UART_PROTO_READY, &hu->flags);
    sdio_bt.hu = hu;
    return 0;
}

static void amlbt_w2ls_uart_tty_close(struct tty_struct *tty)
{
    struct hci_uart *hu = tty->disc_data;

    BTI("amlbt_w2ls_uart_tty_close tty %p", tty);
    /* Detach from the tty */
    tty->disc_data = NULL;

    if (!hu)
        return;

    if (test_bit(HCI_UART_PROTO_READY, &hu->flags)) {
        percpu_down_write(&hu->proto_lock);
        clear_bit(HCI_UART_PROTO_READY, &hu->flags);
        percpu_up_write(&hu->proto_lock);

        //cancel_work_sync(&hu->init_ready);
        cancel_work_sync(&hu->write_work);
        //hu->proto->close(hu);
    }
    clear_bit(HCI_UART_PROTO_SET, &hu->flags);

    percpu_free_rwsem(&hu->proto_lock);
    skb_queue_purge(&hu->tx_queue);
    skb_queue_purge(&hu->rx_queue);
    kfree(hu);
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 20)
static ssize_t amlbt_w2ls_uart_tty_read(struct tty_struct *tty, struct file *file,
                 unsigned char *buf, size_t nr,
                 void **cookie, unsigned long offset)
#else
static ssize_t amlbt_w2ls_uart_tty_read(struct tty_struct *tty, struct file *file,
                             unsigned char __user *buf, size_t nr)
#endif
{
    BTI("%s \n", __func__);
    return 0;
}

static ssize_t amlbt_w2ls_uart_tty_write(struct tty_struct *tty, struct file *file,
                  const unsigned char *data, size_t count)
{
    BTI("%s \n", __func__);
    return 0;
}

static __poll_t amlbt_w2ls_uart_tty_poll(struct tty_struct *tty,
                      struct file *filp, poll_table *wait)
{
    BTI("%s \n", __func__);
    return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
static void amlbt_w2ls_uart_tty_receive(struct tty_struct *tty, const unsigned char *data, char *flags, int count)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0)
static void amlbt_w2ls_uart_tty_receive(struct tty_struct *tty, const uint8_t *data, const char *flags, int count)
#else
static void amlbt_w2ls_uart_tty_receive(struct tty_struct *tty, const uint8_t *data, const uint8_t *flags, size_t count)
#endif
{
    static struct hci_uart_rx rx = { .state = HCI_STATE_RX_TYPE, .payload = NULL };
    struct hci_uart *hu;
    w2l_sdio_bt_t *p_bt = &sdio_bt;

    if (!tty || !data)
    {
        BTE("amlbt_w2ls_uart_tty_receive:tty or data NULL!\n");
        return;
    }

    hu = tty->disc_data;
    if (!hu)
    {
        BTE("amlbt_w2ls_uart_tty_receive:hu NULL!\n");
        return;
    }

    BTP("amlbt_w2ls_uart_tty_receive %d, %p, [%#x,%#x,%#x,%#x,%#x,%#x] \n", count, hu->rx_skb,
                               data[0], data[1], data[2], data[3], data[4], data[5]);

    while (count > 0) {
        switch (rx.state) {
            case HCI_STATE_RX_TYPE:
                rx.type = *data++;
                BTP("type:%#x\n", rx.type);
                if (rx.type != HCI_TYPE_EVENT && rx.type != HCI_TYPE_ACL && rx.type != HCI_TYPE_15P4)
                {
                    BTE("[aml tty error]:type error!!\n");
                    schedule_work(&sdio_bt.exception_work);
                    return;
                }
                count--;
                rx.received_length = 0;
                if (rx.type == HCI_TYPE_EVENT)
                {
                    rx.expected_length = 2;
                    rx.header_len = 3;
                }
                else if (rx.type == HCI_TYPE_ACL)
                {
                    rx.expected_length = 4;
                    rx.header_len = 5;
                }
                else if (rx.type == HCI_TYPE_15P4)
                {
                    rx.expected_length = 4;
                    rx.header_len = 5;
                }

                //rx.expected_length = (rx.type == HCI_TYPE_EVENT) ? 2 : 4;
                rx.state = HCI_STATE_RX_HEADER;
                break;

            case HCI_STATE_RX_HEADER:
                while (count > 0 && rx.received_length < rx.expected_length) {
                    rx.header[rx.received_length++] = *data++;
                    count--;
                }
                if (rx.received_length == rx.expected_length) {
                    // calc header length
                    if (rx.type == HCI_TYPE_EVENT) {
                        rx.expected_length = rx.header[1];
                    } else if (rx.type == HCI_TYPE_ACL) {
                        rx.expected_length = rx.header[2] | (rx.header[3] << 8);
                    } else if (rx.type == HCI_TYPE_15P4) {
                        rx.expected_length = (rx.header[2] | (rx.header[3] << 8)) + 2;  // 15.4 pyaload + 2 bytes crc
                        //BTE("HCI_TYPE_15P4:%d\n", rx.expected_length);
                    }

                    BTP("header %d:[%#x,%#x,%#x,%#x]\n", rx.expected_length,
                        rx.header[0], rx.header[1], rx.header[2], rx.header[3]);

                    //if (rx.type != HCI_TYPE_EVENT && rx.type != HCI_TYPE_ACL && rx.type != HCI_TYPE_15P4)
                    //{
                        ///BTE("[aml tty error]:type error!!\n");
                        //return;
                    //}
                    rx.received_length = 0;
                    rx.payload = kmalloc(rx.expected_length, GFP_KERNEL);
                    if (!rx.payload) {
                        rx.state = HCI_STATE_RX_TYPE;
                        return;
                    }
                    rx.state = HCI_STATE_RX_PAYLOAD;
                }
                break;

            case HCI_STATE_RX_PAYLOAD:
                while (count > 0 && rx.received_length < rx.expected_length) {
                    rx.payload[rx.received_length++] = *data++;
                    count--;
                }
                if (rx.received_length == rx.expected_length) {
                    BTP("payload %d:%d\n", rx.received_length, rx.header_len);
                    hu->rx_skb = alloc_skb(rx.received_length + rx.header_len, GFP_ATOMIC);
                    if (!hu->rx_skb)
                    {
                        BTE("[aml tty error]:skb error!!\n");
                        return;
                    }
                    skb_put_data(hu->rx_skb, &rx.type, 1);
                    if (rx.type == HCI_TYPE_EVENT)
                    {
                        skb_put_data(hu->rx_skb, rx.header, 2);
                    }
                    else if (rx.type == HCI_TYPE_ACL)
                    {
                        skb_put_data(hu->rx_skb, rx.header, 4);
                    }
                    else if (rx.type == HCI_TYPE_15P4)
                    {
                        skb_put_data(hu->rx_skb, rx.header, 4);
                    }
                    skb_put_data(hu->rx_skb, rx.payload, rx.expected_length);
                    //process_hci_packet(rx.type, rx.header, rx.payload, rx.expected_length);
                    kfree(rx.payload);
                    rx.payload = NULL;
                    rx.state = HCI_STATE_RX_TYPE;
#if 0
                    if (rx.type == HCI_TYPE_EVENT || rx.type == HCI_TYPE_ACL)
                    {
                        //BTI("HCI complete bluetooth!\n");
                        skb_queue_tail(&p_bt->bt_rx_queue, hu->rx_skb);
                        wake_up_interruptible(&p_bt->bt_wait_queue);
                    }
                    else if (rx.type == HCI_TYPE_15P4 && rx.header[0] == HCI_TYPE_ZIGBEE)
                    {
                        //BTI("HCI complete zigbee!\n");
                        skb_queue_tail(&p_bt->zigbee_rx_queue, hu->rx_skb);
                        wake_up_interruptible(&p_bt->zigbee_wait_queue);
                    }
                    else if (rx.type == HCI_TYPE_15P4 && rx.header[0] == HCI_TYPE_THREAD)
                    {
                        //BTI("HCI complete thread!\n");
                        skb_queue_tail(&p_bt->thread_rx_queue, hu->rx_skb);
                        wake_up_interruptible(&p_bt->thread_wait_queue);
                    }
                    else
                    {
                        BTF("unknown hci packet!!!\n");
                        kfree(hu->rx_skb);
                    }
#else
                    if (rx.type == HCI_TYPE_EVENT || rx.type == HCI_TYPE_ACL || rx.type == HCI_TYPE_15P4)
                    {
                        BTP("HCI complete bluetooth!\n");
                        skb_queue_tail(&p_bt->bt_rx_queue, hu->rx_skb);
                        wake_up_interruptible(&p_bt->bt_wait_queue);
                    }
                    else
                    {
                        BTF("unknown hci packet!!!\n");
                        kfree(hu->rx_skb);
                    }
#endif
                    hu->rx_skb = NULL;
                }
                break;
        }
    }
}

static int amlbt_w2ls_uart_tx_wakeup(struct hci_uart *hu)
{
    /* This may be called in an IRQ context, so we can't sleep. Therefore
     * we try to acquire the lock only, and if that fails we assume the
     * tty is being closed because that is the only time the write lock is
     * acquired. If, however, at some point in the future the write lock
     * is also acquired in other situations, then this must be revisited.
     */
    if (!percpu_down_read_trylock(&hu->proto_lock))
    {
        BTE("amlbt_w2ls_uart_tx_wakeup error 1! \n");
        return 0;
    }
    if (!test_bit(HCI_UART_PROTO_READY, &hu->flags))
    {
        //printk(KERN_INFO "amlbt_w2ls_uart_tx_wakeup error 2! \n");
        goto no_schedule;
    }
    set_bit(HCI_UART_TX_WAKEUP, &hu->tx_state);
    if (test_and_set_bit(HCI_UART_SENDING, &hu->tx_state))
    {
        //printk(KERN_INFO "amlbt_w2ls_uart_tx_wakeup error 3! \n");
        goto no_schedule;
    }
    //printk(KERN_INFO "amlbt_w2ls_uart_tx_wakeup work! \n");

    schedule_work(&hu->write_work);

no_schedule:
    percpu_up_read(&hu->proto_lock);

    return 0;
}

static void amlbt_w2ls_exception_func(struct work_struct *work)
{
    w2l_sdio_bt_t *p_sdio = &sdio_bt;
    struct sk_buff      *skb;
    unsigned char bt_hw_error[5] = {0x04, 0x10, 0x01, 0x00, 0x00};
    unsigned char zigbee_hw_error[8] = {0x10, 0xfa, 0x42, 0x01, 0x00, 0x00, 0x00, 0x00};
    unsigned char thread_hw_error[8] = {0x10, 0xfa, 0x42, 0x01, 0x00, 0x00, 0x00, 0x00};

    BTF("Coex driver detect excepion! [%#x,%#x,%#x]\n",
        p_sdio->bt_start, p_sdio->zigbee_start, p_sdio->thread_start);

    if (p_sdio->bt_start || p_sdio->zigbee_start || p_sdio->thread_start)
    {
        if (p_sdio->bt_start)
        {
            skb_queue_purge(&p_sdio->bt_tx_queue);
            skb_queue_purge(&p_sdio->bt_rx_queue);
        }
        if (p_sdio->zigbee_start)
        {
            skb_queue_purge(&p_sdio->zigbee_tx_queue);
            skb_queue_purge(&p_sdio->zigbee_rx_queue);
        }
        if (p_sdio->thread_start)
        {
            skb_queue_purge(&p_sdio->thread_tx_queue);
            skb_queue_purge(&p_sdio->thread_rx_queue);
        }

        amlbt_load_firmware(&sdio_bt);

        if (p_sdio->bt_start)
        {
            skb = alloc_skb(sizeof(bt_hw_error), GFP_ATOMIC);
            if (!skb)
            {
                BTF("bt skb error!!\n");
                return;
            }
            skb_put_data(skb, bt_hw_error, sizeof(bt_hw_error));
            BTF("Report bt hw error!\n");
            skb_queue_tail(&p_sdio->bt_rx_queue, skb);
            wake_up_interruptible(&p_sdio->bt_wait_queue);
        }

        if (p_sdio->zigbee_start)
        {
            skb = alloc_skb(sizeof(zigbee_hw_error), GFP_ATOMIC);
            if (!skb)
            {
                BTF("zigbee skb error!!\n");
                return;
            }
            skb_put_data(skb, zigbee_hw_error, sizeof(zigbee_hw_error));
            BTF("Report zigbee hw error!\n");
            skb_queue_tail(&p_sdio->zigbee_rx_queue, skb);
            wake_up_interruptible(&p_sdio->zigbee_wait_queue);
        }

        if (p_sdio->thread_start)
        {
            skb = alloc_skb(sizeof(thread_hw_error), GFP_ATOMIC);
            if (!skb)
            {
                BTF("thread skb error!!\n");
                return;
            }
            skb_put_data(skb, thread_hw_error, sizeof(thread_hw_error));
            BTF("Report thread hw error!\n");
            skb_queue_tail(&p_sdio->thread_rx_queue, skb);
            wake_up_interruptible(&p_sdio->thread_wait_queue);
        }
    }

    BTF("Coex driver excepion finish!\n");
}

