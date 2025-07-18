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

#include "common.h"
#include "gather_data.h"

#define AML_BT_GATHER_DATA_NAME "amlbt_gather_data"


static sdio_wr_word_inf p_wr_word_inf = NULL;
static sdio_rd_word_inf p_rd_word_inf = NULL;
static uint32_t gd_reg = 0x280c30;//0x2fe008;

static ssize_t amlbt_gd_read(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t amlbt_gd_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static struct device_attribute amlbt_gd_dev_attr = {
    .attr = { .name = AML_BT_GATHER_DATA_NAME, .mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH },
    .show = amlbt_gd_read,
    .store = amlbt_gd_write,
};

static int hex_char_to_int(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1; // Invalid hex character
}

// Function to parse a single hex number from the string
static int parse_hex(const char *str, uint32_t *value)
{
    int i = 0;
    *value = 0;

    // Expect the string to start with "0x"
    if (str[i] != '0' || str[i+1] != 'x')
        return -EINVAL; // Invalid format

    i += 2; // Skip "0x"

    // Parse each hex digit
    while (isxdigit(str[i])) {
        int digit = hex_char_to_int(str[i]);
        if (digit == -1)
            return -EINVAL; // Invalid hex character
        *value = (*value << 4) | digit; // Shift left and add the digit
        i++;
    }

    return i; // Return the number of characters processed
}


static ssize_t amlbt_gd_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    // Read from the specified register
    uint32_t reg_data = 0;

    BTI("amlbt_gd_read reg:%#x\n", gd_reg);

    if (p_rd_word_inf)
    {
        reg_data = p_rd_word_inf(gd_reg);
        BTI("amlbt_gd_read data:%#x\n", reg_data);
    }
    // Return the data in a format that `cat` command can display
    return sprintf(buf, "0x%x\n", reg_data);
}

static ssize_t amlbt_gd_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    uint32_t reg_addr = 0, reg_data = 0;
    int pos = 0, parsed;

    BTI("amlbt_gd_write, count: %d\n", count);

    // Parse the first hex number (register address)
    parsed = parse_hex(buf, &reg_addr);
    if (parsed < 0) {
        printk(KERN_ERR "Invalid register address format\n");
        return -EINVAL;
    }
    pos += parsed;

    // Skip any spaces or commas
    while (buf[pos] == ' ' || buf[pos] == ',') {
        pos++;
    }

    // Parse the second hex number (register data)
    parsed = parse_hex(&buf[pos], &reg_data);
    if (parsed < 0) {
        printk(KERN_ERR "Invalid register data format\n");
        return -EINVAL;
    }

    // Write the data to the specified register
    if (p_wr_word_inf)
    {
        p_wr_word_inf(reg_addr, reg_data);
        gd_reg = reg_addr;
        BTI("amlbt_gd_write success\n");
    }
    BTI("amlbt_gd_write: %#x, %#x\n", reg_addr, reg_data);
    return count;
}

int amlbt_gather_data_init(struct device *dev, sdio_wr_word_inf p_wr_word_func, sdio_rd_word_inf p_rd_word_func)
{
    int res = 0;

    BTI("%s, version:%s \n", __func__, AML_GD_VERSION);
    res = device_create_file(dev, &amlbt_gd_dev_attr);
    if (res)
    {
        BTE("%s:Failed to create device attribute\n", __func__);
    }
    else
    {
        p_wr_word_inf = p_wr_word_func;
        p_rd_word_inf = p_rd_word_func;
    }
    return res;
}

void amlbt_gather_data_deinit(struct device *dev)
{
    device_remove_file(dev, &amlbt_gd_dev_attr);
}

