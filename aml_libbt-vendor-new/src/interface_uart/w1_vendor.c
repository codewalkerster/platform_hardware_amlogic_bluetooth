/******************************************************************************
*
*  Copyright (C) 2029-2021 Amlogic Corporation
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
*  Filename:      bt_vendor_aml.c
*
*  Description:   Amlogic vendor specific library implementation
*
******************************************************************************/

#define LOG_TAG "bt_vendor"

#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <ctype.h>
#include <utils/Log.h>
#include <cutils/properties.h>
#include <sys/syscall.h>
#include <signal.h>
#include <errno.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/time.h>
#include <assert.h>

#include "bt_vendor_aml.h"
#include "upio.h"
#include "userial_vendor.h"
#include "vendor_common.h"

int w1_bt_power = 0;

static int libbt_op_power_ctrl(int state, int (*fd_array)[]);
static int libbt_op_fw_cfg(int state, int (*fd_array)[]);
static int libbt_op_sco_cfg(int state, int (*fd_array)[]);
static int libbt_op_userial_open(int state, int (*fd_array)[]);
static int libbt_op_userial_close(int state, int (*fd_array)[]);
static int libbt_op_get_lpm_idle_timeout(int state, int (*fd_array)[]);
static int libbt_op_lpm_set_mode(int state, int (*fd_array)[]);


int (*libbtw1_func[])(int state, int (*fd_array)[]) =
{
    libbt_op_power_ctrl,
    libbt_op_fw_cfg,
    libbt_op_sco_cfg,
    libbt_op_userial_open,
    libbt_op_userial_close,
    libbt_op_get_lpm_idle_timeout,
    libbt_op_lpm_set_mode
};

static const tUSERIAL_CFG userial_init_cfg =
{
    (USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1),
    USERIAL_BAUD_115200
};

static int libbt_op_power_ctrl(int state, int (*fd_array)[])
{
    if (state == BT_VND_PWR_ON)
    {
        //check driver
        w1_bt_power = driver_check("sdio_bt", 20);
        ALOGD("w1_bt_power %d", w1_bt_power);

        // bt en on
        if (upio_power_get() == 0)
        {
            ALOGD("first, set bt power");
            upio_set_bluetooth_power(UPIO_BT_POWER_ON);
            ALOGD("end, set bt power");
        }

        //insmod driver
        insmod("/vendor/lib/modules/aml_sdio.ko", "", "aml_sdio", 200);
        insmod("/vendor/lib/modules/sdio_bt.ko", "aml_w1", "sdio_bt", 200);
    }
    ALOGD("%s %d \n", __func__, state);
    return 0;
}

static int libbt_op_fw_cfg(int state, int (*fd_array)[])
{
    hw_config_start();
    ALOGD("%s \n", __func__);
    return 0;
}

static int libbt_op_sco_cfg(int state, int (*fd_array)[])
{
    ALOGD("%s \n", __func__);
    return 0;
}

static int libbt_op_userial_open(int state, int (*fd_array)[])
{
    int idx = 0;

    g_userial_fd = userial_vendor_open((tUSERIAL_CFG *)&userial_init_cfg);
    if (g_userial_fd != -1)
    {
        for (idx = 0; idx < CH_MAX; idx++)
        {
            (*fd_array)[idx] = g_userial_fd;
        }
        if (w1_bt_power)
        {
            userial_vendor_set_baud(line_speed_to_userial_baud(4000000));
            bt_sdio_fd = userial_vendor_devchar_open();
            return 1;
        }
        if (aml_uart_init() != 0)
        {
            return -1;
        }
        bt_sdio_fd = userial_vendor_devchar_open();
        if (bt_sdio_fd < 0)
        {
            ALOGD("bluetooth node open failed!");
            return -1;
        }
    }
    else
    {
        ALOGD("%s userial_vendor_open failed \n", __func__);
        return -1;
    }
    ALOGD("%s \n", __func__);

    return 1;
}

static int libbt_op_userial_close(int state, int (*fd_array)[])
{
    property_get(PWR_PROP_NAME, shutdwon_status, "unknown");
    ALOGD("%s %s\n", __func__, shutdwon_status);
    if (strstr(shutdwon_status, "0userrequested") != NULL)
    {
        ALOGD("w1 amlbt shutdown");
        aml_woble_configure(g_userial_fd);
    }
    else
    {
        ALOGD("w1 close reset bt");
        aml_reset_bt(g_userial_fd);
    }
    userial_vendor_close();
    return 0;
}

static int libbt_op_get_lpm_idle_timeout(int state, int (*fd_array)[])
{
    ALOGD("%s \n", __func__);
    return 0;
}

static int libbt_op_lpm_set_mode(int state, int (*fd_array)[])
{
    ALOGD("%s \n", __func__);
    return 0;
}


