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

#include "bt_vendor_aml.h"
#include "upio.h"
#include "userial_vendor.h"
#include "bt_vendor_lib.h"
#include "vendor_common.h"

static long bt_recovery = -1;
static long bt_shutdown = -1;

static int libbt_op_power_ctrl(int state, int (*fd_array)[]);
static int libbt_op_fw_cfg(int state, int (*fd_array)[]);
static int libbt_op_sco_cfg(int state, int (*fd_array)[]);
static int libbt_op_userial_open(int state, int (*fd_array)[]);
static int libbt_op_userial_close(int state, int (*fd_array)[]);
static int libbt_op_get_lpm_idle_timeout(int state, int (*fd_array)[]);
static int libbt_op_lpm_set_mode(int state, int (*fd_array)[]);

int (*libbtw2_usb_func[])(int state, int (*fd_array)[]) =
{
    libbt_op_power_ctrl,
    libbt_op_fw_cfg,
    libbt_op_sco_cfg,
    libbt_op_userial_open,
    libbt_op_userial_close,
    libbt_op_get_lpm_idle_timeout,
    libbt_op_lpm_set_mode
};

static int libbt_op_power_ctrl(int state, int (*fd_array)[])
{
    if (state == BT_VND_PWR_ON)
    {
        // bt en on
        if (upio_power_get() == 0)
        {
            ALOGD("bt only, set bt power begin");
            upio_set_bluetooth_power(UPIO_BT_POWER_ON);
            ALOGD("bt only, set bt power end");
            usleep(300000);  //waiting usb device enumerate
        }
        rmmod("wifi_comm", 100);
        snprintf(driver_pram, sizeof(driver_pram), "amlbt_if_type=%u",
                *((unsigned short*)&amlbt_transtype));
        ALOGD("%s %s", __FUNCTION__, driver_pram);
        insmod("/vendor/lib/modules/w2_comm.ko", "bus_type=usb", "w2_comm", 200);
        insmod("/vendor/lib/modules/aml_bt.ko", driver_pram, "aml_bt", 200);
    }
    ALOGD("%s state %d\n", __func__, state);
    return 0;
}

static int libbt_op_fw_cfg(int state, int (*fd_array)[])
{
    hw_config_quick_start();
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

    g_userial_fd = userial_vendor_devchar_open();

    if (g_userial_fd != -1)
    {
        for (idx = 0; idx < CH_MAX; idx++)
        {
            (*fd_array)[idx] = g_userial_fd;
        }
    }
    else
    {
        ALOGD("%s userial_vendor_devchar_open failed \n", __func__);
        return -1;
    }

    ALOGD("%s \n", __func__);
    return 1;
}

static int libbt_op_userial_close(int state, int (*fd_array)[])
{
    property_get(PWR_PROP_NAME, shutdwon_status, "unknown");

    ALOGD("%s %s\n", __func__, shutdwon_status);
    if (!bt_recovery && hw_cfg_cb.state == 0)
    {
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
    if (state == BT_VND_PWR_OFF)
    {
        property_get(PWR_PROP_NAME, shutdwon_status, "unknown");
        if (ioctl(g_userial_fd, IOCTL_GET_BT_RECOVERY, &bt_recovery) != 0)
        {
            ALOGD("ioctl send failed: fd %d, error %s, revData %ld", g_userial_fd, strerror(errno), bt_recovery);
        }
        else
        {
            ALOGD("receive bt recovery=%ld\n", bt_recovery);
        }
        if (strstr(shutdwon_status, "0userrequested") != NULL)
        {
            bt_shutdown = 1;
            if (ioctl(g_userial_fd, IOCTL_SET_BT_SHUTDOWN, &bt_shutdown) != 0)
            {
                ALOGD("ioctl send failed: fd %d, error %s", g_userial_fd, strerror(errno));
            }
            else
            {
                ALOGD("send bt shutdown=%ld\n", bt_shutdown);
            }
        }
        ALOGD("%s \n", __func__);
    }
    return 0;
}


