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
#include <sys/time.h>

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
    int cnt = 0;
    unsigned val = 0;
    unsigned char fw_pc[12] = {0};
    unsigned char fw_log[516] = {0};
    unsigned int fw_log_addr = 0x413b60;
    struct timeval start_time, current_time;
    gettimeofday(&start_time, NULL);
    long elapsed_ms = 0;

    property_get(PWR_PROP_NAME, shutdwon_status, "unknown");
    ALOGD("%s %s ", __FUNCTION__, shutdwon_status);

    ALOGD("try to read pc...");
    ALOGD("bt pc1:");
    *(unsigned int *)&fw_pc[0] = amlbt_get_reg(0x200034);
    ms_delay(5);
    ALOGD("bt pc2:");
    *(unsigned int *)&fw_pc[4] = amlbt_get_reg(0x200034);
    ms_delay(5);
    ALOGD("bt pc3:");
    *(unsigned int *)&fw_pc[8] = amlbt_get_reg(0x200034);
    ALOGD("bt fw log:");
    for (cnt = 0; cnt < sizeof(fw_log); cnt += 4)
    {
        *(unsigned int *)&fw_log[cnt] = amlbt_get_reg(fw_log_addr + cnt);
        gettimeofday(&current_time, NULL);
        elapsed_ms = (current_time.tv_sec - start_time.tv_sec) * 1000 +
                      (current_time.tv_usec - start_time.tv_usec) / 1000;

        if (elapsed_ms > 400)
        {
            ALOGD("break read log because timeout %ld ms, cnt:%d", elapsed_ms, cnt);
            break;
        }
    }
    ALOGD("bt fw log end");
    save_regs_to_file(fw_pc, sizeof(fw_pc), "/data/vendor/fw_pc.txt");
    save_regs_with_time_str(fw_log, sizeof(fw_log), "/data/vendor/fw_log_last.txt");

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


