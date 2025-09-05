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
*  Filename:      vendor_common.c
*
*  Description:  7 April 2025 amlogic android bluetooth libbt isolation,
                 bt_vendor_aml as the entry file,
                 using callback libbt_op0 will be the code according to the product w1/w1u/w2/w2l to do the isolation,
                 a total of calls integrated in the vendor_common file
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
#include <termios.h>

#include "bt_vendor_aml.h"
#include "upio.h"
#include "userial_vendor.h"
#include "vendor_common.h"

//coex dev
static unsigned char coex_status = 0;
static int fd_coex = -1;
static unsigned long bt_shutdown = 1;

static const tUSERIAL_CFG userial_init_cfg =
{
    (USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1),
    USERIAL_BAUD_115200
};

static int libbt_op_power_ctrl(int state, int (*fd_array)[]);
static int libbt_op_fw_cfg(int state, int (*fd_array)[]);
static int libbt_op_sco_cfg(int state, int (*fd_array)[]);
static int libbt_op_userial_open(int state, int (*fd_array)[]);
static int libbt_op_userial_close(int state, int (*fd_array)[]);
static int libbt_op_get_lpm_idle_timeout(int state, int (*fd_array)[]);
static int libbt_op_lpm_set_mode(int state, int (*fd_array)[]);

int (*libbtw2l_uart_func[])(int state, int (*fd_array)[]) =
{
    libbt_op_power_ctrl,
    libbt_op_fw_cfg,
    libbt_op_sco_cfg,
    libbt_op_userial_open,
    libbt_op_userial_close,
    libbt_op_get_lpm_idle_timeout,
    libbt_op_lpm_set_mode
};

static void sigpipeHandler(int signum)
{
    printf("Caught SIGPIPE signal (%d)\n", signum);
}

static int amlbt_chardev_coex_open(void)
{
    int cnt = 0;
    int fd = -1;

open_retry:
    if ((fd = open("/dev/aml_coex", O_RDWR)) < 0)
    {
        ALOGE("%s: unable to open /dev/aml_coex: %s %d", __func__, strerror(errno), cnt);
        usleep(50000);
        cnt++;
        if (cnt < 40)
            goto open_retry;
        ALOGE("/dev/aml_coex open fail!!");
        return -1;
    }
    return fd;
}

static void aml_register_sdio(int fd)
{
    if (ioctl(fd, IOCTL_REGISTER_SDIO, NULL) != 0)
    {
        ALOGD("ioctl send failed: fd %d, error %s", fd, strerror(errno));
    }
    else
    {
        ALOGD("w2l register sdio ok\n");
    }
}

static int libbt_op_power_ctrl(int state, int (*fd_array)[])
{
    int value = 0;
    int retry = 0;
    int fd_coex = -1;
    if (state == BT_VND_PWR_OFF)
    {
        property_get(PWR_PROP_NAME, shutdwon_status, "unknown");
        if (strstr(shutdwon_status, "0userrequested") == NULL)
        {
            fd_coex = amlbt_chardev_coex_open();
            if (fd_coex > 0)
            {
                coex_status = aml_get_w2l_coex_status(fd_coex);
                close(fd_coex);
                fd_coex = -1;
                if ((coex_status & ZIGBEE_ALIVE) || (coex_status & THREAD_ALIVE))
                {
                    ALOGD("15p4 alive");
                }
                else
                {
                    //ALOGD("15p4 not alive, rmmod driver and power off bt_en");
                    //rmmod("w2l_bt", 60);
                    ALOGD("15p4 not alive, power off bt_en");
                    upio_set_bluetooth_power(UPIO_BT_POWER_OFF);
                }
            }
            else
            {
                ALOGD("coex node open failed:%s", strerror(errno));
            }
        }
        if (amlbt_fw_mode == FW_MODE_COEX)
        {
            exit_thread = true;
            aml_15p4_deinit();
        }
    }
    else if (state == BT_VND_PWR_ON)
    {
        snprintf(driver_pram, sizeof(driver_pram), "amlbt_if_type=%u",
                *((unsigned short*)&amlbt_transtype));
        ALOGD("%s %s", __FUNCTION__, driver_pram);
        // bt en on
        if (upio_power_get() == 0)
        {
            ALOGD("first, set bt power");
            upio_set_bluetooth_power(UPIO_BT_POWER_ON);
            ALOGD("end, set bt power");
        }
        rmmod("wifi_comm", 100);
        //insmod driver
        insmod("/vendor/lib/modules/w2l_comm.ko", "bus_type=sdio", "w2l_comm", 200);
        insmod("/vendor/lib/modules/w2l_bt.ko", driver_pram, "w2l_bt", 200);
/*        if (insmod_check("w2l") == 0)
          {
              ALOGD("bt only delay 50ms");
               ms_delay(50);
           }
*/
        fd_coex = amlbt_chardev_coex_open();
        if (fd_coex < 0)
        {
          ALOGD("coex node open failed:%s", strerror(errno));
          return -1;
        }
        aml_register_sdio(fd_coex);
        while (retry < 60)// 300ms
        {
          if (ioctl(fd_coex, IOCTL_GET_SDIO_PROBE_STATUS, &value) == 0)
          {
            if (value == 1)
              break;
          }
          ms_delay(5);
          retry++;
        }

        if (retry >= 60)
        {
          ALOGD("Timeout waiting for g_sdio_after_porbe == 1");
          close(fd_coex);
          return -1;
        }
        ALOGD("%s retry %d \n", __func__, retry);

        close(fd_coex);
        fd_coex = -1;

        if (amlbt_fw_mode == FW_MODE_COEX)
        {
            exit_thread = false;
            if (signal(SIGPIPE, sigpipeHandler) == SIG_ERR) {
                perror("Unable to register SIGPIPE handler");
            }
            if (pthread_create(&aml_15p4_handle_thread, NULL, aml_15p4_socket, NULL) != 0)
            {
                perror("pthread_create");
                return -1;
            }
        }
    }
    ALOGD("%s %d \n", __func__, state);
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
    int i = N_HCI;
    int idx = 0;

    fd_coex = amlbt_chardev_coex_open();
    if (fd_coex < 0)
    {
        ALOGD("coex node open failed:%s", strerror(errno));
        return -1;
    }
    if (fd_coex > 0)
    {
        coex_status = aml_get_w2l_coex_status(fd_coex);
        close(fd_coex);
        fd_coex = -1;

        if ((coex_status & ZIGBEE_ALIVE) || (coex_status & THREAD_ALIVE))
        {
            ALOGD("15p4 alive");
        }
        else
        {
            ALOGD("15p4 not alive, init uart and bind tty!");
            g_userial_fd = userial_vendor_open((tUSERIAL_CFG *)&userial_init_cfg);
            if (g_userial_fd < 0)
            {
                ALOGD("uart open failed!");
                return -1;
            }
            else
            {
                aml_uart_init();
                aml_uart_get_pmu();
                aml_uart_rtl_dbg(0x00f0008c);
                aml_uart_rtl_dbg(0x00f02044);
                aml_uart_rtl_dbg(0x00f02078);
                aml_uart_rtl_dbg(0x00f02058);
                aml_uart_rtl_dbg(0x00205090);
                aml_uart_rtl_dbg(0x00205094);
                aml_uart_rtl_dbg(0x00205098);
                aml_uart_rtl_dbg(0x0020509c);
                aml_uart_rtl_dbg(0x002050a0);
                if (ioctl(g_userial_fd, TIOCSETD, &i) < 0)
                {
                    ALOGD("set line discipline failed! ");
                }
                else
                {
                    ALOGD("set line discipline success ");
                }
            }
        }
        bt_sdio_fd = userial_vendor_devchar_open();
        if (bt_sdio_fd < 0)
        {
            ALOGD("bluetooth node open failed!");
            return -1;
        }
        else
        {
            for (idx = 0; idx < CH_MAX; idx++)
            {
                (*fd_array)[idx] = bt_sdio_fd;
            }
            aml_get_w2l_chip_function(bt_sdio_fd);
        }
    }

    ALOGD("%s \n", __func__);
    return 1;
}

static void libbt_set_shutdown_value(void)
{
    ALOGD("%s %d\n", __func__, bt_sdio_fd);
    property_get(PWR_PROP_NAME, shutdwon_status, "unknown");

    if (strstr(shutdwon_status, "0userrequested") != NULL)
    {
        if (ioctl(bt_sdio_fd, IOCTL_SET_BT_SHUTDOWN, &bt_shutdown) != 0)
        {
            ALOGD("ioctl send failed: fd %d, error %s", bt_sdio_fd, strerror(errno));
        }
        else
        {
            ALOGD("send bt shutdown=%ld\n", bt_shutdown);
        }
    }
}

static void libbt_unregister_sdio(int fd)
{
    ALOGD("%s %d\n", __func__, fd_coex);
    property_get(PWR_PROP_NAME, shutdwon_status, "unknown");

    if (strstr(shutdwon_status, "0userrequested") == NULL)
    {
        if (ioctl(fd, IOCTL_UNREGISTER_SDIO, NULL) != 0)
        {
          ALOGD("ioctl send failed: fd %d, error %s", fd, strerror(errno));
        }
        else
        {
          ALOGD("w2l unregister sdio ok\n");
        }
    }
}


static int libbt_op_userial_close(int state, int (*fd_array)[])
{
/*    int cnt = 0;
    unsigned val = 0;
    unsigned char fw_pc[12] = {0};
    unsigned char fw_log[516] = {0};
    unsigned int fw_log_addr = 0x413b60;*/

    property_get(PWR_PROP_NAME, shutdwon_status, "unknown");
    ALOGD("%s %s ", __FUNCTION__, shutdwon_status);

/*    ALOGD("try to read pc...");
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
    }
    ALOGD("bt fw log end");
    save_regs_to_file(fw_pc, sizeof(fw_pc), "/data/vendor/fw_pc.txt");
    save_regs_with_time_str(fw_log, sizeof(fw_log), "/data/vendor/fw_log_last.txt");*/

    //reset fw
    if (hw_cfg_cb.state == 0)
    {
        aml_reset_bt(bt_sdio_fd);
    }
    //close bt fd
    if (bt_sdio_fd > 0)
    {
        libbt_set_shutdown_value();
        close(bt_sdio_fd);
        bt_sdio_fd = -1;
    }
    fd_coex = amlbt_chardev_coex_open();
    if (fd_coex > 0)
    {
        libbt_unregister_sdio(fd_coex);
        coex_status = aml_get_w2l_coex_status(fd_coex);
        close(fd_coex);
        fd_coex = -1;
        if ((coex_status & ZIGBEE_ALIVE) || (coex_status & THREAD_ALIVE))
        {
            ALOGD("15p4 alive");
        }
        else
        {
            ALOGD("15p4 not alive, close uart!");
            tcflush(g_userial_fd, TCIOFLUSH);
            if (g_userial_fd > 0)
            {
                close(g_userial_fd);
                g_userial_fd = -1;
            }
        }
    }
    else
    {
        ALOGD("coex node open failed:%s", strerror(errno));
        return -1;
    }

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

