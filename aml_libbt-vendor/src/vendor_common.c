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

#define TIMEOUT_TRYSUM    20
#define finit_module(fd, opts, flags) syscall(SYS_finit_module, fd, opts, flags)
int delete_module(const char *, unsigned int);

//w2l variable
bool exit_thread;
static int listenSocket = -1;
static int sessionSocket = -1;
long w2l_chip_func = 1;
pthread_t aml_15p4_handle_thread;

//file operator
int g_userial_fd = -1;
int bt_sdio_fd = -1;

//common array
const char PWR_PROP_NAME[] = "sys.shutdown.requested";
char shutdwon_status[PROPERTY_VALUE_MAX] = {0};
char driver_pram[PROPERTY_VALUE_MAX] = {0};

/*****************driver insmod && rmmod && check*****************/

int driver_check(const char *modname, int timeout_ms)
{
    FILE *modules;
    char line[256];
    char *module;
    int count = 1;

    if ((modules = fopen("/proc/modules", "r")) == NULL)
    {
        ALOGW("open /proc/modules failed! err=%s\n", strerror(errno));
        return 0;
    }

    do
    {
        while ((fgets(line, sizeof(line), modules)) != NULL)
        {
            module = strtok(line, " ");
            if (module == NULL)
            {
                fclose(modules);
                ALOGE("%s: module is NULL", __FUNCTION__);
                return 0;
            }
            if (!strcmp(module, modname))
            {
                fclose(modules);
                ALOGW("driver %s is detected! count=%d\n", modname, count);
                return 1;
            }
        }
        ALOGW("driver %s is not detected! count=%d, usleep(20ms)...\n", modname, count);
        count++;
        usleep(20 * 1000);
        timeout_ms -= 20;
    }
    while (timeout_ms > 0);

    fclose(modules);
    return 0;
}
int insmod_check(const char *modname)
{
    FILE *modules;
    char line[256];
    char *module;

    if ((modules = fopen("/proc/modules", "r")) == NULL)
    {
        ALOGW("open /proc/modules failed! err=%s\n", strerror(errno));
        return 0;
    }
    if ((fgets(line, sizeof(line), modules)) != NULL)
    {
        module = strtok(line, " ");
        if (module == NULL)
        {
            fclose(modules);
            ALOGE("%s: module is NULL", __FUNCTION__);
            return 0;
        }
        if (!strcmp(module, modname))
        {
            fclose(modules);
            ALOGW("driver %s is detected!\n", modname);
            return 1;
        }
    }
    else
    {
        ALOGW("driver %s is not detected!,\n", modname);
    }
    fclose(modules);
    return 0;
}

int insmod(const char *filename, const char *args,
                  const char *modname, int timeout_ms)
{
    int fd = -1;
    int ret;
    char value[PROPERTY_VALUE_MAX] = {'\0'};
    char buf[256];
    const char *p = NULL;

    if (insmod_check(modname))
    {
        ALOGD("[insmod]driver has already insmod: %s\n", buf);
        return 0;
    }
    memset(value, 0, sizeof(value));
    /* Note: you need disable selinux and gives chmod permission for
    ** driver files when specify the path of driver.
    ** e.g. setprop persist.vendor.wifibt_drv_path "/data/vendor"
    */
    if (property_get("persist.vendor.wifibt_drv_path", value, NULL))
    {
        if ((p = strrchr(filename, '/')))
        {
            memset(buf, 0, sizeof(buf));
            snprintf(buf, sizeof(buf), "%s%s", value, p);
            ALOGD("[insmod]driver: %s\n", buf);
            if ((fd = open(buf, O_RDONLY)) < 0)
            {
                ALOGE("[insmod]open: %s failed! %s\n", buf, strerror(errno));
            }
        }
    }
    if (fd < 0)
    {
        ALOGD("[insmod]driver: %s\n", filename);
        if ((fd = open(filename, O_RDONLY)) < 0)
        {
            ALOGE("[insmod]open: %s failed! %s \n", filename, strerror(errno));
            return -1;
        }
    }
    ret = finit_module(fd, args, 0);
    close(fd);
    if (ret < 0)
    {
        ALOGE("[insmod]finit_module failed! %s\n", strerror(errno));
    }
    if (!driver_check(modname, timeout_ms))
    {
        return -1;
    }
    return 0;
}

int rmmod(const char *modname, int timeout_ms)
{
    int ret = -1;
    int count = 1;

    do
    {
        ret = delete_module(modname, O_NONBLOCK | O_EXCL);
        if (ret < 0 && errno == EAGAIN)
        {
            usleep(20 * 1000);
        }
        else
        {
            if (!driver_check(modname, 20))
            {
                ret = 0;
                break;
            }
        }
        timeout_ms -= 20;
    }
    while (timeout_ms > 0);

    if (ret != 0)
        ALOGE("[rmmod]Unable to unload driver module %s!\n", modname);
    return ret;
}

/*****************read event && write cmd*****************/

int do_write(int fd, unsigned char *buf, int len)
{
    int ret = 0;
    int write_offset = 1;
    int write_len = len - 1;

    if (len <= 0)
    {
        ALOGE("Invalid write length: %d", len);
        return -1;
    }
    ret = write(fd, buf, 1);
    if (ret < 0)
    {
        ALOGE("write failed ret = %d", ret);
        return -1;
    }
    else if (ret == 0)
    {
        ALOGE("write failed with ret 0");
        return 0;
    }
    while (write_len > 0)
    {
        ret = write(fd, buf + write_offset, write_len);
        if (ret < 0)
        {
            ALOGE("write failed ret = %d", ret);
            return -1;
        }
        else if (ret == 0)
        {
            ALOGE("write failed with ret 0");
            return 0;
        }
        write_len -= ret;
        write_offset += ret;
        if (write_len)
        {
            ALOGE("Write pending, write_len = %d, write_offset = %d, ret = %d", write_len, write_offset, ret);
        }
    }
    ALOGD("Write success, write_len = %d, write_offset = %d, ret = %d", write_len, write_offset, ret);

    return len;
}

/*******************************************************************************
**
** Function        read_hci_event
**
** Description     Read HCI event during vendor initialization
**
** Returns         int: size to read
**
*******************************************************************************/
int read_hci_event(int fd, unsigned char *buf, int size)
{
    int remain, r;
    int count = 0;
    int retry_cnt = 0;

    if (size <= 0)
    {
        ALOGE("Invalid size argument!");
        return -1;
    }

    //ALOGI("%s: Wait for Command Compete Event from SOC", __FUNCTION__);

    /* The first byte identifies the packet type. For HCI event packets, it
     * should be 0x04, so we read until we get to the 0x04. */
    while (1)
    {
        r = read(fd, buf, 1);
        /*if (r <= 0)
        {
            ALOGE("read_hci_event err: %s \n", strerror(errno));
            return -1;
        }*/

        if (buf[0] == 0x04)
        {
            break;
        }
        usleep(5000);
        if (retry_cnt > TIMEOUT_TRYSUM*5)
        {
            ALOGE("Rx Hci command pkt typetimeout!\n");
            return -1;
        }
        retry_cnt++;
        //ALOGD("TYPE %d", retry_cnt);
    }
    count++;

    /* The next two bytes are the event code and parameter total length. */
    while (count < 3)
    {
        r = read(fd, buf + count, 3 - count);
        if (r <= 0)
            return -1;
        count += r;
        usleep(5000);
        if (retry_cnt > TIMEOUT_TRYSUM*10)
        {
            ALOGE("Rx Hci command pkt headtimeout!\n");
            return -1;
        }
        retry_cnt++;
        //ALOGD("head %d", retry_cnt);
    }
    /* Now we read the parameters. */
    if (buf[2] < (size - 3))
        remain = buf[2];
    else
        remain = size - 3;
    while ((count - 3) < remain)
    {
        r = read(fd, buf + count, remain - (count - 3));
        if (r <= 0)
            return -1;
        count += r;
        usleep(5000);
        if (retry_cnt > TIMEOUT_TRYSUM*15)
        {
            ALOGE("Rx Hci command pkt playloadtimeout!\n");
            return -1;
        }
        retry_cnt++;
        //ALOGD("playload %d", retry_cnt);
    }
    return count;
}

int aml_hci_send_cmd(int fd, unsigned char *cmd, int cmdsize, unsigned char *rsp)
{
    int err = 0;

    //ALOGD("%s [abner test]: ", __FUNCTION__);
    err = do_write(fd, cmd, cmdsize);
    if (err != cmdsize)
    {
        ALOGE("%s: Send failed with ret value: %d", __FUNCTION__, err);
        err = -1;
        goto error;
    }

    usleep(20000);

    memset(rsp, 0, HCI_MAX_EVENT_SIZE);

    /* Wait for command complete event */
    while (1)
    {
        //Wait for command complete event
        err = read_hci_event(fd, rsp, HCI_MAX_EVENT_SIZE);
        ALOGD("aml_hci_send_cmd read rsp [%#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x, %#x]",
            rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6], rsp[7], rsp[8], rsp[9], rsp[10], rsp[11]);
        if (err < 0)
        {
            ALOGE("%s: Failed to set patch info on Controller", __FUNCTION__);
            goto error;
        }

        if (rsp[0] == 0x04 && (rsp[1] == 0x0e || rsp[1] == 0x19))
        {
            break;
        }
    }
error:
    return err;
}

int hci_write_cmd(int fd, unsigned char *buf, int len)
{
    int ret = 0;
    int write_offset = 1;
    int write_len = len - 1;

    if (len <= 0)
    {
        ALOGE("Invalid write length: %d", len);
        return -1;
    }
    ret = write(fd, buf, 1);
    if (ret < 0)
    {
        ALOGE("write failed ret = %d", ret);
        return -1;
    }
    else if (ret == 0)
    {
        ALOGE("write failed with ret 0");
        return 0;
    }
    while (write_len > 0)
    {
        ret = write(fd, buf + write_offset, write_len);
        if (ret < 0)
        {
            ALOGE("write failed ret = %d", ret);
            return -1;
        }
        else if (ret == 0)
        {
            ALOGE("write failed with ret 0");
            return 0;
        }
        write_len -= ret;
        write_offset += ret;
        if (write_len)
        {
            ALOGE("Write pending, write_len = %d, write_offset = %d, ret = %d", write_len, write_offset, ret);
        }
    }
    ALOGD("Write success, write_len = %d, write_offset = %d, ret = %d", write_len, write_offset, ret);

    return len;
}

int hci_read_event(int fd, unsigned char *buf, int size)
{
    int remain, r;
    int count = 0;
    int retry_cnt = 0;

    if (size <= 0)
    {
        ALOGD("Invalid size argument!");
        return -1;
    }

    //log_trace_msg(__FILE__, __LINE__, "%s: Wait for Command Compete Event from SOC", __FUNCTION__);

    /* The first byte identifies the packet type. For HCI event packets, it
     * should be 0x04, so we read until we get to the 0x04. */
    while (retry_cnt < 40)
    {
        r = read(fd, buf, 1);
        if (r <= 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                ALOGD("read_hci_event err: %s \n", strerror(errno));
                return -1;
            }
            else
            {
                usleep(1000);
                ALOGD("hci_read_event retry_cnt %d", retry_cnt);
                retry_cnt++;
                continue;
            }
        }

        if (buf[0] == 0x04)
            break;
    }
    if (retry_cnt >= 40)
    {
        ALOGD("hci_read_event err: retry count max!\n");
        return -1;
    }
    count++;
    retry_cnt = 0;
    /* The next two bytes are the event code and parameter total length. */
    while (count < 3)
    {
        r = read(fd, buf + count, 3 - count);
        if (r <= 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                ALOGD("read_hci_event header err: %s \n", strerror(errno));
                return -1;
            }
            else
            {
                usleep(1000);
                ALOGD("hci_read_event header retry_cnt %d", retry_cnt);
                retry_cnt++;
                if (retry_cnt >= 40)
                {
                    ALOGD("hci_read_event header err: retry count max!\n");
                    return -1;
                }
                continue;
            }
        }
        count += r;
    }

    /* Now we read the parameters. */
    if (buf[2] < (size - 3))
        remain = buf[2];
    else
        remain = size - 3;
    retry_cnt = 0;
    while ((count - 3) < remain)
    {
        r = read(fd, buf + count, remain - (count - 3));
        if (r <= 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                ALOGD("read_hci_event payload err: %s \n", strerror(errno));
                return -1;
            }
            else
            {
                usleep(1000);
                ALOGD("hci_read_event payload retry_cnt %d", retry_cnt);
                retry_cnt++;
                if (retry_cnt >= 40)
                {
                    ALOGD("hci_read_event payload err: retry count max!\n");
                    return -1;
                }
                continue;
            }
        }
        count += r;
    }
    return count;
}

/*****************libbt send hci cmd*****************/

int aml_woble_configure(int fd)
{
    unsigned char rsp[HCI_MAX_EVENT_SIZE];
    unsigned char reset_cmd[] = {0x01, 0x03, 0x0C, 0x00};
    unsigned char read_BD_ADDR[] = {0x01, 0x09, 0x10, 0x00};
    unsigned char APCF_config_manf_data[] = {0x01, 0x22, 0xFC, 0x05, 0x19, 0xff, 0x01, 0x0a, 0xb};

    unsigned char APCF_enable[] = {0x01, 0x57, 0xFD, 0x02, 0x00, 0x01};
    unsigned char le_set_evt_mask[] = {0x01, 0x01, 0x20, 0x08, 0x7F, 0x1A, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char le_scan_param_setting[] = {0x01, 0x0b, 0x20, 0x07, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00};
    unsigned char le_scan_enable[] = {0x01, 0x0c, 0x20, 0x02, 0x01, 0x00};
    unsigned char host_sleep_VSC[] = {0x01, 0x21, 0xfc, 0x01, 0x01};

    aml_hci_send_cmd(fd, (unsigned char *)reset_cmd, sizeof(reset_cmd), (unsigned char *)rsp);
    aml_hci_send_cmd(fd, (unsigned char *)host_sleep_VSC, sizeof(host_sleep_VSC), (unsigned char *)rsp);
    aml_hci_send_cmd(fd, (unsigned char *)APCF_config_manf_data, sizeof(APCF_config_manf_data), (unsigned char *)rsp);
    aml_hci_send_cmd(fd, (unsigned char *)le_set_evt_mask, sizeof(le_set_evt_mask), (unsigned char *)rsp);
    aml_hci_send_cmd(fd, (unsigned char *)le_scan_param_setting, sizeof(le_scan_param_setting), (unsigned char *)rsp);
    aml_hci_send_cmd(fd, (unsigned char *)le_scan_enable, sizeof(le_scan_enable), (unsigned char *)rsp);

    return 0;
}

void aml_reset_bt(int fd)
{
    unsigned char reset_cmd[] = {0x01, 0x03, 0x0C, 0x00};
    unsigned char rsp[HCI_MAX_EVENT_SIZE];

    ALOGD("aml_reset_bt \n");
    aml_hci_send_cmd(fd, (unsigned char *)reset_cmd, sizeof(reset_cmd), (unsigned char *)rsp);
    ALOGD("aml_reset_bt end\n");
}

/*****************ioctl*****************/

unsigned char aml_get_w2l_coex_status(int fd)
{
    unsigned char coex_status = 0;

    if (ioctl(fd, IOCTL_GET_COEX_STATUS, &coex_status) != 0)
    {
        ALOGD("ioctl send failed: fd %d, error %s, coex_status %ld", fd, strerror(errno), coex_status);
    }
    else
    {
        ALOGD("w2l coex status=%d\n", coex_status);
    }

    return coex_status;
}

void aml_get_w2l_chip_function(int fd)
{
    if (ioctl(fd, IOCTL_GET_DEVICE_PID, &w2l_chip_func) != 0)
    {
        ALOGD("ioctl send failed: fd %d, error %s, revData %ld", fd, strerror(errno), w2l_chip_func);
    }
    else
    {
        ALOGD("receive chip id=%ld\n", w2l_chip_func);
    }
}

/*****************uart update baud*****************/

int aml_uart_init(void)
{
    unsigned char cmd[16] = {0x01, 0};
    unsigned char rsp[HCI_MAX_EVENT_SIZE] = {0};
    unsigned char *p = &cmd[1];
    int err = 0;
    unsigned char expected_rsp[] = {0x4,0xe,0x4,0x1,0xf2,0xfe};
    int retry = 0;

    userial_vendor_set_baud(line_speed_to_userial_baud(115200));

    UINT16_TO_STREAM(p, TCI_UPDATE_UART_BAUDRATE);
    *p++ = 8;
    UINT32_TO_STREAM(p, 0xa30128);
#ifdef UART_4M
#ifdef FPGA_ENABLE
    UINT32_TO_STREAM(p, 0x7005);//FPGA 4M
#else
    UINT32_TO_STREAM(p, 0x7009);//ARM 4M
#endif
#endif
#ifdef UART_2M
#ifdef FPGA_ENABLE
    UINT32_TO_STREAM(p, 0x700b);//FPGA 2M
#endif
#endif
    while (retry < 3)
    {
        if (hci_write_cmd(g_userial_fd, cmd, 12) != 12)
        {
            ALOGD("Send failed with ret value: %d", err);
            continue;
        }

        while (1)
        {
            /* Wait for command complete event */
            err = hci_read_event(g_userial_fd, rsp, HCI_MAX_EVENT_SIZE);
            if (err < 0)
            {
                ALOGD("Failed to read rsp from RTL!");
                break;
            }
            else
            {
                ALOGD("rsp:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]",  \
                    rsp[0],rsp[1],rsp[2],rsp[3],rsp[4],rsp[5],rsp[6],rsp[7]);
                break;
            }
        }

        if (!memcmp(expected_rsp, rsp, sizeof(expected_rsp)))
        {
            break;
        }
        else
        {
            upio_set_bluetooth_power(UPIO_BT_POWER_OFF);
            usleep(50000);
            upio_set_bluetooth_power(UPIO_BT_POWER_ON);
            //usleep(200000);
            retry++;
            ALOGD("uart set baud failed!!!!!!!!!!!!!!!!!!!!!!!!, retry %d", retry);
        }
    }

    if (retry >= 3)
    {
        ALOGD("userial_vendor_set_baud FAILED!");
        return -1;
    }
#ifdef UART_4M
    userial_vendor_set_baud(\
                            line_speed_to_userial_baud(4000000) \
                           );
#endif
#ifdef UART_2M
    userial_vendor_set_baud(\
                            line_speed_to_userial_baud(2000000) \
                           );
#endif
    ALOGD("userial_vendor_set_baud SUCCESS");
    return 0;
}

/*****************debug bt pmu && rtl*****************/

int aml_uart_get_pmu(void)
{
    unsigned char cmd[16] = {0x01, 0};
    unsigned char rsp[HCI_MAX_EVENT_SIZE] = {0};
    unsigned char *p = &cmd[1];
    int err = 0;
    unsigned char expected_rsp[] = {0x4,0xe,0x8,0x1,0xf0,0xfe,0,0x6};

    UINT16_TO_STREAM(p, TCI_READ_REG);
    *p++ = 4;
    UINT32_TO_STREAM(p, 0xf02078);
    if (hci_write_cmd(g_userial_fd, cmd, 8) != 8)
    {
        ALOGE("Send failed with ret value: %d", err);
        return -1;
    }

    while (1)
    {
        /* Wait for command complete event */
        err = hci_read_event(g_userial_fd, rsp, HCI_MAX_EVENT_SIZE);
        if (err < 0)
        {
            ALOGD("Failed to read rsp from RTL!");
            break;
        }
        else
        {
            ALOGD("rsp:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]",  \
                rsp[0],rsp[1],rsp[2],rsp[3],rsp[4],rsp[5],rsp[6],rsp[7]);
            break;
        }
    }

    //if (!memcmp(expected_rsp, rsp, sizeof(expected_rsp)))
    if (rsp[7] != 6)
    {
        ALOGD("aml_uart_get_pmu failed");
        ALOGD("rsp:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]",  \
                rsp[0],rsp[1],rsp[2],rsp[3],rsp[4],rsp[5],rsp[6],rsp[7]);
        ALOGD("expected_rsp:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]",  \
                expected_rsp[0],expected_rsp[1],expected_rsp[2],expected_rsp[3],
                expected_rsp[4],expected_rsp[5],expected_rsp[6],expected_rsp[7]);
        return -1;
    }

    ALOGD("aml_uart_get_pmu SUCCESS");
    return 0;
}

int aml_uart_rtl_dbg(unsigned int addr)
{
    unsigned char cmd[16] = {0x01, 0};
    unsigned char rsp[HCI_MAX_EVENT_SIZE] = {0};
    unsigned char *p = &cmd[1];
    int err = 0;

    UINT16_TO_STREAM(p, TCI_READ_REG);
    *p++ = 4;
    UINT32_TO_STREAM(p, addr);
    if (hci_write_cmd(g_userial_fd, cmd, 8) != 8)
    {
        ALOGE("Send failed with ret value: %d", err);
        return -1;
    }
    ALOGD("aml_uart_rtl_dbg %#x", addr);
    while (1)
    {
        /* Wait for command complete event */
        err = hci_read_event(g_userial_fd, rsp, HCI_MAX_EVENT_SIZE);
        if (err < 0)
        {
            ALOGD("Failed to read rsp from RTL!");
            break;
        }
        else
        {
            ALOGD("rsp:[%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x,%#x]",  \
                rsp[0],rsp[1],rsp[2],rsp[3],rsp[4],rsp[5],rsp[6],rsp[7],rsp[8],rsp[9],rsp[10],rsp[11]);
            break;
        }
    }

    ALOGD("aml_uart_rtl_dbg end");
    return 0;
}

/* | HCI_ZIGBEE_FLAG | MHDL |           MID        | BODY LENGTH | checksum |
*  |   0x10 1 Byts   | 0xFA | command id 1 Bytes   |    2 Bytes  |  2 Bytes |
*/
void aml_15p4_data_cb(void *p)
{
    unsigned char *p_mem = (unsigned char *)p;
    unsigned short len = ((p_mem[3] << 8) | p_mem[2]);
    int size;
    unsigned char buf[AML_15P4_SOCKET_SIZE] = {0};

    buf[0] = 0x10;
    memcpy(&buf[1], p_mem, len + 4 + 2);
    if (sessionSocket != -1)
    {
        size = send(sessionSocket, buf, len + 4 + 2 + 1, 0);
        ALOGI("%s rsp %d \n", __func__, size);
        if (size != len + 4 + 2 + 1)
        {
            ALOGI("%s packed error exit\n", __func__);
        }
    }
}

void aml_15p4_deinit(void)
{
    ALOGI("%s", __func__);
    if (pthread_join(aml_15p4_handle_thread, NULL) != 0)
    {
        perror("pthread_join error:");
    }
}

void* aml_15p4_socket(void* arg)
{
    struct sockaddr_un socket_name;
    int sndbuf_size = 8 * 1024;
    int rcvbuf_size = 8 * 1024;
    int data_len;
    int read_len = 0;
    fd_set read_fd;
    fd_set error_fd;
    int max_fd = -1;
    int rval;
    struct timeval timeout = {0, 5000};
    char _15p4_buf[AML_15P4_CMD_BUF_SIZE] = {0};
    unsigned char close_socket[] = {0xff, 0xff, 0xff, 0xaa, 0x55};
    unsigned char get_chip_func[] = {0xff, 0xff, 0xff, 0xaa, 0x11};
    unsigned char socket_rsp[] = {0xff, 0xff, 0xff, 0xaa, 0x11};
    int newSessionSocket;

    if ((listenSocket = socket(AF_UNIX, SOCK_STREAM | O_NONBLOCK, 0)) == -1) {
        ALOGE("socket create failed");
        return NULL;
    }

    if (setsockopt(listenSocket, SOL_SOCKET, SO_SNDBUF, &sndbuf_size, sizeof(sndbuf_size)) == -1)
    {
        ALOGE("setsockopt SO_SNDBUF failed");
        goto done;
    }
    if (setsockopt(listenSocket, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) == -1)
    {
        ALOGE("setsockopt SO_RCVBUF failed");
        goto done;
    }


    memset(&socket_name, 0, sizeof(struct sockaddr_un));
    socket_name.sun_family = AF_UNIX;
    strncpy(socket_name.sun_path, SOCKET_PATH, sizeof(socket_name.sun_path) - 1);

    unlink(socket_name.sun_path);

    if (bind(listenSocket, (const struct sockaddr *)&socket_name, sizeof(struct sockaddr_un)) == -1) {
        ALOGE("bind failed");
        perror("bind failed:");
        goto done;
    }
    if (chmod(SOCKET_PATH, 0770) == -1) {
        ALOGE("chmod failed");
        goto done;
    }
    if (listen(listenSocket, 5) == -1) {
        ALOGE("listen failed");
        goto done;
    }

    ALOGD("socket is listening on %s\n", SOCKET_PATH);

    while (!exit_thread)
    {
        timeout.tv_sec = 0;
        timeout.tv_usec = 5000;
        FD_ZERO(&read_fd);
        FD_ZERO(&error_fd);
        max_fd = -1;
        if (listenSocket != -1)
        {
            FD_SET(listenSocket, &read_fd);
            FD_SET(listenSocket, &error_fd);
        }
        if (sessionSocket != -1)
        {
            FD_SET(sessionSocket, &read_fd);
            FD_SET(sessionSocket, &error_fd);
        }
        max_fd = sessionSocket > listenSocket ? sessionSocket : listenSocket;
        rval = select(max_fd + 1, &read_fd, NULL, &error_fd, &timeout);
        if (rval <= 0)
            continue;
        if (FD_ISSET(listenSocket, &error_fd))
        {
            ALOGE("socket error");
            goto done;
        }

        if (FD_ISSET(listenSocket, &read_fd))
        {
            newSessionSocket = accept(listenSocket, NULL, NULL);
            if (newSessionSocket == -1)
            {
                ALOGE("accept failed");
                goto done;
            }
            if (sessionSocket != -1)
            {
                ALOGE("close pre scoket");
                close(sessionSocket);
            }
            ALOGE("session socket is ready");
            sessionSocket = newSessionSocket;
        }

        if (FD_ISSET(sessionSocket, &error_fd))
        {
            ALOGE("close session scoket");
            close(sessionSocket);
            sessionSocket = -1;
        }

        if (FD_ISSET(sessionSocket, &read_fd))
        {
            memset(_15p4_buf, 0, sizeof(_15p4_buf));
            data_len = recv(sessionSocket, _15p4_buf, 5, 0);
            if (data_len < 0)
            {
                ALOGI("%s read error exit %s\n", __func__, strerror(errno));
                break;
            }
            while (data_len && data_len < 5)
            {
              read_len = recv(sessionSocket, &_15p4_buf[data_len], 5 - data_len, 0);
              data_len += read_len;
            }
            if (!memcmp(close_socket, _15p4_buf, sizeof(close_socket)))
            {
                ALOGD("socket close");
                if (send(sessionSocket, close_socket, sizeof(close_socket), 0) != sizeof(close_socket))
                {
                    ALOGE("%s send close socket rsp error\n", __func__);
                }
                close(sessionSocket);
                sessionSocket = -1;
            }
            else if (!memcmp(get_chip_func, _15p4_buf, sizeof(get_chip_func)))
            {
                ALOGI("get chip function");
                socket_rsp[4] = (w2l_chip_func & 0xff);
                if (send(sessionSocket, socket_rsp, sizeof(socket_rsp), 0) != sizeof(socket_rsp))
                {
                    ALOGE("%s send chip function rsp error\n", __func__);
                }
            }
            else if ((*(unsigned short*)&_15p4_buf[0]) == 0xfa10)
            {
                if ((w2l_chip_func & 1) == 0)
                {
                    ALOGD("15.4 head:len %d,[%#x,%#x,%#x,%#x,%#x]",data_len, _15p4_buf[0], _15p4_buf[1],
                        _15p4_buf[2], _15p4_buf[3], _15p4_buf[4]);
                    read_len = *(unsigned short*)&_15p4_buf[3];
                    ALOGD("15p4 payload len = %d ", read_len);
                    data_len = recv(sessionSocket, &_15p4_buf[5], read_len + 2, 0);
                    if (data_len > 0)
                    {
                        ALOGD("15p4 data len = %d ", read_len + 6);
                        aml_15p4_tx(&_15p4_buf[1], read_len + 6);
                    }
                }
                else
                {
                    ALOGD("chip function %ld, not support 15.4,drop frame!", w2l_chip_func);
                }
            }
        }
    }
done:
    ALOGD("Server exit\n");
    if (listenSocket != -1)
    {
        close(listenSocket);
    }
    if (sessionSocket != -1)
    {
        close(sessionSocket);
    }
    unlink(SOCKET_PATH);
    return NULL;
}



