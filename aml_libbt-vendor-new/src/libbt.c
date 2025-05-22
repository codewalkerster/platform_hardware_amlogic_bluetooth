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
#include "libbt.h"

libbt_func_t *libbt_interface = NULL;

/*****************vendor interface init*****************/

int amlbt_interface_init(void)
{
    if (amlbt_transtype.interface == AML_INTF_USB)
    {
        if (amlbt_transtype.family_id == AML_W1U)
        {
            ALOGD("%s %s\n", __func__, AML_W1UU_VERSION);
            libbt_interface = libbtw1u_usb_func;
        }
        else if (amlbt_transtype.family_id == AML_W2)
        {
            ALOGD("%s %s\n", __func__, AML_W2U_VERSION);
            libbt_interface = libbtw2_usb_func;
        }
        else if (amlbt_transtype.family_id == AML_W2L)
        {
            ALOGD("%s %s\n", __func__, AML_W2LU_VERSION);
            libbt_interface = libbtw2l_usb_func;
        }
    }
    else if (amlbt_transtype.interface == AML_INTF_SDIO)
    {
        if (amlbt_transtype.family_id == AML_W1)
        {
            ALOGD("%s %s\n", __func__, AML_W1_VERSION);
            libbt_interface = libbtw1_func;
        }
        else if (amlbt_transtype.family_id == AML_W1U)
        {
            ALOGD("%s %s\n", __func__, AML_W1US_VERSION);
            libbt_interface = libbtw1u_uart_func;
        }
        else if (amlbt_transtype.family_id == AML_W2)
        {
            ALOGD("%s %s\n", __func__, AML_W2S_VERSION);
            libbt_interface = libbtw2_uart_func;
        }
        else if (amlbt_transtype.family_id == AML_W2L)
        {
            ALOGD("%s %s\n", __func__, AML_W2LS_VERSION);
            libbt_interface = libbtw2l_uart_func;
        }
    }
    else if (amlbt_transtype.interface == AML_INTF_PCIE)
    {
        ALOGD("%s %s\n", __func__, AML_W2P_VERSION);
        libbt_interface = libbtw2_uart_func;
    }
    else
    {
        ALOGE("%s: amlbt_transtype is NULL", __FUNCTION__);
        return -1;
    }
    return 0;
}

void amlbt_interface_exit(void)
{
    libbt_interface = NULL;
}


