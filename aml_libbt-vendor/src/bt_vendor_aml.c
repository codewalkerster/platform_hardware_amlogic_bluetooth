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

//scan file
#define USB_DEVICE_DIR          "/sys/bus/usb/devices"
#define SDIO_DEVICE_DIR         "/sys/class/mmc_host"
#define W1U_VENDOR              0x414D
#define AML_VENDOR              0x1B8E
#define W1_PID                  0x8888

//hardware state
bt_hw_cfg_cb_t hw_cfg_cb;

//hal cback
bt_vendor_callbacks_t *bt_vendor_cbacks = NULL;

//chip type
aml_chip_type amlbt_transtype = {0};

//mac addr
uint8_t vnd_local_bd_addr[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

//attribute value
static const char DRIVER_PROP_NAME[] = "vendor.sys.amlbtsdiodriver";
static const char W1U_DRIVER_PROP_NAME[] = "vendor.sys.amlbt_w1u";
static const char CHIP_TYPE[] = "persist.vendor.bt_name";

#if (SCO_CFG_INCLUDED == TRUE)
    void hw_sco_config(void);
#endif
#if (HW_END_WITH_HCI_RESET == TRUE)
    void hw_epilog_process(void);
#endif

static int check_key_value(char *path, char *key, int value)
{
    FILE *fp;
    char newpath[100];
    char string_get[12];
    int value_int = 0;
    memset(newpath, 0, 100);
    sprintf(newpath, "%s/%s", path, key);

    if ((fp = fopen(newpath, "r")) != NULL)
    {
        ALOGD("check_key_value %s \n", newpath);
        memset(string_get, 0, sizeof(string_get));
        if (fgets(string_get, sizeof(string_get) - 1, fp) != NULL)
            ALOGE("string_get %s =%s\n", key, string_get);
        fclose(fp);
        value_int = strtol(string_get, NULL, 16);
        ALOGD("check_key_value value_int %#x, value %#x\n", value_int, value);
        if (value_int == value)
            return 1;
    }
    return 0;
}

static int get_key_value(char *path, char *key)
{
    FILE *fp;
    char newpath[100];
    char string_get[12];
    int value_int = 0;
    memset(newpath, 0, 100);
    sprintf(newpath, "%s/%s", path, key);
    if ((fp = fopen(newpath, "r")) != NULL)
    {
        ALOGD("get_key_value %s \n", newpath);
        memset(string_get, 0, sizeof(string_get));
        if (fgets(string_get, sizeof(string_get) - 1, fp) != NULL)
            ALOGE("string_get %s =%s\n", key, string_get);
        fclose(fp);
        value_int = strtol(string_get, NULL, 16);
        ALOGD("check_key_value value_int %#x\n", value_int);
        return value_int;
    }
    return 0;
}

static void scan_aml_usb_devices(char *path)
{
    char newpath[100];
    DIR *pdir;
    struct dirent *ptr;
    struct stat filestat;
    unsigned int w2 = 0;
    unsigned int pid = 0;

    if (stat(path, &filestat) != 0)
    {
        ALOGE("The file or path(%s) can not be get stat!\n", newpath);
        return ;
    }
    if ((filestat.st_mode & S_IFDIR) != S_IFDIR)
    {
        ALOGE("(%s) is not be a path!\n", path);
        return;
    }
    pdir = opendir(path);
    /*enter sub direc*/
    while ((ptr = readdir(pdir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
            continue;
        memset(newpath, 0, sizeof(newpath));
        sprintf(newpath, "%s/%s", path, ptr->d_name);
        ALOGD("[AML_USB] The file or path(%s)\n", newpath);
        if (stat(newpath, &filestat) != 0)
        {
            ALOGE("The file or path(%s) can not be get stat!\n", newpath);
            continue;
        }
        /* Check if it is path. */
        if ((filestat.st_mode & S_IFDIR) == S_IFDIR)
        {
            if (check_key_value(newpath, "idVendor", W1U_VENDOR))
            {
                amlbt_transtype.interface = AML_INTF_USB;
                amlbt_transtype.wireless = 0;
                amlbt_transtype.family_rev = AML_REV_E;
                amlbt_transtype.family_id = AML_W1U;
                amlbt_transtype.reserved = 0;
                closedir(pdir);
                ALOGD("[AML_USB] Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
                      amlbt_transtype.family_rev, amlbt_transtype.interface);
                return ;
            }
            else if (check_key_value(newpath, "idVendor", AML_VENDOR))
            {
                pid = get_key_value(newpath, "idProduct");
                amlbt_transtype.interface = pid & 0x07;
                amlbt_transtype.wireless = 0;
                amlbt_transtype.family_rev = (pid >> 6) & 0x03;
                amlbt_transtype.family_id = (pid >> 9) & 0x1f;
                amlbt_transtype.reserved = 0;
                closedir(pdir);
                ALOGD("[AML_USB] Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
                      amlbt_transtype.family_rev, amlbt_transtype.interface);
                return ;
            }
        }
    }
    closedir(pdir);
}

static int amlbt_sdio_check(char *subpathdst)
{
    unsigned int aml = 0;
    unsigned int pid = 0;
    unsigned int chip_type = 0;

    if (check_key_value(subpathdst, "vendor", AML_VENDOR))
    {
        pid = get_key_value(subpathdst, "device");
        amlbt_transtype.interface = pid & 0x07;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = (pid >> 6) & 0x07;
        amlbt_transtype.family_id = (pid >> 9) & 0x1f;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_SDIO] Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
        return 1;
    }
    else if (check_key_value(subpathdst, "vendor", W1_PID))
    {
        amlbt_transtype.interface = AML_INTF_SDIO;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_C;
        amlbt_transtype.family_id = AML_W1;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_SDIO] W1 Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
        return 1;
    }

    return 0;
}

static int scan_file_sys(char *path, int level)
{
    char newpath[100];
    DIR *pdir;
    struct dirent *ptr;
    int chip_find = 0;
    struct stat filestat;

    pdir = opendir(path);
    /*enter sub direc*/
    while ((ptr = readdir(pdir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
            continue;
        memset(newpath, 0, sizeof(newpath));
        sprintf(newpath, "%s/%s", path, ptr->d_name);
        ALOGD("[AML_SDIO] The file or path(%d:%s)\n", level, newpath);
        if (stat(newpath, &filestat) != 0)
        {
            ALOGE("The file 1 or path(%s) can not be get stat!\n", newpath);
            perror("1:");
            continue;
        }
        /* Check if it is path. */

        if (level > 0)
        {
            if ((filestat.st_mode & S_IFDIR) == S_IFDIR)
            {
                chip_find = scan_file_sys(newpath, level - 1);
                if (chip_find)
                {
                    closedir(pdir);
                    return chip_find;
                }
            }
        }
        else
        {
            chip_find = amlbt_sdio_check(newpath);
            if (!chip_find)
            {
                ALOGE("chip_find %d!\n", chip_find);
            }
            else
            {
                closedir(pdir);
                return chip_find;
            }
        }
    }
    closedir(pdir);
    return chip_find;
}

static void scan_aml_sdio_devices(char *path)
{
    scan_file_sys(path, 2);
}

static void amlbt_transtype_check_by_persist(void)
{
    char property[PROPERTY_VALUE_MAX] = {0};

    property_get(CHIP_TYPE, property, NULL);

    ALOGD("[AML_BT] %s %s", __FUNCTION__, property);
    if (strcmp("aml_w1", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_SDIO;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_C;
        amlbt_transtype.family_id = AML_W1;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W1 Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w1u_s", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_SDIO;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_E;
        amlbt_transtype.family_id = AML_W1U;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W1US Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w1u", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_USB;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_E;
        amlbt_transtype.family_id = AML_W1U;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W1UU Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w2_s", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_SDIO;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_A;
        amlbt_transtype.family_id = AML_W2;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W2S Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w2_p", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_PCIE;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_A;
        amlbt_transtype.family_id = AML_W2;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W2P Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w2_u", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_USB;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_A;
        amlbt_transtype.family_id = AML_W2;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W2U Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w2l_u", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_USB;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_A;
        amlbt_transtype.family_id = AML_W2L;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W2LU Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else if (strcmp("aml_w2l_s", property) == 0)
    {
        amlbt_transtype.interface = AML_INTF_SDIO;
        amlbt_transtype.wireless = 0;
        amlbt_transtype.family_rev = AML_REV_A;
        amlbt_transtype.family_id = AML_W2L;
        amlbt_transtype.reserved = 0;
        ALOGD("[AML_BT] W2LS Chip type(%d:%d:%d)\n", amlbt_transtype.family_id,
              amlbt_transtype.family_rev, amlbt_transtype.interface);
    }
    else
    {
        ALOGD("[AML_BT] No Chip persist\n");
    }
}

static void amlbt_transtype_check(void)
{
    scan_aml_usb_devices(USB_DEVICE_DIR);

    if (amlbt_transtype.family_id == AML_UNKNOWN)
    {
        scan_aml_sdio_devices(SDIO_DEVICE_DIR);
    }

    ALOGD("[AML_USB] amlbt_transtype_check(%d:%d:%d)\n", amlbt_transtype.family_id,
          amlbt_transtype.family_rev, amlbt_transtype.interface);
}

/*****************************************************************************
**
**   BLUETOOTH VENDOR INTERFACE LIBRARY FUNCTIONS
**
*****************************************************************************/
static int init(const bt_vendor_callbacks_t *p_cb, unsigned char *local_bdaddr)
{
    int ret = -1;
    ALOGI("%s %s\n", __func__, AML_LIBBT_VERSION);

    if (p_cb == NULL)
    {
        ALOGE("init failed with no user callbacks!");
        return ret;
    }

    load_aml_stack_conf();
    if (amlbt_fw_mode == FW_MODE_15P4_ONLY)
    {
        ALOGE("firmware 15.4 only!\n");
        return ret;
    }
#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
    ALOGW("*****************************************************************");
    ALOGW("*****************************************************************");
    ALOGW("** Warning - BT Vendor Lib is loaded in debug tuning mode!");
    ALOGW("**");
    ALOGW("** If this is not intentional, rebuild libbt-vendor.so ");
    ALOGW("** with VENDOR_LIB_RUNTIME_TUNING_ENABLED=FALSE and ");
    ALOGW("** check if any run-time tuning parameters needed to be");
    ALOGW("** carried to the build-time configuration accordingly.");
    ALOGW("*****************************************************************");
    ALOGW("*****************************************************************");
#endif
    amlbt_transtype_check_by_persist();
    if (amlbt_transtype.family_id == AML_UNKNOWN)
    {
        amlbt_transtype_check();
    }

    userial_vendor_init();
    upio_init();

    vnd_load_conf(VENDOR_LIB_CONF_FILE);

    ret = amlbt_interface_init();
    if (ret != 0)
    {
        ALOGE("init failed: %d \n", __LINE__);
        return ret;
    }

    /* store reference to user callbacks */
    bt_vendor_cbacks = (bt_vendor_callbacks_t *)p_cb;

    /* This is handed over from the stack */
    memcpy(vnd_local_bd_addr, local_bdaddr, 6);

    return 0;
}

/** Requested operations */
static int op(bt_vendor_opcode_t opcode, void *param)
{
    int retval = 0;
    ALOGD("op for %d", opcode);

    switch (opcode)
    {
        case BT_VND_OP_POWER_CTRL:
        {
            int *state = (int *)param;
            retval = libbt_interface[opcode](*state, NULL);
        }
        break;

        case BT_VND_OP_FW_CFG:
        {
            retval = libbt_interface[opcode](0, NULL);
        }
        break;

        case BT_VND_OP_SCO_CFG:
        {
#if (SCO_CFG_INCLUDED == TRUE)
            hw_sco_config();
#else
            return retval;
#endif
        }
        break;

        case BT_VND_OP_USERIAL_OPEN:
        {
            int (*fd_array)[] = (int (*)[])param;
            retval = libbt_interface[opcode](0, fd_array);
        }
        break;

        case BT_VND_OP_USERIAL_CLOSE:
        {
            retval = libbt_interface[opcode](0, NULL);
        }
        break;

        case BT_VND_OP_GET_LPM_IDLE_TIMEOUT:
        {
            uint32_t *timeout_ms = (uint32_t *)param;
            *timeout_ms = hw_lpm_get_idle_timeout();
        }
        break;

        case BT_VND_OP_LPM_SET_MODE:
        {
            uint8_t *mode = (uint8_t *)param;

            retval = libbt_interface[opcode](*mode, NULL);
            if (retval != 0)
            {
                return retval;
            }

            retval = hw_lpm_enable(*mode);
        }
        break;

        case BT_VND_OP_LPM_WAKE_SET_STATE:
        {
            uint8_t *state = (uint8_t *)param;
            uint8_t wake_assert = (*state == BT_VND_LPM_WAKE_ASSERT) ? \
                                  TRUE : FALSE;

            hw_lpm_set_wake_state(wake_assert);
        }
        break;

        case BT_VND_OP_SET_AUDIO_STATE:
        {
            retval = hw_set_audio_state((bt_vendor_op_audio_state_t *)param);
        }
        break;

        case BT_VND_OP_EPILOG:
        {
#if (HW_END_WITH_HCI_RESET == FALSE)
            if (bt_vendor_cbacks)
            {
                bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_SUCCESS);
            }
#else
            hw_epilog_process();
#endif
        }
        break;

        case BT_VND_OP_A2DP_OFFLOAD_START:
        case BT_VND_OP_A2DP_OFFLOAD_STOP:
        default:
            break;
    }

    return retval;
}

/** Closes the interface */
static void cleanup(void)
{
    ALOGD("cleanup");

    amlbt_interface_exit();
    upio_cleanup();

    bt_vendor_cbacks = NULL;
}

// Entry point of DLib
const bt_vendor_interface_t BLUETOOTH_VENDOR_LIB_INTERFACE =
{
    sizeof(bt_vendor_interface_t),
    init,
    op,
    cleanup
};
