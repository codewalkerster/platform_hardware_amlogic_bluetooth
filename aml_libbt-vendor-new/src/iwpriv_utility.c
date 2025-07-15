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
*  Filename:      iwpriv_utility.c
*
*  Description:   Amlogic vendor specific library implementation
*
******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/wireless.h>
#include <unistd.h>
#include <utils/Log.h>
#include <sys/stat.h>

static int sock;

typedef struct iw_priv_args iwprivargs;

static int bt_iwpriv_init(void)
{
    //ALOGD("%s: entry!\n", __func__);

    sock = socket(PF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        ALOGD("%s: bad sock!\n", __func__);
        return -1;
    }

    return 0;
}

static int bt_iwpriv_close(void)
{
    //ALOGD("%s: entry!\n", __func__);

    if (sock >= 0) {
        close(sock);
        sock = -1;
    }

    return 0;
}

static inline int iw_get_ext(int skfd,       /* Socket to the kernel */
       const char * ifname,                  /* Device name */
       int request,                          /* WE ID */
       struct iwreq * pwrq)                  /* Fixed part of the request */
{
    int ret = 0;
    /* Set device name */
    strncpy(pwrq->ifr_name, ifname, IFNAMSIZ);

    ret = ioctl(skfd, request, pwrq);
    if (ret < 0) {
        ALOGE(" %s: err = %s\n", __func__, strerror(errno));
    }
    /* Do the request */
    return ret;
}

/* Size (in bytes) of various events */
static const int priv_type_size[] = {
    0,                                        /* IW_PRIV_TYPE_NONE */
    1,                                        /* IW_PRIV_TYPE_BYTE */
    1,                                        /* IW_PRIV_TYPE_CHAR */
    0,                                        /* Not defined */
    sizeof(__u32),                            /* IW_PRIV_TYPE_INT */
    sizeof(struct iw_freq),                   /* IW_PRIV_TYPE_FLOAT */
    sizeof(struct sockaddr),                  /* IW_PRIV_TYPE_ADDR */
    0,                                        /* Not defined */
};

int iw_get_priv_size(int args)
{
    int num = args & IW_PRIV_SIZE_MASK;
    int type = (args & IW_PRIV_TYPE_MASK) >> 12;

    return(num * priv_type_size[type]);
}

int iw_get_priv_info(int skfd,
        const char * ifname,
        iwprivargs ** ppriv)
{
    struct iwreq wrq;
    iwprivargs * priv = NULL;   /* Not allocated yet */
    int maxpriv = 12;           /* Minimum for compatibility WE < 13 */
    iwprivargs * newpriv;

    /* Some driver may return a very large number of ioctls. Some
     * others a very small number. We now use a dynamic allocation
     * of the array to satisfy everybody. Of course, as we don't know
     * in advance the size of the array, we try various increasing
     * sizes. Jean II */
    do {
        //ALOGD("iw_get_priv_info maxpriv:%d\n", maxpriv);
        /* (Re)allocate the buffer */
        newpriv = realloc(priv, maxpriv * sizeof(priv[0]));
        if (newpriv == NULL) {
            ALOGE(" %s: Allocation failed\n", __func__);
            break;
        }
        priv = newpriv;

        /* Ask the driver if it's large enough */
        wrq.u.data.pointer = (caddr_t) priv;
        wrq.u.data.length = maxpriv;
        wrq.u.data.flags = 0;

        if (iw_get_ext(skfd, ifname, SIOCGIWPRIV, &wrq) >= 0) {
            /* Success. Pass the buffer by pointer */
            *ppriv = priv;
           /* Return the number of ioctls */
           return(wrq.u.data.length);
        }
        ALOGD(" %s: error ioctl[SIOCGIWPRIV], err = %s\n",
            __func__, strerror(errno));

        /* Failed. We probably need a bigger buffer. Check if the kernel
         * gave us any hints. */
        if (wrq.u.data.length > maxpriv) {
            maxpriv = wrq.u.data.length;
        } else {
           maxpriv *= 2;
        }
    }
    while (maxpriv < 1000);

    /* Cleanup */
    if (priv) {
        free(priv);
    }
    *ppriv = NULL;

    return(-1);
}

static int set_private_cmd(int skfd,  /* Socket */
        const char * args[],          /* Command line args */
        int count,                    /* Args count */
        const char * ifname,          /* Dev name */
        const char * cmdname,         /* Command name */
        iwprivargs * priv,            /* Private ioctl description */
        int priv_num,                 /* Number of descriptions */
        char *retval)                 /* Return value */
{
    struct iwreq wrq;
    char buffer[4096];    /* Only that big in v25 and later */
    int i = 0;              /* Start with first command arg */
    int k;                  /* Index in private description table */
    int temp;
    int subcmd = 0;         /* sub-ioctl index */
    int offset = 0;         /* Space for sub-ioctl index */

    /* Check if we have a token index.
     * Do it now so that sub-ioctl takes precedence, and so that we
     * don't have to bother with it later on... */
    if ((count >= 1) && (sscanf(args[0], "[%i]", &temp) == 1)) {
        subcmd = temp;
        args++;
        count--;
    }

    /* Search the correct ioctl */
    k = -1;
    while ((++k < priv_num) && strcmp(priv[k].name, cmdname));

    /* If not found... */
    if (k == priv_num) {
        ALOGD(" %s: Invalid command : %s\n", __func__, cmdname);
        return(-1);
    }

    /* Watch out for sub-ioctls ! */
    if (priv[k].cmd < SIOCDEVPRIVATE) {
        int j = -1;

        /* Find the matching *real* ioctl */
        while ((++j < priv_num) && ((priv[j].name[0] != '\0') ||
            (priv[j].set_args != priv[k].set_args) ||
            (priv[j].get_args != priv[k].get_args)));

        /* If not found... */
        if (j == priv_num) {
            ALOGD(" %s: Invalid private ioctl definition for : %s\n",
                __func__, cmdname);
            return(-1);
        }

        /* Save sub-ioctl number */
        subcmd = priv[k].cmd;
        /* Reserve one int (simplify alignment issues) */
        offset = sizeof(__u32);
        /* Use real ioctl definition from now on */
        k = j;

        ALOGD(" %s: <mapping sub-ioctl %s to cmd 0x%X-%d>\n", __func__,
            cmdname, priv[k].cmd, subcmd);
    }

    /* If we have to set some data */
    if ((priv[k].set_args & IW_PRIV_TYPE_MASK) &&
        (priv[k].set_args & IW_PRIV_SIZE_MASK)) {
        switch (priv[k].set_args & IW_PRIV_TYPE_MASK) {
        case IW_PRIV_TYPE_CHAR:
            if (i < count) {
                /* Size of the string to fetch */
                wrq.u.data.length = strlen(args[i]) + 1;
                if (wrq.u.data.length > (priv[k].set_args & IW_PRIV_SIZE_MASK)) {
                    wrq.u.data.length = priv[k].set_args & IW_PRIV_SIZE_MASK;
                }
                /* Fetch string */
                memcpy(buffer, args[i], wrq.u.data.length);
                buffer[sizeof(buffer) - 1] = '\0';
                i++;
            } else {
                wrq.u.data.length = 1;
                buffer[0] = '\0';
            }
            break;
        default:
            ALOGD(" %s: Not implemented...\n", __func__);
            return(-1);
        }
        if ((priv[k].set_args & IW_PRIV_SIZE_FIXED) &&
            (wrq.u.data.length != (priv[k].set_args & IW_PRIV_SIZE_MASK))) {
            ALOGD(" %s: The command %s needs exactly %d argument(s)...\n",
                __func__, cmdname, priv[k].set_args & IW_PRIV_SIZE_MASK);
            return(-1);
        }
    } else {    /* if args to set */
        wrq.u.data.length = 0L;
    }

    strncpy(wrq.ifr_name, ifname, IFNAMSIZ);

    /* Those two tests are important. They define how the driver
     * will have to handle the data */
    if ((priv[k].set_args & IW_PRIV_SIZE_FIXED) &&
        ((iw_get_priv_size(priv[k].set_args) + offset) <= IFNAMSIZ)) {
        /* First case : all SET args fit within wrq */
        if (offset) {
            wrq.u.mode = subcmd;
        }
        memcpy(wrq.u.name + offset, buffer, IFNAMSIZ - offset);
    } else {
        if ((priv[k].set_args == 0) &&
            (priv[k].get_args & IW_PRIV_SIZE_FIXED) &&
            (iw_get_priv_size(priv[k].get_args) <= IFNAMSIZ)) {
            /* Second case : no SET args, GET args fit within wrq */
            if (offset) {
                wrq.u.mode = subcmd;
            }
        } else {
            /* Third case : args won't fit in wrq, or variable number of args */
            wrq.u.data.pointer = (caddr_t) buffer;
            wrq.u.data.flags = subcmd;
        }
    }

#if 0
    ALOGD("[zhanghong] priv[%d].cmd is %x\n", k, priv[k].cmd);
    ALOGD("[zhanghong] wrq.u.data.length is %d\n", wrq.u.data.length);
    ALOGD("[zhanghong] wrq.ifr_name is %s\n", wrq.ifr_name);
    ALOGD("[zhanghong] wrq.u.mode is %u\n", wrq.u.mode);
    ALOGD("[zhanghong] wrq.u.data.pointer is %s\n", (char *)wrq.u.data.pointer);
    ALOGD("[zhanghong] wrq.u.data.flags is %d\n", wrq.u.data.flags);
#endif

    /* Perform the private ioctl */
    if (ioctl(skfd, priv[k].cmd, &wrq) < 0) {
        ALOGD(" %s: Interface doesn't accept private ioctl...\n", __func__);
        ALOGD(" %s: cmdname: %s, cmd: (%X): %s\n", __func__,
            cmdname, priv[k].cmd, strerror(errno));
        return(-1);
    }

    /* If we have to get some data */
    if ((priv[k].get_args & IW_PRIV_TYPE_MASK) &&
        (priv[k].get_args & IW_PRIV_SIZE_MASK)) {
        int n = 0;    /* number of args */

        ALOGD(" %s: ifname: %-8.16s, cmdname: %s\n", __func__,
            ifname, cmdname);

        /* Check where is the returned data */
        if ((priv[k].get_args & IW_PRIV_SIZE_FIXED) &&
            (iw_get_priv_size(priv[k].get_args) <= IFNAMSIZ)) {
            memcpy(buffer, wrq.u.name, IFNAMSIZ);
            n = priv[k].get_args & IW_PRIV_SIZE_MASK;
        } else {
            n = wrq.u.data.length;
        }

        switch (priv[k].get_args & IW_PRIV_TYPE_MASK) {
        case IW_PRIV_TYPE_CHAR:
            /* Display args */
            buffer[n] = '\0';
            ALOGD(" %s: buffer = %s\n", __func__, buffer);
            break;
        default:
            ALOGD(" %s: Not yet implemented...\n", __func__);
            return(-1);
        }
    } /* if args to set */

    strncpy(retval, buffer, sizeof(buffer));

    return(0);
}

void save_regs_to_file(unsigned char *buf, size_t len, const char *filepath)
{
    FILE *fp = fopen(filepath, "w");
    if (!fp)
    {
        ALOGE("save_regs_to_file error!");
        return;
    }

    for (size_t i = 0; i < len; ++i) {
        fprintf(fp, "%02x ", buf[i]);
    }
    fprintf(fp, "\n");

    fclose(fp);
    chmod(filepath, 0644);
    ALOGD("save file %s finished!", filepath);
}

void save_regs_with_time_str(unsigned char *buf, size_t len, const char *filepath)
{
    FILE *fp = fopen(filepath, "w");
    if (!fp) return;

    const char *time_str = "2025-05-26 15:09:34.989164 ";
    fprintf(fp, "%s", time_str);

    for (size_t i = 0; i < len; i++) {
        fprintf(fp, "%02x ", buf[i]);
    }

    fprintf(fp, "\n");
    fclose(fp);
    chmod(filepath, 0644);
}

unsigned int amlbt_get_reg(unsigned int addr)
{
    const char *ifname = "wlan0";
    iwprivargs *priv = NULL;
    int priv_num;
    char retval[4096] = {0};
    char addr_str[32];
    const char *args[1];
    int ret;
    unsigned int value = 0;
    char *hex_str;

    if (bt_iwpriv_init() != 0)
        return 0;

    priv_num = iw_get_priv_info(sock, ifname, &priv);
    if (priv_num <= 0 || priv == NULL) {
        bt_iwpriv_close();
        return 0;
    }

    snprintf(addr_str, sizeof(addr_str), "0x%08x", addr);
    args[0] = addr_str;

    ret = set_private_cmd(sock, args, 1, ifname, "get_reg", priv, priv_num, retval);
    if (ret != 0) {
        ALOGE("set_private_cmd err %d", ret);
        free(priv);
        bt_iwpriv_close();
        return 0;
    }
    ALOGD("addr:%#x, value:%s", addr, retval);

    hex_str = retval[0] == '&' ? retval + 1 : retval;
    value = (unsigned int)strtoul(hex_str, NULL, 16);

    free(priv);
    bt_iwpriv_close();
    return value;
}


