/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
#ifndef AMLBT_UTILS_H
#define AMLBT_UTILS_H

typedef struct
{
    unsigned int utils_start;
    unsigned int print_cnt;
    struct hrtimer hrtimer;
    struct workqueue_struct *wq;
    struct work_struct work;
    int (*read_word)(unsigned int addr, unsigned int ep, unsigned int *value);
    int (*write_word)(unsigned int addr,unsigned int data, unsigned int ep);
    int (*write_sram)(unsigned char *buf, unsigned char *sram_addr, unsigned int len, unsigned int ep);
    int (*read_sram)(unsigned char *buf,unsigned char *sram_addr, unsigned int len, unsigned int ep);
} w2l_usb_utils_bt_t;

void amlbt_utils_w2l_usb_init(
    int (*read_word)(unsigned int addr, unsigned int ep, unsigned int *value),
    int (*write_word)(unsigned int addr,unsigned int data, unsigned int ep),
    int (*write_sram)(unsigned char *buf, unsigned char *sram_addr, unsigned int len, unsigned int ep),
    int (*read_sram)(unsigned char *buf,unsigned char *sram_addr, unsigned int len, unsigned int ep));

void amlbt_utils_w2l_usb_deinit(void);

#endif

