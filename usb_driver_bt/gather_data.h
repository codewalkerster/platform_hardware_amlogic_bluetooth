/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
#ifndef __GATHER_DATA_H__
#define __GATHER_DATA_H__

#define AML_GD_VERSION    "Gather data version:2024-10-17-1526"

typedef void (*sdio_wr_word_inf)(unsigned int addr, unsigned int data);
typedef unsigned int (*sdio_rd_word_inf)(unsigned int addr);

int amlbt_gather_data_init(struct device *dev, sdio_wr_word_inf p_wr_word_func, sdio_rd_word_inf p_rd_word_func);
void amlbt_gather_data_deinit(struct device *dev);

#endif

