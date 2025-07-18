/*
* Copyright (c) 202X Amlogic, Inc. All rights reserved.
*
* This source code is subject to the terms and conditions defined in the
* file 'LICENSE' which is part of this source code package.
*
* Description:
*/
#ifndef __W2_USB_BT_NEW_H__
#define __W2_USB_BT_NEW_H__

typedef struct
{
    struct cdev dev_cdev;
    int         dev_major;
    struct class *dev_class;
    struct device *dev_device;
    struct early_suspend early_suspend;
    unsigned char firmware_start;
    struct completion comp;
    const unsigned char *iccm_buf;
    const unsigned char *dccm_buf;
    unsigned int antenna;
    unsigned int fw_mode;
    unsigned int bt_sink;
    unsigned int pin_mux;
    unsigned int br_digit_gain;
    unsigned int edr_digit_gain;
    unsigned int fw_log;
    unsigned int driver_log;
    unsigned int factory;
    unsigned int system;
    unsigned int isolation;
    unsigned int rd_state; //read state

    //rx fifo
    gdsl_fifo_t *fw_type_fifo;
    gdsl_fifo_t *fw_evt_fifo;
    gdsl_fifo_t *fw_data_fifo;

    //temp store skb for consistent header & payload process
    spinlock_t dr_type_skb_queue_lock;
    spinlock_t dr_evt_skb_queue_lock;
    spinlock_t dr_data_skb_queue_lock;
    struct sk_buff_head dr_type_skb_queue;
    struct sk_buff_head dr_evt_skb_queue;
    struct sk_buff_head dr_data_skb_queue;
    //tx dr skb
    spinlock_t dr_tx_skb_queue_lock;
    struct sk_buff_head dr_tx_skb_queue;

    //tx fifo
    gdsl_fifo_t *tx_cmd_fifo;
    gdsl_tx_q_t tx_q[8]; //USB_TX_Q_NUM
    //rc manfdata
    unsigned char rc_manfdata[6*8];
    unsigned int manfdata_len;
    //mac addr
    unsigned char mac_addr[6];
    unsigned int sink_mode;
    unsigned int dr_state;
    unsigned char *usb_rx_buf;
    unsigned int usb_rx_len;
    unsigned int credit;
    struct work_struct write_work;
    unsigned char usb_irq_task_quit;
    wait_queue_head_t rd_wait_queue;
    struct mutex bt_credit_mutex;
    struct mutex bt_debug_mutex;
    //ioctl
    unsigned long recovery_value;
    unsigned long shutdown_value;
    //input dev
    unsigned int input_key;
    struct input_dev *amlbt_input_dev;
    struct device_link *link;
    //ktimer
    struct hrtimer poll_timer;
    ktime_t ktime;
    u64 wait_start;
    //irq
    int irq;
    struct work_struct rx_work;
    struct workqueue_struct *rx_work_wq;
} w2_usb_bt_new_t;

int amlbt_w2u_new_init(void);
void amlbt_w2u_new_exit(void);

#endif

