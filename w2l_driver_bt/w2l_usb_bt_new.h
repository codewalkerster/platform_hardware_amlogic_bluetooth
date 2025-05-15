#ifndef __W2L_USB_BT_NEW_H__
#define __W2L_USB_BT_NEW_H__

#define HCI_TYPE_ZIGBEE   0xfa
#define HCI_TYPE_THREAD   0xfa

#define AML_W2LU_MAX_COEX_DEVICES   4

typedef struct
{
    struct cdev dev_cdev[AML_W2LU_MAX_COEX_DEVICES];
    int         dev_major;
    struct class *dev_class;
    struct device *dev_device[AML_W2LU_MAX_COEX_DEVICES];
    struct early_suspend early_suspend;
    unsigned char bt_start;
    unsigned char zigbee_start;
    unsigned char thread_start;
    struct sk_buff_head tx_queue;
    struct work_struct  write_work;
    struct sk_buff_head bt_rx_queue;
    struct sk_buff_head zigbee_rx_queue;
    struct sk_buff_head thread_rx_queue;
    wait_queue_head_t zigbee_wait_queue;
    wait_queue_head_t thread_wait_queue;
    unsigned int zigbee_rd_state;
    unsigned int thread_rd_state;
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
    unsigned int rd_state; //read state
    //rx fifo
    gdsl_fifo_t *fw_type_fifo;
    gdsl_fifo_t *fw_evt_fifo;
    gdsl_fifo_t *fw_data_fifo;
    //tx fifo
    gdsl_fifo_t *tx_cmd_fifo;
    gdsl_tx_q_t tx_q[8]; //USB_TX_Q_NUM
    //15.4 fifo
    gdsl_fifo_t *_15p4_tx_fifo;
    gdsl_fifo_t *_15p4_rx_fifo;
    //rc manfdata
    unsigned char rc_manfdata[6*8];
    unsigned int manfdata_len;
    //mac addr
    unsigned char mac_addr[6];
    unsigned int sink_mode;
    unsigned int dr_state;
    unsigned char *usb_rx_buf;
    unsigned int usb_rx_len;
    struct urb *bt_urb;
    struct task_struct *usb_irq_task;
    struct semaphore usb_irq_sem;
    unsigned char usb_irq_task_quit;
    wait_queue_head_t rd_wait_queue;
    struct wakeup_source *amlbt_wakeup_source;
    struct mutex bt_debug_mutex;
    unsigned int wake_mux;
    //ioctl
    unsigned long recovery_value;
    unsigned long shutdown_value;
    unsigned int input_key;
    struct input_dev *amlbt_input_dev;
    struct device_link *link;
    //task
    struct hrtimer poll_timer;
    ktime_t ktime;
    struct workqueue_struct *check_fw_wq;
    struct work_struct check_fw;
    //ktimer
    u64 wait_start;
} w2l_usb_bt_new_t;

int amlbt_w2lu_new_init(void);
void amlbt_w2lu_new_exit(void);

#endif

