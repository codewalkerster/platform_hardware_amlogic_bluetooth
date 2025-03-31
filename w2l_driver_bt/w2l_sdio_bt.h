#ifndef __W2L_SDIO_BT_H__
#define __W2L_SDIO_BT_H__

typedef struct
{
    struct cdev dev_cdev;
    int         dev_major;
    struct class *dev_class;
    struct device *dev_device;
    struct early_suspend early_suspend;
    const unsigned char *iccm_buf;
    const unsigned char *dccm_buf;
    int irq;
    unsigned int irq_handle;
    struct work_struct wake_work;
    struct input_dev *input_dev;
    unsigned int antenna;
    unsigned int fw_mode;
    unsigned int bt_sink;
    unsigned int pin_mux;
    unsigned int br_digit_gain;
    unsigned int edr_digit_gain;
    unsigned int fw_log;
    unsigned int driver_log;
    unsigned int factory;
    unsigned char firmware_start;
    struct completion notify_comp;
    unsigned int notify_trig;
} w2l_sdio_bt_t;

int amlbt_w2ls_init(void);
void amlbt_w2ls_exit(void);


#endif

