#ifndef __RC_LIST_H__
#define __RC_LIST_H__

#define AML_BT_CHAR_RCLIST_NAME "aml_rclist"

#define MAC_ADDR_LEN            6
#define MAX_MAC_LIST            8
#define MAX_USER_BUF_LEN        (MAX_MAC_LIST*17+1)

#define FIFO_FW_RC_LIST_ADDR    (0x514000 + 2364 + 4) //addr 0x514940 usb bulk 16 bit alignment

int amlbt_rc_list_init(struct device *dev, ws_inf p_ws_func, rs_inf p_rs_func);
void amlbt_rc_list_deinit(struct device *dev);
int amlbt_write_rclist_to_firmware(void);
void amlbt_clear_rclist_from_firmware(void);

typedef struct _rc_list_t
{
    char mac[MAC_ADDR_LEN];
    char used;
} rc_list_t;

#endif

