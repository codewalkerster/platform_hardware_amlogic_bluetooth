#ifndef __DEBUG_DEV_H__
#define __DEBUG_DEV_H__

#define AML_BT_CHAR_DEBUG_DEVICE "aml_debug"
#define AML_BT_CHAR_RECYDBG_NAME "aml_recy_dbg"

#define cmd_len 12
#define evt_len 16

int amlbt_debug_dev_init(ws_inf p_ws_func, rs_inf p_rs_func, ww_inf p_ww_func, rw_inf p_rw_func, void *bt_dev);
void amlbt_debug_dev_deinit(void);
void amlbt_debug_get_event(unsigned int w, unsigned int r, unsigned char *data);
void amlbt_debug_get_cmd(unsigned int w, unsigned int r, unsigned char *data);
void amlbt_debug_get_manfdata(unsigned char *data, unsigned int len);
int amlbt_debug_filter_event(unsigned char *evt_buf);
void amlbt_show_debug(void);

#endif

