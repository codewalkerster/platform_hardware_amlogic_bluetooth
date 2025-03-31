KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(PWD)
###############################################################################
# AMLBT
###############################################################################

W2L_BT_MOD_NAME = w2l_bt

W2L_BT_CFILES := w2l_sdio_bt.c \
				w2l_usb_bt_new.c \
				w2l_bt_entry.c \
				common.c \
				rc_list.c \
				debug_dev.c \
				amlbt_utils.c
#W2L_BT_CFILES := amlbt.c

$(W2L_BT_MOD_NAME)-objs := $(W2L_BT_CFILES:.c=.o)
###############################################################################
# Common
###############################################################################
obj-m += $(W2L_BT_MOD_NAME).o

EXTRA_CFLAGS += -Wno-pointer-to-int-cast -Wno-format


all:
	make -C $(KERNEL_SRC) M=$(M) KBUILD_EXTRA_SYMBOLS=$(EXTRA_SYMBOLS_PATH_USB_BT) modules
modules_install:
	@$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install
	mkdir -p ${OUT_DIR}/../vendor_lib/modules
	cd ${OUT_DIR}/$(M)/; find -name "*.ko" -exec cp {} ${OUT_DIR}/../vendor_lib/modules/ \;

usb:
	make -C $(KERNEL_SRC) M=$(PWD) $(W2L_BT_MOD_NAME).ko

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean


