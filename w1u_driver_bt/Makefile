KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(PWD)
###############################################################################
# AMLBT
###############################################################################

AMLBT_MOD_NAME = w1u_bt

AMLBT_CFILES := w1u_sdio_bt.c \
				w1u_usb_bt_new.c \
				amlbt.c \
				common.c \
				rc_list.c \
				amlbt_utils.c
#AMLBT_CFILES := amlbt.c

$(AMLBT_MOD_NAME)-objs := $(AMLBT_CFILES:.c=.o)

###############################################################################
# Common
###############################################################################
obj-m += $(AMLBT_MOD_NAME).o

EXTRA_CFLAGS += \
	-Wno-pointer-to-int-cast -Wno-format \
	-Wno-implicit-fallthrough \
	-Wno-incompatible-function-pointer-types \
	-Wno-macro-redefined \
	-Wno-strict-prototypes \
	-Wno-unused-variable \
	-Wno-incompatible-pointer-types

all:
	make -C $(KERNEL_SRC) M=$(M) KBUILD_EXTRA_SYMBOLS=$(EXTRA_SYMBOLS_PATH_USB_BT) modules
modules_install:
	@$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install
	mkdir -p ${OUT_DIR}/../vendor_lib/modules
	cd ${OUT_DIR}/$(M)/; find -name "*.ko" -exec cp {} ${OUT_DIR}/../vendor_lib/modules/ \;

usb:
	make -C $(KERNEL_SRC) M=$(PWD) $(AMLBT_MOD_NAME).ko

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean



