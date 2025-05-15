# RELEASE

BOARD_HAVE_BLUETOOTH_AMLOGIC := true
AML_BLUETOOTH_LPM_ENABLE := true

PRODUCT_COPY_FILES += $(call find-copy-subdir-files,*,vendor/amlogic/common/wifi_bt/bluetooth/amlogic/config,$(TARGET_COPY_OUT_VENDOR)/firmware)
PRODUCT_COPY_FILES += $(call find-copy-subdir-files,*,vendor/amlogic/common/wifi_bt/bluetooth/amlogic/firmware,$(TARGET_COPY_OUT_VENDOR)/firmware)
PRODUCT_COPY_FILES += vendor/amlogic/common/wifi_bt/bluetooth/amlogic/config/aml_bt_new.conf:$(TARGET_COPY_OUT_VENDOR)/lib/firmware/aml_bt.conf

PRODUCT_PACKAGES += \
  w1_bt_fw_uart \
  w1u_bt_fw_uart \
  w1u_bt_fw_usb \
  w2_bt_fw_uart \
  w2_bt_fw_usb \
  w2l_bt_15p4_fw_uart \
  w2l_bt_15p4_fw_usb \
  btd \
  btiw \
  dut_test_zc \
  z_chip_init \
  z_ed_join \
  z_nwk_formation \
  z_route_join \
  z_set_role \
  z_stack_coor \
  z_stack_ed \
  z_stack_route \
  z_test_end \
  z_throughput_rx  \
  z_throughput_tx \
  z_get_rssi \
  z_get_channel \
  z_get_role \
  z_help \
  z_set_channel \
  z_get_devices \
  z_get_status \
  z_leave \
  z_active_scan \
  z_permit_open \
  z_permit_close \
  z_throughput_rx_end \
  ot-ctl-aml \
  otbr-agent-aml \
  libmnl_aml \
  libnetfilter_queue_aml \
  libnfnetlink_aml \
  libprotobuf-cpp-lite_aml \
  thread_ncp_aml \
  thread_otbr_aml \
  libmdnssd_aml

PRODUCT_COPY_FILES += frameworks/native/data/etc/android.hardware.bluetooth.xml:$(TARGET_COPY_OUT_VENDOR)/etc/permissions/android.hardware.bluetooth.xml \
       frameworks/native/data/etc/android.hardware.bluetooth_le.xml:$(TARGET_COPY_OUT_VENDOR)/etc/permissions/android.hardware.bluetooth_le.xml

