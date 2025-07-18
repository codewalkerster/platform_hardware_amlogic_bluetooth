# RELEASE

BOARD_HAVE_BLUETOOTH_AMLOGIC := true
AML_BLUETOOTH_LPM_ENABLE := true

PRODUCT_COPY_FILES += \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/dut_test_zc:$(TARGET_COPY_OUT_VENDOR)/xbin/dut_test_zc \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_chip_init:$(TARGET_COPY_OUT_VENDOR)/xbin/z_chip_init \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_ed_join:$(TARGET_COPY_OUT_VENDOR)/xbin/z_ed_join \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_nwk_formation:$(TARGET_COPY_OUT_VENDOR)/xbin/z_nwk_formation \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_route_join:$(TARGET_COPY_OUT_VENDOR)/xbin/z_route_join \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_set_role:$(TARGET_COPY_OUT_VENDOR)/xbin/z_set_role \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_test_end:$(TARGET_COPY_OUT_VENDOR)/xbin/z_test_end \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_throughput_rx:$(TARGET_COPY_OUT_VENDOR)/xbin/z_throughput_rx \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_throughput_tx:$(TARGET_COPY_OUT_VENDOR)/xbin/z_throughput_tx \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_stack_coor:$(TARGET_COPY_OUT_VENDOR)/xbin/z_stack_coor \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_stack_ed:$(TARGET_COPY_OUT_VENDOR)/xbin/z_stack_ed \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_stack_route:$(TARGET_COPY_OUT_VENDOR)/xbin/z_stack_route \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_get_rssi:$(TARGET_COPY_OUT_VENDOR)/xbin/z_get_rssi \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_get_channel:$(TARGET_COPY_OUT_VENDOR)/xbin/z_get_channel \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_get_role:$(TARGET_COPY_OUT_VENDOR)/xbin/z_get_role \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_help:$(TARGET_COPY_OUT_VENDOR)/xbin/z_help \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_set_channel:$(TARGET_COPY_OUT_VENDOR)/xbin/z_set_channel \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_get_devices:$(TARGET_COPY_OUT_VENDOR)/xbin/z_get_devices \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_get_status:$(TARGET_COPY_OUT_VENDOR)/xbin/z_get_status \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_leave:$(TARGET_COPY_OUT_VENDOR)/xbin/z_leave \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_active_scan:$(TARGET_COPY_OUT_VENDOR)/xbin/z_active_scan \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_permit_open:$(TARGET_COPY_OUT_VENDOR)/xbin/z_permit_open \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/zigbee/z_permit_close:$(TARGET_COPY_OUT_VENDOR)/xbin/z_permit_close \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/thread/threadbr_aml:$(TARGET_COPY_OUT_VENDOR)/xbin/threadbr_aml \
# vendor/amlogic/common/wifi_bt/bluetooth/amlogic/thread/udpServer:$(TARGET_COPY_OUT_VENDOR)/xbin/udpServer \

PRODUCT_COPY_FILES += $(call find-copy-subdir-files,*,vendor/amlogic/common/wifi_bt/bluetooth/amlogic/config,$(TARGET_COPY_OUT_VENDOR)/lib/firmware)

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

