# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk

BOARD:=esp32p4rv32
BOARDNAME:=ESP32-P4 RV32IMA emulator
DEVICE_TYPE:=basic
FEATURES:=ramdisk targz
SUBTARGETS:=generic
KERNELNAME:=Image
KERNEL_PATCHVER:=6.12

include $(INCLUDE_DIR)/target.mk

# The ESP-IDF application supplies the kernel, device tree, display and
# virtio-net transport. OpenWrt provides the compact userspace and APK feed.
DEFAULT_PACKAGES := $(filter-out fstools,$(DEFAULT_PACKAGES))
DEFAULT_PACKAGES += apk-mbedtls

define Target/Description
	Build an RV32IMA OpenWrt userspace for the ESP32-P4 emulator.
endef

$(eval $(call BuildTarget))
