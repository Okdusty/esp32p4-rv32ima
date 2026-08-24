include $(TOPDIR)/rules.mk

PKG_NAME:=esp32p4-demo
PKG_RELEASE:=1
PKG_LICENSE:=MIT

include $(INCLUDE_DIR)/package.mk

define Package/esp32p4-demo
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=ESP32-P4 RV32 OpenWrt package-manager test
  ARCH:=all
endef

define Package/esp32p4-demo/description
 A tiny package used to verify signed APK installation on the RV32 guest.
endef

define Build/Compile
endef

define Package/esp32p4-demo/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) ./files/esp32p4-demo $(1)/usr/bin/esp32p4-demo
endef

$(eval $(call BuildPackage,esp32p4-demo))
