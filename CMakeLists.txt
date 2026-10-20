cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32p4")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(uc-rv32ima)

set(LINUX_IMAGE_SOURCE "${CMAKE_CURRENT_LIST_DIR}/main/Image")
# U8g2 panels use the 128x64 description. The firmware writes the configured
# framebuffer geometry into it at boot (prepare_linux_dtb in port-esp.c).
if(CONFIG_RV32_SSD1306_DISPLAY OR CONFIG_RV32_U8G2_DISPLAY)
  if(CONFIG_RV32_SSD1306_128X32)
    set(LINUX_DTB_DISPLAY "ssd1306-32")
  else()
    set(LINUX_DTB_DISPLAY "ssd1306-64")
  endif()
  if(CONFIG_RV32_WIFI_BRIDGE)
    set(LINUX_DTB_SOURCE
        "${CMAKE_CURRENT_LIST_DIR}/main/dtb/uc-${LINUX_DTB_DISPLAY}.dtb")
  else()
    set(LINUX_DTB_SOURCE
        "${CMAKE_CURRENT_LIST_DIR}/main/dtb/uc-${LINUX_DTB_DISPLAY}-nowifi.dtb")
  endif()
elseif(CONFIG_RV32_ST7789_DISPLAY)
  if(CONFIG_RV32_WIFI_BRIDGE)
    set(LINUX_DTB_SOURCE "${CMAKE_CURRENT_LIST_DIR}/main/dtb/uc-st7789.dtb")
  else()
    set(LINUX_DTB_SOURCE
        "${CMAKE_CURRENT_LIST_DIR}/main/dtb/uc-st7789-nowifi.dtb")
  endif()
elseif(CONFIG_RV32_ST7703_DISPLAY)
  if(CONFIG_RV32_WIFI_BRIDGE)
    set(LINUX_DTB_SOURCE "${CMAKE_CURRENT_LIST_DIR}/main/dtb/uc.dtb")
  else()
    set(LINUX_DTB_SOURCE "${CMAKE_CURRENT_LIST_DIR}/main/dtb/uc-nowifi.dtb")
  endif()
else()
  if(CONFIG_RV32_WIFI_BRIDGE)
    set(LINUX_DTB_SOURCE "${CMAKE_CURRENT_LIST_DIR}/main/dtb/uc-headless.dtb")
  else()
    set(LINUX_DTB_SOURCE "${CMAKE_CURRENT_LIST_DIR}/main/dtb/uc-minimal.dtb")
  endif()
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             "${LINUX_IMAGE_SOURCE}" "${LINUX_DTB_SOURCE}")

set(LINUX_IMAGE "${CMAKE_BINARY_DIR}/linux_image.bin")
set(LINUX_DTB "${CMAKE_BINARY_DIR}/linux_dtb.bin")
configure_file("${LINUX_IMAGE_SOURCE}" "${LINUX_IMAGE}" COPYONLY)
configure_file("${LINUX_DTB_SOURCE}" "${LINUX_DTB}" COPYONLY)

esp_partition_register_target(kernel "${LINUX_IMAGE}"
                              ALWAYS_PLAINTEXT FLASH_IN_PROJECT)
esp_partition_register_target(dtb "${LINUX_DTB}"
                              ALWAYS_PLAINTEXT FLASH_IN_PROJECT)

if(CONFIG_RV32_FLASH_ROFS)
  set(ROFS_IMAGE "${CONFIG_RV32_FLASH_ROFS_IMAGE}")
  if(NOT IS_ABSOLUTE "${ROFS_IMAGE}")
    set(ROFS_IMAGE "${CMAKE_CURRENT_LIST_DIR}/${ROFS_IMAGE}")
  endif()
  if(EXISTS "${ROFS_IMAGE}")
    # A .bin copy, like the kernel's, lets flashing skip an unchanged image.
    set(ROFS_BIN "${CMAKE_BINARY_DIR}/rofs.bin")
    configure_file("${ROFS_IMAGE}" "${ROFS_BIN}" COPYONLY)
    esp_partition_register_target(rofs "${ROFS_BIN}"
                                  ALWAYS_PLAINTEXT FLASH_IN_PROJECT)
  else()
    message(WARNING "${ROFS_IMAGE} not found; run make guest-desktop. "
                    "The rofs partition is left as previously flashed.")
  endif()
endif()
