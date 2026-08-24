cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(uc-rv32ima)

set(LINUX_IMAGE_SOURCE "${CMAKE_CURRENT_LIST_DIR}/main/Image")
if(CONFIG_RV32_ST7703_DISPLAY)
  if(CONFIG_RV32_WIFI_BRIDGE)
    set(LINUX_DTB_SOURCE "${CMAKE_CURRENT_LIST_DIR}/main/uc.dtb")
  else()
    set(LINUX_DTB_SOURCE "${CMAKE_CURRENT_LIST_DIR}/main/uc-nowifi.dtb")
  endif()
else()
  if(CONFIG_RV32_WIFI_BRIDGE)
    set(LINUX_DTB_SOURCE "${CMAKE_CURRENT_LIST_DIR}/main/uc-headless.dtb")
  else()
    set(LINUX_DTB_SOURCE "${CMAKE_CURRENT_LIST_DIR}/main/uc-minimal.dtb")
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
