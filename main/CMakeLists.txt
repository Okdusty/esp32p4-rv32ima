set(KERNEL_IMAGE "${CMAKE_CURRENT_LIST_DIR}/Image")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${KERNEL_IMAGE}")
file(SIZE "${KERNEL_IMAGE}" KERNEL_FLASH_SIZE)

set(RV32_SOURCES
	"uc-rv32ima.c"
	"cache.c"
	"dwc2_passthrough.c"
	"port-esp.c"
	"virtio/virtio_blk_bridge.c"
	"virtio/virtio_console_bridge.c"
	"virtio/virtio_flash_bridge.c"
	"virtio/virtio_input_bridge.c")

if(CONFIG_RV32_USB_PORT_HID)
	list(APPEND RV32_SOURCES "usb_hid_input.c")
endif()

if(CONFIG_RV32_NATIVE_JIT)
	list(APPEND RV32_SOURCES
		"rv32_native_jit.c"
		"rv32_jit_trace.c"
		"rv32_native_jit_enter.S")
endif()

if(CONFIG_RV32_CACHE_PERF_STATS)
	list(APPEND RV32_SOURCES "host_cache_profile.c")
endif()

if(CONFIG_RV32_HOST_LANE_REPORT)
  list(APPEND RV32_SOURCES "host_lane_report.c")
endif()
if(CONFIG_RV32_HOST_MICROBENCH)
  list(APPEND RV32_SOURCES "host_microbench.c")
endif()
if(CONFIG_RV32_HOST_PC_SAMPLER)
	list(APPEND RV32_SOURCES "host_pc_sampler.c")
endif()

if(CONFIG_RV32_EXPERIMENT_PANIC_DOWNLOAD)
	list(APPEND RV32_SOURCES "experiment_recovery.c")
endif()

if(CONFIG_RV32_GUEST_PROFILE)
	list(APPEND RV32_SOURCES "guest_profile.c")
endif()

if(CONFIG_RV32_GUEST_HLE)
	list(APPEND RV32_SOURCES "guest_hle.c" "guest_lz4.c")
endif()

set(RV32_LINKER_FRAGMENTS "link.lf")

if(CONFIG_RV32_DISPLAY_ACCEL)
	list(APPEND RV32_SOURCES "display/display_bridge.c")
	list(APPEND RV32_LINKER_FRAGMENTS "link-display.lf")
endif()

if(CONFIG_RV32_VIRTIO_GPU)
	list(APPEND RV32_SOURCES "virtio/virtio_gpu_bridge.c")
endif()

if(CONFIG_RV32_ST7703_DISPLAY)
	list(APPEND RV32_SOURCES "display/display_backend_st7703.c")
elseif(CONFIG_RV32_ST7789_DISPLAY)
	list(APPEND RV32_SOURCES "display/display_backend_st7789.c")
elseif(CONFIG_RV32_SSD1306_DISPLAY)
	list(APPEND RV32_SOURCES "display/display_backend_ssd1306.c")
elseif(CONFIG_RV32_U8G2_DISPLAY)
	list(APPEND RV32_SOURCES "display/display_backend_u8g2.c")
	# The generated U8g2 display menu derives this from the chosen panel.
	if(NOT CONFIG_RV32_U8G2_SETUP MATCHES "^u8g2_Setup_[a-z0-9_]+_f$")
		message(FATAL_ERROR "No U8g2 panel is selected "
			"(CONFIG_RV32_U8G2_SETUP=\"${CONFIG_RV32_U8G2_SETUP}\"); "
			"choose one in the U8g2 display menu")
	endif()
endif()

if(CONFIG_RV32_WIFI_BRIDGE)
	list(APPEND RV32_SOURCES "virtio/virtio_net_bridge.c")
endif()

idf_component_register(SRCS ${RV32_SOURCES}
		       LDFRAGMENTS ${RV32_LINKER_FRAGMENTS}
		       INCLUDE_DIRS "." "display" "virtio"
		       PRIV_REQUIRES esp_lcd esp_driver_gpio esp_driver_ledc
		                     esp_driver_i2c esp_driver_spi
		                     esp_driver_sdmmc esp_driver_dma esp_hal_dma sdmmc
		                     esp_driver_ppa esp_driver_gptimer
		                     esp_hw_support esp_mm esp_timer soc
		                     bootloader_support
		                     esp_partition spi_flash heap esp_hal_usb esp_hal_pmu
		                     esp_event esp_wifi espressif__esp_hosted
		                     espressif__usb espressif__usb_host_hid
		                     waveshare__esp_lcd_st7703 u8g2)

if(CONFIG_RV32_U8G2_DISPLAY)
	# Kconfig strings reach C as string literals; the backend calls the
	# setup function by name.
	target_compile_definitions(${COMPONENT_LIB} PRIVATE
		"RV32_U8G2_SETUP_FUNCTION=${CONFIG_RV32_U8G2_SETUP}")
endif()

if(CONFIG_RV32_EXPERIMENT_PANIC_DOWNLOAD)
	target_link_options(${COMPONENT_LIB} INTERFACE
		"-Wl,--wrap=esp_panic_handler"
		"-Wl,--wrap=esp_psram_extram_reserve_dma_pool")
endif()

# mini-rv32ima is an opcode interpreter: jump-table dispatch and the extra
# inlining/vector-free scalar passes at -O3 materially reduce work per guest
# instruction.  Its code and read-only tables are pinned by link.lf, so this
# does not introduce flash accesses into the hot loop.
target_compile_options(${COMPONENT_LIB} PRIVATE
	-O3 -fjump-tables -ftree-switch-conversion)

# The RISC-V header reports the effective in-memory size, which can include
# zero-filled BSS.  Keep the raw flash payload length separate so the loader
# does not copy erased/stale flash bytes over that BSS.
target_compile_definitions(${COMPONENT_LIB} PRIVATE
                           KERNEL_FLASH_SIZE=${KERNEL_FLASH_SIZE}u)
