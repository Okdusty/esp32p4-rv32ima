set(KERNEL_IMAGE "${CMAKE_CURRENT_LIST_DIR}/Image")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${KERNEL_IMAGE}")
file(SIZE "${KERNEL_IMAGE}" KERNEL_FLASH_SIZE)

set(RV32_SOURCES
	"uc-rv32ima.c"
	"cache.c"
	"dwc2_passthrough.c"
	"port-esp.c"
	"virtio_console_bridge.c")

set(RV32_LINKER_FRAGMENTS "link.lf")

if(CONFIG_RV32_DISPLAY_ACCEL)
	list(APPEND RV32_SOURCES "display_bridge.c")
	list(APPEND RV32_LINKER_FRAGMENTS "link-display.lf")
endif()

if(CONFIG_RV32_ST7703_DISPLAY)
	list(APPEND RV32_SOURCES "display_backend_st7703.c")
elseif(CONFIG_RV32_ST7789_DISPLAY)
	list(APPEND RV32_SOURCES "display_backend_st7789.c")
elseif(CONFIG_RV32_SSD1306_DISPLAY)
	list(APPEND RV32_SOURCES "display_backend_ssd1306.c")
endif()

if(CONFIG_RV32_WIFI_BRIDGE)
	list(APPEND RV32_SOURCES "virtio_net_bridge.c")
endif()

idf_component_register(SRCS ${RV32_SOURCES}
		       LDFRAGMENTS ${RV32_LINKER_FRAGMENTS}
		       INCLUDE_DIRS "."
		       PRIV_REQUIRES esp_lcd esp_driver_gpio esp_driver_ledc
		                     esp_driver_i2c esp_driver_spi
		                     esp_driver_ppa
		                     esp_hw_support esp_mm esp_timer
		                     bootloader_support
		                     esp_partition spi_flash heap esp_hal_usb
		                     esp_event esp_wifi espressif__esp_hosted
		                     waveshare__esp_lcd_st7703)

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
