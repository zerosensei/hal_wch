# Copyright (c) 2023 zerosensei
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(hal_ch58x)

# TODO: ifdef?
zephyr_library_sources(RVMSIS/core_riscv.c)
zephyr_library_sources(StdPeriphDriver/CH58x_adc.c)
zephyr_library_sources(StdPeriphDriver/CH58x_clk.c)
zephyr_library_sources(StdPeriphDriver/CH58x_flash.c)
zephyr_library_sources(StdPeriphDriver/CH58x_gpio.c)
zephyr_library_sources(StdPeriphDriver/CH58x_pwr.c)
zephyr_library_sources(StdPeriphDriver/CH58x_pwm.c)
zephyr_library_sources(StdPeriphDriver/CH58x_spi0.c)
zephyr_library_sources(StdPeriphDriver/CH58x_spi1.c)
zephyr_library_sources(StdPeriphDriver/CH58x_sys.c)
zephyr_library_sources(StdPeriphDriver/CH58x_timer0.c)
zephyr_library_sources(StdPeriphDriver/CH58x_timer1.c)
zephyr_library_sources(StdPeriphDriver/CH58x_timer2.c)
zephyr_library_sources(StdPeriphDriver/CH58x_timer3.c)
zephyr_library_sources(StdPeriphDriver/CH58x_uart0.c)
zephyr_library_sources(StdPeriphDriver/CH58x_uart1.c)
zephyr_library_sources(StdPeriphDriver/CH58x_uart2.c)
zephyr_library_sources(StdPeriphDriver/CH58x_uart3.c)
zephyr_library_sources(StdPeriphDriver/CH58x_usb2dev.c)
zephyr_library_sources(StdPeriphDriver/CH58x_usb2hostBase.c)
zephyr_library_sources(StdPeriphDriver/CH58x_usb2hostClass.c)
zephyr_library_sources(StdPeriphDriver/CH58x_usbdev.c)
zephyr_library_sources(StdPeriphDriver/CH58x_usbhostBase.c)
zephyr_library_sources(StdPeriphDriver/CH58x_usbhostClass.c)

zephyr_include_directories(RVMSIS)
zephyr_include_directories(StdPeriphDriver/inc)

# TODO: kconfig
# zephyr_link_libraries_ifdef(CONFIG_SOC_FLASH_CH5XX ${CMAKE_CURRENT_SOURCE_DIR}/StdPeriphDriver/libISP583.a)
zephyr_link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/StdPeriphDriver/libISP583.a)


add_subdirectory_ifdef(CONFIG_BT ble)

