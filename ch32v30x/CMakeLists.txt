# Copyright (c) 2023 zerosensei
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# TODO: ifdef?
zephyr_library_sources(Core/core_riscv.c)
zephyr_library_sources(Debug/debug.c)
zephyr_library_sources(Peripheral/src/ch32v30x_adc.c)
zephyr_library_sources(Peripheral/src/ch32v30x_bkp.c)
zephyr_library_sources(Peripheral/src/ch32v30x_can.c)
zephyr_library_sources(Peripheral/src/ch32v30x_crc.c)
zephyr_library_sources(Peripheral/src/ch32v30x_dac.c)
zephyr_library_sources(Peripheral/src/ch32v30x_dbgmcu.c)
zephyr_library_sources(Peripheral/src/ch32v30x_dma.c)
zephyr_library_sources(Peripheral/src/ch32v30x_dvp.c)
zephyr_library_sources(Peripheral/src/ch32v30x_eth.c)
zephyr_library_sources(Peripheral/src/ch32v30x_exti.c)
zephyr_library_sources(Peripheral/src/ch32v30x_flash.c)
zephyr_library_sources(Peripheral/src/ch32v30x_fsmc.c)
zephyr_library_sources(Peripheral/src/ch32v30x_gpio.c)
zephyr_library_sources(Peripheral/src/ch32v30x_i2c.c)
zephyr_library_sources(Peripheral/src/ch32v30x_iwdg.c)
zephyr_library_sources(Peripheral/src/ch32v30x_misc.c)
zephyr_library_sources(Peripheral/src/ch32v30x_opa.c)
zephyr_library_sources(Peripheral/src/ch32v30x_pwr.c)
zephyr_library_sources(Peripheral/src/ch32v30x_rcc.c)
zephyr_library_sources(Peripheral/src/ch32v30x_rng.c)
zephyr_library_sources(Peripheral/src/ch32v30x_rtc.c)
zephyr_library_sources(Peripheral/src/ch32v30x_sdio.c)
zephyr_library_sources(Peripheral/src/ch32v30x_spi.c)
zephyr_library_sources(Peripheral/src/ch32v30x_tim.c)
zephyr_library_sources(Peripheral/src/ch32v30x_usart.c)
zephyr_library_sources(Peripheral/src/ch32v30x_wwdg.c)


zephyr_library_sources(User/ch32v30x_it.c)
zephyr_library_sources(User/system_ch32v30x.c)

zephyr_include_directories(User)
zephyr_include_directories(Core)
zephyr_include_directories(Debug)
zephyr_include_directories(Peripheral/inc)
