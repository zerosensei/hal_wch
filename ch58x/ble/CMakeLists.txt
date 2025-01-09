# Copyright (c) 2024 zerosensei
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(ch58x_ble_lib)

zephyr_include_directories(
    .
    lib
)

zephyr_library_sources(
    wch_bt_init.c
    wch_bt_hci.c
    wch_hci_common.c
)

if(CONFIG_BT_OBSERVER OR CONFIG_BT_PERIPHERAL)
zephyr_library_sources(wch_bt_central.c)
endif()

if(CONFIG_BT_BROADCASTER OR CONFIG_BT_CENTRAL)
zephyr_library_sources(wch_bt_peripheral.c)
endif()

zephyr_link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/lib/LIBCH58xBLE.a)
