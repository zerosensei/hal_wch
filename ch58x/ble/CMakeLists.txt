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

zephyr_library_sources_ifdef(CONFIG_BT_OBSERVER wch_observer.c)
zephyr_library_sources_ifdef(CONFIG_BT_BROADCASTER wch_broadcaster.c)
zephyr_library_sources_ifdef(CONFIG_BT_PERIPHERAL wch_peripheral.c)

zephyr_link_libraries(${CMAKE_CURRENT_SOURCE_DIR}/lib/LIBCH58xBLE.a)
