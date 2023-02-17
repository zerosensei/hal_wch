# Copyright (c) 2023 zerosensei
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_HAS_CH32V30X_HAL ch32v30x)
add_subdirectory_ifdef(CONFIG_HAS_CH58X_HAL ch58x)