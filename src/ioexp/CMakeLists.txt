#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
# Copyright (c) 2023 HAV

target_include_directories(app PRIVATE .)
FILE(GLOB app_csources *.c)
target_sources(app PRIVATE ${app_csources})

