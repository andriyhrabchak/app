#
# Copyright (c) 2024 HAV
#
# SPDX-License-Identifier: Apache-2.0
#

macro(zephyr_ext_library)
  zephyr_library_get_current_dir_lib_name(${CMAKE_SOURCE_DIR} lib_name)
  set(ZEPHYR_CURRENT_LIBRARY ${lib_name})
endmacro()

# Add hello_world_driver.h to the set of global include paths.
zephyr_include_directories(.)
# add a new out-of-tree driver for an existing subsystem
zephyr_ext_library()
zephyr_library_sources_ifdef(CONFIG_RTC_RV3032 rtc_rv3032.c)

