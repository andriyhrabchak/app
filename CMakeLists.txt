# SPDX-License-Identifier: Apache-2.0

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers/gpio)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers/rtc)

cmake_minimum_required(VERSION 3.20.0)

# list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mtsatlam_b_ex)

test_sysbuild()

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
FILE(GLOB app_csources src/*.c)
target_sources(app PRIVATE ${app_csources})

if(CONFIG_MODBUS)
  add_subdirectory(src/mbm)
endif()
if(CONFIG_GPIO_PCF8574T)
  add_subdirectory(src/ioexp)
endif()
