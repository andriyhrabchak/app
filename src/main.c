/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include <app_version.h>
#include <ioexp.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define SLEEP_TIME_MS   1000

int main(void)
{
  /* NET_CONFIG_SETTINGS will init DHCP
   * NET_SHELL is enabled to test ping, DNS etc
   */

  LOG_WRN("My 'Hello World!' v%s on %s", APP_VERSION_STRING, CONFIG_BOARD);
  
  if (ioexp_is_ready()) {

    ioexp_led_off(LED_STAT);
    ioexp_led_off(LED_LAN);

    while (1) {
      ioexp_led_tgl(LED_LAN);
      k_msleep(SLEEP_TIME_MS);
    }
  }
  
  return 0;
}
