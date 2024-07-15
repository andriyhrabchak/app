/*
 * Copyright (c) 2022 Grant Ramsay <grant.ramsay@hotmail.com>
 * Copyright (c) 2024 HAV
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(power_phy, CONFIG_PHY_LOG_LEVEL);

#define LAN8720_POWER_PIN	0

static int board_esp32_mtsatlan_b_init(void)
{
	const struct device *gpio = DEVICE_DT_GET(DT_NODELABEL(gpio0));

	if (!device_is_ready(gpio)) {
		return -ENODEV;
	}

  LOG_INF("Power up LAN8720...");

	/* Enable the Ethernet phy */
	int res = gpio_pin_configure(
		gpio, LAN8720_POWER_PIN,
		GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);


	return res;
}

SYS_INIT(board_esp32_mtsatlan_b_init, PRE_KERNEL_2, CONFIG_GPIO_INIT_PRIORITY);
