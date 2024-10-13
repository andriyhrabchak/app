/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/net/hostname.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>

#include <app_version.h>

#ifdef CONFIG_GPIO_PCF8574T
#include <ioexp.h>
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

extern const struct log_backend *log_backend_net_get(void);

#ifdef CONFIG_FAT_FILESYSTEM_ELM
#include <zephyr/fs/fs.h>
#include <ff.h>
/* FatFs work area */
FATFS app_fat_fs;
/* mounting info */
static struct fs_mount_t app_fatfs_mnt = {
	.type = FS_FATFS,
	.fs_data = &app_fat_fs,
	.mnt_point = "/SD:" // Do not change! Allowed only strings from FF_VOLUME_STRS
};
#endif

#define SLEEP_TIME_MS   1000

int module_logid_get(const char *name) {
	uint32_t modules_cnt = log_src_cnt_get(Z_LOG_LOCAL_DOMAIN_ID);
	const char *tmp_name;
	uint32_t i;

	for (i = 0U; i < modules_cnt; i++) {
		tmp_name = log_source_name_get(Z_LOG_LOCAL_DOMAIN_ID, i);

		if (strncmp(tmp_name, name, 64) == 0) {
			return i;
		}
	}
	return -1;
}

int main(void) {
  int rc = 0;
  /* 
   * NET_CONFIG_SETTINGS will init DHCP
   * NET_SHELL is enabled to test ping, DNS etc
   */

  LOG_WRN("'Hello World!' v%s on %s", APP_VERSION_STRING, CONFIG_BOARD);

  k_msleep((SLEEP_TIME_MS * 15));

  /* Example how to start the backend if autostart is disabled.
    * This is useful if the application needs to wait the network
    * to be fully up before the syslog-net is able to work.
    */
  LOG_WRN("Begin logging to syslog");
  k_msleep(SLEEP_TIME_MS);
  const struct log_backend *log_backend_net = log_backend_net_get();
  // set filters for all domains
  const struct shell *shell = shell_backend_uart_get_ptr();
  log_filter_set(shell->log_backend->backend, Z_LOG_LOCAL_DOMAIN_ID, module_logid_get("main"), LOG_LEVEL_DBG);
  // disable shell domain logs
  log_backend_deactivate(shell->log_backend->backend);
  // enable net domain logs (syslogs)
  if (log_backend_net->api->init != NULL) {
    log_backend_net->api->init(log_backend_net);
  }
  log_backend_activate(log_backend_net, NULL);
  net_hostname_set("myhost12345", 11); 

  int count = 5;
	int i = count;
	do {
		LOG_ERR("Error message (%d)", i);
		LOG_WRN("Warning message (%d)", i);
		LOG_INF("Info message (%d)", i);
		LOG_DBG("Debug message (%d)", i);
		k_msleep(SLEEP_TIME_MS);
	} while (--i);
	LOG_DBG("Stopped after %d msg", count);

  #ifdef CONFIG_FAT_FILESYSTEM_ELM
	rc = fs_mount(&app_fatfs_mnt);
  if (rc == -EIO) {
    LOG_WRN("No SD card detected");
  } else if (rc < 0)	{
		LOG_ERR("Error mounting FAT [%d]", rc);
	}
  #endif

  #ifdef CONFIG_GPIO_PCF8574T
  if (ioexp_is_ready()) {

    ioexp_led_off(LED_STAT);
    ioexp_led_off(LED_LAN);

    while (1) {
      ioexp_led_tgl(LED_LAN);
      k_msleep(SLEEP_TIME_MS);
    }
  }
  #endif
  
  return rc;
}
