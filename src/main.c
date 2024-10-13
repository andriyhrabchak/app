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
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <zephyr/net/hostname.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/sntp.h>
#include <zephyr/posix/time.h>

#include <app_version.h>

#ifdef CONFIG_GPIO_PCF8574T
#include <ioexp.h>
#endif

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

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static struct net_mgmt_event_callback mgmt_cb;

/* Semaphore to indicate a lease has been acquired. */
static K_SEM_DEFINE(got_address, 0, 1);

static void handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface) {
	int i;
	bool notified = false;

	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
		return;
	}

	for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		if (iface->config.ip.ipv4->unicast[i].ipv4.addr_type != NET_ADDR_DHCP) {
			continue;
		}

		if (!notified) {
			k_sem_give(&got_address);
			notified = true;
		}
		break;
	}
}

/**
 * Start a DHCP client, and wait for a lease to be acquired.
 */
void app_dhcpv4_startup(void) {
	LOG_INF("Starting DHCPv4...");

	net_mgmt_init_event_callback(&mgmt_cb, handler, NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&mgmt_cb);

	net_dhcpv4_start(net_if_get_default());

	/* Wait for a lease. */
	k_sem_take(&got_address, K_FOREVER);
}

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

#define SNTP_SERVER "pool.ntp.org"

int sntp_sync_time(void) {
	int rc;
	struct sntp_time now;
	struct timespec tspec;

	rc = sntp_simple(SNTP_SERVER, 10000, &now);
	if (rc == 0) {
		tspec.tv_sec = now.seconds;
		tspec.tv_nsec = ((uint64_t)now.fraction * (1000lu * 1000lu * 1000lu)) >> 32;

		clock_settime(CLOCK_REALTIME, &tspec);

		LOG_DBG("Acquired time from NTP server: %u", (uint32_t)tspec.tv_sec);
	} else {
		LOG_ERR("Failed to acquire SNTP, code %d\n", rc);
	}
	return rc;
}

#define SLEEP_TIME_MS   1000

extern const struct log_backend *log_backend_net_get(void);

int main(void) {
  int rc = 0;

  printk("### Run 'app' v%s on %s ###\n", APP_VERSION_STRING, CONFIG_BOARD);

	app_dhcpv4_startup();

  sntp_sync_time();

  /* Example how to start the backend if autostart is disabled.
    * This is useful if the application needs to wait the network
    * to be fully up before the syslog-net is able to work.
    */
  LOG_WRN("Begin logging to syslog");
  k_msleep(SLEEP_TIME_MS);
  // set filters for all domains
  const struct shell *shell = shell_backend_uart_get_ptr();
  log_filter_set(shell->log_backend->backend, Z_LOG_LOCAL_DOMAIN_ID, module_logid_get("main"), LOG_LEVEL_DBG);
  // disable shell domain logs
  log_backend_deactivate(shell->log_backend->backend);
  // enable net domain logs (syslogs)
  net_hostname_set("myhost12345", 11); 
  const struct log_backend *log_backend_net = log_backend_net_get();
  if (log_backend_net->api->init != NULL) {
    log_backend_net->api->init(log_backend_net);
  }
  log_backend_activate(log_backend_net, NULL);

  k_msleep(SLEEP_TIME_MS);

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
