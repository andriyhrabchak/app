/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>

#include <app_version.h>
#ifdef CONFIG_GPIO_PCF8574T
#include <ioexp.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define SLEEP_TIME_MS   1000

#ifdef CONFIG_FAT_FILESYSTEM_ELM
#include <ff.h>
/* FatFs work area */
FATFS app_fat_fs;
/* mounting info */
static struct fs_mount_t app_fatfs_mnt = {
	.type = FS_FATFS,
	.fs_data = &app_fat_fs,
	.mnt_point = "/SD:" // Do not change!
};
#endif

#ifdef CONFIG_FILE_SYSTEM_LITTLEFS
#include <zephyr/fs/littlefs.h>
#define STORAGE_PARTITION_LABEL storage_partition
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(STORAGE_PARTITION_LABEL)
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(cstorage);
static struct fs_mount_t littlefs_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &cstorage,
	.storage_dev = (void *)STORAGE_PARTITION_ID,
	.mnt_point = "/LFS:"
};
#endif

int main(void) {
  int rc = 0;
  /* 
   * NET_CONFIG_SETTINGS will init DHCP
   * NET_SHELL is enabled to test ping, DNS etc
   */

  LOG_WRN("'Hello World!' v%s on %s", APP_VERSION_STRING, CONFIG_BOARD);

  #ifdef CONFIG_FILE_SYSTEM_LITTLEFS
	rc = fs_mount(&littlefs_mnt);
	if (rc < 0)	{
		LOG_ERR("Error mounting littlefs [%d]", rc);
	}
  #endif
  
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
