/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/ext2.h>

#include <app_version.h>
#include <ioexp.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#define SLEEP_TIME_MS   1000

#define DISK_DRIVE_NAME "SDMMC"
#define DISK_MOUNT_PT "/ext"

static struct fs_mount_t mp = {
	.type = FS_EXT2,
	.flags = FS_MOUNT_FLAG_NO_FORMAT,
	.storage_dev = (void *)DISK_DRIVE_NAME,
	.mnt_point = "/ext",
};

#define FS_RET_OK 0
#define MAX_PATH 128
#define SOME_FILE_NAME "some.dat"
#define SOME_DIR_NAME "some"
#define SOME_REQUIRED_LEN MAX(sizeof(SOME_FILE_NAME), sizeof(SOME_DIR_NAME))

/* List dir entry by path
 *
 * @param path Absolute path to list
 *
 * @return Negative errno code on error, number of listed entries on
 *         success.
 */
static int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	int count = 0;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		printk("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	printk("\nListing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			printk("[DIR ] %s\n", entry.name);
		} else {
			printk("[FILE] %s (size = %zu)\n",
				entry.name, entry.size);
		}
		count++;
	}

	/* Verify fs_closedir() */
	fs_closedir(&dirp);
	if (res == 0) {
		res = count;
	}

	return res;
}

static bool create_some_entries(const char *base_path)
{
	char path[MAX_PATH];
	struct fs_file_t file;
	int base = strlen(base_path);

	fs_file_t_init(&file);

	if (base >= (sizeof(path) - SOME_REQUIRED_LEN)) {
		LOG_ERR("Not enough concatenation buffer to create file paths");
		return false;
	}

	LOG_INF("Creating some dir entries in %s", base_path);
	strncpy(path, base_path, sizeof(path));

	path[base++] = '/';
	path[base] = 0;
	strcat(&path[base], SOME_FILE_NAME);

	if (fs_open(&file, path, FS_O_CREATE) != 0) {
		LOG_ERR("Failed to create file %s", path);
		return false;
	}
	fs_close(&file);

	path[base] = 0;
	strcat(&path[base], SOME_DIR_NAME);

	if (fs_mkdir(path) != 0) {
		LOG_ERR("Failed to create dir %s", path);
		/* If code gets here, it has at least successes to create the
		 * file so allow function to return true.
		 */
	}
	return true;
}

static const char *disk_mount_pt = DISK_MOUNT_PT;

int main(void)
{
  /* 
   * NET_CONFIG_SETTINGS will init DHCP
   * NET_SHELL is enabled to test ping, DNS etc
   */

  LOG_WRN("'Hello World!' v%s on %s", APP_VERSION_STRING, CONFIG_BOARD);

	/* raw disk i/o */
	do {
		static const char *disk_pdrv = DISK_DRIVE_NAME;
		uint64_t memory_size_mb;
		uint32_t block_count;
		uint32_t block_size;

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_CTRL_INIT, NULL) != 0) {
			LOG_ERR("Storage init ERROR!");
			break;
		}

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
			LOG_ERR("Unable to get sector count");
			break;
		}
		LOG_INF("Block count %u", block_count);

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
			LOG_ERR("Unable to get sector size");
			break;
		}
		printk("Sector size %u\n", block_size);

		memory_size_mb = (uint64_t)block_count * block_size;
		printk("Memory Size(MB) %u\n", (uint32_t)(memory_size_mb >> 20));

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_CTRL_DEINIT, NULL) != 0) {
			LOG_ERR("Storage deinit ERROR!");
			break;
		}
	} while (0);

	mp.mnt_point = disk_mount_pt;

	int res = fs_mount(&mp);

	if (res == FS_RET_OK) {
		printk("Disk mounted.\n");
		/* Try to unmount and remount the disk */
		res = fs_unmount(&mp);
		if (res != FS_RET_OK) {
			printk("Error unmounting disk\n");
			return res;
		}
		res = fs_mount(&mp);
		if (res != FS_RET_OK) {
			printk("Error remounting disk\n");
			return res;
		}

		if (lsdir(disk_mount_pt) == 0) {
			if (create_some_entries(disk_mount_pt)) {
				lsdir(disk_mount_pt);
			}
		}
	} else {
		printk("Error mounting disk.\n");
	}

	fs_unmount(&mp);

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
