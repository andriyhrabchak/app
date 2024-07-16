/*
 * Copyright (c) 2024 ANITRA system s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2024 HAV
 * 
 */

#define DT_DRV_COMPAT microcrystal_rv3032

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(rv3032, CONFIG_RTC_LOG_LEVEL);

/* RV3032 RAM register addresses */
#define RV3032_REG_SECONDS              0x01
#define RV3032_REG_MINUTES              0x02
#define RV3032_REG_HOURS                0x03
#define RV3032_REG_WEEKDAY              0x04
#define RV3032_REG_DATE                 0x05
#define RV3032_REG_MONTH                0x06
#define RV3032_REG_YEAR                 0x07
#define RV3032_REG_STATUS               0x0D
#define RV3032_REG_STATUS_TEMP          0x0E
#define RV3032_REG_CONTROL1             0x10
#define RV3032_REG_CONTROL2             0x11
#define RV3032_REG_USER_RAM_BEGIN       0x40
#define RV3032_REG_USER_RAM_END         0x4F
#define RV3032_REG_EEPROM_ADDRESS       0x3D
#define RV3032_REG_EEPROM_DATA          0x3E
#define RV3032_REG_EEPROM_COMMAND       0x3F
#define RV3032_REG_PMU                  0xC0
#define RV3032_REG_OFFSET               0xC1

#define RV3032_CONTROL1_TD              GENMASK(1, 0)
#define RV3032_CONTROL1_EERD            BIT(2)
#define RV3032_CONTROL1_TE              BIT(3)
#define RV3032_CONTROL1_USEL            BIT(4)

#define RV3032_CONTROL2_STOP            BIT(0)
#define RV3032_CONTROL2_EIE             BIT(2)
#define RV3032_CONTROL2_AIE             BIT(3)
#define RV3032_CONTROL2_TIE             BIT(4)
#define RV3032_CONTROL2_UIE             BIT(5)
#define RV3032_CONTROL2_CLKIE           BIT(6)

#define RV3032_STATUS_PORF              BIT(1)
#define RV3032_STATUS_EVF               BIT(2)
#define RV3032_STATUS_AF                BIT(3)
#define RV3032_STATUS_TF                BIT(4)
#define RV3032_STATUS_UF                BIT(5)

#define RV3032_STATUS_TEMP_BSF          BIT(0)
#define RV3032_STATUS_TEMP_CLKF         BIT(1)
#define RV3032_STATUS_TEMP_EEBUSY       BIT(2)

#define RV3032_PMU_TCM                  GENMASK(1, 0)
#define RV3032_PMU_TCR                  GENMASK(3, 2)
#define RV3032_PMU_BSM                  GENMASK(5, 4)

#define RV3032_BSM_LEVEL                0x2
#define RV3032_BSM_DIRECT               0x1
#define RV3032_BSM_DISABLED             0x0

#define RV3032_TCM_4400MV               0x3
#define RV3032_TCM_3000MV               0x2
#define RV3032_TCM_1750MV               0x1
#define RV3032_TCM_DISABLED             0x0

/* RV3032 EE command register values */
#define RV3032_EEPROM_CMD_UPDATE        0x11
#define RV3032_EEPROM_CMD_REFRESH       0x12
#define RV3032_EEPROM_CMD_WRITE         0x21
#define RV3032_EEPROM_CMD_READ          0x22

#define RV3032_SECONDS_MASK             GENMASK(6, 0)
#define RV3032_MINUTES_MASK             GENMASK(6, 0)
#define RV3032_HOURS_MASK               GENMASK(5, 0)
#define RV3032_DATE_MASK                GENMASK(5, 0)
#define RV3032_WEEKDAY_MASK             GENMASK(2, 0)
#define RV3032_MONTH_MASK               GENMASK(4, 0)
#define RV3032_YEAR_MASK                GENMASK(7, 0)

/* The RV3032 only supports two-digit years. Leap years are correctly handled from 2000 to 2099 */
#define RV3032_YEAR_OFFSET              (2000 - 1900)

/* The RV3032 enumerates months 1 to 12 */
#define RV3032_MONTH_OFFSET 1

#define RV3032_EEBUSY_POLL_US           10000
#define RV3032_EEBUSY_TIMEOUT_MS        100

/* RTC time fields supported by the RV3032 */
#define RV3032_RTC_TIME_MASK                                                                 \
	(RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |      \
	 RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_MONTHDAY | RTC_ALARM_TIME_MASK_YEAR |     \
	 RTC_ALARM_TIME_MASK_WEEKDAY)

struct rv3032_config {
	const struct i2c_dt_spec i2c;
	uint8_t pmu;
};

struct rv3032_data {
	struct k_sem lock;
};

bool rv3032_validate_rtc_time(const struct rtc_time *timeptr, uint16_t mask)
{
	if ((mask & RTC_ALARM_TIME_MASK_SECOND) && (timeptr->tm_sec < 0 || timeptr->tm_sec > 59)) {
		return false;
	}

	if ((mask & RTC_ALARM_TIME_MASK_MINUTE) && (timeptr->tm_min < 0 || timeptr->tm_min > 59)) {
		return false;
	}

	if ((mask & RTC_ALARM_TIME_MASK_HOUR) && (timeptr->tm_hour < 0 || timeptr->tm_hour > 23)) {
		return false;
	}

	if ((mask & RTC_ALARM_TIME_MASK_MONTH) && (timeptr->tm_mon < 0 || timeptr->tm_mon > 11)) {
		return false;
	}

	if ((mask & RTC_ALARM_TIME_MASK_MONTHDAY) &&
	    (timeptr->tm_mday < 1 || timeptr->tm_mday > 31)) {
		return false;
	}

	if ((mask & RTC_ALARM_TIME_MASK_YEAR) && (timeptr->tm_year < 0 || timeptr->tm_year > 199)) {
		return false;
	}

	if ((mask & RTC_ALARM_TIME_MASK_WEEKDAY) &&
	    (timeptr->tm_wday < 0 || timeptr->tm_wday > 6)) {
		return false;
	}

	if ((mask & RTC_ALARM_TIME_MASK_YEARDAY) &&
	    (timeptr->tm_yday < 0 || timeptr->tm_yday > 365)) {
		return false;
	}

	if ((mask & RTC_ALARM_TIME_MASK_NSEC) &&
	    (timeptr->tm_nsec < 0 || timeptr->tm_nsec > 999999999)) {
		return false;
	}

	return true;
}

static void rv3032_lock_sem(const struct device *dev)
{
	struct rv3032_data *data = dev->data;

	(void)k_sem_take(&data->lock, K_FOREVER);
}

static void rv3032_unlock_sem(const struct device *dev)
{
	struct rv3032_data *data = dev->data;

	k_sem_give(&data->lock);
}

static int rv3032_read_regs(const struct device *dev, uint8_t addr, void *buf, size_t len)
{
	const struct rv3032_config *config = dev->config;
	int err;

	err = i2c_write_read_dt(&config->i2c, &addr, sizeof(addr), buf, len);
	if (err) {
		LOG_ERR("failed to read reg addr 0x%02x, len %d (err %d)", addr, len, err);
		return err;
	}

	return 0;
}

static int rv3032_read_reg8(const struct device *dev, uint8_t addr, uint8_t *val)
{
	return rv3032_read_regs(dev, addr, val, sizeof(*val));
}

static int rv3032_write_regs(const struct device *dev, uint8_t addr, void *buf, size_t len)
{
	const struct rv3032_config *config = dev->config;
	uint8_t block[sizeof(addr) + len];
	int err;

	block[0] = addr;
	memcpy(&block[1], buf, len);

	err = i2c_write_dt(&config->i2c, block, sizeof(block));
	if (err) {
		LOG_ERR("failed to write reg addr 0x%02x, len %d (err %d)", addr, len, err);
		return err;
	}

	return 0;
}

static int rv3032_write_reg8(const struct device *dev, uint8_t addr, uint8_t val)
{
	return rv3032_write_regs(dev, addr, &val, sizeof(val));
}

static int rv3032_update_reg8(const struct device *dev, uint8_t addr, uint8_t mask, uint8_t val)
{
	const struct rv3032_config *config = dev->config;
	int err;

	err = i2c_reg_update_byte_dt(&config->i2c, addr, mask, val);
	if (err) {
		LOG_ERR("failed to update reg addr 0x%02x, mask 0x%02x, val 0x%02x (err %d)", addr,
			mask, val, err);
		return err;
	}

	return 0;
}

static int rv3032_eeprom_wait_busy(const struct device *dev)
{
	uint8_t status = 0;
	int err;
	int64_t timeout_time = k_uptime_get() + RV3032_EEBUSY_TIMEOUT_MS;

	/* Wait while the EEPROM is busy */
	for (;;) {
		err = rv3032_read_reg8(dev, RV3032_REG_STATUS_TEMP, &status);
		if (err) {
			return err;
		}

		if (!(status & RV3032_STATUS_TEMP_EEBUSY)) {
			break;
		}

		if (k_uptime_get() > timeout_time) {
			return -ETIME;
		}

		k_busy_wait(RV3032_EEBUSY_POLL_US);
	}

	return 0;
}

static int rv3032_exit_eerd(const struct device *dev)
{
	return rv3032_update_reg8(dev, RV3032_REG_CONTROL1, RV3032_CONTROL1_EERD, 0);
}

static int rv3032_enter_eerd(const struct device *dev)
{
	uint8_t ctrl1;
	bool eerd;
	int ret;

	ret = rv3032_read_reg8(dev, RV3032_REG_CONTROL1, &ctrl1);
	if (ret) {
		return ret;
	}

	eerd = ctrl1 & RV3032_CONTROL1_EERD;
	if (eerd) {
		return 0;
	}

	ret = rv3032_update_reg8(dev, RV3032_REG_CONTROL1, RV3032_CONTROL1_EERD,
				 RV3032_CONTROL1_EERD);

	ret = rv3032_eeprom_wait_busy(dev);
	if (ret) {
		rv3032_exit_eerd(dev);
		return ret;
	}

	return ret;
}

static int rv3032_eeprom_command(const struct device *dev, uint8_t command)
{
	return rv3032_write_reg8(dev, RV3032_REG_EEPROM_COMMAND, command);
}

static int rv3032_update(const struct device *dev)
{
	int err;

	err = rv3032_eeprom_command(dev, RV3032_EEPROM_CMD_UPDATE);
	if (err) {
		goto exit_eerd;
	}

	err = rv3032_eeprom_wait_busy(dev);

exit_eerd:
	rv3032_exit_eerd(dev);

	return err;
}

static int rv3032_refresh(const struct device *dev)
{
	int err;

	err = rv3032_eeprom_command(dev, RV3032_EEPROM_CMD_REFRESH);
	if (err) {
		goto exit_eerd;
	}

	err = rv3032_eeprom_wait_busy(dev);

exit_eerd:
	rv3032_exit_eerd(dev);

	return err;
}

static int rv3032_update_cfg(const struct device *dev, uint8_t addr, uint8_t mask, uint8_t val)
{
	uint8_t val_old, val_new;
	int err;

	err = rv3032_read_reg8(dev, addr, &val_old);
	if (err) {
		return err;
	}

	val_new = (val_old & ~mask) | (val & mask);
	if (val_new == val_old) {
		return 0;
	}

	err = rv3032_enter_eerd(dev);
	if (err) {
		return err;
	}

	err = rv3032_write_reg8(dev, addr, val_new);
	if (err) {
		rv3032_exit_eerd(dev);
		return err;
	}

	return rv3032_update(dev);
}

static int rv3032_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	uint8_t date[7];
	int err;

	if (timeptr == NULL ||
	    !rv3032_validate_rtc_time(timeptr, RV3032_RTC_TIME_MASK) ||
	    (timeptr->tm_year < RV3032_YEAR_OFFSET)) {
		LOG_ERR("invalid time");
		return -EINVAL;
	}

	rv3032_lock_sem(dev);

	LOG_DBG("set time: year = %d, mon = %d, mday = %d, wday = %d, hour = %d, "
		"min = %d, sec = %d",
		timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, timeptr->tm_wday,
		timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);

	date[0] = bin2bcd(timeptr->tm_sec) & RV3032_SECONDS_MASK;
	date[1] = bin2bcd(timeptr->tm_min) & RV3032_MINUTES_MASK;
	date[2] = bin2bcd(timeptr->tm_hour) & RV3032_HOURS_MASK;
	date[3] = bin2bcd(timeptr->tm_wday) & RV3032_WEEKDAY_MASK;
	date[4] = bin2bcd(timeptr->tm_mday) & RV3032_DATE_MASK;
	date[5] = bin2bcd(timeptr->tm_mon + RV3032_MONTH_OFFSET) & RV3032_MONTH_MASK;
	date[6] = bin2bcd(timeptr->tm_year - RV3032_YEAR_OFFSET) & RV3032_YEAR_MASK;

	err = rv3032_write_regs(dev, RV3032_REG_SECONDS, &date, sizeof(date));
	if (err) {
		goto unlock;
	}

	/* Clear Power On Reset Flag */
	err = rv3032_update_reg8(dev, RV3032_REG_STATUS, RV3032_STATUS_PORF, 0);

unlock:
	rv3032_unlock_sem(dev);

	return err;
}

static int rv3032_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	uint8_t status;
	uint8_t date[7];
	int err;

	if (timeptr == NULL) {
		return -EINVAL;
	}

	err = rv3032_read_reg8(dev, RV3032_REG_STATUS, &status);
	if (err) {
		return err;
	}

	if (status & RV3032_STATUS_PORF) {
		/* Power On Reset Flag indicates invalid data */
		return -ENODATA;
	}

	err = rv3032_read_regs(dev, RV3032_REG_SECONDS, date, sizeof(date));
	if (err) {
		return err;
	}

	memset(timeptr, 0U, sizeof(*timeptr));
	timeptr->tm_sec = bcd2bin(date[0] & RV3032_SECONDS_MASK);
	timeptr->tm_min = bcd2bin(date[1] & RV3032_MINUTES_MASK);
	timeptr->tm_hour = bcd2bin(date[2] & RV3032_HOURS_MASK);
	timeptr->tm_wday = bcd2bin(date[3] & RV3032_WEEKDAY_MASK);
	timeptr->tm_mday = bcd2bin(date[4] & RV3032_DATE_MASK);
	timeptr->tm_mon = bcd2bin(date[5] & RV3032_MONTH_MASK) - RV3032_MONTH_OFFSET;
	timeptr->tm_year = bcd2bin(date[6] & RV3032_YEAR_MASK) + RV3032_YEAR_OFFSET;
	timeptr->tm_yday = -1;
	timeptr->tm_isdst = -1;

	LOG_DBG("get time: year = %d, mon = %d, mday = %d, wday = %d, hour = %d, "
		"min = %d, sec = %d",
		timeptr->tm_year, timeptr->tm_mon, timeptr->tm_mday, timeptr->tm_wday,
		timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);

	return 0;
}

static int rv3032_init(const struct device *dev)
{
	const struct rv3032_config *config = dev->config;
	struct rv3032_data *data = dev->data;
	uint8_t regs[3];
	uint8_t val;
	int err;

	k_sem_init(&data->lock, 1, 1);

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}


	/* Refresh the settings in the RAM with the settings from the EEPROM */
	err = rv3032_enter_eerd(dev);
	if (err) {
		return -ENODEV;
	}
	err = rv3032_refresh(dev);
	if (err) {
		return -ENODEV;
	}

	err = rv3032_update_cfg(dev,
    RV3032_REG_PMU,
    RV3032_PMU_TCM | RV3032_PMU_TCR | RV3032_PMU_BSM,
    config->pmu
  );
	if (err) {
		return -ENODEV;
	}

  struct rtc_time rtctime;
	err = rtc_get_time(dev, &rtctime);
	if (-ENODATA == err) {
		LOG_WRN("RTC not set");
	} else if (err < 0) {
		return -ENODEV;
	} else {
    LOG_INF("RTC time %04d-%02d-%02dT%02d:%02d:%02d.%03d", 
      (rtctime.tm_year + 1900), (rtctime.tm_mon + 1), rtctime.tm_mday, 
      rtctime.tm_hour, rtctime.tm_min, rtctime.tm_sec, (rtctime.tm_nsec / 1000000)
    );
  }

	return 0;
}

static const struct rtc_driver_api rv3032_driver_api = {
	.set_time = rv3032_set_time,
	.get_time = rv3032_get_time,
};

#define RV3032_BSM_FROM_DT_INST(inst)                                                              \
	UTIL_CAT(RV3032_BSM_, DT_INST_STRING_UPPER_TOKEN(inst, backup_switch_mode))

#define RV3032_TCM_FROM_DT_INST(inst)                                                              \
	UTIL_CAT(RV3032_TCM_, DT_INST_STRING_UPPER_TOKEN(inst, charge_switch_mode))

#define RV3032_PMU_FROM_DT_INST(inst)                                                           \
	((FIELD_PREP(RV3032_PMU_BSM, RV3032_BSM_FROM_DT_INST(inst))) |                          \
	 (FIELD_PREP(RV3032_PMU_TCR, DT_INST_ENUM_IDX_OR(inst, trickle_resistor_ohms, 0))) |    \
	 (FIELD_PREP(RV3032_PMU_TCM, RV3032_TCM_FROM_DT_INST(inst))))

#define RV3032_INIT(inst)                                                                          \
	static const struct rv3032_config rv3032_config_##inst = {                                 \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.pmu = RV3032_PMU_FROM_DT_INST(inst)};                                        \
                                                                                                   \
	static struct rv3032_data rv3032_data_##inst;                                              \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &rv3032_init, NULL, &rv3032_data_##inst,                       \
			      &rv3032_config_##inst, POST_KERNEL, CONFIG_RTC_INIT_PRIORITY,        \
			      &rv3032_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RV3032_INIT)
