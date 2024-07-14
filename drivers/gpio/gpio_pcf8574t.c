/**
 * Copyright (c)
 * 2022 Ithinx GmbH
 * 2023 Amrith Venkat Kesavamoorthi <amrith@mr-beam.org>
 * 2023 Mr Beam Lasers GmbH.
 * 2024 HAV
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @see https://www.nxp.com/docs/en/data-sheet/PCF8574_PCF8574A.pdf
 * 
 * Custom driver which supports 8-bit expander with outputs only 
 * and supports Zephyr Modbus library calls.
 */

#define DT_DRV_COMPAT nxp_pcf8574t

#include <zephyr/drivers/gpio/gpio_utils.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(pcf8574t, CONFIG_GPIO_LOG_LEVEL);

struct pcf8574t_pins_cfg {
	uint16_t configured_as_outputs; /* only 1 for output */
	uint16_t outputs_state;
};

/** Runtime driver data of the pcf8574t*/
struct pcf8574t_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	struct pcf8574t_pins_cfg pins_cfg;
	sys_slist_t callbacks;
	struct k_sem lock;
  struct k_work raw_work;
	const struct device *dev;
	int num_bytes;
};

/** Configuration data*/
struct pcf8574t_drv_cfg {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	struct i2c_dt_spec i2c;
};

#define RAW_WORK_MSGQ_SIZE 10
struct raw_work_data_s {
  uint16_t mask; 
  uint16_t value;
  uint16_t toggle;
};
struct k_msgq raw_work_msgq;
char __aligned(4) raw_work_msgq_buffer[RAW_WORK_MSGQ_SIZE * sizeof(struct raw_work_data_s)];

/**
 * @brief Reads the value of the pins from pcf8574t respectively from a connected device.
 *
 * @param dev Pointer to the device structure of the driver instance.
 * @param value Pointer to the input value. It contains the received Bytes(receives 2 Bytes for P0
 * and P1).
 *
 * @retval 0 If successful.
 * @retval Negative value for error code.
 */
static int pcf8574t_process_input(const struct device *dev, gpio_port_value_t *value)
{
	const struct pcf8574t_drv_cfg *drv_cfg = dev->config;
	struct pcf8574t_drv_data *drv_data = dev->data;
	int rc = 0;
	uint8_t rx_buf[2] = {0};

	rc = i2c_read_dt(&drv_cfg->i2c, rx_buf, drv_data->num_bytes);
	if (rc != 0) {
		LOG_ERR("%s: failed to read from device: %d", dev->name, rc);
		return -EIO;
	}

	if (value) {
		*value = sys_get_le16(rx_buf); /*format P07-P00 (bit7-bit0)*/
	}

	return rc;
}

/**
 * @brief This function reads a value from the connected device.
 * Does not work inside ISR.
 *
 * @param dev Pointer to the device structure of a port.
 * @param value Pointer to a variable where pin values will be stored.
 *
 * @retval 0 If successful.
 * @retval Negative value for error code.
 */
static int pcf8574t_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	struct pcf8574t_drv_data *drv_data = dev->data;
	int rc;

	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	/**
	 * Reading of the input port also clears the generated interrupt,
	 * thus the configured callbacks must be fired also here if needed.
	 */
	rc = pcf8574t_process_input(dev, value);

	k_sem_give(&drv_data->lock);

	return rc;
}

/* Work handler to deferred write to the i2c device */
static void pcf8574t_raw_work_handler(struct k_work *work)
{
	struct pcf8574t_drv_data *drv_data = CONTAINER_OF(work, struct pcf8574t_drv_data, raw_work);
  const struct device *dev = drv_data->dev;
	const struct pcf8574t_drv_cfg *drv_cfg = dev->config;
	int rc = 0;
	uint16_t tx_buf;
	uint8_t tx_buf_p[2];

  struct raw_work_data_s data;
  rc = k_msgq_get(&raw_work_msgq, &data, K_NO_WAIT);
  if (rc != 0) {
		LOG_ERR("Failed to get data from queue: %d", rc);
    return;
  }

	tx_buf = (drv_data->pins_cfg.outputs_state & ~data.mask);
	tx_buf |= (data.value & data.mask);
	tx_buf ^= data.toggle;
	sys_put_le16(tx_buf, tx_buf_p);

	rc = i2c_write_dt(&drv_cfg->i2c, tx_buf_p, drv_data->num_bytes);
	if (rc != 0) {
		LOG_ERR("%s: failed to write output port: %d", dev->name, rc);
    return;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);
	drv_data->pins_cfg.outputs_state = tx_buf;
	k_sem_give(&drv_data->lock);

  if (k_msgq_num_used_get(&raw_work_msgq) != 0) {
    /* Submit work handler for next message */
    k_work_submit(&drv_data->raw_work);
  }
}

/**
 * @brief This function realizes the write connection to the i2c device.
 *
 * @param dev A pointer to the device structure
 * @param mask A mask of bits to set some bits to LOW or HIGH
 * @param value The value which is written via i2c to the pcf8574t's output pins
 * @param toggle A way to toggle some bits with xor
 *
 * @retval 0 If successful.
 * @retval Negative value for error code.
 */
static int pcf8574t_port_set_raw(const struct device *dev, uint16_t mask, uint16_t value,
				uint16_t toggle)
{
	const struct pcf8574t_drv_cfg *drv_cfg = dev->config;
	struct pcf8574t_drv_data *drv_data = dev->data;
	int rc = 0;
	uint16_t tx_buf;
	uint8_t tx_buf_p[2];

	if ((drv_data->pins_cfg.configured_as_outputs & value) != value) {
		LOG_ERR("Pin(s) is/are configured as input which should be output.");
		return -EOPNOTSUPP;
	}

	if (k_is_in_isr()) {
		/* Offload and do not block during ISR */

    if (k_msgq_num_used_get(&raw_work_msgq) == 0) {
      /* Submit work handler for very first message */
      k_work_submit(&drv_data->raw_work);
    }

    /* Send data message to raw work queue */
    struct raw_work_data_s data = {
      .mask = mask,
      .value = value,
      .toggle = toggle
    };
    rc = k_msgq_put(&raw_work_msgq, &data, K_NO_WAIT);
    if (rc == -ENOMSG) {
      LOG_ERR("Raw work mesage queue is full");
      return -ENOMEM;
    }

	  return 0;
	}

	tx_buf = (drv_data->pins_cfg.outputs_state & ~mask);
	tx_buf |= (value & mask);
	tx_buf ^= toggle;
	sys_put_le16(tx_buf, tx_buf_p);

	rc = i2c_write_dt(&drv_cfg->i2c, tx_buf_p, drv_data->num_bytes);
	if (rc != 0) {
		LOG_ERR("%s: failed to write output port: %d", dev->name, rc);
		return -EIO;
	}
	k_sem_take(&drv_data->lock, K_FOREVER);
	drv_data->pins_cfg.outputs_state = tx_buf;
	k_sem_give(&drv_data->lock);

	return 0;
}

/**
 * @brief This function fills a dummy because the pcf8574t has no pins to configure.
 * You can use it to set some pins permanent to HIGH or LOW until reset. It uses the port_set_raw
 * function to set the pins of pcf8574t directly.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param pin The bit in the io register which is set to high
 * @param flags Flags like the GPIO direction or the state
 *
 * @retval 0 If successful.
 * @retval Negative value for error.
 */
static int pcf8574t_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct pcf8574t_drv_data *drv_data = dev->data;
	int ret = 0;
	uint16_t temp_pins = drv_data->pins_cfg.outputs_state;
	uint16_t temp_outputs = drv_data->pins_cfg.configured_as_outputs;

	if (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN | GPIO_DISCONNECTED | GPIO_SINGLE_ENDED | GPIO_INPUT)) {
		return -ENOTSUP;
	}
	if (flags & GPIO_OUTPUT) {
		drv_data->pins_cfg.configured_as_outputs |= BIT(pin);
		temp_outputs = drv_data->pins_cfg.configured_as_outputs;
	}
	if (flags & GPIO_OUTPUT_INIT_HIGH) {
		temp_pins |= (1 << pin);
	}
	if (flags & GPIO_OUTPUT_INIT_LOW) {
		temp_pins &= ~(1 << pin);
	}

	ret = pcf8574t_port_set_raw(dev, drv_data->pins_cfg.configured_as_outputs, temp_pins, 0);

	if (ret == 0) {
		k_sem_take(&drv_data->lock, K_FOREVER);
		drv_data->pins_cfg.outputs_state = temp_pins;
		drv_data->pins_cfg.configured_as_outputs = temp_outputs;
		k_sem_give(&drv_data->lock);
	}

	return ret;
}

/**
 * @brief Sets a value to the pins of pcf8574t
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param mask The bit mask which bits should be set
 * @param value	The value which should be set
 *
 * @retval 0 If successful.
 * @retval Negative value for error.
 */
static int pcf8574t_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
				       gpio_port_value_t value)
{
	return pcf8574t_port_set_raw(dev, (uint16_t)mask, (uint16_t)value, 0);
}

/**
 * @brief Sets some output pins of the pcf8574t
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param pins The pin(s) which will be set in a range from P17-P10..P07-P00
 *
 * @retval 0 If successful.
 * @retval Negative value for error.
 */
static int pcf8574t_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return pcf8574t_port_set_raw(dev, (uint16_t)pins, (uint16_t)pins, 0);
}

/**
 * @brief clear some bits
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param pins Pins which will be cleared
 *
 * @retval 0 If successful.
 * @retval Negative value for error.
 */
static int pcf8574t_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	return pcf8574t_port_set_raw(dev, (uint16_t)pins, 0, 0);
}

/**
 * @brief Toggle some bits
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param pins Pins which will be toggled
 *
 * @retval 0 If successful.
 * @retval Negative value for error.
 */
static int pcf8574t_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	return pcf8574t_port_set_raw(dev, 0, 0, (uint16_t)pins);
}

/** Initialize the pcf8574t */
static int pcf8574t_init(const struct device *dev)
{
	const struct pcf8574t_drv_cfg *drv_cfg = dev->config;
	struct pcf8574t_drv_data *drv_data = dev->data;
	int rc;

	if (!device_is_ready(drv_cfg->i2c.bus)) {
		LOG_ERR("%s is not ready", drv_cfg->i2c.bus->name);
		return -ENODEV;
	}

  /* Interpret the reset cause flags */
  uint32_t reset_cause;
  rc = hwinfo_get_reset_cause(&reset_cause);
  if (rc != 0) {
    LOG_ERR("Failed to get hwinfo: %d", rc);
    return -EIO;
  }
  LOG_DBG("Reset cause hwinfo: %08X", reset_cause);
  if (reset_cause & RESET_POR) {
    /* After Power On Reset, begin internal states from scratch */
    drv_data->pins_cfg.outputs_state = 0;
  } else {
    /* After other Resets, restore internal states from I/O expander */
    gpio_port_value_t rd_value;
    rc = pcf8574t_process_input(dev, &rd_value);
    if (rc != 0) {
      LOG_ERR("%s: failed to read inputs: %d", dev->name, rc);
      return -EIO;
    }
    LOG_WRN("%s: restore output states (%02X)", dev->name, rd_value);
    drv_data->pins_cfg.outputs_state = (uint16_t)rd_value;
  }
  drv_data->pins_cfg.configured_as_outputs = UINT16_MAX; // only outputs supported

  k_work_init(&drv_data->raw_work, &pcf8574t_raw_work_handler);
  k_msgq_init(&raw_work_msgq, raw_work_msgq_buffer, sizeof(struct raw_work_data_s), RAW_WORK_MSGQ_SIZE);

	return 0;
}

/** Realizes the functions of gpio.h for pcf8574t*/
static const struct gpio_driver_api pcf8574t_drv_api = {
	.pin_configure = pcf8574t_pin_configure,
	.port_get_raw = pcf8574t_port_get_raw,
	.port_set_masked_raw = pcf8574t_port_set_masked_raw,
	.port_set_bits_raw = pcf8574t_port_set_bits_raw,
	.port_clear_bits_raw = pcf8574t_port_clear_bits_raw,
	.port_toggle_bits = pcf8574t_port_toggle_bits,
};

#define GPIO_PCF8574T_INST(idx)                                                                     \
	static const struct pcf8574t_drv_cfg pcf8574t_cfg##idx = {                                   \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(idx),             \
			},                                                                         \
		.i2c = I2C_DT_SPEC_INST_GET(idx),                                                  \
	};                                                                                         \
	static struct pcf8574t_drv_data pcf8574t_data##idx = {                                       \
		.lock = Z_SEM_INITIALIZER(pcf8574t_data##idx.lock, 1, 1),                           \
		.dev = DEVICE_DT_INST_GET(idx),                                                    \
		.num_bytes = DT_INST_ENUM_IDX(idx, ngpios) + 1,                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, pcf8574t_init, NULL, &pcf8574t_data##idx, &pcf8574t_cfg##idx,      \
			      POST_KERNEL, CONFIG_GPIO_PCF8574T_INIT_PRIORITY, &pcf8574t_drv_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_PCF8574T_INST);
