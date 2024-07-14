/* 

  Purpose: input\output expander driver

  Board: MTSATLAN-B

  Copyright (c) 2024 HAV

  This driver properly configure output ports of I\O expander 
  to retain some ports state during software resets and 
  add usefull API.
  
 */

#include <stdlib.h>
#include <zephyr/drivers/gpio.h>
#include <ioexp.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ioexp, CONFIG_GPIO_LOG_LEVEL);

static bool outputstate[OUTPUT_MAX_NUMBER];
static bool ready = false;

struct gpio {
	struct gpio_dt_spec spec;
	const char *name;
};

static const struct gpio led_pow = {
	.spec = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(green_led), gpios, {0}),
	.name = DT_PROP_OR(DT_NODELABEL(green_led), label, ""),
};
static const struct gpio led_stat = {
	.spec = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(yellow_led), gpios, {0}),
	.name = DT_PROP_OR(DT_NODELABEL(yellow_led), label, ""),
};
static const struct gpio led_net = {
	.spec = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(red_led), gpios, {0}),
	.name = DT_PROP_OR(DT_NODELABEL(red_led), label, ""),
};
static const struct gpio *leds[] = {
  &led_pow,
  &led_stat,
  &led_net
};

static const struct gpio rel1 = {
	.spec = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(relay_1), gpios, {0}),
	.name = DT_PROP_OR(DT_NODELABEL(relay_1), label, ""),
};
static const struct gpio rel2 = {
	.spec = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(relay_2), gpios, {0}),
	.name = DT_PROP_OR(DT_NODELABEL(relay_2), label, ""),
};
static const struct gpio rel3 = {
	.spec = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(relay_3), gpios, {0}),
	.name = DT_PROP_OR(DT_NODELABEL(relay_3), label, ""),
};
static const struct gpio rel4 = {
	.spec = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(relay_4), gpios, {0}),
	.name = DT_PROP_OR(DT_NODELABEL(relay_4), label, ""),
};
static const struct gpio *rels[] = {
  &rel1,
  &rel2,
  &rel3,
  &rel4
};

// ===========================================================================
// LEDs 

void ioexp_led_tgl(int channel) {
	gpio_pin_toggle_dt(&(leds[channel]->spec));
}

void ioexp_led_on(int channel) {
	gpio_pin_set_dt(&(leds[channel]->spec), 1);
}

void ioexp_led_off(int channel) {
	gpio_pin_set_dt(&(leds[channel]->spec), 0);
}

// ===========================================================================
// Outputs 

void ioexp_output_tgl(int channel) {
	if (!gpio_pin_toggle_dt(&(rels[channel]->spec))) outputstate[channel] = !outputstate[channel];
}

void ioexp_output_off(int channel) {
	if (!gpio_pin_set_dt(&(rels[channel]->spec), 0)) outputstate[channel] = false;
}

void ioexp_output_on(int channel) {
	if (!gpio_pin_set_dt(&(rels[channel]->spec), 1)) outputstate[channel] = true;
}

void ioexp_output_set(int channel, bool val) {
	if (!gpio_pin_set_dt(&(rels[channel]->spec), (int)val)) outputstate[channel] = val;
}

bool ioexp_output_read(int channel) {
  return outputstate[channel];
}

// ===========================================================================
// Status

/* 
  Return true if driver was initialized properly
 */
bool ioexp_is_ready(void) {
  return ready;
}

// ===========================================================================

static int ioexp_configure_pin(const struct gpio *pin, gpio_flags_t flags) {
	const struct gpio_dt_spec *spec = &pin->spec;
	int err;

	if (spec->port && !device_is_ready(spec->port)) {
		LOG_ERR("Error: %s device is not ready", spec->port->name);
		return -ENODEV;
	}

  err = gpio_pin_configure_dt(spec, flags);
  if (err == 0) {
    LOG_INF("Configured pin %d ('%s') on device %s", 
      spec->pin, pin->name, spec->port->name);
  } else {
    LOG_ERR("Error %d: failed to configure pin %d ('%s')", 
      err, spec->pin, pin->name);
    return err;
  }
 
  return 0;
}

static int ioexp_restore_pin(const struct gpio *pin) {
	const struct gpio_dt_spec *spec = &pin->spec;
  gpio_dt_flags_t dt_flags = spec->dt_flags;
	int err;
  int value;

	if (spec->port && !device_is_ready(spec->port)) {
		LOG_ERR("Error: %s device is not ready", 
      spec->port->name);
		return -ENODEV;
	}

  value = gpio_pin_get_dt(spec);
  if (dt_flags & GPIO_ACTIVE_LOW) {
    if (value == 0) {
      err = gpio_pin_configure_dt(spec, GPIO_OUTPUT_ACTIVE);
    } else {
      err = gpio_pin_configure_dt(spec, GPIO_OUTPUT_INACTIVE);
    }
  } else {
    if (value == 1) {
      err = gpio_pin_configure_dt(spec, GPIO_OUTPUT_ACTIVE);
    } else {
      err = gpio_pin_configure_dt(spec, GPIO_OUTPUT_INACTIVE);
    }
  }
  if (err == 0) {
    LOG_INF("Restored pin %d ('%s') on device %s (DT:%08X VL:%d)", 
      spec->pin, pin->name, spec->port->name, dt_flags, value);
  } else {
    LOG_ERR("Error %d: failed to restore pin %d ('%s')", 
      err, spec->pin, pin->name);
    return err;
  }

  return 0;
}

static int ioexp_init_resources(void) {
  int err = 0;

  err = ioexp_configure_pin(&led_pow, GPIO_OUTPUT_ACTIVE);
  if (err != 0) return err;

  err = ioexp_configure_pin(&led_stat, GPIO_OUTPUT_ACTIVE);
  if (err != 0) return err;

  err = ioexp_configure_pin(&led_net, GPIO_OUTPUT_ACTIVE);
  if (err != 0) return err;

  err = ioexp_restore_pin(&rel1);
  if (err != 0) return err;

  err = ioexp_restore_pin(&rel2);
  if (err != 0) return err;

  err = ioexp_restore_pin(&rel3);
  if (err != 0) return err;

  err = ioexp_restore_pin(&rel4);
  if (err != 0) return err;

  ready = true;

	return 0;
}

SYS_INIT(ioexp_init_resources, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

