/* 

  Purpose: input\output expander driver

  Board: MTSATLAN-B

  Copyright (c) 2024 HAV

  This driver properly configure output ports of I\O expander 
  to retain some ports state during software resets and 
  add usefull API.

 */

#ifndef _IOEXP_H_
#define _IOEXP_H_

#ifdef __cplusplus
extern "C" {
#endif

// ===========================================================================

#define OUTPUT_MAX_NUMBER 4
enum e_outputs {
	RELAY_OUT1=0,
	RELAY_OUT2,
	RELAY_OUT3,
	RELAY_OUT4
};

#define LED_MAX_NUMBER 3
enum e_leds {
	LED_POW=0,
	LED_STAT,
	LED_LAN
};

// ===========================================================================

void ioexp_led_tgl(int channel);
void ioexp_led_on(int channel);
void ioexp_led_off(int channel);
void ioexp_output_tgl(int channel);
void ioexp_output_off(int channel);
void ioexp_output_on(int channel);
void ioexp_output_set(int channel, bool val);
bool ioexp_output_read(int channel);
bool ioexp_is_ready(void);

// ===========================================================================

#ifdef __cplusplus
}
#endif

#endif  // _IOEXP_H_
