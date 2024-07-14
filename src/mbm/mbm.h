/* 

  Purpose: Modbus RTU master

  Board: MTSATLAN-B

  Copyright (c) 2024 HAV

  This driver configure modbus client, 
  add usefull API and some shell commands.

 */

#ifndef _MBM_H_
#define _MBM_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/drivers/uart.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===========================================================================

#define MBM_RESP_TOUT_DEFAULT		        500 // msec
#define MBM_BAUDRATE_DEFAULT            9600
#define MBM_PARITY_DEFAULT	            UART_CFG_PARITY_NONE
#define MBM_STOPBITS_DEFAULT			      UART_CFG_STOP_BITS_2

#define MBM_BITS_UCHAR									8U
#define MBM_MAX_ALLOWABLE_SLAVEID       247U
#define MBM_MAX_ALLOWABLE_REGS					124U

typedef enum e_modbus_fun {
	MODBUS_FUN_NONE                 =	0,
	MODBUS_READ_COILS               =	1,
	MODBUS_READ_DISCRETEINPUTS      =	2,
	MODBUS_READ_HOLDINGREGISTERS    =	3,
	MODBUS_READ_INPUTREGISTERS      =	4,
	MODBUS_WRITE_COIL               =	5,
	MODBUS_WRITE_REGISTER           =	6,
	MODBUS_WRITE_COILS              =	15,
	MODBUS_WRITE_REGISTERS          =	16
} modbus_fun_t;

// Status and Exception Codes
// Custom result codes used internally and for callbacks but never used for Modbus responce
typedef enum e_modbus_status {
  MODBUS_STATUS_SUCCESS 									    =	0x00,         // Custom. No error
  MODBUS_STATUS_EX_ILLEGAL_FUNCTION			      =	0x01,
  MODBUS_STATUS_EX_ILLEGAL_DATA_ADDRESS	      =	0x02,
  MODBUS_STATUS_EX_ILLEGAL_DATA_VALUE		      =	0x03,
  MODBUS_STATUS_EX_SLAVE_DEVICE_FAILURE	      =	0x04,
  MODBUS_STATUS_EX_ACKNOWLEDGE						    =	0x05,
  MODBUS_STATUS_EX_SLAVE_BUSY						      =	0x06,
  MODBUS_STATUS_EX_MEMORY_PARITY_ERROR		    =	0x08,
  MODBUS_STATUS_EX_DEVICE_PATH_FAILED		      =	0x0A,
  MODBUS_STATUS_EX_DEVICE_FAILED_TO_RESPOND		=	0x0B,
  MODBUS_STATUS_CRC_ERROR								      =	(-EIO),       // Custom. Returned result crc error
  MODBUS_STATUS_INT_ERROR								      =	(-ENOTSUP),   // Custom. Unexpected master error
  MODBUS_STATUS_FRAME_ERROR							      =	(-EMSGSIZE),  // Custom. Inpud data size mismach
  MODBUS_STATUS_SLAVE_DEVICE_TIMEDOUT		      =	(-ETIMEDOUT), // Custom. Operation not finished within reasonable time
  MODBUS_STATUS_BAD_ID              		      =	(-EFAULT),    // Custom. Bad ID (ID = 0)
  MODBUS_STATUS_UNKNOWN             		      =	(-ENODEV)     // Custom.
} modbus_status_t;

// ===========================================================================

bool mbm_pollraw(uint8_t *pdu, size_t *reqlen, size_t *replen, bool hascrc);
int mbm_get_pollraw_delay(void);
modbus_status_t mbm_poll(uint8_t id, modbus_fun_t fun, uint16_t add, uint16_t cnt, void *ptr, bool logerr);
int mbm_configure(uint32_t baudrate, enum uart_config_parity parity, enum uart_config_stop_bits stopbits, uint32_t timeout);
bool mbm_is_ready(void);
uint32_t mbm_packets(void);
uint32_t mbm_errors(void);
char *mbm_status_str(modbus_status_t status);
uint16_t mbm_get_reg(uint16_t *regsptr, int16_t index);
void mbm_set_reg(uint16_t * regsptr, int16_t index, uint16_t value);
bool mbm_get_bit(uint8_t * bitsptr, int16_t index);
void mbm_set_bit(uint8_t *bitsptr, int16_t index, bool value);

// ===========================================================================

#ifdef __cplusplus
}
#endif

#endif  // _MBM_H_
