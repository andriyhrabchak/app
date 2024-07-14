/* 

  Purpose: Modbus RTU master

  Board: MTSATLAN-B

  Copyright (c) 2023 HAV

  This driver configure modbus client, 
  add usefull API and some shell commands.

 */

#include <string.h>
#include <stdlib.h>

#include <zephyr/modbus/modbus.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/shell/shell.h>

#include <mbm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mbm, CONFIG_MODBUS_LOG_LEVEL);

// ===========================================================================
// ===========================================================================

/* Client's mutually exclusive access */
static struct k_mutex iface_lock;

static int client_iface;
static struct modbus_iface_param client_param;
static bool ready;
static uint32_t tot_error_cnt, tot_packet_cnt;

// ===========================================================================
// ===========================================================================
// Access to Modbus RTU data

static const uint8_t mbmFlagMasks[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

void mbm_set_bit(uint8_t * bitsptr, int16_t index, bool value) {
	if (value) bitsptr[index / MBM_BITS_UCHAR] |= mbmFlagMasks[index % MBM_BITS_UCHAR];
	else bitsptr[index / MBM_BITS_UCHAR] &= (uint8_t)~mbmFlagMasks[index % MBM_BITS_UCHAR];
}

bool mbm_get_bit(uint8_t * bitsptr, int16_t index) {
	if (bitsptr[index / MBM_BITS_UCHAR] & mbmFlagMasks[index % MBM_BITS_UCHAR]) return true;
	else return false;
}

void mbm_set_reg(uint16_t * regsptr, int16_t index, uint16_t value) {
	regsptr[index] = value;
}

uint16_t mbm_get_reg(uint16_t * regsptr, int16_t index) {
	return regsptr[index];
}

char *mbm_status_str(modbus_status_t status) {
  static char buf[30];

  switch (status) {
  case MODBUS_STATUS_SUCCESS: strcpy(buf, "success"); break;
  case MODBUS_STATUS_EX_ILLEGAL_FUNCTION: strcpy(buf, "illegal function"); break;
  case MODBUS_STATUS_EX_ILLEGAL_DATA_ADDRESS: strcpy(buf, "illegal data address"); break;
  case MODBUS_STATUS_EX_ILLEGAL_DATA_VALUE: strcpy(buf, "illegal data value"); break;
  case MODBUS_STATUS_EX_SLAVE_DEVICE_FAILURE: strcpy(buf, "device failure"); break;
  case MODBUS_STATUS_EX_ACKNOWLEDGE: strcpy(buf, "acknowledge"); break;
  case MODBUS_STATUS_EX_SLAVE_BUSY: strcpy(buf, "device busy"); break;
  case MODBUS_STATUS_EX_MEMORY_PARITY_ERROR: strcpy(buf, "memory parity error"); break;
  case MODBUS_STATUS_EX_DEVICE_PATH_FAILED: strcpy(buf, "device path failed"); break;
  case MODBUS_STATUS_EX_DEVICE_FAILED_TO_RESPOND: strcpy(buf, "device failed to respond"); break;
  case MODBUS_STATUS_INT_ERROR: strcpy(buf, "internal error"); break;
  case MODBUS_STATUS_FRAME_ERROR: strcpy(buf, "frame size error"); break;
  case MODBUS_STATUS_CRC_ERROR: strcpy(buf, "frame crc error"); break;
  case MODBUS_STATUS_BAD_ID: strcpy(buf, "bad id (id = 0)"); break;
  case MODBUS_STATUS_SLAVE_DEVICE_TIMEDOUT: strcpy(buf, "responce timeout"); break;
  default: strcpy(buf, "unknown error");
  }
  return buf;
}

// ===========================================================================
// ===========================================================================

static int64_t pollraw_delay; // in ms

/*
 * Function to get measured responce delay in raw mode
 * 
 * Return: Responce delay in milliseconds.
 */
int mbm_get_pollraw_delay(void) {
  return (int)pollraw_delay;
};

/* 
  Polls with raw data Modbus RTU slave (server). Polls in blocking way
  This function overwrites PDU passed by the pointer.

  Parameters: 
    pdu - bytes array to exchange with server
    reqlen - length to send
    replen - length to receive
    hascrc = if true, pdu has crc field

  Return: true if the function was successful.
 */
bool mbm_pollraw(uint8_t *pdu, size_t *reqlen, size_t *replen, bool hascrc) {

  if (pdu[0] == 0) {
    return false;
  }

	k_mutex_lock(&iface_lock, K_FOREVER);


  struct modbus_adu send_adu;
  send_adu.unit_id = pdu[0];
  send_adu.fc = pdu[1];
  send_adu.length = *reqlen - 2;
  if (hascrc) send_adu.length -= 2;
  memcpy(send_adu.data, &pdu[2], send_adu.length);

  // LOG_HEXDUMP_DBG(send_adu.data, send_adu.length, "RTU server poll DATA:");
  pollraw_delay = k_uptime_get();
  int err = modbus_raw_backend_txn(client_iface, &send_adu);
  pollraw_delay = k_uptime_get() - pollraw_delay;

  if (err == -ENOTSUP || err == -ENODEV) {

    k_mutex_unlock(&iface_lock);

    return false;
  } else if (err < 0) {
    LOG_ERR("Failed with error: %s", mbm_status_str(err));

    k_mutex_unlock(&iface_lock);

    return false;
  } else {

    // LOG_HEXDUMP_DBG(send_adu.data, send_adu.length, "RTU server responce DATA:");
    pdu[0] = send_adu.unit_id;
    pdu[1] = send_adu.fc;
    memcpy(&pdu[2], send_adu.data, send_adu.length);
    *replen = send_adu.length + 2;
    if (hascrc) {
      pdu[(*replen)++] = (uint8_t)((send_adu.crc) & 0xff);
      pdu[(*replen)++] = (uint8_t)((send_adu.crc >> 8) & 0xff);
    }

    k_mutex_unlock(&iface_lock);

    return true;
  }
}

/* 
  Poll Modbus RTU slave (server) in blocking way

  Parameters: 
    id - server ID
    fun - function
    add - bits or registers address
    cnt - bits or registers count
    ptr - bits or registers array pointer
    logerr - if true, then logs verbosely all errors

  Return: MODBUS_STATUS_SUCCESS (0) if the function was successful.
    IF return value < 0, then master (client) error occurs.
    If return value > 0, then slave (server) exception occurs.
 */
modbus_status_t mbm_poll(uint8_t id, modbus_fun_t fun, uint16_t add, uint16_t cnt, void *ptr, bool logerr) {
  modbus_status_t err = MODBUS_STATUS_UNKNOWN;

  if (id == 0) {
    return MODBUS_STATUS_BAD_ID;
  }

	k_mutex_lock(&iface_lock, K_FOREVER);

  tot_packet_cnt++;

  switch (fun) {
  case MODBUS_READ_COILS:
    err = (modbus_status_t)modbus_read_coils(client_iface, id, add, (uint8_t *)ptr, cnt);
    break;
  case MODBUS_READ_DISCRETEINPUTS:
    err = (modbus_status_t)modbus_read_dinputs(client_iface, id, add, (uint8_t *)ptr, cnt);
    break;
  case MODBUS_WRITE_COIL:
    err = (modbus_status_t)modbus_write_coil(client_iface, id, add, *((bool *)ptr));
    break;
  case MODBUS_WRITE_COILS:
    err = (modbus_status_t)modbus_write_coils(client_iface, id, add, (uint8_t *)ptr, cnt);
    break;
  case MODBUS_READ_HOLDINGREGISTERS:
    err = (modbus_status_t)modbus_read_holding_regs(client_iface, id, add, (uint16_t *)ptr, cnt);
    break;
  case MODBUS_READ_INPUTREGISTERS:
    err = (modbus_status_t)modbus_read_input_regs(client_iface, id, add, (uint16_t *)ptr, cnt);
    break;
  case MODBUS_WRITE_REGISTER:
    err = (modbus_status_t)modbus_write_holding_reg(client_iface, id, add, *((uint16_t *)ptr));
    break;
  case MODBUS_WRITE_REGISTERS:
    err = (modbus_status_t)modbus_write_holding_regs(client_iface, id, add, (uint16_t *)ptr, cnt);
    break;
  default:
    err = MODBUS_STATUS_INT_ERROR;
  }

  if (err == MODBUS_STATUS_SUCCESS) {
    // inout_led_on(LED_STAT);
    // k_msleep(10);
    // inout_led_off(LED_STAT);
    k_msleep(10);
  } else {
    tot_error_cnt++;
    if (logerr) {
      LOG_ERR("Failed with error: %s (fun: %02u, add: %u, cnt: %u)", mbm_status_str(err), fun, add, cnt);
    }
  }

  k_mutex_unlock(&iface_lock);

  return err;
}

/* 
  Configure Modbus RTU master serial line.

  Parameters:
    baudrate: Baudrate of the serial line in bits per second
    parity: UART_CFG_PARITY_NONE, UART_CFG_PARITY_ODD, UART_CFG_PARITY_EVEN
    stopbits: UART_CFG_STOP_BITS_1, UART_CFG_STOP_BITS_2
    timeout: Amount of time client will wait for a response from the server in msec.

  Return: 0 if the function was successful
*/
int mbm_configure(uint32_t baudrate, enum uart_config_parity parity, enum uart_config_stop_bits stopbits, uint32_t timeout) {

  k_mutex_lock(&iface_lock, K_FOREVER);

  if (
    (baudrate == 1200 || baudrate == 2400 || baudrate == 4800 || baudrate == 9600) && 
    (parity == UART_CFG_PARITY_NONE || parity == UART_CFG_PARITY_ODD || parity == UART_CFG_PARITY_EVEN) && 
    (stopbits == UART_CFG_STOP_BITS_1 || stopbits == UART_CFG_STOP_BITS_2)  &&
    (timeout != 0)) {
    /* Correct parameters */

    client_param.mode = MODBUS_MODE_RTU;
    client_param.rx_timeout = timeout * 1000;
    client_param.serial.baud = baudrate;
    client_param.serial.parity = parity;
    client_param.serial.stop_bits_client = stopbits;
    modbus_disable(client_iface);
    int err = modbus_init_client(client_iface, client_param);
    if (err) {
      LOG_ERR("Modbus RTU master configuration failed. Err: %i", err);
      k_mutex_unlock(&iface_lock);
      return err;
    }

    LOG_INF("Modbus RTU master serial: %u %s %s (%d ms)",
      client_param.serial.baud,
      ((client_param.serial.parity == UART_CFG_PARITY_EVEN) ? "E" : 
      ((client_param.serial.parity == UART_CFG_PARITY_ODD) ? "O" : "N")),
      ((client_param.serial.stop_bits_client == UART_CFG_STOP_BITS_2) ? "2" : "1"),
      (client_param.rx_timeout / 1000)
    );

    k_mutex_unlock(&iface_lock);

    return 0;

  } else {

    LOG_ERR("Modbus RTU master wrong configuration parameters");
    k_mutex_unlock(&iface_lock);
    return -EINVAL;
    
  }

}

/* 
  Return true if modbus rtu client was initialized correctly
 */
bool mbm_is_ready(void) {
  return ready;
}

/* 
  Return total polls count
 */
uint32_t mbm_packets(void) {
  return tot_packet_cnt;
}

/* 
  Return total poll errors count
 */
uint32_t mbm_errors(void) {
  return tot_error_cnt;
}

static int mbm_init_resources(void) {

  client_iface = modbus_iface_get_by_name(DEVICE_DT_NAME(DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_modbus_serial)));
	if (client_iface < 0) {
		LOG_ERR("Get Modbus RTU master interface failed. Err: %i", client_iface);
		return client_iface;
	}

  client_param.mode = MODBUS_MODE_RTU;
  client_param.rx_timeout = MBM_RESP_TOUT_DEFAULT * 1000;
  client_param.serial.baud = MBM_BAUDRATE_DEFAULT;
  client_param.serial.parity = MBM_PARITY_DEFAULT;
  client_param.serial.stop_bits_client = MBM_STOPBITS_DEFAULT;
  int err = modbus_init_client(client_iface, client_param);
	if (err) {
		LOG_ERR("Modbus RTU master initialization failed. Err: %i", err);
		return err;
	}

  LOG_INF("Modbus RTU master serial: %u %s %s (%d ms)",
    client_param.serial.baud,
    ((client_param.serial.parity == UART_CFG_PARITY_EVEN) ? "E" : 
    ((client_param.serial.parity == UART_CFG_PARITY_ODD) ? "O" : "N")),
    ((client_param.serial.stop_bits_client == UART_CFG_STOP_BITS_2) ? "2" : "1"),
    (client_param.rx_timeout / 1000)
  );

  k_mutex_init(&iface_lock);

  ready = true;
  
  return 0;
}

SYS_INIT(mbm_init_resources, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// ===========================================================================
// ===========================================================================

static int cmd_info(const struct shell *shell, size_t argc, char **argv) {

	shell_fprintf(shell, SHELL_NORMAL, "%u %s %s (%d ms)\n",
    client_param.serial.baud,
    ((client_param.serial.parity == UART_CFG_PARITY_EVEN) ? "E" : 
    ((client_param.serial.parity == UART_CFG_PARITY_ODD) ? "O" : "N")),
    ((client_param.serial.stop_bits_client == UART_CFG_STOP_BITS_2) ? "2" : "1"),
    (client_param.rx_timeout / 1000)
  );

	return 0;
}

/* Syntax: config <br> <par=N|O|E> <sb=1|2> [<tout>] */
static int cmd_config(const struct shell *shell, size_t argc, char **argv) {

  uint32_t baudrate = strtol(argv[1], NULL, 10);

  enum uart_config_parity parity;
  if (!strcmp(argv[2], "N")) {
    parity = UART_CFG_PARITY_NONE;
  } else if (!strcmp(argv[2], "O")) {
    parity = UART_CFG_PARITY_ODD;
  } else if (!strcmp(argv[2], "E")) {
    parity = UART_CFG_PARITY_EVEN;
  } else {
    shell_error(shell, "Wrong par");
    return -ENOEXEC;  
  }
  
  enum uart_config_stop_bits stopbits;
  if (!strcmp(argv[3], "1")) {
    stopbits = UART_CFG_STOP_BITS_1;
  } else if (!strcmp(argv[3], "2")) {
    stopbits = UART_CFG_STOP_BITS_2;
  } else {
    shell_error(shell, "Wrong sb");
    return -ENOEXEC;  
  }

  uint32_t timeout = 0;
  if (argc == 5) {
    timeout = strtol(argv[4], NULL, 10);
  } else {
    timeout = client_param.rx_timeout / 1000;
  }
  
	int ret = mbm_configure(baudrate, parity, stopbits, timeout);
  if (ret != 0) {
    shell_error(shell, "Failed to configure (err: %d)", ret);
    return -ENOEXEC;
  }

	return 0;
}

/* Read subcommands array. Must be placed alphabetically */
static const char *readsub[] = {
  "coils",
  "holdregs",
  "inpregs",
  "inputs"
};
/* Must be consistent with subcommands array */
static const char *readhelp[] = {
  "read coils (fun = 01)",
  "read holding registers (fun = 03)",
  "read input registers (fun = 04)",
  "read discrete inputs (fun = 02)"
};
/* Must be consistent with subcommands array */
static modbus_fun_t readfun[] = {
  MODBUS_READ_COILS,
  MODBUS_READ_HOLDINGREGISTERS,
  MODBUS_READ_INPUTREGISTERS,
  MODBUS_READ_DISCRETEINPUTS
};

/* Syntax: read <fun_subcommnd> <id> <add> <cnt> */
static int cmd_read(const struct shell *shell, size_t argc, char **argv) {
  
  modbus_fun_t fun = MODBUS_FUN_NONE;
  for (int i = 0; i < sizeof(*readsub); i++) {
    if (!strcmp(argv[1], readsub[i])) {
      fun = readfun[i];
      break;
    }
  }
  if (fun == MODBUS_FUN_NONE) {
    shell_error(shell, "Wrong subcommand");
    return -ENOEXEC;
  }
  
  uint8_t id = strtol(argv[2], NULL, 10);
  if (id > MBM_MAX_ALLOWABLE_SLAVEID) {
    shell_error(shell, "Wrong id");
    return -ENOEXEC;
  }
  
  uint16_t add = strtol(argv[3], NULL, 10);
  
	uint16_t cnt = strtol(argv[4], NULL, 10);
  if (cnt > MBM_MAX_ALLOWABLE_REGS) {
    shell_error(shell, "Wrong cnt");
    return -ENOEXEC;
  }

  size_t datasize = 0;
  switch (fun) {
    case MODBUS_READ_COILS:
    case MODBUS_READ_DISCRETEINPUTS:
      datasize = cnt / MBM_BITS_UCHAR + ((cnt % MBM_BITS_UCHAR) ? 1 : 0);
      break;
    case MODBUS_READ_HOLDINGREGISTERS:
    case MODBUS_READ_INPUTREGISTERS:
      datasize = 2 * cnt;
      break;
    default:
      break;
  }
  void *dataptr = k_malloc(datasize);
  if (!dataptr) {
    shell_error(shell, "Out of memmory");
    return -ENOMEM;
  }
  
  modbus_status_t err = mbm_poll(id, fun, add, cnt, dataptr, false);
	if (err == MODBUS_STATUS_SUCCESS) {
    shell_print(shell,
      "Readout (id %u, fun %02u, add %u, cnt %u):", 
      id, fun, add, cnt
    );
    for (int i = 0; i < cnt; i++) {
      switch (fun) {
      case MODBUS_READ_COILS:
      case MODBUS_READ_DISCRETEINPUTS:
        shell_fprintf(shell, SHELL_NORMAL, "%u ", mbm_get_bit((uint8_t *)dataptr, i));
        break;
      case MODBUS_READ_HOLDINGREGISTERS:
      case MODBUS_READ_INPUTREGISTERS:
        shell_fprintf(shell, SHELL_NORMAL, "%04X ", mbm_get_reg((uint16_t *)dataptr, i));
        break;
      default:
        break;
      }
    }
    shell_fprintf(shell, SHELL_NORMAL, "\n");
	} else {
    shell_error(shell, 
      "Failed with error '%s' (id %u, fun %02u, add %u, cnt %u)", 
      mbm_status_str(err), id, fun, add, cnt
    );
  }
  k_free(dataptr);

	return 0;
}

/* Function returning read subcommands dynamic array */
static void read_fun_get(size_t idx, struct shell_static_entry *entry) {
  if (idx < 4) {
    entry->syntax = readsub[idx];
    entry->help = readhelp[idx];
  } else {
    /* end of array */
    entry->syntax = NULL;
    entry->help = NULL;
  }
	entry->handler = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(sub_read_cmds, read_fun_get);

/* Write subcommands array. Must be placed alphabetically */
const char *writesub[] = {
  "coil",
  "coils",
  "holdreg",
  "holdregs"
};
/* Must be consistent with subcommands array */
const char *writehelp[] = {
  "write coil (fun = 05)",
  "write multiple coils (fun = 15)",
  "write holding register (fun = 06)",
  "write multiple holding register (fun = 16)"
};
/* Must be consistent with subcommands array */
modbus_fun_t writefun[] = {
  MODBUS_WRITE_COIL,
  MODBUS_WRITE_COILS,
  MODBUS_WRITE_REGISTER,
  MODBUS_WRITE_REGISTERS
};

/* Syntax: write <fun_subcommnd> <id> <add> <val1> [<val2> ...] */
static int cmd_write(const struct shell *shell, size_t argc, char **argv) {
  
  modbus_fun_t fun = MODBUS_FUN_NONE;
  for (int i = 0; i < sizeof(*writesub); i++) {
    if (!strcmp(argv[1], writesub[i])) {
      fun = writefun[i];
      break;
    }
  }
  if (fun == MODBUS_FUN_NONE) {
    shell_error(shell, "Wrong subcommand");
    return -ENOEXEC;
  }
  
  uint8_t id = strtol(argv[2], NULL, 10);
  if (id > MBM_MAX_ALLOWABLE_SLAVEID) {
    shell_error(shell, "Wrong id");
    return -ENOEXEC;
  }
  
  uint16_t add = strtol(argv[3], NULL, 10);

  uint16_t cnt = argc - 4;
  if (cnt > MBM_MAX_ALLOWABLE_REGS) {
    shell_error(shell, "Wrong cnt");
    return -ENOEXEC;
  }

  char **data = &argv[4];
  void *dataptr = NULL;
  size_t datasize = 0;
  switch (fun) {
    case MODBUS_WRITE_COIL:
      cnt = 1;
    case MODBUS_WRITE_COILS:
      datasize = cnt / MBM_BITS_UCHAR + ((cnt % MBM_BITS_UCHAR) ? 1 : 0);
      dataptr = k_malloc(datasize);
      if (!dataptr) {
        shell_error(shell, "Out of memmory");
        return -ENOMEM;
      }
      for (int i = 0; i < cnt; i++) {
        mbm_set_bit((uint8_t *)dataptr, i, (bool)strtol(data[i], NULL, 2));
      }
      break;
    case MODBUS_WRITE_REGISTER:
      cnt = 1;
    case MODBUS_WRITE_REGISTERS:
      datasize = 2 * cnt;
      dataptr = k_malloc(datasize);
      if (!dataptr) {
        shell_error(shell, "Out of memmory");
        return -ENOMEM;
      }
      for (int i = 0; i < cnt; i++) {
        mbm_set_reg((uint16_t *)dataptr, i, (uint16_t)strtol(data[i], NULL, 16));
      }
      break;
    default:
      break;
  }
  
  modbus_status_t err = mbm_poll(id, fun, add, cnt, dataptr, false);
	if (err == MODBUS_STATUS_SUCCESS) {
    shell_print(shell,
      "Write successfull (id %u, fun %02u, add %u, cnt %u):", 
      id, fun, add, cnt
    );
	} else {
    shell_error(shell, 
      "Failed with error '%s' (id %u, fun %02u, add %u, cnt %u)", 
      mbm_status_str(err), id, fun, add, cnt
    );
  }
  if (dataptr != NULL) {
    k_free(dataptr);
  }

	return 0;
}

/* Function returning write subcommand sdynamic array */
static void write_fun_get(size_t idx, struct shell_static_entry *entry) {
  if (idx < 4) {
    entry->syntax = writesub[idx];
    entry->help = writehelp[idx];
  } else {
    /* end of array */
    entry->syntax = NULL;
    entry->help = NULL;
  }
	entry->handler = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(sub_write_cmds, write_fun_get);

static int cmd_test_pdu(const struct shell *shell, size_t argc, char **argv) {

  if (argc <= 5) {
    shell_error(shell, "Wrong parameter count");
    return -ENOEXEC;
  }

	uint8_t buf[256];
  size_t buf_len = 0;
  for (int i = 1; i < argc; ++i) {
    buf[buf_len++] = strtol(argv[i], NULL, 16);
  }

  size_t replen;
  if (!mbm_pollraw(buf, &buf_len, &replen, false)) {
    shell_error(shell, "Responce error");
    return -ENOEXEC;
  }

  shell_print(shell, "Responce PDU without CRC:");
  shell_hexdump(shell, buf, replen);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_mbm,
	SHELL_CMD(info, NULL, "Print serial configuration", cmd_info),
	SHELL_CMD_ARG(config, NULL, 
    "Set serial configuration (baudrate parity stopbits resp_tout_ms)\n"
    "Usage: config <1200|2400|4800|9600> <N|O|E> <1|2> [<tout>]",
    cmd_config, 4, 1),
	SHELL_CMD_ARG(read, &sub_read_cmds, 
    "Read Modbus device\n"
    "Usage: read <subcmd> <id> <add> <cnt>,\n"
    "where readouts are in hex (like 04D3) or in bool (0|1)",
    cmd_read, 5, 0),
	SHELL_CMD_ARG(write, &sub_write_cmds, 
    "Write Modbus device\n"
    "Usage: write <subcmd> <id> <add> <val1> [<val2> ...],\n"
    "where valx are in hex (like 04D3) or in bool (0|1)",
    cmd_write, 5, (MBM_MAX_ALLOWABLE_REGS - 1)),
	SHELL_CMD_ARG(pdu, NULL, 
    "Poll with SERIAL PDU without CRC field. "
    "All values are in hex (like D3). "
    "Usage: modbus pdu <id> <fun> <val1> [<val2> ...]", 
    cmd_test_pdu, 4, (128-1)),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(modbus, &sub_mbm, "Modbus RTU master commands.", NULL);
