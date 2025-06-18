/*
 * ertc-datalogger.h
 *
 *  Created on: Apr 8, 2021
 */

#ifndef INC_ERTC_DATALOGGER_H_
#define INC_ERTC_DATALOGGER_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define TX_START_CMD	65
#define TX_STOP_CMD		66

#define datalog __attribute__((packed))

struct ertc_dlog {
	bool tx_enable;
	uint8_t rxbuff[32], txbuff[32];
	UART_HandleTypeDef uart_handle;
};

int ertc_dlog_send(struct ertc_dlog *logger, void *data, int size);
int ertc_dlog_update(struct ertc_dlog *logger);

#endif /* INC_ERTC_DATALOGGER_H_ */
