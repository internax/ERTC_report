/*
 * ertc-datalogger.c
 *
 *  Created on: Apr 8, 2021
 */
#include "ertc-datalogger.h"
#include "cobs.h"

#ifdef STM32F767xx
#include "stm32f7xx_hal_uart.h"
#endif

int ertc_dlog_send(struct ertc_dlog *logger, void *data, int size)
{
	if (logger->tx_enable) {
		cobsEncode((uint8_t *)data, size, logger->txbuff);

		/* Add null terminator */
		logger->txbuff[size + 1] = 0x00;

		/*	Send data packet */
		return HAL_UART_Transmit(&logger->uart_handle, (uint8_t *)logger->txbuff, size + 2, HAL_TIMEOUT);
	}

	return 0;
}

int ertc_dlog_update(struct ertc_dlog *logger)
{
	if (HAL_UART_Receive(&logger->uart_handle, (uint8_t *)logger->rxbuff, 1, HAL_TIMEOUT) == HAL_OK) {
		switch (logger->rxbuff[0]) {
			case TX_START_CMD:
				logger->tx_enable = true;
				break;
			case TX_STOP_CMD:
				logger->tx_enable = false;
				break;
			default:
				logger->tx_enable = false;
		}
	}
	return 0;
}
