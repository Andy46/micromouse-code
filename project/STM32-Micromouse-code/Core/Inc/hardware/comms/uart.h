/*
 * usart.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

// STM32 HAL (Hardware Abstraction Layer) includes
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"

#include <stdint.h>

namespace HARDWARE::COMMS
{

class UART final
{
private:
	UART_HandleTypeDef& huart;

public:
	UART(UART_HandleTypeDef& huart);

	virtual ~UART() = default;

	/**
	 * send
	 *
	 * Send an array of bytes through the UART(TX) line,
	 *
	 * buffer  - Pointer to the data to send
	 * length  - Size of the data to send in bytes
	 * timeout - Timeout for the transaction
	 */
	int send(uint8_t* buffer, const uint16_t length, const uint32_t timeout);

	/**
	 * receive
	 *
	 * Receive an array of bytes from the UART(RX) line
	 *
	 * buffer  - Pointer where to store the data received
	 * length  - Size of the data to receive in bytes
	 * timeout - Timeout for the transaction
	 */
	int receive(uint8_t* buffer, const uint16_t length, const uint32_t timeout);
};

} /* namespace HARDWARE::COMMS */
