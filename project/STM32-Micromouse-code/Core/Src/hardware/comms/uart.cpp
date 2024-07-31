/*
 * spi.cpp
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#include "hardware/comms/uart.h"

namespace HARDWARE::COMMS
{

UART::UART(UART_HandleTypeDef& huart) :
		huart(huart)
{}

int UART::send(uint8_t* buffer, const uint16_t length, const uint32_t timeout)
{
	if (nullptr == buffer)
	{
		return -1;
	}
	// TODO: Add mutex protection
	HAL_UART_Transmit(&huart, buffer, length, timeout);
	return 0;
}

int UART::receive(uint8_t* buffer, const uint16_t length, const uint32_t timeout)
{
	if (nullptr == buffer)
	{
		return -1;
	}
	// TODO: Add mutex protection
	HAL_UART_Receive(&huart, buffer, length, timeout);
	return 0;
}

} /* namespace HARDWARE::COMMS */
