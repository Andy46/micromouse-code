/*
 * spi.cpp
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#include "hardware/comms/spi.h"

namespace HARDWARE::COMMS
{

SPI::SPI(SPI_HandleTypeDef& hspi) :
		hspi(hspi)
{}

int SPI::send(uint8_t* buffer, const uint16_t length, const uint32_t timeout)
{
	if (nullptr == buffer)
	{
		return -1;
	}
	// TODO: Add mutex protection
	HAL_SPI_Transmit(&hspi, buffer, length, timeout);
	return 0;
}

int SPI::receive(uint8_t* buffer, const uint16_t length, const uint32_t timeout)
{
	if (nullptr == buffer)
	{
		return -1;
	}
	// TODO: Add mutex protection
	HAL_SPI_Receive(&hspi, buffer, length, timeout);
	return 0;
}

} /* namespace HARDWARE::COMMS */
