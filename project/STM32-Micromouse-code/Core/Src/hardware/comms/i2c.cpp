/*
 * i2c.cpp
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#include "hardware/comms/i2c.h"

namespace HARDWARE::COMMS
{

I2C::I2C(I2C_HandleTypeDef& hi2c) :
		hi2c(hi2c)
{}

int I2C::send(const I2CAddress_t address, uint8_t* buffer, const uint16_t length, const uint32_t timeout)
{
	if (nullptr == buffer)
	{
		// TODO: Throw exception?
		return -1;
	}
	// TODO: Add mutex protection
	HAL_I2C_Master_Transmit(&hi2c, (address << 1), buffer, length, timeout);
	return 0;
}

int I2C::receive(const I2CAddress_t address, uint8_t* buffer, const uint16_t length, const uint32_t timeout)
{
	if (nullptr == buffer)
	{
		// TODO: Throw exception?
		return -1;
	}
	// TODO: Add mutex protection
	HAL_I2C_Master_Receive(&hi2c, (address << 1 | 1), buffer, length, timeout);
	return 0;
}

} /* namespace HARDWARE::COMMS */
