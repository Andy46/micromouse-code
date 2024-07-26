/*
 * i2c.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

// STM32 HAL (Hardware Abstraction Layer) includes
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"

#include <stdint.h>

namespace HARDWARE::COMMS
{

typedef uint16_t I2CAddress_t;

class I2C final
{
private:
	I2C_HandleTypeDef& hi2c;

public:
	I2C(I2C_HandleTypeDef& hi2c);

	virtual ~I2C() = default;

	/**
	 * send
	 *
	 * Send an array of bytes to a device through the I2C bus
	 *
	 * address - Address of device
	 * buffer  - Pointer to the data to send
	 * length  - Size of the data to send in bytes
	 * timeout - Timeout for the transaction
	 */
	int send(const I2CAddress_t address, uint8_t* buffer, const uint16_t length, const uint32_t timeout = 100);

	/**
	 * receive
	 *
	 * Receive an array of bytes from a device through the I2C bus
	 *
	 * address - Address of device
	 * buffer  - Pointer where to store the data received
	 * length  - Size of the data to receive in bytes
	 * timeout - Timeout for the transaction
	 */
	int receive(const I2CAddress_t address, uint8_t* buffer, const uint16_t length, const uint32_t timeout = 100);
};

} /* namespace HARDWARE::COMMS */
