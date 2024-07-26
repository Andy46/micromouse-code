/*
 * spi.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

// STM32 HAL (Hardware Abstraction Layer) includes
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"

#include <stdint.h>

namespace HARDWARE::COMMS
{

class SPI final
{
private:
	SPI_HandleTypeDef& hspi;

public:
	SPI(SPI_HandleTypeDef& hspi);

	virtual ~SPI() = default;

	/**
	 * send
	 *
	 * Send an array of bytes through the SPI bus,
	 * Chip select of the device shall be set/clear before calling this function
	 *
	 * buffer  - Pointer to the data to send
	 * length  - Size of the data to send in bytes
	 * timeout - Timeout for the transaction
	 */
	int send(uint8_t* buffer, const uint16_t length, const uint32_t timeout = 100);

	/**
	 * receive
	 *
	 * Receive an array of bytes from the SPI bus
	 * Chip select of the device shall be set/clear before calling this function
	 *
	 * buffer  - Pointer where to store the data received
	 * length  - Size of the data to receive in bytes
	 * timeout - Timeout for the transaction
	 */
	int receive(uint8_t* buffer, const uint16_t length, const uint32_t timeout = 100);

};

} /* namespace HARDWARE::COMMS */
