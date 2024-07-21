/*
 * i2c.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

// STM32 HAL (Hardware Abstraction Layer)
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
//#include "stm32f1xx_hal_dma.h"

namespace HARDWARE::COMMS
{

typedef uint16_t I2CAddress_t;

class I2C
{
private:
	I2C_HandleTypeDef& hi2c;
//	DMA_HandleTypeDef& hdma;

public:
	I2C(I2C_HandleTypeDef& hi2c);
	//	I2C(I2C_HandleTypeDef& hi2c, DMA_HandleTypeDef& hdma);

	virtual ~I2C() = default;

	int send(const I2CAddress_t address, uint8_t* buffer, const uint16_t length, const uint32_t timeout = 100);
	int receive(const I2CAddress_t address, uint8_t* buffer, const uint16_t length, const uint32_t timeout = 100);
};

} /* namespace HARDWARE::COMMS */
