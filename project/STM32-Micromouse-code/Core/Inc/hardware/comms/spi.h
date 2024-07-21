/*
 * spi.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

// STM32 HAL (Hardware Abstraction Layer)
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
//#include "stm32f1xx_hal_dma.h"

namespace HARDWARE::COMMS
{

class SPI
{
private:
	SPI_HandleTypeDef& hspi;
//	DMA_HandleTypeDef& hdma;


public:
	SPI(SPI_HandleTypeDef& hspi);
//		SPI(SPI_HandleTypeDef& hspi, DMA_HandleTypeDef& hdma);

	virtual ~SPI() = default;

	int send(uint8_t* buffer, const uint16_t length, const uint32_t timeout);
	int receive(uint8_t* buffer, const uint16_t length, const uint32_t timeout);
};

} /* namespace HARDWARE::COMMS */
