/*
 * usart.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

// STM32 HAL (Hardware Abstraction Layer)
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
//#include "stm32f1xx_hal_dma.h"

namespace HARDWARE::COMMS
{

class UART
{
private:
	UART_HandleTypeDef& huart;
//	DMA_HandleTypeDef& hdma;

public:
	UART(UART_HandleTypeDef& huart);
	//	UART(UART_HandleTypeDef& huart, DMA_HandleTypeDef& hdma);

	virtual ~UART() = default;

	int send(uint8_t* buffer, const uint16_t length, const uint32_t timeout);
	int receive(uint8_t* buffer, const uint16_t length, const uint32_t timeout);
};

} /* namespace HARDWARE::COMMS */
