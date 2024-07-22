/*
 * gpio.h
 *
 *  Created on: Jul 22, 2024
 *      Author: agamb
 */

#pragma once

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

namespace HARDWARE::EXTRA
{

class GPIO
{
private:
	GPIO_TypeDef* port;
	const uint16_t      pin;

public:
	GPIO(GPIO_TypeDef* port, const uint16_t pin);
	virtual ~GPIO() = default;

	uint8_t read();

	void set();
	void clear();
};

} /* namespace HARDWARE::EXTRA */
