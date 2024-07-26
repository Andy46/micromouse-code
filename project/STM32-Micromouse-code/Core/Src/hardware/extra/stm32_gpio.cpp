/*
 * stm32_gpio.cpp
 *
 *  Created on: Jul 26, 2024
 *      Author: agamb
 */

#include "hardware/extra/stm32_gpio.h"

#include "stm32f1xx_hal_gpio.h"

namespace HARDWARE::EXTRA
{

STM32_GPIO::STM32_GPIO(GPIO_TypeDef* port, const uint16_t pin) :
		GPIO(), port(port), pin(pin)
{}

GPIO::State STM32_GPIO::read()
{
	return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? GPIO::State::HIGH : GPIO::State::LOW;
}

void STM32_GPIO::clear()
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void STM32_GPIO::set()
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

} /* namespace HARDWARE::EXTRA */
