/*
 * gpio.cpp
 *
 *  Created on: Jul 22, 2024
 *      Author: agamb
 */

#include "hardware/extra/gpio.h"

namespace HARDWARE::EXTRA
{

GPIO::GPIO(GPIO_TypeDef* port, const uint16_t pin) :
		port(port), pin(pin)
{

}

uint8_t GPIO::read()
{
	return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1 : 0;
}

void GPIO::set()
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

void GPIO::clear()
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

} /* namespace HARDWARE::EXTRA */
