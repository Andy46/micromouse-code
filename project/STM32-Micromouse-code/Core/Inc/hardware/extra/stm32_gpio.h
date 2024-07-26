/*
 * stm32_gpio.h
 *
 *  Created on: Jul 26, 2024
 *      Author: agamb
 */

// STM32 includes
#include "stm32f1xx_hal.h"

// Application includes
#include "hardware/extra/gpio.h"

namespace HARDWARE::EXTRA
{

class STM32_GPIO final : public GPIO
{
private:
	/** STM32 GPIO Port */
	GPIO_TypeDef*  port;

	/** STM32 GPIO Pin */
	const uint16_t pin;

public:

	/**
	 *  Constructor
	 *  port -> STM32 GPIO Port
	 *  pin  -> STM32 GPIO Pin
	 **/
	STM32_GPIO(GPIO_TypeDef* port, const uint16_t pin);
	virtual ~STM32_GPIO() = default;

	/**
	 * Returns the state of the GPIO
	 */
	GPIO::State read() override;

	/**
	 * Sets the GPIO to LOW/OFF/0
	 */
	void clear() override;

	/**
	 * Sets the GPIO to HIGH/ON/1
	 */
	void set() override;
};

} /* namespace HARDWARE::EXTRA */

