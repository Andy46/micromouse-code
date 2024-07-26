/*
 * pcf8574_gpio.cpp
 *
 *  Created on: Jul 26, 2024
 *      Author: agamb
 */

#include "hardware/extra/pcf8574.h"

namespace HARDWARE::EXTRA
{

/* GPIO from expander */
PCF8574::PCF8574_GPIO::PCF8574_GPIO(std::shared_ptr<PCF8574> expander, const uint8_t pin) :
		expander(expander), pin(pin)
{

}

GPIO::State PCF8574::PCF8574_GPIO::read()
{
	return (expander->readPin(pin) != 0x0) ? GPIO::State::HIGH : GPIO::State::LOW;
}

void PCF8574::PCF8574_GPIO::clear()
{
	expander->writePin(pin, false);
}

void PCF8574::PCF8574_GPIO::set()
{
	expander->writePin(pin, true);
}

} /* namespace HARDWARE::EXTRA */
