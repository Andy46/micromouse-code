/*
 * leds.cpp
 *
 *  Created on: Jul 21, 2024
 *      Author: agamb
 */

#include "hardware/extra/leds.h"

namespace {

const int LED_COUNT           = 4;
const int LED_EXPANDER_OFFSET = 4;

} /* namespace */

namespace HARDWARE::EXTRA
{

LEDS::LEDS(std::shared_ptr<PCF8574> expander) : expander(expander)
{

}

void LEDS::set(const uint8_t led, const bool value)
{
	if (led < LED_COUNT)
	{
		expander->writePin(led + LED_EXPANDER_OFFSET, value);
	}
}

bool LEDS::get(const uint8_t led)
{
	if (led < LED_COUNT)
	{
		return expander->readPin(led + LED_EXPANDER_OFFSET);
	}
	return 0;
}

void LEDS::turnOn(const uint8_t led)
{
	set(led, 1);
}

void LEDS::turnOff(const uint8_t led)
{
	set(led, 0);
}

void LEDS::toggle(const uint8_t led)
{
	set(led, !get(led));
}

} /* namespace HARDWARE::EXTRA */
