/*
 * switches.cpp
 *
 *  Created on: Jul 21, 2024
 *      Author: agamb
 */

#include "hardware/extra/switches.h"

namespace {

const int SW_COUNT           = 4;
const int SW_MASK            = 0x0F;
const int SW_EXPANDER_OFFSET = 0;

} /* namespace */


namespace HARDWARE::EXTRA
{

Switches::Switches(std::shared_ptr<PCF8574> expander) : expander(expander)
{

}

uint8_t Switches::get(const uint8_t sw)
{
	if (sw < SW_COUNT)
	{
		return expander->readPin(sw + SW_EXPANDER_OFFSET);
	}
	return 0;
}

uint8_t Switches::getAll()
{
	uint8_t values = expander->readAll();
	return values & SW_MASK;
}

} /* namespace HARDWARE::EXTRA */
