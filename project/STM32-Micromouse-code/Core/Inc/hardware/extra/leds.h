/*
 * leds.h
 *
 *  Created on: Jul 21, 2024
 *      Author: agamb
 */

#pragma once

#include <memory>

#include "pcf8574.h"

namespace HARDWARE::EXTRA
{

class LEDS
{
private:
	std::shared_ptr<PCF8574> expander;

	inline void set(const uint8_t led, const uint8_t value);
	inline uint8_t get(const uint8_t led);

public:
	LEDS(std::shared_ptr<PCF8574> expander);
	virtual ~LEDS() = default;

	void turnOn(const uint8_t led);
	void turnOff(const uint8_t led);
	void toggle(const uint8_t led);

};

} /* namespace HARDWARE::EXTRA */
