/*
 * switches.h
 *
 *  Created on: Jul 21, 2024
 *      Author: agamb
 */

#pragma once

#include <memory>

#include "pcf8574.h"

namespace HARDWARE::EXTRA
{

class Switches
{
private:
	std::shared_ptr<PCF8574> expander;

public:
	Switches(std::shared_ptr<PCF8574> expander);
	virtual ~Switches() = default;

	uint8_t getAll();
	uint8_t get(const uint8_t sw);

};

} /* namespace HARDWARE::EXTRA */
