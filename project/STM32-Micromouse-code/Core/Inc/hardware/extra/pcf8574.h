/*
 * gpio_expander.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

#include <memory>

#include "hardware/comms/i2c.h"

namespace HARDWARE::EXTRA
{

class PCF8574
{
private:
	std::shared_ptr<COMMS::I2C> i2c;
	const COMMS::I2CAddress_t address;

	uint8_t configuration; // 0 = INPUT, 1 = OUTPUT
	uint8_t states;        // 0 = OFF  , 1 = ON

	void sendValues(uint8_t values);
	uint8_t readValues();

public:
	PCF8574(std::shared_ptr<COMMS::I2C> i2c, const COMMS::I2CAddress_t address);
	virtual ~PCF8574() = default;

	// Configuration
	void configAll(const uint8_t pinModes);

	// Read/write all pins
	void writeAll(const uint8_t values);
	uint8_t readAll();

	// Read/write individual pins
	void writePin(const uint8_t pin, const uint8_t values);
	uint8_t readPin(const uint8_t pin);

};

} /* namespace HARDWARE::EXTRA */
