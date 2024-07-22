/*
 * gpio_exapander.cpp
 *
 *  Created on: Jul 21, 2024
 *      Author: agamb
 */
#include "hardware/extra/pcf8574.h"

#include <stdio.h>

namespace
{
constexpr int PCF8574_PIN_COUNT = 8;
constexpr HARDWARE::COMMS::I2CAddress_t DEFAULT_PCF8574_ADDRESS = 0X20;

constexpr uint8_t ALL_LOW  = 0x00;
constexpr uint8_t ALL_HIGH = 0xFF;

inline void setBitLow(uint8_t& reg, uint8_t pin)
{
	const uint8_t mask = ~(1 << pin);
	reg &= mask;
}

inline void setBitHigh(uint8_t& reg, uint8_t pin)
{
	const uint8_t mask = (1 << pin);
	reg |= mask;
}

inline void setBit(uint8_t& reg, uint8_t bit, uint8_t value)
{
	if (value)
	{
		setBitHigh(reg, bit);
	}
	else
	{
		setBitLow(reg, bit);
	}
}

inline uint8_t getBit(uint8_t reg, uint8_t bit)
{
	const uint8_t mask = (1 << bit);
	uint8_t value = (reg & mask) ? 1 : 0;
	return value;
}

} /* namespace */

namespace HARDWARE::EXTRA
{

PCF8574::PCF8574(std::shared_ptr<COMMS::I2C> i2c, const COMMS::I2CAddress_t address) :
	i2c(i2c), address(address), configuration(ALL_HIGH), states(ALL_HIGH)
{

}

void PCF8574::sendValues(uint8_t values)
{
	i2c->send(address, &values, sizeof(values));
}

uint8_t PCF8574::readValues()
{
	uint8_t values;
	i2c->receive(address, &values, sizeof(values));
	return values;
}

// Configuration
void PCF8574::configAll(const uint8_t pinModes)
{
	configuration = pinModes;
}

// Read/write all pins
void PCF8574::writeAll(const uint8_t values)
{
	// TODO: Protect with mutex
	const uint8_t inputMask = ~configuration;
	states = values | inputMask;
	sendValues(states);
}

uint8_t PCF8574::readAll()
{
	// TODO: Protect with mutex
	states = readValues();
	return states;
}

// Read/write individual pins
void PCF8574::writePin(const uint8_t pin, const uint8_t value)
{
	const uint8_t inputMask = ~configuration;
	// TODO: Protect with mutex
	if (pin < PCF8574_PIN_COUNT && getBit(configuration, pin))
	{
	    setBit(states, pin, value);
		states |= inputMask;
		sendValues(states);
	}
}

uint8_t PCF8574::readPin(const uint8_t pin)
{
	// TODO: Protect with mutex
	if (pin < PCF8574_PIN_COUNT)
	{
		states = readValues();
		return getBit(states, pin);
	}
	return 0;
}

std::unique_ptr<PCF8574::GPIO> PCF8574::getGPIO(uint8_t pin)
{
	return std::make_unique<PCF8574::GPIO>(shared_from_this(), pin);
}

/* GPIO from expander */
PCF8574::GPIO::GPIO(std::shared_ptr<PCF8574> expander, const uint8_t pin) :
		expander(expander), pin(pin)
{

}

void PCF8574::GPIO::set()
{
	expander->writePin(pin, 1);
}

void PCF8574::GPIO::clear()
{
	expander->writePin(pin, 0);
}

} /* namespace HARDWARE::EXTRA */
