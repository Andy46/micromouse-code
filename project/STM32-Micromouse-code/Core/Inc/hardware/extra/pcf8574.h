/*
 * gpio_expander.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

#include <memory>

#include "hardware/device.h"
#include "hardware/comms/i2c.h"
#include "hardware/extra/gpio.h"

namespace HARDWARE::EXTRA
{

/**
 * External device "PCF8474" - GPIO expander
 *
 * Description: Provides control over 8 GPIOs through an I2C interface
 *
 * Datasheet: https://www.ti.com/lit/ds/symlink/pcf8574.pdf
 **/
class PCF8574 final : public DEVICE, public std::enable_shared_from_this<PCF8574>
{
private:
	std::shared_ptr<COMMS::I2C> i2c;
	const COMMS::I2CAddress_t address;

	uint8_t configuration; // 0 = INPUT, 1 = OUTPUT
	uint8_t states;        // 0 = OFF  , 1 = ON

	void sendValues(uint8_t values);
	uint8_t readValues();

	/**
	 *  Interface to control an individual pin of the GPIO expander
	 **/
	class PCF8574_GPIO final : public GPIO
	{
	private:
		std::shared_ptr<PCF8574> expander;
		const uint8_t            pin;

	public:
		PCF8574_GPIO(std::shared_ptr<PCF8574> expander, const uint8_t pin);
		PCF8574_GPIO(PCF8574_GPIO&) = delete;
		PCF8574_GPIO(PCF8574_GPIO&&) = default;
		virtual ~PCF8574_GPIO() = default;

		GPIO::State read() override;

		void set() override;
		void clear() override;
	};

public:
	PCF8574(std::shared_ptr<COMMS::I2C> i2c, const COMMS::I2CAddress_t address);
	virtual ~PCF8574() = default;

	// Configuration
	void configure(const uint8_t pinModes);

	// Read/write all pins
	void writeAll(const uint8_t values);
	uint8_t readAll();

	// Read/write individual pins
	void writePin(const uint8_t pin, const bool values);
	bool readPin(const uint8_t pin);

	/**
	 * Returns a GPIO interface to read, set and clear a single pin of the
	 * GPIO expander
	 */
	std::unique_ptr<GPIO> getGPIO(uint8_t pin);
};

} /* namespace HARDWARE::EXTRA */
