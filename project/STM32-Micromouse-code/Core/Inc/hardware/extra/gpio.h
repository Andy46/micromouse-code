/*
 * gpio.h
 *
 *  Created on: Jul 22, 2024
 *      Author: agamb
 */

#pragma once

namespace HARDWARE::EXTRA
{

	/**
	 * GPIO Interface class
	 */
	class GPIO
	{
	private:

	public:
		GPIO() = default;
		virtual ~GPIO() = default;

		enum State
		{
			LOW = 0,
			HIGH
		};

		/**
		 * Returns the state of the GPIO
		 */
		virtual State read() = 0;

		/**
		 * Sets the GPIO to LOW/OFF/0
		 */
		virtual void clear() = 0;

		/**
		 * Sets the GPIO to HIGH/ON/1
		 */
		virtual void set() = 0;
	};

} /* namespace HARDWARE::EXTRA */
