/*
 * platformfactory.h
 *
 *  Created on: Jul 20, 2024
 *      Author: agamb
 */

#include "hardware/platform_factory.h"

#include <memory>
#include <array>

// STM32
//#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"


// #include "dma.h" //TODO: Use it...


// Hardware
#include "hardware/platform.h"
#include "hardware/platform_defs.h"

//#include "hardware/comms/bluetooth.h"
#include "hardware/comms/spi.h"
#include "hardware/comms/i2c.h"
#include "hardware/comms/uart.h"

// Sensors
#include "hardware/sensors/tof_vl53l1x.h"
#include "hardware/sensors/bmi160.h"
//#include "hardware/sensors/encoder.h"

//#include "hardware/actuators/motor.h"
//#include "hardware/actuators/motor_driver.h"

#include "hardware/extra/pcf8574.h"
#include "hardware/extra/leds.h"
#include "hardware/extra/switches.h"
#include "hardware/extra/stm32_gpio.h"

//#include "hardware/platform_defs.h"

namespace HARDWARE
{

Platform PlatformFactory::CreatePlatform()
{

	/******************/
	/* Communications */
	/******************/

	// I2C
	auto i2c1 = std::make_shared<COMMS::I2C>(hi2c1);
	auto i2c2 = std::make_shared<COMMS::I2C>(hi2c2);

	// SPI
	auto spi1 = std::make_shared<COMMS::SPI>(hspi1);
	auto spi2 = std::make_shared<COMMS::SPI>(hspi2);

	// UART
	auto usart1 = std::make_shared<COMMS::UART>(huart1);

	// Bluetooth
	// TODO: Implement and instantiante BLUETOOTH interface (only interface (send and receive)

	/***********/
	/* Devices */
	/***********/

	// GPIO expanders
	auto tof_expander = std::make_shared<EXTRA::PCF8574>(i2c2, GPIOEXPANDER_TOF_ADDRESS);
	tof_expander->configAll(0x55); // 7-6 TOF Left       (Int-XShut), 5-4 TOF FrontLeft (Int-XShut),
                                   // 3-2 TOF FrontRight (Int-XShut), 1-0 TOF Right     (Int-XShut)
	tof_expander->writeAll(0x00);  // All OFF

	auto ledsw_expander = std::make_shared<EXTRA::PCF8574>(i2c2, GPIOEXPANDER_SWLED_ADDRESS);
	ledsw_expander->configAll(0xF0); // 7-4 SW, 3-0 LEDs
	ledsw_expander->writeAll(0x00);  // All OFF

	// LEDS & Switches
	auto leds     = std::make_shared<EXTRA::LEDS>(ledsw_expander);
	auto switches = std::make_shared<EXTRA::Switches>(ledsw_expander);

	/***********/
	/* Sensors */
	/***********/

	// TOFs
	auto tof_1 = std::make_shared<SENSORS::TOF_VL53L1X>(i2c2, tof_expander->getGPIO(GPIOEXPANDER_TOF_TOF0_XSHUT_PIN));
	auto tof_2 = std::make_shared<SENSORS::TOF_VL53L1X>(i2c2, tof_expander->getGPIO(GPIOEXPANDER_TOF_TOF1_XSHUT_PIN));
	auto tof_3 = std::make_shared<SENSORS::TOF_VL53L1X>(i2c2, tof_expander->getGPIO(GPIOEXPANDER_TOF_TOF2_XSHUT_PIN));
	auto tof_4 = std::make_shared<SENSORS::TOF_VL53L1X>(i2c2, tof_expander->getGPIO(GPIOEXPANDER_TOF_TOF3_XSHUT_PIN));

	// BMI160
	auto bmi_cs = std::make_shared<EXTRA::STM32_GPIO> (GPIOB, GPIO_PIN_12);
	auto bmi = std::make_shared<SENSORS::BMI160>(spi1, bmi_cs);

	// Create the platform and return it
	return Platform (tof_1, tof_2, tof_3, tof_4, bmi, leds, switches);
}

} /* namespace HARDWARE */
