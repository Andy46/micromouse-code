/*
 * platform.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

#include "main.h"

#include <stddef.h>
#include <stdint.h>

// #include "actuators/motor.h"
// #include "actuators/motor_driver.h"

 #include "sensors/bmi160.h"
// #include "sensors/encoder.h"
 #include "sensors/tof_vl53l1x.h"

 #include "extra/leds.h"
 #include "extra/switches.h"

constexpr int TOF_COUNT = 4;
constexpr int LED_COUNT = 4;
constexpr int SW_COUNT  = 4;

namespace HARDWARE
{

class Platform
{
private:
	 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_left;
	 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_frontleft;
	 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_frontright;
	 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_right;

	 std::shared_ptr<SENSORS::BMI160> bmi160;

	 std::shared_ptr<EXTRA::LEDS> leds;
	 std::shared_ptr<EXTRA::Switches> switches;

	// ACTUATORS::DRV8833 motor_driver;
	// ACTUATORS::N20 motor_A;
	// ACTUATORS::N20 motor_B;

	// SENSORS::Encoder encoder_A;
	// SENSORS::Encoder encoder_B;
public:
	Platform(std::shared_ptr<SENSORS::TOF_VL53L1X> tof_left,
			 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_frontleft,
			 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_frontright,
			 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_right,
			 std::shared_ptr<SENSORS::BMI160> bmi160,
			 std::shared_ptr<EXTRA::LEDS> leds,
			 std::shared_ptr<EXTRA::Switches> switches);
	virtual ~Platform() = default;

	void init();

	void run_test();

};

} /* namespace HARDWARE */
