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

// #include "actuators/motor_driver.h"
// #include "actuators/motor.h"

// #include "sensors/tof_vl53l1x.h"
// #include "sensors/encoder.h"
// #include "sensors/bmi160.h"

constexpr int TOF_COUNT = 4;

namespace HARDWARE
{

class Platform
{
private:
	// SENSORS::TOF_VL53L1X tof_left;
	// SENSORS::TOF_VL53L1X tof_frontleft;
	// SENSORS::TOF_VL53L1X tof_frontright;
	// SENSORS::TOF_VL53L1X tof_right;

	// ACTUATORS::DRV8833 motor_driver;
	// ACTUATORS::N20 motor_A;
	// ACTUATORS::N20 motor_B;

	// SENSORS::Encoder encoder_A;
	// SENSORS::Encoder encoder_B;
public:
	Platform();
	virtual ~Platform() = default;

	void init();

};

} /* namespace HARDWARE */
