/*
 * tof_vl53l1x.cpp
 *
 *  Created on: Jul 16, 2024
 *      Author: agamb
 */

#include "hardware/sensors/tof_vl53l1x.h"

#include "main.h"

#include <stdio.h>

namespace
{

constexpr uint16_t DEFAULT_TOF_ADDRESS = 0x29;
}

namespace HARDWARE::SENSORS
{
TOF_VL53L1X::TOF_VL53L1X(std::shared_ptr<COMMS::I2C> i2c,
						std::unique_ptr<EXTRA::GPIO> xshut_pin) :
		i2c(i2c), xshut_pin(std::move(xshut_pin)), address (DEFAULT_TOF_ADDRESS)
{

}

void TOF_VL53L1X::turnOn()
{
	xshut_pin->set();
}

void TOF_VL53L1X::turnOff()
{
	xshut_pin->clear();
}

void TOF_VL53L1X::init(TOF_VL53L1X::MODE mode)
{
	/* Wait for device booted */
	uint8_t booted = 0;
	while(!booted){
		BootState(&booted);
		HAL_Delay(2);
	}

	/* Sensor initialization and configuration */
	SensorInit();
	config(mode);
}

void TOF_VL53L1X::config(TOF_VL53L1X::MODE mode)
{
	/* Modify the default configuration */
	switch (mode)
	{
	case TOF_VL53L1X::MODE::SHORT:
		SetDistanceMode(SHORT);
		break;
	case TOF_VL53L1X::MODE::LONG:
		SetDistanceMode(LONG);
		break;
	default:
		printf("Error, mode not valid.");
        return;
	}

	SetTimingBudgetInMs(15);
    SetROI(16, 4);
}

void TOF_VL53L1X::printDistance()
{
    Result_t result;
	uint8_t dataReady = 0;

	while (dataReady == 0){
		CheckForDataReady(&dataReady);
		HAL_Delay(2);
	}

    GetResult(&result);
	ClearInterrupt(); /* clear interrupt has to be called to enable next interrupt*/

	printf("%u, %u, %u, %u, %u\n", result.status, result.distance, result.sigPerSPAD, result.ambient, result.numSPADs);
}

}
