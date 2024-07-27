/*
 * tof_vl53l1x.cpp
 *
 *  Created on: Jul 16, 2024
 *      Author: agamb
 */

#include "hardware/sensors/tof_vl53l1x.h"

#include "main.h"

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

error_t TOF_VL53L1X::init()
{
	/* Wait for device booted */
	uint8_t booted = 0;
	while(!booted){
		BootState(&booted);
		HAL_Delay(2);
	}

	/* Sensor initialization and configuration */
	int8_t status = SensorInit();
    if (status != 0)
    {
    	DEVICE::setError();
    	return error_t::TOF_ERROR;
    }
	DEVICE::setInitialized();
	return error_t::OK;
}

error_t TOF_VL53L1X::configure(TOF_VL53L1X::MODE mode)
{
	int8_t status = 0;

	/* Modify the default configuration */
	switch (mode)
	{
	case TOF_VL53L1X::MODE::SHORT:
		status = SetDistanceMode(SHORT);
		break;
	case TOF_VL53L1X::MODE::LONG:
		status = SetDistanceMode(LONG);
		break;
	default:
        return error_t::TOF_ERROR;
	}
    if (status != 0)
    {
    	DEVICE::setError();
    	return error_t::TOF_ERROR;
    }

	status = SetTimingBudgetInMs(15);
    if (status != 0)
    {
    	DEVICE::setError();
    	return error_t::TOF_ERROR;
    }

	status = SetROI(16, 4);
    if (status != 0)
    {
    	DEVICE::setError();
    	return error_t::TOF_ERROR;
    }

	DEVICE::setReady();
	return error_t::OK;
}

error_t TOF_VL53L1X::start()
{
	int8_t status = StartRanging();
    if (status != 0)
    {
    	DEVICE::setError();
    	return error_t::TOF_ERROR;
    }
	return error_t::OK;
}

bool TOF_VL53L1X::isDataReady()
{
	uint8_t dataReady = 0;

	int8_t status = CheckForDataReady(&dataReady);
	if (status != 0)
	{
		DEVICE::setError();
	}

	return (dataReady != 0);
}

error_t TOF_VL53L1X::getDataAndRestart(Result_t &result)
{
	int8_t status = 0;

    status = GetResult(&result);
    if (status != 0)
    {
    	DEVICE::setError();
    	return error_t::TOF_ERROR;
    }

    status = ClearInterrupt(); /* clear interrupt has to be called to enable next interrupt*/
    if (status != 0)
    {
    	DEVICE::setError();
    	return error_t::TOF_ERROR;
    }

	return error_t::OK;
}

}
