/*
 * tof_vl53l1x.h
 *
 *  Created on: Jul 16, 2024
 *      Author: agamb
 */

#pragma once

#include <VL53L1X_api.h>
#include <VL53L1X_calibration.h>

#define DEFAULT_TOF_ADDRESS 0x52

namespace HARDWARE::SENSORS
{

class TOF_VL53L1X
{
private:

//	static VL53L1_RangingMeasurementData_t RangingData;
	uint16_t dev_address;

	void print();
	void printVersion();
	void printI2CAddress();
	void printInterruptPolarity();
	void printTimingBudgetInMs();
	void printDistanceMode();
	void printInterMeasurementInMs();
	void printBootState();
	void printSensorId();
	void printOffset();
	void printXtalk();
	void printDistanceThreshold();
	void printROI();
	void printSignalThreshold();
	void printSigmaThreshold();

public:
	TOF_VL53L1X(uint16_t tof_address = DEFAULT_TOF_ADDRESS);
	virtual ~TOF_VL53L1X() = default;

	enum MODE {
		SHORT = 1,
		LONG,
	};

	void turnOn();
	void turnOff();

	void init(MODE mode = MODE::SHORT);
	void config(MODE mode);

	void startRanging();
	void printDistance();

	uint8_t isDataReady();
	uint16_t getDistance();

//	void getMeasureData();

};

} /* namespace HARDWARE::SENSORS */
