/*
 * tof_vl53l1x.cpp
 *
 *  Created on: Jul 16, 2024
 *      Author: agamb
 */

#include "hardware/sensors/tof_vl53l1x.h"

#include "main.h"

#include <stdio.h>

namespace HARDWARE::SENSORS
{

TOF_VL53L1X::TOF_VL53L1X(uint16_t tof_address) : dev_address(tof_address)
{
}



void TOF_VL53L1X::init(TOF_VL53L1X::MODE mode)
{
	uint8_t byteData;
	uint16_t wordData;

	VL53L1_RdByte(dev_address, 0x010F, &byteData);
	printf("VL53L1X Model_ID: %X\n", byteData);
	VL53L1_RdByte(dev_address, 0x0110, &byteData);
	printf("VL53L1X Module_Type: %X\n", byteData);
	VL53L1_RdWord(dev_address, 0x010F, &wordData);
	printf("VL53L1X: %X\n", wordData);

	/* Wait for device booted */
	uint8_t booted = 0;
	while(!booted){
		VL53L1X_BootState(dev_address, &booted);
		HAL_Delay(2);
	}

	/* Sensor Initialization */
	VL53L1X_SensorInit(dev_address);

	config(mode);

	print();
}

void TOF_VL53L1X::config(TOF_VL53L1X::MODE mode)
{
	/* Modify the default configuration */
	switch (mode)
	{
	case TOF_VL53L1X::MODE::SHORT:
		// TODO: Differentiate between FAST and PRECISE configurations
		VL53L1X_SetDistanceMode(dev_address, SHORT);
		break;
	case TOF_VL53L1X::MODE::LONG:
		// TODO: Differentiate between FAST and PRECISE configurations
		VL53L1X_SetDistanceMode(dev_address, LONG);
		break;
	default:
		printf("Error, mode not valid.");
	}

	int status = VL53L1X_SetTimingBudgetInMs(dev_address, 15);
    printf("Status after set timing: %d\n", status);
//	VL53L1X_SetInterMeasurementInMs(dev_address, ???);

   printf("SetROI: %d\n", VL53L1X_SetROI(dev_address, 16, 4));
}

void TOF_VL53L1X::startRanging()
{
	VL53L1X_StartRanging(dev_address);   /* This function has to be called to enable the ranging */
}

void TOF_VL53L1X::printDistance()
{
	uint16_t Distance;
	uint16_t SignalRate;
	uint16_t AmbientRate;
	uint16_t SpadNum;
	uint8_t RangeStatus;
	uint8_t dataReady = 0;

	while (dataReady == 0){
		VL53L1X_CheckForDataReady(dev_address, &dataReady);
		HAL_Delay(2);
	}

	VL53L1X_GetRangeStatus(dev_address, &RangeStatus);
	VL53L1X_GetDistance(dev_address, &Distance);
	VL53L1X_GetSignalRate(dev_address, &SignalRate);
	VL53L1X_GetAmbientRate(dev_address, &AmbientRate);
	VL53L1X_GetSpadNb(dev_address, &SpadNum);
	VL53L1X_ClearInterrupt(dev_address); /* clear interrupt has to be called to enable next interrupt*/

	printf("%u, %u, %u, %u, %u\n", RangeStatus, Distance, SignalRate, AmbientRate, SpadNum);
}

uint8_t TOF_VL53L1X::isDataReady()
{
	uint8_t dataReady = 0;
	VL53L1X_CheckForDataReady(dev_address, &dataReady);
	return dataReady;
}

uint16_t TOF_VL53L1X::getDistance()
{
	uint8_t rangeStatus = 0;
	VL53L1X_GetRangeStatus(dev_address, &rangeStatus);
	if(rangeStatus == 0)
	{
		return 0;
	}

	uint16_t distance = 0;
	VL53L1X_GetDistance(dev_address, &distance);

	VL53L1X_ClearInterrupt(dev_address);

	return distance;
}

//VL53L1X_Result_t TOF_VL53L1X::getMeasureData()
//{
//	VL53L1X_Result_t result;
//	VL53L1X_GetResult(dev_address, &result);
//	return result;
//}


/* Print functions */
void TOF_VL53L1X::print()
{
	printVersion();
	printI2CAddress();
	printInterruptPolarity();
	printTimingBudgetInMs();
	printDistanceMode();
	printInterMeasurementInMs();
	printBootState();
	printSensorId();
	printOffset();
	printXtalk();
	printDistanceThreshold();
	printROI();
	printSignalThreshold();
	printSigmaThreshold();
}

void TOF_VL53L1X::printVersion()
{
	VL53L1X_Version_t version;
    VL53L1X_GetSWVersion(&version);
    printf("Version: %u.%u.%u.%lu\n", version.major, version.minor, version.build, version.revision);
}


void TOF_VL53L1X::printI2CAddress()
{
    printf("Address: 0x%02x\n", dev_address);
}


void TOF_VL53L1X::printInterruptPolarity()
{
    uint8_t intPol;
    VL53L1X_GetInterruptPolarity(dev_address, &intPol);
    printf("Polarity: %d\n", intPol);
}


void TOF_VL53L1X::printTimingBudgetInMs()
{
    uint16_t timingBudgetInMs;
    VL53L1X_GetTimingBudgetInMs(dev_address, &timingBudgetInMs);
    printf("Timing budget (ms): %d\n", timingBudgetInMs);
}


void TOF_VL53L1X::printDistanceMode()
{
    uint16_t distanceMode;
    VL53L1X_GetDistanceMode(dev_address, &distanceMode);
    printf("Distance mode: %d\n", distanceMode);
}


void TOF_VL53L1X::printInterMeasurementInMs()
{
    uint16_t IM;
    VL53L1X_GetInterMeasurementInMs(dev_address, &IM);
    printf("InterMeasurements(ms): %d\n", IM);
}


void TOF_VL53L1X::printBootState()
{
    uint8_t bootState;
    VL53L1X_BootState(dev_address, &bootState);
    printf("Boot state: %d\n", bootState);
}


void TOF_VL53L1X::printSensorId()
{
    uint16_t id;
    VL53L1X_GetSensorId(dev_address, &id);
    printf("Sensor ID: %d\n", id);
}


void TOF_VL53L1X::printOffset()
{
	int16_t offset;
    VL53L1X_GetOffset(dev_address, &offset);
    printf("Offset: %d\n", offset);
}


void TOF_VL53L1X::printXtalk()
{
    uint16_t xtalk;
    VL53L1X_GetXtalk(dev_address, &xtalk);
    printf("Xtalk: %d\n", xtalk);
}


void TOF_VL53L1X::printDistanceThreshold()
{
    uint16_t window, low, high;
    VL53L1X_GetDistanceThresholdWindow(dev_address, &window);
    VL53L1X_GetDistanceThresholdLow(dev_address, &low);
    VL53L1X_GetDistanceThresholdHigh(dev_address, &high);
    printf("Window: %d\n", window);
    printf("Low: %d\n", low);
    printf("High: %d\n", high);
}

void TOF_VL53L1X::printROI()
{
    uint16_t roi_x, roi_y;
    uint8_t roicenter;
    VL53L1X_GetROI_XY(dev_address, &roi_x, &roi_y);
    VL53L1X_GetROICenter(dev_address, &roicenter);
    printf("ROI_X: %u\n", roi_x);
    printf("ROI_Y: %u\n", roi_y);
    printf("ROICenter: %u\n", roicenter);
}


void TOF_VL53L1X::printSignalThreshold()
{
    uint16_t signal;
    VL53L1X_GetSignalThreshold(dev_address, &signal);
    printf("Signal threshold: %d\n", signal);
}


void TOF_VL53L1X::printSigmaThreshold()
{
    uint16_t sigma;
    VL53L1X_GetSigmaThreshold(dev_address, &sigma);
    printf("Signal threshold: %d\n", sigma);
}

}
