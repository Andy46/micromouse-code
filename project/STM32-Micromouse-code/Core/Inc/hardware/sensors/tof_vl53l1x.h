/*
 * tof_vl53l1x.h
 *
 *  Created on: Jul 16, 2024
 *      Author: agamb
 */

#pragma once

#include "hardware/extra/pcf8574.h"

namespace HARDWARE::SENSORS
{

class TOF_VL53L1X
{
private:

//	static VL53L1_RangingMeasurementData_t RangingData;

	std::shared_ptr<COMMS::I2C>  i2c;
	std::unique_ptr<EXTRA::GPIO> xshut_pin;
	uint16_t                     address;

	// VL53L1X_Platform calls
	int _I2CWrite(uint8_t *pdata, uint32_t count);
	int _I2CRead(uint8_t *pdata, uint32_t count);
	int8_t I2C_WriteMulti(uint16_t index, uint8_t *pdata, uint32_t count);
	int8_t I2C_ReadMulti(uint16_t index, uint8_t *pdata, uint32_t count);
	int8_t I2C_WrByte(uint16_t index, uint8_t data);
	int8_t I2C_WrWord(uint16_t index, uint16_t data);
	int8_t I2C_RdByte(uint16_t index, uint8_t *data);
	int8_t I2C_RdWord(uint16_t index, uint16_t *data);

	int8_t SensorInit();

	int8_t ClearInterrupt();
	int8_t GetInterruptPolarity(uint8_t *pIntPol);
	int8_t StopRanging();
	int8_t CheckForDataReady(uint8_t *isDataReady);
	int8_t BootState(uint8_t *state);

	int8_t SetI2CAddress(uint8_t new_address);
	int8_t SetTimingBudgetInMs(uint16_t TimingBudgetInMs);
	int8_t GetTimingBudgetInMs(uint16_t *pTimingBudgetInMs);
	int8_t GetDistanceMode(uint16_t *pDistanceMode);
	int8_t SetDistanceMode(uint16_t DistanceMode);
	int8_t SetDistanceThreshold(uint16_t ThreshLow,
				      uint16_t ThreshHigh, uint8_t Window,
				      uint8_t IntOnNoTarget);
	int8_t SetROI(uint16_t X, uint16_t Y);
	int8_t SetROICenter(uint8_t ROICenter);

	int8_t GetSensorId(uint16_t *id);
	int8_t GetDistance(uint16_t *distance);
	int8_t GetSignalRate(uint16_t *signalRate);
	int8_t GetSpadNb(uint16_t *spNb);
	int8_t GetAmbientRate(uint16_t *ambRate);
	int8_t GetRangeStatus(uint8_t *rangeStatus);
	int8_t GetOffset(int16_t *offset);
	int8_t GetROI_XY(uint16_t *ROI_X, uint16_t *ROI_Y);

	// Printing info
//	void printVersion();
//	void printI2CAddress();
//	void printInterruptPolarity();
//	void printTimingBudgetInMs();
//	void printDistanceMode();
//	void printInterMeasurementInMs();
//	void printBootState();
//	void printSensorId();
//	void printOffset();
//	void printXtalk();
//	void printDistanceThreshold();
//	void printROI();
//	void printSignalThreshold();
//	void printSigmaThreshold();

public:
	TOF_VL53L1X(std::shared_ptr<COMMS::I2C> i2c,
				std::unique_ptr<EXTRA::GPIO> xshut_pin);
	virtual ~TOF_VL53L1X() = default;

	enum MODE {
		SHORT = 1,
		LONG,
	};

	// VL53L1X_API calls
    typedef struct {
        uint8_t status;		/*!< ResultStatus */
        uint16_t distance;	/*!< ResultDistance */
        uint16_t ambient;	/*!< ResultAmbient */
        uint16_t sigPerSPAD;/*!< ResultSignalPerSPAD */
        uint16_t numSPADs;	/*!< ResultNumSPADs */
    } Result_t;

	// Pin functions
	void turnOn();
	void turnOff();

	void init(MODE mode = MODE::SHORT);
	void config(MODE mode);

	void setI2CAddress(uint16_t newAddress);

	void printDistance();

	int8_t StartRanging();
    int8_t GetResult(Result_t *pResult);

    void getData(Result_t &result);

//	void print();

};

} /* namespace HARDWARE::SENSORS */
