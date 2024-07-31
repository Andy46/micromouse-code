/*
 * pmw3360.h
 *
 *  Created on: Jul 27, 2024
 *      Author: agamb
 */

#pragma once

/* C/C++ libraries */
#include <stdint.h>
#include <memory>

/* Platform libraries */
#include "hardware/error.h"
#include "hardware/device.h"
#include "hardware/comms/spi.h"
#include "hardware/extra/gpio.h"

#ifndef PMW3360_VERSION
#define PMW3360_VERSION 4
#endif

namespace HARDWARE::SENSORS
{

/**
 * External device "PMW3360" - Mouse sensor (X/Y movement)
 *
 * Description: Provides X/Y movement
 *
 * Datasheet: https://d3s5r33r268y59.cloudfront.net/datasheets/9604/2017-05-07-18-19-11/PMS0058-PMW3360DM-T2QU-DS-R1.50-26092016._20161202173741.pdf
 **/
class PMW3360 : DEVICE
{
public:

	typedef struct
	{
		bool isMotion;        // True if a motion is detected.
		bool isOnSurface;     // True when a chip is on a surface
		int16_t dx;           // displacement on x directions. Unit: Count. (CPI * Count = Inch value)
		int16_t dy;           // displacement on y directions.
		uint8_t SQUAL;        // Surface Quality register, max 0x80. Number of features on the surface = SQUAL * 8
		uint8_t rawDataSum;   // It reports the upper byte of an 18‐bit counter which sums all 1296 raw data in the current frame; * Avg value = Raw_Data_Sum * 1024 / 1296
		uint8_t maxRawData;   // Max raw data value in current frame, max=127
		uint8_t minRawData;   // Min raw data value in current frame, max=127
		unsigned int shutter; // unit: clock cycles of the internal oscillator. shutter is adjusted to keep the average raw data values within normal operating ranges.
	} Result_t;

private:
	std::shared_ptr<COMMS::SPI> spi;
	std::shared_ptr<EXTRA::GPIO> cs;

	static const int FW_SIZE;
	static const uint8_t FW_DATA[];

	error_t resetSPI();
	error_t sendShutdown();
	error_t sendPowerUp();

	error_t uploadFW();
	bool checkSignature();

	void setCPI(unsigned int newCPI);
	unsigned int getCPI();

	error_t readReg(uint8_t reg_addr, uint8_t& reg_data);
	error_t writeReg(uint8_t reg_addr, uint8_t data);

public:
	PMW3360(std::shared_ptr<COMMS::SPI> spi, std::shared_ptr<EXTRA::GPIO> cs);  // set CPI to 800 by default.

	// Initialization
	error_t init();

	// Configuration
	error_t configure();

	// Read data from sensor
	error_t readData(Result_t& result);
	error_t readDataBurst(Result_t& result);

};

} /* namespace HARDWARE::SENSORS */
