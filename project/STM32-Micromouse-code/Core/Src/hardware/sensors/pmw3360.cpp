/*
 * pmw3360.cpp
 *
 *  Created on: Jul 27, 2024
 *      Author: agamb
 */

#include "hardware/sensors/pmw3360.h"

#include "hardware/utils/time.h"

#include <string.h>
#include <stdint.h>

namespace
{

// Registers
#define REG_Product_ID  0x00
#define REG_Revision_ID 0x01
#define REG_Motion  0x02
#define REG_Delta_X_L 0x03
#define REG_Delta_X_H 0x04
#define REG_Delta_Y_L 0x05
#define REG_Delta_Y_H 0x06
#define REG_SQUAL 0x07
#define REG_Raw_Data_Sum  0x08
#define REG_Maximum_Raw_data  0x09
#define REG_Minimum_Raw_data  0x0A
#define REG_Shutter_Lower 0x0B
#define REG_Shutter_Upper 0x0C
#define REG_Control 0x0D
#define REG_Config1 0x0F
#define REG_Config2 0x10
#define REG_Angle_Tune  0x11
#define REG_Frame_Capture 0x12
#define REG_SROM_Enable 0x13
#define REG_Run_Downshift 0x14
#define REG_Rest1_Rate_Lower  0x15
#define REG_Rest1_Rate_Upper  0x16
#define REG_Rest1_Downshift 0x17
#define REG_Rest2_Rate_Lower  0x18
#define REG_Rest2_Rate_Upper  0x19
#define REG_Rest2_Downshift 0x1A
#define REG_Rest3_Rate_Lower  0x1B
#define REG_Rest3_Rate_Upper  0x1C
#define REG_Observation 0x24
#define REG_Data_Out_Lower  0x25
#define REG_Data_Out_Upper  0x26
#define REG_Raw_Data_Dump 0x29
#define REG_SROM_ID 0x2A
#define REG_Min_SQ_Run  0x2B
#define REG_Raw_Data_Threshold  0x2C
#define REG_Config5 0x2F
#define REG_Power_Up_Reset  0x3A
#define REG_Shutdown  0x3B
#define REG_Inverse_Product_ID  0x3F
#define REG_LiftCutoff_Tune3  0x41
#define REG_Angle_Snap  0x42
#define REG_LiftCutoff_Tune1  0x4A
#define REG_Motion_Burst  0x50
#define REG_LiftCutoff_Tune_Timeout 0x58
#define REG_LiftCutoff_Tune_Min_Length  0x5A
#define REG_SROM_Load_Burst 0x62
#define REG_Lift_Config 0x63
#define REG_Raw_Data_Burst  0x64
#define REG_LiftCutoff_Tune2  0x65

} /* namespace */

namespace HARDWARE::SENSORS
{

PMW3360::PMW3360(std::shared_ptr<COMMS::SPI> spi, std::shared_ptr<EXTRA::GPIO> cs) :
		spi(spi), cs(cs)
{
	// Set SPI chip select line to unselect
	cs->set();
}

error_t PMW3360::init()
{
	cs->set();
	HAL_Delay(3);

	// Shutdown sensor
	sendShutdown();
	HAL_Delay(300);

	// Reset
	resetSPI();

	sendPowerUp();
	HAL_Delay(50);

	Result_t result;
	readData(result);

	uploadFW();

	error_t status = error_t::OK;

	if (checkSignature())
	{
		status = error_t::OK;
		DEVICE::setInitialized();
	}
	else
	{
		status = error_t::PMW3360_ERROR;
		DEVICE::setError();
	}

	return status;
}

error_t PMW3360::configure()
{
	// TODO: Implement proper configuration
//	setCPI(800);

	writeReg(0x10, 0x00); // Rest mode & independant X/Y CPI disabled
	HAL_Delay_us(40);
	writeReg(0x0d, 0x00); // Camera angle
	HAL_Delay_us(40);
	writeReg(0x11, 0x00); // Camera angle fine tuning
	HAL_Delay_us(40);
	uint8_t dpi = 0x03;
	writeReg(0x0f, dpi); // DPI
	HAL_Delay_us(40);
	// LOD Stuff
	writeReg(0x63, 0x03); // LOD: 0x00 disable lift detection, 0x02 = 2mm, 0x03 = 3mm
	HAL_Delay_us(40);
	writeReg(0x2b, 0x80); // Minimum SQUAL for zero motion data (default: 0x10)��max is 0x80
	HAL_Delay_us(40);
	writeReg(0x2c, 0x0a); // Minimum Valid features (reduce SQUAL score) (default: 0x0a)
	HAL_Delay_us(40);

	DEVICE::setReady();
	return error_t::OK;
}

error_t PMW3360::resetSPI()
{
	cs->clear();
	HAL_Delay_us(40);
	cs->set();
	HAL_Delay_us(40);

	return error_t::OK;
}

error_t PMW3360::sendShutdown()
{
	writeReg(REG_Shutdown, 0xb6);
	return error_t::OK;
}

error_t PMW3360::sendPowerUp()
{
	writeReg(REG_Power_Up_Reset, 0x5A);
	return error_t::OK;
}

error_t PMW3360::readData(Result_t& result)
{
	uint8_t regData = 0;
	uint8_t regDataH = 0;
	uint8_t regDataL = 0;

	writeReg(REG_Motion, 0x00);

	readReg(REG_Motion, regData);
	result.isMotion = (regData & 0x80) != 0;
	result.isOnSurface = (regData & 0x08) == 0;   // 0 if on surface / 1 if off surface

	if (result.isMotion)
	{
		readReg(REG_Delta_X_L, regDataL);
		readReg(REG_Delta_X_H, regDataH);
		result.dx = regDataH<<8 | regDataL;

		readReg(REG_Delta_Y_L, regDataL);
		readReg(REG_Delta_Y_H, regDataH);
		result.dy = regDataH<<8 | regDataL;

		result.SQUAL = 0;
		result.rawDataSum = 0;
		result.maxRawData = 0;
		result.minRawData = 0;
	}

	return error_t::OK;
}

error_t PMW3360::readDataBurst(Result_t& result)
{
	// "4.0 Burst mode operation" process from Datasheet
	uint8_t burstBuffer[12];

	// Activate burst mode
	writeReg(REG_Motion_Burst, 0x00);

	// Activate SPI chip select line
	cs->clear();

	uint8_t cmd = REG_Motion_Burst;
	spi->send(&cmd, sizeof(cmd));
	HAL_Delay_us(35); // waits for tSRAD

	spi->receive(burstBuffer, sizeof(burstBuffer));
	HAL_Delay_us(1); // tSCLK-NCS for read operation is 120ns

    // Deactivate SPI chip select line
	cs->set();

	// Process data received
	result.isMotion    = (burstBuffer[0] & 0x80) != 0;
	result.isOnSurface = (burstBuffer[0] & 0x08) == 0;   // 0 if on surface / 1 if off surface

	result.dx          = (burstBuffer[3] << 8) | burstBuffer[2];
	result.dy          = (burstBuffer[5] << 8) | burstBuffer[4];
	result.shutter     = (burstBuffer[11] << 8) | burstBuffer[10];

	result.SQUAL       = burstBuffer[6];
	result.rawDataSum  = burstBuffer[7];
	result.maxRawData  = burstBuffer[8];
	result.minRawData  = burstBuffer[9];

	return error_t::OK;
}

error_t PMW3360::uploadFW()
{
	// "5.0 SROM Download" process from Datasheet (steps 2..8)

	// 2. Write 0 to Rest_En bit of Config2 register to disable Rest mode.
	writeReg(REG_Config2, 0x00);
	HAL_Delay(10);

	// 3. Write 0x1d to SROM_Enable register for initializing
	writeReg(REG_SROM_Enable, 0x1d);

	// 4. Wait for 10 ms
	HAL_Delay(10);

	// 5. Write 0x18 to SROM_Enable register again to start SROM Download
	writeReg(REG_SROM_Enable, 0x18);
	HAL_Delay(10);

	// 6. Write SROM file into SROM_Load_Burst register, 1 st data must start with SROM_Load_Burst address.
	//    All the SROM data must be downloaded before SROM starts running
	cs->clear();

	uint8_t cmd = REG_SROM_Load_Burst | 0x80; // write burst destination adress
	spi->send(&cmd, sizeof(cmd));
//	HAL_SPI_Transmit(&hspi1, &cmd, sizeof(cmd), 1000);

	// send all bytes of the firmware
	uint8_t byte;
	for (int i = 0; i < FW_SIZE; i++) {
		HAL_Delay_us(16);
		byte = FW_DATA[i]; //(unsigned char)pgm_read_byte(firmware_data + i);
		spi->send(&byte, sizeof(byte));
	}
	HAL_Delay_us(18);
	cs->set();
	HAL_Delay_us(200);

	// 7. Read the SROM_ID register to verify the ID before any other register reads or writes.
	uint8_t reg_data;
	readReg(REG_SROM_ID, reg_data);

	// 8. Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
	writeReg(REG_Config2, 0x00);

	return error_t::OK;
}

bool PMW3360::checkSignature()
{
	uint8_t pid = 0;
	uint8_t iv_pid = 0;
	uint8_t SROM_ver = 0;

	readReg(REG_Product_ID, pid);
	readReg(REG_Inverse_Product_ID, iv_pid);
	readReg(REG_SROM_ID, SROM_ver);

	constexpr uint8_t EXPECTED_ID = 0x42;
	constexpr uint8_t EXPECTED_IV_PID = 0xBD;
	constexpr uint8_t EXPECTED_SROM_VER = 0x04;

	return pid==EXPECTED_ID &&
			iv_pid==EXPECTED_IV_PID &&
			EXPECTED_SROM_VER==0x04;
}

void PMW3360::setCPI(unsigned int cpi)
{
	uint8_t cpival = (cpi/100)-1;

	// Limits CPI to 0..119
	cpival = (cpival < 0) ? 0 : cpival;
	cpival = (0x77 < cpival) ? 0x77 : cpival;

	//_CPI = (cpival + 1)*100;

	writeReg(REG_Config1, cpival);
}

unsigned int PMW3360::getCPI()
{
	uint8_t cpival;
	readReg(REG_Config1, cpival);

	return (cpival + 1)*100;
}


error_t PMW3360::readReg(uint8_t reg_addr, uint8_t& reg_data)
{
    uint8_t buffer = reg_addr & 0x7f;

	  // Activate SPI chip select line
    cs->clear();

    // Send command
    spi->send(&buffer, sizeof(buffer));
	HAL_Delay_us(200);

    // Receive data
    spi->receive(&reg_data, sizeof(reg_data));
	HAL_Delay_us(120);

    // Deactivate SPI chip select line
    cs->set();

    return error_t::OK;
}

error_t PMW3360::writeReg(uint8_t reg_addr, uint8_t data)
{
    // Compose transmission buffer (address + data)
	constexpr uint8_t len = sizeof(reg_addr)+sizeof(data);
    uint8_t buffer[len];
    buffer[0] = reg_addr | 0x80;
    buffer[1] = data;

    // Activate SPI chip select line
    cs->clear();

    // Send buffer
    spi->send(buffer, sizeof(buffer));
	HAL_Delay_us(35);

    // Deactivate SPI chip select line
    cs->set();

    return error_t::OK;
}


} /* namespace HARDWARE::SENSORS */
