/*
 * tof_API.cpp
 *
 *  Created on: Jul 21, 2024
 *      Author: agamb
 */

#include "hardware/sensors/tof_vl53l1x.h"

namespace
{

#define SOFT_RESET											0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS					0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND        0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 		0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 	0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 	0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM					0x001E
#define MM_CONFIG__INNER_OFFSET_MM							0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 							0x0022
#define GPIO_HV_MUX__CTRL									0x0030
#define GPIO__TIO_HV_STATUS       							0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO 						0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP     				0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI   				0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A        				0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B						0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI  					0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO  					0x0062
#define RANGE_CONFIG__SIGMA_THRESH 							0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS			0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH      				0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD				0x006C
#define SYSTEM__THRESH_HIGH 								0x0072
#define SYSTEM__THRESH_LOW 									0x0074
#define SD_CONFIG__WOI_SD0                  				0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0        				0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD					0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE		0x0080
#define SYSTEM__SEQUENCE_CONFIG								0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD 				0x0082
#define SYSTEM__INTERRUPT_CLEAR       						0x0086
#define SYSTEM__MODE_START                 					0x0087
#define VL53L1_RESULT__RANGE_STATUS							0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0		0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD					0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0				0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 	0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL					0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS                      0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID                     0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD				0x013E

const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
        0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
        0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
        0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
        0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
        0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
        0x00, /* 0x32 : not user-modifiable */
        0x02, /* 0x33 : not user-modifiable */
        0x08, /* 0x34 : not user-modifiable */
        0x00, /* 0x35 : not user-modifiable */
        0x08, /* 0x36 : not user-modifiable */
        0x10, /* 0x37 : not user-modifiable */
        0x01, /* 0x38 : not user-modifiable */
        0x01, /* 0x39 : not user-modifiable */
        0x00, /* 0x3a : not user-modifiable */
        0x00, /* 0x3b : not user-modifiable */
        0x00, /* 0x3c : not user-modifiable */
        0x00, /* 0x3d : not user-modifiable */
        0xff, /* 0x3e : not user-modifiable */
        0x00, /* 0x3f : not user-modifiable */
        0x0F, /* 0x40 : not user-modifiable */
        0x00, /* 0x41 : not user-modifiable */
        0x00, /* 0x42 : not user-modifiable */
        0x00, /* 0x43 : not user-modifiable */
        0x00, /* 0x44 : not user-modifiable */
        0x00, /* 0x45 : not user-modifiable */
        0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
        0x0b, /* 0x47 : not user-modifiable */
        0x00, /* 0x48 : not user-modifiable */
        0x00, /* 0x49 : not user-modifiable */
        0x02, /* 0x4a : not user-modifiable */
        0x0a, /* 0x4b : not user-modifiable */
        0x21, /* 0x4c : not user-modifiable */
        0x00, /* 0x4d : not user-modifiable */
        0x00, /* 0x4e : not user-modifiable */
        0x05, /* 0x4f : not user-modifiable */
        0x00, /* 0x50 : not user-modifiable */
        0x00, /* 0x51 : not user-modifiable */
        0x00, /* 0x52 : not user-modifiable */
        0x00, /* 0x53 : not user-modifiable */
        0xc8, /* 0x54 : not user-modifiable */
        0x00, /* 0x55 : not user-modifiable */
        0x00, /* 0x56 : not user-modifiable */
        0x38, /* 0x57 : not user-modifiable */
        0xff, /* 0x58 : not user-modifiable */
        0x01, /* 0x59 : not user-modifiable */
        0x00, /* 0x5a : not user-modifiable */
        0x08, /* 0x5b : not user-modifiable */
        0x00, /* 0x5c : not user-modifiable */
        0x00, /* 0x5d : not user-modifiable */
        0x01, /* 0x5e : not user-modifiable */
        0xcc, /* 0x5f : not user-modifiable */
        0x0f, /* 0x60 : not user-modifiable */
        0x01, /* 0x61 : not user-modifiable */
        0xf1, /* 0x62 : not user-modifiable */
        0x0d, /* 0x63 : not user-modifiable */
        0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
        0x68, /* 0x65 : Sigma threshold LSB */
        0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
        0x80, /* 0x67 : Min count Rate LSB */
        0x08, /* 0x68 : not user-modifiable */
        0xb8, /* 0x69 : not user-modifiable */
        0x00, /* 0x6a : not user-modifiable */
        0x00, /* 0x6b : not user-modifiable */
        0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
        0x00, /* 0x6d : Intermeasurement period */
        0x0f, /* 0x6e : Intermeasurement period */
        0x89, /* 0x6f : Intermeasurement period LSB */
        0x00, /* 0x70 : not user-modifiable */
        0x00, /* 0x71 : not user-modifiable */
        0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
        0x00, /* 0x73 : distance threshold high LSB */
        0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
        0x00, /* 0x75 : distance threshold low LSB */
        0x00, /* 0x76 : not user-modifiable */
        0x01, /* 0x77 : not user-modifiable */
        0x0f, /* 0x78 : not user-modifiable */
        0x0d, /* 0x79 : not user-modifiable */
        0x0e, /* 0x7a : not user-modifiable */
        0x0e, /* 0x7b : not user-modifiable */
        0x00, /* 0x7c : not user-modifiable */
        0x00, /* 0x7d : not user-modifiable */
        0x02, /* 0x7e : not user-modifiable */
        0xc7, /* 0x7f : ROI center, use SetROI() */
        0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
        0x9B, /* 0x81 : not user-modifiable */
        0x00, /* 0x82 : not user-modifiable */
        0x00, /* 0x83 : not user-modifiable */
        0x00, /* 0x84 : not user-modifiable */
        0x01, /* 0x85 : not user-modifiable */
        0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
        0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after init() call, put 0x40 in location 0x87 */
    };


const uint8_t status_rtn[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
	255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
	255, 255, 11, 12
};

}

namespace HARDWARE::SENSORS
{

int8_t TOF_VL53L1X::SensorInit()
{
	int8_t status = 0;
	uint8_t Addr = 0x00, tmp;

	for (Addr = 0x2D; Addr <= 0x87; Addr++){
		status |= I2C_WrByte(Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
	}
	status |= StartRanging();
	tmp  = 0;
	while(tmp==0){
			status |= CheckForDataReady(&tmp);
	}
	status |= ClearInterrupt();
	status |= StopRanging();
	status |= I2C_WrByte(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
	status |= I2C_WrByte(0x0B, 0); /* start VHV from the previous temperature */
	return status;
}

int8_t TOF_VL53L1X::ClearInterrupt()
{
	int8_t status = 0;

	status |= I2C_WrByte(SYSTEM__INTERRUPT_CLEAR, 0x01);
	return status;
}

int8_t TOF_VL53L1X::GetInterruptPolarity(uint8_t *pInterruptPolarity)
{
	uint8_t Temp;
	int8_t status = 0;

	status |= I2C_RdByte(GPIO_HV_MUX__CTRL, &Temp);
	Temp = Temp & 0x10;
	*pInterruptPolarity = !(Temp>>4);
	return status;
}

int8_t TOF_VL53L1X::StartRanging()
{
	int8_t status = 0;

	status |= I2C_WrByte(SYSTEM__MODE_START, 0x40);	/* Enable VL53L1X */
	return status;
}

int8_t TOF_VL53L1X::StopRanging()
{
	int8_t status = 0;

	status |= I2C_WrByte(SYSTEM__MODE_START, 0x00);	/* Disable VL53L1X */
	return status;
}

int8_t TOF_VL53L1X::CheckForDataReady(uint8_t *isDataReady)
{
	uint8_t Temp;
	uint8_t IntPol;
	int8_t status = 0;

	status |= GetInterruptPolarity(&IntPol);
	status |= I2C_RdByte(GPIO__TIO_HV_STATUS, &Temp);
	/* Read in the register to check if a new value is available */
	if (status == 0){
		if ((Temp & 1) == IntPol)
			*isDataReady = 1;
		else
			*isDataReady = 0;
	}
	return status;
}

int8_t TOF_VL53L1X::BootState(uint8_t *state)
{
	int8_t status = 0;
	uint8_t tmp = 0;

	status |= I2C_RdByte(VL53L1_FIRMWARE__SYSTEM_STATUS, &tmp);
	*state = tmp;
	return status;
}

int8_t TOF_VL53L1X::SetI2CAddress(uint8_t new_address)
{
	int8_t status = 0;

	status |= I2C_WrByte(VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address);
	return status;
}

int8_t TOF_VL53L1X::SetTimingBudgetInMs(uint16_t TimingBudgetInMs)
{
	uint16_t DM;
	int8_t  status=0;

	status |= GetDistanceMode(&DM);
	if (DM == 0)
		return 1;
	else if (DM == 1) {	/* Short DistanceMode */
		switch (TimingBudgetInMs) {
		case 15: /* only available in short distance mode */
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x01D);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x0027);
			break;
		case 20:
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x0051);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x006E);
			break;
		case 33:
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x00D6);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x006E);
			break;
		case 50:
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x1AE);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x01E8);
			break;
		case 100:
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x02E1);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x0388);
			break;
		case 200:
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x03E1);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x0496);
			break;
		case 500:
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x0591);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x05C1);
			break;
		default:
			status = 1;
			break;
		}
	} else {
		switch (TimingBudgetInMs) {
		case 20:
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x001E);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x0022);
			break;
		case 33:
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x0060);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x006E);
			break;
		case 50:
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x00AD);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x00C6);
			break;
		case 100:
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x01CC);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x01EA);
			break;
		case 200:
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x02D9);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x02F8);
			break;
		case 500:
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x048F);
			I2C_WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x04A4);
			break;
		default:
			status = 1;
			break;
		}
	}
	return status;
}

int8_t TOF_VL53L1X::GetTimingBudgetInMs(uint16_t *pTimingBudgetInMs)
{
	uint16_t Temp;
	int8_t status = 0;

	status |= I2C_RdWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &Temp);
	switch (Temp) {
		case 0x001D :
			*pTimingBudgetInMs = 15;
			break;
		case 0x0051 :
		case 0x001E :
			*pTimingBudgetInMs = 20;
			break;
		case 0x00D6 :
		case 0x0060 :
			*pTimingBudgetInMs = 33;
			break;
		case 0x1AE :
		case 0x00AD :
			*pTimingBudgetInMs = 50;
			break;
		case 0x02E1 :
		case 0x01CC :
			*pTimingBudgetInMs = 100;
			break;
		case 0x03E1 :
		case 0x02D9 :
			*pTimingBudgetInMs = 200;
			break;
		case 0x0591 :
		case 0x048F :
			*pTimingBudgetInMs = 500;
			break;
		default:
			status = 1;
			*pTimingBudgetInMs = 0;
	}
	return status;
}

int8_t TOF_VL53L1X::SetDistanceMode(uint16_t distanceMode)
{
	uint16_t TB;
	uint8_t status = 0;

	status |= GetTimingBudgetInMs(&TB);
	if (status != 0)
		return 1;
	switch (distanceMode) {
	case 1:
		status = I2C_WrByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
		status = I2C_WrByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
		status = I2C_WrByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
		status = I2C_WrByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
		status = I2C_WrWord(SD_CONFIG__WOI_SD0, 0x0705);
		status = I2C_WrWord(SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
		break;
	case 2:
		status = I2C_WrByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
		status = I2C_WrByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
		status = I2C_WrByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
		status = I2C_WrByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
		status = I2C_WrWord(SD_CONFIG__WOI_SD0, 0x0F0D);
		status = I2C_WrWord(SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
		break;
	default:
		status = 1;
		break;
	}

	if (status == 0)
		status |= SetTimingBudgetInMs(TB);
	return status;
}

int8_t TOF_VL53L1X::GetDistanceMode(uint16_t *pDistanceMode)
{
	uint8_t TempDM = 0;
	int8_t status=0;

	status |= I2C_RdByte(PHASECAL_CONFIG__TIMEOUT_MACROP, &TempDM);
	if (TempDM == 0x14)
		*pDistanceMode=1;
	if(TempDM == 0x0A)
		*pDistanceMode=2;
	return status;
}

int8_t TOF_VL53L1X::SetDistanceThreshold(uint16_t ThreshLow,
				  uint16_t ThreshHigh, uint8_t Window,
				  uint8_t IntOnNoTarget)
{
	uint8_t status = 0;
	uint8_t Temp = 0;

	status |= I2C_RdByte(SYSTEM__INTERRUPT_CONFIG_GPIO, &Temp);
	Temp = Temp & 0x47;
	if (IntOnNoTarget == 0) {
		status = I2C_WrByte(SYSTEM__INTERRUPT_CONFIG_GPIO,
			       (Temp | (Window & 0x07)));
	} else {
		status = I2C_WrByte(SYSTEM__INTERRUPT_CONFIG_GPIO,
			       ((Temp | (Window & 0x07)) | 0x40));
	}
	status |= I2C_WrWord(SYSTEM__THRESH_HIGH, ThreshHigh);
	status |= I2C_WrWord(SYSTEM__THRESH_LOW, ThreshLow);
	return status;
}
                  
int8_t TOF_VL53L1X::SetROI(uint16_t X, uint16_t Y)
{
	uint8_t OpticalCenter;
	int8_t status = 0;

	status |=I2C_RdByte(VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, &OpticalCenter);
	if (X > 16)
		X = 16;
	if (Y > 16)
		Y = 16;
	if (X > 10 || Y > 10){
		OpticalCenter = 199;
	}
	status |= I2C_WrByte(ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter);
	status |= I2C_WrByte(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
		       (Y - 1) << 4 | (X - 1));
	return status;
}

int8_t TOF_VL53L1X::SetROICenter(uint8_t ROICenter)
{
	int8_t status = 0;
	status |= I2C_WrByte(ROI_CONFIG__USER_ROI_CENTRE_SPAD, ROICenter);
	return status;
}


int8_t TOF_VL53L1X::GetSensorId(uint16_t *sensorId)
{
	int8_t status = 0;
	uint16_t tmp = 0;

	status |= I2C_RdWord(VL53L1_IDENTIFICATION__MODEL_ID, &tmp);
	*sensorId = tmp;
	return status;
}

int8_t TOF_VL53L1X::GetDistance(uint16_t *distance)
{
	int8_t status = 0;
	uint16_t tmp;

	status |= (I2C_RdWord(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp));
	*distance = tmp;
	return status;
}

int8_t TOF_VL53L1X::GetSignalRate(uint16_t *signalRate)
{
	int8_t status = 0;
	uint16_t tmp;

	status |= I2C_RdWord(VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &tmp);
	*signalRate = tmp*8;
	return status;
}

int8_t TOF_VL53L1X::GetSpadNb(uint16_t *spNb)
{
	int8_t status = 0;
	uint16_t tmp;

	status |= I2C_RdWord(VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &tmp);
	*spNb = tmp >> 8;
	return status;
}

int8_t TOF_VL53L1X::GetAmbientRate(uint16_t *ambRate)
{
	int8_t status = 0;
	uint16_t tmp;

	status |= I2C_RdWord(RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &tmp);
	*ambRate = tmp*8;
	return status;
}

int8_t TOF_VL53L1X::GetRangeStatus(uint8_t *rangeStatus)
{
	int8_t status = 0;
	uint8_t RgSt;

	*rangeStatus = 255;
	status |= I2C_RdByte(VL53L1_RESULT__RANGE_STATUS, &RgSt);
	RgSt = RgSt & 0x1F;
	if (RgSt < 24)
		*rangeStatus = status_rtn[RgSt];
	return status;
}

int8_t TOF_VL53L1X::GetResult(TOF_VL53L1X::Result_t *pResult)
{
	int8_t status = 0;
	uint8_t Temp[17];
	uint8_t RgSt = 255;

	status |= I2C_ReadMulti(VL53L1_RESULT__RANGE_STATUS, Temp, 17);
	RgSt = Temp[0] & 0x1F;
	if (RgSt < 24)
		RgSt = status_rtn[RgSt];
	pResult->status = RgSt;
	pResult->ambient = (Temp[7] << 8 | Temp[8]) * 8;
	pResult->numSPADs = Temp[3];
	pResult->sigPerSPAD = (Temp[15] << 8 | Temp[16]) * 8;
	pResult->distance = Temp[13] << 8 | Temp[14];

	return status;
}

int8_t TOF_VL53L1X::GetOffset(int16_t *offset)
{
	int8_t status = 0;
	uint16_t Temp;

	status |= I2C_RdWord(ALGO__PART_TO_PART_RANGE_OFFSET_MM, &Temp);
	Temp = Temp<<3;
	Temp = Temp>>5;
   *offset = (int16_t)(Temp);

   if(*offset > 1024) 
   {
		*offset = *offset - 2048;
   }

	return status;
}

int8_t TOF_VL53L1X::GetROI_XY(uint16_t *ROI_X, uint16_t *ROI_Y)
{
	int8_t status = 0;
	uint8_t tmp;

	status = I2C_RdByte(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, &tmp);
	*ROI_X |= ((uint16_t)tmp & 0x0F) + 1;
	*ROI_Y |= (((uint16_t)tmp & 0xF0) >> 4) + 1;
	return status;
}

}
