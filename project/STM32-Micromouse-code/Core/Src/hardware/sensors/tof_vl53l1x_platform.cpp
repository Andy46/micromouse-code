/*
 * tof_vl53l1x.cpp
 *
 *  Created on: Jul 16, 2024
 *      Author: agamb
 */

#include "hardware/sensors/tof_vl53l1x.h"

#include "main.h"

#include <stdio.h>
#include <string.h>

namespace
{

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1

#define ERROR_NONE 								((int8_t)  0)
#define ERROR_CALIBRATION_WARNING               ((int8_t) - 1)
	/*!< Warning invalid calibration data may be in used
		\a  VL53L1_InitData()
		\a VL53L1_GetOffsetCalibrationData
		\a VL53L1_SetOffsetCalibrationData */
#define ERROR_MIN_CLIPPED                       ((int8_t) - 2)
	/*!< Warning parameter passed was clipped to min before to be applied */

#define ERROR_UNDEFINED                         ((int8_t) - 3)
	/*!< Unqualified error */
#define ERROR_INVALID_PARAMS                    ((int8_t) - 4)
	/*!< Parameter passed is invalid or out of range */
#define ERROR_NOT_SUPPORTED                     ((int8_t) - 5)
	/*!< Function is not supported in current mode or configuration */
#define ERROR_RANGE_ERROR                       ((int8_t) - 6)
	/*!< Device report a ranging error interrupt status */
#define ERROR_TIME_OUT                          ((int8_t) - 7)
	/*!< Aborted due to time out */
#define ERROR_MODE_NOT_SUPPORTED                ((int8_t) - 8)
	/*!< Asked mode is not supported by the device */
#define ERROR_BUFFER_TOO_SMALL                  ((int8_t) - 9)
	/*!< ... */
#define ERROR_COMMS_BUFFER_TOO_SMALL            ((int8_t) - 10)
	/*!< Supplied buffer is larger than I2C supports */
#define ERROR_GPIO_NOT_EXISTING                 ((int8_t) - 11)
	/*!< User tried to setup a non-existing GPIO pin */
#define ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED  ((int8_t) - 12)
	/*!< unsupported GPIO functionality */
#define ERROR_CONTROL_INTERFACE                 ((int8_t) - 13)
	/*!< error reported from IO functions */
#define ERROR_INVALID_COMMAND                   ((int8_t) - 14)
	/*!< The command is not allowed in the current device state
	 *  (power down) */
#define ERROR_DIVISION_BY_ZERO                  ((int8_t) - 15)
	/*!< In the function a division by zero occurs */
#define ERROR_REF_SPAD_INIT                     ((int8_t) - 16)
	/*!< Error during reference SPAD initialization */
#define ERROR_GPH_SYNC_CHECK_FAIL               ((int8_t) - 17)
	/*!<  GPH sync interrupt check fail - API out of sync with device*/
#define ERROR_STREAM_COUNT_CHECK_FAIL           ((int8_t) - 18)
	/*!<  Stream count check fail - API out of sync with device */
#define ERROR_GPH_ID_CHECK_FAIL                 ((int8_t) - 19)
	/*!<  GPH ID check fail - API out of sync with device */
#define ERROR_ZONE_STREAM_COUNT_CHECK_FAIL      ((int8_t) - 20)
	/*!<  Zone dynamic config stream count check failed - API out of sync */
#define ERROR_ZONE_GPH_ID_CHECK_FAIL            ((int8_t) - 21)
	/*!<  Zone dynamic config GPH ID check failed - API out of sync */

#define ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL   ((int8_t) - 22)
	/*!<  Thrown when run_xtalk_extraction fn has 0 succesful samples
	 * when using the full array to sample the xtalk. In this case there is
	 * not enough information to generate new Xtalk parm info. The function
	 * will exit and leave the current xtalk parameters unaltered */
#define ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL ((int8_t) - 23)
	/*!<  Thrown when run_xtalk_extraction fn has found that the
	 * avg sigma estimate of the full array xtalk sample is > than the
	 * maximal limit allowed. In this case the xtalk sample is too noisy for
	 * measurement. The function will exit and leave the current xtalk parameters
	 * unaltered. */


#define ERROR_OFFSET_CAL_NO_SAMPLE_FAIL           ((int8_t) - 24)
	/*!<  Thrown if there one of stages has no valid offset calibration
	 *    samples. A fatal error calibration not valid */
#define ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL    ((int8_t) - 25)
	/*!<  Thrown if there one of stages has zero effective SPADS
	 *    Traps the case when MM1 SPADs is zero.
	 *    A fatal error calibration not valid */
#define ERROR_ZONE_CAL_NO_SAMPLE_FAIL             ((int8_t) - 26)
	/*!<  Thrown if then some of the zones have no valid samples
	 *    A fatal error calibration not valid */

#define ERROR_TUNING_PARM_KEY_MISMATCH             ((int8_t) - 27)
	/*!<  Thrown if the tuning file key table version does not match with
	 * expected value. The driver expects the key table version to match
	 * the compiled default version number in the define
	 * #VL53L1_TUNINGPARM_KEY_TABLE_VERSION_DEFAULT
	 * */

#define VL53L1_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS   ((int8_t) - 28)
	/*!<  Thrown if there are less than 5 good SPADs are available. */
#define VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH      ((int8_t) - 29)
	/*!<  Thrown if the final reference rate is greater than
	      the upper reference rate limit - default is 40 Mcps.
	      Implies a minimum Q3 (x10) SPAD (5) selected */
#define VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW       ((int8_t) - 30)
	/*!<  Thrown if the final reference rate is less than
	      the lower reference rate limit - default is 10 Mcps.
	      Implies maximum Q1 (x1) SPADs selected */


#define VL53L1_WARNING_OFFSET_CAL_MISSING_SAMPLES       ((int8_t) - 31)
	/*!<  Thrown if there is less than the requested number of
	 *    valid samples. */
#define VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH        ((int8_t) - 32)
	/*!<  Thrown if the offset calibration range sigma estimate is greater
	 *    than 8.0 mm. This is the recommended min value to yield a stable
	 *    offset measurement */
#define VL53L1_WARNING_OFFSET_CAL_RATE_TOO_HIGH         ((int8_t) - 33)
	/*!< Thrown when VL53L1_run_offset_calibration()  peak rate is greater
	     than that 50.0Mcps. This is the recommended  max rate to avoid
	     pile-up influencing the offset measurement */
#define VL53L1_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW    ((int8_t) - 34)
	/*!< Thrown when VL53L1_run_offset_calibration() when one of stages
	     range has less that 5.0 effective SPADS. This is the recommended
	     min value to yield a stable offset */


#define VL53L1_WARNING_ZONE_CAL_MISSING_SAMPLES       ((int8_t) - 35)
	/*!<  Thrown if one of more of the zones have less than
	      the requested number of valid samples */
#define VL53L1_WARNING_ZONE_CAL_SIGMA_TOO_HIGH        ((int8_t) - 36)
	/*!<  Thrown if one or more zones have sigma estimate value greater
	 *    than 8.0 mm. This is the recommended min value to yield a stable
	 *    offset measurement */
#define VL53L1_WARNING_ZONE_CAL_RATE_TOO_HIGH         ((int8_t) - 37)
	/*!< Thrown if one of more zones have  peak rate higher than
	      that 50.0Mcps. This is the recommended  max rate to avoid
	     pile-up influencing the offset measurement */


#define VL53L1_WARNING_XTALK_MISSING_SAMPLES             ((int8_t) - 38)
	/*!< Thrown to notify that some of the xtalk samples did not yield
	 * valid ranging pulse data while attempting to measure
	 * the xtalk signal in vl53l1_run_xtalk_extract(). This can signify any of
	 * the zones are missing samples, for further debug information the
	 * xtalk_results struct should be referred to. This warning is for
	 * notification only, the xtalk pulse and shape have still been generated
	 */
#define VL53L1_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT     ((int8_t) - 39)
	/*!< Thrown to notify that some of teh xtalk samples used for gradient
	 * generation did not yield valid ranging pulse data while attempting to
	 * measure the xtalk signal in vl53l1_run_xtalk_extract(). This can signify
	 * that any one of the zones 0-3 yielded no successful samples. The
	 * xtalk_results struct should be referred to for further debug info.
	 * This warning is for notification only, the xtalk pulse and shape
	 * have still been generated.
	 */
#define VL53L1_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT    ((int8_t) - 40)
/*!< Thrown to notify that some of the xtalk samples used for gradient
	 * generation did not pass the sigma limit check  while attempting to
	 * measure the xtalk signal in vl53l1_run_xtalk_extract(). This can signify
	 * that any one of the zones 0-3 yielded an avg sigma_mm value > the limit.
	 * The xtalk_results struct should be referred to for further debug info.
	 * This warning is for notification only, the xtalk pulse and shape
	 * have still been generated.
	 */

#define ERROR_NOT_IMPLEMENTED                   ((int8_t) - 41)
	/*!< Tells requested functionality has not been implemented yet or
	 * not compatible with the device */
#define ERROR_PLATFORM_SPECIFIC_START           ((int8_t) - 60)
	/*!< Tells the starting code for platform */
/** @} VL53L1_define_Error_group */

uint8_t _I2CBuffer[256];

}

namespace HARDWARE::SENSORS
{

int TOF_VL53L1X::_I2CWrite(uint8_t *pdata, uint32_t count)
{
	int status;
	int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;

	status = i2c->send(address, pdata, count, i2c_time_out); // TODO: Verify address
	if (status) {
		//VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
		//XNUCLEO6180XA1_I2C1_Init(&hi2c1);
	}
	return status;
}

int TOF_VL53L1X::_I2CRead(uint8_t *pdata, uint32_t count)
{
	int status;
	int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;

	status = i2c->receive(address, pdata, count, i2c_time_out); // TODO: Verify address
	if (status) {
		//VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
		//XNUCLEO6180XA1_I2C1_Init(&hi2c1);
	}
	return status;
}

int8_t TOF_VL53L1X::I2C_WriteMulti(uint16_t index, uint8_t *pdata, uint32_t count)
{
    int status_int;
    int8_t Status = 0;
    if (count > sizeof(_I2CBuffer) - 1) {
        return ERROR_INVALID_PARAMS;
    }
    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    memcpy(&_I2CBuffer[2], pdata, count);

    status_int = _I2CWrite(_I2CBuffer, count + 2);
    if (status_int != 0) {
        Status = ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

int8_t TOF_VL53L1X::I2C_ReadMulti(uint16_t index, uint8_t *pdata, uint32_t count) {
    int8_t Status = 0;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;

    status_int = _I2CWrite(_I2CBuffer, 2);
    if (status_int != 0) {
        Status = ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(pdata, count);
    if (status_int != 0) {
        Status = ERROR_CONTROL_INTERFACE;
    }
done:
    return Status;
}

int8_t TOF_VL53L1X::I2C_WrByte(uint16_t index, uint8_t data) {
    int8_t Status = 0;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data;

    status_int = _I2CWrite(_I2CBuffer, 3);
    if (status_int != 0) {
        Status = ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

int8_t TOF_VL53L1X::I2C_WrWord(uint16_t index, uint16_t data)
{
    int8_t Status = ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
    _I2CBuffer[1] = index&0xFF;
    _I2CBuffer[2] = data >> 8;
    _I2CBuffer[3] = data & 0x00FF;

    status_int = _I2CWrite(_I2CBuffer, 4);
    if (status_int != 0) {
        Status = ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

int8_t TOF_VL53L1X::I2C_RdByte(uint16_t index, uint8_t *data) {
	int8_t Status = 0;
    int32_t status_int;

	_I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index&0xFF;

    status_int = _I2CWrite(_I2CBuffer, 2);
    if( status_int ){
        Status = ERROR_CONTROL_INTERFACE;
        goto done;
    }

    status_int = _I2CRead(data, 1);
    if (status_int != 0) {
        Status = ERROR_CONTROL_INTERFACE;
    }
done:
    return Status;
}

int8_t TOF_VL53L1X::I2C_RdWord(uint16_t index, uint16_t *data)
{
    int8_t Status = ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index>>8;
	_I2CBuffer[1] = index&0xFF;

    status_int = _I2CWrite(_I2CBuffer, 2);
    if( status_int ){
        Status = ERROR_CONTROL_INTERFACE;
        goto done;
    }

    status_int = _I2CRead(_I2CBuffer, 2);
    if (status_int != 0) {
        Status = ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint16_t)_I2CBuffer[0]<<8) + (uint16_t)_I2CBuffer[1];

done:
    return Status;
}

} /* namespace HARDWARE::SENSORS */
