/*
 * bmi160.h
 *
 *  Created on: Jul 17, 2024
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

/* BMI160 sensor library */
#include <bmi160.h>

namespace HARDWARE::SENSORS
{

/**
 * External device "BMI160" - IMU (Accelerometer + Gyroscope)
 *
 * Description: Provides accelerometer and gyroscope data from BMI160 sensor through the SPI interface
 *
 * Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf
 **/
class BMI160 : public DEVICE
{
private:
	std::shared_ptr<COMMS::SPI> spi;
	std::shared_ptr<EXTRA::GPIO> cs;
	struct bmi160_dev bmi160dev;

public:
	BMI160(std::shared_ptr<COMMS::SPI> spi, std::shared_ptr<EXTRA::GPIO> cs);
	virtual ~BMI160() = default;

	error_t init();
	error_t configure();

	error_t read_accel(struct bmi160_sensor_data& accel_data);
	error_t read_gyro(struct bmi160_sensor_data& gyro_data);
	error_t read_all(struct bmi160_sensor_data& accel_data, struct bmi160_sensor_data& gyro_data);
};

} /* namespace HARDWARE::SENSORS */
