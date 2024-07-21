/*
 * encoder.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

/* C/C++ libraries */
#include <stdint.h>

/* BMI160 sensor library */
#include <bmi160.h>

namespace HARDWARE::SENSORS
{

class BMI160
{
private:
	struct bmi160_dev bmi160dev;
	int16_t calibrationZ = 0;

//	GPIO_TypeDef* gpio;
//	uint16_t      pin;

public:
	BMI160();
	virtual ~BMI160() = default;

	int8_t init();

	int8_t read_accel(struct bmi160_sensor_data* accel_data);
	int8_t read_gyro(struct bmi160_sensor_data* gyro_data);
	int8_t read_all(struct bmi160_sensor_data* accel_data, struct bmi160_sensor_data* gyro_data);
};

} /* namespace HARDWARE::SENSORS */
