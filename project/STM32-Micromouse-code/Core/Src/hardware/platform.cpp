/*
 * platform.cpp
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#include "hardware/platform.h"
#include <stdio.h>
#include <array>

#include "hardware/sensors/pmw3360.h"

namespace HARDWARE
{

Platform::Platform(std::shared_ptr<SENSORS::TOF_VL53L1X> tof_left,
					 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_frontleft,
					 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_frontright,
					 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_right,
					 std::shared_ptr<SENSORS::BMI160> bmi160,
					 std::shared_ptr<SENSORS::PMW3360> pmw3360,
					 std::shared_ptr<EXTRA::LEDS> leds,
					 std::shared_ptr<EXTRA::Switches> switches
				   ) :
		tof_left(tof_left), tof_frontleft(tof_frontleft),
		tof_frontright(tof_frontright), tof_right(tof_right),
		bmi160(bmi160), pmw3360(pmw3360), leds(leds), switches(switches)
{

}

void Platform::init()
{
	/* Init TOF sensors */
	constexpr uint8_t TOF_1_ADDRESS = 0x71;
	constexpr uint8_t TOF_2_ADDRESS = 0x72;
	constexpr uint8_t TOF_3_ADDRESS = 0x73;
	constexpr uint8_t TOF_4_ADDRESS = 0x74;
	constexpr int TOF_TURN_ON_DELAY_MS = 10;

	tof_left->turnOff();
	tof_frontleft->turnOff();
	tof_frontright->turnOff();
	tof_right->turnOff();

	printf("=== Init Platform ===\n");

	printf("Init TOF Left\n");
	tof_left->turnOn();
	HAL_Delay(TOF_TURN_ON_DELAY_MS);
	tof_left->changeAddress(TOF_1_ADDRESS);
	tof_left->init();
	tof_left->configure(SENSORS::TOF_VL53L1X::SHORT);

	printf("Init TOF Front Left\n");
	tof_frontleft->turnOn();
	HAL_Delay(TOF_TURN_ON_DELAY_MS);
	tof_frontleft->changeAddress(TOF_2_ADDRESS);
	tof_frontleft->init();
	tof_frontleft->configure(SENSORS::TOF_VL53L1X::SHORT);

	printf("Init TOF Front Right\n");
	tof_frontright->turnOn();
	HAL_Delay(TOF_TURN_ON_DELAY_MS);
	tof_frontright->changeAddress(TOF_3_ADDRESS);
	tof_frontright->init();
	tof_frontright->configure(SENSORS::TOF_VL53L1X::SHORT);

	printf("Init TOF Right\n");
	tof_right->turnOn();
	HAL_Delay(TOF_TURN_ON_DELAY_MS);
	tof_right->changeAddress(TOF_4_ADDRESS);
	tof_right->init();
	tof_right->configure(SENSORS::TOF_VL53L1X::SHORT);

	/* Init LEDs */
//	printf("Init LEDs\n");
//	leds->turnOff(0);
//	leds->turnOff(1);
//	leds->turnOff(2);
//	leds->turnOff(3);

	/* Init Switch status */

	/* Init IMUs */
	printf("Init BMI\n");
	bmi160->init();
	bmi160->configure();

	/* Init PMW sensor */
	printf("Init PMW3360\n");
	pmw3360->init();
	pmw3360->configure();
}

void Platform::checkStatus()
{
	printf("=== Platform status ===\n");
	printf("TOF status:\n");
	printf("  Left        : %d\n", tof_left->getStatus());
	printf("  Front-left  : %d\n", tof_frontleft->getStatus());
	printf("  Front-right : %d\n", tof_frontright->getStatus());
	printf("  Right       : %d\n", tof_right->getStatus());
	printf("\n");
	printf("IMU status:\n");
	printf("  BMI160      : %d\n", bmi160->getStatus());
	printf("  MPU9250     : Not available\n");
	printf("\n");
	printf("Other status:\n");
	printf("  PMW3360     : %d\n", pmw3360->getStatus());
}

void Platform::run_test()
{

	constexpr int TOF_COUNT = 4;
	std::array<std::shared_ptr<SENSORS::TOF_VL53L1X>, TOF_COUNT> tofs {tof_left, tof_frontleft, tof_frontright, tof_right};
	std::array<SENSORS::TOF_VL53L1X::Result_t, TOF_COUNT> tof_results;

	for (int i=0; i<TOF_COUNT; i++)
	{
		tofs[i]->start();
	}

    struct bmi160_sensor_data accel_data, gyro_data;

	SENSORS::PMW3360::Result_t pmw_result;
	int pmw_pos_x = 0, pmw_pos_y = 0;


	printf("TOF L - TOF FL - TOF FR - TOF R - BMI GZ - PMW X - PMW Y\n");
	while (1)
	{

		for (int i=0; i<TOF_COUNT; i++)
		{
			if(tofs[i]->isDataReady())
			{
				tofs[i]->getDataAndRestart(tof_results[i]);
			}
			else
			{
				tof_results[i].status = 1;
			}
		}

        bmi160->read_all(accel_data, gyro_data);

		pmw3360->readDataBurst(pmw_result);
		if (pmw_result.isMotion && pmw_result.isOnSurface)
		{
			pmw_pos_x += pmw_result.dx;
			pmw_pos_y += pmw_result.dy;
		}

		printf("%5d - %6d - %6d - %5d - %6d - %5d - %5d\n",
				(tof_results[0].status == 0) ? tof_results[0].distance : 0,
				(tof_results[1].status == 0) ? tof_results[1].distance : 0,
				(tof_results[2].status == 0) ? tof_results[2].distance : 0,
				(tof_results[3].status == 0) ? tof_results[3].distance : 0,
				gyro_data.z, pmw_pos_x, pmw_pos_y);
	}
}
} /* namespace HARDWARE */
