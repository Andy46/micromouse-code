/*
 * platform.cpp
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#include "hardware/platform.h"
#include <stdio.h>

namespace HARDWARE
{

Platform::Platform(std::shared_ptr<SENSORS::TOF_VL53L1X> tof_left,
					 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_frontleft,
					 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_frontright,
					 std::shared_ptr<SENSORS::TOF_VL53L1X> tof_right,
					 std::shared_ptr<SENSORS::BMI160> bmi160,
					 std::shared_ptr<EXTRA::LEDS> leds,
					 std::shared_ptr<EXTRA::Switches> switches
				   ) :
		tof_left(tof_left), tof_frontleft(tof_frontleft),
		tof_frontright(tof_frontright), tof_right(tof_right),
		bmi160(bmi160), leds(leds), switches(switches)
{

}

void Platform::init()
{

}

#define TEST_TOFS
#define TEST_LEDS
#define TEST_SWITCHES
#define TEST_BMI


void Platform::run_test()
{

#ifdef TEST_TOFS
	tof_left->turnOff();
	tof_frontleft->turnOff();
	tof_frontright->turnOff();
	tof_right->turnOff();
	HAL_Delay(10);
    
	//	tof_1->configure();
	//	tof_2->configure();
	//	tof_3->configure();
	//	tof_4->configure();


	if (tof_left)
	{
        printf("\n\n=== Testing Left TOF ===\n");
		tof_left->turnOn();
		HAL_Delay(20);
		tof_left->init();
		HAL_Delay(20);
		tof_left->StartRanging();

		SENSORS::TOF_VL53L1X::Result_t result;
        printf("Statu;Dista;sSPAD;ambie;nSPAD\n");
		for (int i=0; i<10; i++)
		{
			tof_left->getData(result);
			printf("%5u;%5u;%5u;%5u;%5u\n", result.status, result.distance, result.sigPerSPAD, result.ambient, result.numSPADs);
		}
		tof_left->turnOff();
		HAL_Delay(100);
	}

//	if (tof_frontleft)
//	{
//      printf("\n\n=== Testing Front-left TOF ===\n");
//		tof_frontleft->turnOn();
//		HAL_Delay(20);
//		tof_frontleft->init();
//		HAL_Delay(20);
//		tof_frontleft->StartRanging();
//		for (int i=0; i<20; i++)
//		{
//			tof_frontleft->printDistance();
//		}
//		tof_frontleft->turnOff();
//		HAL_Delay(100);
//	}
//
//	if (tof_frontright)
//	{
//      printf("\n\n=== Testing Front-right TOF ===\n");
//		tof_frontright->turnOn();
//		HAL_Delay(20);
//		tof_frontright->init();
//		HAL_Delay(20);
//		tof_frontright->StartRanging();
//		for (int i=0; i<20; i++)
//		{
//			tof_frontright->printDistance();
//		}
//		tof_frontright->turnOff();
//		HAL_Delay(100);
//	}
//
//	if (tof_right)
//	{
//      printf("\n\n=== Testing Right TOF ===\n");
//		tof_right->turnOn();
//		HAL_Delay(20);
//		tof_right->init();
//		HAL_Delay(20);
//		tof_right->StartRanging();
//		for (int i=0; i<20; i++)
//		{
//			tof_right->printDistance();
//		}
//		tof_right->turnOff();
//		HAL_Delay(100);
//	}
#endif

#ifdef TEST_LEDS
	if (leds)
	{
		printf("\n\n=== Testing LEDs ===\n");
		for (int i=0; i<LED_COUNT; i++)
		{
			printf("Turn LED %d: ON\n", i);
			leds->turnOn(i);
		}
		for (int i=0; i<LED_COUNT; i++)
		{
			printf("Turn LED %d: OFF\n", i);
			leds->turnOff(i);
		}
		for (int i=0; i<LED_COUNT; i++)
		{
			printf("Toogle LED %d: ON\n", i);
			leds->toggle(i);
			printf("Toogle LED %d: OFF\n", i);
			leds->toggle(i);
		}
	}
	HAL_Delay(500);
#endif

#ifdef TEST_SWITCHES
	if (switches)
	{
		printf("\n\n=== Testing Switches ===\n");
		for (int i=0; i<SW_COUNT; i++)
		{
			uint8_t sw = switches->get(i);
			printf("Switch %d: %u\n", i, sw);
		}
		uint8_t switches_val = switches->getAll();
		printf("Switches: 0x%02x\n", switches_val);
	}
	HAL_Delay(500);
#endif

#ifdef TEST_BMI
	if(bmi160)
	{
		printf("\n\n=== Testing BMI160 ===\n");
		bmi160->init();
        bmi160->configure();

	    struct bmi160_sensor_data accel_data;
        printf("TimeS;AcceX;AcceY;AcceZ\n");
		for (int i=0; i<10; i++)
	    {
            bmi160->read_accel(accel_data);
            printf("%5lu;%5d;%5d;%5d\n", accel_data.sensortime, accel_data.x, accel_data.y, accel_data.z);
        }

	    struct bmi160_sensor_data gyro_data;
        printf("\nTimeS;GyroX;GyroY;GyroZ\n");
		for (int i=0; i<10; i++)
	    {
            bmi160->read_gyro(gyro_data);
            printf("%5lu;%5d;%5d;%5d\n", gyro_data.sensortime, gyro_data.x, gyro_data.y, gyro_data.z);
        }
 	}
	HAL_Delay(500);
#endif

}
} /* namespace HARDWARE */
