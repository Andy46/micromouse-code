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

//#define TEST_TOFS
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
		printf("Left TOF!\n");
		tof_left->turnOn();
		HAL_Delay(20);
		tof_left->init();
		HAL_Delay(20);
		tof_left->StartRanging();

//		SENSORS::TOF_VL53L1X::Result_t result;
//		for (int i=0; i<20; i++)
//		{
//			tof_left->getData(result);
//			printf("%u, %u, %u, %u, %u\n", result.status, result.distance, result.sigPerSPAD, result.ambient, result.numSPADs);
//		}
		tof_left->turnOff();
		HAL_Delay(100);
	}

//	if (tof_frontleft)
//	{
//		printf("Front-left TOF!\n");
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
//		printf("Front-right TOF!\n");
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
//		printf("Right TOF!\n");
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
		printf("Testing BMI160!\n");
		bmi160->init();
        bmi160->configure();

	    struct bmi160_sensor_data gyro_data;

	    int64_t cal_sum = 0;
	    for (int i = 0; i < 100; i++)
	    {
	    	bmi160->read_gyro(&gyro_data);
	    	cal_sum += gyro_data.z;
	    }

	    int16_t calZ = cal_sum / 100;

	    printf("Calibration is: %d\n", calZ);

	    uint32_t lastTime;
	    uint32_t firstRun = 1;
	    float x_deg = 0;
	    constexpr int SENSITIVITY = 50;
	    while (1)
	    {
	        for (int i = 0; i < SENSITIVITY; i++)
	        {
	        	bmi160->read_gyro(&gyro_data);
	//            printf("GYRO:%-5d;%-5d;%-5d\n", gyro_data.x, gyro_data.y, gyro_data.z-calZ);
	//            printf("Time: %lu\n", gyro_data.sensortime);
	            if (firstRun == 1)
	            {
	                lastTime = gyro_data.sensortime;
	                firstRun = 0;
	            }
	//            printf("Delta : %lu", gyro_data.sensortime -lastTime);
	            if (abs(gyro_data.z) > 15)
	            {
	                x_deg += (gyro_data.z) * 0.061 * ((gyro_data.sensortime - lastTime) * 0.000039f);
	            }
	            lastTime = gyro_data.sensortime;
	        }
	        printf("%5d - %3.3f\n", gyro_data.z, x_deg);
	    }

	}
	HAL_Delay(500);
#endif

}
} /* namespace HARDWARE */
