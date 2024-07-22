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
					 std::shared_ptr<EXTRA::LEDS> leds,
					 std::shared_ptr<EXTRA::Switches> switches
				   ) :
		tof_left(tof_left), tof_frontleft(tof_frontleft),
		tof_frontright(tof_frontright), tof_right(tof_right),
		leds(leds), switches(switches)
{

}

void Platform::init()
{

}

void Platform::run_test()
{
#if DEBUG
//	tof_left->turnOff();
//	tof_frontleft->turnOff();
//	tof_frontright->turnOff();
//	tof_right->turnOff();
//	HAL_Delay(10);

	if (tof_left)
	{
		printf("Left TOF!\n");
		tof_left->turnOn();
		HAL_Delay(20);
		tof_left->init();
		HAL_Delay(20);
		tof_left->StartRanging();
		for (int i=0; i<20; i++)
		{
			tof_left->printDistance();
		}
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

//	if (leds)
//	{
//		for (int i=0; i<LED_COUNT; i++)
//		{
//			printf("Turn LED %d: ON\n", i);
//			leds->turnOn(i);
//		}
//		for (int i=0; i<LED_COUNT; i++)
//		{
//			printf("Turn LED %d: OFF\n", i);
//			leds->turnOff(i);
//		}
//		for (int i=0; i<LED_COUNT; i++)
//		{
//			printf("Toogle LED %d: ON\n", i);
//			leds->toggle(i);
//			printf("Toogle LED %d: OFF\n", i);
//			leds->toggle(i);
//		}
//	}
//	HAL_Delay(500);

//	if (switches)
//	{
//		for (int i=0; i<SW_COUNT; i++)
//		{
//			uint8_t sw = switches->get(i);
////			printf("Switch %d: %u\n", i, sw);
//		}
//		uint8_t switches_val = switches->getAll();
////		printf("Switches: 0x%02x\n", switches_val);
//	}
//	HAL_Delay(500);

#endif
}
} /* namespace HARDWARE */
