/*
 * app.cpp
 *
 *  Created on: Jul 20, 2024
 *      Author: Andy46
 */

/* Hardware definition */
//
// #include "hardware/platform.h"
// #include "hardware/platform_factory.h"


// HARDWARE::Platform platform = HARDWARE::PlatformFactory::CreatePlatform();




//HARDWARE::COMMS::
//HARDWARE::Platform platform = HAR;

void setup();

extern "C"
int app_main()
{


	// Initialize platform's hardware
    setup();
//	platform.init();

	// Application logic
	while(1)
	{




	}

	return -1;
}

void setup()
{

}
