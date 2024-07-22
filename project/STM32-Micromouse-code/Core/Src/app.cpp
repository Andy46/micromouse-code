/*
 * app.cpp
 *
 *  Created on: Jul 20, 2024
 *      Author: Andy46
 */

#include <stdio.h>

#include "main.h"

#include "hardware/platform.h"
#include "hardware/platform_factory.h"

/* Hardware definition */


void setup();

extern "C"
int app_main()
{
	printf("Starting app!\n");

	HARDWARE::Platform platform = HARDWARE::PlatformFactory::CreatePlatform();

	// Initialize platform's hardware
    setup();
//	platform.init();

	// Application logic
	while(1)
	{
		printf("Start loop!\n");
#if DEBUG
		platform.run_test();
#endif
		printf("End loop!\n");




	}

	return -1;
}

void setup()
{

}
