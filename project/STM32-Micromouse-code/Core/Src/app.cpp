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

void setup();

extern "C"
int app_main()
{
	printf("Starting app!\n");

	/* Hardware definition */
	HARDWARE::Platform platform = HARDWARE::PlatformFactory::getPlatform();

	platform.init();
	platform.checkStatus();

	// Run tests over platform
	platform.run_test();

	// Application logic
	//  setup();
	//  TBD

    printf("Application ended!\n");
	return 0;
}

void setup()
{

}
