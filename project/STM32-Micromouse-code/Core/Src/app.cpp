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
HARDWARE::Platform platform = HARDWARE::PlatformFactory::getPlatform();

void setup();

extern "C"
int app_main() noexcept
{
    try
    {
        printf("Starting app!\n");

        // Run tests over platform
        platform.run_test();


        // Application logic
        //	platform.init();
		//  setup();
        //  TBD
    }
    catch(const InitializationException& e)
    {
        printf("Captured initialization error: %d\n", e.getCode());
    }
    catch(const Exception& e)
    {
        printf("Captured exception: %d\n", e.getCode());
    }
    catch(...)
    {
        printf("Unhandled exception!\n");
    }

    printf("Application ended!\n");
	return 0;
}

void setup()
{

}
