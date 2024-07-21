/*
 * platform_factory.h
 *
 *  Created on: Jul 20, 2024
 *      Author: agamb
 */

#pragma once

#include "platform.h"

namespace HARDWARE
{

class PlatformFactory
{
private:
	PlatformFactory() = delete;
	~PlatformFactory() = delete;

public:

	static Platform CreatePlatform();
};

} /* namespace HARDWARE */
