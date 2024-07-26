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

/**
 * Platform Factory
 *
 * Factory class to create a platform object
 */
class PlatformFactory final
{
private:
	PlatformFactory() = delete;
	~PlatformFactory() = delete;

public:

	/**
	 * Return the platform object, shall be called only once
	 */
	static Platform getPlatform();
};

} /* namespace HARDWARE */
