/*
 * error.h
 *
 *  Created on: Jul 27, 2024
 *      Author: agamb
 */

#pragma once

enum error_t
{
    OK = 0,
    FATAL_ERROR,
    GENERIC_ERROR,
	NULL_POINTER,
	DEVICE_NOT_READY,

    NOT_INITALIZED,

    COMMS_ERROR = 100,
    I2C_ERROR,
    SPI_ERROR,
    UART_ERROR,
    BLUETOOTH_ERROR,

    DEVICE_ERROR = 200,
    BMI160_ERROR,
    PCF8574_ERROR,
    TOF_ERROR,
    PMW3360_ERROR,

};
