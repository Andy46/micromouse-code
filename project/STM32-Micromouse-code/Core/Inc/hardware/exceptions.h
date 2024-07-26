/*
 * exceptions.h
 *
 *  Created on: Jul 26, 2024
 *      Author: agamb
 */

#pragma once

#include <exception>

/** ERROR CODES **/
enum ERROR_CODE
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

class Exception : public std::exception
{
public:
    Exception(const ERROR_CODE code) : code(code) {}
    virtual ~Exception() = default;

    ERROR_CODE getCode() const {return(code);}

    // Override the what() method to return our message
    const char* what() const throw()
    {
        return "";
    }
private:
    ERROR_CODE code;
};

class InitializationException : public Exception
{
public:
    InitializationException(const ERROR_CODE code) : Exception(code) {}
    virtual ~InitializationException() = default;
};

class CriticalException : public Exception
{
public:
    CriticalException(const ERROR_CODE code) : Exception(code) {}
    virtual ~CriticalException() = default;
};

class FunctionalException : public Exception
{
public:
    FunctionalException(const ERROR_CODE code) : Exception(code) {}
    virtual ~FunctionalException() = default;
};

//#ifdef DEBUG
#define ASSERT_DEV_NOT_READY { if (!DEVICE::isReady()) throw CriticalException(ERROR_CODE::DEVICE_NOT_READY); }
// #define ASSERT_NOT_NULL (p) { if (p == nullptr) throw CriticalException(ERROR_CODE::NULL_POINTER); }
//#else
//#define ASSERT_DEV_NOT_READY /* Do nothing */
//#define ASSERT_NOT_NULL (p) /* Do nothing */
//#endif
