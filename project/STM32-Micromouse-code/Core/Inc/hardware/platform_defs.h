/*
 * platform.h
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#pragma once

#include "main.h"

#include <stddef.h>
#include <stdint.h>

#define TBD NULL

//typedef GPIO_TypeDef port_ptr_t ;
//
///***********/
///* Sensors */
///***********/
//
//// Mouse sensor (PMW3360)
//constexpr port_ptr_t PMW3360_CS_PORT     = PMW_CS_GPIO_Port;
//constexpr uint16_t   PMW3360_CS_PIN      = PMW_CS_Pin;
//
//constexpr port_ptr_t PMW3360_MOTION_PORT = PMW_MOTION_GPIO_Port;
//constexpr uint16_t   PMW3360_MOTION_PIN  = PMW_MOTION_Pin;
//constexpr port_ptr_t PMW3360_NRESET_PORT = PMW_NRESET_GPIO_Port;
//constexpr uint16_t   PMW3360_NRESET_PIN  = PMW_NRESET_Pin;
//
//// Accelerometer/Gyroscope (BMI160)
//constexpr port_ptr_t BMI160_CS_PORT      = BMI_CS_GPIO_Port;
//constexpr uint16_t   BMI160_CS_PIN       = BMI_CS_Pin;
//
//// Motor Encoders
//constexpr port_ptr_t ENCODERA_C1_PORT    = ENCODER_A_C1_GPIO_Port;
//constexpr uint16_t   ENCODERA_C1_PIN     = ENCODER_A_C1_Pin;
//constexpr port_ptr_t ENCODERA_C2_PORT    = ENCODER_A_C2_GPIO_Port;
//constexpr uint16_t   ENCODERA_C2_PIN     = ENCODER_A_C2_Pin;
//
//constexpr port_ptr_t ENCODERB_C1_PORT    = ENCODER_B_C1_GPIO_Port;
//constexpr uint16_t   ENCODERB_C1_PIN     = ENCODER_B_C1_Pin;
//constexpr port_ptr_t ENCODERB_C2_PORT    = ENCODER_B_C2_GPIO_Port;
//constexpr uint16_t   ENCODERB_C2_PIN     = ENCODER_B_C2_Pin;
//
///***********/
///* Storage */
///***********/
//
//constexpr port_ptr_t FLASH_CS_PORT      = FLASH_CS_GPIO_Port;
//constexpr uint16_t   FLASH_CS_PIN       = FLASH_CS_Pin;
//
//constexpr port_ptr_t BT_STATE_PORT      = BT_STATE_GPIO_Port;
//constexpr uint16_t   BT_STATE_PIN       = BT_STATE_Pin;
//constexpr port_ptr_t BT_ENABLE_PORT     = BT_ENABLE_GPIO_Port;
//constexpr uint16_t   BT_ENABLE_PIN      = BT_ENABLE_Pin;
//
//constexpr port_ptr_t SDCARD_CS_PORT     = SD_CS_GPIO_Port;
//constexpr uint16_t   SDCARD_CS_PIN      = SD_CS_Pin;
//constexpr port_ptr_t SDCARD_DETECT_PORT = SD_DETECT_GPIO_Port;
//constexpr uint16_t   SDCARD_DETECT_PIN  = SD_DETECT_Pin;
//
///*************/
///* Actuators */
///*************/
//
//// Motor Driver (DRV8833)
//constexpr port_ptr_t MOTORA_A1_PORT    = MOTOR_A_1_GPIO_Port;
//constexpr uint16_t   MOTORA_A1_PIN     = MOTOR_A_1_Pin;
//constexpr port_ptr_t MOTORA_A2_PORT    = MOTOR_A_1_GPIO_Port;
//constexpr uint16_t   MOTORA_A2_PIN     = MOTOR_A_2_Pin;
//
//constexpr port_ptr_t MOTORA_B1_PORT    = MOTOR_B_2_GPIO_Port;
//constexpr uint16_t   MOTORA_B1_PIN     = MOTOR_B_2_Pin;
//constexpr port_ptr_t MOTORA_B2_PORT    = MOTOR_B_1_GPIO_Port;
//constexpr uint16_t   MOTORA_B2_PIN     = MOTOR_B_1_Pin;
//
//constexpr port_ptr_t MOTORA_SLEEP_PORT = MOTOR_SLEEP_GPIO_Port;
//constexpr uint16_t   MOTORA_SLEEP_PIN  = MOTOR_SLEEP_Pin;
//constexpr port_ptr_t MOTORA_FAULT_PORT = MOTOR_FAULT_GPIO_Port;
//constexpr uint16_t   MOTORA_FAULT_PIN  = MOTOR_FAULT_Pin;
//
///******************/
///* Communications */
///******************/
//
//// I2C1
//constexpr port_ptr_t I2C1_SCL_PORT         = GPIOB;
//constexpr uint16_t   I2C1_SCL_PIN         = GPIO_PIN_8;
//constexpr port_ptr_t I2C1_SDA_PORT         = GPIOB;
//constexpr uint16_t   I2C1_SDA_PIN         = GPIO_PIN_9;
//
//// I2C2
//constexpr port_ptr_t I2C2_SCL_PORT         = GPIOB;
//constexpr uint16_t   I2C2_SCL_PIN         = GPIO_PIN_10;
//constexpr port_ptr_t I2C2_SDA_PORT         = GPIOB;
//constexpr uint16_t   I2C2_SDA_PIN         = GPIO_PIN_11;
//
//// SPI1
//constexpr port_ptr_t SPI1_CLK_PORT        = GPIOA;
//constexpr uint16_t   SPI1_CLK_PIN         = GPIO_PIN_5;
//constexpr port_ptr_t SPI1_MOSI_PORT       = GPIOA;
//constexpr uint16_t   SPI1_MOSI_PIN        = GPIO_PIN_7;
//constexpr port_ptr_t SPI1_MISO_PORT       = GPIOA;
//constexpr uint16_t   SPI1_MISO_PIN        = GPIO_PIN_6;
//
//// SPI17
//constexpr port_ptr_t SPI2_CLK_PORT         = GPIOB;
//constexpr uint16_t   SPI2_CLK_PIN         = GPIO_PIN_13;
//constexpr port_ptr_t SPI2_MOSI_PORT        = GPIOB;
//constexpr uint16_t   SPI2_MOSI_PIN        = GPIO_PIN_15;
//constexpr port_ptr_t SPI2_MISO_PORT        = GPIOB;
//constexpr uint16_t   SPI2_MISO_PIN        = GPIO_PIN_14;
//
//// USART1
//constexpr port_ptr_t USART1_TX_PORT        = GPIOB;
//constexpr uint16_t   USART1_TX_PIN         = GPIO_PIN_6;
//constexpr port_ptr_t USART1_RX_PORT        = GPIOB;
//constexpr uint16_t   USART1_RX_PIN         = GPIO_PIN_7;
//
//// Programmer (SWDIO)
//constexpr port_ptr_t PROG_SWCLK_PORT       = GPIOA;
//constexpr uint16_t   PROG_SWCLK_PIN        = GPIO_PIN_13;
//constexpr port_ptr_t PROG_SWDIO_PORT       = GPIOA;
//constexpr uint16_t   PROG_SWDIO_PIN        = GPIO_PIN_14;
//constexpr port_ptr_t PROG_SWO_PORT         = GPIOB;
//constexpr uint16_t   PROG_SWO_PIN          = GPIO_PIN_3;
//
//
///******************/
///* GPIO Expanders */
///******************/
//
//// TOF GPIO Expander
//constexpr uint16_t   GPIOEXPANDER_TOF_ID             = 0;
constexpr uint16_t   GPIOEXPANDER_TOF_ADDRESS        = 0x20;

//constexpr port_ptr_t GPIOEXPANDER_TOF_INT_PORT       = TOF_INT_GPIO_Port;
//constexpr uint16_t   GPIOEXPANDER_TOF_INT_PIN        = GPIO_PIN_12;

constexpr uint16_t   GPIOEXPANDER_TOF_TOF0_XSHUT_PIN = 0;
constexpr uint16_t   GPIOEXPANDER_TOF_TOF0_INT_PIN   = 1;
constexpr uint16_t   GPIOEXPANDER_TOF_TOF1_XSHUT_PIN = 2;
constexpr uint16_t   GPIOEXPANDER_TOF_TOF1_INT_PIN   = 3;
constexpr uint16_t   GPIOEXPANDER_TOF_TOF2_XSHUT_PIN = 4;
constexpr uint16_t   GPIOEXPANDER_TOF_TOF2_INT_PIN   = 5;
constexpr uint16_t   GPIOEXPANDER_TOF_TOF3_XSHUT_PIN = 6;
constexpr uint16_t   GPIOEXPANDER_TOF_TOF3_INT_PIN   = 7;
//
//// LED-Switch GPIO Expander
//constexpr uint16_t   GPIOEXPANDER_SWLED_ID           = 1;
constexpr uint16_t   GPIOEXPANDER_SWLED_ADDRESS      = 0x21;
//
//constexpr port_ptr_t GPIOEXPANDER_SWLED_INT_PORT     = nullptr;
//constexpr uint16_t   GPIOEXPANDER_SWLED_INT_PIN      = NULL;
//
//constexpr uint16_t   GPIOEXPANDER_SWLED_LED0_PIN     = 0;
//constexpr uint16_t   GPIOEXPANDER_SWLED_LED1_PIN     = 1;
//constexpr uint16_t   GPIOEXPANDER_SWLED_LED2_PIN     = 2;
//constexpr uint16_t   GPIOEXPANDER_SWLED_LED3_PIN     = 3;
//constexpr uint16_t   GPIOEXPANDER_SWLED_SW0_PIN      = 4;
//constexpr uint16_t   GPIOEXPANDER_SWLED_SW1_PIN      = 5;
//constexpr uint16_t   GPIOEXPANDER_SWLED_SW2_PIN      = 6;
//constexpr uint16_t   GPIOEXPANDER_SWLED_SW3_PIN      = 7;

/***********************/
/* Platform definition */
/***********************/

