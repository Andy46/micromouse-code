/*
 * bmi160.cpp
 *
 *  Created on: Jul 17, 2024
 *      Author: agamb
 */

#include "hardware/sensors/bmi160.h"

/* C/C++ libraries */
#include <stdio.h>

#define BMI160_CS_PORT GPIOB
#define BMI160_CS_PIN  GPIO_PIN_1

#undef DEBUG

namespace
{
constexpr int CALIBRATION_READS = 1000;

#define BMI_COUNT 1
struct bmi_cb_data_t
{
	std::shared_ptr<HARDWARE::COMMS::SPI> spi;
	std::shared_ptr<HARDWARE::EXTRA::GPIO> cs;
};
struct bmi_cb_data_t bmi_cbs [BMI_COUNT];
static uint8_t bmi_created_count = 0;

// I2C Callbacks
int8_t bmi160spi_read_cb (uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
#ifdef DEBUG
    printf("Reading from SPI!\n");
    printf("dev: 0x%x\n", dev_addr);
    printf("reg: 0x%x\n", reg_addr);
    printf("Len: %d\n", len);
#endif
    // Activate SPI chip select line
//    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_RESET);
    bmi_cbs[dev_addr].cs->clear();

    // Send command
//    HAL_SPI_Transmit(&hspi1, &reg_addr, sizeof(reg_addr), 1000);
    bmi_cbs[dev_addr].spi->send(&reg_addr, sizeof(reg_addr), 1000);

    // Receive data
//    HAL_SPI_Receive(&hspi1, read_data, len, 1000);
    bmi_cbs[dev_addr].spi->receive(read_data, len, 1000);

    // Deactivate SPI chip select line
//    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_SET);
    bmi_cbs[dev_addr].cs->set();

    return 0;
}

int8_t bmi160spi_write_cb (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
#ifdef DEBUG
    printf("Writing to SPI!\n");
    printf("dev: 0x%x\n", dev_addr);
    printf("reg: 0x%x\n", reg_addr);
    printf("Len: %d\n", len);
#endif

    // Compose transmission buffer (address + data)
    uint8_t buffer[len+1];
    buffer[0] = reg_addr;
    memcpy(&(buffer[1]), data, len);

    // Activate SPI chip select line
    bmi_cbs[dev_addr].cs->clear();
//    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_RESET);

    // Send buffer
//    HAL_SPI_Transmit(&hspi1, buffer, sizeof(buffer), 1000);
    bmi_cbs[dev_addr].spi->send(buffer, sizeof(buffer), 1000);

    // Deactivate SPI chip select line
//    HAL_GPIO_WritePin(BMI160_CS_PORT, BMI160_CS_PIN, GPIO_PIN_SET);
    bmi_cbs[dev_addr].cs->set();

    return 0;
}

void bmi160spi_delay_ms_cb (uint32_t period_ms)
{
    HAL_Delay(period_ms);
}


}

namespace HARDWARE::SENSORS
{

//BMI160::BMI160(SPI_HandleTypeDef* spi, GPIO_TypeDef * gpio, uint16_t pin)
BMI160::BMI160(std::shared_ptr<COMMS::SPI> spi, std::shared_ptr<EXTRA::GPIO> cs) :
		spi(spi), cs(cs)
{
	// ID is used to identify device in callback functions
    bmi160dev.id = bmi_created_count;

    /* Set interface address spi address */
    bmi160dev.intf = BMI160_SPI_INTF;

    // Set callback parameters
    bmi_cbs[bmi160dev.id].cs  = cs;
    bmi_cbs[bmi160dev.id].spi = spi;

    // Set communication callback functions for BMI160 library
    bmi160dev.read     = bmi160spi_read_cb;
    bmi160dev.write    = bmi160spi_write_cb;
    bmi160dev.delay_ms = bmi160spi_delay_ms_cb;

    // Set SPI chip select line to unselect
    cs->set();

    bmi_created_count++;
}

int8_t BMI160::init()
{
	int8_t status = 0;

    // Initialize sensor using library
    status = bmi160_init(&bmi160dev);
    if (status == BMI160_OK)
    {
        printf("BMI160 initialized!\n");
        printf("Chip ID 0x%x\n", bmi160dev.chip_id);
    }
    else
    {
    	printf("Error initializing BMI160!\n");
        status = 1;
        return status;
    }
//
//	/* Calibrate */
//
//	int64_t sum = 0;
//	struct bmi160_sensor_data gyro_data;
//
//	for (int i=0; i<CALIBRATION_READS; i++)
//	{
//		read_gyro(&gyro_data);
//		sum += gyro_data.z;
//		printf("%d - %lld\n",gyro_data.z, sum);
//		HAL_Delay(10);
//	}
//
//	calibrationZ = sum / CALIBRATION_READS;

	/* Configure */

    /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev.accel_cfg.odr   = BMI160_ACCEL_ODR_100HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    bmi160dev.accel_cfg.bw    = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev.gyro_cfg.odr   = BMI160_GYRO_ODR_1600HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160dev.gyro_cfg.bw    = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    status = bmi160_set_sens_conf(&bmi160dev);
    if (status == BMI160_OK)
    {
    	printf("BMI160 configured!\n");
    }
    else
    {
    	printf("Error configuring BMI160!");
        status = -1;
    }
    return status;
}


int8_t BMI160::read_accel(struct bmi160_sensor_data* accel_data)
{
	if (accel_data == NULL)
	{
		return -1;
	}

    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_TIME_SEL), accel_data, NULL, &bmi160dev);
    return 0;
}

int8_t BMI160::read_gyro(struct bmi160_sensor_data* gyro_data)
{
	if (gyro_data == NULL)
	{
		return -1;
	}

    bmi160_get_sensor_data((BMI160_GYRO_SEL | BMI160_TIME_SEL), NULL, gyro_data, &bmi160dev);
    return 0;
}

int8_t BMI160::read_all(struct bmi160_sensor_data* accel_data, struct bmi160_sensor_data* gyro_data)
{
	if (accel_data == NULL || gyro_data == NULL)
	{
		return -1;
	}

    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), accel_data, gyro_data, &bmi160dev);
    return 0;
}

} /* namespace HARDWARE::SENSORS */
