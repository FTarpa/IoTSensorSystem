/*
 * SHTC3_Driver.h
 *
 *  Created on: Feb 1, 2021
 *      Author: maitr
 */

#ifndef SHC_LIBRARY_INC_SHTC3_DRIVER_H_
#define SHC_LIBRARY_INC_SHTC3_DRIVER_H_

#include "stdint.h"
#include "main.h"
#define SHTC3_ADDR (0x45<<1)
#pragma pack(push)
#pragma pack(1)
typedef struct
{
	float tem;
	float hum;
} SHTC3_data_t;
typedef enum
{
	shtc3_disconnected,
	shtc3_connected
}SHTC3_Connected_t;
#pragma pack(push)
#pragma pack(1)
typedef struct
{
	I2C_HandleTypeDef* interface;
	SHTC3_Connected_t connection;
	SHTC3_data_t data;

}SHTC3_Sensor_t;
typedef enum
{
	SHTC3_OK,
	SHTC3_ERROR
}SHTC3_Status_t;
static uint8_t crc8(uint8_t *data, uint8_t len);
SHTC3_Status_t SHTC3_Init(SHTC3_Sensor_t *sensor);
SHTC3_Status_t SHTC3_Measurement(SHTC3_Sensor_t *sensor);


#endif /* SHC_LIBRARY_INC_SHTC3_DRIVER_H_ */
