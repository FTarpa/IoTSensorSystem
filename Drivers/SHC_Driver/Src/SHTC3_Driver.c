/*
 * SHTC3_Driver.c
 *
 *  Created on: Feb 1, 2021
 *      Author: maitr
 */

#ifndef SHC_LIBRARY_SRC_SHTC3_DRIVER_C_
#define SHC_LIBRARY_SRC_SHTC3_DRIVER_C_
#include "SHTC3_Driver.h"

static uint8_t crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xff;
    uint8_t i = 0, j = 0;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }
    return crc;
}

SHTC3_Status_t SHTC3_Init(SHTC3_Sensor_t *sensor)
{
	if(HAL_I2C_IsDeviceReady(sensor->interface, SHTC3_ADDR, 3, 1000) == HAL_OK)
	{
		sensor->connection=shtc3_connected;
		return SHTC3_OK;
	}
	else
	{
		sensor->connection=shtc3_disconnected;
		return SHTC3_ERROR;
	}
}

SHTC3_Status_t SHTC3_Measurement(SHTC3_Sensor_t *sensor)
{
	uint8_t weekup_cmd[2] = {0x35, 0x17};
	if(HAL_I2C_Master_Transmit(sensor->interface, SHTC3_ADDR, weekup_cmd, sizeof(weekup_cmd), 5000)!=HAL_OK)
		return SHTC3_ERROR;
	uint8_t measure_cmd[2] = {0x35, 0x17};
	if(HAL_I2C_Master_Transmit(sensor->interface, SHTC3_ADDR, measure_cmd, sizeof(measure_cmd), 5000)!=HAL_OK)
			return SHTC3_ERROR;
	while(HAL_I2C_IsDeviceReady(sensor->interface, SHTC3_ADDR, 1, 1000) != HAL_OK);
	uint8_t data_raw[6] = {0};
	if(HAL_I2C_Master_Receive(sensor->interface, SHTC3_ADDR, data_raw, sizeof(data_raw), 5000)!=HAL_OK)
			return SHTC3_ERROR;
	if(crc8(&data_raw[0], 2)!=data_raw[2])
		return SHTC3_ERROR;
	if(crc8(&data_raw[3], 2)!=data_raw[5])
		return SHTC3_ERROR;
	uint8_t sleep_cmd[2] = {0xB0, 0x98};
	if(HAL_I2C_Master_Transmit(sensor->interface, SHTC3_ADDR, sleep_cmd, sizeof(sleep_cmd), 5000)!=HAL_OK)
		return SHTC3_ERROR;
	uint16_t Rh = (data_raw[0]<<8) | (data_raw[1]);
	uint16_t Rt = (data_raw[3]<<8) | (data_raw[4]);
	sensor->data.hum = ((float)(Rh*100))/(65535.0);
	sensor->data.tem = ((float)(Rt*175))/(65535.0) - 45.0;
	return SHTC3_OK;
}
#endif /* SHC_LIBRARY_SRC_SHTC3_DRIVER_C_ */
