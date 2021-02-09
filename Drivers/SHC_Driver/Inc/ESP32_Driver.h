/*
 * ESP32_Driver.h
 *
 *  Created on: Jan 29, 2021
 *      Author: maitr
 */

#ifndef SHC_LIBRARY_INC_ESP32_DRIVER_H_
#define SHC_LIBRARY_INC_ESP32_DRIVER_H_
#include "stdint.h"
#include "string.h"
#include "stdio.h"

#define ESP_DEBUG printf("[ESP]: ");printf
#define ESP_DEBUG_RESULT printf
#define BUFFER_SIZE 512
#define TIME_OUT 0xffff
typedef int16_t (*Receive_Func_t)( uint8_t *, uint16_t , uint32_t);
typedef int16_t (*Transmit_Func_t)( uint8_t *, uint16_t);


typedef struct {
  Receive_Func_t    IO_Receive;
  Transmit_Func_t	IO_Transmit;
  uint8_t buffer[BUFFER_SIZE];
  uint16_t read_pos;
  uint16_t write_pos;
} Network_IO_t;

typedef struct
{
	char SSID[32];
	char Pass[32];
	char Connected;
	Network_IO_t IO;
}Network_t;

typedef enum
{
	NETWORK_OK,
	NETWORK_ERROR,
	NETWORK_TIMEOUT
}Network_Status_t;
typedef struct{
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;

}Network_time_t ;


static Network_Status_t ESP32_SendCommand(uint8_t* cmd);

Network_Status_t ESP32_Init(Network_t *network);
Network_Status_t ESP32_MQTT_Connect(Network_t *network, uint8_t* client_id);
Network_Status_t ESP32_MQTT_Public(Network_t *network, uint8_t* topic, uint8_t* message);
Network_time_t ESP32_GetTime();
#endif /* SHC_LIBRARY_INC_ESP32_DRIVER_H_ */
