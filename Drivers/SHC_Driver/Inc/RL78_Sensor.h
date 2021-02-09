#include "string.h"
#include "main.h"
#define	PING_CMD		0x55
#define WAKEUP_CMD		0xC0
#define GETVALUE_CMD	0xAA	

#define TRY_TIME_CONNECT	3
#define TRY_TIME_GET_NAME	3
typedef enum 
{
	unconnected,
	connected
}Sensor_connect_t;

typedef struct
{
	Sensor_connect_t isConnected;
	uint8_t name[10];
	float value;
	UART_HandleTypeDef *uart_itf;
} Sensor_t;

typedef enum
{
	Sensor_ERROR,
	Sensor_OK
} Sensor_status_t;

#pragma pack(push)
#pragma pack(1) 
typedef struct
{
	uint8_t header;
	uint16_t data;
} data_raw_t;

static Sensor_status_t Sensor_Send_Command(Sensor_t* sensor, uint8_t command);
static Sensor_status_t Sensor_Recv_Respond(Sensor_t* sensor, uint8_t* buff, uint8_t buffSize, uint16_t timeOut);
static float Pare_Data(data_raw_t data_raw);

Sensor_status_t Sensor_Init(Sensor_t* sensor);
Sensor_status_t Sensor_Get_Value(Sensor_t* sensor);
