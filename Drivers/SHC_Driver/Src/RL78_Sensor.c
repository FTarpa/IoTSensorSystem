#include "RL78_Sensor.h"

Sensor_status_t Sensor_Init(Sensor_t* sensor)
{
	uint8_t temp_buff[12] = {0};
	if(Sensor_Send_Command(sensor, PING_CMD)!=Sensor_OK)
		return Sensor_ERROR;
	uint8_t try_get_name = 0;
	do
	{
		uint8_t try_connect = 0;
		while(Sensor_Recv_Respond(sensor,temp_buff, sizeof(temp_buff), 100)!=Sensor_OK)
		{
			if(++try_connect>=TRY_TIME_CONNECT)
			{
				sensor->isConnected = unconnected;
				return Sensor_ERROR;
			}
		}
		if((temp_buff[1] <= strlen(&temp_buff[2]))&&(temp_buff[1] <= 10)&&(temp_buff[1] > 0))
		{
			for(int i = 0; i < temp_buff[1]; i++)
			{
				sensor->name[i] = temp_buff[2+i];
			}
			printf("[Sensor]:name: %s\r\n", sensor->name);
			sensor->isConnected = connected;
			return Sensor_OK;
		}
	}while(++try_get_name < TRY_TIME_GET_NAME);
	
	sensor->isConnected = unconnected;
	return Sensor_ERROR;
}

Sensor_status_t Sensor_Get_Value(Sensor_t* sensor)
{
	if(sensor->isConnected == connected)
	{
		if(Sensor_Send_Command(sensor, WAKEUP_CMD)!= Sensor_OK)
				return Sensor_ERROR;
			uint8_t temp = 0;
			if(Sensor_Recv_Respond(sensor,&temp, sizeof(temp), 60000)!= Sensor_OK)
				return Sensor_ERROR;

			if(Sensor_Send_Command(sensor, GETVALUE_CMD)!= Sensor_OK)
				return Sensor_ERROR;
			data_raw_t data_raw = {0};
			if(Sensor_Recv_Respond(sensor, &data_raw, sizeof(data_raw), 100) != Sensor_OK)
				return Sensor_ERROR;
			sensor->value = Pare_Data(data_raw);
			return Sensor_OK;
	}
	else
	{
		return Sensor_ERROR;
	}
	
}

static float Pare_Data(data_raw_t data_raw)
{
	float result = 0.0;
	uint16_t value_interger = 0;
	uint8_t value_decimal = 0;
	
	value_interger	= (data_raw.data>>4)&(0x0fff);
	value_decimal	= (data_raw.data)&(0x0f);
	
	result = (float)value_interger + (float)value_decimal/(10.0);
	
	return result;
}

static Sensor_status_t Sensor_Send_Command(Sensor_t* sensor, uint8_t command)
{
	if(HAL_UART_Transmit(sensor->uart_itf, &command, 1, HAL_MAX_DELAY) == HAL_OK)
		return Sensor_OK;
	else
		return Sensor_ERROR;
}

static Sensor_status_t Sensor_Recv_Respond(Sensor_t* sensor, uint8_t* buff, uint8_t buffSize, uint16_t timeOut)
{
	HAL_StatusTypeDef res = HAL_UART_Receive(sensor->uart_itf, buff, buffSize, timeOut);

	if(res != HAL_ERROR)
		return Sensor_OK;
	else
		return Sensor_ERROR;

}
