/*
 * ESP32_Driver.c
 *
 *  Created on: Jan 29, 2021
 *      Author: maitr
 */
#include "ESP32_Driver.h"

Network_t *Network = {0};


static Network_Status_t ESP32_SendCommand(uint8_t* cmd)
{

	int16_t res = Network->IO.IO_Transmit(cmd, strlen((char*)cmd));
	uint8_t recv_buff[BUFFER_SIZE] = {0};

	if(res >=0 )
	{
		res = Network->IO.IO_Receive(recv_buff, BUFFER_SIZE, TIME_OUT);
		if(res >=0 )
		{
			//ESP_DEBUG(recv_buff);
			return NETWORK_OK;
		}
		else
		{
			ESP_DEBUG_RESULT("ERROR.!!!");
			ESP_DEBUG(recv_buff);
			return NETWORK_ERROR;
		}

	}
	return NETWORK_OK;
}


Network_Status_t ESP32_Init(Network_t *network)
{
	Network = network;

	ESP32_SendCommand((uint8_t*)"AT+RST\r\n"); 						//Reset ESP32
	ESP32_SendCommand((uint8_t*)"ATE0\r\n"); 						//turn off echo
	Network_Status_t res = ESP32_SendCommand((uint8_t*)"AT\r\n");	//test with AT command
	if(res == NETWORK_OK )
	{
		ESP_DEBUG("Set Station mode\r\n");
		if(ESP32_SendCommand((uint8_t*)"AT+CWMODE=1\r\n")>=0)
		{
			ESP_DEBUG("Connect wifi with ssid: %s, pass: %s\r\n",  Network->SSID, Network->Pass);
			uint8_t cmd[64] = {0};
			sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", Network->SSID, Network->Pass);
			if(ESP32_SendCommand(cmd)<0)
				return NETWORK_ERROR;
		}
		else
		{
			return NETWORK_ERROR;
		}

		ESP_DEBUG("Config SSL\r\n");
		if(ESP32_SendCommand("AT+CIPSSLCCONF=3,0,0\r\n")<0)
		{
			return NETWORK_ERROR;
		}

		ESP_DEBUG("Config mode\r\n");
		if(ESP32_SendCommand("AT+CWMODE=1\r\n")<0)
		{
			return NETWORK_ERROR;
		}

		ESP_DEBUG("Config time server\r\n");
		if(ESP32_SendCommand("AT+CIPSNTPCFG=1,0,\"sg.pool.ntp.org\"\r\n")<0)
		{
			return NETWORK_ERROR;
		}

	}
	return NETWORK_OK;
}

Network_Status_t ESP32_MQTT_Connect(Network_t *network, uint8_t* client_id)
{
	uint8_t esp_cmd[128] = {0};
	ESP_DEBUG("Set TLS: ");
	sprintf(esp_cmd, "AT+MQTTUSERCFG=0,4,\"%s\",\"espressif\",\"1234567890\",0,0,\"\"\r\n", client_id);
	if(ESP32_SendCommand(esp_cmd)<0)
	{
		ESP_DEBUG_RESULT("ERROR.!");
		return NETWORK_ERROR;
	}
	ESP_DEBUG_RESULT("OK\r\n");
	ESP_DEBUG("Connect to IoT AWS: ");
//	if(ESP32_SendCommand("AT+MQTTCONN=0,\"as76wtq33csyg-ats.iot.ap-southeast-1.amazonaws.com\",8883,1\r\n")<0)
	if(ESP32_SendCommand("AT+MQTTCONN=0,\"a2m7a41bhehfuw-ats.iot.us-east-2.amazonaws.com\",8883,1\r\n")<0)
	{
		ESP_DEBUG_RESULT("ERROR.!");
		return NETWORK_ERROR;
	}
	ESP_DEBUG_RESULT("OK\r\n");
	return NETWORK_OK;
}

Network_Status_t ESP32_MQTT_Public(Network_t *network, uint8_t* topic, uint8_t* message)
{
	uint8_t esp_cmd[1024] = {0};

	//sprintf(esp_cmd, "AT+MQTTPUB=0,\"%s\",\"%s\",1,0\r\n", topic, message);
	sprintf(esp_cmd, "AT+MQTTPUBRAW=0,\"%s\",%d,1,0\r\n", topic, strlen(message));
	ESP_DEBUG("[PUBLIC]: %s\r\n", esp_cmd);

	if(ESP32_SendCommand(esp_cmd)<0)
	{
		ESP_DEBUG_RESULT("ERROR.!\r\n");
		return NETWORK_ERROR;
	}
//	ESP_DEBUG_RESULT("OK\r\n");

	ESP_DEBUG("[PUBLIC]: topic: %s, message: %s\r\n", topic, message);

	if(ESP32_SendCommand(message)<0)
	{
		ESP_DEBUG_RESULT("ERROR.!\r\n");
		return NETWORK_ERROR;
	}
//	ESP_DEBUG_RESULT("OK\r\n");

	return NETWORK_OK;
}
static void standard_string_time(uint8_t* time_str)
{
	int index = 0;

	do
	{
		if(time_str[index] == 0)
		{
			break;
		}
		else if ((time_str[index] < '0')||(time_str[index] > '9'))
		{
			time_str[index] = '0';
		}
		index++;
	}while(1);

}
static uint8_t month_encode(uint8_t* month_str)
{
	if(strstr((char*)(char*)month_str, "Jan"))
		return 1;
	else if(strstr((char*)month_str, "Feb"))
		return 2;
	else if(strstr((char*)month_str, "Mar"))
		return 3;
	else if(strstr((char*)month_str, "Apr"))
		return 4;
	else if(strstr((char*)month_str, "May"))
		return 5;
	else if(strstr((char*)month_str, "Jun"))
		return 6;
	else if(strstr((char*)month_str, "Jul"))
		return 7;
	else if(strstr((char*)month_str, "Aug"))
		return 8;
	else if(strstr((char*)month_str, "Sep"))
		return 9;
	else if(strstr((char*)month_str, "Oct"))
		return 10;
	else if(strstr((char*)month_str, "Nov"))
		return 11;
	else if(strstr((char*)month_str, "Dec"))
		return 12;
	else
		return 0;
}

Network_time_t ESP32_GetTime()
{
	uint8_t esp_cmd[256] = {0};
	Network_time_t result = {0};
	sprintf((char*)esp_cmd, "AT+CIPSNTPTIME?\r\n");
	int16_t res = Network->IO.IO_Transmit(esp_cmd, strlen((char*)esp_cmd));
	uint8_t recv_buff[BUFFER_SIZE] = {0};

	if(res >=0 )
	{
		res = Network->IO.IO_Receive(recv_buff, BUFFER_SIZE, TIME_OUT);
		if(res >=0 )
		{
			//ESP_DEBUG(recv_buff);
		}
		else
		{
			ESP_DEBUG_RESULT("ERROR.!!!");
			return result;
		}
	}
	char* time_str = strstr((char*)recv_buff, "+CIPSNTPTIME:");
	uint8_t day_str[3] 	= {0};
	uint8_t month_str[4]= {0};
	uint8_t year_str[5] = {0};

	uint8_t hour_str[3] = {0};
	uint8_t min_str[3] 	= {0};
	uint8_t sec_str[3]	= {0};

	memcpy(hour_str, time_str+24, 2);
	memcpy(min_str, time_str+27, 2);
	memcpy(sec_str, time_str+30, 2);

	memcpy(day_str, time_str+21, 2);
	memcpy(month_str, time_str+17, 3);
	memcpy(year_str, time_str+33, 4);

	standard_string_time(hour_str);
	standard_string_time(min_str);
	standard_string_time(sec_str);

	standard_string_time(day_str);
	standard_string_time(year_str);

	uint8_t day_num = 0;
	uint8_t month_num = 0;
	uint16_t year_num = 0;

	uint8_t hour_num = 0;
	uint8_t min_num = 0;
	uint8_t sec_num = 0;

	day_num = atoi((char*)day_str);
	month_num = month_encode(month_str);
	year_num = atoi(year_str);

	hour_num = atoi(hour_str);
	min_num = atoi(min_str);
	sec_num = atoi(sec_str);
	ESP_DEBUG("TIME: %d/%d/%d, %d:%d:%d\r\n", day_num, month_num, year_num, hour_num, min_num, sec_num);

	result.year		=	year_num;
	result.month	=	month_num;
	result.day		= 	day_num;

	result.hour		=	hour_num;
	result.min		=	min_num;
	result.sec		=	sec_num;
	return result;
}
