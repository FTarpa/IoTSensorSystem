/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ESP32_Driver.h"
#include "RL78_Sensor.h"
#include "SHTC3_Driver.h"

#include "string.h"
#include "stdio.h"
#include "time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _SSID "aterm-460e2a-g"
#define _PASS "173ecaa68d508"
#define PRODUCT_TYPE	"IoTSensorUnit01"
#define NUMBER			"2101-1234567890"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
volatile Network_t ESP32 = {0};
Sensor_t Sensor_1 = {0};
Sensor_t Sensor_2 = {0};
Sensor_t Sensor_3 = {0};
Sensor_t Sensor_4 = {0};

SHTC3_Sensor_t HT_Sensor = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
int16_t Receive_Func( uint8_t *buffer, uint16_t buffer_size, uint32_t time_out);
int16_t Transmit_Func( uint8_t *data, uint16_t data_size);
void Get_Time_Format(uint8_t* time_str, int size ,int timeZone);
void Format_Data(Sensor_t Sensor_1,Sensor_t Sensor_2,Sensor_t Sensor_3,Sensor_t Sensor_4, SHTC3_Sensor_t HT_Sensor,char* data_format, char* time_str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart3);
  memcpy(ESP32.SSID, _SSID, strlen(_SSID));
  memcpy(ESP32.Pass, _PASS, strlen(_PASS));
  ESP32.IO.IO_Receive = Receive_Func;
  ESP32.IO.IO_Transmit = Transmit_Func;

  printf("----------START PROGRAM----------\r\n");
  Sensor_1.uart_itf = &huart4;
  Sensor_Init(&Sensor_1);

  Sensor_2.uart_itf = &huart5;
  Sensor_Init(&Sensor_2);

  Sensor_3.uart_itf = &huart7;
  Sensor_Init(&Sensor_3);

  Sensor_4.uart_itf = &huart8;
  Sensor_Init(&Sensor_4);

  HT_Sensor.interface = &hi2c1;
  SHTC3_Init(&HT_Sensor);

  ESP32_Init(&ESP32);
  ESP32_MQTT_Connect(&ESP32, "client_id");
  Network_time_t network_time = ESP32_GetTime();

  RTC_TimeTypeDef sTime = {0};
  sTime.Hours	= network_time.hour;
  sTime.Minutes	= network_time.min;
  sTime.Seconds	= network_time.sec;

  RTC_DateTypeDef sDate = {0};
  sDate.Month	= network_time.month;
  sDate.Date	= network_time.day;
  sDate.Year	= network_time.year - 2000;

  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t time_str[128] = {0};
	  Get_Time_Format(time_str, sizeof(time_str), 9);

	  Sensor_Get_Value(&Sensor_1);
	  Sensor_Get_Value(&Sensor_2);
	  Sensor_Get_Value(&Sensor_3);
	  Sensor_Get_Value(&Sensor_4);
	  SHTC3_Measurement(&HT_Sensor);

	  char data2send[512] = {0};
	  Format_Data(Sensor_1, Sensor_2, Sensor_3, Sensor_4, HT_Sensor, data2send, time_str);
	  ESP32_MQTT_Public(&ESP32, "IoTSensorUnit", data2send);

	  HAL_Delay(60000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 38400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 38400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 38400;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 38400;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
//  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  HAL_UART_Receive_DMA(&huart2,  ESP32.IO.buffer, BUFFER_SIZE);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

int16_t Receive_Func( uint8_t *buffer, uint16_t buffer_size, uint32_t time_out)
{
	uint32_t timeout = 0;
	memset(buffer, 0, buffer_size);
	uint16_t read_index = 0;
	do
	{
		uint8_t check_buff[BUFFER_SIZE] = {0};
		uint8_t cnt=0;
		read_index = ESP32.IO.read_pos;

		do
		{
			check_buff[cnt++] = ESP32.IO.buffer[read_index];
			if(++read_index>=BUFFER_SIZE)
				read_index = 0;
		}while(ESP32.IO.buffer[read_index] != 0);

		//if((strstr((char*)ESP32.IO.buffer + ESP32.IO.read_pos, "\r\nOK\r\n")!=0)||(strstr((char*)ESP32.IO.buffer + ESP32.IO.read_pos, "\r\nERROR\r\n")!=0))
		if((strstr((char*)check_buff, "\r\nOK")!=0)||(strstr((char*)check_buff, "ERROR")!=0)
				||(strstr((char*)check_buff, "ready")!=0)||(strstr((char*)check_buff, "+MQTTPUB:OK")!=0))
		{
			uint8_t index = 0;
			memcpy(buffer, check_buff, cnt);
			for(int i = 0; i < cnt; i++)
			{
				int temp = ((ESP32.IO.read_pos + i)<BUFFER_SIZE)?(ESP32.IO.read_pos + i):((ESP32.IO.read_pos + i)-BUFFER_SIZE);
				ESP32.IO.buffer[temp] = 0;
			}
			printf("[debug]: %s--position: %d\r\n", buffer, ESP32.IO.read_pos);
			ESP32.IO.read_pos = read_index;
			return index;
		}

		if(++timeout >= time_out)
			return -1;
		HAL_Delay(1);
	}while(1);
}

int16_t Transmit_Func( uint8_t *data, uint16_t data_size)
{
	HAL_StatusTypeDef res = HAL_UART_Transmit(&huart2, data, data_size, HAL_MAX_DELAY);
	if(res != HAL_OK)
		return -1;
	else
		return 0;

}

void Get_Time_Format(uint8_t* time_str, int size ,int timeZone)
{
	RTC_TimeTypeDef stime = {0};
	RTC_DateTypeDef sdate = {0};
	HAL_RTC_GetTime(&hrtc, &stime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdate, RTC_FORMAT_BIN);

	struct tm proc_time = {0};

	proc_time.tm_year = 2000 + sdate.Year - 1900;
	proc_time.tm_mon = sdate.Month - 1;
	proc_time.tm_mday = sdate.Date;
	proc_time.tm_hour = stime.Hours;
	proc_time.tm_min = stime.Minutes;
	proc_time.tm_sec = stime.Seconds;

	mktime(&proc_time);
	proc_time.tm_hour += timeZone;
	mktime(&proc_time);
	//2021-01-12 12:00:00+09
	memset(time_str, 0, size);
	sprintf(time_str, "%.4d-%.2d-%.2d %.2d:%.2d:%.2d+%.2d:00", (int)proc_time.tm_year + 1900, (int)proc_time.tm_mon + 1
	, (int)proc_time.tm_mday, (int)proc_time.tm_hour, (int)proc_time.tm_min, (int)proc_time.tm_sec, timeZone);
}

void Format_Data(Sensor_t Sensor_1,Sensor_t Sensor_2,Sensor_t Sensor_3,Sensor_t Sensor_4, SHTC3_Sensor_t HT_Sensor,char* data_format, char* time_str)
{
	Sensor_t Sensors[4] = {Sensor_1, Sensor_2, Sensor_3, Sensor_4};
	char* sensor_value[256] = {0};
	if(HT_Sensor.connection == shtc3_connected)
	{
		sprintf(sensor_value, "%s\t\t\"%s\":%.1f,\r\n", sensor_value, "Temp",HT_Sensor.data.tem);
		sprintf(sensor_value, "%s\t\t\"%s\":%.1f,\r\n", sensor_value, "Rh",HT_Sensor.data.hum);
	}

	for(int i = 0; i < 4; i++)
	{
		if(Sensors[i].isConnected == connected)
		{
			sprintf(sensor_value, "%s\t\t\"%s\":%.1f", sensor_value, Sensors[i].name, Sensors[i].value);
			if(i<(4-1))
				sprintf(sensor_value,"%s,\r\n", sensor_value);
		}
	}

	sprintf(data_format, "{\r\n\t\"time\":\"%s\",\r\n\t\"product type\":\"%s\",\r\n\t\"number\":\"%s\",\r\n\t\"value\":\r\n\t{\r\n%s\r\n\t}\r\n}",
			time_str, PRODUCT_TYPE, NUMBER, sensor_value);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
