/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef sTime;
RTC_DateTypeDef DateToUpdate;
uint8_t RX_data;
uint8_t TX_BUFFER[7];
uint8_t profileF;
uint8_t i2cadd=0x76<<1;
uint8_t Tsensor;
uint16_t timecounter=950;
uint8_t max=0;
uint16_t c[8];
uint8_t status;
uint8_t command;
uint8_t readresult[2];
char err;
uint32_t conversion;
uint8_t temp[3];

double OFF_;
float Aux;
uint64_t dT, Temperature;
uint64_t SENS;
uint32_t D1_Pres, D2_Temp;		 
uint32_t TEMP2, T2, OFF2, SENS2; 
uint32_t Pressure;			
uint32_t Depth;
float finalPressure;
float finalTemp;
float finalDepth;
uint8_t storeinit=0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void sinking();
void floating();
void resetoff();
void SensorReset();
unsigned char crc4(unsigned int n_prom[]) ;
void SensorGetCvalues();
void SensorGetConversion(uint8_t command);
void SensorGetData();
void AskForConnection();
void ProfileRun();


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
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	

	
	resetoff();
	
	
//reset pressure sensor
	//SensorReset();
	
//checking whether pressure sensor is ready
	//if(HAL_I2C_IsDeviceReady (&hi2c1, i2cadd, 10, HAL_MAX_DELAY)==HAL_OK)status=1;
	//HAL_Delay(10);
	//HAL_Delay(40);
	/*c1-c6:
	Pressure sensitivity, Pressure offset, 
	Temperature coefficient of pressure sensitivity,Temperature coefficient of pressure offset,
	Reference temperature, Temperature coefficient of the temperature */
	//SensorGetCvalues();
	HAL_Delay(40);
	HAL_UART_Receive_IT(&huart1, &RX_data, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc,&DateToUpdate,RTC_FORMAT_BIN);
		TX_BUFFER[0]=sTime.Hours;
		TX_BUFFER[1]=sTime.Minutes;
		TX_BUFFER[2]=sTime.Seconds;
		TX_BUFFER[3]=DateToUpdate.Year;
		TX_BUFFER[4]=DateToUpdate.Month;
		TX_BUFFER[5]=DateToUpdate.Date;
		TX_BUFFER[6]=0xff;
		
		
		//measure pressure each two seconds 
		//SensorGetData();
		//TX_BUFFER[7]=finalDepth;
		HAL_UART_Transmit(&huart1, TX_BUFFER, 7, 100);
		
			HAL_Delay(100);
			timecounter++;
			
			if(timecounter==450)
					resetoff();
			if(timecounter==460)
				 floating();
			
			if(timecounter==970){
				resetoff();
				timecounter=980;
			}
			
			if(timecounter==1000)
				timecounter=980;


//				//if(finalDepth==storeinit+500*3)
//				if(finalDepth==storeinit+150)
//					floating();
//				if(finalDepth==storeinit+10||finalDepth==storeinit-15)
//					resetoff();

		
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.OwnAddress1 = 64;
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
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 15;
  sTime.Minutes = 30;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
  DateToUpdate.Month = RTC_MONTH_APRIL;
  DateToUpdate.Date = 30;
  DateToUpdate.Year = 23;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  
{
	 if(RX_data==77||RX_data==66){
		 timecounter=0;
		ProfileRun();
	 }
		 
	if(RX_data==80||RX_data==88)
		 resetoff();
	 
	 HAL_UART_Receive_IT(&huart1, &RX_data, 1);
	 
}
void ProfileRun(){
		sinking();
		
		
}
unsigned char crc4(unsigned int n_prom[]) // n_prom defined as 8x unsigned int (n_prom[8])
{
	int cnt; // simple counter
	unsigned int n_rem=0; // crc remainder
	unsigned char n_bit;
	n_prom[0]=((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
	n_prom[7]=0; // Subsidiary value, set to 0
	for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
	 { // choose LSB or MSB
	 if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
	 else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
	 for (n_bit = 8; n_bit > 0; n_bit--)
	 {
	 if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
	 else n_rem = (n_rem << 1);
	 }
	 }
	n_rem= ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
	return (n_rem ^ 0x00);
}
void sinking(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
}
void floating(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
}
void resetoff(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
}
void SensorReset(){
	command=0x1E;
	HAL_I2C_Master_Transmit(&hi2c1, (i2cadd), &command, 1, HAL_MAX_DELAY);
	HAL_Delay(20);
}
//c1-c6:
//Pressure sensitivity, Pressure offset, 
//Temperature coefficient of pressure sensitivity,Temperature coefficient of pressure offset,
//Reference temperature, Temperature coefficient of the temperature 
void SensorGetCvalues(){
	
	for(uint8_t i=0;i<7;i++){
		command=0xA0+i*2;
		//command=0xA0+i<<1);
		//HAL_Delay(20);
		HAL_I2C_Master_Transmit(&hi2c1, (i2cadd), &command, 1, HAL_MAX_DELAY);
		HAL_Delay(20);
		HAL_I2C_Master_Receive(&hi2c1, (i2cadd), readresult,2,100);
		
		
	HAL_Delay(20);
//		HAL_I2C_Master_Receive(&hi2c1, (i2cadd), readresult,1,100);
//		command=0xA0+i*2+2;
//		HAL_Delay(20);
//		HAL_I2C_Master_Transmit(&hi2c1, (i2cadd), &command, 1, HAL_MAX_DELAY);
//		HAL_Delay(20);
//		HAL_I2C_Master_Receive(&hi2c1, (i2cadd), &readresult[1],1,100);
//  	HAL_Delay(20);
		c[i] = (((uint16_t)readresult[1] << 8) | readresult[0]);
		
		//c[i]=readresult;
	}
	
	err=crc4((unsigned int*)c);
}

void SensorGetConversion(uint8_t command){

	
	HAL_I2C_Master_Transmit(&hi2c1, (i2cadd), &command, 1, HAL_MAX_DELAY);
	HAL_Delay(20);	
	command=0x00;
	HAL_I2C_Master_Transmit(&hi2c1, (i2cadd), &command, 1, HAL_MAX_DELAY);
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1, (i2cadd), temp,3,100);
	conversion = ((uint32_t)temp[0] << 16) | ((uint32_t)temp[1] << 8) | temp[2];
	
}

void SensorGetData(){
	
	SensorGetConversion(0x46);
	D1_Pres=conversion;
	HAL_Delay(20);
	SensorGetConversion(0x56);
	D2_Temp=conversion;

	if (D2_Temp > (((uint32_t)c[4]) * 256.0))
	{
		dT = D2_Temp - (((uint32_t)c[6]) * 256);
		Temperature = 2000.0 + dT * ((uint32_t)c[6]) / 8388608.0f;
		OFF_ = (uint32_t)c[2] * 65536.0f + ((uint32_t)c[4] * dT) / 128.0f;
		SENS = (uint32_t)c[1] * 32768.0f + ((uint32_t)c[3] * dT) / 256.0f;
	}
	else
	{
		dT = (((uint32_t)c[5]) * 256.0f) - D2_Temp;
		Temperature = 2000.0f + dT * ((uint32_t)c[6]) / 8388608.0f;
		OFF_ = (uint32_t)c[2] * 65536.0f - ((uint32_t)c[4] * dT) / 128.0;
		SENS = (uint32_t)c[1] * 32768.0f - ((uint32_t)c[3] * dT) / 256.0;
	}
	
	if (Temperature < 2000) // low temp
	{
		Aux = (2000.0 - Temperature) * (2000.0 - Temperature);
		T2 = 3.0 * (dT * dT) / 8589934592.0f;
		OFF2 = 3 * Aux / 2;
		SENS2 = 5 * Aux / 8;
	}
	else
	{
		Aux = (Temperature - 2000) * (Temperature - 2000);
		T2 = 2 * (dT * dT) / 137438953472.0f;
		OFF2 = 1 * Aux / 16;
		SENS2 = 0;
	}
	OFF_ = OFF_ - OFF2;
	SENS = SENS - SENS2;
	
	finalTemp = (float)(Temperature - T2) / 100.0f;
	finalPressure =(float)((D1_Pres * SENS / 2097152.0f - OFF_) / 8192.0f) / 10.0f;
	//finalDepth = 0-(0.986923267* ( (0.0f-finalPressure-1013.25)*0.001))/10;
	finalDepth=0.986923267f*(finalPressure-985.0f);
	
	if(storeinit==0){
		storeinit=finalPressure;
	}
}

void AskForConnection(){
	profileF=88;
	while(1){
		HAL_UART_Transmit(&huart1, &profileF, 1, 100);
		HAL_Delay(500);
		if(RX_data==88){
			RX_data=0;
			break;
		}
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
