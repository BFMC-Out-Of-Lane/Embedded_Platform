/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>


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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
float  Map(float Value, float fromLow, float fromHigh, float toLow, float toHigh);
void CommandProcessing();
void ADCaverage();

int16_t interpolate(int speed, const int speedValuesP[], const int speedValuesN[], const int stepValues[], int size);
int conversion(int f_speed, int step_value, int zero_default);

void ReadIMU();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t adc_IN[3];
int vBat=0;
int instCons=0;

int aveV=0;
int aveI=0;
int countADC=0;
int count2=0;

char rxData[1];				//Buffer Uart Recieve
char txData[250];			//Buffer Uart Transmit

char command[30];			//Buffer Command


int speed=0;
float kPSpeed=0.01;
float kDSpeed=0.01;
int steer=0;
int kl=0;

int timCount=0;
float realSpeed=0;			//velocidad medida con optocoplador
float speedProp=0;			//
float distance=0;


float distanceSpeed;
int	timSpeed=0;
int countTimeSpeed=25;

int optoCount=0;			//Cuenta de optocoplador.
int optoCountWErrors=0;



float optoFrecuency=0;

float optoK =5.66974; 				//5.58;			//Distancia recorrida por pulso de optocoplador


int speedValuesP[25] = {
     40, 50, 60, 70, 80, 90, 100, 110, 120, 130,
    140, 150, 160, 170, 180, 190, 200, 210, 220, 260,
    300, 350, 400, 450, 500
};

int speedValuesN[25] = {
     -40, -50, -60, -70, -80, -90, -100, -110, -120, -130,
    -140, -150, -160, -170, -180, -190, -200, -210, -220, -260,
    -300, -350, -400, -450, -500
};

// StepValues have a scale factor applied (*10)
int stepValues[25] = {
    214, 176, 152, 134, 120, 110, 102, 94, 86, 82,
    78, 74, 70, 68, 66, 64, 60, 58, 56, 50,
    48, 42, 38, 36, 34
};

int velInter=0;

int velInter2=0;


//IMU
int i2cBuff[2];

int direction=0;
HAL_I2C_StateTypeDef estado;
HAL_I2C_StateTypeDef estado2;

uint8_t mode_config = 0x00;  // CONFIGMODE
uint8_t mode_ndof   = 0x0C;  // NDOF //orientacion sin magnetometro=0x0B----con magnetometro 0x0C



uint16_t const BNO055_ADDRESS=82;		//28
uint16_t const BNO055_OPR_MODE_ADDR=0x3D;

uint8_t euler_data[6];
int16_t heading;
int16_t roll;
int16_t pitch;

float f_heading;
float f_headingOffset;

float f_roll;
float f_pitch;

uint16_t const BNO055_EULER_H_LSB_ADDR = 0x1A;

//calculo de coordenadas
float xCoordinates;
float yCoordinates;
float deltaDistance;
float angleOffset=0;

//Optocoplador ADC
int flagRisingOpto=0;


int pepe=1;


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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  sprintf (txData,
		  "\r\n\r\n"
		  "#################\r\n"
		  "#               #\r\n"
		  "#   I'm GROOT   #\r\n"
		  "#               #\r\n"
		  "#################\r\n"
		  "\r\n");

  HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));		//Activa interrupcion de Transmicion UART

  HAL_UART_Receive_IT (&huart2,(uint8_t *)rxData,1);					//Activa interrupcion de recepcion UART

 // HAL_ADC_Start(&hadc1);

  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);							//activa interrupcion PWM de timer 3
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);							//activa interrupcion PWM de timer 2

  HAL_TIM_Base_Start_IT(&htim3);										//Activa interrupcion de fin de cuenta timer 3
  HAL_TIM_Base_Start(&htim3);											//Inicia el timer 3
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_IN, 3);						//inicio de DMA para ADC con 3 canales

  //IMU

//    estado=HAL_I2C_GetState(&hi2c1);
//    estado2=HAL_I2C_IsDeviceReady(&hi2c1, 28, 1, 100);

    if(HAL_I2C_IsDeviceReady(&hi2c1, 82, 1, 10)== HAL_OK)				//consulta si el dispositivo de la direccion 82 responde
    {
  	  direction=82;

    }else


    HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, 1, &mode_config, 1, HAL_MAX_DELAY);		//configuracion del IMU
    HAL_Delay(25);
    HAL_I2C_Mem_Write(&hi2c1, BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, 1, &mode_ndof, 1, HAL_MAX_DELAY);		//Configuracion del IMU
    HAL_Delay(250);

	ReadIMU();
	angleOffset=f_heading;
	f_headingOffset=f_heading-angleOffset;		//setea en 0 el angulo con un offset


	//Cosas para optocoplador

	HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE);  // Modo One Pulse (una vez)
	HAL_TIM_OnePulse_Start(&htim4, TIM_CHANNEL_1);

	__HAL_TIM_SET_COUNTER(&htim4, 0);					//setea la cuenta en 0
	__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);		//resetea el flag del timer

	TIM4->CR1 |= TIM_CR1_CEN;							//Dispara el timer


/*
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

	HAL_TIM_OnePulse_Start(&htim4, TIM_OPMODE_SINGLE);
*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 //Prueba parpadeo de led con tim4
	  /*
	  if ((__HAL_TIM_GET_COUNTER(&htim4) > 7000 || __HAL_TIM_GET_COUNTER(&htim4)==0) )
	  	{
	      	__HAL_TIM_SET_COUNTER(&htim4, 0);
	      	__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

	  		TIM4->CR1 |= TIM_CR1_CEN;

	      	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);		//TIM3 Output test

	  	}

*/


//	  pepe=__HAL_TIM_GET_COUNTER(&htim4);		//lee el contador del timer

	if(kl<30)
		{
			speed=0;
			htim2.Instance->CCR2= 1500;			//Setea el ancho del PWM en uS
		}

	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0)		//Pregunta si se pulso el boton azul de la NUCLEO
	{
		speed=0;
		htim2.Instance->CCR2=1500;
		kl=0;

		ReadIMU();
		//angleOffset=f_heading;
		//f_headingOffset=f_heading-angleOffset;
		distance=0;
		//xCoordinates=0;
		//f_headingOffsetyCoordinates=0;

		sprintf (txData,"Lara Stopped Manualy\r\n\r\n"
				"@kl:%d;;\r\n"
				"@speed:%d;;\r\n"
				"@distance:%.2f;;\r\n"
				"@position:%.2f;%.2f;\r\n"
				"@imu:%.2f;%.2f;%.2f;0;0;0;\r\n"
				"\r\n\r\n"

				"cant de pulsos------------------>%d\r\n"
				"cant de pulsos + errores-------->%d\r\n\r\n"

				,
				kl, speed, distance, xCoordinates, yCoordinates, f_roll, f_pitch, f_headingOffset, optoCount, optoCountWErrors);
		HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));

	}

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim4, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)				//Interrupcion de que se recibio alg en la UART
{

	if(huart->Instance ==USART2)									//Pregunta si la interrupcion es de la UART2
	{
		HAL_UART_Receive_IT (&huart2,(uint8_t *)rxData,1);			//Lee el bye recibido y lo guarda en rxData

		if(rxData[0] == '#')
		{
			strcpy(command, "");									//Borra el string de command
		}
		strncat(command, rxData, 1);								//concatena command con rxData

		if(rxData[0] == '\n')
		{
			//Pcess Command
			//CommandProcessing();

			char cmd[20];
			int value1;
			int value2;
			int value3;
			sscanf(command, "#%[^:]:%d;%d;%d", cmd, &value1, &value2, &value3);		//Separa los parametros

			if(strcmp(cmd, "batteryCapacity") == 0)
			{
				sprintf (txData,"@batteryCapacity:ack;;\r\n");
				HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
			}

			if(strcmp(cmd, "kl") == 0)
			{
				kl=value1;
				sprintf (txData,"@kl:%d;;\r\n", kl);
				HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
				return;
			}

			if(strcmp(cmd, "speed") == 0)
			{

				if(kl >= 30)
				{
					speed=value1;

					int stepValue=interpolate(speed, speedValuesP, speedValuesN, stepValues, 25);

					htim2.Instance->CCR2=conversion(speed, stepValue, 1491);

					sprintf (txData,"@speed:%d;;\r\n", speed);
					HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));

				}else
				{
					speed=0;
					sprintf (txData,"kl should be 30\r\n");
					HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
				}

				return;
			}

			if(strcmp(cmd, "steer") == 0)
			{
				steer=value1;
				sprintf (txData,"@steer:%d;;\r\n", steer);
				HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));

				htim3.Instance->CCR2= Map(steer,-230,230,1000,2000);
				return;
			}

			if(strcmp(cmd, "setDistance") == 0)
				{
					distance=value1;

					sprintf (txData,"@setDistance:%.2f;;\r\n", distance);
					HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
				}

			if(strcmp(cmd, "setPosition") == 0)
				{
					xCoordinates=value1;
					yCoordinates=value2;

					sprintf (txData,"@setPosition:%.2f;%.2f;\r\n", xCoordinates, yCoordinates);
					HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
				}

			if(strcmp(cmd, "setAngle") == 0)
				{
					ReadIMU();

					angleOffset=f_heading-value1;

					f_headingOffset=f_heading-angleOffset;

					sprintf (txData,"@setAngle:%.2f;:\r\n", f_headingOffset);
					HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
				}


		}
	}
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)		//Timer
{
	if(htim->Instance ==TIM3)
	{
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);		//TIM3 Output test

		//Speed measurement
		timSpeed++;
		if(timSpeed>=countTimeSpeed)
		{
			//realSpeed=copysign(distanceSpeed*2, speed);
			realSpeed=distanceSpeed/(0.02*countTimeSpeed);

			optoFrecuency=distanceSpeed/(optoK*0.02*countTimeSpeed);

			distanceSpeed=0;
			timSpeed=0;




			if(kl >= 30)
			{
				sprintf (txData,
						"@realSpeed:%.2f;;\r\n"
						"@distance:%.2f;;\r\n"
						"@position:%.2f;%.2f;\r\n"
						"@imu:%.2f;%.2f;%.2f;0;0;0;\r\n",
						realSpeed, distance, xCoordinates, yCoordinates, f_roll, f_pitch, f_headingOffset);
				HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
			}else if(kl >= 15)
			{
				sprintf (txData, "@imu:%.2f;%.2f;%.2f;0;0;0;\r\n", f_roll, f_pitch ,f_headingOffset);
				HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
			}

		}
		//Setting real speed
		if(speed !=0)
		{
	//		speedProp+=(speed-realSpeed)*kPSpeed+(speed-realSpeed)*(speed-realSpeed)*kDSpeed;
//			speedProp+=(speed-realSpeed)*kPSpeed;	//kSpeed=0.01
//			htim2.Instance->CCR2= Map(speedProp,500,-500,1360,1640);
/*
			htim2.Instance->CCR2= Map(speed,500,-500,1360,1640);

			if(htim2.Instance->CCR2>1640)
			{
				htim2.Instance->CCR2=1640;
			}
			if(htim2.Instance->CCR2<1360)
			{
				htim2.Instance->CCR2=1360;
			}

		}else
		{
			speedProp=0;
			htim2.Instance->CCR2=1500;
		}*/




		}
	}
}
/*
if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE))
{
	  // Se cumplió el tiempo

	  __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
	  // ... tu código acá ...
	  __HAL_TIM_SET_COUNTER(&htim4, 0);
	  TIM4->CR1 |= TIM_CR1_CEN;

	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
*/


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
/*{
	if (GPIO_Pin == GPIO_PIN_10)				//Optocoplator flanco descendente
	    {
	    	optoCountWErrors++;
	    }


if (GPIO_Pin == GPIO_PIN_10 && __HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) && !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10))				//Optocoplator flanco descendente
    {
    	__HAL_TIM_SET_COUNTER(&htim4, 0);
    	__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

		TIM4->CR1 |= TIM_CR1_CEN;

    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);		//TIM3 Output test
    }
*/
    //if (GPIO_Pin == GPIO_PIN_10 && __HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) /*&& HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10)*/)				//Optocoplator flanco descendente
/*	if (GPIO_Pin == GPIO_PIN_10 && (__HAL_TIM_GET_COUNTER(&htim4) > 7000 || __HAL_TIM_GET_COUNTER(&htim4)==0))				//Optocoplator flanco descendente
	{
    	__HAL_TIM_SET_COUNTER(&htim4, 0);
    	__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

		TIM4->CR1 |= TIM_CR1_CEN;

    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);		//TIM3 Output test

    	ReadIMU();
    	optoCount++;

    	//
    	if(speed > 0)
    	{
    		distance+=optoK;			//0.215214;				//5.729;
    		distanceSpeed+=optoK;		//0.215214;				//5.729;

    	   	xCoordinates+=optoK*cos(f_headingOffset* (M_PI / 180.0));
   			yCoordinates+=optoK*sin(f_headingOffset* (M_PI / 180.0));

    	}else if(speed < 0)
    	{
    		distance-=optoK;
    		distanceSpeed-=optoK;

    	   	xCoordinates-=optoK*cos(f_headingOffset* (M_PI / 180.0));
   			yCoordinates-=optoK*sin(f_headingOffset* (M_PI / 180.0));

    	}
    }
}
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)		//interrupcion de lectura ADC
{
	if (hadc->Instance == ADC1/* && (hadc->Instance->ISR & ADC_FLAG_EOC_7)*/)
	{
		if(countADC <10000)			//10000=1Seg
		{
			aveV+=adc_IN[0];
			aveI+=adc_IN[1];
			countADC++;
		}else
		{
			vBat=aveV*2.215/10000;
			instCons=aveI*8.35/10000;
			aveV=0;
			aveI=0;
			countADC=0;

			if(kl >= 15)
			{
				sprintf (txData,"@battery:%d;;\r\n""@instant:%d;;\r\n", vBat, instCons);
				HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
			}
		}

		if(adc_IN[2] > 3000)
		{
			flagRisingOpto=1;
		}
		if(adc_IN[2] < 1000 && flagRisingOpto==1)
		{
			flagRisingOpto=0;
			/*
			__HAL_TIM_SET_COUNTER(&htim4, 0);
			__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

			TIM4->CR1 |= TIM_CR1_CEN;
*/
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);		//TIM3 Output test

			ReadIMU();
			optoCount++;

			//
			if(speed > 0)
			{
				distance+=optoK;			//0.215214;				//5.729;
				distanceSpeed+=optoK;		//0.215214;				//5.729;

				xCoordinates+=optoK*cos(f_headingOffset* (M_PI / 180.0));
				yCoordinates+=optoK*sin(f_headingOffset* (M_PI / 180.0));

			}else if(speed < 0)
			{
				distance-=optoK;
				distanceSpeed-=optoK;

				xCoordinates-=optoK*cos(f_headingOffset* (M_PI / 180.0));
				yCoordinates-=optoK*sin(f_headingOffset* (M_PI / 180.0));

			}
		}
	}
}



void ReadIMU()
{
	//leer orientacion
	HAL_I2C_Mem_Read(&hi2c1, BNO055_ADDRESS, BNO055_EULER_H_LSB_ADDR, 1, euler_data, 6, 50);

	heading = (euler_data[1] << 8) | euler_data[0];
	roll    = (euler_data[3] << 8) | euler_data[2];
	pitch   = (euler_data[5] << 8) | euler_data[4];

	// Convertir a grados (unidad es 1/16 grados)
	f_heading = 360-(heading / 16.0);
	f_roll    = roll / 16.0;
	f_pitch   = pitch / 16.0;



	//calculo de coordenadas
	f_headingOffset=f_heading-angleOffset;
	if(f_headingOffset>360)
	{
		f_headingOffset-=360;
	}
	if(f_headingOffset<0)
	{
		f_headingOffset+=360;
	}

}

void CommandProcessing()
{
	char cmd[20];
	int value1;
	int value2;
	int value3;
	sscanf(command, "#%[^:]:%d;%d;%d", cmd, &value1, &value2, &value3);

	if(strcmp(cmd, "batteryCapacity") == 0)
	{
		sprintf (txData,"@batteryCapacity:ack;;\r\n");
		HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
	}

	if(strcmp(cmd, "kl") == 0)
	{
		kl=value1;
		sprintf (txData,"@kl:%d;;\r\n", kl);
		HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
		return;
	}

	if(strcmp(cmd, "speed") == 0)
	{

		if(kl >= 30)
		{
			speed=value1;

			int stepValue=interpolate(speed, speedValuesP, speedValuesN, stepValues, 25);

			htim2.Instance->CCR2=conversion(speed, stepValue, 1491);

			sprintf (txData,"@speed:%d;;\r\n", speed);
			HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));

		}else
		{
			speed=0;
			sprintf (txData,"kl should be 30\r\n");
			HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
		}

		return;
	}

	if(strcmp(cmd, "steer") == 0)
	{
		steer=value1;
		sprintf (txData,"@steer:%d;;\r\n", steer);
		HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));

		htim3.Instance->CCR2= Map(steer,-230,230,1000,2000);
		return;
	}

	if(strcmp(cmd, "setDistance") == 0)
		{
			distance=value1;

			sprintf (txData,"@setDistance:%.2f;;\r\n", distance);
			HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
		}

	if(strcmp(cmd, "setPosition") == 0)
		{
			xCoordinates=value1;
			yCoordinates=value2;

			sprintf (txData,"@setPosition:%.2f;%.2f;\r\n", xCoordinates, yCoordinates);
			HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
		}

	if(strcmp(cmd, "setAngle") == 0)
		{
			ReadIMU();

			angleOffset=f_heading-value1;

			f_headingOffset=f_heading-angleOffset;

			sprintf (txData,"@setAngle:%.2f;:\r\n", f_headingOffset);
			HAL_UART_Transmit_IT(&huart2,(uint8_t *)txData, strlen(txData));
		}


}

float  Map(float Value, float fromLow, float fromHigh, float toLow, float toHigh)
{
	float Output=0;
	Output=(((toHigh-toLow)/(fromHigh-fromLow))*(Value-fromLow))+toLow;
	return Output;
}

int16_t interpolate(int speed, const int speedValuesP[], const int speedValuesN[], const int stepValues[], int size)
{
        if(speed <= speedValuesP[0]){
            if (speed >= speedValuesN[0])
            {
                return stepValues[0];
            }
            else{
                for(uint8_t i=1; i<size; i++)
                {
                    if (speed >= speedValuesN[i])
                    {
                        int slope = (stepValues[i] - stepValues[i-1]) / (speedValuesN[i] - speedValuesN[i-1]);
                        return stepValues[i-1] + slope * (speed - speedValuesN[i-1]);
                    }
                }
            }

        }
        if(speed >= speedValuesP[size-1]) return stepValues[size-1];
        if(speed <= speedValuesN[size-1]) return stepValues[size-1];

        for(uint8_t i=1; i<size; i++)
        {
            if (speed <= speedValuesP[i])
            {
                int slope = (stepValues[i] - stepValues[i-1]) / (speedValuesP[i] - speedValuesP[i-1]);
                return stepValues[i-1] + slope * (speed - speedValuesP[i-1]);
            }
        }

        return -1;
    }

int conversion(int f_speed, int step_value, int zero_default)
{
    return ((step_value * -f_speed)/100 + zero_default);
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
