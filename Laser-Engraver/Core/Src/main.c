/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>


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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* Definitions for LaserEngrave */
osThreadId_t LaserEngraveHandle;
const osThreadAttr_t LaserEngrave_attributes = {
  .name = "LaserEngrave",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LoadInstruction */
osThreadId_t LoadInstructionHandle;
const osThreadAttr_t LoadInstruction_attributes = {
  .name = "LoadInstruction",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for valueQueue */
osMessageQueueId_t valueQueueHandle;
const osMessageQueueAttr_t valueQueue_attributes = {
  .name = "valueQueue"
};
/* Definitions for running */
osSemaphoreId_t runningHandle;
const osSemaphoreAttr_t running_attributes = {
  .name = "running"
};
/* Definitions for setup */
osSemaphoreId_t setupHandle;
const osSemaphoreAttr_t setup_attributes = {
  .name = "setup"
};
/* USER CODE BEGIN PV */
void myprintf(const char *fmt, ...);
int Xcurrent = 0; // current X coordinate of laser (units of .1mm)
int Ycurrent = 0; // current Y coordinate of laser (units of .1mm)
int XDIR = 1;	// X motor Direction (1 is increasing X coordinate, 0 is decreasing)
int YDIR = 1;	// Y motor Direction (1 is increasing Y coordinate, 0 is decreasing)
int Xend = 0; // End coordinate for the X motor (units of .1mm)
int Yend = 0; // End coordinate for the Y motor (units of .1mm)
int feed = 0;
int laser = 0;
int setup = 2;
int XCurrentCalculate = 0;
int YCurrentCalculate = 0;

typedef struct {
	int Xend;
	int Yend;
	int Xspeed;
	int Yspeed;
	int laserSpeed;
} Executable;

FRESULT fres; //Result after operations

//some variables for FatFs
FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
void StartLaserEngrave(void *argument);
void StartLoadInstruction(void *argument);
void StartControlTask(void *argument);

/* USER CODE BEGIN PFP */
void MotorStraightLine();
Executable ComputeExecutables(char[], char[], char[], char[], char[], char[]);
Executable GCommandParse(TCHAR* buff);
void addChar(char*,char);
void LaserEngrave(Executable);
void InitiateMotors();
void SetOutputs(Executable);
void StartEngrave(Executable);
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define LCD_ADDR 0x27 << 1 // LCD I2C Address (0x27 or 0x3F, depending on your LCD)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void myprintf(const char *fmt, ...)	// Function to print over UART nicely
{
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}

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
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  //some variables for FatFs
  FATFS FatFs;    //Fatfs handle
  FRESULT fres;    //Result after operations

  //Open the file system
  fres = f_mount(&FatFs, "", 1); //1=mount now
  if (fres != FR_OK) {
	myprintf("f_mount error (%i)\r\n", fres);
	while(1);
  }

  fres = f_open(&fil, "BYUI.txt", FA_READ);
  if (fres != FR_OK) {
	myprintf("f_open error (%i)\r\n", fres);
	while(1);
  }

  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */

  //Open the file system


  HAL_TIM_Base_Start_IT(&htim2);	// Starts the timer for PWM
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */

  HAL_Delay(1000); //a short delay is important to let the SD card settle



  /* USER CODE BEGIN 2 */


  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of running */
  runningHandle = osSemaphoreNew(1, 0, &running_attributes);

  /* creation of setup */
  setupHandle = osSemaphoreNew(1, 0, &setup_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of valueQueue */
  valueQueueHandle = osMessageQueueNew (100, 20, &valueQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LaserEngrave */
  LaserEngraveHandle = osThreadNew(StartLaserEngrave, NULL, &LaserEngrave_attributes);

  /* creation of LoadInstruction */
  LoadInstructionHandle = osThreadNew(StartLoadInstruction, NULL, &LoadInstruction_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 79999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1000;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 400;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 1000;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 400;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, YPUL_Pin|XDIR_Pin|XEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, YEN_Pin|YDIR_Pin|XPUL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : shutdownButton_Pin */
  GPIO_InitStruct.Pin = shutdownButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(shutdownButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : YPUL_Pin XDIR_Pin XEN_Pin */
  GPIO_InitStruct.Pin = YPUL_Pin|XDIR_Pin|XEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CD_Pin */
  GPIO_InitStruct.Pin = SPI1_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SPI1_CD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : YEN_Pin YDIR_Pin XPUL_Pin */
  GPIO_InitStruct.Pin = YEN_Pin|YDIR_Pin|XPUL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// function call to find 0 0 and align the motors to the correct starting position
void InitiateMotors()
{

	// setup variable which ensures the enable pins don't shut the system down during setup
	setup = 2;

	// writes a 0 to the enable pins to disable the motors
	HAL_GPIO_WritePin(XEN_GPIO_Port, XEN_Pin, 0);
	HAL_GPIO_WritePin(YEN_GPIO_Port, YEN_Pin, 0);

	HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, 1);

	// Sets the Y end point, motor prescaler value and starts the motor
	Yend = 99999999;
	__HAL_TIM_SET_PRESCALER(&htim17, 200);
	HAL_TIM_Base_Start_IT(&htim17);

	// Waits for the out of bounds button to be hit
	osSemaphoreAcquire(setupHandle, osWaitForever);
	HAL_Delay(500);
	setup--;
	Xcurrent = 0;
	Xend = 0;
	Ycurrent = 0;
	Yend = -3200;

	HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, 0);
	YDIR = 0;
	__HAL_TIM_SET_PRESCALER(&htim17, 50);
	HAL_TIM_Base_Start_IT(&htim17);

	osSemaphoreAcquire(runningHandle, osWaitForever);

	HAL_Delay(500);

	HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, 1);

	// Sets the X end point, motor prescaler value and starts the motor
	Xend = 99999999;
	__HAL_TIM_SET_PRESCALER(&htim16, 200);
	HAL_TIM_Base_Start_IT(&htim16);

	// Waits for the out of bounds button to be hit
	osSemaphoreAcquire(setupHandle, osWaitForever);
	HAL_Delay(500);
	setup--;
	Xcurrent = 0;
	Xend = -3200;
	Ycurrent = 0;
	Yend = 0;

	HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, 0);
	XDIR = 0;
	__HAL_TIM_SET_PRESCALER(&htim16, 50);
	HAL_TIM_Base_Start_IT(&htim16);

	osSemaphoreAcquire(runningHandle, osWaitForever);

	HAL_Delay(500);

	Xcurrent = 0;
	Ycurrent = 0;

}

// Interrupt Handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	// code for the shutdown buttons which trigger if the motors run out of bounds
	if (GPIO_Pin == shutdownButton_Pin)
	{

		// Turns off the motor timers so they don't send a signal anymore
		HAL_TIM_Base_Stop_IT(&htim16);
		HAL_TIM_Base_Stop_IT(&htim17);

		// Turns of the laser's PWM
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		HAL_TIM_Base_Stop_IT(&htim2);

		// Puts the code in a while loop so the program has to be reset if it enters this state
		while(setup == 0){
			// writes a 1 to the enable pins to disable the motors
			HAL_GPIO_WritePin(XEN_GPIO_Port, XEN_Pin, 1);
			HAL_GPIO_WritePin(YEN_GPIO_Port, YEN_Pin, 1);
		}
		osSemaphoreRelease(setupHandle);
	}
}

Executable GCommandParse(TCHAR* line1)
{

	  // Creates local variables for the different possible commands in Gcode (max 10 characters)
	  char Gcommand[10] = "";
	  char Xcoordinate[10] = "";
	  char Ycoordinate[10] = "";
	  char Zcoordinate[10] = "";
	  char feedRate[10] = "";
	  char laserSpeed[10] = "";

	  // New struct to save the values of the executables which are calculated in these functions
	  Executable newExecutable;

	  int i = 0;
	  // Initiates a for loop which loops each character of the Gcode line
	  while (1)
	  {

		  // Creates a temporary variable for the Gcode command and the value attached to it
		  char command = line1[i];	// assigns the first value of the Gcode as the command
		  char newValue[10] = "";

		  // Temporary, incrementing variables
		  int j = 0;	// represents the current charcter in the word
		  i++;		// indicates the total character in the line

		  // while loop that loops through the rest of the command and stores the value in newValue
		  while ((line1[i] != ' ') && (line1[i] != '\0') && (line1[i] != '\n'))
		  {
			  newValue[j] = line1[i];
			  i++;
			  j++;
		  }

		  //adds the null operator to the end of the newValue
		  newValue[j] = '\0';

		  // Switch statement for the value of command to split the current word into it's variable
		  switch (command)
		  {
		  case 'G':
			  strncpy(Gcommand, newValue, 10);	// copies the value in newValue to the Gcommand variable
				break;
		  case 'X':
			  strncpy(Xcoordinate, newValue, 10);	// copies the value in newValue to the Xcoordinate variable
				break;
		  case 'Y':
			  strncpy(Ycoordinate, newValue, 10);	// copies the value in newValue to the Ycoordinate variable
				break;
		  case 'Z':
			  strncpy(Zcoordinate, newValue, 10);	// copies the value in newValue to the Zcoordinate variable
				break;
		  case 'M':
			  strncpy(Gcommand, newValue, 10);	// copies the value in newValue to the Mcommand variable
				break;
		  case 'S':
			  strncpy(laserSpeed, newValue, 10);	// copies the value in newValue to the laserSpeed variable
				break;
		  case 'F':
			  strncpy(feedRate, newValue, 10);	// copies the value in newValue to the feedRate variable
				break;
		  case 'R':
				break;
		  default:
				break;
		  }

		  // When the new line operator occurs, this means the line is over, so we break the while loop
		  if (line1[i] == '\n')
			  break;

		  // increments i to the value of the first character of the next command
		  i++;
	  }

	  // Calls the Compute Executables command which computes the output Executables
	  newExecutable = ComputeExecutables(Gcommand, Xcoordinate, Ycoordinate, Zcoordinate, feedRate, laserSpeed);

	  // Returns the values in the Executable struct
	  return newExecutable;
}

// function which adds a char to the end of a char array
void addChar(char *s, char c)
{
	while (*s++);

	*(s - 1) = c;

	*s = '\0';
}

// Command Execute takes the parameters from the Gcode line and controls the motors accordingly
Executable ComputeExecutables(char Gcommand[], char Xcommand[], char Ycommand[], char Zcommand[], char feedRate[], char laserSpeed[])
{

	// creates a temporary Executable struct to save the values we compute
	Executable newExecutable;

	// If the Gcode command is G0, runs with rapid positioning (full speed move)
	if ((!strcmp(Gcommand,"0")) || (!strcmp(Gcommand,"1")))
	{

		// If there is a value in Xcommand, it computes the end point of the system (1600 pulses per motor rotation, 43.39 mm per rotation)
		if (Xcommand[0] != '\0')
		{
			newExecutable.Xend = ((1600 / 43.39) * atof(Xcommand));	// Converts Xcommand to an int, changes units to .1 mms and updates the global variable
		}

		// If there is no value in Xcommand, Xend equals the current Xvalue (Xcurrent is a global variable that stores the X position that is currently being calculated
		else
		{
			newExecutable.Xend = XCurrentCalculate;
		}

		// If there is a value in Ycommand, it computes the end point of the system (1600 pulses per motor rotation, 43.39 mm per rotation)
		if (Ycommand[0] != '\0')
		{
			newExecutable.Yend = ((1600 / 43.39) * atof(Ycommand));	// Converts Ycommand to an int, changes units to .1 mms and updates the global variable
		}

		// If there is no value in Xcommand, Xend equals the current Xvalue (Xcurrent is a global variable that stores the X position that is currently being calculated
		else
		{
			newExecutable.Yend = YCurrentCalculate;
		}

		// Calculates distance to be traveled in the current engrave
		float Xdistance = newExecutable.Xend - XCurrentCalculate;
		float Ydistance = newExecutable.Yend - YCurrentCalculate;

		// sets the current calculate global variable for the next line to use
		XCurrentCalculate = newExecutable.Xend;
		YCurrentCalculate = newExecutable.Yend;

		// sets the feedRate variable (sometimes the Gcommand doesn't put in a new feedRate variable and the feedRate stays the same
		// In order to calculate each step, we need to save the feedRate because the next command may need it
		if (feedRate[0] != '\0')
		{
			feed = atoi(feedRate);	// Converts feedRate to an int
		}

		// Initiates the Xspeed and Yspeed variables to 1 (so if the speed doesn't change, the next instruction won't divide by 0)
		// It is set to one so that the timer interrupt will be called quickly, and the stopped motor won't disrupt the other motor)
		newExecutable.Xspeed = 1;
		newExecutable.Yspeed = 1;

		// calculates the total distance in order to evaluate the speed (feedRate is given in a direct, diagonal path)
		float totalDistance = sqrt((Xdistance * Xdistance) + (Ydistance * Ydistance));

		// ensures the distance won't be divided by 0
		if (Xdistance != 0)
		{

			// Calcualtes the required prescaler value and saves it in the Xspeed variable
			newExecutable.Xspeed = 162712.482 / ((abs(Xdistance) / totalDistance) * feed);
		}

		// Ensures the distance won't be divided by 0
		if (Ydistance != 0)
		{

			// Calculates the required prescaler value and saves it in the Yspeed variable
			newExecutable.Yspeed = 162712.482 / ((abs(Ydistance) / totalDistance) * feed);
		}

		// Converters laserSpeed to an int
		newExecutable.laserSpeed = atoi(laserSpeed);

	}

	// if there are any other possible input comands (M command, G02, G03, etc. we can put them here)

	// returns the Executable struct
	return newExecutable;



}

// Function to set the laser power. input must be a value from 0 to 255
void SetLaserPower(uint8_t power) {

	// the timer requires the duty cycle in a ratio from 0 to 80000
	float dutyCycle = (power/255.0) * 80000;
    TIM2->CCR1 = dutyCycle;  // Set duty cycle
}

// Function to start the motors and laser for engraving
void StartEngrave(Executable output){

	  // Starts the PWM for the laser
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_Base_Start_IT(&htim2);

	  // Starts the timers for the motors
	  HAL_TIM_Base_Start_IT(&htim16);
	  HAL_TIM_Base_Start_IT(&htim17);

}

// Function which sets all the parameters in order to engrave
void SetOutputs(Executable output){

	// Sets the Xend and Yend global variables so the motors know when to stop
	Xend = output.Xend;
	Yend = output.Yend;

	// Updates the X Direction variable and writes to the pin
	if (output.Xend > Xcurrent)
	{
		XDIR = 1;
		HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, 1);
	}
	else if (output.Xend < Xcurrent)
	{
		XDIR = 0;
		HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, 0);
	}

	// Updates the X Direction variable and writes to the pin
	if (output.Yend > Ycurrent)
	{
		YDIR = 1;
		HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, 1);
	}
	else if (output.Yend < Ycurrent)
	{
		YDIR = 0;
		HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, 0);
	}

	// Sets the prescaler values so the motors turn at the correct speed
	__HAL_TIM_SET_PRESCALER(&htim16, output.Xspeed);
	__HAL_TIM_SET_PRESCALER(&htim17, output.Yspeed);

	// Sets the Laser Power to the correct value
	SetLaserPower(output.laserSpeed);
}

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, 0xFFFF);
	return ch;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLaserEngrave */
/**
  * @brief  Function implementing the LaserEngrave thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLaserEngrave */
void StartLaserEngrave(void *argument)
{
  /* USER CODE BEGIN 5 */
	InitiateMotors();

	while (setup != 0) osDelay(1);
  // Ensures the enable pins are turned off to allow the motors to turn
  HAL_GPIO_WritePin(XEN_GPIO_Port, XEN_Pin,0);
  HAL_GPIO_WritePin(YEN_GPIO_Port, YEN_Pin,0);

  // Declares the executable outside the for loop so that it doesn't have to be redeclared every time
  Executable newExecutable;

  // gives the loadInstruction Task a chance to load data from the SD card to fill the queue
  osDelay(100);

  osSemaphoreRelease(runningHandle);

  /* Infinite loop */
  for(;;)
  {
	  	  // This is where the program waits for the engrave to finish. The semaphore is initiated as fulfilled so it begins correctly.
		  osSemaphoreAcquire(runningHandle, osWaitForever);

		  // Gets the executable struct from the queue with the next engrave
		  osMessageQueueGet(valueQueueHandle, (Executable *) &newExecutable, 0, osWaitForever);

		  // Sets the outputs such as the DIR outputs, the motor speeds and the laser PWM duty cycle
		  SetOutputs(newExecutable);

		  // Begins the timers for the engrave
		  StartEngrave(newExecutable);
  }


  osThreadTerminate(NULL); // In case we accidentally exit from task loop
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLoadInstruction */
/**
This is the LoadInstruction Task whose main purpose is to load information from the SD card, translate it into executable data, and add it to the queue
This task has a lower priority than the laser engrave task, since we can load information in the background while the motors are running rather than during the laser engrave task
*/
/* USER CODE END Header_StartLoadInstruction */
void StartLoadInstruction(void *argument)
{
  /* USER CODE BEGIN StartLoadInstruction */
  /* Infinite loop */
	while (setup != 0) osDelay(1);

	// Declares variables outside of the loop so we don't have to reallocate memory for them each iteration of the for loop
	BYTE readBuf[100];	// char array to store data from SD card
	TCHAR* rres;	// return variable determining if the SD card is out of data
	Executable newExecutable;	// int array with variables for us to execute

  for(;;)
  {
	  // reads a line from the SD card and saves it in readBuf
	  rres = f_gets((TCHAR*)readBuf, 100, &fil);

	  // if there is data sent, it calls the GcommandParse function which computes the values we need to save in the executable struct
	  if(rres != 0) {
		newExecutable = GCommandParse((TCHAR*)readBuf);

	  // if f_get returns 0, the program ends and enters a while loop
	  } else {
		f_close(&fil);	// closes the SD card file
		f_mount(NULL, "", 0);	// un mounts the SD card
		while(1){}		// loops forever because the program has ended
	  }

	  // Once we have the executable struct, we save the address to the Queue
	  osMessageQueuePut(valueQueueHandle, &newExecutable, 0, osWaitForever);
  }

  osThreadTerminate(NULL); // In case we accidentally exit from task loop
  /* USER CODE END StartLoadInstruction */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the ControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for(;;)
  {

	  osDelay(1);
  }
  /* USER CODE END StartControlTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

	if (htim == &htim16)	// X motor timer
	{
		if (Xcurrent != Xend)	// Evaluates if the X motor has arrived in it's position
		{
			HAL_GPIO_TogglePin(XPUL_GPIO_Port, XPUL_Pin);	// Toggles the XPUL pin

			// Increments the Xcurrent value if XDIR is positive and decrements if Xcurrent value is negative
			// only increments every other cycle
			if (!HAL_GPIO_ReadPin(XPUL_GPIO_Port, XPUL_Pin))
			{
				if (XDIR == 1) Xcurrent++;

				else Xcurrent--;
			}
		}
		else
		{
			HAL_TIM_Base_Stop_IT(&htim16);	// Once the X motor arrives to it's final position, this stops the timer

			// if the X motor and Y motor have both arrived at their destinations
			if ((Ycurrent == Yend))
			{

				// stops the laser PWM
				HAL_TIM_Base_Stop_IT(&htim2);
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

				// Gives the semaphore so the program can continue
				if (osSemaphoreGetCount(runningHandle) == 0)
					osSemaphoreRelease(runningHandle);
			}
		}
	}


	if (htim == &htim17)	// Y motor timer
	{

		if (Ycurrent != Yend)	// Evaluates if the Y motor has arrived in it's position
		{

			HAL_GPIO_TogglePin(YPUL_GPIO_Port, YPUL_Pin);	// Toggles the YPUL pin

			// Increments the Ycurrent value if YDIR is positive and decrements if Ycurrent value is negative
			if (!HAL_GPIO_ReadPin(YPUL_GPIO_Port, YPUL_Pin))
			{
				if (YDIR == 1) Ycurrent++;

				else Ycurrent--;
			}
		}
		else
		{
		  HAL_TIM_Base_Stop_IT(&htim17);	// Once the Y motor arrives to it's final position, this stops the timer

		  // if the X motor and Y motor have both arrived at their destinations
		  if (Xcurrent == Xend)
		  {
			  // stops the laser PWM
			  HAL_TIM_Base_Stop_IT(&htim2);
			  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				// Gives the semaphore so the program can continue
			  if (osSemaphoreGetCount(runningHandle) == 0)
				  osSemaphoreRelease(runningHandle);
		  }
		}
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* USER CODE END Callback 1 */
}

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
