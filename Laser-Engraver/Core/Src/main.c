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
#include "fatfs.h"
// Heyyy

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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
void myprintf(const char *fmt, ...);
int Xcurrent = 0; // current X coordinate of laser (units of .1mm)
int Ycurrent = 0; // current Y coordinate of laser (units of .1mm)
int XDIR = 1;	// X motor Direction (1 is increasing X coordinate, 0 is decreasing)
int YDIR = 1;	// Y motor Direction (1 is increasing Y coordinate, 0 is decreasing)
int Xend = 0; // End coordinate for the X motor (units of .1mm)
int Yend = 0; // End coordinate for the Y motor (units of .1mm)
int feed = 0;
int laserSpeed = 0;
int setup = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void MotorStraightLine();
void GcommandExecute(char[], char[], char[], char[], char[], char[]);
void McommandExecute(char[]);
void GcommandParse(TCHAR* buff);
void addChar(char*,char);
void LaserEngrave(int, int);
void InitiateMotors();
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void myprintf(const char *fmt, ...)
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
  /* USER CODE BEGIN 2 */
  	  myprintf("\r\n~ SD card demo by kiwih ~\r\n\r\n");

      HAL_Delay(1000); //a short delay is important to let the SD card settle

      //some variables for FatFs
      FATFS FatFs; 	//Fatfs handle
      FIL fil; 		//File handle
      FRESULT fres; //Result after operations

      //Open the file system
      fres = f_mount(&FatFs, "", 1); //1=mount now
      if (fres != FR_OK) {
    	myprintf("f_mount error (%i)\r\n", fres);
    	while(1);
      }

      fres = f_open(&fil, "test.txt", FA_READ);
	  if (fres != FR_OK) {
		myprintf("f_open error (%i)\r\n", fres);
		while(1);
	  }
	  myprintf("I was able to open 'gtest.txt' for reading!\r\n");


  HAL_GPIO_WritePin(XEN_GPIO_Port, XEN_Pin,0);
  HAL_GPIO_WritePin(YEN_GPIO_Port, YEN_Pin,0);

  InitiateMotors();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (HAL_GPIO_ReadPin(SPI1_CD_GPIO_Port, SPI1_CD_Pin))
	  {
		  BYTE readBuf[100];
		  TCHAR* rres = f_gets((TCHAR*)readBuf, 100, &fil);
		  if(rres != 0) {
			  GcommandParse((TCHAR*)readBuf);
		  } else {
			f_close(&fil);
			f_mount(NULL, "", 0);
			while(1){}
		  }
	  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
  HAL_NVIC_SetPriority(EXTI0_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void InitiateMotors()
{
	__HAL_TIM_SET_PRESCALER(&htim16, 100);
	__HAL_TIM_SET_PRESCALER(&htim17, 100);

	// Starts the motor timers
	  HAL_TIM_Base_Start_IT(&htim16);
	  HAL_TIM_Base_Start_IT(&htim17);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == shutdownButton_Pin)
	{
		if (setup == 1)
		{
			HAL_TIM_Base_Stop_IT(&htim16);
			HAL_TIM_Base_Stop_IT(&htim17);

			feed = 5000;
			LaserEngrave(50, 50);
			Xcurrent = 0;
			Ycurrent = 0;
		}
		else
		{
			HAL_GPIO_WritePin(XEN_GPIO_Port, XEN_Pin, 1);
			HAL_GPIO_WritePin(YEN_GPIO_Port, YEN_Pin, 1);

			HAL_TIM_Base_Stop_IT(&htim16);
			HAL_TIM_Base_Stop_IT(&htim17);

			while(1){}
		}
	}
}

void GcommandParse(TCHAR* line1)
{

	  // Creates local variables for the different possible commands in Gcode (max 10 characters)
	  char Gcommand[10] = "";
	  char Xcoordinate[10] = "";
	  char Ycoordinate[10] = "";
	  char Zcoordinate[10] = "";
	  char feedRate[10] = "";
	  char laserSpeed[10] = "";

	  int i = 0;
	  // Initiates a for loop which loops each character of the Gcode line
	  while (1)
	  {

		  // Creates a temporary variable for the Gcode command and the value attached to it
		  char command = line1[i];	// assigns the first value of the Gcode as the command
		  char newValue[10] = "";

		  int j = 0;
		  i++;

		  while ((line1[i] != ' ') && (line1[i] != '\0') && (line1[i] != '\n'))	// while loop that loops through the rest of the command and stores the value in newValue
		  {

			  newValue[j] = line1[i];
			  i++;
			  j++;
		  }
		  newValue[j] = '\0';	//adds the null operator to the end of the newValue

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
		  if (line1[i] == '\n')
			  break;
		  i++;
	  }
	  if (strcmp(Gcommand,"G")){
		  GcommandExecute(Gcommand, Xcoordinate, Ycoordinate, Zcoordinate, feedRate, laserSpeed);	// Calls the Gcommand Execute function which will execute the given command
		  myprintf("Gcommand: %s Xcoordinate: %s Ycoordinate: %s Zcoordinate: %s feedRate: %s laserSpeed %s\n", Gcommand, Xcoordinate, Ycoordinate, Zcoordinate, feedRate, laserSpeed);
	  }
	  else if (strcmp(Gcommand,"M")){
		  McommandExecute(Gcommand);
	  }
}

// function which adds a char to the end of a char array
void addChar(char *s, char c)
{
	while (*s++);

	*(s - 1) = c;

	*s = '\0';
}

// Command Execute takes the parameters from the Gcode line and controls the motors accordingly
void GcommandExecute(char Gcommand[], char Xcommand[], char Ycommand[], char Zcommand[], char feedRate[], char laserSpeed[])
{

	if ((!strcmp(Gcommand,"0")) || (!strcmp(Gcommand,"1")))	// If the Gcode command is G0, runs with rapid positioning (full speed move)
	{
		if (Xcommand[0] != '\0')
		{
			Xend = ((1600 / 43.39) * atof(Xcommand));	// Converts Xcommand to an int, changes units to .1 mms and updates the global variable
		} else
		{
			Xend = Xcurrent;
		}
		if (Ycommand[0] != '\0')
		{
			Yend = ((1600 / 43.39) * atof(Ycommand));	// Converts Ycommand to an int, changes units to .1 mms and updates the global variable
		} else
		{
			Yend = Ycurrent;
		}

		// Calculates distance to be traveled
		float Xdistance = Xend - Xcurrent;
		float Ydistance = Yend - Ycurrent;

		// Updates the Direction variable and writes to the pin
		if (Xdistance > 0)
		{
			XDIR = 1;
			HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, 1);
		} else if (Xdistance < 0)
		{
			XDIR = 0;
			HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, 0);
		}

		if (Ydistance > 0)
		{
			YDIR = 1;
			HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, 1);
		} else if (Ydistance < 0)
		{
			YDIR = 0;
			HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, 0);
		}

		int laser = atoi(laserSpeed);	// Converts laserSpeed to an int

		if (feedRate[0] != '\0')
		{
			feed = atoi(feedRate);	// Converts feedRate to an int
		}

		LaserEngrave(Xdistance, Ydistance);	// Calls the laserEngrave function

	}

	// We'll need to add all of the G commands here

}

void McommandExecute(char Mcommand[])
{
	if ((Mcommand[1] == '5') && Mcommand[2] == '\0')
	{
		// Add code that turns the laser off
	}
	if ((Mcommand[1] == '3') && ((Mcommand[2] == '\0') || (Mcommand[2] == ' ')))
	{
		// Add code to turn laser on and PWM of value specified
	}
}


void LaserEngrave(int Xdistance, int Ydistance)
{
	int Xspeed = 65535;
	int Yspeed = 65535;

	float totalDistance = sqrt((Xdistance * Xdistance) + (Ydistance * Ydistance));
	if (Xdistance != 0)
	{
		Xspeed = 162712.482 / ((abs(Xdistance) / totalDistance) * feed);
	}
	if (Ydistance != 0)
	{
		Yspeed = 162712.482 / ((abs(Ydistance) / totalDistance) * feed);
	}

	__HAL_TIM_SET_PRESCALER(&htim16, Xspeed);
	__HAL_TIM_SET_PRESCALER(&htim17, Yspeed);

	// Starts the motor timers
	  HAL_TIM_Base_Start_IT(&htim16);
	  HAL_TIM_Base_Start_IT(&htim17);
	  // Here is where the PWM starts

	  while(((Xcurrent == Xend) && (Ycurrent == Yend)) == 0){}	// Waits for the motors to be done before proceeding
}

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, 0xFFFF);
	return ch;
}

/* USER CODE END 4 */

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
				if (XDIR == 1)
				{
					Xcurrent++;
				}
				else
				{
					Xcurrent--;
				}
			}
		}
		else
		{
			HAL_TIM_Base_Stop_IT(&htim16);	// Once the X motor arrives to it's final position, this stops the timer
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
				if (YDIR == 1)
				{
					Ycurrent++;
				}
				else
				{
					Ycurrent--;
				}
			}
		}
		else
		{
		  HAL_TIM_Base_Stop_IT(&htim17);	// Once the Y motor arrives to it's final position, this stops the timer
		}
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
