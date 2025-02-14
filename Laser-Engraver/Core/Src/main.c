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
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int Xcurrent = 0; // current X coordinate of laser (units of .1mm)
int Ycurrent = 0; // current Y coordinate of laser (units of .1mm)
int XDIR = 1;	// X motor Direction (1 is increasing X coordinate, 0 is decreasing)
int YDIR = 1;	// Y motor Direction (1 is increasing Y coordinate, 0 is decreasing)
int Xend = 0; // End coordinate for the X motor (units of .1mm)
int Yend = 0; // End coordinate for the Y motor (units of .1mm)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
void MotorStraightLine();
void GcommandExecute(char[], char[], char[], char[], char[], char[]);
void GcommandParse(char[]);
void addChar(char*,char);
void laserEngrave(int, int);
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
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
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(XEN_GPIO_Port, XEN_Pin,0);
  HAL_GPIO_WritePin(YEN_GPIO_Port, YEN_Pin,0);

  // Tests the GcommandParse function
  //GcommandParse("G0 X0.0000 Y0.0000 Z1.5000 F5000 S0\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Test code to turn both motors forward 10000 steps with prescaler at 100, then back with half speed with .5 second delays in between
	  Xend = 10000;
	  Yend = 10000;
	  XDIR = 1;	// sets the global variable for XDIR to 0 (decreasing X coordinate
	  YDIR = 1;	// sets the gobal variable for YDIR to 0 (decreasing Y coordinate
	  __HAL_TIM_SET_PRESCALER(&htim16,100);	// sets the prescaler to 100 (1 revolution per second)
	  __HAL_TIM_SET_PRESCALER(&htim17,100);
	  HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin,1);	// Sets the direction to decreasing X and Y coordinate
	  HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin,1);

	  // Starts the timers to pulse the X and Y motors
	  HAL_TIM_Base_Start_IT(&htim16);
	  HAL_TIM_Base_Start_IT(&htim17);

	  while((Xcurrent != Xend) && (Ycurrent != Yend)){} // Waits for the motors to be done
	  HAL_Delay(500);	// Creates a .5 second delay to pause the motors

	  // Repeats the process but changes the direction and halves the speed
	  Xend = 0;
	  Yend = 0;
	  XDIR = 0;
	  YDIR = 0;
	  __HAL_TIM_SET_PRESCALER(&htim16, 50);
	  __HAL_TIM_SET_PRESCALER(&htim17, 50);
	  HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin,1);
	  HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin,1);


	  HAL_TIM_Base_Start_IT(&htim16);
	  HAL_TIM_Base_Start_IT(&htim17);
	  while((Xcurrent != Xend) && (Ycurrent != Yend)){}
	  HAL_Delay(500);

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
  htim16.Init.Period = 250;
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
  htim17.Init.Period = 250;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, YPUL_Pin|XDIR_Pin|XEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, YEN_Pin|YDIR_Pin|XPUL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin YEN_Pin YDIR_Pin XPUL_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|YEN_Pin|YDIR_Pin|XPUL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : YPUL_Pin XDIR_Pin XEN_Pin */
  GPIO_InitStruct.Pin = YPUL_Pin|XDIR_Pin|XEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void GcommandParse(char line1[])
{

	  // Creates local variables for the different possible commands in Gcode (max 10 characters)
	  char Gcommand[10] = "";
	  char Mcommand[10] = "";
	  char Xcoordinate[10] = "";
	  char Ycoordinate[10] = "";
	  char Zcoordinate[10] = "";
	  char feedRate[10] = "";
	  char laserSpeed[10] = "";

	  // Initiates a for loop which loops each character of the Gcode line
	  for (int i = 0; i < strlen(line1); i++)
	  {

		  // Creates a temporary variable for the Gcode command and the value attached to it
		  char command = line1[i];	// assigns the first value of the Gcode as the command
		  char newValue[10] = "";

		  int j = 0;
		  i++;

		  while (line1[i] != ' ' && line1[i] != '\0')	// while loop that loops through the rest of the command and stores the value in newValue
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
			  strncpy(Mcommand, newValue, 10);	// copies the value in newValue to the Mcommand variable
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
	  }

	  GcommandExecute(Gcommand, Xcoordinate, Ycoordinate, Zcoordinate, feedRate, laserSpeed);	// Calls the Gcommand Execute function which will execute the given command
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

	if (strcmp(Gcommand,"0") == 0)	// If the Gcode command is G0, runs with rapid positioning (full speed move)
	{

		Xend = 10 * atof(Xcommand);	// Converts Xcommand to an int, changes units to .1 mms and updates the global variable
		Yend = 10 * atof(Ycommand);	// Converts Ycommand to an int, changes units to .1 mms and updates the global variable
		int feed = atoi(feedRate);	// Converts feedRate to an int
		int laser = atoi(laserSpeed);	// Converts laserSpeed to an int
		laserEngrave(feed, laser);	// Calls the laserEngrave function

	}

	if (strcmp(Gcommand,"1")==0)	// Linear interpolation command
	{
	}

	// We'll need to add all of the G commands here

}


void laserEngrave(int feedRate, int laserSpeed)
{
	// Calculates distance to be traveled
	int Xdistance = Xend - Xcurrent;
	int Ydistance = Yend - Ycurrent;

	// Updates the Direction variable and writes to the pin
	if (Xdistance > 0)
	{
		XDIR = 1;
		HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, 1);
	} else
	{
		XDIR = 0;
		HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, 0);
	}

	if (Ydistance > 0)
	{
		YDIR = 1;
		HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, 1);
	} else
	{
		YDIR = 0;
		HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, 0);
	}

	// Updates the prescaler ( I need to put in a calculation based on speed into the prescaler value
	__HAL_TIM_SET_PRESCALER(&htim16,800);
	__HAL_TIM_SET_PRESCALER(&htim17,800);

	// Starts the motor timers
	  HAL_TIM_Base_Start_IT(&htim16);
	  HAL_TIM_Base_Start_IT(&htim17);

	  while((Xcurrent != Xend) && (Ycurrent != Yend)){}	// Waits for the motors to be done before proceeding
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
			if (XDIR == 1)
			{
				Xcurrent++;
			}
			else
			{
				Xcurrent--;
			}
		}
		else
		{
			HAL_TIM_Base_Stop_IT(&htim17);	// Once the X motor arrives to it's final position, this stops the timer
		}
	}

	if (htim == &htim17)	// Y motor timer
	{

		if (Ycurrent != Yend)	// Evaluates if the Y motor has arrived in it's position
		{

			HAL_GPIO_TogglePin(YPUL_GPIO_Port, YPUL_Pin);	// Toggles the YPUL pin

			// Increments the Ycurrent value if YDIR is positive and decrements if Ycurrent value is negative
			if (YDIR == 1)
			{
				Ycurrent++;
			}
			else
			{
				Ycurrent--;
			}
		}
		else
		{
		  HAL_TIM_Base_Stop_IT(&htim16);	// Once the Y motor arrives to it's final position, this stops the timer
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
