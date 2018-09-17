/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

osThreadId RampHandle;
osThreadId VerifyHandle;
osThreadId CounterHandle;
osThreadId SerialHandle;
osThreadId OverCurrentHandle;
osThreadId ComecaHandle;
osThreadId RPMHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void RampTask(void const * argument);
void VerifyTask(void const * argument);
void CounterTask(void const * argument);
void SerialTask(void const * argument);
void OverCurrentTask(void const * argument);
void Comeca_Task(void const * argument);
void RPMTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//! If it's on the soft is in a ramp
char fg_Start = 0;

//! Shows if the soft is idle
char fg_End = 1;
char fg_ADC_current = 0;

//! Is the acceleration time
int tempo_up = 10000; //Tem que ser em ms e ser substituido pelo valor recebido pela serial.

//! Deceleration time
int tempo_down = 7000; //Tem que ser em ms e ser substituido pelo valor recebido pela serial.

//! time to the counter
float tempo; //Tem que ser em ms e ser substituido pelo valor recebido pela serial.

//! Delay to the pulse on triac
int PULSE_DELAY = 8330;	//Tempo em us

//! Width of the triac's pulse
int PULSE_WIDTH = 1;	//Tempo em us
int PULSE_WIDTH_CONST = 1000;	//Tempo em us

//Minimal delay time to triac
int minimal_delay = 500;

float delay;

//! Defines if it's a acceleration or deceleration ramp
int RampUp = 1;

//! Is the RPM of the motor
int RPM;

int counter;

int Irms;
int Irms_over = 1400; //� a corrente do motor quando o eixo for travado - tem que medir e mudar aqui na vari�vel
int Vrms;
int Vrms_over = 220;  // � a tens�o de sobrecorrente
float relation = 1.4325;

int corrente_nominal = 400;

//! Indicates if the soft shall start through the serial
int fg_serial = 0;

uint16_t adc_current[16];

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	fg_ADC_current = 1;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of Ramp */
	osThreadDef(Ramp, RampTask, osPriorityNormal, 0, 128);
	RampHandle = osThreadCreate(osThread(Ramp), NULL);

	/* definition and creation of Verify */
	osThreadDef(Verify, VerifyTask, osPriorityNormal, 0, 128);
	VerifyHandle = osThreadCreate(osThread(Verify), NULL);

	/* definition and creation of Counter */
	osThreadDef(Counter, CounterTask, osPriorityNormal, 0, 128);
	CounterHandle = osThreadCreate(osThread(Counter), NULL);

	/* definition and creation of Serial */
	osThreadDef(Serial, SerialTask, osPriorityNormal, 0, 128);
	SerialHandle = osThreadCreate(osThread(Serial), NULL);

	/* definition and creation of OverCurrent */
	osThreadDef(OverCurrent, OverCurrentTask, osPriorityNormal, 0, 128);
	OverCurrentHandle = osThreadCreate(osThread(OverCurrent), NULL);

	/* definition and creation of Comeca */
	osThreadDef(Comeca, Comeca_Task, osPriorityRealtime, 0, 128);
	ComecaHandle = osThreadCreate(osThread(Comeca), NULL);

	/* definition and creation of RPM */
	osThreadDef(RPM, RPMTask, osPriorityNormal, 0, 128);
	RPMHandle = osThreadCreate(osThread(RPM), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void) {

	TIM_SlaveConfigTypeDef sSlaveConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 9999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM2 init function */
static void MX_TIM2_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_SlaveConfigTypeDef sSlaveConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 83;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 8333;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 83;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 520;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM4 init function */
static void MX_TIM4_Init(void) {

	TIM_SlaveConfigTypeDef sSlaveConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 9999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(bypass_relay_GPIO_Port, bypass_relay_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, Green_LED_Pin | Yellow_LED_Pin | Red_LED_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : Button_Pin */
	GPIO_InitStruct.Pin = Button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : bypass_relay_Pin */
	GPIO_InitStruct.Pin = bypass_relay_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(bypass_relay_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Green_LED_Pin Yellow_LED_Pin Red_LED_Pin */
	GPIO_InitStruct.Pin = Green_LED_Pin | Yellow_LED_Pin | Red_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* RampTask function */
void RampTask(void const * argument) {

	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		if (__HAL_TIM_GET_COUNTER(&htim2) <= (PULSE_DELAY - 500)) {
			__HAL_TIM_SET_AUTORELOAD(&htim2, PULSE_DELAY + PULSE_WIDTH - 1);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PULSE_DELAY);

		}
		osDelay(25);
	}
	/* USER CODE END 5 */
}

/* VerifyTask function */
void VerifyTask(void const * argument) {
	/* USER CODE BEGIN VerifyTask */
	/* Infinite loop */

	for (;;) {
		if ((HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin) == GPIO_PIN_RESET
				|| fg_serial) && fg_End == 1) {

			fg_serial = 0;
			fg_Start = 1;
			fg_End = 0;
			if (RampUp == 1) {
				tempo = tempo_up;
				counter = tempo;
			} else {
				HAL_GPIO_WritePin(bypass_relay_GPIO_Port, bypass_relay_Pin,
						GPIO_PIN_RESET);
				tempo = tempo_down;
				counter = 0;
			}
		}

		if (Irms >= 0 && Irms < corrente_nominal * 1.5) {
			HAL_GPIO_WritePin(GPIOC, Green_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, Yellow_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, Red_LED_Pin, GPIO_PIN_RESET);
		}

		if (Irms > corrente_nominal * 1.5 && Irms < corrente_nominal * 2) {
			HAL_GPIO_WritePin(GPIOC, Green_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, Yellow_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, Red_LED_Pin, GPIO_PIN_RESET);
		}

		if (Irms > corrente_nominal * 2) {
			HAL_GPIO_WritePin(GPIOC, Green_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, Yellow_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, Red_LED_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(bypass_relay_GPIO_Port, bypass_relay_Pin,
					GPIO_PIN_RESET);
			counter = 1;
			RampUp = 1;
			fg_Start = 0;
			fg_End = 1;
			PULSE_DELAY = 8330;
			PULSE_WIDTH = 1;
		}

		osDelay(20);
	}
	/* USER CODE END VerifyTask */
}

/* CounterTask function */
void CounterTask(void const * argument) {
	/* USER CODE BEGIN CounterTask */
	/* Infinite loop */
	for (;;) {

		//Verifies if acceleration ramp is over
		if (RampUp == 1 && counter <= 0) {
			HAL_GPIO_WritePin(bypass_relay_GPIO_Port, bypass_relay_Pin,
					GPIO_PIN_SET);
			fg_Start = 0;
			fg_End = 1;
			RampUp = !RampUp;
		}

		//Verifies if deceleration ramp is over
		if (RampUp == 0 && counter >= tempo) {
			fg_Start = 0;
			fg_End = 1;
			RampUp = !RampUp;
		}
		if (fg_Start) {
			if (RampUp == 1) {
				counter = counter - 2;
				if (HAL_GPIO_ReadPin(GPIOC, Yellow_LED_Pin) == GPIO_PIN_SET)
					counter = counter + 2;
			}
			if (RampUp == 0)
				counter = counter + 2;

			//delay = (counter / tempo * (8333-PULSE_WIDTH-minimal_delay));
			delay = (counter / tempo * (8333 - minimal_delay));
			PULSE_DELAY = delay + minimal_delay;

		}

		if ((PULSE_DELAY >= (8333 - PULSE_WIDTH) && RampUp == 0))
			PULSE_WIDTH = (8333 - PULSE_DELAY);
		if ((PULSE_WIDTH < PULSE_WIDTH_CONST && RampUp == 1))
			PULSE_WIDTH = (8333 - PULSE_DELAY);

		osDelay(2);
	}
	/* USER CODE END CounterTask */
}

/* SerialTask function */
void SerialTask(void const * argument) {
	/* USER CODE BEGIN SerialTask */
	/* Infinite loop */
	char rx, tempo_serial[6];
	char texto[100];
	int fg_status = 0;
	int size, aux;
	int numero;

	for (;;) {

		//Sends an overview of the current status
		if (fg_status) {
			//size =sprintf(texto,"Counter = %d	Tempo de delay = %d	Fg_start = %d	RPM = %d	\n",counter, PULSE_DELAY, fg_Start, RPM);
			size = sprintf(texto, "Irms = %d\n", Irms);
			HAL_UART_Transmit(&huart2, texto, size, 100);
			osDelay(100);
		}

		if (HAL_UART_Receive(&huart2, tempo_serial, 3, 1) == HAL_OK) {

			if (tempo_serial[0] == 'u') {
				tempo_serial[0] = tempo_serial[1];
				tempo_serial[1] = tempo_serial[2];
				tempo_serial[2] = 0;
				tempo_serial[3] = 0;

				tempo_up = atoi(tempo_serial);
				tempo_up *= 1000;
				fg_serial = 1;
			}

			if (tempo_serial[0] == 'd') {
				tempo_serial[0] = tempo_serial[1];
				tempo_serial[1] = tempo_serial[2];
				tempo_serial[2] = 0;
				tempo_serial[3] = 0;

				tempo_down = atoi(tempo_serial);
				tempo_down *= 1000;
				fg_serial = 1;
			}

			if (tempo_serial[0] == '$') {
				size = sprintf(texto, "I%i;V%i;R%i\n", Irms, Vrms, RPM);
				HAL_UART_Transmit(&huart2, texto, size, 100);
			}

		}

		//Receives a single char from serial
		//HAL_UART_Receive(&huart2, &rx, 1, 1);
		//osDelay(1);

		//u stands for changing the acceleration time
		if (rx == 'u') {
			if (HAL_UART_Receive(&huart2, tempo_serial, 2, 5000) == HAL_OK) {
				tempo_up = atoi(tempo_serial);
				tempo_up *= 1000;

				fg_serial = 1;
//				if (RampUp == 1 && fg_End == 1) {
//					tempo = tempo_up;
//					counter = tempo;
//					fg_Start = 1;
//					fg_End = 0;
//				}

			} else {
				size = sprintf(texto, "TIMEOUT - Acceleration Time\n");
				HAL_UART_Transmit(&huart2, texto, size, 100);
			}
		}

		//d stand for changing the deceleration time
		if (rx == 'd') {
			if (HAL_UART_Receive(&huart2, tempo_serial, 2, 5000) == HAL_OK) {
				tempo_down = atoi(tempo_serial);
				tempo_down *= 1000;

				fg_serial = 1;

//				fg_Start = 1;
//				fg_End = 0;
//				HAL_GPIO_WritePin(bypass_relay_GPIO_Port, bypass_relay_Pin,
//						GPIO_PIN_RESET);

				//counter = 0;

			} else {
				size = sprintf(texto, "TIMEOUT - Deceleration Time\n");
				HAL_UART_Transmit(&huart2, texto, size, 100);
			}
		}

		//t stand for toggling the status through the serial
		if (rx == 't') {
			if (fg_status == 0)
				fg_status = 1;
			else
				fg_status = 0;
		}

		//c stand for defining the new relation of current
		if (rx == 'c') {
			if (HAL_UART_Receive(&huart2, tempo_serial, 3, 3000) == HAL_OK) {
				relation = 1;
				osDelay(250);
				relation = Irms;
				relation /= atoi(tempo_serial);

				corrente_nominal = atoi(tempo_serial);

				//relation = Irms/atoi(tempo_serial);
			}
		}

		//h stand for printing a serial command list
		if (rx == 'h') {
			size =
					sprintf(texto,
							"LIST OF SERIAL COMMANDS:\nu - Changes the acceleration time\nd - changes the deceleration time\nt - toggles the status through the serial\nh - prints the help menu\n");
			HAL_UART_Transmit(&huart2, texto, size, 100);
		}

		if (rx == '#') {
			size = sprintf(texto, "u %i d %i\n", tempo_up, tempo_down);
			HAL_UART_Transmit(&huart2, texto, size, 100);
		}

		rx = 0;
		osDelay(1);
	}
	/* USER CODE END SerialTask */
}

/* OverCurrentTask function */
void OverCurrentTask(void const * argument) {
	/* USER CODE BEGIN OverCurrentTask */
	/* Infinite loop */
	float ADC_value[16];
	int index = 0;
	int aux;
	int result;
	int rms;
	int Medidas_ADC = 16;
	int sum[32];

	for (;;) {
		if (fg_ADC_current) {
			fg_ADC_current = 0;

			for (aux = 0; aux < Medidas_ADC; aux++) { //Faz com que o valor seja de 311 de pico com 2v na entrada
				//ADC_value[aux] = (adc_current[aux] / relation); //Rela��o entre 311 e o valor do AD pra 2v
				ADC_value[aux] = (adc_current[aux]); //Rela��o entre 311 e o valor do AD pra 2v
			}

			rms = 0;
			for (aux = 0; aux < Medidas_ADC; aux++) {
				rms += pow(ADC_value[aux], 2);
			}

			rms = sqrt((rms / Medidas_ADC));

			rms /= relation;

			if (index > 31) {
				index = 0;
				Irms = 0;
				for (aux = 0; aux < 32; aux++) {
					Irms += sum[aux];
				}
				Irms /= 32;
				Irms -= 5;
				if (Irms < 0)
					Irms = 0;
			}

			sum[index] = rms;
			index++;
			//quando der overcurrent mandar um "i" via serial (S� O CARACTER)
			//quando der overvoltage mandar um "v" via serial (S� O CARACTER)

		}

		osDelay(10);
	}
	/* USER CODE END OverCurrentTask */
}

/* Comeca_Task function */
void Comeca_Task(void const * argument) {
	/* USER CODE BEGIN Comeca_Task */
	/* Infinite loop */
	char texto[50];
	int size;

	tempo = tempo_up;
	counter = tempo;

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_current, 16); //N�O CONSIGO FAZER ESSA PORRA FUNCIONAR!!!!!
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start(&htim1);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	__HAL_TIM_SET_AUTORELOAD(&htim2, PULSE_DELAY + PULSE_WIDTH - 1);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PULSE_DELAY);

	size = sprintf(texto, "Iniciando perifericos");
	HAL_UART_Transmit(&huart2, texto, size, 100);

	vTaskSuspend(ComecaHandle);

	size = sprintf(texto, "fechando perifericos");
	HAL_UART_Transmit(&huart2, texto, size, 100);

	for (;;) {
		osDelay(1);
	}
	/* USER CODE END Comeca_Task */
}

/* RPMTask function */
void RPMTask(void const * argument) {
	/* USER CODE BEGIN RPMTask */
	/* Infinite loop */
	int size;
	char texto[100];
	for (;;) {
		RPM = __HAL_TIM_GET_COUNTER(&htim1);
		RPM *= 60;

		size = sprintf(texto, "RPM: %d\n", RPM);
		//HAL_UART_Transmit(&huart2, texto, size, 100);

		__HAL_TIM_SET_COUNTER(&htim1, 0);

		osDelay(1000);
	}
	/* USER CODE END RPMTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM11 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM11) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
