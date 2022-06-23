/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 * Autorzy: Paweł Dyrda, Michał Zieliński
 * Projekt stroika do instrumentów muzycznych.
 * Możliwe jest strojenie za pomocą mikrofonu lub wejścia Jack 3.5mm (możliwa
 * konieczność zastowowania przejściówki). Wynik wraz z informacją o konieczności
 * dostrojenia wraz z kierunkiem będzie wyświetlany na wyświetlaczu LCD.
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "fft.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct FLAGS {
	bool JACK;
	bool FFT_DONE;
} FLAGS;

//typedef struct PITCH {
//	float E2;
//	float A2;
//	float D3;
//	float G3;
//	float B3;
//	float E4;
//} PITCH;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* FFT settings */
#define FFT_SIZE	2048
#define A4			440

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Global variables */
//float Input[SAMPLES];
//float Output[FFT_SIZE];
FLAGS flags = {
		true, false
};
//PITCH pitch = {
//		82,
//		110,
//		147,
//		196,
//		247,
//		330
//};

Lcd_PortType ports[] = {
		D0_GPIO_Port, D1_GPIO_Port, D2_GPIO_Port, D3_GPIO_Port, D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port
};
GPIO_PinState prevStateBtn = GPIO_PIN_SET, currStateBtn = GPIO_PIN_SET;
Lcd_HandleTypeDef lcd;
Lcd_PinType pins[] = {D0_Pin, D1_Pin, D2_Pin, D3_Pin, D4_Pin, D5_Pin, D6_Pin, D7_Pin};
int samples[FFT_SIZE];
int pitchOffset = 0;
const char * NOTES[] = {
		"A",
		"A#",
		"B",
		"C",
		"C#",
		"D",
		"D#",
		"E",
		"F",
		"F#",
		"G",
		"G#"
};
int counter = 0;
double result = 0, previousResult = 0;
double halfSteps = 0;
bool FFTDone = false;
char higherNotLower = '+';

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void calculatePitch();
void sendDebug();
void printResult();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
	if (ch == '\n') {
		__io_putchar('\r');
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

	return 1;
}

void selectChannelADC(uint32_t channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void updateButton() {
	prevStateBtn = currStateBtn;
	currStateBtn = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

	if (currStateBtn != prevStateBtn && prevStateBtn == GPIO_PIN_SET) {
		flags.JACK = !flags.JACK;
		if(flags.JACK){
			selectChannelADC(ADC_CHANNEL_1);
		}else {
			selectChannelADC(ADC_CHANNEL_0);
		}
	}
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
	MX_ADC1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	selectChannelADC(ADC_CHANNEL_1);
	lcd = Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin, LCD_8_BIT_MODE);
	HAL_TIM_Base_Start_IT(&htim3);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//	  printf("%s", "test");
		if(FFTDone){
			printResult();

			calculatePitch();
			sendDebug();
		}
		//	  HAL_Delay(150);

		updateButton();

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
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
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
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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

	//  ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	//  sConfig.Channel = ADC_CHANNEL_1;
	//  sConfig.Rank = 1;
	//  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	//  {
	//    Error_Handler();
	//  }
	//
	//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	//  */
	//  sConfig.Channel = ADC_CHANNEL_0;
	//  sConfig.Rank = 2;
	//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	//  {
	//    Error_Handler();
	//  }
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 19;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 94;
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
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, D0_Pin|D1_Pin|D3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin|EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, D4_Pin|D6_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, D5_Pin|RS_Pin|D7_Pin|D2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : D0_Pin D1_Pin D3_Pin */
	GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin EN_Pin */
	GPIO_InitStruct.Pin = LD2_Pin|EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : D4_Pin D5_Pin RS_Pin D7_Pin
                           D6_Pin D2_Pin */
	GPIO_InitStruct.Pin = D4_Pin|D5_Pin|RS_Pin|D7_Pin
			|D6_Pin|D2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

double complexAbs(double complex z) {
	return sqrt(creal(z) * creal(z) + cimag(z) * cimag(z));
}

void doFFT() {

	previousResult = result;

	double complex samplesAfterFFT[FFT_SIZE];

	fft(samples, samplesAfterFFT, FFT_SIZE);

	double max = 0;
	uint32_t maxElementIndex = 1; //zaczynamy od 1, bo nie interesuje nas prazek 0 reprezentujacy napiecie stale
	for (int i = 1; i < FFT_SIZE; i++) {
		if (complexAbs(samplesAfterFFT[i]) > max) {
			max = complexAbs(samplesAfterFFT[i]);
			maxElementIndex = i;
		}
	}
	//44210.52 to dokladna czestotliwosc probkowania
	result = 44210.52 / (double) FFT_SIZE * (double) maxElementIndex;
	if (result > 10000) result = previousResult;

	if(max <4000){
		result=0;
	}
	FFTDone = true;
}

uint32_t getSample() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(&hadc1)-1200;
}

//próbkowanie z fs=44kHz
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {
		if (counter < FFT_SIZE && !FFTDone) {
			uint32_t sample = getSample();
			if (abs(sample) < 100){
				sample = 0;
			}
			samples[counter] = sample;

			counter++;
		}
		if (counter == FFT_SIZE) {
			doFFT();
			counter = 0;
		}
	}
}

void printResult() {
	char buffer[16];

	Lcd_clear(&lcd);
	if(result!=0){
		sprintf(buffer, "%s %c (%.3f Hz)", NOTES[pitchOffset], higherNotLower, result);
		Lcd_cursor(&lcd, 0, 0);
		Lcd_string(&lcd, buffer);
	}
	if (flags.JACK){
		sprintf(buffer, "JACK");
	} else {
		sprintf(buffer, "MICROPHONE");
	}
	Lcd_cursor(&lcd, 1, 0);
	Lcd_string(&lcd, buffer);
	FFTDone = false;
}

void calculatePitch(){
	halfSteps = 12 * log2(result/A4);

	int roundedHalfSteps = (int)round(halfSteps);

	if (abs(roundedHalfSteps) < abs(halfSteps)) {
		higherNotLower = '-';
	}
	else if (abs(roundedHalfSteps) > abs(halfSteps)) {
		higherNotLower = '+';
	}
	else if (roundedHalfSteps - halfSteps < 0.5) {
		higherNotLower = '0';
	}

	pitchOffset = roundedHalfSteps % 12;

	if (pitchOffset < 0) pitchOffset += 12;
}

void sendDebug() {
	char buffer[16];
//	char buffer2[128];

	if (flags.JACK){
			sprintf(buffer, "JACK");
		} else {
			sprintf(buffer, "MICROPHONE");
		}

//	sprintf(buffer2, "DEBUG INFO--------------------\n Detected freq: %.3f | Calculated half-steps: %.2f \n", result, halfSteps);
//	HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, strlen(buffer2), HAL_MAX_DELAY);
//	sprintf(buffer2, "Closest pitch: %s\n | Connection type: %s", NOTES[pitchOffset], buffer);
//	HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, strlen(buffer2), HAL_MAX_DELAY);

	printf("DEBUG INFO--------------------\n Detected freq: %.3f | Calculated half-steps: %.2f \n", result, halfSteps);
	printf("Closest pitch: %s\n | Connection type: %s", NOTES[pitchOffset], buffer);
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
