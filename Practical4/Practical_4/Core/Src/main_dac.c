/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_dac.c
  * @brief          : Alternate main using inbuilt DAC (DAC1_CH1 on PA4)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "stm32f4xx.h"
#include "lcd_stm32f4.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NS             256U           // LUT length (>=128)
#define TIM6CLK        16000000UL     // APB1 timer clock when HSI=16MHz
#define F_SIGNAL       440U           // Output tone frequency when using periodic LUTs
#define USE_SAMPLE_RATE_44100  0      // Set 1 to run at 44.1 kHz sample rate instead of NS*F_SIGNAL
#define DEBOUNCE_MS    120U
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;
DAC_HandleTypeDef hdac;

/* USER CODE BEGIN PV */
static volatile uint8_t g_wave_index = 0; // 0..5
static volatile uint32_t g_last_press_ms = 0;

static uint16_t Sin_LUT[NS];
static uint16_t Saw_LUT[NS];
static uint16_t Triangle_LUT[NS];
static uint16_t Piano_LUT[NS];
static uint16_t Guitar_LUT[NS];
static uint16_t Drum_LUT[NS];

static uint16_t* get_wave_ptr(uint8_t idx){
	switch(idx){
		case 0: return Sin_LUT;
		case 1: return Saw_LUT;
		case 2: return Triangle_LUT;
		case 3: return Piano_LUT;
		case 4: return Guitar_LUT;
		case 5: return Drum_LUT;
		default: return Sin_LUT;
	}
}

static const char* WaveName(uint8_t idx){
	switch(idx){
		case 0: return "Sine";
		case 1: return "Saw";
		case 2: return "Tri";
		case 3: return "Piano";
		case 4: return "Guitar";
		case 5: return "Drum";
		default: return "?";
	}
}

static void Generate_LUTs(void){
	for (uint32_t i = 0; i < NS; i++){
		float phase = (2.0f * 3.14159265358979f * (float)i) / (float)NS;
		// 12-bit 0..4095
		float s = sinf(phase);
		Sin_LUT[i] = (uint16_t)(2047.5f * (s + 1.0f));
		Saw_LUT[i] = (uint16_t)((4095.0f * (float)i) / (float)(NS - 1));
		if (i < NS/2){
			Triangle_LUT[i] = (uint16_t)((4095.0f * (2.0f * (float)i)) / (float)(NS - 1));
		} else {
			float j = (float)(i - NS/2);
			Triangle_LUT[i] = (uint16_t)(4095.0f - (4095.0f * (2.0f * j)) / (float)(NS - 1));
		}
		float piano = 0.8f * sinf(phase) + 0.2f * sinf(2.0f * phase);
		Piano_LUT[i] = (uint16_t)(2047.5f * (piano + 1.0f));
		float guitar = 0.7f * sinf(phase) + 0.2f * sinf(3.0f * phase) + 0.1f * sinf(5.0f * phase);
		Guitar_LUT[i] = (uint16_t)(2047.5f * (guitar + 1.0f));
		float drum = 0.6f * sinf(phase) + 0.25f * sinf(2.7f * phase) + 0.15f * sinf(5.3f * phase);
		Drum_LUT[i] = (uint16_t)(2047.5f * (drum + 1.0f));
	}
}

/* Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(uint32_t sample_rate);
static void MX_DAC1_Init(void);

/* Interrupt prototype */
void EXTI0_IRQHandler(void);

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	init_LCD();
	Generate_LUTs();
	g_wave_index = 0;
	lcd_command(CLEAR);
	lcd_putstring("Wave: ");
	lcd_putstring((char*)WaveName(g_wave_index));

	// Init DAC and TIM6
	MX_DAC1_Init();
	uint32_t sample_rate = USE_SAMPLE_RATE_44100 ? 44100UL : (NS * (uint32_t)F_SIGNAL);
	MX_TIM6_Init(sample_rate);

	// Start DAC channel 1
	if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK){ Error_Handler(); }
	// Enable DMA request on DAC without using NVIC IRQs (circular)
	if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)get_wave_ptr(g_wave_index), NS, DAC_ALIGN_12B_R) != HAL_OK){ Error_Handler(); }
	// Start TIM6 to trigger DAC
	if (HAL_TIM_Base_Start(&htim6) != HAL_OK){ Error_Handler(); }

	while (1){ /* idle */ }
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// LCD pins
	GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);

	// Button PA0 EXTI
	GPIO_InitStruct.Pin = Button0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	// DAC pin PA4 (DAC1_OUT1)
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_DAC1_Init(void)
{
	__HAL_RCC_DAC_CLK_ENABLE();
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK){ Error_Handler(); }
	DAC_ChannelConfTypeDef sConfig = {0};
	sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK){ Error_Handler(); }
}

static void MX_TIM6_Init(uint32_t sample_rate)
{
	__HAL_RCC_TIM6_CLK_ENABLE();
	htim6.Instance = TIM6;
	uint32_t psc = 0;
	uint32_t arr = (TIM6CLK / sample_rate) - 1U;
	if (arr > 0xFFFFU){
		psc = (arr / 0xFFFFU);
		if (psc > 0xFFFFU) psc = 0xFFFFU;
		uint32_t timclk_div = (psc + 1U);
		arr = (TIM6CLK / (timclk_div * sample_rate)) - 1U;
		if (arr > 0xFFFFU) arr = 0xFFFFU;
	}
	htim6.Init.Prescaler = (uint16_t)psc;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = (uint16_t)arr;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK){ Error_Handler(); }
	TIM_MasterConfigTypeDef sMaster = {0};
	sMaster.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMaster.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMaster) != HAL_OK){ Error_Handler(); }
}

void EXTI0_IRQHandler(void)
{
	uint32_t now = HAL_GetTick();
	if ((now - g_last_press_ms) < DEBOUNCE_MS){
		HAL_GPIO_EXTI_IRQHandler(Button0_Pin);
		return;
	}
	g_last_press_ms = now;

	g_wave_index = (uint8_t)((g_wave_index + 1) % 6);
	lcd_command(CLEAR);
	lcd_putstring("Wave: ");
	lcd_putstring((char*)WaveName(g_wave_index));

	// Restart DAC DMA with new buffer
	HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
	if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)get_wave_ptr(g_wave_index), NS, DAC_ALIGN_12B_R) != HAL_OK){
		Error_Handler();
	}

	HAL_GPIO_EXTI_IRQHandler(Button0_Pin);
}
