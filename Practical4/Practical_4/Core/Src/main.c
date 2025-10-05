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
#include "stm32f4xx.h"
#include "lcd_stm32f4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Configuration values
#define NS             256U           // Number of samples in LUT (>= 128)
#define TIM2CLK        16000000UL     // TIM2 clock frequency (see .ioc)
#define F_SIGNAL       440UL          // Output analog signal frequency (Hz)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// Lookup tables (12-bit: 0..4095)
uint32_t Sin_LUT[NS];
uint32_t Saw_LUT[NS];
uint32_t Triangle_LUT[NS];
uint32_t Piano_LUT[NS];
uint32_t Guitar_LUT[NS];
uint32_t Drum_LUT[NS];

// Current waveform selection
static volatile uint8_t g_wave_index = 0;  // 0..5
static volatile uint32_t *g_current_lut = Sin_LUT;





// Equation to calculate TIM2_Ticks: TIM2CLK / (NS * F_SIGNAL)
uint32_t TIM2_Ticks = (uint32_t)((TIM2CLK + (NS * F_SIGNAL / 2)) / (NS * F_SIGNAL)); // rounded
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_IRQHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include <math.h>

static void Generate_LUTs(void){
  for (uint32_t i = 0; i < NS; i++){
    float phase = (2.0f * 3.14159265358979f * (float)i) / (float)NS;

    // Sine 0..4095
    float s = sinf(phase);
    Sin_LUT[i] = (uint32_t)(2047.5f * (s + 1.0f));

    // Saw 0..4095
    Saw_LUT[i] = (uint32_t)((4095.0f * (float)i) / (float)(NS - 1));

    // Triangle 0..4095
    if (i < (NS/2)){
      Triangle_LUT[i] = (uint32_t)((4095.0f * (2.0f * (float)i)) / (float)(NS - 1));
    } else {
      float j = (float)(i - NS/2);
      Triangle_LUT[i] = (uint32_t)(4095.0f - (4095.0f * (2.0f * j)) / (float)(NS - 1));
    }

    // Placeholder musical timbres (replace with .wav-derived tables for Task 1.2)
    float piano = 0.8f * sinf(phase) + 0.2f * sinf(2.0f * phase);
    Piano_LUT[i] = (uint32_t)(2047.5f * (piano + 1.0f));

    float guitar = 0.7f * sinf(phase) + 0.2f * sinf(3.0f * phase) + 0.1f * sinf(5.0f * phase);
    Guitar_LUT[i] = (uint32_t)(2047.5f * (guitar + 1.0f));

    float drum = 0.6f * sinf(phase) + 0.25f * sinf(2.7f * phase) + 0.15f * sinf(5.3f * phase);
    Drum_LUT[i] = (uint32_t)(2047.5f * (drum + 1.0f));
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Boot indicator: brief PB7 blinks using direct registers (no HAL)
  // Enable GPIOB clock
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  // Set PB7 as output
  GPIOB->MODER &= ~(GPIO_MODER_MODER7);
  GPIOB->MODER |= GPIO_MODER_MODER7_0;
  // Blink a few times
  for (volatile uint32_t k = 0; k < 3; ++k){
    // PB7 high
    GPIOB->BSRR = GPIO_BSRR_BS7;
    for (volatile uint32_t d = 0; d < 200000; ++d) { __NOP(); }
    // PB7 low
    GPIOB->BSRR = GPIO_BSRR_BR7;
    for (volatile uint32_t d = 0; d < 200000; ++d) { __NOP(); }
  }

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Generate_LUTs();
  g_wave_index = 0;
  g_current_lut = Sin_LUT;

  // Initialize LCD and print waveform name
  init_LCD();
  lcd_command(CLEAR);
  lcd_putstring("Wave: ");
  lcd_putstring((char*)WaveName(g_wave_index));

  // Start PWM on TIM3 CH3
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }

  // Start TIM2 OC on CH1
  if (HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

  // Start DMA circular transfer: LUT -> TIM3->CCR3
  if (HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)g_current_lut, DestAddress, NS) != HAL_OK) { Error_Handler(); }

  // Enable TIM2 CC1 DMA request
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Heartbeat blink every ~250ms
    static uint32_t last_toggle = 0;
    uint32_t now = HAL_GetTick();
    if ((now - last_toggle) >= 250U) {
      last_toggle = now;
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* TIM2_CH1 DMA Init */
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;         // TIM2_CH1 is on channel 3
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH; // Memory -> TIM3->CCR3
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;    // Peripheral address fixed
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;        // Memory address increments
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;            // Repeat LUT automatically
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Link DMA handle to TIM2 handle */
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4095; // 12-bit resolution
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // -------------------------------
  // LCD pins configuration
  // -------------------------------
  // Configure PC14 (RS) and PC15 (E) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure PB8 (D4) and PB9 (D5) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure PA12 (D6) and PA15 (D7) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Set all LCD pins LOW initially
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);


  // -------------------------------
  // Button0 configuration (PA0)
  // -------------------------------
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge
  GPIO_InitStruct.Pull = GPIO_PULLUP;         // Use pull-up resistor
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable and set EXTI line 0 interrupt priority
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  // -------------------------------
  // Heartbeat LED on PB7 (unused pin)
  // -------------------------------
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void){

	// Debounce using HAL_GetTick()
	static uint32_t last = 0;
	uint32_t now = HAL_GetTick();
	if ((now - last) < 150) { // ~150ms debounce
		HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
		return;
	}
	last = now;

	// Disable DMA request and abort current transfer
	__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
	HAL_DMA_Abort(&hdma_tim2_ch1);

	// Cycle waveform index and select new LUT
	g_wave_index = (uint8_t)((g_wave_index + 1) % 6);
	switch(g_wave_index){
		case 0: g_current_lut = Sin_LUT; break;
		case 1: g_current_lut = Saw_LUT; break;
		case 2: g_current_lut = Triangle_LUT; break;
		case 3: g_current_lut = Piano_LUT; break;
		case 4: g_current_lut = Guitar_LUT; break;
		case 5: g_current_lut = Drum_LUT; break;
		default: g_current_lut = Sin_LUT; break;
	}

	// Update LCD with current waveform name
	lcd_command(CLEAR);
	lcd_putstring("Wave: ");
	lcd_putstring((char*)WaveName(g_wave_index));

	// Restart DMA with the new source LUT and re-enable TIM2 CC1 DMA
	if (HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)g_current_lut, DestAddress, NS) != HAL_OK) {
		Error_Handler();
	}
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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
#ifdef USE_FULL_ASSERT
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
