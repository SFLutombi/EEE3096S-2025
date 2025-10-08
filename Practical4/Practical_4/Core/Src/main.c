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
/*
Link: 
Group Number: 19
Group members: LTMSIM001 & MDLYAH001


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//include lookup tables
//#include "luts.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//for strlen in display_waveform
#include <string.h>
#include "stm32f4xx.h"
#include "lcd_stm32f4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
//Task 2
#define NS     128   // Number of samples in LUT
#define TIM2CLK  16000000 // STM Clock frequency: is 16MHz
#define F_SIGNAL  0.5	// Frequency of output analog signal is 100Hz target to help with noisy targets

//defining debounce delay for the button press in ms
#define DEBOUNCE_DELAY 200


//defining enum to easily track the waveform
typedef enum {
	WAVE_SINE, WAVE_SAW, WAVE_TRIANGLE, WAVE_PIANO, WAVE_GUITAR, WAVE_DRUM, NUM_WAVEFORMS
} WaveformType;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs
//Task 1
//these values were produced using lookup tables in python code, each has a minimum of 128 values
uint32_t Sin_LUT[NS] = {2048, 2148, 2248, 2348, 2447, 2545, 2642, 2737, 2831, 2923, 3013, 3100, 3185, 3267, 3346, 3423, 3495, 3565, 3630, 3692, 3750, 3804, 3853, 3898, 3939, 3975, 4007, 4034, 4056, 4073, 4085, 4093, 4095, 4093, 4085, 4073, 4056, 4034, 4007, 3975, 3939, 3898, 3853, 3804, 3750, 3692, 3630, 3565, 3495, 3423, 3346, 3267, 3185, 3100, 3013, 2923, 2831, 2737, 2642, 2545, 2447, 2348, 2248, 2148, 2048, 1947, 1847, 1747, 1648, 1550, 1453, 1358, 1264, 1172, 1082, 995, 910, 828, 749, 672, 600, 530, 465, 403, 345, 291, 242, 197, 156, 120, 88, 61, 39, 22, 10, 2, 0, 2, 10, 22, 39, 61, 88, 120, 156, 197, 242, 291, 345, 403, 465, 530, 600, 672, 749, 828, 910, 995, 1082, 1172, 1264, 1358, 1453, 1550, 1648, 1747, 1847, 1947};
uint32_t Saw_LUT[NS] = {0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448, 480, 512, 544, 576, 608, 640, 672, 704, 736, 768, 800, 832, 864, 896, 928, 960, 992, 1024, 1056, 1088, 1120, 1152, 1184, 1216, 1248, 1280, 1312, 1344, 1376, 1408, 1440, 1472, 1504, 1536, 1568, 1600, 1632, 1664, 1696, 1728, 1760, 1792, 1824, 1856, 1888, 1920, 1952, 1984, 2016, 2048, 2079, 2111, 2143, 2175, 2207, 2239, 2271, 2303, 2335, 2367, 2399, 2431, 2463, 2495, 2527, 2559, 2591, 2623, 2655, 2687, 2719, 2751, 2783, 2815, 2847, 2879, 2911, 2943, 2975, 3007, 3039, 3071, 3103, 3135, 3167, 3199, 3231, 3263, 3295, 3327, 3359, 3391, 3423, 3455, 3487, 3519, 3551, 3583, 3615, 3647, 3679, 3711, 3743, 3775, 3807, 3839, 3871, 3903, 3935, 3967, 3999, 4031, 4063};
uint32_t Triangle_LUT[NS] = {0, 64, 128, 192, 256, 320, 384, 448, 512, 576, 640, 704, 768, 832, 896, 960, 1024, 1088, 1152, 1216, 1280, 1344, 1408, 1472, 1536, 1600, 1664, 1728, 1792, 1856, 1920, 1984, 2048, 2111, 2175, 2239, 2303, 2367, 2431, 2495, 2559, 2623, 2687, 2751, 2815, 2879, 2943, 3007, 3071, 3135, 3199, 3263, 3327, 3391, 3455, 3519, 3583, 3647, 3711, 3775, 3839, 3903, 3967, 4031, 4095, 4031, 3967, 3903, 3839, 3775, 3711, 3647, 3583, 3519, 3455, 3391, 3327, 3263, 3199, 3135, 3071, 3007, 2943, 2879, 2815, 2751, 2687, 2623, 2559, 2495, 2431, 2367, 2303, 2239, 2175, 2111, 2048, 1984, 1920, 1856, 1792, 1728, 1664, 1600, 1536, 1472, 1408, 1344, 1280, 1216, 1152, 1088, 1024, 960, 896, 832, 768, 704, 640, 576, 512, 448, 384, 320, 256, 192, 128, 64};
uint32_t Piano_LUT[NS] = {4094, 4095, 4092, 4085, 4072, 4052, 4028, 4001, 3971, 3937, 3898, 3853, 3803, 3750, 3690, 3624, 3554, 3481, 3401, 3313, 3221, 3128, 3032, 2932, 2829, 2722, 2613, 2505, 2397, 2286, 2174, 2061, 1946, 1831, 1718, 1606, 1495, 1387, 1283, 1181, 1081, 985, 891, 799, 711, 628, 550, 477, 410, 346, 285, 233, 186, 146, 110, 79, 52, 30, 15, 6, 1, 0, 3, 12, 27, 47, 71, 100, 132, 169, 209, 254, 304, 360, 419, 479, 543, 608, 675, 745, 814, 883, 954, 1026, 1098, 1170, 1241, 1312, 1383, 1451, 1517, 1580, 1640, 1700, 1756, 1809, 1857, 1901, 1942, 1980, 2015, 2046, 2073, 2094, 2112, 2128, 2141, 2151, 2157, 2160, 2160, 2155, 2147, 2136, 2121, 2102, 2081, 2056, 2028, 2000, 1971, 1939, 1905, 1873, 1840, 1806, 1772, 1737};
uint32_t Guitar_LUT[NS] = {1524, 1425, 1321, 1201, 1076, 965, 874, 792, 716, 646, 583, 536, 515, 500, 471, 435, 411, 390, 363, 326, 279, 232, 202, 183, 159, 130, 105, 71, 35, 10, 2, 0, 8, 37, 92, 174, 276, 383, 469, 537, 614, 69};
uint32_t Drum_LUT[NS] = {
  0, 188, 150, 75, 274, 682, 951, 1091, 1338, 1456, 1489, 1924, 2381, 2182, 1612, 1075,
  838, 1209, 1730, 2031, 2617, 3289, 3214, 2655, 2494, 2966, 3423, 3176, 2649, 2590, 2816, 2945,
  2811, 2332, 2144, 2730, 3439, 3864, 4095, 3950, 3740, 3977, 3945, 3165, 2601, 2811, 3155, 3235,
  3203, 3122, 3117, 3278, 3461, 3321, 2778, 2322, 2203, 2064, 2176, 2983, 3773, 3998, 3842, 3267,
  2655, 2424, 2193, 2219, 2864, 3278, 3203, 3085, 2837, 2794, 2913, 2429, 2037, 2504, 2875, 2848,
  2880, 2606, 2053, 1671, 1537, 1945, 2746, 2934, 2418, 1983, 1790, 1714, 1929, 2085, 1542, 774,
  774, 1370, 1773, 1628, 1134, 1016, 1483, 1870, 2069, 2316, 2236, 1730, 1182, 688, 430, 591,
  983, 1424, 1704, 1698, 1526, 1322, 1231, 1263, 978, 441, 408, 844, 1000, 876, 967, 1037
};

//global variables for waveform switching needed for task 5
WaveformType current_waveform = WAVE_SINE;
const uint32_t* current_lut = Sin_LUT;
const char* waveform_names[] = {"Sine", "Sawtooth", "Triangle", "Piano", "Guitar", "Drum"};

//global variable for debouncing
static uint32_t last_tick = 0;

// TODO: Equation to calculate TIM2_Ticks
//Task 3
//tim2_ticks= Clock freq/ freq output signal x number of samples in lut: 16000000/1000x128 = 125-1 ticks
// TIM2_Ticks (ARR) = (TIM2CLK / F_sample) - 1
// TIM2_Ticks = (16,000,000 / (1000 * 128)) - 1 = 125 - 1 = 124
uint32_t TIM2_Ticks = (TIM2CLK /(F_SIGNAL*NS)) - 1; // How often to write new LUT value
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

//helps to display LCD
void display_waveform(const char* name);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void display_waveform(const char* name) {
    lcd_command(0xC0); //moves cursor to the second line of the LCD
    //lcd_putstring("Waveform: ");
    lcd_putstring((char*)name);  //prints waveforms name
    //clear the rest of the line if the new name is shorter
    if (strlen(name) < 10) {  //10 character spaces for waveform name
        for(int i = 0; i < (10 - strlen(name)); i++) {
            lcd_putchar(' ');
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //initialise LCD
  init_LCD();
  lcd_command(CLEAR);

  //Task 4
  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1. Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)current_lut, DestAddress, NS);

  // TODO: Write current waveform to LCD(Sine is the first waveform)
  display_waveform(waveform_names[current_waveform]);

  // TODO: Enable DMA (start transfer from LUT to CCR)
  //uses HAL ENABLE DMA to start DMA transfer
  ////enables TIM2_CH1 to trigger DMA request
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  //set to tim2 ticks to control the DMA trigger frequency=124 using calculated TIM2_Ticks value
  htim2.Init.Period = TIM2_Ticks;
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

  //set DMA peripheral address to TIM3->CCR3
  //hdma_tim2_ch1.Init.PeriphAddress = DestAddress;

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
  //this timer value needs to match the 12bit DAC resolution=4095
  htim3.Init.Period = 4095;
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

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void){

	//check if interrupt pin is PA0(Button0_Pin)
	if (__HAL_GPIO_EXTI_GET_IT(Button0_Pin) != RESET){
		// TODO: Debounce using HAL_GetTick()
		uint32_t current_tick = HAL_GetTick();

		// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
		// HINT: Consider using C's "switch" function to handle LUT changes

		if (current_tick - last_tick >= DEBOUNCE_DELAY){
			last_tick = current_tick;
			//goes to next waveform
			current_waveform = (current_waveform + 1) % NUM_WAVEFORMS;

			//select the correct lookup table
		switch(current_waveform){
			case WAVE_SINE:
				current_lut = Sin_LUT;
				break;
			case WAVE_SAW:
				current_lut = Saw_LUT;
				break;
			case WAVE_TRIANGLE:
				current_lut = Triangle_LUT;
				break;
			case WAVE_PIANO:
				current_lut = Piano_LUT;
				break;
			case WAVE_GUITAR:
				current_lut = Guitar_LUT;
				break;
			case WAVE_DRUM:
				current_lut = Drum_LUT;
				break;
			default:
				current_lut = Sin_LUT;
				break;
			}

		//stop DMA transfer before changing source address
		//disable ch1 DMA to allow safe restart
		__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);

		//stop previous DMA transfer
		HAL_DMA_Abort_IT(&hdma_tim2_ch1);

		//start new DMA transfer with new lut
		//since DMA circular have to set up again with new source address
		HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)current_lut, DestAddress, NS);

		//enable DMA request in timer again
		__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

		//update LCD with current waveform name
		display_waveform(waveform_names[current_waveform]);
		}

		//clear interrupt flags
		HAL_GPIO_EXTI_IRQHandler(Button0_Pin);
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

