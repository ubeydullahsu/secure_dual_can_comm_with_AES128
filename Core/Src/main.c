/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ISO/SAE 21434 Compliant Dual-CAN Security Gateway
  * @author			: ubeydullahsu
  * https://github.com/ubeydullahsu
  ******************************************************************************
  * Dual-CAN implementation with AES-128 encryption for legacy automotive systems
  * Supports both transmitter and receiver modes
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "aes.h" // Tiny-AES library



/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart2;

uint8_t aes_key[16] = "MirilZeynaYumak"; // AES encryption key (16-bytes)

SecureData current_data;

// Receiver state variables
uint8_t encrypted_buffer[16];
uint32_t received_time = 0;
uint8_t received = 0;

// Transmitter state variables
uint32_t tx_counter = 0;
uint32_t last_tx_time = 0;

// Debug buffer
char debug_buff[128];


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_UART_Init(void);
void init_can(void);
void init_crypto(void);


/**
  * @brief  AES encryption function
  * @param  input: Pointer to data
  * @param  output: Length of data in bytes
  */
void encrypt_data(uint8_t* input, uint8_t* output)
{

  struct AES_ctx ctx;
  AES_init_ctx(&ctx, aes_key);
  AES_ECB_encrypt(&ctx, (uint8_t*)input);
  memcpy(output, input, 16);

}

/**
  * @brief  AES decryption function
  * @param  input: Pointer to data
  * @param  output: Length of data in bytes
  */
void decrypt_data(uint8_t* input, uint8_t* output)
{

  struct AES_ctx ctx;
  AES_init_ctx(&ctx, aes_key);
  AES_ECB_decrypt(&ctx, (uint8_t*)input);
  memcpy(output, input, 16);

}

/**
  * @brief  Hardware-accelerated CRC calculation
  * @param  data: Pointer to data
  * @param  len: Length of data in bytes
  * @retval CRC-32 value
  */
uint32_t calculate_crc(void *data, size_t len)
{

  __HAL_RCC_CRC_CLK_ENABLE(); // Enable CRC clock
  CRC->CR = CRC_CR_RESET;     // Reset CRC calculator

  uint32_t *ptr = (uint32_t*)data;
  size_t word_count = len / 4;

  // Process 32-bit words
  for(size_t i = 0; i < word_count; i++)
  {
    CRC->DR = __builtin_bswap32(*ptr++);
  }

  // Process remaining bytes
  if(len % 4)
  {
    uint32_t temp = 0;
    memcpy(&temp, ptr, len % 4);
    CRC->DR = __builtin_bswap32(temp);
  }

  return CRC->DR;

}

/**
  * @brief  Process received secure data
  * @param  data: Pointer to decrypted data
  */
void process_secure_data(SecureData *data)
{
	// Check timestamp freshness
	uint32_t current_time = HAL_GetTick();
	uint32_t time_delta = current_time - data->timestamp;

	if(time_delta > STALE_DATA_MS)
	{
		debug_print("[SEC] WARNING: Stale data! Delta: %lums\\r\\n", time_delta);
		return;
	}

	// Verify CRC
	uint32_t calculated_crc = calculate_crc(data, sizeof(SecureData) - sizeof(uint32_t));

	if(data->crc != calculated_crc)
	{
		debug_print("[SEC] ERROR: CRC mismatch! Exp:0x%lX, Act:0x%lX\\r\\n",
                calculated_crc, data->crc);
		return;
	}

	// Process valid data
	debug_print("[SEC] Valid data: #%lu, Val:%.2f, Age:%lums\\r\\n",
              data->counter, data->sensor_val, time_delta);

	// Visual indicator
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_6); // Toggle LED
}


uint16_t read_sensor(void)
{
	return (uint16_t)(rand() * SENSOR_CONST);
}

/**
  * @brief  Print debug messages over USART2
  * @param  format: printf-style format string
  */

void debug_print(const char *format, ...)
{

  va_list args;
  va_start(args, format);
  vsnprintf(debug_buff, sizeof(debug_buff), format, args);
  va_end(args);
  HAL_UART_Transmit(&huart2, (uint8_t*)debug_buff, strlen(debug_buff), 100);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();

  // Initialize cryptographic hardware
  init_crypto();

  // Initialize CAN peripherals
  init_can();

  // Print system information
  debug_print("\\r\\n\\r\\n=== Dual-CAN Security Gateway ===\\r\\n");
  debug_print("System Clock: %lu Hz\\r\\n", HAL_RCC_GetHCLKFreq());
  debug_print("ISO/SAE 21434 Compliant Prototype\\r\\n");

  // Initial LED state
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);


  srand((unsigned int)time(NULL));

  /* Infinite loop */
  while (1)
  {

	  // Receiver mode - nothing to do in main loop
	  if(NODE_ROLE == MODE_RECEIVER)
	  {
		  HAL_Delay(100);

	  }

	  // Transmitter mode - send data periodically
	  else if(NODE_ROLE == MODE_TRANSMITTER)
	  {

		  uint32_t current_time = HAL_GetTick();

	      // Send data at fixed interval
	      if((current_time - last_tx_time) >= TX_INTERVAL_MS)
	      {
	    	  send_dual_can(&hcan1, &hcan2);
	    	  last_tx_time = current_time;

	    	  // Toggle LED on successful transmission
	    	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
	      }

	      HAL_Delay(10);

	  }

  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief  Initialize CAN peripherals
  */
void init_can(void)
{

	// Configure CAN filters
	CAN_FilterTypeDef filter_config = {
			.FilterIdHigh = 0x0000,
			.FilterIdLow = 0x0000,
			.FilterMaskIdHigh = 0x0000,
			.FilterMaskIdLow = 0x0000,
			.FilterFIFOAssignment = CAN_RX_FIFO0,
			.FilterBank = 0,
			.FilterMode = CAN_FILTERMODE_IDMASK,
			.FilterScale = CAN_FILTERSCALE_32BIT,
			.FilterActivation = ENABLE,
			.SlaveStartFilterBank = 14
	};

	// Apply filters to both CAN controllers
	HAL_CAN_ConfigFilter(&hcan1, &filter_config);
	HAL_CAN_ConfigFilter(&hcan2, &filter_config);

	// Start CAN controllers
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);

	// Activate RX notifications
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

	// Configure interrupts
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
	HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);

	// For TX interrupt (optional)
	//HAL_NVIC_SetPriority(CAN1_TX_IRQn, 3, 0);
	//HAL_NVIC_SetPriority(CAN2_TX_IRQn, 3, 0);
	//HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
	//HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);

	debug_print("[CAN] Initialized with %s mode\\r\\n",
              (NODE_ROLE == MODE_RECEIVER) ? "RECEIVER" : "TRANSMITTER");

}

/**
  * @brief  Initialize cryptographic peripherals
  */
void init_crypto(void)
{

  // Enable CRC clock
  __HAL_RCC_CRC_CLK_ENABLE();
  debug_print("[CRYPTO] Hardware CRC enabled\\r\\n");

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {}
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
