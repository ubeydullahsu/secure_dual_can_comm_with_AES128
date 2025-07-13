/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"

extern SecureData current_data;

// Receiver state variables
extern uint8_t encrypted_buffer[16];
extern uint32_t received_time;
extern uint8_t received;

// Transmitter state variables
extern uint32_t tx_counter;
extern uint32_t last_tx_time;

// Debug buffer
extern char debug_buff[128];

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
   while (1){}
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1){}
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  while (1){}
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  while (1){}
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  while (1){}
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void){}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void){}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void){}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/


/**
  * @brief  Prepare and send secure data
  */
void send_dual_can(CAN_HandleTypeDef *hcan1, CAN_HandleTypeDef *hcan2)
{

	// Prepare secure data
	SecureData data;
	data.counter = tx_counter++;
	data.timestamp = HAL_GetTick();
	data.sensor_val = read_sensor(); // Simulate sensor data
	data.crc = calculate_crc(&data, sizeof(SecureData) - sizeof(uint32_t));

	// Encrypt data
	uint8_t encrypted[16];
	encrypt_data((uint8_t*)&data, encrypted);

	// Split into two CAN datas
	uint8_t part1[8], part2[8];
	memcpy(part1, encrypted, 8);
	memcpy(part2, encrypted + 8, 8);

	// Prepare CAN headers
	CAN_TxHeaderTypeDef tx_header = {
			.StdId = CAN_ID_FRAME1,  // Standard ID
			.ExtId = 0,
			.RTR = CAN_RTR_DATA,     // Data data
			.IDE = CAN_ID_STD,       // Standard identifier
			.DLC = 8,                // Data length: 8 bytes
			.TransmitGlobalTime = DISABLE
	};

	// Send data parts
	uint32_t mailbox;
	HAL_StatusTypeDef status;

	// Send part1 on CAN1
	status = HAL_CAN_AddTxMessage(hcan1, &tx_header, part1, &mailbox);

	if(status != HAL_OK)
	{
		debug_print("CAN [TX] CAN1 error: %d\\r\\n", status);
	}

	// Send part2 on CAN2 with different ID
	tx_header.StdId = CAN_ID_FRAME1;
	status = HAL_CAN_AddTxMessage(hcan2, &tx_header, part2, &mailbox);

	if(status != HAL_OK)
	{
		debug_print("CAN [TX] CAN2 error: %d\\r\\n", status);
	}

	debug_print("CAN [TX] Sent data #%lu\\r\\n", data.counter);
}


/**
  * @brief  CAN reception callback
  * @param  hcan: Pointer to CAN handle
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	// Read received message
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	// Process based on CAN bus and data ID
	if(hcan->Instance == CAN1 && rx_header.StdId == CAN_ID_FRAME1)
	{
		// First data received on CAN1
	    memcpy(encrypted_buffer, rx_data, 8);
	    received_time = HAL_GetTick();
	    received = 1;
	    debug_print("CAN [RX] data1 received\\r\\n");
	}

	else if(hcan->Instance == CAN2 && rx_header.StdId == CAN_ID_FRAME2)
	{
	    // Second data received on CAN2
	    if(received)
	    {
	    	uint32_t time_delta = HAL_GetTick() - received_time;

	    	if(time_delta < FRAME_TIMEOUT_MS)
	    	{
	    		// Copy second part
	    		memcpy(encrypted_buffer + 8, rx_data, 8);

	    		// Decrypt and process
	    		uint8_t decrypted[16];
	    		decrypt_data(encrypted_buffer, decrypted);

	    		// Copy to data structure
	    		memcpy(&current_data, decrypted, sizeof(SecureData));
	    		process_secure_data(&current_data);
	    	}

	    	else
	    	{
	    		debug_print("CAN [RX] ERROR: data2 timeout! Delta: %lums\\r\\n", time_delta);
	    	}

	    	// Reset state
	    	received = 0;
	    }
	    else
	    {
	      debug_print("CAN [RX] ERROR: data2 without data1!\\r\\n");
	    }

	}

	else
	{
		debug_print("CAN [RX] ERROR!\\r\\n");
	}

}
