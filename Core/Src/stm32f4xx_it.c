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

void send_dual_can(CAN_HandleTypeDef *hcan1, CAN_HandleTypeDef *hcan2)
{

  // Encrypt data
  uint8_t encrypted[16];
  encrypt_data((uint8_t*)&tx_data, encrypted);

  // Split the data into two parts
  uint8_t part1[8], part2[8];
  memcpy(part1, encrypted, 8);     // First 8-bytes
  memcpy(part2, encrypted+8, 8);   // Last 8-bytes

  // Send to CAN1 and CAN2 simultaneously
  CAN_TxHeaderTypeDef tx_header = {
    .StdId = 0x123,     // Standart ID
    .RTR = CAN_RTR_DATA,
    .IDE = CAN_ID_STD,
    .DLC = 8,           // 8 byte veri
    .TransmitGlobalTime = DISABLE
  };

  uint32_t mailbox;
  HAL_CAN_AddTxMessage(hcan1, &tx_header, part1, &mailbox);
  HAL_CAN_AddTxMessage(hcan2, &tx_header, part2, &mailbox);
}

uint8_t rx_buffer[16];
uint32_t last_rx_time = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  // Determine which CAN port the message came from
  if(hcan == &hcan1) {
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
    memcpy(rx_buffer, rx_data, 8);  // first partt
    last_rx_time = HAL_GetTick();
  }
  else if(hcan == &hcan2) {
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);
    memcpy(rx_buffer+8, rx_data, 8); // second part

    // Time control (if arrived within 10ms)
    if((HAL_GetTick() - last_rx_time) < 10) {
      // Decrypt data
      uint8_t decrypted[16];
      decrypt_data(rx_buffer, decrypted);
      memcpy(&rx_data, decrypted, sizeof(SecureData));

      // 6. CRC check
      if(rx_data.crc == calculate_crc(&rx_data, sizeof(SecureData)-4)) {
        // secure data processing
        process_secure_data(&rx_data);

      }
    }
  }
}
