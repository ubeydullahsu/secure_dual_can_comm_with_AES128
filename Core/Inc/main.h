/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>

#define SENSOR_CONST	5u

#define CAN_ID_FRAME1      0x111
#define CAN_ID_FRAME2      0x112

#define TX_INTERVAL_MS     1000    // 1 second transmission interval
#define FRAME_TIMEOUT_MS   10      // Max time between frames
#define STALE_DATA_MS      500     // Data older than 500ms is stale

// Uncomment based on node role
//#define NODE_ROLE MODE_TRANSMITTER
#define NODE_ROLE MODE_RECEIVER
/* USER CODE END PD */

typedef struct {
	uint32_t counter;
	uint32_t timestamp;
	uint16_t sensor_val;
	uint32_t crc;
} SecureData;

typedef enum {
	MODE_RECEIVER,
	MODE_TRANSMITTER
} OperationMode;

/* Functions prototypes ---------------------------------------------*/
void Error_Handler(void);

void encrypt_data(uint8_t* input, uint8_t* output);
void decrypt_data(uint8_t* input, uint8_t* output);
uint32_t calculate_crc(void *data, size_t len);
uint16_t read_sensor(void);
void process_secure_data(SecureData *data);
void debug_print(const char *format, ...);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
