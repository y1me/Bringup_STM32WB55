/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */


typedef enum {
    ST_I2C_IDLE,
    ST_I2C_INIT,
	ST_I2C_DMA_RX,
	ST_I2C_DMA_TX,
	ST_I2C_RX,
	ST_I2C_TX,
	ST_I2C_ERROR
} state_i2c_t;

typedef struct {
	state_i2c_t currState;
} i2c_stateMachine_t;

typedef enum {
    EV_I2C_INIT_DONE,
	EV_I2C_DMA_RX_DONE,
	EV_I2C_DMA_TX_DONE,
	EV_I2C_DMA_TX_RX_TX_DONE,
	EV_I2C_RX_DONE,
	EV_I2C_TX_DONE,
	EV_I2C_DMA_RX,
	EV_I2C_DMA_TX,
	EV_I2C_DMA_TX_RX,
	EV_I2C_RX,
	EV_I2C_TX,
	EV_I2C_ERROR,
	EV_I2C_NONE
} event_i2c_t;

typedef struct {
	I2C_HandleTypeDef* i2cHandle;
	uint16_t address;
	uint8_t* buffer;
	//uint8_t* bufferRx;
	uint16_t sizeTx;
	uint16_t sizeRx;
	uint32_t timeout;
	state_i2c_t currState;
	event_i2c_t event;
} i2cFunctionParam_t;


/* USER CODE END Private defines */

//void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */

void Running_StateMachine_Iteration(void);
void I2C_RX_TX_DMA_ACK(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
