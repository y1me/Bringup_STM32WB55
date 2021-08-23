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
    ST_I2C_INIT,
    ST_I2C_IDLE,
	ST_I2C_DMA_RX,
	ST_I2C_DMA_TX,
	ST_I2C_RX,
	ST_I2C_TX,
	ST_I2C_ERROR
} state_i2c_t;

typedef enum {
    EV_I2C_INIT_DONE,
	EV_I2C_DMA_RX_DONE,
	EV_I2C_DMA_TX_DONE,
	EV_I2C_RX_DONE,
	EV_I2C_TX_DONE,
	EV_I2C_ERROR,
} event_i2c_t;

typedef struct {
    state_t currState;
    event_t event;
    state_t nextState;
} stateTransMatrixRow_i2c_t;

/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
void I2C_DMA_TX(I2C_HandleTypeDef* , uint16_t, uint8_t*, uint16_t );
void I2C_DMA_RX(I2C_HandleTypeDef* , uint16_t, uint8_t*, uint16_t );
void I2C_TX(I2C_HandleTypeDef* , uint16_t, uint8_t*, uint16_t, uint32_t );
void I2C_RX(I2C_HandleTypeDef* , uint16_t, uint8_t*, uint16_t, uint32_t );

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
