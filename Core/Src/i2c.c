/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "stdio.h"
#include "Utils/Commons.h"

/* USER CODE BEGIN 0 */

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

i2cFunctionParam_t i2c_params_data = {
		NULL,
		0,
		NULL,
		0,
		0,
		0,
		ST_I2C_INIT,
		EV_I2C_NONE
};

/* USER CODE BEGIN Private Prototypes */
void MX_I2C1_Init(i2cFunctionParam_t *);
void I2C_DMA_TX(i2cFunctionParam_t *);
void I2C_DMA_RX(i2cFunctionParam_t *);
void I2C_TX(i2cFunctionParam_t *);
void I2C_RX(i2cFunctionParam_t *);
void I2C_TX_RX_Done(i2cFunctionParam_t *);
void I2C_Error(i2cFunctionParam_t * );


void StateMachine_Iteration(i2cFunctionParam_t *);
/* USER CODE END Private Prototypes */

typedef struct {
    const char * name;
    void (* const func)(i2cFunctionParam_t *);
} stateFunctionRow_t;

static stateFunctionRow_t I2C_stateFunction[] = {
        // NAME         // FUNC
	{ "ST_I2C_IDLE",	NULL },
    { "ST_I2C_INIT",	MX_I2C1_Init},
    { "ST_I2C_DMA_RX",	I2C_DMA_RX },
    { "ST_I2C_DMA_TX",	I2C_DMA_TX },
    { "ST_I2C_RX",		I2C_RX },
    { "ST_I2C_TX",		I2C_TX },
    { "ST_I2C_ERROR",	I2C_Error }
};

typedef struct {
	state_i2c_t currState;
    event_i2c_t event;
    state_i2c_t nextState;
} stateTransMatrixRow_t;

static stateTransMatrixRow_t I2C_stateTransMatrix[] = {
    // CURR STATE  v// EVENT           // NEXT STATE
    { ST_I2C_INIT,		EV_I2C_INIT_DONE,			ST_I2C_IDLE  },
    { ST_I2C_DMA_RX,	EV_I2C_DMA_RX_DONE,			ST_I2C_IDLE  },
    { ST_I2C_DMA_TX,	EV_I2C_DMA_TX_DONE,			ST_I2C_IDLE  },
	{ ST_I2C_DMA_TX,	EV_I2C_DMA_TX_RX_TX_DONE,	ST_I2C_DMA_RX  },
    { ST_I2C_RX,     	EV_I2C_RX_DONE,				ST_I2C_IDLE  },
    { ST_I2C_TX,		EV_I2C_TX_DONE,				ST_I2C_IDLE  },
    { ST_I2C_IDLE,		EV_I2C_DMA_RX,				ST_I2C_DMA_RX  },
    { ST_I2C_IDLE,		EV_I2C_DMA_TX,				ST_I2C_DMA_TX  },
    { ST_I2C_IDLE,		EV_I2C_DMA_TX_RX,			ST_I2C_DMA_TX  },
    { ST_I2C_IDLE,		EV_I2C_RX,					ST_I2C_RX  },
    { ST_I2C_IDLE,		EV_I2C_TX,					ST_I2C_TX  },
    { ST_I2C_INIT,		EV_I2C_ERROR,				ST_I2C_ERROR  },
    { ST_I2C_DMA_RX,	EV_I2C_ERROR,				ST_I2C_ERROR  },
    { ST_I2C_DMA_TX,	EV_I2C_ERROR,				ST_I2C_ERROR  },
    { ST_I2C_RX,     	EV_I2C_ERROR,				ST_I2C_ERROR  },
    { ST_I2C_TX,		EV_I2C_ERROR,				ST_I2C_ERROR  },
    { ST_I2C_ERROR,		EV_I2C_NONE,				ST_I2C_IDLE  },
    { ST_I2C_IDLE,		EV_I2C_NONE,				ST_I2C_IDLE  },
	{ ST_I2C_INIT,		EV_I2C_NONE,				ST_I2C_INIT  }
};



/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

/* I2C1 init function */
void MX_I2C1_Init(i2cFunctionParam_t* data)
{

	/* USER CODE BEGIN I2C1_Init 0 */
	if (data->i2cHandle != 0)
	{
		/* USER CODE END I2C1_Init 0 */

		/* USER CODE BEGIN I2C1_Init 1 */

		/* USER CODE END I2C1_Init 1 */
		hi2c1.Instance = I2C1;
		hi2c1.Init.Timing = 0x00802171;
		hi2c1.Init.OwnAddress1 = 0;
		hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		hi2c1.Init.OwnAddress2 = 0;
		hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
		hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		if (HAL_I2C_Init(&hi2c1) != HAL_OK)
		{
			data->event = EV_I2C_ERROR;
			//Error_Handler();
		}
		/** Configure Analogue filter
		 */
		if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
		{
			data->event = EV_I2C_ERROR;
			//Error_Handler();
		}
		/** Configure Digital filter
		 */
		if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
		{
			data->event = EV_I2C_ERROR;
			//Error_Handler();
		}
		/* USER CODE BEGIN I2C1_Init 2 */
		data->currState = ST_I2C_IDLE;
		data->event = EV_I2C_INIT_DONE;
		data->buffer= NULL;
	}
	/* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 DMA Init */
    /* I2C1_RX Init */
    hdma_i2c1_rx.Instance = DMA1_Channel1;
    hdma_i2c1_rx.Init.Request = DMA_REQUEST_I2C1_RX;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c1_rx);

    /* I2C1_TX Init */
    hdma_i2c1_tx.Instance = DMA1_Channel2;
    hdma_i2c1_tx.Init.Request = DMA_REQUEST_I2C1_TX;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c1_tx);

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void I2C_DMA_TX(i2cFunctionParam_t *Params)
{
	if(HAL_I2C_Master_Transmit_DMA(Params->i2cHandle, Params->address << 1, Params->buffer, Params->sizeTx)!= HAL_OK)
	{
		/* Error_Handler() function is called when error occurs. */
		Params->event = EV_I2C_ERROR;
		//Error_Handler();
	}
}

void I2C_DMA_RX(i2cFunctionParam_t *Params)
{
	if(HAL_I2C_Master_Receive_DMA(Params->i2cHandle, Params->address << 1, Params->buffer, Params->sizeRx) != HAL_OK)
	{
		/* Error_Handler() function is called when error occurs. */
		Params->event = EV_I2C_ERROR;
		//Error_Handler();
	}
}

void I2C_TX(i2cFunctionParam_t *Params)
{
	if(HAL_I2C_Master_Transmit(Params->i2cHandle, Params->address << 1, Params->buffer, Params->sizeTx, Params->timeout)!= HAL_OK)
	{
		/* Error_Handler() function is called when error occurs. */
		Params->event = EV_I2C_ERROR;
		//Error_Handler();
	}
}

void I2C_RX(i2cFunctionParam_t *Params)
{
	if(HAL_I2C_Master_Receive(Params->i2cHandle, Params->address << 1, Params->buffer, Params->sizeRx, Params->timeout)!= HAL_OK)
	{
		/* Error_Handler() function is called when error occurs. */
		Params->event = EV_I2C_ERROR;
		//Error_Handler();
	}
}

void Running_StateMachine_Iteration(void)
{
	StateMachine_Iteration(&i2c_params_data);
}

void StateMachine_Iteration(i2cFunctionParam_t *data)
{

    // Iterate through the state transition matrix, checking if there is both a match with the current state
    // and the event
    for(int i = 0; i < sizeof(I2C_stateTransMatrix)/sizeof(I2C_stateTransMatrix[0]); i++) {
        if(I2C_stateTransMatrix[i].currState == data->currState) {
            if(I2C_stateTransMatrix[i].event == data->event) {

                // Transition to the next state
            	data->currState =  I2C_stateTransMatrix[i].nextState;

                // Call the function associated with transition
            	if ( (I2C_stateFunction[data->currState].func) != NULL )
            	{
            		(I2C_stateFunction[data->currState].func)(data);
            	}
            	break;
            }
        }
    }
}

void I2C_RX_TX_DMA_ACK(void)
{
	I2C_TX_RX_Done(&i2c_params_data);
}

void I2C_TX_RX_Done(i2cFunctionParam_t *data)
{

    // Update i2c even status when transmission success
	if (data->currState == ST_I2C_DMA_RX)
	{
		data->event = EV_I2C_DMA_RX_DONE;
	}

	if (data->currState == ST_I2C_DMA_TX)
	{

		if (data->event == EV_I2C_DMA_TX_RX)
		{
			data->event = EV_I2C_DMA_TX_RX_TX_DONE;
		}
		else
		{
			data->event = EV_I2C_DMA_TX_DONE;
		}
	}

	if (data->currState == ST_I2C_TX)
	{
		data->event = EV_I2C_TX_DONE;
	}

	if (data->currState == ST_I2C_RX)
	{
		data->event = EV_I2C_RX_DONE;
	}
}

void I2C_Error(i2cFunctionParam_t *data)
{

    // Print error when transmission fail and update event
	if (data->currState == ST_I2C_INIT)
	{
		printf("i2c initialisation error");
	}

	if (data->currState == ST_I2C_DMA_RX)
	{
		printf("i2c dma tx error");
	}

	if (data->currState == ST_I2C_DMA_TX)
	{
		printf("i2c dma rx error");
	}

	if (data->currState == ST_I2C_TX)
	{
		printf("i2c tx error");
	}

	if (data->currState == ST_I2C_RX)
	{
		printf("i2c rx error");
	}
	//data->event = EV_I2C_NONE;
}

int16_t read_I2C_device_DMA(I2C_HandleTypeDef* i2cHandle, uint16_t addr, uint8_t* buffer, uint16_t size)
{

	if(i2c_params_data.currState != ST_I2C_IDLE )
	{

	}
	i2c_params_data.i2cHandle = i2cHandle;
	i2c_params_data.buffer = buffer;
	i2c_params_data.sizeTx = 1;
	i2c_params_data.sizeRx = size;
	i2c_params_data.address = addr;
	i2c_params_data.event = EV_I2C_DMA_TX_RX;

}

event_i2c_t get_I2C_last_event(I2C_HandleTypeDef* i2cHandle, uint8_t addr)
{
	return i2c_params_data.event;
}

state_i2c_t get_I2C_current_state(I2C_HandleTypeDef* i2cHandle, uint8_t addr)
{
	return i2c_params_data.currState;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
