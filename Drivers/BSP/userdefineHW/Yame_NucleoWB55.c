/*
 * test.c
 *
 *  Created on: Mar 10, 2021
 *      Author: blobby
 */

/* Includes ------------------------------------------------------------------*/
#include "Yame_NucleoWB55.h"

/**
  * @brief STM32WBxx NUCLEO BSP Driver
  */
#define __STM32WBxx_NUCLEO_BSP_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __STM32WBxx_NUCLEO_BSP_VERSION_SUB1   (0x00U) /*!< [23:16] sub1 version */
#define __STM32WBxx_NUCLEO_BSP_VERSION_SUB2   (0x02U) /*!< [15:8]  sub2 version */
#define __STM32WBxx_NUCLEO_BSP_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define __STM32WBxx_NUCLEO_BSP_VERSION        ((__STM32WBxx_NUCLEO_BSP_VERSION_MAIN << 24)\
                                             |(__STM32WBxx_NUCLEO_BSP_VERSION_SUB1 << 16)\
                                             |(__STM32WBxx_NUCLEO_BSP_VERSION_SUB2 << 8 )\
                                             |(__STM32WBxx_NUCLEO_BSP_VERSION_RC))

  /** @defgroup STM32WBXX_NUCLEO_LOW_LEVEL_Private_Variables Private Variables
    * @{
    */
  GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT, LED3_GPIO_PORT};
  const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN};

  GPIO_TypeDef* GPIOTEST_PORT[TESTn] = {TEST1_GPIO_PORT, TEST2_GPIO_PORT};
  const uint16_t GPIOTEST_PIN[TESTn] = {TEST1_PIN, TEST2_PIN};

  GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {BUTTON_SW1_GPIO_PORT, BUTTON_SW2_GPIO_PORT, BUTTON_SW3_GPIO_PORT};
  const uint16_t BUTTON_PIN[BUTTONn] = {BUTTON_SW1_PIN, BUTTON_SW2_PIN, BUTTON_SW3_PIN};
  const uint8_t BUTTON_IRQn[BUTTONn] = {BUTTON_SW1_EXTI_IRQn, BUTTON_SW2_EXTI_IRQn, BUTTON_SW3_EXTI_IRQn};

  uint32_t BSP_GetVersion(void)
  {
    return __STM32WBxx_NUCLEO_BSP_VERSION;
  }

  /** @defgroup STM32WBXX_NUCLEO_LED_Functions LED Functions
    * @{
    */

  /**
    * @brief  Configures LED GPIO.
    * @param  Led: LED to be configured.
    *          This parameter can be one of the following values:
    *            @arg LED1
    *            @arg LED2
    *            @arg LED3
    * @retval None
    */
  void BSP_LED_Init(Led_TypeDef Led)
  {
    GPIO_InitTypeDef  gpioinitstruct = {0};

    /* Enable the GPIO_LED Clock */
    LEDx_GPIO_CLK_ENABLE(Led);

    /* Configure the GPIO_LED pin */
    gpioinitstruct.Pin = GPIO_PIN[Led];
    gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
    gpioinitstruct.Pull = GPIO_NOPULL;
    gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIO_PORT[Led], &gpioinitstruct);

    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  }

  /**
    * @brief  DeInit LEDs.
    * @param  Led: LED to be de-init.
    *   This parameter can be one of the following values:
    *     @arg  LED1
    *     @arg  LED2
    *     @arg  LED3
    * @note Led DeInit does not disable the GPIO clock nor disable the Mfx
    * @retval None
    */
  void BSP_LED_DeInit(Led_TypeDef Led)
  {
    GPIO_InitTypeDef  gpio_init_structure;

    /* Turn off LED */
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
    /* DeInit the GPIO_LED pin */
    gpio_init_structure.Pin = GPIO_PIN[Led];
    HAL_GPIO_DeInit(GPIO_PORT[Led], gpio_init_structure.Pin);
  }

  void BSP_TEST_Init(TestPin_TypeDef Pin)
    {
      GPIO_InitTypeDef  gpioinitstruct = {0};

      /* Enable the GPIO_LED Clock */
      TESTx_GPIO_CLK_ENABLE(Pin);

      /* Configure the GPIO_LED pin */
      gpioinitstruct.Pin = GPIOTEST_PIN[Pin];
      gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
      gpioinitstruct.Pull = GPIO_NOPULL;
      gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;

      HAL_GPIO_Init(GPIOTEST_PORT[Pin], &gpioinitstruct);

      HAL_GPIO_WritePin(GPIOTEST_PORT[Pin], GPIOTEST_PIN[Pin], GPIO_PIN_SET);
    }

    /**
      * @brief  DeInit LEDs.
      * @param  Led: LED to be de-init.
      *   This parameter can be one of the following values:
      *     @arg  LED1
      *     @arg  LED2
      *     @arg  LED3
      * @note Led DeInit does not disable the GPIO clock nor disable the Mfx
      * @retval None
      */
    void BSP_TEST_DeInit(TestPin_TypeDef Pin)
    {
      GPIO_InitTypeDef  gpio_init_structure;

      /* Turn off LED */
      HAL_GPIO_WritePin(GPIOTEST_PORT[Pin], GPIOTEST_PIN[Pin], GPIO_PIN_RESET);
      /* DeInit the GPIO_LED pin */
      gpio_init_structure.Pin = GPIOTEST_PIN[Pin];
      HAL_GPIO_DeInit(GPIOTEST_PORT[Pin], gpio_init_structure.Pin);
    }

    void BSP_TEST_Toggle(TestPin_TypeDef Pin)
     {
       HAL_GPIO_TogglePin(GPIOTEST_PORT[Pin], GPIOTEST_PIN[Pin]);
     }
  /**
    * @brief  Turns selected LED On.
    * @param  Led: Specifies the Led to be set on.
    *   This parameter can be one of following parameters:
    *     @arg LED1
    *     @arg LED2
    *     @arg LED3
    * @retval None
    */
  void BSP_LED_On(Led_TypeDef Led)
  {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
  }

  /**
    * @brief  Turns selected LED Off.
    * @param  Led: Specifies the Led to be set off.
    *   This parameter can be one of following parameters:
    *     @arg LED1
    *     @arg LED2
    *     @arg LED3
    * @retval None
    */
  void BSP_LED_Off(Led_TypeDef Led)
  {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  }

  /**
    * @brief  Toggles the selected LED.
    * @param  Led: Specifies the Led to be toggled.
    *   This parameter can be one of following parameters:
    *     @arg LED1
    *     @arg LED2
    *     @arg LED3
    * @retval None
    */
  void BSP_LED_Toggle(Led_TypeDef Led)
  {
    HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
  }

  /**
    * @}
    */

  /** @defgroup STM32WBXX_NUCLEO_BUTTON_Functions BUTTON Functions
    * @{
    */

  /**
    * @brief  Configures Button GPIO and EXTI Line.
    * @param  Button: Specifies the Button to be configured.
    *   This parameter can be one of following parameters:
    *     @arg BUTTON_SW1
    *     @arg BUTTON_SW2
    *     @arg BUTTON_SW3
    * @param  ButtonMode: Specifies Button mode.
    *   This parameter can be one of following parameters:
    *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
    *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
    *                            generation capability
    * @retval None
    */
  void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
  {
    GPIO_InitTypeDef gpioinitstruct = {0};

    /* Enable the BUTTON Clock */
    BUTTONx_GPIO_CLK_ENABLE(Button);

    if(ButtonMode == BUTTON_MODE_GPIO)
    {
      /* Configure Button pin as input */
      gpioinitstruct.Pin = BUTTON_PIN[Button];
      gpioinitstruct.Mode = GPIO_MODE_INPUT;
      gpioinitstruct.Pull = GPIO_PULLUP;
      gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);

      /* Wait Button pin startup stability */
      HAL_Delay(1);
    }

    if(ButtonMode == BUTTON_MODE_EXTI)
    {
      /* Configure Button pin as input with External interrupt */
      gpioinitstruct.Pin = BUTTON_PIN[Button];
      gpioinitstruct.Pull = GPIO_PULLUP;
      gpioinitstruct.Mode = GPIO_MODE_IT_FALLING;
      HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);

      /* Enable and set Button EXTI Interrupt to the lowest priority */
      HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0x00);
      HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
    }
  }

  /**
    * @brief  Push Button DeInit.
    * @param  Button: Button to be configured
    *   This parameter can be one of following parameters:
    *     @arg BUTTON_SW1
    *     @arg BUTTON_SW2
    *     @arg BUTTON_SW3
    * @note PB DeInit does not disable the GPIO clock
    * @retval None
    */
  void BSP_PB_DeInit(Button_TypeDef Button)
  {
      GPIO_InitTypeDef gpio_init_structure;

      gpio_init_structure.Pin = BUTTON_PIN[Button];
      HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
      HAL_GPIO_DeInit(BUTTON_PORT[Button], gpio_init_structure.Pin);
  }

  /**
    * @brief  Returns the selected Button state.
    * @param  Button: Specifies the Button to be checked.
    *   This parameter can be one of following parameters:
    *     @arg BUTTON_SW1
    *     @arg BUTTON_SW2
    *     @arg BUTTON_SW3
    * @retval The Button GPIO pin value.
    */
  uint32_t BSP_PB_GetState(Button_TypeDef Button)
  {
    return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
  }
