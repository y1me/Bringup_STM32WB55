/*
 * test.h
 *
 *  Created on: Mar 10, 2021
 *      Author: blobby
 */

#ifndef BSP_USERDEFINEHW_YAME_NUCLEOWB55_H_
#define BSP_USERDEFINEHW_YAME_NUCLEOWB55_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"

/** @defgroup STM32WBXX_NUCLEO_Exported_Types Exported Types
  * @{
  */
typedef enum
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  /* Color led aliases */
  LED_BLUE   = LED1,
  LED_GREEN  = LED2,
  LED_RED    = LED3
}Led_TypeDef;

typedef enum
{
  TEST_PIN1 = 0,
  TEST_PIN2 = 1,
}TestPin_TypeDef;

typedef enum
{
  BUTTON_SW1 = 0,
  BUTTON_SW2 = 1,
  BUTTON_SW3 = 2,
}Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;

/** @defgroup TEST pin Constants
  * @{
  */
#define TESTn                                   2

#define TEST1_PIN                                GPIO_PIN_5
#define TEST1_GPIO_PORT                          GPIOA
#define TEST1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOA_CLK_ENABLE()
#define TEST1_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOA_CLK_DISABLE()

#define TEST2_PIN                                GPIO_PIN_6
#define TEST2_GPIO_PORT                          GPIOA
#define TEST2_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOA_CLK_ENABLE()
#define TEST2_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOA_CLK_DISABLE()

#define TESTx_GPIO_CLK_ENABLE(__INDEX__)         __HAL_RCC_GPIOA_CLK_ENABLE() /* All Led on same port */
#define TESTx_GPIO_CLK_DISABLE(__INDEX__)        __HAL_RCC_GPIOA_CLK_DISABLE() /* All Led on same port */

/** @defgroup STM32WBXX_NUCLEO_LED LED Constants
  * @{
  */
#define LEDn                                    3

#define LED1_PIN                                GPIO_PIN_5
#define LED1_GPIO_PORT                          GPIOB
#define LED1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LED2_PIN                                GPIO_PIN_0
#define LED2_GPIO_PORT                          GPIOB
#define LED2_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LED3_PIN                                GPIO_PIN_1
#define LED3_GPIO_PORT                          GPIOB
#define LED3_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)         __HAL_RCC_GPIOB_CLK_ENABLE() /* All Led on same port */
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)        __HAL_RCC_GPIOB_CLK_DISABLE() /* All Led on same port */
/**
  * @}
  */

/** @defgroup STM32WBXX_NUCLEO_BUTTON BUTTON Constants
  * @{
  */
#define BUTTONn                                 3

/**
 * @brief Key push-button
 */
#define BUTTON_SW1_PIN                          GPIO_PIN_4
#define BUTTON_SW1_GPIO_PORT                    GPIOC
#define BUTTON_SW1_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define BUTTON_SW1_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
#define BUTTON_SW1_EXTI_LINE                    GPIO_PIN_4
#ifdef CORE_CM0PLUS
#define BUTTON_SW1_EXTI_IRQn                    EXTI15_4_IRQn
#else
#define BUTTON_SW1_EXTI_IRQn                    EXTI4_IRQn
#endif

#define BUTTON_SW2_PIN                          GPIO_PIN_0
#define BUTTON_SW2_GPIO_PORT                    GPIOD
#define BUTTON_SW2_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOD_CLK_ENABLE()
#define BUTTON_SW2_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOD_CLK_DISABLE()
#define BUTTON_SW2_EXTI_LINE                    GPIO_PIN_0
#ifdef CORE_CM0PLUS
#define BUTTON_SW2_EXTI_IRQn                    EXTI1_0_IRQn
#else
#define BUTTON_SW2_EXTI_IRQn                    EXTI0_IRQn
#endif

#define BUTTON_SW3_PIN                          GPIO_PIN_1
#define BUTTON_SW3_GPIO_PORT                    GPIOD
#define BUTTON_SW3_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOD_CLK_ENABLE()
#define BUTTON_SW3_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOD_CLK_DISABLE()
#define BUTTON_SW3_EXTI_LINE                    GPIO_PIN_1
#ifdef CORE_CM0PLUS
#define BUTTON_SW3_EXTI_IRQn                    EXTI1_0_IRQn
#else
#define BUTTON_SW3_EXTI_IRQn                    EXTI1_IRQn
#endif

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do { if ((__INDEX__) == BUTTON_SW1) BUTTON_SW1_GPIO_CLK_ENABLE(); else \
                                              if ((__INDEX__) == BUTTON_SW2) BUTTON_SW2_GPIO_CLK_ENABLE(); else \
                                              if ((__INDEX__) == BUTTON_SW3) BUTTON_SW3_GPIO_CLK_ENABLE();} while(0)

#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)    do { if ((__INDEX__) == BUTTON_SW1) BUTTON_SW1_GPIO_CLK_DISABLE(); else \
                                              if ((__INDEX__) == BUTTON_SW2) BUTTON_SW2_GPIO_CLK_DISABLE(); else \
                                              if ((__INDEX__) == BUTTON_SW3) BUTTON_SW3_GPIO_CLK_DISABLE();} while(0)

/**
  * @}
  */


/** @addtogroup STM32WBXX_NUCLEO_Exported_Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);

/** @addtogroup STM32WBXX_NUCLEO_LED_Functions
  * @{
  */
void             BSP_LED_Init(Led_TypeDef Led);
void             BSP_LED_DeInit(Led_TypeDef Led);
void             BSP_LED_On(Led_TypeDef Led);
void             BSP_LED_Off(Led_TypeDef Led);
void             BSP_LED_Toggle(Led_TypeDef Led);
/**
  * @}
  */

/** @addtogroup TEST_PIN_Functions
  * @{
  */
void             BSP_TEST_Init(TestPin_TypeDef Pin);
void             BSP_TEST_DeInit(TestPin_TypeDef Pin);
void             BSP_TEST_Toggle(TestPin_TypeDef Pin);
/**
  * @}
  */

/** @addtogroup STM32WBXX_NUCLEO_BUTTON_Functions
  * @{
  */
void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void             BSP_PB_DeInit(Button_TypeDef Button);
uint32_t         BSP_PB_GetState(Button_TypeDef Button);

#endif /* BSP_USERDEFINEHW_YAME_NUCLEOWB55_H_ */
