/*
 * stm32f072xx_gpio_driver.h
 *
 *  Created on: Sep 6, 2023
 *      Author: ITC2KOR
 */

#ifndef INC_STM32F072XX_GPIO_DRIVER_H_
#define INC_STM32F072XX_GPIO_DRIVER_H_


#include "stm32f072xx.h"

typedef struct{
	uint8_t GPIO_PinNumber;				/*!<possibel values from @GPIO_PIN_NUMBERS>*/
	uint8_t GPIO_PinMode;				/*!<possibel values from @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinSpeed;				/*!<possibel values from @GPIO_PIN_SPEEDS>*/
	uint8_t GPIO_PuPdControl;			/*!<possibel values from @GPIO_PIN_PUPD>*/
	uint8_t GPIO_PinOPType;				/*!<possibel values from @GPIO_PIN_OUTPUT_TYPES>*/
	uint8_t GPIO_PinAltFunMode;			/*!<possibel values from @GPIO_PIN_ALT_MODES>*/
}GPIO_PinConfig_t;

/**
 * This is a handle structure for GPIO pin
 **/

typedef struct{

	GPIO_RegDef_t *pGPIOx_addr;			/*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;	/*!< This holds GPIO pin configuration setting >*/
} GPIO_Handle_t;

/**
 * @GPIO_PIN_NUMBERS
 * Possible modes macros for GPIO pin
 **/
#define GPIO_PIN_NO_0				0
#define GPIO_PIN_NO_1				1
#define GPIO_PIN_NO_2				2
#define GPIO_PIN_NO_3				3
#define GPIO_PIN_NO_4				4
#define GPIO_PIN_NO_5				5
#define GPIO_PIN_NO_6				6
#define GPIO_PIN_NO_7				7
#define GPIO_PIN_NO_8				8
#define GPIO_PIN_NO_9				9
#define GPIO_PIN_NO_10				10
#define GPIO_PIN_NO_11				11
#define GPIO_PIN_NO_12				12
#define GPIO_PIN_NO_13				13
#define GPIO_PIN_NO_14				14
#define GPIO_PIN_NO_15				15

/**
 * @GPIO_PIN_MODES
 * Possible modes macros for GPIO pin
 **/
#define GPIO_MODE_IN 				0
#define GPIO_MODE_OUT 				1
#define GPIO_MODE_ALTFN				2
#define GPIO_MODE_ANALOG 			3
#define GPIO_MODE_IT_FALL_EDGE		4
#define GPIO_MODE_IT_RISE_EDGE		5
#define GPIO_MODE_IT_RE_FE_TRIG		6

/**
 * @GPIO_PIN_OUTPUT_TYPES
 * Possible output types macros for GPIO pin
 **/
#define GPIO_OP_TYPE_PP				0
#define GPIO_OP_TYPE_OD				1

/**
 * @GPIO_PIN_SPEEDS
 * Possible output speeds macros for GPIO pin
 **/
#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_FAST				2
#define GPIO_SPEED_HIGH				3

/**
 * @GPIO_PIN_PUPD
 * Possible pull up and pull down macros for GPIO pin
 **/
#define GPIO_NO_PUPD				0
#define GPIO_PIN_PU					1
#define GPIO_PIN_PD					2
/************************************************************************************************
 *                                API Supported by this driver
 *                 For more information about the API, please check the function definition
 ************************************************************************************************/

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnableOrDisable);

/*
 * Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISRhandling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnableOrDisable);
void GPIO_IRQHandling(uint8_t PinNumber);





















#endif /* INC_STM32F072XX_GPIO_DRIVER_H_ */
