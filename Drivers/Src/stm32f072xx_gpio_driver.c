/*
 * stm32f072xx_gpio_driver.c
 *
 *  Created on: Sep 6, 2023
 *      Author: ITC2KOR
 */


#include "stm32f072xx_gpio_driver.h"

/*
 * Peripheral clock setup
 */

/*******************************************************************************
 *  @fn					- GPIO_Init
 *
 *  @brief				- This API controls the peripheral clock
 *
 *  @param[in]			- Base address of the GPIO Peripheral
 *  @param[in]			- ENABLE or DISABLE macros
 *
 *  @return				- None
 *
 *  @Note				- None
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnableOrDisable){
	if(EnableOrDisable == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
	}
	else if(EnableOrDisable == DISABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
	}
}

/*
 * Init and DeInit
 */

/*******************************************************************************
 *  @fn					- GPIO_Init
 *
 *  @brief				- This API controls the peripheral clock
 *
 *  @param[in]			- Base address of the GPIO Peripheral
 *  @param[in]			- ENABLE or DISABLE macros
 *
 *  @return				- None
 *
 *  @Note				- None
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	// Enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx_addr,ENABLE);
	uint32_t temp = 0;
	//1. Configure the mode of the GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		pGPIOHandle->pGPIOx_addr->MODER &= ~((0x3) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // reset the pins
		pGPIOHandle->pGPIOx_addr->MODER |= temp; // setting the pins
	}
	else{
		// this part will initialize interrupt mode later on
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FALL_EDGE){
			//1. configure the falling edge trigger selection register (FTSR)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// RTSR must be reset before setting the FTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RISE_EDGE){
			//1. configure the rising edge trigger selection register (RTSR)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// FTSR must be reset before setting the RTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RE_FE_TRIG){
			//1. configure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. configure the GPIO port selection in SYSCFG_EXTICR

		uint8_t temp1, temp2;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4);
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx_addr);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portCode << (temp2*4);

		//3. enable the EXTI interrupt delivery using IMR (Interrupt Mask Register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	//2. Configure the speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx_addr->OSPEEDR &= ~((0x3) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  //reset
	pGPIOHandle->pGPIOx_addr->OSPEEDR |= temp;

	//3. Configure the pull up or pull down settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PuPdControl << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx_addr->PUPDR &= ~((0x3) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  //reset
	pGPIOHandle->pGPIOx_addr->PUPDR |= temp;

	//4. Configure the output type
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx_addr->OTYPER &= ~((0x3) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  //reset
	pGPIOHandle->pGPIOx_addr->OTYPER |= temp;

	//5. Configure the alternate functionality
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		// will do later
		uint8_t new_pin = 0;
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber > 8){
			new_pin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8;
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*(new_pin)));
			pGPIOHandle->pGPIOx_addr->AFRH &= ~((0xF) << (new_pin));
			pGPIOHandle->pGPIOx_addr->AFRH |= temp;
		}
		else{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
			pGPIOHandle->pGPIOx_addr->AFRH &= ~((0xF) << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx_addr->AFRL |= temp;
		}
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
}

/*
 * data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t bit_value;
	bit_value = (((pGPIOx->IDR) >> PinNumber) & 0x00000001);
	return bit_value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){
	if(value == GPIO_PIN_SET){
		// write 1 in the pin number
		pGPIOx->ODR |= value << PinNumber;
	}
	else if(value == GPIO_PIN_RESET){
		pGPIOx->ODR &= ~(value << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint16_t num;
	num = ((pGPIOx->ODR >> PinNumber) & (uint16_t)(0x1));
	if(num == 0){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else if(num == 1){
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
//	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Priority Configuration
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. find the IPR register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;

	uint8_t shift_amount = ((8 * iprx_section) + (8 - NO_OF_BITS_INCLUIDED));
	*(NVIC_PR_BASE_ADDR + (iprx)) = (IRQPriority << (shift_amount));
}
/*
 * IRQ Configuration and ISRhandling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnableOrDisable){
	if(EnableOrDisable == ENABLE){
		if(IRQNumber <= 31){
			// we have 31 IRQ list to handle
			*NVIC_ISER |= (1 << IRQNumber);
		}
	}
	else{
		if(IRQNumber <= 31){
			*NVIC_ICER |= (1 << IRQNumber);
		}
	}
}
void GPIO_IRQHandling(uint8_t PinNumber){
	// work is to clear the EXTI_PR register to the pin number
	if((EXTI->PR) & (1 << PinNumber)){
		// clear the pending register, if the PinNumber by user
		// is matching with the PR pin of the EXTI
		EXTI->PR |= (1 << PinNumber);
	}
}
