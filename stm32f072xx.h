/*
 * stm32f0xx.h
 *
 *  Created on: Sep 6, 2023
 *      Author: ITC2KOR
 */

#ifndef INC_STM32F072XX_H_
#define INC_STM32F072XX_H_

#include <stdint.h>
#define __vo volatile
/*
 * base addresses of FLASH and SRAM memories
 */

#define FLASH_BASE_ADDR 								(uint32_t)0x08000000
#define SRAM_BASE_ADDR								 	(uint32_t)0x20000000
#define ROM_BASE_ADDR								 	(uint32_t)0x1FFFEC00

/**
 * AHBx and APBx Bus peripheral base address
 **/

#define PERIPHERAL_BASE_ADDR							(uint32_t)0x40000000
#define APB1PERIPH_BASE_ADDR							PERIPHERAL_BASE_ADDR
#define APB2PERIPH_BASE_ADDR							(uint32_t)0x40010000
#define AHB1PERIPH_BASE_ADDR							(uint32_t)0x40020000
#define AHB2PERIPH_BASE_ADDR							(uint32_t)0x48000000


/**
 * Base address of the peripherals hanging
 * on APB1 bus
 **/
#define SPI2_BASE_ADDR									(uint32_t)0x40003800
#define USART2_BASE_ADDR								(uint32_t)0x40004400
#define USART3_BASE_ADDR								(uint32_t)0x40004800
#define USART4_BASE_ADDR								(uint32_t)0x40004C00
#define USART5_BASE_ADDR								(uint32_t)0x40005000
#define I2C1_BASE_ADDR									(uint32_t)0x40005400
#define I2C2_BASE_ADDR									(uint32_t)0x40005800

/**
 * Base address of the peripherals hanging
 * on APB2 bus
 **/
#define SYSCFGCOMP_BASE_ADDR							(uint32_t)0x40010000
#define EXTI_BASE_ADDR									(uint32_t)0x40010400
#define USART6_BASE_ADDR								(uint32_t)0x40011400
#define USART7_BASE_ADDR								(uint32_t)0x40011800
#define USART8_BASE_ADDR								(uint32_t)0x40011C00
#define SPI1_BASE_ADDR									(uint32_t)0x40013000
#define USART1_BASE_ADDR								(uint32_t)0x40013800

/**
 * Base address of the peripherals hanging
 * on AHB2 bus
 **/
#define GPIOA_BASE_ADDR									(uint32_t)(0x48000000)
#define GPIOB_BASE_ADDR									(uint32_t)(0x48000400)
#define GPIOC_BASE_ADDR									(uint32_t)(0x48000800)
#define GPIOD_BASE_ADDR									(uint32_t)(0x48000C00)
#define GPIOE_BASE_ADDR									(uint32_t)(0x48001000)
#define GPIOF_BASE_ADDR									(uint32_t)(0x48001400)

/**
 * Base address of the peripherals hanging
 * on AHB2 bus
 **/
#define RCC_BASE_ADDR									(uint32_t)(0x40021000)

/******************Peripheral register definition structures***********************/
/**
 * Note: Registers of peripherals are specific to MCU
 * e.g: The number of registers present in SPI for L4 series
 * may be different from the registers present in SPI for F0 board
 * Please check the device Reference Manual before proceeding
 **/
typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;
	__vo uint32_t BRR;
} GPIO_RegDef_t;


typedef struct{
	__vo uint32_t RCC_CR;
	__vo uint32_t RCC_CFGR;
	__vo uint32_t RCC_CIR;
	__vo uint32_t RCC_APB2RSTR;
	__vo uint32_t RCC_APB1RSTR;
	__vo uint32_t RCC_AHBENR;
	__vo uint32_t RCC_APB2ENR;
	__vo uint32_t RCC_APB1ENR;
	__vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;
	__vo uint32_t RCC_AHBRSTR;
	__vo uint32_t RCC_CFGR2;
	__vo uint32_t RCC_CFGR3;
	__vo uint32_t RCC_CR2;
} RCC_RegDef_t;
/**
 *  peripheral definition ( peripheral base addresses type-casted to xxx_RegDef_t )
 **/
#define GPIOA											((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB											((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC											((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD											((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE											((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF											((GPIO_RegDef_t*)GPIOF_BASE_ADDR)

#define RCC												((RCC_RegDef_t*)(RCC_BASE_ADDR))

/**
 * clock enable macros for GPIOx peripherals
 **/
#define GPIOA_PCLK_EN() 								(RCC->RCC_AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN()									(RCC->RCC_AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN()									(RCC->RCC_AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN()									(RCC->RCC_AHBENR |= (1 << 20))
#define GPIOE_PCLK_EN()									(RCC->RCC_AHBENR |= (1 << 21))
#define GPIOF_PCLK_EN()									(RCC->RCC_AHBENR |= (1 << 22))

/**
 * clock enable macros for I2Cx peripherals
 **/
#define I2C1_PCLK_EN()									(RCC->RCC_APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()									(RCC->RCC_APB1ENR |= (1<<22))

/**
 * clock enable macros for SPIx peripherals
 **/
#define SPI1_PCLK_EN()									(RCC->RCC_APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()									(RCC->RCC_APB1ENR |= (1<<14))

/**
 * clock enable macros for USARTx peripherals
 **/
#define USART1_PCLK_EN()								(RCC->RCC_APB2ENR |= (1<<14))
#define USART2_PCLK_EN()								(RCC->RCC_APB1ENR |= (1<<17))
#define USART3_PCLK_EN()								(RCC->RCC_APB1ENR |= (1<<18))
#define USART4_PCLK_EN()								(RCC->RCC_APB1ENR |= (1<<19))
#define USART5_PCLK_EN()								(RCC->RCC_APB1ENR |= (1<<20))
#define USART6_PCLK_EN()								(RCC->RCC_APB2ENR |= (1<<5))
#define USART7_PCLK_EN()								(RCC->RCC_APB2ENR |= (1<<6))
#define USART8_PCLK_EN()								(RCC->RCC_APB2ENR |= (1<<7))

/**
 * clock enable macros for SYSCFGx peripherals
 **/
#define SYSCFG_PCLK_EN()								(RCC->RCC_APB2ENR |= (1<<0))

/**
 * clock disable macros for GPIOx peripherals
 **/
#define GPIOA_PCLK_DI() 								(RCC->RCC_AHBENR &= ~(1<<17))
#define GPIOB_PCLK_DI()									(RCC->RCC_AHBENR &= ~(1<<18))
#define GPIOC_PCLK_DI()									(RCC->RCC_AHBENR &= ~(1<<19))
#define GPIOD_PCLK_DI()									(RCC->RCC_AHBENR &= ~(1<<20))
#define GPIOE_PCLK_DI()									(RCC->RCC_AHBENR &= ~(1<<21))
#define GPIOF_PCLK_DI()									(RCC->RCC_AHBENR &= ~(1<<22))

/**
 * clock disable macros for I2Cx peripherals
 **/
#define I2C1_PCLK_DI()									(RCC->RCC_APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()									(RCC->RCC_APB1ENR &= ~(1<<22))

/**
 * clock disable macros for SPIx peripherals
 **/
#define SPI1_PCLK_DI()									(RCC->RCC_APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()									(RCC->RCC_APB1ENR &= ~(1<<14))

/**
 * clock disable macros for USARTx peripherals
 **/
#define USART1_PCLK_DI()								(RCC->RCC_APB2ENR &= ~(1<<14))
#define USART2_PCLK_DI()								(RCC->RCC_APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()								(RCC->RCC_APB1ENR &= ~(1<<18))
#define USART4_PCLK_DI()								(RCC->RCC_APB1ENR &= ~(1<<19))
#define USART5_PCLK_DI()								(RCC->RCC_APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()								(RCC->RCC_APB2ENR &= ~(1<<5))
#define USART7_PCLK_DI()								(RCC->RCC_APB2ENR &= ~(1<<6))
#define USART8_PCLK_DI()								(RCC->RCC_APB2ENR &= ~(1<<7))
/**
 * clock disable macros for SYSCFGx peripherals
 **/
#define SYSCFG_PCLK_DI()								(RCC->RCC_APB2ENR &= ~(1<<0))

/**
 * macros to disable GPIOx peripherals
 **/
#define GPIOA_REG_RESET()								(RCC->RCC_AHBRSTR |= (1<<17))
#define GPIOB_REG_RESET()								(RCC->RCC_AHBRSTR |= (1<<18))
#define GPIOC_REG_RESET()								(RCC->RCC_AHBRSTR |= (1<<19))
#define GPIOD_REG_RESET()								(RCC->RCC_AHBRSTR |= (1<<20))
#define GPIOE_REG_RESET()								(RCC->RCC_AHBRSTR |= (1<<21))
#define GPIOF_REG_RESET()								(RCC->RCC_AHBRSTR |= (1<<22))



// some generic macros
#define ENABLE 											1
#define DISABLE 										0
#define SET 											ENABLE
#define RESET 											DISABLE
#define GPIO_PIN_SET									SET
#define GPIO_PIN_RESET									RESET







#include "stm32f072xx_gpio_driver.h"

#endif /* INC_STM32F072XX_H_ */
