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

/**************************START PROCESSOR SPECIFIC DETAILS**************************
* ARM Cortex M0 Processor NVIC Register Address
**/
#define NVIC_ISER										((__vo uint32_t*)0xE000E100)
#define NVIC_ICER										((__vo uint32_t*)0xE000E180)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation*
 */
#define NVIC_PR_BASE_ADDR								((__vo uint32_t*)(0xE000E400))

#define NO_OF_BITS_INCLUIDED							4
/*
 * base addresses of FLASH and SRAM memories, info
 * available in boot configuration in reference manual
 */

#define FLASH_BASE_ADDR 								(uint32_t)0x08000000	// Main flash memory
#define SRAM_BASE_ADDR								 	(uint32_t)0x20000000	// SRAM
#define ROM_BASE_ADDR								 	(uint32_t)0x1FFFC800	// System memory

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

/**
 *  peripheral register definition structure to SPI
 **/
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;

/**
 *  peripheral register definition structure to GPIO
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

/**
 *  peripheral register definition structure to RCC
 **/

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
 *  peripheral register definition structure to EXTI
 **/
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_RegDef_t;

/**
 *  peripheral register definition structure to SYSCFG
 **/
typedef struct{
	__vo uint32_t CFGR1;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CFGR2;
	__vo uint32_t ITLINE[31];
} SYSCFG_RegDef_t;

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

#define EXTI											((EXTI_RegDef_t*)(EXTI_BASE_ADDR))
#define SYSCFG											((SYSCFG_RegDef_t*)(SYSCFGCOMP_BASE_ADDR))
#define SPI1											((SPI_RegDef_t*)(SPI1_BASE_ADDR))
#define SPI2											((SPI_RegDef_t*)(SPI2_BASE_ADDR))


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

/**
 * macros to disable SPIx peripherals
 **/
#define SPI1_REG_RESET()								(RCC->RCC_APB2RSTR |= (1<<12))
#define SPI2_REG_RESET()								(RCC->RCC_APB1RSTR |= (1<<14))


/*
 * IRQ(Interrupt Request Number) for STM32F072RB
 * IRQ number for different EXTI
 * Note: Further more details can be added
 **/
#define IRQ_NO_EXTI0_1									5
#define IRQ_NO_EXTI2_3									6
#define IRQ_NO_EXTI4_15									7

/*
 * macros for all the priority
 */
#define NVIC_IRQ_PRIORITY0								0
#define NVIC_IRQ_PRIORITY1								1
#define NVIC_IRQ_PRIORITY2								2
#define NVIC_IRQ_PRIORITY3								3
#define NVIC_IRQ_PRIORITY4								4
#define NVIC_IRQ_PRIORITY5								5
#define NVIC_IRQ_PRIORITY6								6
#define NVIC_IRQ_PRIORITY7								7
#define NVIC_IRQ_PRIORITY8								8
#define NVIC_IRQ_PRIORITY9								9
#define NVIC_IRQ_PRIORITY10								10
#define NVIC_IRQ_PRIORITY11								11
#define NVIC_IRQ_PRIORITY12								12
#define NVIC_IRQ_PRIORITY13								13
#define NVIC_IRQ_PRIORITY14								14
#define NVIC_IRQ_PRIORITY15								15


// some generic macros
#define ENABLE 											1
#define DISABLE 										0
#define SET 											ENABLE
#define RESET 											DISABLE
#define GPIO_PIN_SET									SET
#define GPIO_PIN_RESET									RESET
#define FLAG_RESET										RESET
#define FLAG_SET										SET

#define GPIO_BASEADDR_TO_CODE(x)						((x == GPIOA) ? 0:\
														(x == GPIOB) ? 1:\
														(x == GPIOC) ? 2:\
														(x == GPIOD) ? 3:\
														(x == GPIOE) ? 4:\
														(x == GPIOF) ? 5:0)


/*****************************************************************************************************
 *Bit position definition of SPI peripheral
 ****************************************************************************************************/
#define SPI_CR1_CPHA									0
#define SPI_CR1_CPOL									1
#define SPI_CR1_MSTR									2
#define SPI_CR1_BR										3
#define SPI_CR1_SPE										6
#define SPI_CR1_LSBFIRST								7
#define SPI_CR1_SSI										8
#define SPI_CR1_SSM										9
#define SPI_CR1_RXONLY									10
#define SPI_CR1_CRCL									11
#define SPI_CR1_CRCNEXT									12
#define SPI_CR1_CRCEN									13
#define SPI_CR1_BIDIOE									14
#define SPI_CR1_BIDIMODE								15


#define SPI_CR2_RXDMAEN									0
#define SPI_CR2_TXDMAEN									1
#define SPI_CR2_SSOE									2
#define SPI_CR2_NSSP									3
#define SPI_CR2_FRF										4
#define SPI_CR2_ERRIE									5
#define SPI_CR2_RXNEIE									6
#define SPI_CR2_TXEIE									7
#define SPI_CR2_DS										8
#define SPI_CR2_FRXT_H									12
#define SPI_CR2_LDMA_RX									13
#define SPI_CR2_LDMA_TX									14


#define SPI_SR_RXNE										0
#define SPI_SR_TXE										1
#define SPI_SR_CHSIDE									2
#define SPI_SR_UDR										3
#define SPI_SR_CRCERR									4
#define SPI_SR_MODF										5
#define SPI_SR_OVR										6
#define SPI_SR_BSY										7
#define SPI_SR_FRE										8
#define SPI_SR_FRLV										9
#define SPI_SR_FTLVL									11

#include "stm32f072xx_gpio_driver.h"
#include "stm32f072xx_spi_driver.h"

#endif /* INC_STM32F072XX_H_ */
