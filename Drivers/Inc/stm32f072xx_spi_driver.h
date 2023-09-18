/*
 * stm32f072xx_spi_driver.h
 *
 *  Created on: Sep 11, 2023
 *      Author: ITC2KOR
 */

#ifndef INC_STM32F072XX_SPI_DRIVER_H_
#define INC_STM32F072XX_SPI_DRIVER_H_


#include "stm32f072xx.h"


/**
 * This is a configuration structure for SPIx peripheral
 **/

typedef struct{
	uint8_t SPI_DeviceMode;				/*!<possibel values from @SPI_DEVICE_MODES>*/
	uint8_t SPI_BusConfig;				/*!<possibel values from @SPI_BUS_CONFIG*/
	uint8_t SPI_SClkSpeed;				/*!<possibel values from @SPI_SERIAL_CLOCK_SPEED>*/
	uint8_t SPI_CRCL;					/*!<possibel values from @SPI_DATAFRAME_FORMAT>*/
	uint8_t SPI_CPOL;					/*!<possibel values from @SPI_CLOCK_POLARITY>*/
	uint8_t SPI_CPHA;					/*!<possibel values from @SPI_CLOCK_PHASE>*/
	uint8_t SPI_SSM;					/*!<possibel values from @SPI_SLAVE_SELECT_MANAGEMENT>*/
} SPI_PinConfig_t;

/**
 * This is a handle structure for SPIx peripheral
 **/

typedef struct{

	SPI_RegDef_t *pSPIx_addr;			/*!< This holds the base address of the SPIx(x:1,2) peripheral>*/
	SPI_PinConfig_t SPI_PinConfig;		/*!< This holds SPI pin configuration setting >*/
} SPI_Handle_t;

/**
 * @SPI_DEVICE_MODES
 * master or slave mode for SPIx peripheral
 **/

#define SPI_DEVICE_MODE_SLAVE					0
#define SPI_DEVICE_MODE_MASTER					1

/**
 * @SPI_BUS_CONFIG
 * Full Duplex, Half Duplex or Simplex for Bus Config of SPIx peripheral
 **/

#define SPI_BUS_CONFIG_FD						1
#define SPI_BUS_CONFIG_HD						2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY			3

/**
 * @SPI_SERIAL_CLOCK_SPEED
 * set the clock speed
 **/
#define SPI_SCLK_SPEED_DIV2						0
#define SPI_SCLK_SPEED_DIV4						1
#define SPI_SCLK_SPEED_DIV8						2
#define SPI_SCLK_SPEED_DIV16					3
#define SPI_SCLK_SPEED_DIV32					4
#define SPI_SCLK_SPEED_DIV64					5
#define SPI_SCLK_SPEED_DIV128					6
#define SPI_SCLK_SPEED_DIV256					7

/**
 * @SPI_DATAFRAME_FORMAT
 * bit length of the data communicated 8-bit or 16-bit
 **/
#define SPI_DATAFRAME_8BITS						0
#define SPI_DATAFRAME_16BITS					1

/**
 * @SPI_CLOCK_POLARITY
 * set the clock polarity
 **/
#define SPI_CPOL_LOW							0
#define SPI_CPOL_HIGH							1

/**
 * @SPI_CLOCK_PHASE
 * set the clock phase
 **/
#define SPI_CPHA_LOW							0
#define SPI_CPHA_HIGH							1

/**
 * @SPI_SLAVE_SELECT_MANAGEMENT
 * bit length of the data communicated 8-bit or 16-bit
 **/
#define SPI_SSM_DI								0
#define SPI_SSM_EN								1

/*
 * SPI related status flag definition
 */
#define SPI_TXE_FLAG							(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG							(1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG							(1 << SPI_SR_BSY)
/************************************************************************************************
 *                                API Supported by this driver
 *                 For more information about the API, please check the function definition
 ************************************************************************************************/

/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx_addr, uint8_t EnableOrDisable);

/*
 * Init and DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIhandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx_addr);

/*
 * data send and receive
 * polling (blocking based) and interrupt (non-blocking based) and DMA based
 */

void SPI_SendData(SPI_RegDef_t *pSPIx_addr, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx_addr,uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISRhandling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnableOrDisable);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other peripheral control API
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx_addr, uint8_t EnableOrDisable);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx_addr, uint8_t EnableOrDisable);
void SPI_SSOEControl(SPI_RegDef_t *pSPIx_addr, uint8_t EnableOrDisable);

#endif /* INC_STM32F072XX_SPI_DRIVER_H_ */
