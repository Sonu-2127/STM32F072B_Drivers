/*
 * stm32f072xx_spi_driver.c
 *
 *  Created on: Sep 12, 2023
 *      Author: ITC2KOR
 */
#include "stm32f072xx_spi_driver.h"

/*
 * Peripheral clock setup
 */
/*******************************************************************************
 *  @fn					- SPI_Init
 *
 *  @brief				- This API controls the SPI peripheral clock
 *
 *  @param[in]			- Base address of the SPI Peripheral
 *  @param[in]			- ENABLE or DISABLE macros
 *
 *  @return				- None
 *
 *  @Note				- None
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx_addr, uint8_t EnableOrDisable){
	if(EnableOrDisable == ENABLE){
		if(pSPIx_addr == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx_addr == SPI2){
			SPI2_PCLK_EN();
		}
	}
	else if(EnableOrDisable == DISABLE){
		if(pSPIx_addr == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx_addr == SPI2){
			SPI2_PCLK_DI();
		}
	}
}

/*
 * Init
 */
/*******************************************************************************
 *  @fn					- SPI_Init
 *
 *  @brief				- This API controls the SPI Initialization
 *
 *  @param[in]			- SPI handle pointer
 *
 *  @return				- None
 *
 *  @Note				- None
 *
 */
void SPI_Init(SPI_Handle_t *pSPIhandle){
	// Enbale the SPI peripheral clock
	SPI_PeriClockControl(pSPIhandle->pSPIx_addr, ENABLE);
	//configure SPI_CR1 Register
	uint32_t tempreg = 0;

	//1. SPI Device mode selection (master or slave)
	tempreg |= (pSPIhandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	//2. configure SPI bus as full duplex, half duplex or slave
	if(pSPIhandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// BIDI MODE should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIhandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// BIDI MODE should be set
		tempreg |= (SET << SPI_CR1_BIDIMODE);
	}
	else if(pSPIhandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// BIDIOE should be disabled
		tempreg &= ~(SET << SPI_CR1_BIDIMODE);
		// and RXONLY bit must be set
		tempreg |= (SET << SPI_CR1_RXONLY);
	}
	//3. set the clock speed of the SPI
	tempreg |= (pSPIhandle->SPI_PinConfig.SPI_SClkSpeed << SPI_CR1_BR);

	//4. data-frame format (8 or 16 bit)
	tempreg |= (pSPIhandle->SPI_PinConfig.SPI_CRCL << SPI_CR1_CRCL);

	//5. set the clock polarity as 0 or 1
	tempreg |= (pSPIhandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6. set the clock phase as 0 0r 1
	tempreg |= (pSPIhandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA);

	//7. SSM management
	if(pSPIhandle->SPI_PinConfig.SPI_SSM == SPI_SSM_EN){
		tempreg |= (pSPIhandle->SPI_PinConfig.SPI_SSM << SPI_CR1_SSM);
	}

	pSPIhandle->pSPIx_addr->CR1 = tempreg;
}

/*
 * De-Init
 */

/*******************************************************************************
 *  @fn					- SPI-DeInit
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
void SPI_DeInit(SPI_RegDef_t *pSPIx_addr){
	if(pSPIx_addr == SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx_addr == SPI2){
		SPI2_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx_addr, uint32_t FlagName){
	if(pSPIx_addr->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*******************************************************************************
 *  @fn					- SPI_SendData
 *
 *  @brief				- This API is for sending the data
 *
 *  @param[in]			- Base address of the SPI Peripheral
 *  @param[in]			- pointer to the data buffer
 *  @param[in]			- length of the buffer
 *
 *  @return				- This is a blocking call
 *
 *  @Note				- None
 *
 */

void SPI_SendData(SPI_RegDef_t *pSPIx_addr, uint8_t *pTxBuffer, uint32_t Len){
		while(Len > 0){
			//1. wait till TXE bit is set
			while(SPI_GetFlagStatus(pSPIx_addr, SPI_TXE_FLAG) == FLAG_RESET);

			//2. check the DFF/CRCL bit in CR1
			if(pSPIx_addr->CR1 & (1 << SPI_CR1_CRCL)){
				// 16 bit DataFrame
				//1. load the data into the data register in 16 bit format
				pSPIx_addr->DR = *((uint16_t*)pTxBuffer);
				Len--;
				Len--;
				(uint16_t*)pTxBuffer++;
			}
			else{
				//8 bit DataFrame
				//1. load the data into the data register in 8 bit format
				pSPIx_addr->DR = *pTxBuffer;
				Len--;
				pTxBuffer++;
			}
		}
}

/*******************************************************************************
 *  @fn					- SPI_ReceiveData
 *
 *  @brief				- This API is for sending the data
 *
 *  @param[in]			- Base address of the SPI Peripheral
 *  @param[in]			- pointer to the data buffer
 *  @param[in]			- length of the buffer
 *
 *  @return				- This is a blocking call
 *
 *  @Note				- None
 *
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx_addr,uint8_t *pRxBuffer, uint32_t Len){
		while(Len > 0){
			//1. wait till RXNE bit is set
			while(SPI_GetFlagStatus(pSPIx_addr, SPI_RXNE_FLAG) == FLAG_RESET);

			//2. check the DFF/CRCL bit in CR1
			if(pSPIx_addr->CR1 & (1 << SPI_CR1_CRCL)){
				// 16 bit DataFrame
				//1. load the data from DR into the RxBuffer address
				*((uint16_t*)pRxBuffer) = pSPIx_addr->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}
			else{
				//8 bit DataFrame
				//1. load the data into the data register in 8 bit format
				*pRxBuffer = pSPIx_addr->DR;
				Len--;
				pRxBuffer++;
			}
		}
}

/*******************************************************************************
 *  @fn					- SPI_PeripheralControl
 *
 *  @brief				- This API is for making the peripheral of SPI
 *
 *  @param[in]			- Base address of the SPI Peripheral
 *  @param[in]			- enable or disable
 *
 *
 *  @return				- This is a blocking call
 *
 *  @Note				- None
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx_addr, uint8_t EnableOrDisable){
	if(EnableOrDisable == ENABLE){
		pSPIx_addr->CR1 |= (1 << SPI_CR1_SPE);
	}
	else if(EnableOrDisable == DISABLE){
		pSPIx_addr->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx_addr, uint8_t EnableOrDisable){
	if(EnableOrDisable == ENABLE){
		pSPIx_addr->CR1 |= (SET << SPI_CR1_SSI);
	}
	else{
		pSPIx_addr->CR1 &= ~(SET << SPI_CR1_SSI);
	}
}

void SPI_SSOEControl(SPI_RegDef_t *pSPIx_addr, uint8_t EnableOrDisable){
	if(EnableOrDisable == ENABLE){
		pSPIx_addr->CR2 |= (SET << SPI_CR2_SSOE);
	}
	else{
		pSPIx_addr->CR2 &= ~(SET << SPI_CR2_SSOE);
	}
}
