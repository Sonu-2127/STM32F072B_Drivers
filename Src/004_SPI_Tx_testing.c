/*
 * 004_SPI_Tx_testing.c
 *
 *  Created on: Sep 14, 2023
 *      Author: ITC2KOR
 */

#include <string.h>
#include "stm32f072xx.h"

//1. PB12---> SPI1_NSS
//2. PB13---> SPI1_SCK
//3. PB14---> SPI1_MISO
//4. PB15---> SPI1_MOSI
//ALT-Func mode is 0
void SPI2_GPIOInits(void){
	GPIO_Handle_t SPI2_Pins;
	SPI2_Pins.pGPIOx_addr = GPIOB;
	SPI2_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIOB_SPI2_MODE;
	SPI2_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI2_Pins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	SPI2_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SPI2_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13; //CLK
	GPIO_Init(&SPI2_Pins);

	SPI2_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15; //MOSI
	GPIO_Init(&SPI2_Pins);

//	SPI1_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6; //MISO
//	GPIO_Init(&SPI2_Pins);
//
//	SPI1_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4; //NSS
//	GPIO_Init(&SPI2_Pins);

}
void SPI2_Inits(void){
	SPI_Handle_t SPI2_handle;
	SPI2_handle.pSPIx_addr = SPI2;
	SPI2_handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_handle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_handle.SPI_PinConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV8; //HSI generates frequency of 8MHz---> SPI2 will get 1MHz clock cycle
	SPI2_handle.SPI_PinConfig.SPI_CRCL = SPI_DATAFRAME_8BITS;
	SPI2_handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_HIGH;
	SPI2_handle.SPI_PinConfig.SPI_SSM = SPI_SSM_EN; // SSM enabled for NSS pin

	SPI_Init(&SPI2_handle);
}

int main(void){
	char user_data[] = "AAABBBCCC";

	// this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// This function is used to initialize the SPI type(1 or 2) with all the configuration
	SPI2_Inits();

	// this makes NSS signal high and avoids MODF error
	SPI_SSIConfig(SPI2,ENABLE);

	// This function is used to enable the SPI peripherals
	SPI_PeripheralControl(SPI2, ENABLE);

	// this function is used to send the data using SPI
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	// confirmation that SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

	// This function is used to disable the SPI peripherals
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);
	return 0;
}
