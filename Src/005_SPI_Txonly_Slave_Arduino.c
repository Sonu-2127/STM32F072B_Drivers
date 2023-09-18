/*
 * 005_SPI_Txonly_Slave_Arduino.c
 *
 *  Created on: Sep 15, 2023
 *      Author: ITC2KOR
 */

#include "stm32f072xx.h"
#include <string.h>

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
	SPI2_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4; //NSS
	GPIO_Init(&SPI2_Pins);

}
void SPI2_Inits(void){
	SPI_Handle_t SPI2_handle;
	SPI2_handle.pSPIx_addr = SPI2;
	SPI2_handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_handle.SPI_PinConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_handle.SPI_PinConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV4; //HSI generates frequency of 8MHz---> SPI2 will get 2MHz clock cycle
	SPI2_handle.SPI_PinConfig.SPI_CRCL = SPI_DATAFRAME_8BITS;
	SPI2_handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_handle.SPI_PinConfig.SPI_SSM = SPI_SSM_DI; // SSM enabled for NSS pin

	SPI_Init(&SPI2_handle);
}

void PushButtonInit(void){
	GPIO_Handle_t PushBtn;
	PushBtn.pGPIOx_addr = GPIOA;
	PushBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	PushBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	PushBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	PushBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	PushBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&PushBtn);
}

void delay(void){
	for(uint32_t i = 0; i < 250000; i++);
}

int main(void){
	char user_data[] = "AAABBBCCC";

	// push button initialization
	PushButtonInit();

	// this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// This function is used to initialize the SPI type(1 or 2) with all the configuration
	SPI2_Inits();

	// Set the SSOE pin, to pull the NSS low till the SPI is enabled
	SPI_SSOEControl(SPI2, ENABLE);
	while(1){
		// hang till button is not pressed
		while(!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));

		// delay is created for button de-bounce
		delay();

		// This function is used to enable the SPI peripherals
		SPI_PeripheralControl(SPI2, ENABLE);

		// send the length information to the slave
		uint8_t dataLength = strlen(user_data);
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// this function is used to send the data using SPI
		// third parameter is one-byte data length
		SPI_SendData(SPI2, &dataLength, 1);

		// confirmation that SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		// This function is used to disable the SPI peripherals
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
