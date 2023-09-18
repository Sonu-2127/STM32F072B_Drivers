/*
 * 002UserButtonLedGlow.c
 *
 *  Created on: Sep 8, 2023
 *      Author: ITC2KOR
 */


#include "stm32f072xx.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED HIGH
void manual_delay(void){
	for(uint32_t i = 0; i < 250000; i++);
}

void RedLed(void){
	GPIO_Handle_t gpio_led1;
	gpio_led1.pGPIOx_addr = GPIOC;
	gpio_led1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	gpio_led1.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&gpio_led1);
}
void PushButton(void){
	GPIO_Handle_t PushBtn;
	PushBtn.pGPIOx_addr = GPIOA;
	PushBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	PushBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	PushBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	PushBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	PushBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&PushBtn);
}
int main(void){
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);
	RedLed();

	while(1){
		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) == BTN_PRESSED){
			manual_delay();
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_6);
		}
	}
	return 0;
}
