/*
 * 001ToogleLED..c
 *
 *  Created on: Sep 7, 2023
 *      Author: ITC2KOR
 */
#include "stm32f072xx.h"

void manual_delay(void){
	for(uint32_t i = 0; i < 75000; i++);
}
void RedLed(void){
	GPIO_Handle_t gpio_led2;
	gpio_led2.pGPIOx_addr = GPIOC;
	gpio_led2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	gpio_led2.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&gpio_led2);
}

void OrangeLed(void){
	GPIO_Handle_t gpio_led1;
	gpio_led1.pGPIOx_addr = GPIOC;
	gpio_led1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	gpio_led1.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU;
	GPIO_Init(&gpio_led1);
}
void BlueLed(void){
	GPIO_Handle_t gpio_led3;
	gpio_led3.pGPIOx_addr = GPIOC;
	gpio_led3.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led3.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led3.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpio_led3.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	gpio_led3.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU;
	GPIO_Init(&gpio_led3);
}
void GreenLed(void){
	GPIO_Handle_t gpio_led4;
	gpio_led4.pGPIOx_addr = GPIOC;
	gpio_led4.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led4.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led4.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // push-pull(led will glow) or open-drain(led will not glow)
	gpio_led4.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	gpio_led4.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU; // no pull-up pull-down
	GPIO_Init(&gpio_led4);
}


int main(void){
	GPIO_PeriClockControl(GPIOC, ENABLE);
	RedLed();
//	GreenLed();
//	BlueLed();
//	OrangeLed();

	while(1){
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_6);
		manual_delay();
//		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_9);
//		manual_delay();
//		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_7);
//		manual_delay();
//		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_8);
//		manual_delay();
	}
	return 0;
}

