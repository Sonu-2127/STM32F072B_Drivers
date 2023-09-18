/*
 * 003ButtonInterrupt.c
 *
 *  Created on: Sep 11, 2023
 *      Author: ITC2KOR
 */

#include "stm32f072xx.h"


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

void PushButton(void){
	GPIO_Handle_t PushBtn;
	PushBtn.pGPIOx_addr = GPIOA;
	PushBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RE_FE_TRIG;
	PushBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	PushBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	PushBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&PushBtn);
}
int main(void){
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);
	OrangeLed();
	PushButton();

	// IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0_1, NVIC_IRQ_PRIORITY0);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0_1, ENABLE);
	while(1);

	return 0;
}

void EXTI0_1_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_8);
}
