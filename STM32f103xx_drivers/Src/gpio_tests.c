#include <string.h>
#include <stdint.h>
#include "stm32f103xx.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{
	GPIO_Handle_t LED, Btn;
	memset(&LED,0,sizeof(LED));
	memset(&Btn,0,sizeof(Btn));

	LED.pGPIOx = GPIOA;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&LED);

//	GPIO_Handle_t Btn;
//	Btn.pGPIOx = GPIOB;
//	Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_FL;
//	Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
//	GPIO_Init(&Btn);


	Btn.pGPIOx = GPIOB;
	Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIO_Interrupt_Setup(&Btn,IRQ_NO_EXTI9_5,5, ENABLE);

	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_10, SET);

	while(1)
	{
		/*
		 * LED Example
		 */
//		//GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_8, SET);
//		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_10);
//		delay();
////		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_8, RESET);
////		delay();

		/*
		 * Button Example
		 */
//		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_7) == 1)
//		{
//			delay();
//			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_10);
//		}

		/*
		 * Button Interrupt example
		 */



	}
}


void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_7); // clear the pending event from the EXTI line
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_10);
}
