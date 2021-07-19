#include <stdio.h>
#include <string.h>
#include "stm32f103xx.h"
#include "lcd.h"

/*
 * USART1_TX = A9
 * USART1_RX = A10
 *
 */

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

void indicate()
{
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, SET);
	delay();
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_12, RESET);
}

USART_Handle_t usart3_handle;

char msg[1024] = "USART Tx testing.\n\r";

void USART3_GPIOInits(void)
{
	GPIO_Handle_t USART3Pins;

	USART3Pins.pGPIOx = GPIOB;
	USART3Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_PP;

	//USART1 TX
	AFIO_PeriClockControl(USART3Pins.pGPIOx, ENABLE);
	USART3Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&USART3Pins);

	//USART1 RX
	USART3Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIO_Init(&USART3Pins);

}

void USART3_Init(void)
{
	usart3_handle.pUSARTx = USART3;
	usart3_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart3_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart3_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart3_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart3_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart3_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart3_handle);

}

void Button_Init(void)
{
	GPIO_Handle_t Btn;

	Btn.pGPIOx = GPIOB;
	Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_PP;
	Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;

	GPIO_Init(&Btn);
}

void LED_Init(void)
{
	GPIO_Handle_t LED;

	LED.pGPIOx = GPIOB;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;

	GPIO_Init(&LED);
}

int main(void)
{
	lcd_init();
	LCD_PrintString("Sayari", 0, 0);

	Button_Init();
	LED_Init();

	USART3_GPIOInits();
	USART3_Init();
	USART_PeripheralControl(USART3, ENABLE);


	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_0) );
		indicate();

		delay();

		USART_SendData(&usart3_handle, (uint8_t*)msg, strlen(msg));

	}

	return 0;
}



