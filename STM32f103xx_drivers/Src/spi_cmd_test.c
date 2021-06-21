#include <string.h>
#include <stdint.h>
#include "stm32f103xx.h"

//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1


//arduino led
#define LED_PIN  9


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

void indicate()
{
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, SET);
	delay();
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_0, RESET);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{

	if(ackbyte == (uint8_t)0xF5)
	{
		//ack
		return 1;
	}
	return 0;
}

int main(void)
{
	char user_data[] = "Hello world";
	GPIO_Handle_t LED, Btn;

	LED.pGPIOx = GPIOB;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Init(&LED);

	Btn.pGPIOx = GPIOB;
	Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_FL;
	Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&Btn);

	GPIO_WriteToOutputPin(LED.pGPIOx, GPIO_PIN_NO_0, RESET);


	//--------Initialize SPI pins
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_PP;
	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	AFIO_PeriClockControl(SPIPins.pGPIOx, ENABLE);
	GPIO_Init(&SPIPins);
	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);
	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);
	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);

	//--------Initialize SPI peripheral
	SPI_Handle_t SPI1handle;
	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_MASTER_MODE;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generated sclk of 2MHz
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management enabled for nss pin
	SPI_Init(&SPI1handle);
	SPI_SSOEConfig(SPI1, ENABLE);

//	//this makes NSS signal internally high and avoids MODF error
//	SPI_SSIConfig(SPI1handle.pSPIx,ENABLE);

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;


	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_7) == 1)
		{
			//indicate();
			//enable the SPI1 peripheral
			SPI_PeripheralControl(SPI1handle.pSPIx,ENABLE);

			//1. CMD_LED_CTRL  	<pin no(1)>     <value(1)>
			uint8_t commandcode = COMMAND_LED_CTRL;
			uint8_t ackbyte;
			uint8_t args[2];

			indicate();
			//send command
			SPI_SendData(SPI1,&commandcode,1);
			//-------------
			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI1,&dummy_read,1);

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI1,&dummy_write,1);

			//read the ack byte received
			SPI_ReceiveData(SPI1,&ackbyte,1);

			if( SPI_VerifyResponse(ackbyte))
			{
				args[0] = LED_PIN;
				args[1] = LED_ON;

				//send arguments
				SPI_SendData(SPI1,args,2);
				// dummy read
				SPI_ReceiveData(SPI1,args,2);

			}
			//end of COMMAND_LED_CTRL


			//lets confirm SPI is not busy
			while( SPI_GetFlagStatus(SPI1handle.pSPIx,SPI_BUSY_FLAG) );

			//Disable the SPI1 peripheral
			SPI_PeripheralControl(SPI1,DISABLE);
			//indicate();

		}

	}
}

