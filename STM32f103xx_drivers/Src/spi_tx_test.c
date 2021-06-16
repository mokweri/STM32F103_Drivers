#include <string.h>
#include <stdint.h>
#include "stm32f103xx.h"

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
//	//MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
//	//NSS
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


	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_7) == 1)
		{
			//indicate();
			//enable the SPI1 peripheral
			SPI_PeripheralControl(SPI1handle.pSPIx,ENABLE);


			//first send length of information
			uint8_t dataLen = strlen(user_data);
			SPI_SendData(SPI1, &dataLen,1);

			//to send data
			SPI_SendData(SPI1handle.pSPIx,(uint8_t*)user_data,strlen(user_data));

			//lets confirm SPI is not busy
			while( SPI_GetFlagStatus(SPI1handle.pSPIx,SPI_BUSY_FLAG) );

			//Disable the SPI1 peripheral
			SPI_PeripheralControl(SPI1,DISABLE);
			indicate();

		}

	}
}

