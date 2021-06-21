#include <stdio.h>
#include <string.h>
#include "stm32f103xx.h"


SPI_Handle_t SPI1handle;

#define MAX_LEN 500
char RcvBuff[MAX_LEN];

volatile char ReadByte;
volatile uint8_t recvStop = 0;
volatile uint8_t dataAvailable = 0; /*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */

/*
 * PA6 --> SPI1_MISO
 * PA7 --> SPI1_MOSI
 * PA5 --> SPI1_SCLK
 * PA4 --> SPI2_NSS
 */

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

/*This function configures the gpio pin over which SPI peripheral issues data available interrupt */
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiINTpin;
	memset(&spiINTpin,0,sizeof(spiINTpin));

	spiINTpin.pGPIOx = GPIOB;
	spiINTpin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	spiINTpin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;

	GPIO_Interrupt_Setup(&spiINTpin,IRQ_NO_EXTI9_5,5, ENABLE);

}


int main(void)
{
	GPIO_Handle_t LED;

	LED.pGPIOx = GPIOB;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Init(&LED);

	Slave_GPIO_InterruptPinInit();

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

	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_MASTER_MODE;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; //SPI_SCLK_SPEED_DIV8 generated sclk of 2MHz
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management enabled for NSS pin

	SPI_Init(&SPI1handle);
	SPI_SSOEConfig(SPI1, ENABLE);
	SPI_IRQInterruptConfig(IRQ_NO_SPI1, ENABLE);



	uint8_t dummy = 0xff;
	printf("started\n");



	while(1)
	{
		recvStop = 0;

		while(!dataAvailable); //wait till data available interrupt frame transmitter device(slave)


		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);
		//enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, ENABLE);

		while(!recvStop)
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while( SPI_SendDataIT(&SPI1handle, &dummy, 1) == SPI_BUSY_IN_TX);
			while( SPI_ReceiveDataIT(&SPI1handle, &ReadByte, 1)  == SPI_BUSY_IN_RX);
		}

		// confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI1,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI1,DISABLE);

		printf("Rcvd data = %s\n",RcvBuff);
		indicate();

		dataAvailable = 0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

//		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_7) == 1)
//		{
//		}
	}

	return 0;
}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI1_IRQHandler(void)
{
	SPI_IRQHandling(&SPI1handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i = 0;

	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		RcvBuff[i++] = ReadByte;
		if(ReadByte == '\0' || (i == MAX_LEN))
		{
			recvStop = 1;
			RcvBuff[i-1] = '\0';
			i=0;
		}

	}
}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_5); // clear the pending event from the EXTI line
	dataAvailable = 1;
	//indicate();
}

