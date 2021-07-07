#include <stdio.h>
#include <string.h>
#include "stm32f103xx.h"


#define MY_ADDR 0x61;
#define SLAVE_ADDR  0x68

/*
 * I2C1_SCL = PB6
 * I2C1_SDA = PB7
 *
 * I2C2_SCL = PB10
 * I2C2_SDA = PB11
 */

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

void indicate()
{
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_10, SET);
	delay();
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_10, RESET);
}

I2C_Handle_t I2C1Handle;

//some data
uint8_t some_data[] = "Testing the I2C master Tx\n";

//rcv buffer
uint8_t rcv_buf[32];

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2C1Pins;

	/*Note : External pull-up resistors are used */
	I2C1Pins.pGPIOx = GPIOB;
	I2C1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_OD;

	//SCL
	AFIO_PeriClockControl(I2C1Pins.pGPIOx, ENABLE);
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2C1Pins);

	//SDA
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2C1Pins);

}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
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
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;

	GPIO_Init(&LED);
}

int main(void)
{
	uint8_t commandcode;
	uint8_t len;

	Button_Init();
	LED_Init();

	I2C1_GPIOInits();
	I2C1_Inits();

	//enable i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_0) );
		indicate();

		delay();

		/*
		I2C_MasterSendData(&I2C1Handle, some_data,strlen((char*)some_data), SLAVE_ADDR, 0);
		*/
		commandcode = 0x51;

		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1Handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR);

		rcv_buf[len+1] = '\0';

		//printf("Data: %s",rcv_buf);

	}

	return 0;
}


