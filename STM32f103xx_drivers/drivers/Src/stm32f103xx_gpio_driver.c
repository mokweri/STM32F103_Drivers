/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Jun 4, 2021
 *      Author: OBED
 */

#include "stm32f103xx_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}

	}
	else
	{
		//Disable
		//TODO
	}

}

void AFIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			AFIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			AFIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			AFIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			AFIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			AFIOE_PCLK_EN();
		}
	}
}


/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;

	 //enable the peripheral clock

	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1 . configure the mode of gpio pin

	if((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode != GPIO_MODE_IT_RT) && (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode != GPIO_MODE_IT_RFT))
	{
		//the non interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->CRL &= ~(0xF << (4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing nibble
			pGPIOHandle->pGPIOx->CRL |= temp; //setting
		}else{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8)));
			pGPIOHandle->pGPIOx->CRH &= ~(0xF << (4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8))); //clearing nibble
			pGPIOHandle->pGPIOx->CRH |= temp; //setting
		}

	}else
	{
		//this part will code later. ( interrupt mode)
	}

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value =(uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001 );

	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value =(uint16_t)pGPIOx->IDR;

	return value;
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

void GPIO_Interrupt_Setup(GPIO_Handle_t *pGPIOHandle, uint8_t IRQNumber,uint32_t IRQPriority, uint8_t EnorDi)
{
	AFIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//set pin as input
	//pGPIOHandle->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_FL;

	//Configure the GPIO port selection in AFIO_EXTICR -- External interrupt configuration register
	uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
	uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
	uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
	AFIO->EXTICR[temp1] = portcode << (temp2 * 4);

	//set IT mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
	{
		EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //configure the FTSR
		EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear the RTSR bit

	}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
	{
		EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //configure the RTSR
		EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear the FTSR bit

	}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
	{
		//Configure both FTSR and RTSR
		EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}


	//Enable the EXTI interrupt delivery using IMR
	EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	//Set IRQ priority
	GPIO_IRQPriorityConfig(IRQNumber, IRQPriority);

	//Set IRQ number and enable Interrupt reception on processor
	GPIO_IRQInterruptConfig(IRQNumber, EnorDi);

}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}

	}else{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//there are four bits of priority, the priority value is stored in bits [7:4] (MSB first) of the byte.
	// find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & (1<< PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}

