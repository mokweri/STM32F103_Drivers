/*
 * stm32f103xx_usart.c
 *
 *  Created on: 12 Jul 2021
 *      Author: OBED
 */
#include "stm32f103xx_usart_driver.h"


void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCCK_EN();
		}else if (pUSARTx == USART2)
		{
			USART2_PCCK_EN();
		}else if (pUSARTx == USART3)
		{
			USART3_PCCK_EN();
		}
		else if (pUSARTx == UART4)
		{
			UART4_PCCK_EN();
		}else if(pUSARTx == UART5)
		{
			UART5_PCCK_EN();
		}
	}
	else
	{
		//TODO
	}
}

/*********************************************************************
 * @fn      		  - USART_Init
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
void USART_Init(USART_Handle_t *pUSARTHandle)
{

}

