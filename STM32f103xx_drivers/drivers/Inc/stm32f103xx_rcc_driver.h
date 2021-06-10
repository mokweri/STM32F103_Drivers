/*
 * stm32f103xx_rcc.h
 *
 *  Created on: Jun 4, 2021
 *      Author: OBED
 */

#ifndef INC_STM32F103XX_RCC_DRIVER_H_
#define INC_STM32F103XX_RCC_DRIVER_H_

#include "stm32f103xx.h"

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F103XX_RCC_DRIVER_H_ */
