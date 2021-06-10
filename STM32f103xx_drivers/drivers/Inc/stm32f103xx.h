/*
 * stm32f103xx.h
 *
 *  Created on: Jun 4, 2021
 *      Author: OBED
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include<stddef.h>
#include<stdint.h>


#define __vo volatile
#define __weak __attribute__((weak))

/**********************************START:Processor Specific Details ******************************/

/*
 * ARM Cortex M3 Processor NVIC ISERx register Addresses
 * Irq 0 to 239 Set Enable Register
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 ) 	//Irq 0-31
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )	//Irq 32-63
//#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )	//Irq 64-95
//#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )	//Irq 96-127


/*
 * ARM Cortex M3 Processor NVIC ICERx register Addresses
 * Irq 0 to 239 Clear Enable Register
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
//#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
//#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)


/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U   		/*!<explain this macro briefly here  */
#define SRAM1_BASEADDR						0x20000000U  		/*!<explain this macro briefly here  */
#define ROM_BASEADDR						0x1FFFF000U
#define SRAM 								SRAM1_BASEADDR


/*
 *  AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHBPERIPH_BASEADDR						0x40020000U

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */

#define GPIOA_BASEADDR                   (APB2PERIPH_BASEADDR + 0x0800)
#define GPIOB_BASEADDR                   (APB2PERIPH_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR 					 (APB2PERIPH_BASEADDR + 0x1000)
#define GPIOD_BASEADDR 					 (APB2PERIPH_BASEADDR + 0x1400)
#define GPIOE_BASEADDR 					 (APB2PERIPH_BASEADDR + 0x1800)
#define AFIO_BASEADDR					 (APB2PERIPH_BASEADDR)

#define EXTI_BASEADDR					 (APB2PERIPH_BASEADDR + 0x0400)

/*
 * Base addresses of peripherals which are hanging on AHB bus
 * TODO : Complete for all other peripherals
 */
#define RCC_BASEADDR                     (AHBPERIPH_BASEADDR + 0x1000)





/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */


/*
 * peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t CRL;                          /*!< GPIO port configuration registers CRL,      	        Address offset: 0x00      */
	__vo uint32_t CRH;                          /*!< GPIO port configuration registers CRH,      	        Address offset: 0x04      */
	__vo uint32_t IDR;							/*!< GPIO port input data register,                 		Address offset: 0x08      */
	__vo uint32_t ODR;							/*!< GPIO port output data register,                		Address offset: 0x0C      */
	__vo uint32_t BSRR;							/*!< GPIO port bit set/reset register,              		Address offset: 0x10      */
	__vo uint32_t BRR;							/*!< GPIO port bit reset register,              			Address offset: 0x14      */
	__vo uint32_t LCKR;							/*!< GPIO port configuration lock register,         		Address offset: 0x18       */
}GPIO_RegDef_t;


/**
  * @brief Alternate Function I/O
  */
typedef struct
{
	__vo uint32_t EVCR;			/*Event control register							0x00 */
	__vo uint32_t MAPR;			/*AF remap and debug I/O configuration register		0x04*/
	__vo uint32_t EXTICR[4];	/*External interrupt configuration register			0x08 */
	uint32_t RESERVED0;			/* */
	__vo uint32_t MAPR2;		/*AF remap and debug I/O configuration register2    0x1C */
} AFIO_RegDef_t;


/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
  __vo uint32_t CR;            /*!< RCC Control register							Address offset: 0x00 */
  __vo uint32_t CFGR;          /*!< RCC clock configuration register,				Address offset: 0x04 */
  __vo uint32_t CIR;           /*!< RCC Clock interrupt Register,     				Address offset: 0x08 */
  __vo uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,     		Address offset: 0x0C */
  __vo uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,    			Address offset: 0x10 */
  __vo uint32_t AHBENR;       /*!< RCC AHB1 peripheral clock enable register,     	Address offset: 0x14 */
  __vo uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,     	Address offset: 0x18 */
  __vo uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,     	Address offset: 0x1C */
  __vo uint32_t BDCR;          /*!< RCC Backup domain control register,     		Address offset: 0x20 */
  __vo uint32_t CSR;           /*!< RCC clock control & status register,     		Address offset: 0x24 */
} RCC_RegDef_t;


/**
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
	__vo uint32_t IMR;		/*!<Interrupt mask register	 			Address offset: 0x00 */
	__vo uint32_t EMR;		/*!<Event mask register		 			Address offset: 0x04 */
	__vo uint32_t RTSR;		/*!<Rising trigger selection register 	Address offset: 0x08 */
	__vo uint32_t FTSR;		/*!<Falling trigger selection register	Address offset: 0x0C */
	__vo uint32_t SWIER;	/*!<Software interrupt event register	Address offset: 0x10 */
	__vo uint32_t PR;		/*!<Pending register					Address offset: 0x14 */
} EXTI_RegDef_t;




/*
 * peripheral definitions ( Peripheral base addresses type casted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define AFIO				((AFIO_RegDef_t*)AFIO_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI                ((EXTI_RegDef_t*)EXTI_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()    	(RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()		(RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()		(RCC->APB2ENR |= (1 << 6))
//Alternate function enable
#define AFIOA_PCLK_EN()    	(RCC->APB2ENR |= (1 << 2) | (1 << 0))
#define AFIOB_PCLK_EN()		(RCC->APB2ENR |= (1 << 3) | (1 << 0))
#define AFIOC_PCLK_EN()		(RCC->APB2ENR |= (1 << 4) | (1 << 0))
#define AFIOD_PCLK_EN()		(RCC->APB2ENR |= (1 << 5) | (1 << 0))
#define AFIOE_PCLK_EN()		(RCC->APB2ENR |= (1 << 6) | (1 << 0))


/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 0)); (RCC->APB2RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 1)); (RCC->APB2RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); }while(0)




/*
 * This macro returns a code( between 0 to 4) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:0)

/*
 * IRQ(Interrupt Request) Numbers of STM32F103x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET         	RESET
#define FLAG_SET 			SET


#include "stm32f103xx_rcc_driver.h"
#include "stm32f103xx_gpio_driver.h"


#endif /* INC_STM32F103XX_H_ */
