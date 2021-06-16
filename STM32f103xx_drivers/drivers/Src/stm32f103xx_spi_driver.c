/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: 15 Jun 2021
 *      Author: OBED
 */

#include "stm32f103xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - Enable the SPI peripheral clock
 *
 * @param[in]         -SPIx
 * @param[in]         -Enable or Disable
 *
 * @return            -None
 *
 * @Note              -

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(pSPIx == SPI1){
			SP1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SP2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SP3_PCLK_EN();
		}

	}else{
		if(pSPIx == SPI1){
			SP1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SP2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SP3_PCLK_DI();
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             -Setting up the SPI peripheral
 *
 * @param[in]         -SPI handle structure
 *
 * @return            -None
 *
 * @Note              -

 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) //Full duplex
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM; //Software slave management

	pSPIHandle->pSPIx->CR1 = tempreg;
}


/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
 //todo
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         - SPI peripheral register structure
 * @param[in]         - pointer to transmit data
 * @param[in]         - data length
 *
 * @return            - None
 *
 * @Note              - This is blocking call

 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			//16 bit DFF
			//load data in to the data register DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_ReceiveData
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1. wait until RXNE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );

			//2. check the DFF bit in CR1
			if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
			{
				//16 bit DFF
				//1. load the data from DR to Rxbuffer address
				 *((uint16_t*)pRxBuffer) = pSPIx->DR ;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8 bit DFF
				*(pRxBuffer) = pSPIx->DR ;
				Len--;
				pRxBuffer++;
			}
		}

}


/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         - Enable or Disable
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This bit has an effect only when the SSM bit is set.
 * 						he value of this bit is forced onto the NSS pin and the IO value of the NSS pin is ignored.
 */
void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - SS output enable

 */
void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE); //SS output is enabled in master mode and when the cell is enabled. The cell cannot work in a multimaster environment
	}else
	{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE); //SS output is disabled in master mode and the cell can work in multimaster configuration
	}

}




