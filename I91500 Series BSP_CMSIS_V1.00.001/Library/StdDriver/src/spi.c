/****************************************************************************//**
 * @file     spi.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/10/29 10:00a $
 * @brief    I91500 SPI driver source file
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_SPI_Driver SPI Driver
  @{
*/

/** @addtogroup I91500_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief  This function make SPI module be ready to transfer.
  *         By default, the SPI transfer sequence is MSB first and
  *         the automatic slave select function is disabled. In
  *         Slave mode, the u32BusClock must be NULL and the SPI clock
  *         divider setting will be 0.
  * @param  spi is the base address of SPI module.
  * @param  u32MasterSlave decides the SPI module is operating in master mode or in slave mode. Valid values are:
  *                    - \ref SPI_SLAVE
  *                    - \ref SPI_MASTER
  * @param  u32SPIMode decides the transfer timing. Valid values are:
  *                    - \ref SPI_MODE_0
  *                    - \ref SPI_MODE_1
  *                    - \ref SPI_MODE_2
  *                    - \ref SPI_MODE_3
  * @param  u32DataWidth decides the data width of a SPI transaction.
  * @param  u32BusClock is the expected frequency of SPI bus clock in Hz.
  * @return Actual frequency of SPI peripheral clock.
  */
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode,  uint32_t u32DataWidth, uint32_t u32BusClock)
{
    // Config SPI's control register.
	spi->CTL = ( u32MasterSlave | (((u32DataWidth>=32)?0:u32DataWidth) << SPI_CTL_DWIDTH_Pos) | (u32SPIMode) );
	// Set SPI's bus clock and return real frequency.
	return SPI_SetBusClock(spi,u32BusClock);
}

/**
  * @brief Disable SPI peripheral clock.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI_Close(SPI_T *spi)
{
	
}

/**
  * @brief Clear Rx FIFO buffer.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI_ClearRxFIFO(SPI_T *spi)
{
    spi->FIFOCTL |= SPI_FIFOCTL_RXRST_Msk;
}

/**
  * @brief Clear Tx FIFO buffer.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI_ClearTxFIFO(SPI_T *spi)
{
    spi->FIFOCTL |= SPI_FIFOCTL_TXRST_Msk;
}

/**
  * @brief Disable the automatic slave select function.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI_DisableAutoSS(SPI_T *spi)
{
    spi->SSCTL &= ~SPI_SSCTL_AUTOSS_Msk;
}

/**
  * @brief Enable the automatic slave select function. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @param  u32SSPinMask specifies slave select pins. Valid values are:
  *                     - \ref SPI_SS0
  *                     - \ref SPI_SS1
  * @param  u32ActiveLevel specifies the active level of slave select signal. Valid values are:
  *                     - \ref SPI_SS_ACTIVE_HIGH
  *                     - \ref SPI_SS_ACTIVE_LOW
  * @return none
  */
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    spi->SSCTL |= (u32SSPinMask | u32ActiveLevel) | SPI_SSCTL_AUTOSS_Msk;
}

/**
  * @brief Set the SPI bus clock. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @param  u32BusClock is the expected frequency of SPI bus clock.
  * @return Actual frequency of SPI peripheral clock.
  */
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock)
{
    uint32_t u32Div = 0xffff,u32Pclk = CLK_GetHCLKFreq();

    if( u32BusClock !=0 ) 
	{
        u32Div = (u32Pclk/u32BusClock) - 1;
        spi->CLKDIV = (spi->CLKDIV & ~SPI_CLKDIV_DIVIDER_Msk) | ((u32Div>0xff)?0xff:u32Div);
    } 
	else
        spi->CLKDIV = 0;

    return (u32Pclk/((spi->CLKDIV& SPI_CLKDIV_DIVIDER_Msk)+1));
}

/**
  * @brief Set Tx FIFO threshold and Rx FIFO threshold configurations.
  * @param  spi is the base address of SPI module.
  * @param  u32TxThreshold decides the Tx FIFO threshold.
  * @param  u32RxThreshold decides the Rx FIFO threshold.
  * @return none
  */
void SPI_SetFIFOThreshold(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold)
{
    spi->FIFOCTL &= (~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk));
	spi->FIFOCTL |= (u32TxThreshold << SPI_FIFOCTL_TXTH_Pos);
	spi->FIFOCTL |= (u32RxThreshold << SPI_FIFOCTL_RXTH_Pos);
}

/**
  * @brief Get the actual frequency of SPI bus clock. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @return Actual SPI bus clock frequency.
  */
uint32_t SPI_GetBusClock(SPI_T *spi)
{
    return (CLK_GetHCLKFreq()/ ((spi->CLKDIV&0xff) + 1));
}

/**
  * @brief Enable FIFO related interrupts specified by u32Mask parameter.
  * @param  spi is the base address of SPI module.
  * @param  u32Mask is the combination of all related interrupt enable bits.
  *         Each bit corresponds to a interrupt bit.
  *         This parameter decides which interrupts will be enabled. Valid values are:
  *           - \ref SPI_UNITIEN_MASK
  *           - \ref SPI_SSINAIEN_MASK
  *           - \ref SPI_SSACTIEN_MASK
  *           - \ref SPI_SLVURIEN_MASK
  *           - \ref SPI_SLVBEIEN_MASK
  *           - \ref SPI_SLVTOIEN_MASK
  *           - \ref SPI_FIFO_TXTHIEN_MASK
  *           - \ref SPI_FIFO_RXTHIEN_MASK
  *           - \ref SPI_FIFO_RXOVIEN_MASK
  *           - \ref SPI_FIFO_TXUFIEN_MASK
  *           - \ref SPI_FIFO_RXTOIEN_MASK
  * @return none
  */
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask)
{
    if(u32Mask & SPI_UNITIEN_MASK)
        spi->CTL |= SPI_CTL_UNITIEN_Msk;

    if(u32Mask & SPI_SSINAIEN_MASK)
        spi->SSCTL |= SPI_SSCTL_SSINAIEN_Msk;

    if(u32Mask & SPI_SSACTIEN_MASK)
        spi->SSCTL |= SPI_SSCTL_SSACTIEN_Msk;

    if(u32Mask & SPI_SLVURIEN_MASK)
        spi->SSCTL |= SPI_SSCTL_SLVUDRIEN_Msk;

    if(u32Mask & SPI_SLVBEIEN_MASK)
        spi->SSCTL |= SPI_SSCTL_SLVBCEIEN_Msk;

    if(u32Mask & SPI_SLVTOIEN_MASK)
        spi->SSCTL |= SPI_SSCTL_SLVTOIEN_Msk;

    if(u32Mask & SPI_FIFO_TXTHIEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_TXTHIEN_Msk;

    if(u32Mask & SPI_FIFO_RXTHIEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_RXTHIEN_Msk;

    if(u32Mask & SPI_FIFO_RXOVIEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_RXOVIEN_Msk;

    if(u32Mask & SPI_FIFO_TXUFIEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_TXUDFIEN_Msk;

    if(u32Mask & SPI_FIFO_RXTOIEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_RXTOIEN_Msk;
}

/**
  * @brief Disable FIFO related interrupts specified by u32Mask parameter.
  * @param  spi is the base address of SPI module.
  * @param  u32Mask is the combination of all related interrupt enable bits.
  *         Each bit corresponds to a interrupt bit.
  *         This parameter decides which interrupts will be enabled. Valid values are:
  *           - \ref SPI_UNITIEN_MASK
  *           - \ref SPI_SSINAIEN_MASK
  *           - \ref SPI_SSACTIEN_MASK
  *           - \ref SPI_SLVURIEN_MASK
  *           - \ref SPI_SLVBEIEN_MASK
  *           - \ref SPI_SLVTOIEN_MASK
  *           - \ref SPI_FIFO_TXTHIEN_MASK
  *           - \ref SPI_FIFO_RXTHIEN_MASK
  *           - \ref SPI_FIFO_RXOVIEN_MASK
  *           - \ref SPI_FIFO_TXUFIEN_MASK
  *           - \ref SPI_FIFO_RXTOIEN_MASK
  * @return none
  */
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask)
{
    if(u32Mask & SPI_UNITIEN_MASK)
        spi->CTL &= ~SPI_CTL_UNITIEN_Msk;

    if(u32Mask & SPI_SSINAIEN_MASK)
        spi->SSCTL &= ~SPI_SSCTL_SSINAIEN_Msk;

    if(u32Mask & SPI_SSACTIEN_MASK)
        spi->SSCTL &= ~SPI_SSCTL_SSACTIEN_Msk;

    if(u32Mask & SPI_SLVURIEN_MASK)
        spi->SSCTL &= ~SPI_SSCTL_SLVUDRIEN_Msk;

    if(u32Mask & SPI_SLVBEIEN_MASK)
        spi->SSCTL &= ~SPI_SSCTL_SLVBCEIEN_Msk;

    if(u32Mask & SPI_SLVTOIEN_MASK)
        spi->SSCTL &= ~SPI_SSCTL_SLVTOIEN_Msk;

    if(u32Mask & SPI_FIFO_TXTHIEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_TXTHIEN_Msk;

    if(u32Mask & SPI_FIFO_RXTHIEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXTHIEN_Msk;

    if(u32Mask & SPI_FIFO_RXOVIEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXOVIEN_Msk;

    if(u32Mask & SPI_FIFO_TXUFIEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_TXUDFIEN_Msk;

    if(u32Mask & SPI_FIFO_RXTOIEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXTOIEN_Msk;
}

/*@}*/ /* end of group I91500_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_SPI_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
