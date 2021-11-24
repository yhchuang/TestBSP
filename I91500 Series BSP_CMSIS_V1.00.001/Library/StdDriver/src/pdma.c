/**************************************************************************//**
 * @file     pdma.c
 * @version  V2.00
 * $Revision: 1 $
 * $Date: 17/10/11 2:52p $
 * @brief    I91500 series PDMA driver source file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "I91500.h"

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  * @{
  */

/** @addtogroup I91500_PDMA_Driver PDMA Driver
  * @{
  */

/** @addtogroup I91500_PDMA_EXPORTED_FUNCTIONS PDMA Exported Functions
  @{
*/

/**
 * @brief       PDMA Open
 *
 * @param[in]   u32Mask     Channel enable bits.
 *
 * @return      None
 *
 * @details     This function enable the PDMA channels.
 */
void PDMA_Open(uint32_t u32Mask)
{	
    PDMA_GCR->GCRCSR |= (u32Mask<<PDMA_GCRCSR_HCLK_EN_Pos);
}

/**
 * @brief       PDMA Close
 *
 * @return      None
 *
 * @details     This function disable all PDMA channels.
 */
void PDMA_Close(void)
{
    PDMA_GCR->GCRCSR = 0;
}

/**
 * @brief       Set PDMA Transfer Count
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Width        Data width. Valid values are
 *                - \ref PDMA_WIDTH_8
 *                - \ref PDMA_WIDTH_16
 *                - \ref PDMA_WIDTH_32 
 * @param[in]   u32TransCount   Transfer count
 *
 * @return      None
 *
 * @details     This function set the selected channel data width and transfer count.
 */
void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount)
{
    PDMA_T* pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
	
    pdma->CSR = (pdma->CSR & ~PDMA_CSR_APB_TWS_Msk) | u32Width;
	
    switch(u32Width)
    {
		case PDMA_WIDTH_8:	pdma->BCR = u32TransCount;			break;
		case PDMA_WIDTH_16:	pdma->BCR = (u32TransCount << 1);	break;
		default:			pdma->BCR = (u32TransCount << 2);   break;
    }
}

/**
 * @brief       Set PDMA Transfer Address
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32SrcAddr      Source address
 * @param[in]   u32SrcCtrl      Source control attribute. Valid values are
 *                - \ref PDMA_SAR_INC
 *                - \ref PDMA_SAR_FIX
 *                - \ref PDMA_SAR_WRA 
 * @param[in]   u32DstAddr      destination address
 * @param[in]   u32DstCtrl      destination control attribute. Valid values are
 *                - \ref PDMA_DAR_INC
 *                - \ref PDMA_DAR_FIX
 *                - \ref PDMA_DAR_WRA 
 *
 * @return      None
 *
 * @details     This function set the selected channel source/destination address and attribute.
 */
void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl)
{
    PDMA_T* pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
	
    pdma->SAR = u32SrcAddr;
    pdma->DAR = u32DstAddr;
    pdma->CSR = (pdma->CSR & ~(PDMA_CSR_SAD_SEL_Msk | PDMA_CSR_DAD_SEL_Msk)) | (u32SrcCtrl | u32DstCtrl);
}

/**
 * @brief       Set PDMA Transfer Mode
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Peripheral   The selected peripheral. Valid values are
 *                - \ref PDMA_SPI0_RX
 *                - \ref PDMA_SPI0_TX
 *                - \ref PDMA_I2S_RX
 *                - \ref PDMA_I2S_TX
 *                - \ref PDMA_UART1_RX 
 *                - \ref PDMA_UART1_TX 
 *                - \ref PDMA_UART0_RX 
 *                - \ref PDMA_UART0_TX 
 *                - \ref PDMA_SDADC 
 *                - \ref PDMA_DAC 
 *                - \ref PDMA_SARADC 
 *                - \ref PDMA_SPI1_RX 
 *                - \ref PDMA_SPI1_TX 
 *                - \ref PDMA_MEM
 *
 * @return      None
 *
 * @details     This function set the selected channel transfer mode. Include peripheral setting.
 */
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Peripheral)
{
    PDMA_T* pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
	
	uint32_t u32TransferMode = (u32Peripheral>>16)&0xf;

	// Set transfer mode.
	pdma->CSR = (pdma->CSR & (~PDMA_CSR_MODE_SEL_Msk) ) | u32TransferMode ;
	
	//
	if( u32TransferMode != PDMA_SRAM_SRAM )
	{
		*(volatile uint32_t *)((uint32_t)&PDMA_GCR->PDSSR0+(((u32Peripheral>>8)&0x1)*4)) &= ~(0xful << (u32Peripheral&0xff));
		*(volatile uint32_t *)((uint32_t)&PDMA_GCR->PDSSR0+(((u32Peripheral>>8)&0x1)*4)) |= (u32Ch << (u32Peripheral&0xff));
	}
}

/**
 * @brief       Set PDMA Transfer Direction
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Direction    Transfer direction
 *                - \ref PDMA_APB_SRAM
 *                - \ref PDMA_SRAM_APB 
 *
 * @return      None
 *
 * @details     This function select the PDMA transfer direction.
 */
void PDMA_SetTransferDirection(uint32_t u32Ch, uint32_t u32Direction)
{
    PDMA_T *pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
	
    pdma->CSR = (pdma->CSR & (~PDMA_CSR_MODE_SEL_Msk) ) | u32Direction ;
}

/**
 * @brief       Trigger PDMA
 *
 * @param[in]   u32Ch           The selected channel
 *
 * @return      None
 *
 * @details     This function trigger the selected channel.
 */
void PDMA_Trigger(uint32_t u32Ch)
{
    PDMA_T* pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->CSR |= (PDMA_CSR_TRIG_EN_Msk | PDMA_CSR_PDMACEN_Msk);
}

/**
 * @brief       Enable Interrupt
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Mask         The Interrupt Type
 *
 * @return      None
 *
 * @details     This function enable the selected channel interrupt.
 */
void PDMA_EnableInt(uint32_t u32Ch, uint32_t u32Mask)
{
    PDMA_T* pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->IER |= u32Mask;
}

/**
 * @brief       Disable Interrupt
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Mask         The Interrupt Type
 *
 * @return      None
 *
 * @details     This function disable the selected channel interrupt.
 */
void PDMA_DisableInt(uint32_t u32Ch, uint32_t u32Mask)
{
    PDMA_T* pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));

    pdma->IER &= ~u32Mask;
}

/**
 * @brief       PDMA Software Engine Reset
 *
 * @param[in]   u32Ch           The selected channel
 *
 * @return      None
 *
 * @details     This function will do PDMA software reset.
 */
void PDMA_SoftwareReset(uint32_t u32Ch)
{
    PDMA_T* pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
	
    pdma->CSR = (pdma->CSR & (~PDMA_CSR_SW_RST_Msk) ) | PDMA_CSR_SW_RST_Msk ;
}

/**
 * @brief       Select PDMA Wrap Interrupt Mode
 *
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Peripheral   The selected peripheral. Valid values are
 *                - \ref PDMA_HALF_WRAP_MODE
 *                - \ref PDMA_FULL_WRAP_MODE
 *                - \ref PDMA_BOTH_WRAP_MODE
 *
 * @return      None
 *
 * @details     This function select the PDMA wrap interrupt mode.
 */
void PDMA_WrapIntSelect(uint32_t u32Ch, uint32_t u32Mode)
{
    PDMA_T* pdma = (PDMA_T *)((uint32_t) PDMA0_BASE + (0x100 * u32Ch));
	
    pdma->CSR = (pdma->CSR & (~PDMA_CSR_WRA_INT_SEL_Msk) ) | (u32Mode << PDMA_CSR_WRA_INT_SEL_Pos) ;
}

/*@}*/ /* end of group I91500_PDMA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_PDMA_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
