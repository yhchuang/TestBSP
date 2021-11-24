/**************************************************************************//**
 * @file     PDMA.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/10/27 2:52p $
 * @brief    I91500 Series PDMA Controller Driver Header File
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __PDMA_H__
#define __PDMA_H__

#ifdef __cplusplus
extern "C"
{
#endif
/** @addtogroup I91500_Device_Driver I91500 Device Driver
  * @{
  */

/** @addtogroup I91500_PDMA_Driver PDMA Driver
  * @{
  */

/** @addtogroup I91500_PDMA_EXPORTED_CONSTANTS PDMA Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Data Width Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_WIDTH_8        0x00080000UL            /*!<DMA Transfer Width 8-bit */
#define PDMA_WIDTH_16       0x00100000UL            /*!<DMA Transfer Width 16-bit */
#define PDMA_WIDTH_32       0x00000000UL            /*!<DMA Transfer Width 32-bit */

/*---------------------------------------------------------------------------------------------------------*/
/*  Address Attribute Constant Definitions                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_SAR_INC        0x00000000UL            /*!<DMA SAR increment */
#define PDMA_SAR_FIX        0x00000020UL            /*!<DMA SAR fix address */
#define PDMA_SAR_WRA        0x00000030UL            /*!<DMA SAR wrap around */
#define PDMA_DAR_INC        0x00000000UL            /*!<DMA DAR increment */
#define PDMA_DAR_FIX        0x00000080UL            /*!<DMA DAR fix address */
#define PDMA_DAR_WRA        0x000000C0UL            /*!<DMA DAR wrap around */

/*---------------------------------------------------------------------------------------------------------*/
/*  PDMA Transfer Direction Constant Definitions                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_SRAM_SRAM      0x00000000UL            /*!<DMA Transfer from SRAM to SRAM */
#define PDMA_APB_SRAM       0x00000004UL            /*!<DMA Transfer from APB to SRAM */
#define PDMA_SRAM_APB       0x00000008UL            /*!<DMA Transfer from SRAM to APB */

/*---------------------------------------------------------------------------------------------------------*/
/*  Peripheral Transfer Mode Constant Definitions                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_PDSSR0         (0x0<<8)
#define PDMA_PDSSR1         (0x1<<8)

#define PDMA_SPI0_RX        ((PDMA_APB_SRAM<<16)|PDMA_PDSSR0|PDMA_PDSSR0_SPI0RXSEL_Pos)     /*!<DMA Connect to SPI0 RX */
#define PDMA_SPI0_TX        ((PDMA_SRAM_APB<<16)|PDMA_PDSSR0|PDMA_PDSSR0_SPI0TXSEL_Pos)     /*!<DMA Connect to SPI0 TX */
#define PDMA_I2S_RX         ((PDMA_APB_SRAM<<16)|PDMA_PDSSR0|PDMA_PDSSR0_I2SRXSEL_Pos)      /*!<DMA Connect to I2S RX */
#define PDMA_I2S_TX         ((PDMA_SRAM_APB<<16)|PDMA_PDSSR0|PDMA_PDSSR0_I2STXSEL_Pos)      /*!<DMA Connect to I2S TX */
#define PDMA_UART1_RX       ((PDMA_APB_SRAM<<16)|PDMA_PDSSR0|PDMA_PDSSR0_UART1RXSEL_Pos)    /*!<DMA Connect to UART1 RX */
#define PDMA_UART1_TX       ((PDMA_SRAM_APB<<16)|PDMA_PDSSR0|PDMA_PDSSR0_UART1TXSEL_Pos)    /*!<DMA Connect to UART1 TX */
#define PDMA_UART0_RX       ((PDMA_APB_SRAM<<16)|PDMA_PDSSR0|PDMA_PDSSR0_UART0RXSEL_Pos)    /*!<DMA Connect to UART0 RX */
#define PDMA_UART0_TX       ((PDMA_SRAM_APB<<16)|PDMA_PDSSR0|PDMA_PDSSR0_UART0TXSEL_Pos)    /*!<DMA Connect to UART0 TX */
#define PDMA_SDADC          ((PDMA_APB_SRAM<<16)|PDMA_PDSSR1|PDMA_PDSSR1_SDADCSEL_Pos)      /*!<DMA Connect to SDADC */
#define PDMA_DAC            ((PDMA_SRAM_APB<<16)|PDMA_PDSSR1|PDMA_PDSSR1_DACTXSEL_Pos)      /*!<DMA Connect to DAC */
#define PDMA_SARADC         ((PDMA_APB_SRAM<<16)|PDMA_PDSSR1|PDMA_PDSSR1_SARADCSEL_Pos)     /*!<DMA Connect to SARADC */
#define PDMA_SPI1_RX        ((PDMA_APB_SRAM<<16)|PDMA_PDSSR1|PDMA_PDSSR1_SPI1RXSEL_Pos)     /*!<DMA Connect to SPI1 RX */
#define PDMA_SPI1_TX        ((PDMA_SRAM_APB<<16)|PDMA_PDSSR1|PDMA_PDSSR1_SPI1TXSEL_Pos)     /*!<DMA Connect to SPI1 TX */
#define PDMA_MEM            (PDMA_SRAM_SRAM<<16)                                            /*!<DMA Connect to Memory */

/*---------------------------------------------------------------------------------------------------------*/
/*  Peripheral Transfer Mode Constant Definitions                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_HALF_WRAP_MODE        0x00000004UL      /*!<DMA Wrap Interrupt Half Mode */
#define PDMA_FULL_WRAP_MODE        0x00000001UL      /*!<DMA Wrap Interrupt Full Mode */
#define PDMA_BOTH_WRAP_MODE        0x00000005UL      /*!<DMA Wrap Interrupt Both Mode */

/*---------------------------------------------------------------------------------------------------------*/
/*  PDMA Channel Interrupt Status Constant Definitions                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_HALF_WRAP_FLAG			(0x4UL << PDMA_ISR_WAR_IF_Pos)			/*!<DMA Wrap Interrupt Half Complete */
#define PDMA_FULL_WRAP_FLAG        	(0x1UL << PDMA_ISR_WAR_IF_Pos)			/*!<DMA Wrap Interrupt Full Complete */
#define PDMA_TRANSFER_DONE_FLAG		(PDMA_ISR_BLKD_IF_Msk)							/*!<DMA Wrap Interrupt Transfer Done */
#define PDMA_ABORT_FLAG				(PDMA_ISR_TABORT_IF_Msk)						/*!<DMA Wrap Interrupt Transfer Done */

/*@}*/ /* end of group I91500_PDMA_EXPORTED_CONSTANTS */

/** @addtogroup I91500_PDMA_EXPORTED_FUNCTIONS PDMA Exported Functions
  @{
*/

/**
 * @brief       Get PDMA Global Interrupt Status
 *
 * @return      Interrupt Status
 *
 * @details     This macro gets the global interrupt status.
 */
#define PDMA_GET_INT_STATUS()   ((uint32_t)(PDMA_GCR->GCRISR))

/**
 * @brief       Get PDMA Channel Interrupt Status
 *
 * @param[in]   u32Ch   Selected DMA channel
 *
 * @return      Interrupt Status
 *
 * @details     This macro gets the channel interrupt status.
 */
#define PDMA_GET_CH_INT_STS(u32Ch)   (*((__IO uint32_t *)((uint32_t)&PDMA0->ISR + (uint32_t)((u32Ch)*0x100))))

/**
 * @brief       Clear PDMA Channel Interrupt Flag
 *
 * @param[in]   u32Ch   Selected DMA channel
 * @param[in]   u32Mask Interrupt Mask
 *
 * @return      None
 *
 * @details     This macro clear the channel interrupt flag.
 */
#define PDMA_CLR_CH_INT_FLAG(u32Ch, u32Mask)   (*((__IO uint32_t *)((uint32_t)&PDMA0->ISR + (uint32_t)((u32Ch)*0x100))) = (u32Mask))

/**
 * @brief       Check Channel Status
 *
 * @param[in]   u32Ch    The selected channel
 *
 * @retval      0        The selected channel is idle
 * @retval      1        The selected channel is busy
 *
 * @details     Check the selected channel is busy or not.
 */
#define PDMA_IS_CH_BUSY(u32Ch)    ((*((__IO uint32_t *)((uint32_t)&PDMA0->CSR +(uint32_t)((u32Ch)*0x100)))&PDMA_CSR_TRIG_EN_Msk)? 1 : 0)

/**
 * @brief       Set Source Address
 *
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Addr   The selected address
 *
 * @return      None
 *
 * @details     This macro set the selected channel source address.
 */
#define PDMA_SET_SRC_ADDR(u32Ch, u32Addr) (*((__IO uint32_t *)((uint32_t)&PDMA0->SAR + (uint32_t)((u32Ch)*0x100))) = (u32Addr))

/**
 * @brief       Set Destination Address
 *
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Addr   The selected address
 *
 * @return      None
 *
 * @details     This macro set the selected channel destination address.
 */
#define PDMA_SET_DST_ADDR(u32Ch, u32Addr) (*((__IO uint32_t *)((uint32_t)&PDMA0->DAR + (uint32_t)((u32Ch)*0x100))) = (u32Addr))

/**
 * @brief       Set Transfer Count
 *
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Count  Transfer Count
 *
 * @return      None
 *
 * @details     This macro set the selected channel transfer count.
 */
#define PDMA_SET_TRANS_CNT(u32Ch, u32Count) {   \
    if (((uint32_t)*((__IO uint32_t *)((uint32_t)&PDMA0->CSR + (uint32_t)((u32Ch)*0x100))) & PDMA_CSR_APB_TWS_Msk) == PDMA_WIDTH_32)  \
        *((__IO uint32_t *)((uint32_t)&PDMA0->BCR + (uint32_t)((u32Ch)*0x100))) = ((u32Count) << 2);  \
    else if (((uint32_t)*((__IO uint32_t *)((uint32_t)&PDMA0->CSR + (uint32_t)((u32Ch)*0x100))) & PDMA_CSR_APB_TWS_Msk) == PDMA_WIDTH_8)  \
        *((__IO uint32_t *)((uint32_t)&PDMA0->BCR + (uint32_t)((u32Ch)*0x100))) = (u32Count); \
    else if (((uint32_t)*((__IO uint32_t *)((uint32_t)&PDMA0->CSR + (uint32_t)((u32Ch)*0x100))) & PDMA_CSR_APB_TWS_Msk) == PDMA_WIDTH_16) \
        *((__IO uint32_t *)((uint32_t)&PDMA0->BCR + (uint32_t)((u32Ch)*0x100))) = ((u32Count) << 1);  \
}

/**
 * @brief       Stop the channel
 *
 * @param[in]   u32Ch     The selected channel
 *
 * @return      None
 *
 * @details     This macro stop the selected channel.
 */
#define PDMA_STOP(u32Ch) (*((__IO uint32_t *)((uint32_t)&PDMA0->CSR + (uint32_t)((u32Ch)*0x100))) &= ~PDMA_CSR_PDMACEN_Msk)

void PDMA_Open(uint32_t u32Mask);
void PDMA_Close(void);
void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferDirection(uint32_t u32Ch, uint32_t u32Direction);
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Periphral);
void PDMA_Trigger(uint32_t u32Ch);
void PDMA_EnableInt(uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(uint32_t u32Ch, uint32_t u32Mask);
void PDMA_SoftwareReset(uint32_t u32Ch);
void PDMA_WrapIntSelect(uint32_t u32Ch, uint32_t u32Mode);

/*@}*/ /* end of group I91500_PDMA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_PDMA_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif  // __PDMA_H__ 

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
