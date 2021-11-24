/**************************************************************************//**
 * @file     DAC.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/11/02 07:27p $
 * @brief    I91500 Series DAC Driver Header File
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DAC_H__
#define __DAC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_DAC_Driver DAC Driver
  @{
*/

/** @addtogroup I91500_DAC_EXPORTED_CONSTANTS DAC Exported Constants
  @{
*/ 

/*---------------------------------------------------------------------------------------------------------*/
/* DAC Sample rate & OSR Constant Definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/	
#define DAC_DVOL_MUTE                            (0x00)
#define DAC_DVOL_NEG80DB                         (0x53)
#define DAC_DVOL_0DB                             (0xF3)
#define DAC_DVOL_6DB                             (0xFF)

#define DAC_BV1P5_NEG12DB                        (0x2D)
#define DAC_BV1P5_NEG6DB                         (0x33)
#define DAC_BV1P5_0DB                            (0x39)
#define DAC_BV1P5_6DB                            (0x3F)

#define DAC_CTL0_DATAWIDTH_8BIT                  (0x2)
#define DAC_CTL0_DATAWIDTH_16BIT                 (0x1)
#define DAC_CTL0_DATAWIDTH_24BIT                 (0x3)
#define DAC_CTL0_DATAWIDTH_32BIT                 (0x0)

#define DAC_CTL0_MODESEL_STEREO                  (0x0)
#define DAC_CTL0_MODESEL_MONO                    (0x1)

#define DAC_CTL0_CLKSEL_256K                     (0)
#define DAC_CTL0_CLKSEL_250K                     (DAC_CTL0_CLKSET_Msk)

/*@}*/ /* end of group I91500_DAC_EXPORTED_CONSTANTS */

/** @addtogroup I91500_DAC_EXPORTED_FUNCTIONS DAC Exported Functions
  @{
*/

/**
  * @brief      SET DAC clock source mode.
  * @param[in]  dac The base address of DAC module
  * @param[in]  u32Value is The clock source mode(DAC_CTL0_CLKSEL_256K/DAC_CTL0_CLKSEL_250K)
  * @return     None
  */
#define DAC_SET_CLKSEL(dac,u32Value)             (dac->CTL0 = (dac->CTL0 &~(DAC_CTL0_CLKSET_Msk))|u32Value) 

/**
  * @brief      Clear DAC FIFO data.
  * @param[in]  dac The base address of DAC module
  * @return     None
  */
#define DAC_CLEAR_FIFO(dac)                      (dac->CTL0 |= DAC_CTL0_FCLR_Msk) 

/**
  * @brief      Enable DAC FIFO.
  * @param[in]  dac The base address of DAC module
  * @return     None
  */
#define DAC_ENABLE_FIFO(dac)                      (dac->CTL0 |= DAC_CTL0_FIFOEN_Msk) 


/**
  * @brief      Enable DAC FIFO threshold interrupt.
  * @param[in]  dac The base address of DAC module
  * @return     None
  * @details    DAC FIFO threshold interrupt Enabled.
  */
#define DAC_ENABLE_FIFOTHRESHOLDINT(dac,u8Value) (dac->CTL0 = (dac->CTL0 &~(DAC_CTL0_THIE_Msk|DAC_CTL0_TH_Msk))|((u8Value<<DAC_CTL0_TH_Pos)|DAC_CTL0_THIE_Pos)) 

/**
  * @brief      Disable DAC FIFO threshold interrupt.
  * @param[in]  dac The base address of DAC module
  * @return     None
  * @details    DAC FIFO threshold interrupt Disabled.			
  */
#define DAC_DISABLE_FIFOTHRESHOLDINT(dac)        (dac->CTL0 &= (~DAC_CTL0_THIE_Pos))

/**
  * @brief      Check DAC FIFO full or not
  * @param[in]  dpwm The base address of DAC module
  * @return     0 = FIFO is not full
  *             1 = FIFO is full
  */
#define DAC_IS_FIFOFULL(dac)                     (dac->FIFOSTS&DAC_FIFOSTS_FULL_Msk)

/**
  * @brief      Check DAC FIFO empty or not
  * @param[in]  dpwm The base address of DAC module
  * @return     0 = FIFO is not empty
  *             1 = FIFO is empty
  */
#define DAC_IS_FIFOEMPTY(dac)                    (dac->FIFOSTS&DAC_FIFOSTS_EMPTY_Msk)

/**
  * @brief     Set DAC data value
  * @param[in] dac The base address of DAC module6
  * @param[in] u8Value is data value for dac.
  * @return    None
  */
#define DAC_SET_DATA(dac,u32Value)               (dac->DAT = (u32Value))

/**
  * @brief     Set DAC fifo data width.
  * @param[in] dac The base address of DAC module6
  * @param[in] u8Width is data width for dac.
  * @return    None
  */
#define DAC_SET_DATAWIDTH(dac,u8Width)           (dac->CTL0 = (dac->CTL0&~DAC_CTL0_FIFOWIDTH_Msk)|(u8Width<<DAC_CTL0_FIFOWIDTH_Pos))

/**
  * @brief     Enable DAC PDMA interface.
  * @param[in] dac The base address of DAC module
  * @return    None.
  * @details   DAC will request data from PDMA controller whenever there is space in FIFO.
  */
#define DAC_ENABLE_PDMA(dac)                     (dac->PDMACTL = 1)

/**
  * @brief     Disable DAC PDMA interface.
  * @param[in] dac The base address of DAC module
  * @return    None.
  */
#define DAC_DISABLE_PDMA(dac)                    (dac->PDMACTL = 0)

/**
  * @brief     Enable DAC right channel. 
  * @param[in] dac The base address of DAC module6
  * @return    None
  */
#define DAC_ENABLE_RIGHT_CHANNEL(dac)            (dac->CTL1 |= DAC_CTL1_DACENR_Msk)

/**
  * @brief     Disable DAC right channel. 
  * @param[in] dac The base address of DAC module6
  * @return    None
  */
#define DAC_DISABLE_RIGHT_CHANNEL(dac)           (dac->CTL1 &= ~DAC_CTL1_DACENR_Msk)

/**
  * @brief     Enable DAC left channel. 
  * @param[in] dac The base address of DAC module6
  * @return    None
  */
#define DAC_ENABLE_LEFT_CHANNEL(dac)             (dac->CTL1 |= DAC_CTL1_DACENL_Msk)

/**
  * @brief     Disable DAC left channel. 
  * @param[in] dac The base address of DAC module6
  * @return    None
  */
#define DAC_DISABLE_LEFT_CHANNEL(dac)            (dac->CTL1 &= ~DAC_CTL1_DACENL_Msk)

/**
  * @brief     Set DAC right channel volume. 
  * @param[in] dac The base address of DAC module6
  * @param[in] u8Vol is data vol for dac.
  * @return    None
  */
#define DAC_SET_RIGHT_CHANNEL_VOL(dac,u8Vol)     (dac->DVOL = (dac->DVOL&~DAC_DVOL_DACRVOL_Msk)|(u8Vol<<DAC_DVOL_DACRVOL_Pos))

/**
  * @brief     Set DAC left channel volume. 
  * @param[in] dac The base address of DAC module6
  * @param[in] u8Vol is data vol for dac.
  * @return    None
  */
#define DAC_SET_LEFT_CHANNEL_VOL(dac,u8Vol)      (dac->DVOL = (dac->DVOL&~DAC_DVOL_DACLVOL_Msk)|(u8Vol<<DAC_DVOL_DACLVOL_Pos))

/**
  * @brief     Set DAC headphone driver volume. 
  * @param[in] dac The base address of DAC module
  * @param[in] u8Vol is data vol for headphone.
  * @return    None
  */
#define DAC_SET_HP_VOL(dac,u8Vol)     (dac->ANA0 = (dac->ANA0&~DAC_ANA0_BV1P5_Msk)|(u8Vol<<DAC_ANA0_BV1P5_Pos))


void DAC_EnableAnalog(void);

void DAC_DisableAnalog(void);

void DAC_WriteFIFO(int32_t *pi32Stream, uint32_t u32count);

uint32_t DAC_SetSampleRate(uint32_t u32SampleRate);

uint32_t DAC_GetSampleRate(void);


/*@}*/ /* end of group I91500_DAC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_DAC_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__DAC_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
