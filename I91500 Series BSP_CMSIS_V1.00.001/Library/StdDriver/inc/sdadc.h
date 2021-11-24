/**************************************************************************//**
 * @file     sdadc.h
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 16/08/22 13:27p $
 * @brief    I91500 Series SDADC Driver Header File
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __SDADC_H__
#define __SDADC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_SDADC_Driver SDADC Driver
  @{
*/

/** @addtogroup I91500_SDADC_EXPORTED_CONSTANTS SDADC Exported Constants
  @{
*/  

/*---------------------------------------------------------------------------------------------------------*/
/* SDADC Over-sampling Ratio Of The Decimation Filter Constant Definitions                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define SDADC_DS_RATION_0   0       /*!< SDADC reserved down sampling Ratio            */
#define SDADC_DS_RATION_16  1       /*!< SDADC x16 down sampling Ratio                 */
#define SDADC_DS_RATION_32  2       /*!< SDADC x32 down sampling Ratio                 */
#define SDADC_DS_RATION_64  3       /*!< SDADC x64 down sampling Ratio                 */

/*---------------------------------------------------------------------------------------------------------*/
/* SDADC And ALC Interrupt Flag Constant Definitions                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define SDADC_FIFO_INT        (BIT0)             /*!< SDADC FIFO level interrupt enable flag   */
#define SDADC_CMP0_INT        (BIT1)             /*!< SDADC compared 0 interrupt enable flag   */
#define SDADC_CMP1_INT        (BIT2)             /*!< SDADC compared 1 interrupt enable flag   */
#define SDADC_ALC_PLMT_INT    (BIT8)             /*!< ALC Peak limiting Interrupt              */
#define SDADC_ALC_NG_INT  	  (BIT9)             /*!< ALC Noise Gating Interrupt               */
#define SDADC_ALC_GINC_INT    (BIT10)            /*!< GAIN Increase Interrupt                  */
#define SDADC_ALC_GDEC_INT    (BIT11)            /*!< GAIN Decrease Interrupt                  */
#define SDADC_ALC_GMAX_INT    (BIT12)            /*!< GAIN More Than Maximum GAIN Interrupt    */
#define SDADC_ALC_GMIN_INT    (BIT13)            /*!< GAIN Less Than Minimum GAIN Interrupt    */

/*---------------------------------------------------------------------------------------------------------*/
/* FIFO Data Bits Selections                                                							   */
/*---------------------------------------------------------------------------------------------------------*/
#define SDADC_FIFODATA_32BITS        (0x00ul << SDADC_CTL_FIFOBITS_Pos)             /*!< SDADC FIFO data 32 bits   */
#define SDADC_FIFODATA_16BITS        (0x01ul << SDADC_CTL_FIFOBITS_Pos)             /*!< SDADC FIFO data 16 bits   */
#define SDADC_FIFODATA_8BITS         (0x02ul << SDADC_CTL_FIFOBITS_Pos)             /*!< SDADC FIFO data 8 bits   */
#define SDADC_FIFODATA_24BITS        (0x03ul << SDADC_CTL_FIFOBITS_Pos)             /*!< SDADC FIFO data 24 bits   */

/*---------------------------------------------------------------------------------------------------------*/
/* SDADC ADCMPR Constant Definitions                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define SDADC_CMP_LESS_THAN          (0x00ul << SDADC_CMPR0_CMPCOND_Pos)           /*!< The compare condition is "less than"          */
#define SDADC_CMP_GREATER_OR_EQUAL   (0x01ul << SDADC_CMPR0_CMPCOND_Pos)           /*!< The compare condition is "greater than or equal to" */

/*---------------------------------------------------------------------------------------------------------*/
/* PGA Gain constant definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/ 
#define SDADC_PGACTL_GAIN_minus_12DB    0x0         /*!< The Gain is -12dB   */
#define SDADC_PGACTL_GAIN_minus_6DB     0x4         /*!< The Gain is -6dB   */
#define SDADC_PGACTL_GAIN_0DB           0x8         /*!< The Gain is 0dB   */
#define SDADC_PGACTL_GAIN_plus_6DB      0xC         /*!< The Gain is 6dB   */
#define SDADC_PGACTL_GAIN_plus_12DB     0x10        /*!< The Gain is 12dB   */
#define SDADC_PGACTL_GAIN_plus_18DB     0x14        /*!< The Gain is 18dB   */
#define SDADC_PGACTL_GAIN_plus_24DB     0x18        /*!< The Gain is 24dB   */
#define SDADC_PGACTL_GAIN_plus_30DB     0x1C        /*!< The Gain is 30dB   */
#define SDADC_PGACTL_GAIN_plus_34_5DB   0x1F        /*!< The Gain is 34.5dB   */

#define SDADC_BST_GAIN_0DB    (0x0ul << SDADC_ANA1_BSTMODE_Pos)         /*!< The Gain is 0dB  */
#define SDADC_BST_GAIN_26DB   (0x1ul << SDADC_ANA1_BSTMODE_Pos)         /*!< The Gain is 26dB  */

#define SDADC_PGACTL_REFSEL_VMID       (0x0ul << ANA_PGACTL_REFSEL_Pos)           /*!< Select VMID(VCCA/2) voltage as analog ground reference.   */
#define SDADC_PGACTL_REFSEL_VBG        (0x1ul << ANA_PGACTL_REFSEL_Pos)           /*!<  Select Bandgap(1.2V) voltage as analog ground reference.   */

#define SDADC_PGACTL_REFSEL_VMID       (0x0ul << ANA_PGACTL_REFSEL_Pos)           /*!< Select VMID(VCCA/2) voltage as analog ground reference.   */
#define SDADC_PGACTL_REFSEL_VBG        (0x1ul << ANA_PGACTL_REFSEL_Pos)           /*!<  Select Bandgap(1.2V) voltage as analog ground reference.   */

/*---------------------------------------------------------------------------------------------------------*/
/* ALC CTL constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define SDADC_ALCCTL_NORMAL_MODE     (0x0ul << ALC_CTL_MODESEL_Pos)  /*!<  ALC operates on normal mode.   */
#define SDADC_ALCCTL_LIMITER_MODE    (0x1ul << ALC_CTL_MODESEL_Pos)  /*!<  ALC operates on limiter mode.   */

#define SDADC_ALCCTL_ABS_PEAK       (0x0ul << ALC_CTL_PKSEL_Pos)   /*!<  use absolute peak value for ALC training.    */
#define SDADC_ALCCTL_P2P_PEAK       (0x1ul << ALC_CTL_PKSEL_Pos)   /*!<  use peak-to-peak value for ALC training.   */

#define SDADC_ALCCTL_FASTDEC_ON     (0x0ul << ALC_CTL_PKLIMEN_Pos)  /*!< enable fast decrement when signal exceeds 87.5% of full scale.    */
#define SDADC_ALCCTL_FASTDEC_OFF    (0x1ul << ALC_CTL_PKLIMEN_Pos)  /*!< disable fast decrement when signal exceeds 87.5% of full scale.    */

#define SDADC_ALCCTL_NGPEAK_ABS     (0x1ul << ALC_CTL_NGPKSEL_Pos)   /*!<  use absolute peak value for for noise gate threshold determination.    */
#define SDADC_ALCCTL_NGPEAK_P2P     (0x0ul << ALC_CTL_NGPKSEL_Pos)   /*!<  use peak-to-peak value for for noise gate threshold determination.   */

#define SDADC_CTL_CLKSEL_256K       (0)
#define SDADC_CTL_CLKSEL_250K       (SDADC_CTL_SPDS_Msk)

/*@}*/ /* end of group I91500_SDADC_EXPORTED_CONSTANTS */


/** @addtogroup I91500_SDADC_EXPORTED_FUNCTIONS SDADC Exported Functions
  @{
*/

/**
  * @brief      SET SDADC clock source mode.
  * @param[in]  sdadc The base address of DAC module
  * @param[in]  u32Value is The clock source mode(SDADC_CTL_CLKSEL_256K/SDADC_CTL_CLKSEL_250K)
  * @return     None
  */
#define SDADC_SET_CLKSEL(sdadc,u32Value)           (sdadc->CTL = (sdadc->CTL &~(SDADC_CTL_SPDS_Msk))|u32Value) 

/**
  * @brief     Set SDCLK divisor.
  * @param     sdadc Base address of SDADC module.
  * @param     u8Divisor SDCLK divisore whcih must be greater than 2.
  * @return    None
  * @details   The clock division ratio is between the incoming SDADC_CLK (default HCLK)
  *            and the Delta-Sigma sampling clock of the SDADC.
  */
#define SDADC_SET_SDCLKDIV(sdadc, u8Divisor)   (sdadc->CLKDIV = (u8Divisor&0xf))
                         
/**
  * @brief     Set DSRATE for SDADC down sampling ratio.
  * @param     sdadc Base address of SDADC module.
  * @param     u8Ratio down sampling ratio.
  *            - \ref SDADC_DS_RATION_0 
  *            - \ref SDADC_DS_RATION_16
  *            - \ref SDADC_DS_RATION_32
  *            - \ref SDADC_DS_RATION_64
  * @return    None
  * @details   This macro only determines the down sampling ratio of SDADC. If BIQ filter is enabled,
  *			   real DSR = SDADC_CTL.DSRATE * BIQ_CTL.ADCWNSR
  */
#define SDADC_SET_DSRATIO(sdadc, u8Ratio)   (sdadc->CTL = (sdadc->CTL&~SDADC_CTL_DSRATE_Msk)|u8Ratio) 												 
 
/**
  * @brief     read data from the audio FIFO.
  * @param     sdadc Base address of SDADC module.
  * @return    Signed 16 bits audio data.
  * @details   A read of this register will read data from the audio FIFO
  *            and increment the read pointer.
  */
#define SDADC_GET_FIFODATA(sdadc)   (sdadc->DAT&SDADC_DAT_RESULT_Msk)

/**
  * @brief     FIFO data bits selection.
  * @param     sdadc Base address of SDADC module.
  * @param     u8Bits data bits.
  *            - \ref SDADC_FIFODATA_32BITS 
  *            - \ref SDADC_FIFODATA_16BITS
  *            - \ref SDADC_FIFODATA_8BITS
  *            - \ref SDADC_FIFODATA_24BITS
  * @return    None.
  */
#define SDADC_SET_FIFODATABITS(sdadc, \
							  u8Bits)   (sdadc->CTL = (sdadc->CTL&(~SDADC_CTL_FIFOBITS_Msk))|u8Bits)  

/**
  * @brief     Set FIFO interrupt level.
  * @param     sdadc Base address of SDADC module.
  * @param     u8Level is number of words present in SDADC FIFO, total 8 word levels.
  * @return    None.
  * @details   Determines at what level the SDADC FIFO will generate a servicing
  *            interrupt to the CPU. Interrupt will be generated when number of
  *            words present in SDADC FIFO is greater than u8Level.
  */
#define SDADC_SET_FIFOINTLEVEL(sdadc, \
                             u8Level)    (sdadc->CTL = (sdadc->CTL&(~SDADC_CTL_FIFOTH_Msk))|((u8Level&0xf)<<SDADC_CTL_FIFOTH_Pos))

/**
  * @brief      Check SDADC FIFO full or not
  * @param[in]  sdadc The base address of SDADC module
  * @return     0 = FIFO is not full
  *             1 = FIFO is full
  */
#define SDADC_IS_FIFOFULL(sdadc)    (sdadc->FIFOSTS&SDADC_FIFOSTS_FULL_Msk)

/**
  * @brief      Check SDADC FIFO empty or not
  * @param[in]  sdadc The base address of SDADC module
  * @return     0 = FIFO is not empty
  *             1 = FIFO is empty
  */
#define SDADC_IS_FIFOEMPTY(sdadc)    (sdadc->FIFOSTS&SDADC_FIFOSTS_EMPTY_Msk)

/**
  * @brief     Enable SDADC PDMA receive channel.
  * @param     sdadc Base address of SDADC module.
  * @return    None.
  * @details   SDADC will request PDMA service when data is available. When PDMA transfer is enabled,
  *            the SDADC interrupt must be disabled.
  */
#define SDADC_ENABLE_PDMA(sdadc)    sdadc->PDMACTL |= SDADC_PDMACTL_PDMAEN_Msk; \
									sdadc->CTL &= (~SDADC_CTL_FIFOTHIE_Msk)
                                  
/**
  * @brief     Disable SDADC PDMA receive channel.
  * @param     sdadc Base address of SDADC module.
  * @return    None.
  */
#define SDADC_DISABLE_PDMA(sdadc)     (sdadc->PDMACTL &= (~SDADC_PDMACTL_PDMAEN_Msk))

/**
  * @brief     Configure the comparator 0 and enable it.
  * @param     sdadc Base address of SDADC module.
  * @param     u32Condition Specifies the compare condition. Valid values are:
  *            - \ref SDADC_CMP_LESS_THAN            :The compare condition is "less than the compare value".
  *            - \ref SDADC_CMP_GREATER_OR_EQUAL     :The compare condition is "greater than or equal to the compare value.
  * @param     u32Data Specifies 23 bits value to compare FIFO data, the valid value are between 0 ~ 0x007fffff.
  * @param     u32MatchCount Specifies the match count setting, valid values are between 1~16.
  * @return    None
  * @details   For example, SDADC_ENABLE_CMP0(SDADC, SDADC_CMP_GREATER_OR_EQUAL, 0x800, 10);
  *            Means SDADC will assert comparator 0 flag if conversion result is greater or
  *            equal to 0x800 for 10 times continuously.
  */
#define SDADC_ENABLE_CMP0(sdadc, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (sdadc->CMPR[0] = u32Condition | \
														 ((u32Data&0x007fffff) << SDADC_CMPR_CMPD_Pos) | \
														 (((u32MatchCount) - 1) << SDADC_CMPR_CMPMATCNT_Pos) | \
														 SDADC_CMPR_CMPOEN_Msk)

/**
  * @brief     Disable comparator 0.
  * @param     sdadc Base address of SDADC module.
  * @return    None
  * @details   Set CMPEN (CMP0[0]) to 0 to disable SDADC controller to compare CMPD[30:8].
  */
#define SDADC_DISABLE_CMP0(sdadc) (sdadc->CMPR[0] = 0)

/**
  * @brief     Configure the comparator 1 and enable it.
  * @param     sdadc Base address of SDADC module.
  * @param     u32Condition Specifies the compare condition. Valid values are:
  *            - \ref SDADC_CMP_LESS_THAN            :The compare condition is "less than the compare value".
  *            - \ref SDADC_CMP_GREATER_OR_EQUAL     :The compare condition is "greater than or equal to the compare value.
  * @param     u32Data Specifies 23 bits value to compare FIFO data, the valid value are between 0 ~ 0x007fffff.
  * @param     u32MatchCount Specifies the match count setting, valid values are between 1~16.
  * @return    None
  * @details   For example, SDADC_ENABLE_CMP1(SDADC, SDADC_CMP_GREATER_OR_EQUAL, 0x800, 10);
  *            Means SDADC will assert comparator 1 flag if conversion result is greater or
  *            equal to 0x800 for 10 times continuously.
  */
#define SDADC_ENABLE_CMP1(sdadc, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (sdadc->CMPR[1] = u32Condition | \
														 ((u32Data&0x007fffff) << SDADC_CMPR_CMPD_Pos) | \
														 (((u32MatchCount) - 1) << SDADC_CMPR_CMPMATCNT_Pos) | \
														 SDADC_CMPR_CMPOEN_Msk)

/**
  * @brief     Disable comparator 1.
  * @param     sdadc Base address of SDADC module.
  * @return    None
  * @details   Set CMPEN (CMP1[0]) to 0 to disable SDADC controller to compare CMPD[30:8].
  */
#define SDADC_DISABLE_CMP1(sdadc) (sdadc->CMPR[1] = 0)

/**
  * @brief     Start the A/D conversion.
  * @param     sdadc Base address of SDADC module.
  * @return    None
  * @details   SDADCEN (SDADCEN[0]) can be set to 1 for SDADC conversion enabling. 
  */
#define SDADC_START_CONV(sdadc) (sdadc->EN = SDADC_EN_SDADCEN_Msk)

/**
  * @brief     Stop the A/D conversion.
  * @param     sdadc Base address of SDADC module.
  * @return    None
  * @details   SDADCEN (SDADCEN[0]) can be set to 0 for SDADC conversion disabling
  */
#define SDADC_STOP_CONV(sdadc) (sdadc->EN = ~SDADC_EN_SDADCEN_Msk)

/**
  * @brief      Set programmable gain amplifier (PGA) 
  * @param      sdadc The base address of SDADC module
  * @param      u32Gain is boost stage gain setting
  *             - \ref 0x0~0x1F
  * @return     None
  * @details    I91500 provides a Programmable Gain Amplifier (PGA) as the front-end
  *             to the SDADC to allow the adjustment of signal path gain. 
  */
#define SDADC_SET_PGA(sdadc, u32Gain)  (sdadc->ANA2 = (sdadc->ANA2& ~SDADC_ANA2_GAINSET_Msk)|(u32Gain))

/**
  * @brief      Enable programmable gain amplifier (PGA) 
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_ENABLE_PGA(sdadc)  (sdadc->ANA0 |= SDADC_ANA0_PGA_PU_Msk)

/**
  * @brief      Disable programmable gain amplifier
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_DISABLE_PGA(sdadc)   (sdadc->ANA0 &= ~SDADC_ANA0_PGA_PU_Msk)

/**
  * @brief      PGA signal mute-on 
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_MUTEON_PGA(sdadc)    (sdadc->ANA0 |= SDADC_ANA0_PGA_MUTE_Msk)

/**
  * @brief      PGA signal mute-off 
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_MUTEOFF_PGA(sdadc)   (sdadc->ANA0 &= ~SDADC_ANA0_PGA_MUTE_Msk) 

/**
  * @brief      Set gain boost (BST) 
  * @param      sdadc The base address of SDADC module
  * @param      u32Gain is boost stage gain setting
  *             - \ref SDADC_BST_GAIN_0DB
											 SDADC_BST_GAIN_26DB
  * @return     None
  * @details    I91500 provides a gain boost (BST) as the front-end
  *             to the SDADC to allow the adjustment of signal path gain. 
  */
#define SDADC_SET_BST(sdadc, u32Gain)  (sdadc->ANA1 = (sdadc->ANA1& ~BIT9)|(u32Gain))

/**
  * @brief      Enable gain boost (BST) 
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_ENABLE_BST(sdadc)  (sdadc->ANA1 |= SDADC_ANA1_BSTPUP_Msk)

/**
  * @brief      Disable gain boost (BST)
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_DISABLE_BST(sdadc)   (sdadc->ANA1 &= ~SDADC_ANA1_BSTPUP_Msk)

/**
  * @brief      BST signal mute-on 
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_MUTEON_BST(sdadc)    (sdadc->ANA1 |= SDADC_ANA1_BSTMUTE_Msk)

/**
  * @brief      BST signal mute-off 
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_MUTEOFF_BST(sdadc)   (sdadc->ANA1 &= ~SDADC_ANA1_BSTMUTE_Msk) 

/**
  * @brief      Enable SDADC analog block power
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_ANALOG_POWERON(sdadc)    (sdadc->ANA0 &= ~SDADC_ANA0_PD_Msk)

/**
  * @brief      Disable SDADC analog block power
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_ANALOG_POWEROFF(sdadc)    (sdadc->ANA0 |= SDADC_ANA0_PD_Msk)

/**
  * @brief      Enable SDADC analog block power
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_ENABLE_CMLCK(sdadc)    (sdadc->ANA0 &= ~SDADC_ANA0_PD_Msk)

/**
  * @brief      Disable SDADC analog block power
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_DISABLE_CMLCK(sdadc)    (sdadc->ANA0 |= SDADC_ANA0_PD_Msk)


int32_t SDADC_SetAudioSampleRate(uint32_t u32SampleRate);
uint32_t SDADC_GetAudioSampleRate(void);
void SDADC_EnableInt(uint32_t u32Mask);
void SDADC_DisableInt(uint32_t u32Mask);
uint32_t SDADC_GetIntFlag(uint32_t u32Mask);
void SDADC_ClearIntFlag(uint32_t u32Mask);


/*@}*/ /* end of group I91500_SDADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_SDADC_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SDADC_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
