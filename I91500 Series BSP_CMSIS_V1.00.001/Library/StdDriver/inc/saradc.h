/**************************************************************************//**
 * @file     saradc.h
 * @version  V1.00
 * $Revision: 1$
 * $Date: 16/08/23 2:35p $
 * @brief    I91500 series SARADC driver header file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __SARADC_H__
#define __SARADC_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_SARADC_Driver SARADC Driver
  @{
*/

/** @addtogroup I91500_SARADC_EXPORTED_CONSTANTS SARADC Exported Constants
  @{
*/

// Define single end channels
#define SARADC_SEL_CH0				(0)							/*!< SARADC external Channel 0   \hideinitializer */		
#define SARADC_SEL_CH1				(1)							/*!< SARADC external Channel 1 	 \hideinitializer */			
#define SARADC_SEL_CH2				(2)							/*!< SARADC external Channel 2   \hideinitializer */
#define SARADC_SEL_CH3				(3)							/*!< SARADC external Channel 3   \hideinitializer */
#define SARADC_SEL_CH4				(4)							/*!< SARADC external Channel 4   \hideinitializer */
#define SARADC_SEL_CH5				(5)							/*!< SARADC external Channel 5   \hideinitializer */
#define SARADC_SEL_CH6				(6)							/*!< SARADC external Channel 6 	 \hideinitializer */
#define SARADC_SEL_CH7				(7)							/*!< SARADC external Channel 7	 \hideinitializer */
#define SARADC_SEL_CH8				(8)							/*!< SARADC external Channel 8   \hideinitializer */
#define SARADC_SEL_CH9				(9)							/*!< SARADC external Channel 9   \hideinitializer */
#define SARADC_SEL_CH10				(10)						/*!< SARADC external Channel 10  \hideinitializer */
#define SARADC_SEL_CH11				(11)						/*!< SARADC external Channel 11	 \hideinitializer */

// Define Sequence
#define SARADC_SEL_SEQ0				(0)							/*!< SARADC Sequence 0   \hideinitializer */		
#define SARADC_SEL_SEQ1				(1)							/*!< SARADC Sequence 1 	 \hideinitializer */			
#define SARADC_SEL_SEQ2				(2)							/*!< SARADC Sequence 2   \hideinitializer */
#define SARADC_SEL_SEQ3				(3)							/*!< SARADC Sequence 3   \hideinitializer */
#define SARADC_SEL_SEQ4				(4)							/*!< SARADC Sequence 4   \hideinitializer */
#define SARADC_SEL_SEQ5				(5)							/*!< SARADC Sequence 5   \hideinitializer */
#define SARADC_SEL_SEQ6				(6)							/*!< SARADC Sequence 6 	 \hideinitializer */
#define SARADC_SEL_SEQ7				(7)							/*!< SARADC Sequence 7	 \hideinitializer */
#define SARADC_SEL_SEQ8				(8)							/*!< SARADC Sequence 8   \hideinitializer */
#define SARADC_SEL_SEQ9				(9)							/*!< SARADC Sequence 9   \hideinitializer */
#define SARADC_SEL_SEQ10    		(10)						/*!< SARADC Sequence 10  \hideinitializer */
#define SARADC_SEL_SEQ11			(11)						/*!< SARADC Sequence 11	 \hideinitializer */
#define SARADC_SEL_SEQ12    		(12)						/*!< SARADC Sequence 13  \hideinitializer */
#define SARADC_SEL_SEQ13			(13)						/*!< SARADC Sequence 13	 \hideinitializer */

#define SARADC_OPERATION_MODE_SINGLE        (0x0UL << SARADC_CTL_OPMODE_Pos)      /*!< SARADC operation mode set to single conversion  \hideinitializer */
#define SARADC_OPERATION_MODE_SINGLE_CYCLE  (0x2UL << SARADC_CTL_OPMODE_Pos)      /*!< SARADC operation mode set to single cycle scan  \hideinitializer */
#define SARADC_OPERATION_MODE_CONTINUOUS    (0x3UL << SARADC_CTL_OPMODE_Pos)      /*!< SARADC operation mode set to continuous scan  \hideinitializer */

#define SARADC_LOW_LEVEL_TRIGGER		(0x0UL << SARADC_CTL_HWTRGCOND_Pos)    /*!< External Trigger Condition: Low level  \hideinitializer */
#define SARADC_HIGH_LEVEL_TRIGGER		(0x1UL << SARADC_CTL_HWTRGCOND_Pos)    /*!< External Trigger Condition: High level  \hideinitializer */
#define SARADC_FALLING_EDGE_TRIGGER		(0x2UL << SARADC_CTL_HWTRGCOND_Pos)    /*!< External Trigger Condition: Falling edge  \hideinitializer */
#define SARADC_RISING_EDGE_TRIGGER		(0x3UL << SARADC_CTL_HWTRGCOND_Pos)    /*!< External Trigger Condition: Rising edge  \hideinitializer */

#define SARADC_OUT_FORMAT_UNSIGNED   	(0x0UL << SARADC_CTL_ADCFM_Pos)      /*!< A/D Converion Result Format: unsigned  \hideinitializer */
#define SARADC_OUT_FORMAT_2COMPLEMENT 	(SARADC_CTL_ADCFM_Msk)               /*!< A/D Converion Result Format: 2's complement  \hideinitializer */

#define SARADC_DIFFERENTIAL_INPUT   	(0x0UL << SARADC_ACTL_SAR_SE_MODE_Pos)    /*!< SARADC input in differential mode  \hideinitializer */

#define SARADC_CMP_LESS_THAN                (0x0UL)  		 /*!< SARADC compare condition less than  \hideinitializer */
#define SARADC_CMP_GREATER_OR_EQUAL_TO      (0x1UL)          /*!< SARADC compare condition greater or equal to  \hideinitializer */

#define SARADC_ADF_INT                  (SARADC_STATUS0_ADIF_Msk)           /*!< SARADC convert complete interrupt \hideinitializer */
#define SARADC_CMP0_INT                 (SARADC_STATUS0_ADCMPF0_Msk)        /*!< SARADC comparator 0 interrupt  \hideinitializer */
#define SARADC_CMP1_INT                 (SARADC_STATUS0_ADCMPF1_Msk)        /*!< SARADC comparator 1 interrupt  \hideinitializer */

/*@}*/ /* end of group I91500_SARADC_EXPORTED_CONSTANTS */


/** @addtogroup I91500_SARADC_EXPORTED_FUNCTIONS SARADC Exported Functions
  @{
*/

/**
  * @brief     Get the latest SARADC conversion data
  * @param[in] saradc Base address of SARADC module
  * @param[in] u32ChNum Channel number
  *            - \ref SARADC_SEL_CH0 ~ SARADC_SEL_CH11
  *			   - \ref SARADC_SEL_DIFF_CH0
  *			   - \ref SARADC_SEL_DIFF_CH2
  *			   - \ref SARADC_SEL_DIFF_CH6
  *			   - \ref SARADC_SEL_DIFF_CH4
  * @return    Latest SARADC conversion data
  * \hideinitializer
  */
#define SARADC_GET_CONVERSION_DATA(saradc, u32ChNum) ((saradc->DAT[u32ChNum])&SARADC_DAT_RESULT_Msk)

/**
  * @brief     Return the user-specified interrupt flags
  * @param[in] saradc Base address of SARADC module
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *            - \ref SARADC_ADF_INT
  *            - \ref SARADC_CMP0_INT
  *            - \ref SARADC_CMP1_INT
  * @return    User specified interrupt flags
  * \hideinitializer
  */
#define SARADC_GET_INT_FLAG(saradc, u32Mask) (saradc->STATUS0 & (u32Mask))

/**
  * @brief     This macro clear the selected interrupt status bits
  * @param[in] saradc Base address of SARADC module
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *            - \ref SARADC_ADF_INT
  *            - \ref SARADC_CMP0_INT
  *            - \ref SARADC_CMP1_INT
  * @return    None
  * \hideinitializer
  */
#define SARADC_CLR_INT_FLAG(saradc, u32Mask) (saradc->STATUS0 = (saradc->STATUS0 & ~(SARADC_ADF_INT | \
                                                                       SARADC_CMP0_INT | \
                                                                       SARADC_CMP1_INT)) | (u32Mask))

/**
  * @brief     Get the busy state of SARADC
  * @param[in] saradc Base address of SARADC module
  * @return    busy state of SARADC
  * @retval    0 SARADC is not busy
  * @retval    1 SARADC is busy
  * \hideinitializer
  */
#define SARADC_IS_BUSY(saradc) (saradc->STATUS0 & SARADC_STATUS0_BUSY_Msk ? 1 : 0)

/**
  * @brief     Check if the SARADC conversion data is over written or not
  * @param[in] saradc Base address of SARADC module
  * @param[in] u32ChNum Channel number
  *            - \ref SARADC_SEL_CH0 ~ SARADC_SEL_CH11
  *			   - \ref SARADC_SEL_DIFF_CH0
  *			   - \ref SARADC_SEL_DIFF_CH2
  *			   - \ref SARADC_SEL_DIFF_CH6
  *			   - \ref SARADC_SEL_DIFF_CH4
  * @return    Over run state of SARADC data
  * @retval    0 SARADC data is not overwritten
  * @retval    1 SARADC data is overwritten
  * \hideinitializer
  */
#define SARADC_IS_DATA_OVERRUN(saradc, u32ChNum) ((saradc->DAT[u32ChNum] & SARADC_DAT_OV_Msk) ? 1 : 0)

/**
  * @brief     Check if the SARADC conversion data is valid or not
  * @param[in] saradc Base address of SARADC module
  * @param[in] u32ChNum Channel number
  *            - \ref SARADC_SEL_CH0 ~ SARADC_SEL_CH11
  *			   - \ref SARADC_SEL_DIFF_CH0
  *			   - \ref SARADC_SEL_DIFF_CH2
  *			   - \ref SARADC_SEL_DIFF_CH6
  *			   - \ref SARADC_SEL_DIFF_CH4
  * @return    Valid state of SARADC data
  * @retval    0 SARADC data is not valid
  * @retval    1 SARADC data is valid
  * \hideinitializer
  */
#define SARADC_IS_DATA_VALID(saradc, u32ChNum) ((saradc->DAT[u32ChNum] & SARADC_DAT_VALID_Msk) ? 1 : 0)

/**
  * @brief     Power down SARADC module
  * @param[in] saradc Base address of SARADC module
  * @return    None
  * \hideinitializer
  */
#define SARADC_POWER_DOWN(saradc) (saradc->CTL &= ~SARADC_CTL_ADCEN_Msk)

/**
  * @brief     Power on SARADC module
  * @param[in] saradc Base address of SARADC module
  * @return    None
  * \hideinitializer
  */
#define SARADC_POWER_ON(saradc) (saradc->CTL |= SARADC_CTL_ADCEN_Msk)

/**
  * @brief     Configure the comparator 0 and enable it
  * @param[in] saradc Base address of SARADC module
  * @param[in] u32ChNum  Specifies the source channel
  *            - \ref SARADC_SEL_CH0 ~ SARADC_SEL_CH11
  *			   - \ref SARADC_SEL_DIFF_CH0
  *			   - \ref SARADC_SEL_DIFF_CH2
  *			   - \ref SARADC_SEL_DIFF_CH6
  *			   - \ref SARADC_SEL_DIFF_CH4
  * @param[in] u32Condition Specifies the compare condition
  *            - \ref SARADC_CMP_LESS_THAN
  *            - \ref SARADC_CMP_GREATER_OR_EQUAL_TO
  * @param[in] u32Data Specifies the 12-bit compared data. Valid value are between 0 ~ 0xFFF
  * @param[in] u32MatchCount Specifies the match count setting, valid values are between 1~16
  * @return    None
  * @details   For example, SARADC_ENABLE_CMP0(SARADC, SARADC_SEL_CH2, SARADC_CMP_GREATER_OR_EQUAL_TO, 0xc, 10);
  *            Means SARADC will assert comparator 0 flag if channel 2 conversion result is
  *            greater or equal to 0xc for 10 times continuously.
  * \hideinitializer
  */
#define SARADC_ENABLE_CMP0(saradc, \
                        u32ChNum, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (saradc->CMP[0] = ((u32ChNum) << SARADC_CMP_CMPCH_Pos) | \
                                                         (u32Condition << SARADC_CMP_CMPCOND_Pos) | \
                                                         ((u32Data&SARADC_CMP_CMPDAT_Msk) << SARADC_CMP_CMPDAT_Pos) | \
                                                         ((u32MatchCount - 1) << SARADC_CMP_CMPMCNT_Pos) |\
                                                         SARADC_CMP_ADCMPEN_Msk)
														 
/**
  * @brief     Disable comparator 0
  * @param[in] saradc Base address of SARADC module
  * \hideinitializer
  */
#define SARADC_DISABLE_CMP0(saradc) (saradc->CMP[0] = 0)	

/**
  * @brief     Configure the comparator 1 and enable it
  * @param[in] saradc Base address of SARADC module
  * @param[in] u32ChNum  Specifies the source channel
  *            - \ref SARADC_SEL_CH0 ~ SARADC_SEL_CH11
  *			   - \ref SARADC_SEL_DIFF_CH0
  *			   - \ref SARADC_SEL_DIFF_CH2
  *			   - \ref SARADC_SEL_DIFF_CH6
  *			   - \ref SARADC_SEL_DIFF_CH4
  * @param[in] u32Condition Specifies the compare condition
  *            - \ref SARADC_CMP_LESS_THAN
  *            - \ref SARADC_CMP_GREATER_OR_EQUAL_TO
  * @param[in] u32Data Specifies the 12-bit compared data. Valid value are between 0 ~ 0xFFF
  * @param[in] u32MatchCount Specifies the match count setting, valid values are between 1~16
  * @return    None
  * @details   For example, SARADC_ENABLE_CMP1(SARADC, SARADC_SEL_CH2, SARADC_CMP_GREATER_OR_EQUAL_TO, 0xc, 10);
  *            Means SARADC will assert comparator 0 flag if channel 2 conversion result is
  *            greater or equal to 0xc for 10 times continuously.
  * \hideinitializer
  */
#define SARADC_ENABLE_CMP1(saradc, \
                        u32ChNum, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (saradc->CMP[1] = ((u32ChNum) << SARADC_CMP_CMPCH_Pos) | \
                                                         (u32Condition << SARADC_CMP_CMPCOND_Pos) | \
                                                         ((u32Data&SARADC_CMP_CMPDAT_Msk) << SARADC_CMP_CMPDAT_Pos) | \
                                                         ((u32MatchCount - 1) << SARADC_CMP_CMPMCNT_Pos) |\
                                                         SARADC_CMP_ADCMPEN_Msk)

/**
  * @brief     Disable comparator 1
  * @param[in] saradc Base address of SARADC module
  * \hideinitializer
  */
#define SARADC_DISABLE_CMP1(saradc) (saradc->CMP[1] = 0)	

/**
  * @brief     Start the A/D conversion.
  * @param[in] saradc Base address of SARADC module
  * @return    None
  * \hideinitializer
  */
#define SARADC_START_CONV(saradc) (saradc->CTL |= SARADC_CTL_SWTRG_Msk)

/**
  * @brief     Stop the A/D conversion.
  * @param[in] saradc Base address of SARADC module
  * @return    None
  * \hideinitializer
  */
#define SARADC_STOP_CONV(saradc) (saradc->CTL &= ~SARADC_CTL_SWTRG_Msk)

/**
  * @brief     Set the output format in differential input type.
  * @param[in] saradc Base address of SARADC module
  * @param[in] u32Format output format. Valid values are:
  *            - \ref SARADC_OUT_FORMAT_UNSIGNED
  *            - \ref SARADC_OUT_FORMAT_2COMPLEMENT
  * @return    None
  * \hideinitializer
  */
#define SARADC_SET_OUTFORMAT(saradc, u32Format) (saradc->CTL = (saradc->CTL & ~SARADC_CTL_ADCFM_Msk) | u32Format)

/**
  * @brief     Set A/D converter operation mode.
  * @param[in] saradc Base address of SARADC module
  * @param[in] u8Mode operation mode
  *            - \ref SARADC_OPERATION_MODE_SINGLE
  *            - \ref SARADC_OPERATION_MODE_SINGLE_CYCLE
  *            - \ref SARADC_OPERATION_MODE_CONTINUOUS
  * @return    None
  * \hideinitializer
  */
#define SARADC_SET_OPERATION(saradc, u8Mode)	(saradc->CTL = (saradc->CTL & ~SARADC_CTL_OPMODE_Msk) | u8Mode)

/**
  * @brief     Enable SARADC PDMA receive channel.
  * @param     saradc Base address of SARADC module.
  * @return    None.
  * @details   SARADC will request PDMA service when data is available. When PDMA transfer is enabled,
  *            the SARADC interrupt must be disabled.
  */
#define SARADC_ENABLE_PDMA(saradc)     (saradc->CTL = (saradc->CTL&~SARADC_CTL_ADCIE_Msk)|SARADC_CTL_PDMAEN_Msk)
                                  
/**
  * @brief     Disable SARADC PDMA receive channel.
  * @param     saradc Base address of SARADC module.
  * @return    None.
  */
#define SARADC_DISABLE_PDMA(saradc)     (saradc->CTL &= (~SARADC_CTL_PDMAEN_Msk))

void SARADC_Open(void);
void SARADC_Close(void);
void SARADC_EnableInt(uint32_t u32Mask);
void SARADC_DisableInt(uint32_t u32Mask);
uint32_t SARADC_GetClockSrc(void);
uint32_t SARADC_SetSampleRate(uint32_t u32SampleRate);
uint32_t SARADC_GetSampleRate(void);
void SARADC_EnableChannelSequence(uint32_t u32Sequence, uint32_t u32Channel);

												 
/*@}*/ /* end of group I91500_SARADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_SARADC_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SARADC_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/

