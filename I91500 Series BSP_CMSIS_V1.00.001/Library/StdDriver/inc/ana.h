/**************************************************************************//**
 * @file     ana.h
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 16/08/22 13:27p $
 * @brief    I91500 Series SDADC Driver Header File
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __ANA_H__
#define __ANA_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_ANA_Driver ANA Driver
  @{
*/

/** @addtogroup I91500_ANA_EXPORTED_CONSTANTS ANA Exported Constants
  @{
*/  

#define ANA_MICBCTR_MICBVSEL_90VCCA    (0x0ul << ANA_MICBCTR_MICBVSEL_Pos)         /*!< 90% VCCA  */
#define ANA_MICBCTR_MICBVSEL_65VCCA    (0x1ul << ANA_MICBCTR_MICBVSEL_Pos)         /*!< 65% VCCA  */
#define ANA_MICBCTR_MICBVSEL_75VCCA    (0x2ul << ANA_MICBCTR_MICBVSEL_Pos)         /*!< 75% VCCA  */
#define ANA_MICBCTR_MICBVSEL_50VCCA    (0x3ul << ANA_MICBCTR_MICBVSEL_Pos)         /*!< 50% VCCA  */
#define ANA_MICBCTR_MICBVSEL_2P4V      (0x4ul << ANA_MICBCTR_MICBVSEL_Pos)         /*!< 2.4V  */
#define ANA_MICBCTR_MICBVSEL_1P7V      (0x5ul << ANA_MICBCTR_MICBVSEL_Pos)         /*!< 1.7V  */
#define ANA_MICBCTR_MICBVSEL_2P0V      (0x6ul << ANA_MICBCTR_MICBVSEL_Pos)         /*!< 2.0V  */
#define ANA_MICBCTR_MICBVSEL_1P3V      (0x7ul << ANA_MICBCTR_MICBVSEL_Pos)         /*!< 1.3V  */


void ANA_MICBIAS_Enable(uint8_t u8Level);
void ANA_MICBIAS_Disable(void);
void ANA_VMID_Enable(void);

/*@}*/ /* end of group I91500_ANA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_ANA_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__ANA_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
