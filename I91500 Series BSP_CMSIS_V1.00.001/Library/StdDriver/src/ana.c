/**************************************************************************//**
 * @file     ANA.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 20/11/26 07:27p $
 * @brief    I91500 ANA driver source file
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include  "Platform.h"

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_ANA_Driver ANA Driver
  @{
*/

/** @addtogroup I91500_ANA_EXPORTED_FUNCTIONS ANA Exported Functions
  @{
*/

void ANA_VMID_Enable(void)
{
    uint32_t u32i;
    
    CLK_EnableModuleClock(ANA_MODULE);
    CLK_EnableModuleClock(DAC_MODULE);
    DAC->ANA0 &= ~DAC_ANA0_VREFSEL_Msk;
    DAC->ANA1 |= (DAC_ANA1_PDBDAC2_Msk | DAC_ANA1_PDBDAC1_Msk);
    DAC->ANA1 &= ~(DAC_ANA1_PDIBGEN_Msk | DAC_ANA1_PDVBUF2_Msk | DAC_ANA1_PDVBUF1_Msk);
    ANA->VMID = 0;
    u32i = 0xA00000; //delay 1s
    while(u32i>0) u32i--; 
    ANA->VMID |= (ANA_VMID_VMIDLRL_Msk | ANA_VMID_VMIDHRL_Msk);
}

/**
  * @brief      Enable MICBIAS and Set MICBIAS Level
  * @param      u8Level is micbias level.
  *             - \ref ANA_MICBCTR_MICBVSEL_90VCCA 
  *             - \ref ANA_MICBCTR_MICBVSEL_65VCCA 
  *             - \ref ANA_MICBCTR_MICBVSEL_75VCCA
  *             - \ref ANA_MICBCTR_MICBVSEL_50VCCA
  *             - \ref ANA_MICBCTR_MICBVSEL_2P4V
  *             - \ref ANA_MICBCTR_MICBVSEL_1P7V
  *             - \ref ANA_MICBCTR_MICBVSEL_2P0V
  *             - \ref ANA_MICBCTR_MICBVSEL_1P3V
  * @return     None.
  * @details    Set Micbias level and enable micbias         
  */
void ANA_MICBIAS_Enable(uint8_t u8Level)
{
    uint32_t u32i;
    
    CLK_EnableModuleClock(ANA_MODULE);
    ANA->MICBCTR = (u8Level | ANA_MICBCTR_MICBEN_Msk);
    u32i = 0x1000; //delay 500us
    while(u32i>0) u32i--; 
}

/**
  * @brief      Disable MICBIAS
  * @return     None.
  * @details    Disable micbias         
  */

void ANA_MICBIAS_Disable(void)
{
    CLK_EnableModuleClock(ANA_MODULE);
    ANA->MICBCTR &= ~ANA_MICBCTR_MICBEN_Msk;
}


/*@}*/ /* end of group I91500_ANA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_ANA_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/

