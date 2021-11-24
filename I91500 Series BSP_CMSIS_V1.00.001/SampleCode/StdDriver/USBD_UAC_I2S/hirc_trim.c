/**************************************************************************//**
 * @file     hirc_trim.c
 * @brief    HIRC Trim sample file
 * @version  1.0.0
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <string.h> 
#include "Platform.h"
#include "hirc_trim.h"

//----------------------------------------------------------------------------
//  Global variables for Control Pipe
//---------------------------------------------------------------------------- 

volatile uint8_t g_u8HIRCAutoTrimDone;
volatile uint8_t g_u8HIRCAutoTrimCnt;
volatile uint8_t g_u8HIRCAutoTrimEnable;

/**
 * @brief       Enable HIRC auto trim by USB SOF
 *
 * @param[in]   u32FreqSel      HIRC frequency selection
 * @param[in]   u16TrimedValue  update value after trimed 
 *
 * @return      None
 *
 * @details     This function is used to enable HIRC auto trim function, and trim by USB SOF
 */
void HIRC_AutoTrim_Enable(uint32_t u32FreqSel, uint16_t *u16TrimedValue)
{
    if(g_u8HIRCAutoTrimDone == 0)
    {
        if(g_u8HIRCAutoTrimEnable == 0)
        {
            g_u8HIRCAutoTrimEnable = 1;
            SYS_SET_TRIMHIRC_LOOPSEL(SYS_IRCTCTL_LOOPSEL_4);    // Only LOOPSEL_4 is correct
            SYS_SET_TRIMHIRC_RETRYCNT(SYS_IRCTCTL_RETRYCNT_64);				
            SYS_ENABLE_TRIMHIRC_CLKERRSTOP();  // No-Stop trimming when clock inaccuracy
            SYS_SET_TRIMHIRC_REFCLK(SYS_IRCTCTL_REFCKSEL_USBD_SOF);
            SYS_CLEAR_TRIMHIRC_INT_FLAG(SYS_IRCTISTS_TRIMFAIL_INT_FLAG|SYS_IRCTISTS_CLKERROR_INT_FLAG); 
            SYS_ENABLE_TRIMHIRC_INT(SYS_IRCTIEN_TRIMFAIL_INT_MASK|SYS_IRCTIEN_CLKERROR_INT_MASK);   // Enable clock error / trim fail interrupt 
            SYS_EnableTrimHIRC(u32FreqSel);
        } 
        if(SYS_IS_TRIMHIRC_DONE())
        {
            g_u8HIRCAutoTrimCnt++;
            g_u8HIRCAutoTrimEnable = 0;   
            SYS_DisableTrimHIRC();
            SYS_CLEAR_TRIMHIRC_INT_FLAG(SYS_IRCTISTS_FREQLOCK_Msk);
            if(g_u8HIRCAutoTrimCnt == 5)
            {
                g_u8HIRCAutoTrimDone = 1;
                g_u8HIRCAutoTrimCnt = 0;
                g_u8HIRCAutoTrimEnable = 0;
                *u16TrimedValue = SYS_GET_OSCTRIM_VALUE(); 
            }   
        }
        else if(SYS_GET_TRIMHIRC_INT_FLAG(SYS_IRCTISTS_TRIMFAIL_INT_FLAG | SYS_IRCTISTS_CLKERROR_INT_FLAG))
        {
            g_u8HIRCAutoTrimCnt = 0;
            g_u8HIRCAutoTrimEnable = 0;
            SYS_DisableTrimHIRC();              
            SYS_CLEAR_TRIMHIRC_INT_FLAG(SYS_IRCTISTS_TRIMFAIL_INT_FLAG|SYS_IRCTISTS_CLKERROR_INT_FLAG);
            SYS_EnableTrimHIRC(u32FreqSel); 
        }    
    }
}

/**
 * @brief       Disable HIRC auto trim
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to disable HIRC auto trim function, reset related variables
 */
void HIRC_AutoTrim_Reset(void)
{
    SYS_DisableTrimHIRC();         
    g_u8HIRCAutoTrimDone = 0;
    g_u8HIRCAutoTrimCnt = 0;
    g_u8HIRCAutoTrimEnable = 0;
}
