/**************************************************************************//**
 * @file     hirc_trim.h
 * @brief    HIRC Trim sample file
 * @version  1.0.0
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __HIRC_TRIM_H__
#define __HIRC_TRIM_H__

void HIRC_AutoTrim_Enable(uint32_t u32FreqSel, uint16_t *u16TrimedValue);
void HIRC_AutoTrim_Reset(void);

#endif //__HIRC_TRIM_H__
