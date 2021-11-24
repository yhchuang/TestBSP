/**************************************************************************//**
 * @file     system_I91500.h
 * @brief    CMSIS Cortex-M0 Device System Header File
 *           for CM0 Device Series
 * @version  V1.01
 * @date     20/07/14
 *
 * @note
 * Copyright (C) 2020 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/


#ifndef __SYSTEM_I91500_H__
#define __SYSTEM_I91500_H__

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------------
  Define system clocks sources
 *----------------------------------------------------------------------------*/
#define __HXT         (12000000UL)   /*!< High Speed External Crystal Clock Frequency 12MHz */
#define __LIRC        (10000UL)      /*!< Low speed internal Oscillator Frequency 10kHz */

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */
extern uint32_t CyclesPerUs;         /*!< Cycles per micro second              */
extern uint32_t g_u32HIRC;           /*!< HIRC frequency                       */
extern uint32_t g_u32BCLK;           /*!< BCLK frequency                       */
extern uint32_t g_u32MCLKI;          /*!< MCLKI frequency                      */

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit (void);

/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock 
 *         retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_I91500_H__ */
