/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2020 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "Platform.h"

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t g_u32HIRC = 49152000UL;
uint32_t SystemCoreClock = 49152000UL;                     /*!< System Clock Frequency (Core Clock)*/
uint32_t CyclesPerUs = ((49152000UL) / 1000000);           /*!< Cycles per micro second            */
uint32_t g_u32MCLKI = 0UL;
uint32_t g_u32BCLK = 0UL;

/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)                          /* Get Core Clock Frequency      */
{	
	uint32_t u32ClkSrc;
	
	switch((CLK->CLKSEL0&CLK_CLKSEL0_HCLKSEL_Msk))
	{
		case 0:  u32ClkSrc = __HXT;            break;
		case 1:  u32ClkSrc = CLK_GetPLLFreq(); break;
		case 2:  u32ClkSrc = __LIRC;           break;
		default:  u32ClkSrc = g_u32HIRC;       break;
	}
	
	/* Update System Core Clock */
	SystemCoreClock = u32ClkSrc/((CLK->CLKDIV0&CLK_CLKDIV0_HCLKDIV_Msk)+1);
	
    CyclesPerUs = SystemCoreClock/1000000;
}

/**
 * Initialize the system
 * @return none
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{
  
}
