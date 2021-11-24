/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date:    20/11/24 9:36a $
 * @brief    Please refer readme.txt
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h> 
#include "Platform.h"
#include "hid_mouse.h"

// main ========================================================================================================= 
int main(void)
{	
    /* Unlock Protected Registers */
    SYS_UnlockReg();
    /* Enable Internal HIRC and LIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
	// Set HIRC frequency = 48M
	CLK_SetHIRCFrequency(CLK_CLKSEL0_HIRCSEL_48M_VCC33);
    /* Switch HCLK Clock Source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
    /* Update System Core Clock */
    /* User can Use SystemCoreClockUpdate() to Calculate SystemCoreClock. */
    SystemCoreClockUpdate();
	
	// Initiate HID mouse(include USBD hardware IP)
	{
		// Reset module.
		SYS_ResetModule(USBD_RST);
		// Enable USBD module clock.
		CLK_EnableModuleClock(USBD_MODULE);
		// Set USBD clock divid
		CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL2_USBDSEL_HIRC, CLK_CLKDIV_USBD(1));
		// Initiate USBD hardware IP and input HID request for hand-shake.
		USBD_Open(&gsInfo, HID_ClassRequest, NULL);
		
		// Initiate HID for endpoint configuration 
		HID_Init();
		// Enable USB IRQ
		NVIC_EnableIRQ(USBD_IRQn);
		// Start USBD for processing.
		USBD_Start();
	}
	// Process in main loop.
	while(1)
	{
		HID_UpdateMouseData();
	}
}


/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
