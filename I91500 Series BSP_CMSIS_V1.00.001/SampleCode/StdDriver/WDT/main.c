/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 20/11/19 2:52p $
 * @brief    WDT Driver Sample Code.
 *           1. Initiate system clock.
 *           2. Initiate and start WDT IP interface.
 *           3. Keep clear WDT reset counter in main loop.
 *              - Flash 'DA0' LED.
 *              - Press 'SWA4' to stop WDT reset counter.
 *           4. When user press SWA4, MCU will be reset at WDT time-out.
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Platform.h"

// Pre-declare function.
void System_Initiate(void);

void UART_Init(void);

void DEMO_InitGPIO(void);
BOOL DEMO_IsSWA4Pressing(void);
void DEMO_FlashDA0(void);

void ShowMessage_Setup(void);
void ShowMessage_Process(void);
void ShowMessage_PreReset(void);

int main(void)
{
	// Initiate system clock
	System_Initiate();		
	
	// Initiate UART.
	UART_Init();
	
	// GPIO configuration for this demo sample(DA0,SWA4)]
	DEMO_InitGPIO();
	
	// Message : WDT setup.
	ShowMessage_Setup();

	// Initiate WDT
	{
		// Enable IP clock
		CLK_EnableModuleClock(WDT_MODULE);
		// Select IP clock source
		CLK_SetModuleClock(WDT_MODULE,MODULE_NoMsk,MODULE_NoMsk);
		// Open WDT interface.
		// - WDT_TIMEOUT_2POW12 : Time-out frequency(2^12*WDT clock).
		// - TRUE : Enable reset MCU.
		WDT_Open(WDT_TIMEOUT_2POW12,TRUE);
		// Disable WDT interrupt for clear reset counter in main loop.
		WDT_DisableInt();
	}
	
	// Message : WDT in process.
	ShowMessage_Process();
	
	// If SWA4 is pressing, quit this loop and stop reset WDT counter.
	while(DEMO_IsSWA4Pressing()==FALSE)
	{
		// Reset WDT counter.
		WDT_RESET_COUNTER();
		// Flash DA0 LED.
		DEMO_FlashDA0();
	}

	// Message : WDT prepare to reset MCU
	ShowMessage_PreReset();
	
	while(1);
}

UINT32 u32DEMO_Counter = 0;
// =====================================================================================
void System_Initiate(void)
{
	// Unlock protected registers
	SYS_UnlockReg();
	// Enable External XTL32K 
	CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_LIRCEN_Msk);
	// Switch HCLK clock source to HXT
	CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
	// Update System Core Clock
	SystemCoreClockUpdate();
	// Lock protected registers
	SYS_LockReg();
}
void UART_Init(void)
{
	/* Enable UART Module Clock */
	CLK_EnableModuleClock(UART0_MODULE);

	/* Configure UART0 and Set UART0 Baud Rate */
	UART_Open(UART0, 115200);

    /* Set GPD Multi-function Pins for UART0 TXD(PD.8) and RXD(PD.9) */
    SYS->GPD_MFP  = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD8MFP_Msk|SYS_GPD_MFP_PD9MFP_Msk) ) | (SYS_GPD_MFP_PD8MFP_UART0_TX|SYS_GPD_MFP_PD9MFP_UART0_RX);
}
void DEMO_InitGPIO(void)
{
	// Set PA0 to output pin(DA0). 
	GPIO_SetMode(PA,BIT0,GPIO_MODE_OUTPUT);
	// Set PA4 to input pin(SWA4)
	GPIO_SetMode(PA,BIT4,GPIO_MODE_INPUT);	
	// Wait PA4 is ready.
	while(DEMO_IsSWA4Pressing()==TRUE)
	{
		if(u32DEMO_Counter++>100000)
			break;
	}
}
BOOL DEMO_IsSWA4Pressing(void)
{
	return ((GPIO_GET_IN_DATA(PA)&BIT4)?FALSE:TRUE);
}
void DEMO_FlashDA0(void)
{
	if((u32DEMO_Counter++)<100000)
		GPIO_SET_OUT_DATA(PA,GPIO_GET_OUT_DATA(PA)&~(BIT0));
	else 
	{
		if(u32DEMO_Counter<200000)
			GPIO_SET_OUT_DATA(PA,GPIO_GET_OUT_DATA(PA)|BIT0);
		else
			u32DEMO_Counter = 0;
	}
}
void ShowMessage_Setup(void)
{
	printf("\n+------------------------------------------------------------------------+\n");
	printf("|                       WDT Driver Sample Code                           |\n");
	printf("+------------------------------------------------------------------------+\n");	
	if( u32DEMO_Counter>=100000 )
	{
		printf("SWA4 is not ready(in LOW state).\n");
		while(1);
	}
	else
	{
		u32DEMO_Counter = 0;
		printf("(1) Setup WDT, includes.\n");
		printf("    1. Enable and set WDT IP clock.\n");
		printf("    2. Set WDT Time-Out frequency = (2^12)*WDT's clock.\n");
		printf("    3. Enable reset MCU.\n");	
	}		
}
void ShowMessage_Process(void)
{
	printf("(2) WDT is running now.\n");	
	printf("    1. DA0 is flash at the same time.\n");	
	printf("    2. User can press SWA4 to stop WDT reset counter clear.\n");	
}
void ShowMessage_PreReset(void)
{
	printf("(3) Stop flash DA0 and prepare to reset MCU.\n");
}
