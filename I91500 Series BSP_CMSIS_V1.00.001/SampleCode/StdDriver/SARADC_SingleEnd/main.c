/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/06/28 10:50am $
 * @brief    Demo SARADC SAR0 (GPA0) single end; GPB8 connect 2V
 *
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Platform.h"

#define BUFF_SIZE	(20)
volatile uint32_t u32Value[BUFF_SIZE], u32SampleRate;
volatile uint8_t u8Idx;

// =====================================================================================
void SARADC_IRQHandler(void)
{
        UINT32 u32Tmp = 0;
        
	if (SARADC_GET_INT_FLAG(SARADC,SARADC_ADF_INT)&SARADC_ADF_INT)
        {
                u32Tmp = SARADC_GET_CONVERSION_DATA(SARADC, SARADC_SEL_CH0);
		u32Value[u8Idx++] = u32Tmp;
	}		
	SARADC_CLR_INT_FLAG(SARADC, SARADC_ADF_INT);
}

void System_Initiate(void)
{
    // Unlock protected registers
    SYS_UnlockReg();
    // Enable clock source 
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
	// Set HIRC frequency = 48M
	CLK_SetHIRCFrequency(CLK_CLKSEL0_HIRCSEL_48M_VCC33);
    // Switch HCLK clock source to HIRC
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
	// Update System Core Clock
    SystemCoreClockUpdate();
    // Lock protected registers
    SYS_LockReg();	
}

void SARADC_Init(void)
{
	// Set gpio input mode for SARADC
	GPIO_SetMode(PA, BIT0, GPIO_MODE_INPUT);
	
	// Enable SARADC module clock. 
	CLK_EnableModuleClock(SARADC_MODULE);
	
	SYS_ResetModule(SARADC_RST);
	
	// Set SARADC clock source: HIRC = 49512000.
	CLK_SetModuleClock(SARADC_MODULE, CLK_CLKSEL1_SARADCSEL_HIRC, CLK_CLKDIV_SARADC(1));
	
	// A/D Converter Enable 
	SARADC_POWER_ON(SARADC);
	
	// Set DLYTRIM to level3 for best performance
	SARADC->CTL |= SARADC_CTL_DLYTRIM_Msk;
	
	// Set SARADC sample rate
	u32SampleRate = SARADC_SetSampleRate(100000);
	
	printf("SARADC Sample rate %d Hz.\r\n\r\n", u32SampleRate);
	
	SARADC_EnableChannelSequence(SARADC_SEL_SEQ0,SARADC_SEL_CH0);
	
	SARADC_SET_OPERATION(SARADC, SARADC_OPERATION_MODE_CONTINUOUS);
	
	SARADC_EnableInt(SARADC_ADF_INT);
	
	NVIC_ClearPendingIRQ(SARADC_IRQn);
	
	NVIC_EnableIRQ(SARADC_IRQn);
	
	SARADC_START_CONV(SARADC);
}

void UART0_Init(void)
{
    // Enable SARADC module clock. 
	CLK_EnableModuleClock(UART0_MODULE);
	
	/* Reset UART module */
    SYS_ResetModule(UART0_RST);
	
	/* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, MODULE_NoMsk, CLK_CLKDIV_UART0(1));
	    
    // Init I/O multi-function ; UART0: GPA4=TX, GPA5= RX
	SYS->GPD_MFP  = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD8MFP_Msk|SYS_GPD_MFP_PD9MFP_Msk) ) |  (SYS_GPD_MFP_PD8MFP_UART0_TX|SYS_GPD_MFP_PD9MFP_UART0_RX)	;

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}


int main(void)
{
	uint8_t u8i;

	System_Initiate();
	
	UART0_Init();
	
    printf("\r\nCPU @ %d Hz\r\n", SystemCoreClock);
    printf("+---------------------------------------------+\r\n");
    printf("| Demo SARADC SAR0 (GPA0) single end		  |\r\n");
    printf("+---------------------------------------------+\r\n");
	
	u8Idx = 0;
	SARADC_Init();
	
	while(1)
	{
		if (u8Idx>=BUFF_SIZE)
		{
			SARADC_STOP_CONV(SARADC);
			SARADC_DisableInt(SARADC_ADF_INT);
			NVIC_DisableIRQ(SARADC_IRQn);
			for (u8i = 0; u8i<BUFF_SIZE; u8i++)
				printf("The value: %d ..\r\n", u32Value[u8i] );
		}
	}
}
