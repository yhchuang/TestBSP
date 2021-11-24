/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 21/04/22 10:25a $
 * @brief    Demonstrate DAC play 8kHz audio with different sample rate 
 *           and modulation frequency
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Platform.h"

//=========================================================================================
// Functions declaration
//=========================================================================================
void SYS_Init(void);
void UART_Init(void);
void DAC_Init(uint32_t u32SampleRate);

//=========================================================================================
// Global Variable
//=========================================================================================
extern uint32_t u32audioBegin, u32audioEnd;
uint32_t DACSampleRateFreq[4] = {8000, 16000, 32000, 48000}; //Sample rate
int16_t *pi16AudioAdd;
int16_t *pi16AudioEnd;

//=========================================================================================
// Main Function
//=========================================================================================
int32_t main(void)
{				
	uint8_t u8Option, u8Exit=0;
	uint32_t u32Count,u32Delay;

	/* Init System, IP clock and multi-function I/O */
	//In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.
	SYS_Init(); 	
	// UART initial for printf
	UART_Init();
	
	printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    ANA_VMID_Enable();
	// Enable DAC clock, Set DAC clock source and sample rate 
	DAC_Init(8000);

	/* Init audio data */
	pi16AudioEnd = (int16_t *)&u32audioEnd;
	printf("\n\n");
	printf("+----------------------------------------------------------------------+\n");
	printf("|                       DAC Driver Sample Code                         |\n");
	printf("+----------------------------------------------------------------------+\n");
	printf("\n");
	printf("The DAC configuration is ready.\n");
	printf("DAC sampling rate: %d Hz\n", DAC_GetSampleRate());	 
	printf("Please connect headphone to HPOR and HPRL pin\n");
	printf("Press Enter key to start...\n");
	getchar();
	printf("\nDAC Test Begin...............\n");
		
	while(u8Exit == 0)
	{
		// Input audio data into DAC fifo.
		pi16AudioAdd = (int16_t *)&u32audioBegin;
		u32Count = pi16AudioEnd-pi16AudioAdd;
		
		while(u32Count > 0)	
		{
			while(DAC_IS_FIFOFULL(DAC)); //FIFO is full
			DAC_SET_DATA(DAC,(uint32_t)*pi16AudioAdd);
			pi16AudioAdd++;
			u32Count--;	
		}
		
		// Wait all data playback end.
		while(!DAC_IS_FIFOEMPTY(DAC));
		
		u32Delay = 10000000;
		while((u32Delay--)!=0);
		
		printf("Key 0: Change DAC clock frequency\n");
		printf("Key 1: Exit\n");
		u8Option = getchar();
		printf("\n");

		switch(u8Option)
		{
			case '0':
				printf("Select sample rate: \n");
				printf("Key 0: 8000 Hz\n");
				printf("Key 1: 16000 Hz\n");
				printf("Key 2: 32000 Hz\n");
				printf("Key 3: 48000 Hz\n");
				u8Option = getchar()&0x03;
				DAC_SetSampleRate(DACSampleRateFreq[u8Option]);	
				printf("Playing 8k Hz audio by DAC sampling rate %d Hz\n", DAC_GetSampleRate()); 
			break;
			case '1':
				u8Exit = 1;
				printf("Exit DAC Demo\n");
			break;
		}
		
		pi16AudioAdd = (int16_t *)&u32audioBegin;
		printf("\n");
	}
	
	CLK_DisableModuleClock(DAC_MODULE);
	while(1);
}

//=========================================================================================
// DAC
//=========================================================================================
void DAC_Init(uint32_t u32SampleRate)
{  
    /* Enable DAC clock. */
    CLK_EnableModuleClock(DAC_MODULE);
    CLK_SetModuleClock(DAC_MODULE,CLK_CLKSEL2_DACSEL_HIRC,CLK_CLKDIV_DAC(1));
    /* Sample rate configure */
    DAC_SetSampleRate(u32SampleRate);
    /* Enable DAC FIFO */
	DAC_ENABLE_FIFO(DAC);
	/* Enable channel */
	DAC_ENABLE_RIGHT_CHANNEL(DAC);
	DAC_ENABLE_LEFT_CHANNEL(DAC);
	/* Set channel volume */
	DAC_SET_RIGHT_CHANNEL_VOL(DAC,DAC_DVOL_0DB);
	DAC_SET_LEFT_CHANNEL_VOL(DAC,DAC_DVOL_0DB);
	/* Set fifo data width */
	DAC_SET_DATAWIDTH(DAC,DAC_CTL0_DATAWIDTH_16BIT);
    DAC_SET_HP_VOL(DAC,DAC_BV1P5_NEG12DB);
    /* Initialize analog for dac */
    DAC_EnableAnalog();    
}


// Initiate System.
void SYS_Init(void)
{
    // Unlock Protected Registers
    SYS_UnlockReg();
    // Enable Internal OSC49M
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
	// Set HIRC frequency = 48M
	CLK_SetHIRCFrequency(CLK_CLKSEL0_HIRCSEL_49M_VCC33);
    // Switch HCLK Clock Source to HIRC 
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
    // Update System Core Clock
    // User can Use SystemCoreClockUpdate() to Calculate SystemCoreClock. 
    SystemCoreClockUpdate();
    // Lock Protected Registers 
    SYS_LockReg();
}

// Initiate UART for printting.
void UART_Init(void)
{
    /* Enable UART Module Clock */ 
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART0 clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV_UART0(1)); 
    /* Configure UART0 and Set UART0 Baud Rate */ 
    UART_Open(UART0, 115200);
    /* Set GPD Multi-function Pins for UART0 TXD(PD.8) and RXD(PD.9) */
    SYS->GPD_MFP  = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD8MFP_Msk|SYS_GPD_MFP_PD9MFP_Msk) ) | (SYS_GPD_MFP_PD8MFP_UART0_TX|SYS_GPD_MFP_PD9MFP_UART0_RX);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
