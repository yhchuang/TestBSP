/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/11/18 10:20a $
 * @brief    Driver BOD demo function.
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
#include "Platform.h"

void BOD_IRQHandler(void)
{
	/* Clear interrupt flag */
	BOD_ClearIntFlag();

	if(BOD_GetOutput() == 1)
	{
		GPIO_SET_OUT_DATA(PA, 0x000F); // turn off LED
	}	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable External OSC49M */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Enable External OSC10K */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
	// Set HIRC frequency = 48M
	CLK_SetHIRCFrequency(CLK_CLKSEL0_HIRCSEL_48M_VCC33);
    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Lock protected registers */
    SYS_LockReg();
		
	// LED pin setup
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA0MFP_Msk) ) | SYS_GPA_MFP_PA0MFP_GPIO  ;
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA1MFP_Msk) ) | SYS_GPA_MFP_PA1MFP_GPIO  ;
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA2MFP_Msk) ) | SYS_GPA_MFP_PA2MFP_GPIO  ;
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA3MFP_Msk) ) | SYS_GPA_MFP_PA3MFP_GPIO  ;
	GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);
	GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT);
	GPIO_SetMode(PA, BIT2, GPIO_MODE_OUTPUT);
	GPIO_SetMode(PA, BIT3, GPIO_MODE_OUTPUT);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
	int i,j;
	
	/* Lock protected registers */
	if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
	 SYS_LockReg();

	/* Init System, IP clock and multi-function I/O
	  In the end of SYS_Init() will issue SYS_LockReg()
	  to lock protected register. If user want to write
	  protected register, please issue SYS_UnlockReg()
	  to unlock protected register if necessary */
	SYS_Init();
	
	for (i=0;i<5;i++)
	{
		for (j=0;j<300000;j++) GPIO_SET_OUT_DATA(PA, ~0x000F); // turn on LED
		for (j=0;j<2000000;j++) GPIO_SET_OUT_DATA(PA, 0x000F); // turn off LED
	}
	GPIO_SET_OUT_DATA(PA, ~0x000F); // turn on LED
	
	/*---------------------------------------------------------------------------------------------------------*/
	/* Configure BOD                                                                                           */
	/*---------------------------------------------------------------------------------------------------------*/
	/* Enable BOD and interrupt mode */
	BOD_Open( BOD_INTERRUPT_MODE, BOD_BODVL_24V);
	
	BOD_LVR_Enable();
	
	/* Enable hysteresis */
	BOD_HYS_Enable();
	
	NVIC_EnableIRQ(BOD_IRQn);
	
	while(1);
}
