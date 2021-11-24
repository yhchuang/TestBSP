/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/07/15 10:20a $
 * @brief    Demonstrate the usage of SYS driver by changing different 
 *           HCLK setting for the system clock source. 
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "Platform.h"

//=========================================================================================
// Functions declaration
//=========================================================================================
void UART_Init(void);

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

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
	
    /* Enable External OSC49M */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    
	/* Enable External OSC10K */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
    
    /* Enable PLL */
    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, 48000000);
    /* Waiting for PLL clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);
    
	/* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    uint32_t u32data;
    uint8_t u8Option = 0;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    UART_Init();
    /*
        This sample code will show some function about system manager controller and clock controller:
        1. Read PDID
        2. Get and clear reset source
        3. Lock & unlock protected register.
	    4. Change system clock.
	    5. Reset 
    */
    printf("\n+------------------------------------------------------------------------+\n");
    printf("|                      System Driver Sample Code                         |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("CPU @ %dHz\n", SystemCoreClock );

    /*---------------------------------------------------------------------------------------------------------*/
    /* Misc system function test                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Read Part Device ID */
    printf("Product ID 0x%x\n", SYS_ReadPDID() );

    /* Get reset source from last operation */
    u32data = SYS_GetResetSrc();
    printf("Reset Source 0x%x\n", u32data);
	
    /* Clear reset source */
    SYS_ClearResetSrc(u32data);
	
    /* Unlock protected registers for Brown-Out Detector settings */
    SYS_UnlockReg();
	
    printf("Select HCLK Clock Selection(0-3), Exit(Other Key):\n");
    printf("Key 0: Internal OSC 49152000 Hz\n");
    printf("Key 1: External Xtal 12000000 Hz\n");
    printf("Key 2: Internal OSC 10000 Hz(Only support semihost mode to display)\n");
	printf("Key 3: PLL Frequency Output\n");
	
    do
    {
        u8Option = getchar();
		
        switch(u8Option)
        {
            case '0':
            CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
            break;	
			
            case '1':
            CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT, CLK_CLKDIV_HCLK(1));
            break;									

            case '2':
            CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_LIRC, CLK_CLKDIV_HCLK(1));
            break;	
	
            case '3':
            CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLLFOUT, CLK_CLKDIV_HCLK(1));
            break;

            default:
            u8Option = 0;
            break;
        }
		
        if( u8Option != 0 )
            printf("HCLK frequency became %d Hz\n", CLK_GetHCLKFreq());

    }while( u8Option != 0 );	
}


