/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/10/27 09:10a $
 * @brief    I91500 General Purpose I/O Driver Sample Code
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "I91500.h"

volatile uint8_t u8Counter = 0;
uint8_t u8Option = 0;

void TMR0_IRQHandler(void)
{
    printf("Timer IRQ handler run counter = %d\n", ++u8Counter );
    TIMER_ClearIntFlag(TIMER0);	
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
	
    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    // Update System Core Clock
    // User can use SystemCoreClockUpdate() to calculate SystemCoreClock.
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    /* Lock protected registers */
    if(SYS_IsRegLocked() == 0) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------------+ \n");
    printf("|      Timer Driver Sample Code       | \n");
    printf("+-------------------------------------+ \n");

    while(1) 
    {
        printf("\nSelect Test Item(1-2):\n");
        printf("Key 1: Timer IRQ Handeler.\n");
        printf("Key 2: Timer Delay 2 sec.\n");
				
        u8Option = getchar();

        switch(u8Option)
        {
            case '1':
            printf("\nStart to Timer IRQ Handeler test.\n\n" );
            // Using TIMER0 PERIODIC_MODE , Freq=10Hz.
            TIMER_Open( TIMER0, TIMER_PERIODIC_MODE, 10); 
            u8Counter = 0;
            // Start Timer 0
            TIMER_Start(TIMER0);     
            // Enable timer interrupt
            TIMER_EnableInt(TIMER0); 
            NVIC_EnableIRQ(TMR0_IRQn);	
            // Wait interrupt happened 20 times.
            while( u8Counter < 20 ); 
            // Close timer for next test.
            TIMER_Close(TIMER0);     

            printf("\nTimer IRQ Handeler test complete.\n" );			
            break;
           
		    case '2':
            TIMER_Delay( TIMER0, 333333);	 // Delay 333.333 milli second via timer0.
            TIMER_Delay( TIMER0, 333333);	 // Delay 333.333 milli second via timer0.
            TIMER_Delay( TIMER0, 333333);	 // Delay 333.333 milli second via timer0.
            TIMER_Delay( TIMER0, 333333);	 // Delay 333.333 milli second via timer0.
            TIMER_Delay( TIMER0, 333333);	 // Delay 333.333 milli second via timer0.
            TIMER_Delay( TIMER0, 333333);	 // Delay 333.333 milli second via timer0.

            printf("\nTimer Delay 2 sec test complete.\n" );
            break;
        }				
    }
}
