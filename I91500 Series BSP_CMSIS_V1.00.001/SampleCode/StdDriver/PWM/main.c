/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/08/21 10:20a $
 * @brief    PWM Generator for One-shot and Auto-reload Mode Driver Sample Code
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
//#include <string.h>
#include "Platform.h"

void SYS_Init(void)
{
    /* Unlock Protected Registers */
    SYS_UnlockReg();

    /* Enable Internal HIRC and LIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LIRCEN_Msk);

	// Set HIRC frequency = 48M
	CLK_SetHIRCFrequency(CLK_CLKSEL0_HIRCSEL_48M_VCC33);
	
    /* Switch HCLK Clock Source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Reset PWM0 Channel 0~3 */
    SYS_ResetModule(PWM0_RST);
	
	/* Select PWM clock source */
	CLK_SetModuleClock(PWM0_MODULE,CLK_CLKSEL1_PWM0SEL_HIRC,NULL);

    /* Set GPA Multi-function Pins for PWM0 CH0(PA.0) */
    SYS->GPD_MFP  = (SYS->GPD_MFP & (~SYS_GPD_MFP_PD0MFP_Msk) ) | SYS_GPD_MFP_PD0MFP_PWM00  ;

    /* Update System Core Clock */
    /* User can Use SystemCoreClockUpdate() to Calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock Protected Registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Enable UART Module Clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Configure UART0 and Set UART0 Baud Rate */
    UART_Open(UART0, 115200);

    /* Set GPD Multi-function Pins for UART0 TXD(PD.8) and RXD(PD.9) */
    SYS->GPD_MFP  = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD8MFP_Msk|SYS_GPD_MFP_PD9MFP_Msk) ) | (SYS_GPD_MFP_PD8MFP_UART0_TX|SYS_GPD_MFP_PD9MFP_UART0_RX);
}

/* MAIN */
int main (void)
{
    uint8_t u8Item = 0;
    bool exit = TRUE;

    SYS_Init();

    UART0_Init();
	
	while(exit==TRUE)
    {
        printf("\n+------------------------------------------------------------------------+\n");
        printf("|                          PWM Driver Sample Code                        |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("|This Sample Code will Set One-shot or Auto-reload Mode to Generate PWM  |\n");
        printf("|Singal. We Set PWM0 CH0 with Frequency 100Hz and 25 Percent Duty        |\n");
        printf("|I/O Configuration:                                                      |\n");
        printf("|  Waveform Output Pin: PWM0_CH0(PD.0)                                   |\n");
        printf("|[1] Set One-shot Mode to Generate PWM0 CH0 Output Waveform              |\n");
        printf("|[2] Set Auto-shot Mode to Generate PWM0 CH0 Output Waveform             |\n");
        printf("|[x] Exit the Demo                                                       |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("Please Input the Correct Number to Trigger Sample.\n");

        u8Item = getchar();
        /* Assume PWM Output Frequency is 100Hz and Duty Ratio is 25%, User Can Calculate PWM Settings by the Following.
           Duty Ratio = (CMR+1)/(CNR+1)
           Cycle Time = CNR+1
           High Level = CMR+1
           PWM Clock Source Frequency = 10K = 10000
           (CNR+1) = PWM Clock Source Frequency/Prescaler/PWM Output Frequency
                   = 10000/2/100 = 50
           (Note: CNR is 16 Bits, so if Calculated Value is Larger than 65536, User Should Increase Prescale Value.)
           CNR = 49
           Duty Ratio = 25% ==> (CMR+1)/(CNR+1) = 25%
           CMR = 11
           Prescale Value is 1 : Prescaler= 2 */

        /* The API "PWM_ConfigOutputChannel" default is to Set CNTMODE of PWM0->CTL as Auto-reload Mode
        	 So we use "PWM_ConfigOutputChannel_with_Mode" show different mode */

        switch(u8Item)
        {
            case '1':
                /* Set PWM0 Channel 0 Output as One-shot Mode */
                PWM_ConfigOutputChannel_with_Mode(PWM0, PWM_ONE_SHOT_MODE, 200, 25, 0, 0, 0);
                /* Enable PWM Output path for PWM channel 0 */
                PWM_EnableOutput(PWM0, 0);
                /* Start */
                PWM_Start(PWM0);
                break;
            case '2':
                /* Set PWM0 Channel 0 Output as Auto-reload Mode */
                PWM_ConfigOutputChannel_with_Mode(PWM0, PWM_AUTO_RELOAD_MODE, 100, 25, 0, 0, 0);
                /* Enable PWM Output path for PWM channel 0 */
                PWM_EnableOutput(PWM0, 0);
                /* Start */
                PWM_Start(PWM0);
                break;
            case 'x':
                /* Exit the Demo */
                exit = FALSE;
                PWM_Stop(PWM0);
                printf("\nDemo End!!\n");
                break;
            default:
                printf("\nPlease Input the Correct Number to Trigger Sample.\n");
                break;
        }
    }
    
}
