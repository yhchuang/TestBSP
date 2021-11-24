/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief
 *       Use PWM Capture Input Mode to Meassure PWM Output Driver Sample Code
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "Platform.h"

/* Numbers of Capture Data between Rising and Falling Down-Counter */
#define NUMS				2

uint16_t u32Count[NUMS] = {0};
uint16_t u16RisingCount, u16FallingCount, u16HighPeriod, u16LowPeriod, u16TotalPeriod = 0;

void SYS_Init(void)
{
	/* Unlock Protected Registers */
	SYS_UnlockReg();

	/* Enable Internal HIRC and LIRC */
	CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LIRCEN_Msk);

	/* Switch HCLK Clock Source to HIRC */
	CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));

	/* Enable IP clock */
	CLK_EnableModuleClock(PWM0_MODULE);
	CLK_EnableModuleClock(PWM1_MODULE);

	/* Select PWM module Clock Source */
	CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0SEL_HIRC, 0);
	CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL1_PWM1SEL_HIRC, 0);

	/* Reset PWM0 and PWM1 Channel 0~3 */
	SYS_ResetModule(PWM0_RST);
	SYS_ResetModule(PWM1_RST);

	/* Set GPA Multi-function Pins for PWM0 CH2(PC.4) and PWM1 CAP1(PA.6) */
	SYS->GPC_MFP  = (SYS->GPC_MFP & (~SYS_GPC_MFP_PC4MFP_Msk) ) | SYS_GPC_MFP_PC4MFP_PWM02  ;
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA6MFP_Msk) ) | SYS_GPA_MFP_PA6MFP_CAP1  ;

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

void CalPWM(void)
{
	uint8_t cnt = 0;
	int i = 0;

	/* Clear Transition Indicator */
	PWM_ClearCaptureIntFlag(PWM1, PWM_CAPCTL_CFLIF_Msk | PWM_CAPCTL_CRLIF_Msk);

	/* Wait for at Least One Period between Rising and Falling */
	while(PWM_GetCaptureIntFlag(PWM1) <= 2);

	/* Clear Falling Transition Indicator */
	PWM_ClearCaptureIntFlag(PWM1, PWM_CAPCTL_CFLIF_Msk | PWM_CAPCTL_CRLIF_Msk);
	
	while(cnt < NUMS)
	{
		while((i = PWM_GetCaptureIntFlag(PWM1)) == 0);
		
    // Check the first Edge is Rising or Falling.
		// THe PWM counter would reload at the first Edge.
		if(i == 1)
			/* Get the PWM Counter on a Rising Edge of the Input Signal */
			u32Count[cnt++] = PWM_GET_CAPTURE_RISING_DATA(PWM1);
		else if(i == 2)
			/* Get the PWM Counter on a Falling Edge of the Input Signal */
			u32Count[cnt++] = PWM_GET_CAPTURE_FALLING_DATA(PWM1);

		// Wait for the second transition.
		while(PWM_GetCaptureIntFlag(PWM1) <= 2);
		
		if(i == 1)
			u32Count[cnt++] = PWM_GET_CAPTURE_FALLING_DATA(PWM1);
		else if(i == 2)
			u32Count[cnt++] = PWM_GET_CAPTURE_RISING_DATA(PWM1);
			
		/* Clear Transition Indicator */
		PWM_ClearCaptureIntFlag(PWM1, PWM_CAPCTL_CRLIF_Msk | PWM_CAPCTL_CFLIF_Msk);
	}
		
	if(i == 1)
	{
		u16RisingCount = u32Count[0];
		u16FallingCount = u32Count[1];
		// From Falling to next Rising.
		u16LowPeriod = u16FallingCount - u16RisingCount;
		// After first Rising reload to next Falling.
		u16HighPeriod = 0xFFFFFFFF - u16FallingCount;
		// After Rising reload to next Rising.
		u16TotalPeriod = 0xFFFFFFFF - u16RisingCount;
	}
	else if(i == 2)
	{
		u16FallingCount = u32Count[0];
		u16RisingCount = u32Count[1];
		// After first Falling reload to next Rising.
		u16LowPeriod = 0xFFFFFFFF - u16RisingCount;
		// From Rising to next Falling.
		u16HighPeriod = u16RisingCount - u16FallingCount;
		// After Falling reload to next Falling.
		u16TotalPeriod = 0xFFFFFFFF - u16FallingCount;
	}
		
	/* Show Numbers of Capture Data between Rising and Falling Down-Counter */
	for(i = 0; i < NUMS ; i++)
			printf("u32Count[%d]=%d\n", i, u32Count[i]);

	/* We can Apply Three Methods to Cerify Period by Compute
		 1. Total of u16HighPeriod and u16LowPeriod
		 2. The Difference of Falling Down-Counter N and N+1
		 3. The Difference of Rising Down-Counter N and N+1 */
	printf("\nCapture Result:\nRising Down-Counter = %d, Falling Down-Counter = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
				 u16RisingCount, u16FallingCount, u16HighPeriod, u16LowPeriod, u16TotalPeriod);
}

/* Main */
int main(void)
{
    uint8_t u8Item;
    bool exit = TRUE;

    SYS_Init();

    UART0_Init();

    do
    {
        printf("\n+------------------------------------------------------------------------+\n");
        printf("|                           PWM Driver Sample Code                       |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("|This Sample Code will Use PWM Capture Input Mode to Meassure PWM Output.|\n");
        printf("|We Set PWM0 CH2 with Frequency 100Hz and 25 Percent Duty and Enable PWM1|\n");
        printf("|Capture Input Mode.                                                     |\n");
        printf("|I/O Configuration:                                                      |\n");
        printf("|  Waveform Output Pin: PWM0_CH2(PC.4), Input Pin: PWM1 CRP1(PA.6)       |\n");
        printf("|[1] Capture the PWM0 CH2 Output Waveform by PWM1 CRP1 Input             |\n");
        printf("|[2] Capture the PWM0 CH2 Output(Invert) Waveform by PWM1 CRP1 Input     |\n");
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
        switch(u8Item)
        {
            case '1':
                /* Set PWM0 Channel 2 Output Configuration */
                PWM_ConfigOutputChannel(PWM0, 100, 0, 0, 25, 0);
                /* Enable PWM Output Generation of Selected Channel */
                PWM_EnableOutput(PWM0, 2);
                /* Disable the Waveform of PWM0 CH2 Invertion */
                PWM_DISABLE_OUTPUT_INVERTER(PWM0);
                /* Set PWM1 Capture Input Mode Configuration */
                PWM_ConfigCaptureChannel(PWM1, 0, 0);
                /* Enable Capture Input and Capture Channel Input Transition*/
                PWM_EnableCapture(PWM1);
                printf("\n\nPress any key to start PWM Capture Test\n");
                getchar();
                /* Enable PWM-Timer */
                PWM_Start(PWM0);
                PWM_Start(PWM1);
                /* Wait until PWM0 Timer Start to Count */
                while(PWM_GET_COUNTER(PWM0) == 0);
                /* Calculate and Check the Result whether It's Right */
                CalPWM();
                break;
            case '2':
                /* Set PWM0 Channel 2 Output Configuration */
                PWM_ConfigOutputChannel(PWM0, 100, 0, 0, 25, 0);
                /* Enable PWM Output Generation of Selected Channel */
                PWM_EnableOutput(PWM0, 2);
                /* Enable the Waveform of PWM0 CH2 Invertion */
                PWM_ENABLE_OUTPUT_INVERTER(PWM0);
                /* Set PWM1 Capture Input Mode Configuration */
                PWM_ConfigCaptureChannel(PWM1, 0, 0);
                /* Enable Capture Input and Capture Channel Input Transition*/
                PWM_EnableCapture(PWM1);
                printf("\n\nPress any key to start PWM Capture Test\n");
                getchar();
                /* Enable PWM-Timer */
                PWM_Start(PWM0);
                PWM_Start(PWM1);
                /* Wait until PWM0 Timer Start to Count */
                while(PWM_GET_COUNTER(PWM0) == 0);
                /* Calculate and Check the Result whether It's Right */
                CalPWM();
                break;
            case 'x':
                /* Exit the Demo */
                exit = FALSE;
                PWM_Stop(PWM0);
                PWM_Stop(PWM1);
                printf("\nDemo End!!\n");
                break;
            default:
                printf("\nPlease Input the Correct Number to Trigger Sample.\n");
                break;
        }
        /* To Conform with the Computed Result. We Set Positive and Negative Range at One */
        if(u8Item == '1')
        {
            if(((u16HighPeriod <= 13) && (u16HighPeriod >= 11)) && ((u16LowPeriod <= 39) && (u16LowPeriod >= 37)) && ((u16TotalPeriod <= 51) && (u16TotalPeriod >= 49)))
                printf("Capture Test Pass!!\n");
            else
                printf("Capture Test Fail!!\n");
        }
        else if(u8Item == '2')
        {
            if(((u16HighPeriod <= 39) && (u16HighPeriod >= 37)) && ((u16LowPeriod <= 13) && (u16LowPeriod >= 11)) && ((u16TotalPeriod <= 51) && (u16TotalPeriod >= 49)))
                printf("Capture Test Pass!!\n");
            else
                printf("Capture Test Fail!!\n");
        }
    }
    while(exit);
}
/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
