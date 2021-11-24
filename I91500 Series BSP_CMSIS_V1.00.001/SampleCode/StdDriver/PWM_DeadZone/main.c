/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief
 *       PWM DeadZone Generator Driver Sample Code
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "Platform.h"

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

    /* Select PWM Module Clock Source */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL1_PWM0SEL_HIRC, 0);

    /* Reset PWM0 Channel 0~3 */
    SYS_ResetModule(PWM0_RST);

    /* Set GPC Multi-function Pins for PWM0 CH0(PC.2) and CH1(PC.3) */
    SYS->GPC_MFP  = (SYS->GPC_MFP & ~(SYS_GPC_MFP_PC2MFP_Msk|SYS_GPC_MFP_PC3MFP_Msk) ) | (SYS_GPC_MFP_PC2MFP_PWM00|SYS_GPC_MFP_PC3MFP_PWM01);

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

/* Main */
int main(void)
{
    uint8_t u8Item;
    bool exit = TRUE;

    SYS_Init();

    UART0_Init();

    while(exit==TRUE)
    {
        printf("\n+------------------------------------------------------------------------+\n");
        printf("|                           PWM Driver Sample Code                       |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("|This Sample Code will Output PWM Signals between Channel 0~1 with 100Hz |\n");
        printf("|Frequency and 50 Percent Duty, and Enable DeadZone Function.            |\n");
        printf("|I/O configuration:                                                      |\n");
        printf("|  waveform output pin: PWM0_CH0(PC.2), PWM0_CH1(PC.3)                   |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("|[1] PWM DeadZone Generator with Specified Interval(0x1)                 |\n");
        printf("|[2] PWM DeadZone Generator with Specified Interval(0x5)                 |\n");
        printf("|[x] Exit the Demo                                                       |\n");
        printf("+------------------------------------------------------------------------+\n");
        printf("Please Input the Correct Number to Trigger Sample.\n");

        u8Item = getchar();
        switch(u8Item)
        {
            case '1':
                /* Set PWM Channel 0 Output Configuration */
                PWM_ConfigOutputChannel(PWM0, 100, 50, 0, 0, 0);
                /* Enable DeadZone Function with Specified Interval */
                PWM_EnableDeadZone(PWM0, 0, 0x01);
                /* Enable PWM Output Generation of Selected Channel */
                PWM_EnableOutput(PWM0, 0);
                PWM_EnableOutput(PWM0, 1);
                /* Enable PWM-Timer */
                PWM_Start(PWM0);
                break;
            case '2':
                /* Set PWM Channel 0 Output Configuration */
                PWM_ConfigOutputChannel(PWM0, 100, 50, 0, 0, 0);
                /* Enable DeadZone Function with Specified Interval */
                PWM_EnableDeadZone(PWM0, 0, 0x05);
                /* Enable PWM Output Generation of Selected Channel */
                PWM_EnableOutput(PWM0, 0);
                PWM_EnableOutput(PWM0, 1);
                /* Enable PWM-Timer */
                PWM_Start(PWM0);
                break;
            case 'x':
                /* Exit the Demo */
                exit = FALSE;
                printf("\nDemo End!!\n");
                break;
            default:
                printf("\nPlease Input the Correct Number to Trigger Sample.\n");
                break;
        }
    }
}
/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
