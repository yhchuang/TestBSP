/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief
 *       Show Deep Sleep Mode and Wake-up Methods.
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "Platform.h"

/* Use SW Flags to Replace with Print in IRQHandler */
bool GPIO_Trigger = FALSE;
bool WDT_Trigger = FALSE;

void SYS_Initiate(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable External OSC49M */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Enable External OSC10K */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
    /* Update System Core Clock */
    SystemCoreClockUpdate();		
    /* Lock protected registers */
    SYS_LockReg();
}

void GPIO_Initiate(void)
{
    /* Unlock protected registers */
	SYS_UnlockReg();

	printf("Set un-used GPIO input pull up to prevent leakage current(Depend on HW PCB design)\n"); 
	// Set all gpio pin input with pull-up
	PA->MODE = 0xFFFFFFFF;    
	PB->MODE = 0xF;     
	PC->MODE = 0xFFFFFFFF; 
	PD->MODE = 0xFFFFFFFF;

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

void ShowCurrentWakeupStatus(void)
{
    uint32_t reset_type = 0;
    reset_type = SYS_GetResetSrc();
    printf("\nShow Current Wakeup Status\n");
    printf("\n--------------------------\n");
    if(reset_type & SYS_RSTSTS_PORWK)
        printf("Wakeup from DPD From POR!!\n");
    if(reset_type & SYS_RSTSTS_TIMWK)
        printf("Wakeup from DPD From TIMER!!\n");
    if(reset_type & SYS_RSTSTS_PINWK)
        printf("Wakeup from DPD From PIN!!\n");
    if(reset_type & SYS_RSTSTS_PMURSTF)
        printf("Reset Source From PMU!!\n");
    if(reset_type & SYS_RSTSTS_BODF)
        printf("BOD Reset Flag!!\n");
    if(reset_type & SYS_RSTSTS_LVRF)
        printf("LVR Reset Flag!!\n");
    if(reset_type & SYS_RSTSTS_WDTRF)
        printf("Reset Source From WDT!!\n");
    if(reset_type & SYS_RSTSTS_PINRF)
        printf("Reset Pin Reset Flag!!\n");
    if(reset_type & SYS_RSTSTS_PORF)
        printf("POR Reset Flag!!\n");
    if(GPIO_Trigger)
    {
        printf("Wakeup from GPIO Trigger!!\n");
        GPIO_Trigger = FALSE;
    }
    if(WDT_Trigger)
    {
        printf("Wakeup from WDT Trigger!!\n");
        WDT_Trigger = FALSE;
    }
    printf("--------------------------\n");

    SYS_ClearResetSrc(SYS_RSTSTS_ALL);
}

void GetLastPowerState(void)
{
    if(CLK_GET_POWERDOWNFLAG(CLK, CLK_PFLAG_STOPF_Msk)) // 0x3
        printf("The Last Power State is in Stop Mode\n");
    else if (CLK_GET_POWERDOWNFLAG(CLK, CLK_PFLAG_DSF_Msk)) //0x1
        printf("The Last Power State is in DEEP SLEEP Mode\n");
    else
        printf("The Current Power State is in Normal Mode\n");

    CLK_CLEAR_POWERDOWNFLAG(CLK, CLK_PFLAG_STOPF_Msk | CLK_PFLAG_DSF_Msk);
}

void GPA_IRQHandler(void)
{
    /* Get GPIO Pin Interrupt Flag */
    NVIC_ClearPendingIRQ(GPA_IRQn);
	GPIO_CLR_INT_FLAG(PA,BIT15);
    GPIO_Trigger = TRUE;
}

/* Main */
int main(void)
{
    SYS_Initiate();

    UART0_Init();

    do
    {
        printf("\n+-----------------------------------------------------------------------------+\n");
        printf("|                  PMU Demo Sample(Deep Sleep Mode)                           |\n");
        printf("+-----------------------------------------------------------------------------+\n");
		
        /* Get Last Power State from Power Down Flag Register */
        /* Clear Power Down Flag Register before the Following Tests */
        GetLastPowerState();
		
        /* Show All Kinds of Flags for System Reset Source */
        /* Clear All System Reset Flags to Observe Clearly Flags for Next Time */
        ShowCurrentWakeupStatus();

        /* Disable Sample Related Interrupts To Demonstrate Individual Test Items Every Round */
        GPIO_DisableInt(PA,15);
        WDT_DisableInt();
		
		printf("+-----------------------------------------------------------------------------+\n");
        printf("| Please press 'Enter' into 'Deep Sleep mode' and wake-up from GPIO Pin.      |\n");
        printf("+-----------------------------------------------------------------------------+\n");
        getchar();
		GPIO_Initiate();
		/* Configure PA.15 as input pullup Mode and Enable Interrupt by Falling Edge Trigger */
		GPIO_SetMode(PA, BIT15, GPIO_MODE_PULLUP);
		GPIO_EnableInt(PA, 15, GPIO_INT_FALLING);
		NVIC_EnableIRQ(GPA_IRQn);

		printf("Enter to Deep Sleep Mode......\n");
		/* Wait Specified Uart Port Transmission is Over */
		UART_WAIT_TX_EMPTY(UART0);
		/* Disable LDO */
		CLK_POWERDOWN_LDO(CLK);
		/* Enter to Deep Sleep Mode */
		CLK_DeepSleep();
		__WFI();
    }
    while(1);
}
/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
