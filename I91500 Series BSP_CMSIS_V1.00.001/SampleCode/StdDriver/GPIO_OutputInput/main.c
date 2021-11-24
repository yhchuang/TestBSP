/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/10/28 10:20a $
 * @brief    General Purpose I/O Driver Sample Code
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "Platform.h"

uint32_t loopcount=0;
uint32_t Count=0;

void GPA_IRQHandler(void)
{
    /* To check if interrupt occurred */
    if (PA->INTSRC & BIT9)
	{
        PA->INTSRC = BIT9;
        printf("PA9 INT occurred. %d  %d\n",Count, loopcount);
    }
	Count++;
}

void GPC_IRQHandler(void)
{
    /* To check if interrupt occurred */
	if (PC->INTSRC & BIT10)
	{
		PC->INTSRC = BIT10;
		printf("PC10 INT occurred. %d  %d\n",Count, loopcount);
	}
	Count++;
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

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    int32_t i32Err;
	
    /* Lock protected registers */
    if(SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
        SYS_LockReg();

		/* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();
		
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------------+ \n");
    printf("|           GPIO Driver Sample Code   | \n");
    printf("+-------------------------------------+ \n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin                              */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect PA.9 and PC.10 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
 
    /* Configure PA.9 as Output mode and PC.10 as Input mode then close it */
	SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA9MFP_Msk) ) | SYS_GPA_MFP_PA9MFP_GPIO  ;
	SYS->GPC_MFP  = (SYS->GPC_MFP & (~SYS_GPC_MFP_PC10MFP_Msk) ) | SYS_GPC_MFP_PC10MFP_GPIO  ;
    
	GPIO_SetMode(PA, BIT9, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT10, GPIO_MODE_INPUT);
		
    i32Err = 0;
    printf("  GPIO Output/Input test ...... \n");

    GPIO_SET_OUT_DATA(PA, (GPIO_GET_OUT_DATA(PA)&~BIT9) ); // PA9=0
	if ((GPIO_GET_IN_DATA(PC)&BIT10)!=0) {
        i32Err = 1;
    }

	GPIO_SET_OUT_DATA(PA, (GPIO_GET_OUT_DATA(PA)|BIT9) );  // PA9=1
    if ((GPIO_GET_IN_DATA(PC)&BIT10)!=BIT10) {
        i32Err = 1;
    }

    if ( i32Err ) 
        printf("  [FAIL] --- Please make sure PA.9 and PC.10 are connected. \n");
    else
        printf("  [OK] \n");


    /* Configure PA.5 and PC.10 to default Pull_Up mode */
    GPIO_SetMode(PA, BIT9, GPIO_MODE_PULLUP);
    GPIO_SetMode(PC, BIT10, GPIO_MODE_PULLUP);
	
    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("\n  PA9, PC10 are used to test interrupt\n");

    /* Configure PA9 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PA, BIT9, GPIO_MODE_INPUT);
    GPIO_EnableInt(PA, 9, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPA_IRQn);
	
    /*  Configure PC10 as Quasi-bi-direction mode and enable interrupt by both rising and falling edge trigger */
    GPIO_SetMode(PC, BIT10, GPIO_MODE_PULLUP);
	GPIO_EnableInt(PC, 10, GPIO_INT_BOTH_EDGE);	
    NVIC_EnableIRQ(GPC_IRQn);

    /* Waiting for interrupts */
    while (1)
	{
		GPIO_SET_OUT_DATA(PC, ~((loopcount>>12)&0x20));
		loopcount++;
	}
}
