/******************************************************************************
 * @file     LDROM_iap.c
 * @version  V1.00
 * @brief    FMC LDROM IAP sample program run on LDROM.
 *
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "stdio.h"
#include "Platform.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define global constants                                                                   			   */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/* Define global constants                                                                   			   */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
int32_t  flash_test(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern);

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
	uint8_t u8Item;
	uint32_t au32Config[2];

	// Lock protected registers 
	if (SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
			SYS_LockReg();

	// Init System, IP clock and multi-function I/O 
	SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

	UART0_Init();
		
	printf("\n\n");
	printf("+-------------------------------------+\n");
	printf("|       FMC IAP Sample Code           |\n");
	printf("|          [LDROM code]               |\n");
	printf("+-------------------------------------+\n");
	
	// Enable FMC ISP function 
	SYS_UnlockReg();
	FMC_Open();
	SYS_LockReg();
	
	printf("\n\n");
	printf("+-------------------------------------+\n");
	printf("| User can select which way to boot from APROM.\n");
	printf("| [0] Set user 'Chip Boot Select' in config[0] to boot from APROM. \n");
	printf("| [1] Set 'Boot Select' in FMC_ISPCTL to boot from APROM. \n");
	printf("+----------------------------------------+\n");
	printf("Please select...");
	u8Item = getchar();            /* block waiting to receive any one character from UART0 */
	printf("%c\n", u8Item);        /* print out the selected item */
	UART_WAIT_TX_EMPTY(UART0);
	
	switch(u8Item)
	{
		// This section will change the 'Chip Boot Select' in user config[0] to boot from APROM.
		// First read the current configuration, and then change set the CBS[7].
		// Write the new configuration to register, and print out the result.
		// To boot from the LDROM, user could call "SYS_ResetChip()" or trigger the reset pin(RESET button).
		case '0':
		{
			SYS_UnlockReg();
			FMC_ReadConfig(au32Config, 2);
			printf("  User Config 0 .............................. [0x%08x]\n", au32Config[0]);
			printf("  User Config 1 .............................. [0x%08x]\n", au32Config[1]);
			
			au32Config[0] = au32Config[0] | (BIT7);
			
			FMC_WriteConfig(au32Config, 2);
			
			FMC_ReadConfig(au32Config, 2);
			printf("  User Config 0 .............................. [0x%08x]\n", au32Config[0]);
			printf("  User Config 1 .............................. [0x%08x]\n", au32Config[1]);
			
			printf(" Enter any key to reset chip to boot from APROM.\n");
			u8Item = getchar();            /* block waiting to receive any one character from UART0 */
			UART_WAIT_TX_EMPTY(UART0);
			SYS_ResetChip();
		}
		break;
		case '1':
		{
			SYS_UnlockReg();
			/*  NOTE!
			 *     Before change VECMAP, user MUST disable all interrupts.
			 */
			NVIC->ICER[0] |= 0xFFFFFFFF;
			SYS_ClearResetSrc(SYS_RSTSTS_ALL);
			FMC_SET_APROM_BOOT();
			
			printf(" Enter any key to reset chip to boot from APROM.\n");
			u8Item = getchar();            /* block waiting to receive any one character from UART0 */
			UART_WAIT_TX_EMPTY(UART0);
			SYS_ResetCPU();
			
			while(1);
		}
		default:
		break;
	}
	
	// Disable FMC ISP function 
	FMC_Close();

	// Lock protected registers 
	SYS_LockReg();

	while (1);
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
	CLK_EnableModuleClock(ISP_MODULE);
	/* Update System Core Clock */
	/* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
	SystemCoreClockUpdate();
	/* Lock protected registers */
	SYS_LockReg();
}

void UART0_Init(void)
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
