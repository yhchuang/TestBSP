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
extern uint32_t  LoaderImage1Base, LoaderImage1Limit;   /* symbol of image start and end */

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
static int  Load_Image_to_Flash(uint32_t image_base, uint32_t image_limit, uint32_t flash_addr, uint32_t max_size);

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
	uint8_t u8Item;
	uint32_t u32Data, au32Config[2];

	// Lock protected registers 
	if (SYS->REGLCTL == 1) // In end of main function, program issued CPU reset and write-protection will be disabled.
			SYS_LockReg();

	// Init System, IP clock and multi-function I/O 
	SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

	UART0_Init();
		
	printf("\n\n");
	printf("+-------------------------------------+\n");
	printf("|         FMC Sample Code             |\n");
	printf("|          [APROM code]               |\n");
	printf("+-------------------------------------+\n");
	printf(" Application code in APROM will load image to LDROM. \n");
	printf("\n\n");
	
	// Enable FMC ISP function 
	SYS_UnlockReg();
	FMC_Open();
	
	// Read Company ID 
	u32Data = FMC_ReadCID();
	if (u32Data != 0xda) {
			printf("Wrong CID: 0x%x\n", u32Data);
			goto lexit;
	}
		
	u32Data = FMC_ReadCID();
	printf(" Company ID ................................. [0x%08x]\n", u32Data);

	u32Data = FMC_ReadDID();
	printf(" Device ID .................................. [0x%08x]\n", u32Data);

	// Read Data Flash base address 
	u32Data = FMC_ReadDataFlashBaseAddr();
	printf("  Data Flash Base Address .................... [0x%08x]\n", u32Data);

	FMC_ENABLE_LD_UPDATE();    /* Enable LDROM update capability */
	/*
	 *  The binary image of LDROM code is embedded in this sample.
	 *  load_image_to_flash() will program this LDROM code to LDROM.
	 */
	if (Load_Image_to_Flash((uint32_t)&LoaderImage1Base, (uint32_t)&LoaderImage1Limit, FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
	{
		printf("Load image to LDROM failed!\n");
		goto lexit;            /* Load LDROM code failed. Program aborted. */
	}
	else
		printf("Load image to LDROM Success!\n");
	FMC_DISABLE_LD_UPDATE();   /* Disable LDROM update capability */
	SYS_LockReg();
	
	printf("\n\n");
	printf("+-------------------------------------+\n");
	printf("| User can select which way to boot from LDROM.\n");
	printf("| [0] Set 'Chip Boot Select' in user config[0] to boot from LDROM. \n");
	printf("| [1] Set 'Boot Select' in FMC_ISPCTL to boot from LDROM. \n");
	printf("+----------------------------------------+\n");
	printf("Please select...");
	u8Item = getchar();            /* block waiting to receive any one character from UART0 */
	printf("%c\n", u8Item);        /* print out the selected item */
	UART_WAIT_TX_EMPTY(UART0);
	
	switch(u8Item)
	{
		// This section will change the 'Chip Boot Select' in user config[0] to boot from LDROM.
		// First read the current configuration, and then change set the CBS[7].
		// Write the new configuration to register, and print out the result.
		// To boot from the LDROM, user could call "SYS_ResetChip()" or trigger the reset pin(RESET button).
		case '0':
		{
			SYS_UnlockReg();
			FMC_ReadConfig(au32Config, 2);
			printf("  User Config 0 .............................. [0x%08x]\n", au32Config[0]);
			printf("  User Config 1 .............................. [0x%08x]\n", au32Config[1]);
			
			au32Config[0] = au32Config[0] & (~BIT7);
			
			FMC_WriteConfig(au32Config, 2);
			
			FMC_ReadConfig(au32Config, 2);
			printf("  User Config 0 .............................. [0x%08x]\n", au32Config[0]);
			printf("  User Config 1 .............................. [0x%08x]\n", au32Config[1]);
			
			printf(" Enter any key to reset chip to boot from LDROM.\n");
			u8Item = getchar();            /* block waiting to receive any one character from UART0 */
			UART_WAIT_TX_EMPTY(UART0);
			SYS_ResetChip();
		}
		break;
		// This section will set the 'Boot Select' in user FMC_ISPCTL to boot from LDROM.
		// To boot from the LDROM, user should call "SYS_ResetCPU()" to reset CPU.
		case '1':
		{
			SYS_UnlockReg();
			/*  NOTE!
			 *     Before change VECMAP, user MUST disable all interrupts.
			 */
			NVIC->ICER[0] |= 0xFFFFFFFF;
			SYS_ClearResetSrc(SYS_RSTSTS_ALL);
			FMC_SET_LDROM_BOOT();
			
			printf(" Enter any key to reset chip to boot from LDROM.\n");
			u8Item = getchar();            /* block waiting to receive any one character from UART0 */
			UART_WAIT_TX_EMPTY(UART0);
			SYS_ResetCPU();
			
			while (1);
		}
		default:
		break;
	}
	
	lexit:
	
	// Disable FMC ISP function 
	FMC_Close();

	// Lock protected registers 
	SYS_LockReg();

	printf("\nFMC Sample Code Completed.\n");

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

/**
  * @brief    Load an image to specified flash address. The flash area must have been enabled by
  *           caller. For exmaple, if caller want to program an image to LDROM, FMC_ENABLE_LD_UPDATE()
  *           must be called prior to calling this function.
  * @return   Image is successfully programmed or not.
  * @retval   0   Success.
  * @retval   -1  Program/verify failed.
  */
static int  Load_Image_to_Flash(uint32_t image_base, uint32_t image_limit, uint32_t flash_addr, uint32_t max_size)
{
	uint32_t   i, j, u32Data, u32ImageSize, *pu32Loader;

	u32ImageSize = max_size;           /* Give the maximum size of programmable flash area. */

	printf("Program image to flash address 0x%x...", flash_addr);    /* information message */

	/*
	 * program the whole image to specified flash area
	 */
	pu32Loader = (uint32_t *)image_base;
	for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)  {

			FMC_Erase(flash_addr + i);     /* erase a flash page */
			for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4) {               /* program image to this flash page */
					FMC_Write(flash_addr + i + j, pu32Loader[(i + j) / 4]);
			}
	}
	printf("OK.\nVerify ...");

	/* Verify loader */
	for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE) {
			for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4) {
					u32Data = FMC_Read(flash_addr + i + j);        /* read a word from flash memory */

					if (u32Data != pu32Loader[(i+j)/4]) {          /* check if the word read from flash be matched with original image */
							printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", flash_addr + i + j, u32Data, pu32Loader[(i+j)/4]);
							return -1;             /* image program failed */
					}

					if (i + j >= u32ImageSize) /* check if it reach the end of image */
							break;
			}
	}
	printf("OK.\n");
	return 0;                          /* success */
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
