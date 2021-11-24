/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
#include <stdio.h>
#include "Platform.h"
#include "SPIFlash.h"

#define SPIFLASH_CLOCK (12000000)

// Define the delay time before write instruction of SPI flash
#define SPIFLASH_WRITE_DELAY_TIME     (5*1000) //us
//Define the LDO stable time
#define SPIFLASH_LDO_STABLE_TIME      (0.2*1000)//us

#define SPIFLASH_STABLE_DEAY_TIME     (SPIFLASH_WRITE_DELAY_TIME+SPIFLASH_LDO_STABLE_TIME)

#define SPIFLASH_MAX_VERIFY_COUNT     ((unsigned int)((SPIFLASH_STABLE_DEAY_TIME)/(((double)((32+8)*1000000))/(SPIFLASH_CLOCK))))

__align(4) UINT8 g_au8Buf[SPIFLASH_PAGE_SIZE];

void SPIFlashDemo(void);

S_SPIFLASH_HANDLER g_sSPIFlash;

int main()
{
	UINT32 u32Clock = 0;
	UINT32 u32JEDECID;
   	
	// ----------------------------------------------------------------------
	// Configure system clock
	// ----------------------------------------------------------------------
    // Unlock protected registers
    SYS_UnlockReg();
    // Enable clock source 
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_LIRCEN_Msk);
	// Set HIRC frequency = 48M
	CLK_SetHIRCFrequency(CLK_CLKSEL0_HIRCSEL_48M_VCC33);
	// Switch HCLK clock source to HXT 
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
	// Update System Core Clock
    SystemCoreClockUpdate();
	// Lock protected registers
	SYS_LockReg();	
	
	printf("\n\nCPU @ %dHz\n", SystemCoreClock);

	/* SPI0: PA12=MOSI0, PA13=CLK, PA14=MISO0, PD8= SS0*/
	SYS->GPA_MFP  = (SYS->GPA_MFP & ~(SYS_GPA_MFP_PA12MFP_Msk|SYS_GPA_MFP_PA13MFP_Msk|SYS_GPA_MFP_PA14MFP_Msk) ) |SYS_GPA_MFP_PA12MFP_SPI0_MOSI0|SYS_GPA_MFP_PA13MFP_SPI0_CLK|SYS_GPA_MFP_PA14MFP_SPI0_MISO0;
	SYS->GPD_MFP  = (SYS->GPD_MFP & ~SYS_GPD_MFP_PD8MFP_Msk ) |SYS_GPD_MFP_PD8MFP_SPI0_SS0;
	
	/* Reset IP module */
	CLK_EnableModuleClock(SPI0_MODULE);
	SYS_ResetModule(SPI0_RST);
	SPIFlash_Open(SPI0,SPI_SS0,SPIFLASH_CLOCK,&g_sSPIFlash);
	
	u32Clock = SPIFlash_GetSPIClock(&g_sSPIFlash);
	printf("SPIFlash run on actual clock: %d.\n", u32Clock);
    
	printf("Press enter to start SPI flash detection....\n");
	if ( getchar() != 0x0d )
		while(1);
	
	// Wait SPIFlash LDO & write stable.
	SPIFlash_WaitStable(&g_sSPIFlash, SPIFLASH_MAX_VERIFY_COUNT);

	SPIFlash_GetChipInfo(&g_sSPIFlash);

	if ( g_sSPIFlash.u32FlashSize == 0 )
	{
		printf("Can not find any SPI flash\n");
		while(1);
	}
	printf("\nFind a SPI flash with %d M-bit\n\n", g_sSPIFlash.u32FlashSize*8/1024/1024);

	if (g_sSPIFlash.u32FlashSize > 0x1000000) // > 128M-bits
	{
		printf("\nSPI flash is 4 byte address mode\n\n");
		SPIFlash_EN4BAddress(&g_sSPIFlash);
	}
	else
	{
		printf("\nSPI flash is 3 byte address mode\n\n");
		SPIFlash_EX4BAddress(&g_sSPIFlash);
	}
	
	SPIFlash_GlobalProtect(&g_sSPIFlash,FALSE);

	u32JEDECID = SPIFlash_GetJedecID(&g_sSPIFlash);
	
	printf("JEDEC ID = 0x%x\n", u32JEDECID);
	
	/* Enable QE bit in status register for quad mode*/	
#if ( SPIFLASH_INTERFACE_MODE == 1)	//dual mode
	printf("SPIFlash is at dual mode\n\n\r");
	SPIFlash_QuadMode_W25Q16DVS(&g_sSPIFlash, 0);
#elif ( SPIFLASH_INTERFACE_MODE == 2)	// quad mode 
	printf("SPIFlash is at quad mode\n\n\r");
	
	// Set quad mode by manufacturer device
	// The sample code demo winbond W25Q128JV SPIFlash
	SPIFlash_QuadMode_W25Q16DVS(&g_sSPIFlash, 1);

	// Change /HOLD and /WP pins to (MISO1) and (MOSI1)
	SYS->GPA_MFP |= (SYS_GPA_MFP_PA10MFP_SPI0_MISO1|SYS_GPA_MFP_PA11MFP_SPI0_MOSI1);
#endif
	  
	SPIFlashDemo();

	printf("\nSPI Flash erase/program/read sample end...\n");

	SPIFlash_Close(&g_sSPIFlash);
	
	CLK_DisableModuleClock(SPI0_MODULE);
	
	while(1);
}

#define START_BLOCK	(0)
#define END_BLOCK (START_BLOCK+32)
#define BLOCK_NUMBER 32

void SPIFlashDemo(void)
{
	UINT32 u32StartAddr, u32EndAdd, j;

	// ------------------------------------------------------------------------------------------------------
	printf("\tStart program ...\n\n");

	for(j = 0 ; j < BLOCK_NUMBER ; j++)
	{
		SPIFlash_Erase64K(&g_sSPIFlash, j);
		printf("\tErase %dth 64K block ...\n", j);
	}
	u32EndAdd =  64*1024*END_BLOCK;
	printf("\tWrite in by Page Program\n");    
	
	for( u32StartAddr = 64*1024*START_BLOCK; u32StartAddr < u32EndAdd; u32StartAddr += SPIFLASH_PAGE_SIZE)
	{
		SPIFlash_Read(&g_sSPIFlash, u32StartAddr, g_au8Buf, SPIFLASH_PAGE_SIZE);
		for( j = 0; j < SPIFLASH_PAGE_SIZE; j ++ )
		{
			if ( g_au8Buf[j] != 0xff )
			{
				printf("\t\tErase block failed!\n");
				while(1);
			}
		}

		for( j = 0; j < SPIFLASH_PAGE_SIZE; j ++ )
		{
			g_au8Buf[j] = j;
		}
        
		SPIFlash_Write(&g_sSPIFlash, u32StartAddr, g_au8Buf, SPIFLASH_PAGE_SIZE);
	}
	// Read each page and check the content	 
	printf("\tStart verify pages by Fast Read ...\n");

	for(u32StartAddr = 64*1024*START_BLOCK; u32StartAddr < u32EndAdd; u32StartAddr += SPIFLASH_PAGE_SIZE)
	{
		SPIFlash_Read(&g_sSPIFlash, u32StartAddr, g_au8Buf, SPIFLASH_PAGE_SIZE);
		for( j = 0; j < SPIFLASH_PAGE_SIZE; j ++ )
		{
			if ( g_au8Buf[j] != j )
			{
				printf("\t\tVerify failed in %dth element!\n", j);
				printf("data is %d\n",g_au8Buf[j]);
				while(1);
			}
		}
	}
}
