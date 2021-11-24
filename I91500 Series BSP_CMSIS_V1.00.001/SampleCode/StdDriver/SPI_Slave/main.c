/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/08/17 10:00p $
 * @brief    I91500 SPI slave demo sample source.
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Platform.h"

#define TEST_COUNT 			(16)	    /* Test data count */

__align(4) uint32_t g_au32SourceData[TEST_COUNT];
__align(4) uint32_t g_au32DestinationData[TEST_COUNT];

uint32_t g_u32TxDataCount;
uint32_t g_u32RxDataCount;

// SPI Slave =========================================================================
#define SPI_SLAVE_PINS_MSK    (SYS_GPA_MFP_PA1MFP_Msk|SYS_GPA_MFP_PA2MFP_Msk|SYS_GPA_MFP_PA3MFP_Msk|SYS_GPA_MFP_PA4MFP_Msk)
#define SPI_SLAVE_PINS        (SYS_GPA_MFP_PA1MFP_SPI_MOSI0|SYS_GPA_MFP_PA2MFP_SPI_SCLK|SYS_GPA_MFP_PA3MFP_SPI_SSB|SYS_GPA_MFP_PA4MFP_SPI_MISO0)	

void SYS_Init(void);
void SPI_Init(void);
void SPI_IRQHandler(void);

int main()
{
	uint32_t u32Counter = 0;
	
	/* Initiate SPI */
	SPI_Init();

	printf("\r\n+------------------------------------------------------------------------+\r\n");
	printf("|                      SPI Driver Sample Code                           |\r\n");
	printf("+------------------------------------------------------------------------+\r\n");
	printf("Configure SPI as a slave.\r\n");
	
	printf("PA.12=SPI0_MOSI0, PA.13=SPI0_CLK, PA.14=SPI0_MISO0, PA.15=SPI0_SS0.\r\n");
	printf("SPI controller will transfer %d data to a off-chip master device.\r\n", TEST_COUNT);
	printf("In the meanwhile the SPI controller will receive %d data from the off-chip master device.\r\n", TEST_COUNT);
	printf("After the transfer is done, the %d received data will be printed out.\r\n", TEST_COUNT);
	printf("The SPI slave configuration is ready.\r\n");

	for(u32Counter=0; u32Counter<TEST_COUNT; u32Counter++)
	{
			g_au32SourceData[u32Counter] = (0xAA55AA00 + u32Counter);
			g_au32DestinationData[u32Counter] = 0;
	}

	// Wait user press key on EVB board.
	printf("<<Press 'Enter' for SPI slave start to listen>>\r\n");
	getchar();
	printf("<<Slave start to listern>>\r\n");

	g_u32TxDataCount = 0;
	g_u32RxDataCount = 0;
	
	SPI_EnableInt(SPI0, SPI_UNITIEN_MASK);
	NVIC_EnableIRQ(SPI0_IRQn);
	
	SPI_TRIGGER(SPI0);
	SPI_WRITE_TX(SPI0, g_au32SourceData[g_u32TxDataCount] );
	
	/* Wait for transfer done */
	while(g_u32RxDataCount < TEST_COUNT)
	{ 
		// To DO
	}
	
	SPI_DisableInt(SPI0, SPI_UNITIEN_MASK); 
	
	printf("Received data:\r\n");
	for(u32Counter=0; u32Counter<TEST_COUNT; u32Counter++)
		printf("%d:\t0x%X\r\n", u32Counter, g_au32DestinationData[u32Counter]);

	printf("\r\nThe data transfer was done.\r\n"); 
	printf("Exit SPI0 driver slave sample code.\r\n");
	while(1);
}

void SYS_Init(void)
{
    // Unlock protected registers
    SYS_UnlockReg();
    // Enable clock source 
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_LIRCEN_Msk);
	/* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
	// Update System Core Clock
    SystemCoreClockUpdate();
	// Lock protected registers
	SYS_LockReg();	
}

void SPI_Init()
{
	/* Enable module clock */
	CLK_EnableModuleClock(SPI0_MODULE);

	/* Reset SPI module */
	SYS_ResetModule(SPI0_RST);

	/* Configure SPI as a slave, MSB first, SPI Mode-0 timing, clock is from master provide */
	SPI_Open(SPI0, SPI_SLAVE, SPI_MODE_0, 32, 4000000 );

	SPI_SET_SUSPEND_CYCLE(SPI0, 0xf);

	/* Disable the automatic hardware slave select function */
	SPI_DisableAutoSS(SPI0);
	
	SPI_DISABLE_BYTE_REORDER(SPI0);
	
	SPI_ClearRxFIFO(SPI0);
	SPI_ClearTxFIFO(SPI0);

	// Init I/O multi-function ; 
    // PA10=SPI0_MISO1, PA11=SPI0_MOSI1, PA12=SPI0_MOSI0, PA13=SPI0_CLK, PA14=SPI0_MISO0, PA15=SPI0_SS0
    SYS->GPA_MFP &= 0x000FFFFF;
    SYS->GPA_MFP |= 0x55500000;	
}

void SPI0_IRQHandler(void)
{
		SPI_CLR_UNIT_TRANS_INT_FLAG(SPI0);
	
    if( ((SPI_GET_STATUS(SPI0) & SPI_STATUS_TXFULL_Msk)==0) && (g_u32TxDataCount<TEST_COUNT) ) 
		{SPI_WRITE_TX(SPI0, g_au32SourceData[g_u32TxDataCount++]);}	
		
		
    if((SPI_GET_STATUS(SPI0) & SPI_STATUS_RXEMPTY_Msk)==0) 
		{g_au32DestinationData[g_u32RxDataCount++] = SPI_READ_RX(SPI0);}
	

    
		if ((g_u32RxDataCount>=TEST_COUNT))
		{   /* Disable Tx FIFO threshold interrupt and RX FIFO time-out interrupt */
        SPI_DisableInt(SPI0, SPI_UNITIEN_MASK);
 	      NVIC_DisableIRQ(SPI0_IRQn);
		}
}
