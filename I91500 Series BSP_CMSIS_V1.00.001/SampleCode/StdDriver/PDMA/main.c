/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 1 $
 * $Date: 20/10/28 10:20a $
 * @brief    PDMA Driver Sample Code
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include <string.h>
#include "Platform.h"

#define DEMO_CH            (0)
#define DEMO_DATA_LENGTH   (64)

// Pre-declare function.
void System_Initiate(void);

uint32_t au32Source[DEMO_DATA_LENGTH];
uint32_t au32Destination[DEMO_DATA_LENGTH];
uint32_t volatile u32TransStatus = 0;

int main(void)
{
	uint32_t u32i;

	// Initiate system clock
	System_Initiate();		
	
	// Initiate variable.
	for(u32i=0; u32i<DEMO_DATA_LENGTH; u32i++)
	{
		au32Source[u32i]=u32i;
		au32Destination[u32i]=0;
	}
	u32TransStatus = 0;
	
	// Message : PDMA setup.
    printf("+--------------------------------------+ \n");
    printf("|   I91500 PDMA Driver Sample Code     | \n");
    printf("+--------------------------------------+ \n");
	printf("(1) Setup PDMA, includes.\n");
	printf("    1. Enable PDMA IP clock.\n");
	printf("    2. Open PDMA HW channel.\n");	
	printf("    3. Set transfer address, mode, counts.\n");
	printf("    4. Enable interrupt.\n");		
	printf("    5. Start PDMA to transfer data.\n");	
	
	// Initiate PDMA
	{
		// Enable IP clock
		CLK_EnableModuleClock(PDMA_MODULE);
		/* Reset PDMA IP */
		SYS_ResetModule(PDMA_RST);
		// Open PDMA HW channel.(enable HCLK_EN bit)
		PDMA_Open(1<<DEMO_CH);
		// Set transfer address.(source & destination)
		PDMA_SetTransferAddr(DEMO_CH, (uint32_t)au32Source, PDMA_SAR_INC, (uint32_t)au32Destination, PDMA_DAR_INC);
		// Set transfer mode.
		PDMA_SetTransferMode(DEMO_CH, PDMA_MEM);
		// Set transfer data length(data width & counts).
		PDMA_SetTransferCnt(DEMO_CH, PDMA_WIDTH_32, DEMO_DATA_LENGTH);
		// Enable interrupt.
		PDMA_EnableInt(DEMO_CH, PDMA_IER_BLKD_IE_Msk|PDMA_IER_TABORT_IE_Msk);
		NVIC_EnableIRQ(PDMA_IRQn);
		// Start PDMA to transfer data.
		PDMA_Trigger(DEMO_CH);			
	}
	
	// Message : PDMA process.
	printf("(2) Data transfering.\n");
	
	// Wait trans done status.
	while(u32TransStatus==0);

	// Process trans status.
	if( u32TransStatus&PDMA_ISR_TABORT_IF_Msk )
	{
		printf("(3) Data transfer fail.\n");
	}
	else if( u32TransStatus&PDMA_ISR_BLKD_IF_Msk )
	{
		if( memcmp(au32Source,au32Destination,sizeof(au32Source))==0 )
		{
			printf("(3) Data transfer success.\n");			
		}
		else
		{
			printf("(3) Data transfer finish but compare fail.\n");		
		}
	}

	PDMA_Close();
	while(1);
}

void PDMA_IRQHandler(void)
{
    if(PDMA_GET_INT_STATUS()&(1<<DEMO_CH)) 
    {
        u32TransStatus = PDMA_GET_CH_INT_STS(DEMO_CH);
        PDMA_CLR_CH_INT_FLAG(DEMO_CH, PDMA_ISR_BLKD_IF_Msk|PDMA_ISR_TABORT_IF_Msk);
    }
}

void System_Initiate(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg(); 
    /* Enable Internal HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    // Switch HCLK clock source to HXT
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
	// Update System Core Clock
	SystemCoreClockUpdate();
}
