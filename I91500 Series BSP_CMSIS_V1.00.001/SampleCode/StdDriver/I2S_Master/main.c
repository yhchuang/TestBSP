/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * $Revision: 1 $
 * $Date: 20/11/24 10:04a $
 * @brief
 *           Demonstrate how I2S works in Master mode.
 *           This sample code needs to work with I2S_Slave.
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Platform.h"
#include "BUFCTRL.h"

#define DEMO_DATA_COUNT (256)

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/   
void UART0_Initiate(void);
void I2S0_Master_Initiate(S_BUFCTRL* psInBufCtrl,S_BUFCTRL* psOutBufCtrl);
void I2S0_Master_Start(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile S_BUFCTRL sInBufCtrl,sOutBufCtrl; // Buffer control handler.
int32_t ai32InBuf[DEMO_DATA_COUNT];        // Buffer array: provide I2S_Master receiver data. 
int32_t ai32OutBuf[DEMO_DATA_COUNT];       // Buffer array: provide I2S_Master send data. 

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
	UINT32 u32i;
	
    /* Unlock Protected Registers */
    SYS_UnlockReg();
    /* Enable Internal HIRC and LIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Switch HCLK Clock Source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
    /* Update System Core Clock */
    /* User can Use SystemCoreClockUpdate() to Calculate SystemCoreClock. */
    SystemCoreClockUpdate();
	
	// These defines are from  BUFCTRL.h for buffer control in this samle. 
	// Buffer control handler configuration. 
	BUFCTRL_CFG((&sInBufCtrl),ai32InBuf,sizeof(ai32InBuf)/sizeof(uint32_t));
	BUFCTRL_CFG((&sOutBufCtrl),ai32OutBuf,sizeof(ai32OutBuf)/sizeof(uint32_t));
	
	// Initiate UART0 for print message.
	UART0_Initiate();
	
	// Initiate I2S0 master
	I2S0_Master_Initiate((S_BUFCTRL*)(&sInBufCtrl),(S_BUFCTRL*)(&sOutBufCtrl));

	printf("+-----------------------------------------------------------+\n");
	printf("|            I2S Driver Sample Code (master mode)           |\n");
	printf("+-----------------------------------------------------------+\n");
	printf("I2S configuration:\n");
	printf("Sample rate 48 KHz\n");
	printf("Word width 32 bits\n");
	printf("Stereo mode\n");
	printf("I2S format\n");
	printf("TX value: 0x5A5A0000, 0x5A5A0001, ..., 0x5A5A00FF\n");
	printf("The I/O connection for I2S:\n");
	printf("Master PA2(BCLK) < - > Slave PA2(BCLK)\n");
	printf("Master PA3(DO) < - > Slave PA4(DI)\n");
	printf("Master PA4(DI) < - > Slave PA3(DO)\n");
	printf("Master PA1(LRCK) < - > Slave PA1(LRCK)\n");
	printf("Press any key to start ...");
	getchar();
	printf("\n");
	
	// Set send data into output buffer.
	for( u32i=0; u32i<DEMO_DATA_COUNT; u32i++ )
	{
		BUFCTRL_WRITE(((S_BUFCTRL*)(&sOutBufCtrl)),(0x5A5A0000+u32i));
		ai32InBuf[u32i] = 0xFFFFFFFF;
	}
	
	// Start send data from output buffer and receive data into input buffer.
	I2S0_Master_Start();
	
	// Wait until input buffer receive data counts = DEMO_DATA_COUNT.
	while(BUFCTRL_GET_COUNT(((&sInBufCtrl))) < DEMO_DATA_COUNT);
	
	// Verity receive data and compare with send data.
	for( u32i=0; u32i<DEMO_DATA_COUNT; u32i++ )
	{
		printf("Received Datap[%d]:  %x.\n", u32i, ai32InBuf[u32i]);
	}
	
	printf("Demo End...\n\n");
	while(1);
}

// I2S_Master ===================================================================================================
#define I2S_MASTER_PIN_MASK     (SYS_GPA_MFP_PA0MFP_Msk|SYS_GPA_MFP_PA1MFP_Msk|SYS_GPA_MFP_PA2MFP_Msk|SYS_GPA_MFP_PA3MFP_Msk|SYS_GPA_MFP_PA4MFP_Msk)
#define I2S_MASTER_PIN          (SYS_GPA_MFP_PA0MFP_I2S0_MCLK|SYS_GPA_MFP_PA1MFP_I2S0_LRCK|SYS_GPA_MFP_PA2MFP_I2S0_BCLK|SYS_GPA_MFP_PA3MFP_I2S0_DO|SYS_GPA_MFP_PA4MFP_I2S0_DI)
#define I2S_MASTER_SAMPLE_RATE  (48000)

// Provide I2S0 master's buffer control.
S_BUFCTRL* g_psI2S0_InBufCtrl = NULL;
S_BUFCTRL* g_psI2S0_OutBufCtrl = NULL;      

void I2S0_Master_Initiate(S_BUFCTRL* psInBufCtrl,S_BUFCTRL* psOutBufCtrl)
{
	// Set I2S MFP
	SYS->GPA_MFP = (SYS->GPA_MFP&~I2S_MASTER_PIN_MASK)|I2S_MASTER_PIN;
	
	// Enable I2S0 clock.
	CLK_EnableModuleClock(I2S0_MODULE);
	// Select I2S0 clock.
	CLK_SetModuleClock(I2S0_MODULE, CLK_CLKSEL1_I2S0SEL_HIRC, NULL);
	// I2S IPReset.
	SYS_ResetModule(I2S0_RST);	
	// Open I2S0 hardware IP
	I2S_Open(I2S0, I2S_MASTER, I2S_MASTER_SAMPLE_RATE, I2S_DATABIT_32, I2S_STEREO, I2S_FORMAT_I2S);
	// I2S0 Configuration
	I2S_SET_PCMSYNC(I2S0, I2S_PCMSYNC_BCLK);
	I2S_SET_MONO_RX_CHANNEL(I2S0, I2S_MONO_RX_RIGHT);
	I2S_SET_STEREOORDER(I2S0, I2S_ORDER_EVENLOW);
	// Set channel width.
	I2S_SET_CHWIDTH(I2S0, I2S_CHWIDTH_32);
	// Set FIFO threshold.
	I2S_SET_TXTH(I2S0, I2S_FIFO_TX_LEVEL_WORD_8);
	I2S_SET_RXTH(I2S0, I2S_FIFO_RX_LEVEL_WORD_8);
	// Enable interrupt.
	I2S_ENABLE_INT(I2S0, I2S_TXTH_INT_MASK|I2S_RXTH_INT_MASK);
	// Enable I2S's NVIC. 
	NVIC_EnableIRQ(I2S0_IRQn);
	// Set buffer control pointer.
	g_psI2S0_InBufCtrl = psInBufCtrl;
	g_psI2S0_OutBufCtrl = psOutBufCtrl;
}

void I2S0_Master_Start(void)
{
	UINT32 u32Data = 0;
	
	if( g_psI2S0_InBufCtrl != NULL )
	{
		// Clear TX, RX FIFO buffer 
		I2S_CLR_RX_FIFO(I2S0);
		// Enable Rx function
		I2S_ENABLE_RX(I2S0);	
	}
	if( g_psI2S0_OutBufCtrl != NULL )
	{
		// Clear TX FIFO buffer 
		I2S_CLR_TX_FIFO(I2S0);
		// Set send data into TX FIFO buffer;
		while( !I2S_GET_TX_IS_FULL(I2S0) && !BUFCTRL_IS_EMPTY(g_psI2S0_OutBufCtrl) )
		{
			BUFCTRL_READ(g_psI2S0_OutBufCtrl,&u32Data);
			I2S_WRITE_TX_FIFO(I2S0,u32Data);
		}
		// Enable Tx function.
		I2S_ENABLE_TX(I2S0);
	}
	if( g_psI2S0_InBufCtrl != NULL || g_psI2S0_OutBufCtrl != NULL )
	{
		I2S_ENABLE(I2S0);
	}
}

void I2S0_IRQHandler() 
{
	UINT32 u32Data, u32Count;

	if(I2S_GET_INT_FLAG(I2S0, I2S_TXTH_INT_FLAG))
	{
		// Write data process.
		if( g_psI2S0_OutBufCtrl != NULL )
		{
			// Max write data count per time.
			u32Count = 8; 		
			while( !I2S_GET_TX_IS_FULL(I2S0) && u32Count != 0 )
			{
				if(!BUFCTRL_IS_EMPTY(g_psI2S0_OutBufCtrl))
				{
					BUFCTRL_READ(g_psI2S0_OutBufCtrl,&u32Data);
					I2S_WRITE_TX_FIFO(I2S0,u32Data);
				}
				else
					I2S_WRITE_TX_FIFO(I2S0,0x00);
				
				u32Count--;
			}
		}
	}
	
	if(I2S_GET_INT_FLAG(I2S0, I2S_RXTH_INT_FLAG))
	{
		// Read data process.
		if( g_psI2S0_InBufCtrl != NULL )
		{
			// Max read data count per time.
			u32Count = 8; 
			while( !I2S_GET_RX_IS_EMPTY(I2S0) && u32Count != 0 )
			{
				// Read the data from I2S RXFIFO.
				u32Data = I2S_READ_RX_FIFO(I2S0);
				if( !BUFCTRL_IS_FULL(g_psI2S0_InBufCtrl) )
					BUFCTRL_WRITE(g_psI2S0_InBufCtrl,u32Data);
				u32Count--;	
			}
		}
	}
}

// UART0 ======================================================================================================== 
void UART0_Initiate(void)
{
	// Reset UART0 module
	SYS_ResetModule(UART0_RST);
    /* Enable UART Module Clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Configure UART0 and Set UART0 Baud Rate */
    UART_Open(UART0, 115200);
    /* Set GPD Multi-function Pins for UART0 TXD(PD.8) and RXD(PD.9) */
    SYS->GPD_MFP  = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD8MFP_Msk|SYS_GPD_MFP_PD9MFP_Msk) ) | (SYS_GPD_MFP_PD8MFP_UART0_TX|SYS_GPD_MFP_PD9MFP_UART0_RX);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
