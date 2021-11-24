/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 18/01/30 4:43p $
 * @brief    Please refer readme.txt
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h> 
#include "Platform.h"
#include "hid_trans.h"

#define DEMO_BUFSIZE     (HIDTRANS_SECTORSIZE*8)

// Test demo buffer to upload/download through HID report
uint8_t g_u8DemoBuffer[DEMO_BUFSIZE] = {0};    

// Pre-declare function.
void UART0_Initiate(void);
void HIRC_AutoTrim_Init(void);
void HIRC_AutoTrim_RefSof(void);

// HID transfer command function ================================================================================ 
void HIDTrans_EraseSector(uint32_t u32StartSector,uint32_t u32Sectors)
{
	printf("  >> Get erase secore request -\r\n");
	printf("     - Start sector number : %d\r\n", u32StartSector);
	printf("     - Erase sector counts : %d\r\n", u32Sectors);
	
	memset(g_u8DemoBuffer+u32StartSector*HIDTRANS_SECTORSIZE, 0xFF, u32Sectors*HIDTRANS_SECTORSIZE);
}
// Provide user prepare read buffer for USB request.
void HIDTrans_PrepareReadPage(uint32_t* pu32Address,uint32_t u32StartPage,uint32_t u32Pages)
{	
	printf("  >> Get read page request -\r\n");
	printf("     - Start page number : %d\r\n", u32StartPage);
	printf("     - Read page counts  : %d\r\n", u32Pages);
	
	if( (u32Pages>0) && ((u32StartPage+u32Pages)<DEMO_BUFSIZE/HIDTRANS_PAGESIZE) )
		*pu32Address = (uint32_t)g_u8DemoBuffer + u32StartPage*HIDTRANS_PAGESIZE;
	else
		*pu32Address = NULL;
}
// Provide user prepare write buffer for USB request.
void HIDTrans_PrepareWritePage(uint32_t* pu32Address,uint32_t u32StartPage,uint32_t u32Pages)
{
	printf("  >> Get write page request -\r\n");
	printf("     - Start page number : %d\r\n", u32StartPage);
	printf("     - Write page counts : %d\r\n", u32Pages);
	
	if( (u32Pages>0) && ((u32StartPage+u32Pages)<DEMO_BUFSIZE/HIDTRANS_PAGESIZE) )
		*pu32Address = (uint32_t)g_u8DemoBuffer + u32StartPage*HIDTRANS_PAGESIZE;
	else
		*pu32Address = NULL;
}
// Provide user get write data.
void HIDTrans_GetWriteData(uint32_t u32Address,uint32_t u32Pages)
{
	printf("  >> Get write data finish message.\r\n");
	printf("     - Write page counts : %d\r\n", u32Pages);	
}

// main ========================================================================================================= 
int main(void)
{	
	// Initiate system clock(Configure in ConfigSysClk.h)
    /* Unlock Protected Registers */
    SYS_UnlockReg();
    /* Enable Internal HIRC and LIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
	// Set HIRC frequency = 48M
	CLK_SetHIRCFrequency(CLK_CLKSEL0_HIRCSEL_48M_VCC33);
    /* Switch HCLK Clock Source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
    /* Update System Core Clock */
    /* User can Use SystemCoreClockUpdate() to Calculate SystemCoreClock. */
    SystemCoreClockUpdate();
	
	// Initiate UART0 for printf
	UART0_Initiate();

	printf("\r\n+-------------------------------------------------+\r\n");
	printf("|          HID Transfer Demo Sample Code          |\r\n");
	printf("+-------------------------------------------------+\r\n");
	
	// Initiate HID Transfer(include USBD hardware IP)
	{
		// Reset module.
		SYS_ResetModule(USBD_RST);
		// Enable USBD module clock.
		CLK_EnableModuleClock(USBD_MODULE);
		// Set USBD clock divid
		CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL2_USBDSEL_HIRC, CLK_CLKDIV_USBD(1));
		// Initiate USBD hardware IP and input HID request for hand-shake.
		USBD_Open(&gsInfo, HIDTrans_ClassRequest, NULL);
		
		// Initiate HID for endpoint configuration 
		HIDTrans_Initiate();
		// Enable USB IRQ
		NVIC_EnableIRQ(USBD_IRQn);
		// Start USBD for processing.
		USBD_Start();
	}
	// Process in iterrupt.
	while(1);
}


// UART0 ======================================================================================================== 
#define UART0_BAUDRATE     (115200)

void UART0_Initiate(void)
{
    // Enable UART module clock 
    CLK_EnableModuleClock(UART0_MODULE);
	// Configure UART0 and set UART0 Baud rate
    UART_Open(UART0, UART0_BAUDRATE);
    /* Set GPD Multi-function Pins for UART0 TXD(PD.8) and RXD(PD.9) */
    SYS->GPD_MFP  = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD8MFP_Msk|SYS_GPD_MFP_PD9MFP_Msk) ) | (SYS_GPD_MFP_PD8MFP_UART0_TX|SYS_GPD_MFP_PD9MFP_UART0_RX);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
