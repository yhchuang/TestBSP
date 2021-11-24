/******************************************************************************
 * @file     main.c
 * @brief    Please refer readme.txt
 * @version  1.0.0
 * @date     2018/04/03
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Platform.h"
#include "printer_hidtrans.h"

// Define VCOM & HIDTrans variable.
#define HIDTRANS_BUFSIZE          (HIDTRANS_SECTORSIZE*8)

// Test demo buffer to upload/download through HID report
uint8_t   g_u8DemoBuffer[HIDTRANS_BUFSIZE] = {0};    

// Pre-declare function.
void Printer_And_HIDTrans_Initiate(void);
void HIRC_AutoTrim_Init(void);
void HIRC_AutoTrim_RefSof(void);

// main ========================================================================================================= 
int32_t main (void)
{

    
	// Initiate system clock
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

	// Initiate Printer, HID transfer and USBD hardware IP.
    Printer_And_HIDTrans_Initiate();
	
	// Process in interrupt
    while(1) 
	{

        
    };
}

// HID transfer command callback function ======================================================================= 
// Provide user to erase sector for processing.
void HIDTrans_EraseSector(uint32_t u32StartSector,uint32_t u32Sectors)
{
	memset(g_u8DemoBuffer+u32StartSector*HIDTRANS_SECTORSIZE, 0xFF, u32Sectors*HIDTRANS_SECTORSIZE);
}
// Provide user prepare read buffer for USB request.
void HIDTrans_PrepareReadPage(uint32_t* pu32Address,uint32_t u32StartPage,uint32_t u32Pages)
{	
	if( (u32Pages>0) && ((u32StartPage+u32Pages)<HIDTRANS_BUFSIZE/HIDTRANS_PAGESIZE) )
		*pu32Address = (uint32_t)g_u8DemoBuffer + u32StartPage*HIDTRANS_PAGESIZE;
	else
		*pu32Address = NULL;
}
// Provide user prepare write buffer for USB request.
void HIDTrans_PrepareWritePage(uint32_t* pu32Address,uint32_t u32StartPage,uint32_t u32Pages)
{
	if( (u32Pages>0) && ((u32StartPage+u32Pages)<HIDTRANS_BUFSIZE/HIDTRANS_PAGESIZE) )
		*pu32Address = (uint32_t)g_u8DemoBuffer + u32StartPage*HIDTRANS_PAGESIZE;
	else
		*pu32Address = NULL;
}
// Provide user get write data.
void HIDTrans_GetWriteData(uint32_t u32Address,uint32_t u32Pages)
{
}

// Micro printer callback function ============================================================================== 
// Provide user receive data from PC.
void Printer_ReceiveData(uint8_t* pu8DataBuf, uint32_t u32DataCount)
{
	// Different printers will receive different structures.
}

// Printer and HID transfer ===================================================================================== 
void Printer_And_HIDTrans_Initiate(void)
{
	// Reset module.
	SYS_ResetModule(USBD_RST);
	// Enable USBD module clock.
	CLK_EnableModuleClock(USBD_MODULE);
	// Set USBD clock divid
	CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL2_USBDSEL_HIRC, CLK_CLKDIV_USBD(1));
	// Initiate USBD hardware IP and input HID request for hand-shake.
	USBD_Open(&gsInfo, Printer_HIDTrans_ClassRequest, NULL);
	
	// Initiate endpoint configuration of VCOM & HID 
	Printer_HIDTrans_Init();
	// Enable USB IRQ
	NVIC_EnableIRQ(USBD_IRQn);
	// Start USBD for processing.
	USBD_Start();	
}
