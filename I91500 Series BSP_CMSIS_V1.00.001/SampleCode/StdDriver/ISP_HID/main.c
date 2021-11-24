/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to transfer data between USB device and PC through USB HID interface.
 *           A windows tool is also included in this sample code to connect with USB device.
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "Platform.h"
#include "targetdev.h"

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg(); 
    /* Enable Internal HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Enable External HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk); 
    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC,  CLK_CLKDIV_HCLK(1));		
    // Enable PDMA clock.
    CLK_EnableModuleClock(PDMA_MODULE);
    /* Reset PDMA IP */
	SYS_ResetModule(PDMA_RST);
    /* Disable PDMA IRQ */
    NVIC_DisableIRQ(PDMA_IRQn);    
}

// Initiate AUDIO (include USBD hardware IP)
void USBD_Init(void)
{         
	// Reset USBD module.
    SYS_ResetModule(USBD_RST);  
    // Enable USBD module clock 
    CLK_EnableModuleClock(USBD_MODULE);       
    // Set USBD clock divid 
    CLK_SetModuleClock(USBD_MODULE,CLK_CLKSEL2_USBDSEL_HIRC,CLK_CLKDIV_USBD(1));		
    // Initiate USBD hardware IP and input HID request for hand-shake.
    /* Open USB controller */
    USBD_Open(&gsInfo, HIDTrans_ClassRequest, NULL);
    /*Init Endpoint configuration for HID */
    HID_Init();
    // Enable USB IRQ
    NVIC_EnableIRQ(USBD_IRQn);
    /* Start USB device */
    USBD_Start();
}

volatile uint32_t isp_delay=0x1ffff;
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
	uint8_t *ptr;
    
  /* Unlock write-protected registers */
	SYS_UnlockReg();
	/* Init system and multi-funcition I/O */
	SYS_Init();
	FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;	// (1ul << 0)
	GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
	USBD_Init();
	
	GPIO_SetMode(PD,BIT0,GPIO_MODE_PULLUP);			//set PD0 as input with pullup for DetectPin
	
	while (DetectPin == 0)
	{

		while (DetectPin == 0)
		{
			if (bUsbDataReady == TRUE) 
			{
				ParseCmd((uint8_t *)usb_rcvbuf, 64);
				ptr = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));
				/* Prepare the data for next HID IN transfer */
				USBD_MemCopy(ptr, response_buff, EP2_MAX_PKT_SIZE);
				USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);
				bUsbDataReady = FALSE;
			}
		}

		goto _APROM;
   }

		/* Waiting for down-count to zero */
		while(isp_delay-- >0);

_APROM:
	outpw(&SYS->RSTSTS, 3);//clear bit
	outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
	outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

	/* Trap the CPU */
	while (1);
}
/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
