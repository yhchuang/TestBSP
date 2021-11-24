/**************************************************************************//**
 * @file     usbd_audio.c
 * @brief    ISD series USBD driver Sample file
 * @version  1.0.0
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <stdio.h>
#include <string.h>
#include "Platform.h"
#include "usbd_audio.h"
#include "peripheral.h"
#include "hirc_trim.h"
#include "audio_stream.h"

//----------------------------------------------------------------------------
//  Global variables for Control Pipe
//---------------------------------------------------------------------------- 

// PlayBack Buffer Control Variable
volatile uint8_t g_u8PlaybackState = 0;
volatile uint8_t g_au8PlaybackRingBuff[(PLAY_RATE_MAX/1000)*RING_BUFF_LEVEL*PLAY_CHANNELS*PLAY_RES_BYTE];
volatile uint16_t g_u16PlayBack_Read_Ptr;
volatile uint16_t g_u16PlayBack_Write_Ptr;
volatile uint16_t g_u16PlaybackRingBuffLen; 
volatile uint16_t g_u16PlaybackRingBuffUpper; 
volatile uint16_t g_u16PlaybackRingBuffLower; 

// Record Buffer Control Variable
volatile uint8_t g_u8RecordState = 0;
volatile uint8_t g_au8RecordRingBuff[(PLAY_RATE_MAX/1000)*RING_BUFF_LEVEL*REC_CHANNELS*REC_RES_BYTE];
volatile uint16_t g_u16Record_Read_Ptr;
volatile uint16_t g_u16Record_Write_Ptr;
volatile uint16_t g_u16RecordRingBuffLen; 
volatile uint16_t g_u16RecordRingBuffUpper; 
volatile uint16_t g_u16RecordRingBuffLower; 

// Audio Parameter
volatile uint32_t g_usbd_PlaySampleRate = PLAY_RATE_MAX;
volatile uint8_t g_usbd_PlayMute      = 0x00;       /* Play MUTE control. 0 = normal. 1 = MUTE */
volatile int16_t g_usbd_PlayVolumeL   = 0xF400;     /* -12 dB, Play left channel volume */
volatile int16_t g_usbd_PlayVolumeR   = 0xF400;     /* -12 dB, Play right channel volume */
volatile int16_t g_usbd_PlayMaxVolume = 0x0000;     /* 0 dB, Play left and right channel maximum volume */
volatile int16_t g_usbd_PlayMinVolume = 0xCE00;     /* -50 dB, Play left and right channel minimum volume*/
volatile int16_t g_usbd_PlayResVolume = 0x100;      /* 1 dB, Play left and right channel volume resolution */

volatile uint32_t g_usbd_RecSampleRate = REC_RATE_MAX;
volatile uint8_t g_usbd_RecMute       = 0x00;       /* Record MUTE control. 0 = normal. 1 = MUTE */
volatile int16_t g_usbd_RecVolumeL    = 0x1800;     /* 24 dB, Record left channel volume */
volatile int16_t g_usbd_RecVolumeR    = 0x1800;     /* 24 dB, Record right channel volume */
volatile int16_t g_usbd_RecMaxVolume  = 0x1E00;     /* 30 dB Record left and right channel maximum volume */
volatile int16_t g_usbd_RecMinVolume  = 0x0000;     /* 0 dB Record left and right channel minimum volume */
volatile int16_t g_usbd_RecResVolume  = 0x600;      /* 6dB, Record left and right channel volume resolution  */

volatile uint8_t g_u8USBPlayEn = 0;
volatile uint8_t g_u8USBRecEn = 0;

volatile uint8_t g_u8NoUSBPlaybackCount = 0;


/*--------------------------------------------------------------------------*/
/**
 * @brief       Playback EP detection
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to detect usb playback EP
 */
void USBPlaybackPauseDetect(void)
{
    if(g_u8NoUSBPlaybackCount > 2)
    {
        g_u8PlaybackState = UAC_PLAYBACK_START;
        g_u8USBPlayEn = 0;    
        g_u8NoUSBPlaybackCount = 0;    
        return;        
    }
    g_u8NoUSBPlaybackCount++;
}

/*--------------------------------------------------------------------------*/
/**
 * @brief       Playback EP counter reset
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to reset playback EP counter
 */
void USBPlaybackPauseDetectReset(void)
{
    g_u8NoUSBPlaybackCount = 0;
}


/*--------------------------------------------------------------------------*/
/**
 * @brief       Playback ring buffer initiate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to set playback ring buffer threshold and pointer
 */
void PlaybackBuffInit(void)
{
    bUnderFlow = FALSE;
    bOverFlow = FALSE;    
    g_u16PlayBack_Read_Ptr = 0;
    g_u16PlayBack_Write_Ptr = 0;
    g_u16PlaybackRingBuffLen = (g_usbd_PlaySampleRate / 1000) * RING_BUFF_LEVEL;
    g_u16PlaybackRingBuffUpper = (g_u16PlaybackRingBuffLen >> 1) + (g_usbd_PlaySampleRate / 1000);
    g_u16PlaybackRingBuffLower = (g_u16PlaybackRingBuffLen >> 1) - (g_usbd_PlaySampleRate / 1000);
}

/*--------------------------------------------------------------------------*/
/**
 * @brief       Record ring buffer initiate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to set record ring buffer threshold and pointer
 */
void RecordBuffInit(void)
{
    g_u16Record_Read_Ptr = 0;
    g_u16Record_Write_Ptr = 0;
    g_u16RecordRingBuffLen = (g_usbd_RecSampleRate / 1000) * RING_BUFF_LEVEL;
    g_u16RecordRingBuffUpper = (g_u16RecordRingBuffLen >> 1) + (g_usbd_RecSampleRate / 1000);
    g_u16RecordRingBuffLower = (g_u16RecordRingBuffLen >> 1) - (g_usbd_RecSampleRate / 1000);
}

/*--------------------------------------------------------------------------*/
/**
 * @brief       USBD Interrupt Service Routine
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is the USBD ISR
 */
void USBD_IRQHandler(void)
{
    uint8_t u8Buf[8];
    uint32_t u32IntSts = USBD_GET_INT_FLAG();		// Read USBD_INTSTS register 
    uint32_t u32State = USBD_GET_BUS_STATE();		// reset, suspend, resume, timeout flags 

    if (u32IntSts & USBD_INTSTS_FLDET) 
	{
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED()) 
		{
            // USB Plug In 
            USBD_ENABLE_USB();
        } 
		else 
		{
            // USB Un-plug 
            USBD_DISABLE_USB();
        }
    }

    if ( u32IntSts & USBD_INTSTS_SOFIF_Msk )
    {              
        USBD_CLR_INT_FLAG(USBD_INTSTS_SOF);
        HIRC_AutoTrim_Enable(((CLK->CLKSEL0 & CLK_CLKSEL0_OSCFSEL_Msk) == CLK_CLKSEL0_HIRCSEL_48M_VCC33)?SYS_IRCTCTL_FREQSEL_48M:SYS_IRCTCTL_FREQSEL_49M, (uint16_t *)&g_u16TrimedValue);    
    }
    
    if (u32IntSts & USBD_INTSTS_BUS) 
	{
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);
        HIRC_AutoTrim_Reset();
        if (u32State & USBD_STATE_USBRST) 
		{
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
        }
        if (u32State & USBD_STATE_SUSPEND) 
		{
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }
        if (u32State & USBD_STATE_RESUME) 
		{
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
        }
    }

    if (u32IntSts & USBD_INTSTS_USB) 
	{
        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP) 
		{	
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();				
        }

        // EP events
        if (u32IntSts & USBD_INTSTS_EP0) 
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);
            // control IN
            USBD_CtrlIn();
        }
        
        if (u32IntSts & USBD_INTSTS_EP1) 
        {     
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);
            // control OUT
            USBD_CtrlOut();
            
            USBD_GetSetupPacket(u8Buf);
            
            if(u8Buf[0] == 0x22)
            {
                if (u8Buf[3] == SAMPLING_FREQ_CONTROL && (u8Buf[4] == (EP2 | EP_INPUT)))
                {
                    g_u8USBRecEn = 1; 
                    g_u8RecordDeviceEn = 0;
                    RecordBuffInit();                    
                }
            }     
        }     	

        if (u32IntSts & USBD_INTSTS_EP2) 
		{
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            // Interrupt IN
            EP2_Handler();
        }
        
        if (u32IntSts & USBD_INTSTS_EP3) 
		{
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
			EP3_Handler();
        }
        
        if (u32IntSts & USBD_INTSTS_EP4) 
		{
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
#if __FEEDBACK__
			/* Isochronous IN for Feedback */
            EP4_Handler();
#endif
        }	
    }	
}

/**
 * @brief       EP2 Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EP2 event
 */
void EP2_Handler(void)  /* Interrupt handler */
{
    uint32_t i, j, k;
	uint16_t u16distance;
	uint8_t *pu8buf;
	uint16_t u16EP2_Sample_Count;
    
	if (g_u8RecordState == UAC_RECORD_BUSY)
	{
		if ( g_u16Record_Write_Ptr >= g_u16Record_Read_Ptr ) 
		{
			u16distance = g_u16Record_Write_Ptr - g_u16Record_Read_Ptr;	
		}
		else 
		{   
			u16distance = (g_u16RecordRingBuffLen - g_u16Record_Read_Ptr) + g_u16Record_Write_Ptr;	
		}
		
		if ( u16distance >= g_u16RecordRingBuffUpper) 
		{
			u16EP2_Sample_Count = ((g_usbd_RecSampleRate/1000) + 1);		
		}
		else if ( u16distance <= g_u16RecordRingBuffLower ) 
		{
			u16EP2_Sample_Count = ((g_usbd_RecSampleRate/1000) - 1);		
		}
		else 
		{
			u16EP2_Sample_Count = (g_usbd_RecSampleRate/1000);
		}
              
		pu8buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2));

		for(i=0; i<u16EP2_Sample_Count; i++)
		{           
            for(k=0; k<REC_CHANNELS; k++)                       
                for(j=0; j<REC_RES_BYTE; j++)      
                    pu8buf[i*REC_CHANNELS*REC_RES_BYTE+k*REC_RES_BYTE+j] = g_au8RecordRingBuff[g_u16Record_Read_Ptr*REC_CHANNELS*REC_RES_BYTE+k*REC_RES_BYTE+j];
 
            g_u16Record_Read_Ptr++;	
			
			if (g_u16Record_Read_Ptr >= g_u16RecordRingBuffLen)       
			{
				g_u16Record_Read_Ptr = 0;
			}         
		}
		USBD_SET_PAYLOAD_LEN(EP2, (u16EP2_Sample_Count*REC_CHANNELS*REC_RES_BYTE));
	}
	else
	{	
        if(g_u16Record_Write_Ptr >= (g_u16RecordRingBuffLen >> 1))
        {
            g_u8RecordState = UAC_RECORD_BUSY; 
        }    
		USBD_SET_PAYLOAD_LEN(EP2, 0);
	}
}

#if __FEEDBACK__ 
#else

/**
 * @brief       HIRC manual trim
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to trim HIRC according to USB playback EP and PDMA IRQ
 */
void HIRC_ManualTrim_Enable(void)
{
    uint16_t u16distance;
    
    if(g_u8USBPlayEn)
    {
        if ( g_u16PlayBack_Write_Ptr >= g_u16PlayBack_Read_Ptr )
        {
            u16distance = g_u16PlayBack_Write_Ptr - g_u16PlayBack_Read_Ptr;
        }
        else
        {
            u16distance = (g_u16PlaybackRingBuffLen - g_u16PlayBack_Read_Ptr) + g_u16PlayBack_Write_Ptr;	
        }        
        if(u16distance >= g_u16PlaybackRingBuffUpper)
        {
            if(g_u8NoUSBPlaybackCount < 1)
            {   
                SYS->OSCTRIM += 1;
            }    
        }
        else if(u16distance <= g_u16PlaybackRingBuffLower)
        {
            if(g_u8NoUSBPlaybackCount > 1)
            {   
                SYS->OSCTRIM -= 1;
            }
        }      
        g_u8NoUSBPlaybackCount = 0;   
    }  
}

/*
volatile uint32_t u32CheckBufferTimerCount = 0;

void CheckBufferDataCount(void)
{
	uint16_t u16distance;
	uint16_t u32TrimTemp;
	
	if (g_u16PlayBack_Write_Ptr >= g_u16PlayBack_Read_Ptr)
	{
		if(g_u16PlayBack_Write_Ptr == g_u16PlayBack_Read_Ptr)
		{
			if (bOverFlow == TRUE)
				u16distance = g_u16PlaybackRingBuffLen;
			else
				u16distance =0;
		}
		else
			u16distance = g_u16PlayBack_Write_Ptr - g_u16PlayBack_Read_Ptr;
	}
	else
	{
		u16distance = (g_u16PlaybackRingBuffLen - g_u16PlayBack_Read_Ptr) + g_u16PlayBack_Write_Ptr;	
	}

	if ( u16distance >= g_u16PlaybackRingBuffUpper ) 
	{
        u32TrimTemp = g_u16TrimedValue +1;
	}
	else
	{
        if ( u16distance <= g_u16PlaybackRingBuffLower )
            u32TrimTemp = g_u16TrimedValue -1;
        else
            u32TrimTemp = g_u16TrimedValue;
	}

	SYS->OSCTRIM = (SYS->OSCTRIM & ~SYS_OSCTRIM_TRIM_Msk) | (u32TrimTemp << SYS_OSCTRIM_TRIM_Pos);	
}
*/
#endif

/**
 * @brief       EP3 Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EP3 event
 */
void EP3_Handler(void)  /* Interrupt handler */
{
    uint32_t i, j;
    uint16_t u16PayLoadLen;
#if ((PLAY_RES_MAX == RES_32) || (PLAY_RES_MAX == RES_24))
    uint32_t *puRingBuff = (uint32_t *)&g_au8PlaybackRingBuff[0];
    uint32_t *pubuf = (uint32_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));      
#elif (PLAY_RES_MAX == RES_16)
    uint16_t *puRingBuff = (uint16_t *)&g_au8PlaybackRingBuff[0];
    uint16_t *pubuf = (uint16_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));  
#else
    uint8_t *puRingBuff = (uint8_t *)&g_au8PlaybackRingBuff[0];
    uint8_t *pubuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));   
#endif
     
    u16PayLoadLen = USBD_GET_PAYLOAD_LEN(EP3);  //Total EP3 byte data for copy    
	if (g_u8PlaybackState == UAC_PLAYBACK_BUSY)
	{
#if __FEEDBACK__ 
#else  
/*        
        // Chaeck RingBuffer data count for HIRC adjustment every 5 times EP3
        u32CheckBufferTimerCount++;
        if (u32CheckBufferTimerCount == 5)
        {
            CheckBufferDataCount();
            u32CheckBufferTimerCount = 0;
        }
*/
        HIRC_ManualTrim_Enable();
#endif                  
        for (i=0; i<(u16PayLoadLen/PLAY_RES_BYTE/PLAY_CHANNELS); i++)
        {
            if((g_u16PlayBack_Read_Ptr == g_u16PlayBack_Write_Ptr) && (bUnderFlow == FALSE))
            {
                bOverFlow = TRUE;
            }
            else
            {
                bUnderFlow = FALSE;
                for(j=0; j<PLAY_CHANNELS; j++)
                    puRingBuff[g_u16PlayBack_Write_Ptr*PLAY_CHANNELS+j] = pubuf[i*PLAY_CHANNELS+j];
                g_u16PlayBack_Write_Ptr++;
        
                if( g_u16PlayBack_Write_Ptr >= g_u16PlaybackRingBuffLen)
                {
                    g_u16PlayBack_Write_Ptr = 0;
                }
            }
        }        
        if (g_u16PlayBack_Read_Ptr == g_u16PlayBack_Write_Ptr)
            bOverFlow = TRUE;
   
        if(!g_u8USBPlayEn)
        {
            if(g_u16PlayBack_Write_Ptr >= (g_u16PlaybackRingBuffLen >> 1))
            {
                g_u8USBPlayEn = 1;  
            }        
        }    
	}
    else if (g_u8PlaybackState == UAC_PLAYBACK_START)
	{
#if __FEEDBACK__
        uint8_t *pu8buf;
        /* Get the address in USB buffer */
        pu8buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4));
        
        /* Prepare the data to USB IN buffer */
        *pu8buf++ = 0x00;
        *pu8buf++ = ((g_usbd_PlaySampleRate/1000) & 0x3) << 6;
        *pu8buf = ((g_usbd_PlaySampleRate/1000) & 0xFC) >> 2;
        /* Trigger ISO IN */
        USBD_SET_PAYLOAD_LEN(EP4, 3); 
#endif        
		
        g_u8PlaybackState = UAC_PLAYBACK_BUSY;
        PlaybackBuffInit();   
		
        for (i=0; i<(u16PayLoadLen/PLAY_RES_BYTE/PLAY_CHANNELS); i++) 
        {
            for(j=0; j<PLAY_CHANNELS; j++)
                puRingBuff[g_u16PlayBack_Write_Ptr*PLAY_CHANNELS+j] = pubuf[i*PLAY_CHANNELS+j];
            g_u16PlayBack_Write_Ptr++;
        }
	}
    USBPlaybackPauseDetectReset();
	USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
}	

#if __FEEDBACK__
/**
 * @brief       EP4 Handler (Iso IN feedback interrupt handler)
 *
 * @param[in]   None
 *
 * @return      None
 *
 */
void EP4_Handler(void)  /* Interrupt handler */
{
    uint8_t *pu8buf;
    uint16_t u16distance;
    uint32_t u32SampleRate;
	
	if ( g_u16PlayBack_Write_Ptr >= g_u16PlayBack_Read_Ptr )
	{
        u16distance = g_u16PlayBack_Write_Ptr - g_u16PlayBack_Read_Ptr;	
	}
	else
	{
        u16distance = (g_u16PlaybackRingBuffLen - g_u16PlayBack_Read_Ptr) + g_u16PlayBack_Write_Ptr;		
	}

	if ( u16distance >= g_u16PlaybackRingBuffUpper ) 
	{
		u32SampleRate = (g_usbd_PlaySampleRate / 1000) - 1; 	
	}
	else if ( u16distance <= g_u16PlaybackRingBuffLower ) 
	{
        u32SampleRate = (g_usbd_PlaySampleRate / 1000) + 1; 
	}			
	else
	{
        u32SampleRate = (g_usbd_PlaySampleRate / 1000); 
	}
    /* Get the address in USB buffer */
	pu8buf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4));
    /* Prepare the data to USB IN buffer */
    *pu8buf++ = 0x00;
    *pu8buf++ = (u32SampleRate & 0x3) << 6;
    *pu8buf = (u32SampleRate & 0xFC) >> 2;
    /* Trigger ISO IN */
    USBD_SET_PAYLOAD_LEN(EP4, 3);    
}

#endif

/*--------------------------------------------------------------------------*/
/**
 * @brief       UAC Class Initial
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure endpoints for UAC class
 */
void UAC_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer range for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

	
    /*****************************************************/
    /* EP2 ==> Iso IN endpoint, address 2 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | USBD_CFG_TYPE_ISO | EP2);
    /* Buffer range for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Iso Out endpoint, address 3 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | USBD_CFG_TYPE_ISO | EP3);
    /* Buffer range for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);

    /* trigger to receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
	
    /* EP4 ==> Interrupt IN endpoint number 4 */
	USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | USBD_CFG_TYPE_ISO | EP4);	
    /* Buffer range for EP4 */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);
		
    /* EP5 ==> Interrupt IN endpoint number 5 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_IN | EP5);
    /* Buffer range for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);
}

/**
 * @brief       UAC class request
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process UAC class requests
 */
void UAC_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);
    
    /* request to endpoint */
    if((buf[0] & 0x1F) == 0x02) 
	{ 
        /* device to host */
		if(buf[0] & 0x80)
		{
			switch(buf[1])
			{
                case UAC_GET_CUR:
                {
                    if(buf[3]==SAMPLING_FREQ_CONTROL && (buf[4]==EP3))
					{
                        M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 0) = (uint8_t)g_usbd_PlaySampleRate;
                        M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = (uint8_t)(g_usbd_PlaySampleRate >> 8);
                        M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 2) = (uint8_t)(g_usbd_PlaySampleRate >> 16);     
                        /* Data stage */
                        USBD_SET_DATA1(EP0);
                        USBD_SET_PAYLOAD_LEN(EP0, 3);
                        break;
                    }
                    else if(buf[3]==SAMPLING_FREQ_CONTROL && (buf[4]==(EP2 | EP_INPUT)))
					{
                        M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 0) = (uint8_t)g_usbd_RecSampleRate;
                        M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = (uint8_t)(g_usbd_RecSampleRate >> 8);
                        M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 2) = (uint8_t)(g_usbd_RecSampleRate >> 16);     
                        /* Data stage */
                        USBD_SET_DATA1(EP0);
                        USBD_SET_PAYLOAD_LEN(EP0, 3);
                        break;
                    }
					else
					{
                        /* STALL control pipe */
                        USBD_SetStall(0);
                    }
                    // Trigger next Control Out DATA1 Transaction.
                    /* Status stage */
                    USBD_PrepareCtrlOut(0,0);
                    break;
                }
								 
                default:
                {
                    /* STALL control pipe */
                    USBD_SetStall(0);
                    break;
                }                
            }             
        }
        /* host to device */
		else    
		{
            switch(buf[1])
			{
                case UAC_SET_CUR:
                {   
                    if(buf[3] == SAMPLING_FREQ_CONTROL && (buf[4] == EP3))
					{    
                        USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlaySampleRate, buf[6]);										                            
                        /* Status stage */
                        USBD_SET_DATA1(EP0);
                        USBD_SET_PAYLOAD_LEN(EP0, 0);
                        break;
                    }
                    else if (buf[3] == SAMPLING_FREQ_CONTROL && (buf[4] == (EP2 | EP_INPUT)))
                    {
                        USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecSampleRate, buf[6]);
                        /* Status stage */
                        USBD_SET_DATA1(EP0);
                        USBD_SET_PAYLOAD_LEN(EP0, 0);  
                        break;
                    }                    
					else
					{
                        /* STALL control pipe */
                        USBD_SetStall(0);
                        break;
                    }
                }
                default:
                {
                    /* Setup error, stall the device */
                    USBD_SetStall(0);
                    break;
                }    
             }
        }      
    }
    /* request to interface */
	else
	{ 
        /* device to host */
        if (buf[0] & 0x80)  
        { 
            switch(buf[1])
            {
                case UAC_GET_CUR:
                {
                    switch(buf[3])
                    {
                        case MUTE_CONTROL:
                        {
                            if(REC_FEATURE_UNITID == buf[5])
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecMute;
                            else if(PLAY_FEATURE_UNITID == buf[5])
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayMute;
                            /* Data stage */ 
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 1);
                            break;
                        }
                        
                        case VOLUME_CONTROL:
                        {                               
                            if(REC_FEATURE_UNITID == buf[5])
                            {
                                /* Left or right channel */ 
                                if(buf[2] == 1)
                                {
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecVolumeL;
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecVolumeL >> 8;
                                }
                                else
                                {
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecVolumeR;
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecVolumeR >> 8;
                                }

                            }
                            else if(PLAY_FEATURE_UNITID == buf[5])
                            {
                                /* Left or right channel */
                                if(buf[2] == 1)
                                {
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayVolumeL;
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayVolumeL >> 8;
                                }
                                else
                                {
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayVolumeR;
                                    M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayVolumeR >> 8;
                                }
                            }
                            /* Data stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 2);
                            break;
                        }
                        default:
                        {
                            /* Setup error, stall the device */
                            USBD_SetStall(0);
                        }
                    }
                    // Trigger next Control Out DATA1 Transaction.
                    /* Status stage */
                    USBD_PrepareCtrlOut(0, 0);
                    break;
                }

                case UAC_GET_MIN:
                {
                    switch(buf[3])
                    {
                        case VOLUME_CONTROL:
                        {
                            if(REC_FEATURE_UNITID == buf[5])
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecMinVolume;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecMinVolume >> 8;
                            }
                            else if(PLAY_FEATURE_UNITID == buf[5])
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayMinVolume;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayMinVolume >> 8;
                            }
                            /* Data stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 2);
                            break;
                        }
                        default:
                            /* STALL control pipe */
                            USBD_SetStall(0);
                    }
                    // Trigger next Control Out DATA1 Transaction.
                    /* Status stage */
                    USBD_PrepareCtrlOut(0, 0);
                    break;
                }

                case UAC_GET_MAX:
                {
                    switch(buf[3])
                    {
                        case VOLUME_CONTROL:
                        {
                            if(REC_FEATURE_UNITID == buf[5])
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecMaxVolume;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecMaxVolume >> 8;
                            }
                            else if(PLAY_FEATURE_UNITID == buf[5])
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayMaxVolume;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayMaxVolume >> 8;
                            }
                            /* Data stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 2);
                            break;
                        }
                        default:
                            /* STALL control pipe */
                            USBD_SetStall(0);
                    }
                    // Trigger next Control Out DATA1 Transaction.
                    /* Status stage */ 
                    USBD_PrepareCtrlOut(0, 0);
                    break;
                }

                case UAC_GET_RES:
                {
                    switch(buf[3])
                    {
                        case VOLUME_CONTROL:
                        {
                            if(REC_FEATURE_UNITID == buf[5])
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_RecResVolume;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_RecResVolume >> 8;
                            }
                            else if(PLAY_FEATURE_UNITID == buf[5])
                            {
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)) = g_usbd_PlayResVolume;
                                M8(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0) + 1) = g_usbd_PlayResVolume >> 8;
                            }
                            /* Data stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 2);
                            break;
                        }
                        default:
                            /* STALL control pipe */
                            USBD_SetStall(0);
                    }
                    // Trigger next Control Out DATA1 Transaction.
                    /* Status stage */
                    USBD_PrepareCtrlOut(0, 0);
                    break;
                }
								

                default:
                {
                    // Setup error, stall the device 
                    USBD_SetStall(0);
                }
            }
        }
        /* host to device */
        else
        {   
            switch(buf[1])
            {
                case UAC_SET_CUR:
                {
                    switch(buf[3])
                    {
                        case MUTE_CONTROL:
                            if(REC_FEATURE_UNITID == buf[5])
                                USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecMute, buf[6]);
                            else if(PLAY_FEATURE_UNITID == buf[5])
                            {
                                USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlayMute, buf[6]);
                            }
                            /* Status stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 0);
                            break;

                        case VOLUME_CONTROL:
                            if(REC_FEATURE_UNITID == buf[5])
                            {
                                if(buf[2] == 1)
                                {
                                    /* Prepare the buffer for new record volume of left channel */
                                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecVolumeL, buf[6]);
                                }
                                else
                                {
                                    /* Prepare the buffer for new record volume of right channel */
                                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_RecVolumeR, buf[6]);
                                }
                            }
                            else if(PLAY_FEATURE_UNITID == buf[5])
                            {
                                if(buf[2] == 1)
                                {
                                    /* Prepare the buffer for new play volume of left channel */
                                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlayVolumeL, buf[6]);
                                }
                                else
                                {
                                    /* Prepare the buffer for new play volume of right channel */
                                    USBD_PrepareCtrlOut((uint8_t *)&g_usbd_PlayVolumeR, buf[6]);
                                }
                            }
                            /* Status stage */
                            USBD_SET_DATA1(EP0);
                            USBD_SET_PAYLOAD_LEN(EP0, 0);
                            break;

                        default:
                            /* STALL control pipe */
                            USBD_SetStall(0);
                            break;
                    }
                    break;
                }
								
                // HID 
                case SET_REPORT:
                {
                    if(buf[3] == 2)
                    {
                        // Request Type = Output 
                        USBD_SET_DATA1(EP1);
                        USBD_SET_PAYLOAD_LEN(EP1, buf[6]);
                        // Status stage 
                        USBD_PrepareCtrlIn(0, 0);
                    }
                    break;
                }
                case SET_IDLE:
                {
                    /* Status stage */
                    USBD_SET_DATA1(EP0);
                    USBD_SET_PAYLOAD_LEN(EP0, 0);
                    break;
                }
                case SET_PROTOCOL:

                default:
                {
                    /* Setup error, stall the device */
                    USBD_SetStall(0);
                    break;
                }
            }
        }
    }
}


/**
 * @brief       Set Interface standard request
 *
 * @param[in]   u32AltInterface Interface
 *
 * @return      None
 *
 * @details     This function is used to set UAC Class relative setting
 */
void UAC_SetInterface(void)			
{
    uint8_t buf[8];
    uint8_t u32AltInterface;
    uint8_t g_usbd_UsbInterface;

    USBD_GetSetupPacket(buf);

    u32AltInterface = buf[2];
    g_usbd_UsbInterface = buf[4];

    /* Record */	
	if ( g_usbd_UsbInterface == 1 )		 
	{
        USBD_SET_DATA1(EP2);
        USBD_SET_PAYLOAD_LEN(EP2, 0);           
		if (u32AltInterface == 1)
        {			
            g_u8RecordState = UAC_RECORD_START;		
            g_u8USBRecEn = 1;      
            RecordBuffInit();        
        }
        else
		{   
            g_u8RecordState = UAC_RECORD_STOP;
            g_u8USBRecEn = 0;               
		}
	}
    /* Playback */
	else if ( g_usbd_UsbInterface == 2 )
	{
		if (u32AltInterface == 1)
        {
            g_u8PlaybackState = UAC_PLAYBACK_START;        
        }
		else
        {       
            g_u8PlaybackState = UAC_PLAYBACK_STOP;
            g_u8USBPlayEn = 0;
        }
	}
}
