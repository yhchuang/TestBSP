/**************************************************************************//**
 * @file     audio_stream.c
 * @brief    Audio stream process sample file
 * @version  1.0.0
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <string.h> 
#include "Platform.h"
#include "usbd_audio.h"
#include "audio_stream.h"
#include "peripheral.h"

//----------------------------------------------------------------------------
//  Global variables for Control Pipe
//---------------------------------------------------------------------------- 

volatile BOOL bUnderFlow = FALSE;   //No RingBuffer for reading
volatile BOOL bOverFlow = FALSE;    //No RingBuffer for Writting


void PlaybackPingPongBuffProcess(uint8_t u8Index)
{
    uint32_t i, j;
#if ((PLAY_RES_MAX == RES_32) || (PLAY_RES_MAX == RES_24))
    uint32_t *puPingpongBuff = (uint32_t *)&g_au8PlaybackPingpongBuff[0];
    uint32_t *puRingBuff = (uint32_t *)&g_au8PlaybackRingBuff[0];          
#elif (PLAY_RES_MAX == RES_16)
    uint16_t *puPingpongBuff = (uint16_t *)&g_au8PlaybackPingpongBuff[0];
    uint16_t *puRingBuff = (uint16_t *)&g_au8PlaybackRingBuff[0];
#else
    uint8_t *puPingpongBuff = (uint8_t *)&g_au8PlaybackPingpongBuff[0];
    uint8_t *puRingBuff = (uint8_t *)&g_au8PlaybackRingBuff[0];            
#endif
    for (i=0; i<(g_usbd_PlaySampleRate/1000); i++)
    {
        // When song played to end or pause, there will be no EP3 anymore so Ringbuffer underflow
        // If keep last data, there will be a fixed DC on speaker, long time DC possibly makes speaker hot	
        if ((g_u16PlayBack_Read_Ptr == g_u16PlayBack_Write_Ptr) && (bOverFlow == FALSE))                
        {
            for(j=0; j<PLAY_CHANNELS; j++)
                puPingpongBuff[(((g_usbd_PlaySampleRate/1000)*PLAY_CHANNELS)*u8Index)+(i*PLAY_CHANNELS+j)] = 0;
        }
        else
        {
            bOverFlow = FALSE;
            if (g_usbd_PlayMute) 
            {
                for(j=0; j<PLAY_CHANNELS; j++)
                    puPingpongBuff[(((g_usbd_PlaySampleRate/1000)*PLAY_CHANNELS)*u8Index)+(i*PLAY_CHANNELS+j)] = 0;
                g_u16PlayBack_Read_Ptr++;  
            }
            else 
            {
                for(j=0; j<PLAY_CHANNELS; j++)
                    puPingpongBuff[(((g_usbd_PlaySampleRate/1000)*PLAY_CHANNELS)*u8Index)+(i*PLAY_CHANNELS+j)] = puRingBuff[g_u16PlayBack_Read_Ptr*PLAY_CHANNELS+j];
                g_u16PlayBack_Read_Ptr++;      
            }   
        }                
                      
        if ( g_u16PlayBack_Read_Ptr >= g_u16PlaybackRingBuffLen)                
        {
            g_u16PlayBack_Read_Ptr = 0;                                  
        }
    }
    if (g_u16PlayBack_Read_Ptr == g_u16PlayBack_Write_Ptr)
        bUnderFlow = TRUE;
}

void RecordPingPongBuffProcess(uint8_t u8Index)
{
    uint32_t i, j;
#if ((PLAY_RES_MAX == RES_32) || (PLAY_RES_MAX == RES_24))
    uint32_t *puPingpongBuff = (uint32_t *)&g_au8RecordPingpongBuff[0];
    uint32_t *puRingBuff = (uint32_t *)&g_au8RecordRingBuff[0];     
#elif (PLAY_RES_MAX == RES_16)
    uint16_t *puPingpongBuff = (uint16_t *)&g_au8RecordPingpongBuff[0];
    uint16_t *puRingBuff = (uint16_t *)&g_au8RecordRingBuff[0];
#else
    uint8_t *puPingpongBuff = (uint8_t *)&g_au8RecordPingpongBuff[0];
    uint8_t *puRingBuff = (uint8_t *)&g_au8RecordRingBuff[0];   
#endif
    if (g_usbd_RecMute)
    {
        for(i=0; i<(g_usbd_RecSampleRate/1000); i++)
        {
            for(j=0; j<REC_CHANNELS; j++)
                puRingBuff[g_u16Record_Write_Ptr*REC_CHANNELS+j] = 0;         
            g_u16Record_Write_Ptr++;
            if (g_u16Record_Write_Ptr >= g_u16RecordRingBuffLen)
            {
                g_u16Record_Write_Ptr = 0;
            }
        }        
    }
    else
    {
        for(i=0; i<(g_usbd_RecSampleRate/1000); i++)
        {
            for(j=0; j<REC_CHANNELS; j++)
                puRingBuff[g_u16Record_Write_Ptr*REC_CHANNELS+j] = puPingpongBuff[(((g_usbd_RecSampleRate/1000)*REC_CHANNELS)*u8Index)+(i*REC_CHANNELS+j)];         
            g_u16Record_Write_Ptr++;
            if (g_u16Record_Write_Ptr >= g_u16RecordRingBuffLen)
            {
                g_u16Record_Write_Ptr = 0;
            }
        }     
    } 
}
