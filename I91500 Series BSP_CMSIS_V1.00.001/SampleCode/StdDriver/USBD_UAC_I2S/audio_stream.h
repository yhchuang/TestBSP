/**************************************************************************//**
 * @file     audio_stream.h
 * @brief    Audio stream process sample file
 * @version  1.0.0
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __AUDIO_STREAM_H__
#define __AUDIO_STREAM_H__

extern volatile	BOOL bUnderFlow;    //No RingBuffer for reading
extern volatile	BOOL bOverFlow; //No RingBuffer for Writting

void PlaybackPingPongBuffProcess(uint8_t u8Index);
void RecordPingPongBuffProcess(uint8_t u8Index);   

#endif //__AUDIO_STREAM_H__

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
