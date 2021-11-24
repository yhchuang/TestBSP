/**************************************************************************//**
 * @file     peripheral.h
 * @brief    Peripheral configure Sample file
 * @version  1.0.0
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __PERIPHERAL_H__
#define __PERIPHERAL_H__


#define CRYSTAL_LESS            0   //0: enable HXT, PLL clock is from HXT, set HIRC=49.152MHz. 1: set HIRC=48MHz.

#define PLAY_PDMA_CH            1
#define REC_PDMA_CH             2
#define PDMATX                  ((PDMA_T *) (PDMA0_BASE + PLAY_PDMA_CH * 0x100))    //PDMA name for playback
#define PDMARX                  ((PDMA_T *) (PDMA0_BASE + REC_PDMA_CH * 0x100))    //PDMA name for rec


extern volatile uint8_t g_au8RecordPingpongBuff[];
extern volatile uint8_t g_au8PlaybackPingpongBuff[];

extern volatile uint8_t g_u8PlaybackDeviceEn;
extern volatile uint8_t g_u8RecordDeviceEn;

extern volatile uint16_t g_u16TrimedValue;


void SYS_Init(void);
void UART0_Init(void);
void USBD_Init(void);
void DAC_Init(void);
void DAC_Start(void);
void DAC_Stop(void);
void PDMA_DAC_Init(void);
void SDADC_Init (void);
void SDADC_Start(void);
void SDADC_Stop(void);
void PDMA_SDADC_Init(void);
void PDMA_Init (void);
void ANA_Init (void);
void AudioDeviceCtrl(void);
void RecordVolumeControl(void);
    
#endif //__PERIPHERAL_H__

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
