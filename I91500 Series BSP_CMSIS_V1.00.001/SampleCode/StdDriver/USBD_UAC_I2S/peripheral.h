/**************************************************************************//**
 * @file     peripheral.h
 * @brief    Peripheral configure Sample file
 * @version  1.0.0
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __PERIPHERAL_H__
#define __PERIPHERAL_H__


#define PLAY_PDMA_CH            1
#define REC_PDMA_CH             2
#define PDMATX                  ((PDMA_T *) (PDMA0_BASE + PLAY_PDMA_CH * 0x100))    //PDMA name for playback
#define PDMARX                  ((PDMA_T *) (PDMA0_BASE + REC_PDMA_CH * 0x100))    //PDMA name for rec

#define I2C_TOUT_EN             0           // I2C hardware timeout function is enabled while I2C transfer
#define I2C_TOUT_CNT            1000000     // I2C software timeout counter


extern volatile uint8_t g_au8RecordPingpongBuff[];
extern volatile uint8_t g_au8PlaybackPingpongBuff[];

extern volatile uint8_t g_u8PlaybackDeviceEn;
extern volatile uint8_t g_u8RecordDeviceEn;

extern volatile uint16_t g_u16TrimedValue;


void SYS_Init(void);
void UART0_Init(void);
void USBD_Init(void);
void PDMA_Init (void);
void ANA_Init (void);
void AudioDeviceCtrl(void);
void RecordVolumeControl(void);
void I2S0_Init(void);
void I2C0_Init(void);

int32_t i2c_reg_read_16b(I2C_T *i2c, uint8_t u8dev, uint16_t u16reg, uint16_t *pu16val);
int32_t i2c_reg_write_16b(I2C_T *i2c, uint8_t u8dev, uint16_t u16reg, uint16_t u16val);

    
#endif //__PERIPHERAL_H__

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
