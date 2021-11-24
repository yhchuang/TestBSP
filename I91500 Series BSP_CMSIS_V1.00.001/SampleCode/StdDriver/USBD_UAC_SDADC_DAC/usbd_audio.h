/**************************************************************************//**
 * @file     usbd_audio.h
 * @brief    ISD series USB driver header file
 * @version  1.0.0
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_AUDIO_H__
#define __USBD_AUDIO_H__

#define __FEEDBACK__                                0   /* 1: Playback ISOCHRONOUS Async 0: Playback ISOCHRONOUS Adaptive */

/* Define the vendor id and product id */
#define USBD_VID                                    0x0416
#define USBD_PID                                    0x130A

/*!<Define Audio information */
#define RES_16                                      16
#define SAMPLE_RATE_16K                             16000
#define SAMPLE_RATE_48K                             48000

/* Playback */
#define PLAY_CHANNELS    	                        2         		  /* Number of channels. (Must be 2) */
#if (PLAY_CHANNELS == 1)
    #define PLAY_CH_CFG                             1
#elif (PLAY_CHANNELS == 2)
    #define PLAY_CH_CFG                             3
#endif
#define PLAY_RES_MAX                                RES_16            /* Resolution of channel. */
#define PLAY_RATE_MAX                               SAMPLE_RATE_48K   /* The audo play sampling rate. */
#if (PLAY_RES_MAX == RES_32)
#define PLAY_RES_BYTE                               4
#elif (PLAY_RES_MAX == RES_24)
#define PLAY_RES_BYTE                               4
#elif (PLAY_RES_MAX == RES_16)
#define PLAY_RES_BYTE                               2
#elif (PLAY_RES_MAX == RES_8)   
#define PLAY_RES_BYTE                               1
#endif

/* Record */
#define REC_CHANNELS    	                        1                   /* Number of channels. (Must be 1 or 2)*/
#if (REC_CHANNELS == 1)
    #define REC_CH_CFG                              0x1
#elif(REC_CHANNELS == 2)
    #define REC_CH_CFG                              0x3
#endif
#define REC_RES_MAX     	                        RES_16              /* Resolution of channel. */
#define REC_RATE_MAX       		                    SAMPLE_RATE_48K     /* The record sampling rate. Must be the same with PLAY_RATE */
#if (REC_RES_MAX == RES_32)
#define REC_RES_BYTE                                4
#elif (REC_RES_MAX == RES_24)
#define REC_RES_BYTE                                4   
#elif (REC_RES_MAX == RES_16)
#define REC_RES_BYTE                                2
#elif (REC_RES_MAX == RES_8)
#define REC_RES_BYTE                                1
#endif

/* Unit ID */
#define PLAY_IT_UNITID                              0x01
#define PLAY_FEATURE_UNITID                         0x02
#define PLAY_OT_UNITID                              0x03
#define REC_IT_UNITID                               0x04
#define REC_FEATURE_UNITID                          0x05
#define REC_OT_UNITID                               0x06

/* Ring buffer definition */
#define RING_BUFF_LEVEL                             8


/* Define EP maximum packet size */
#define EP0_MAX_PKT_SIZE                            8     
#define EP1_MAX_PKT_SIZE                            EP0_MAX_PKT_SIZE
/* Maximum Packet Size for Recprd Endpoint */
#define EP2_MAX_PKT_SIZE                            (REC_RATE_MAX*REC_CHANNELS*3/1000)	
/* Maximum Packet Size for Play Endpoint */
#define EP3_MAX_PKT_SIZE                            (PLAY_RATE_MAX*PLAY_CHANNELS*3/1000)
/* Maximum Packet Size for Feedback Endpoint */
#define EP4_MAX_PKT_SIZE                            8  
/* Maximum Packet Size for HID Endpoint */
#define EP5_MAX_PKT_SIZE                            16  
/* Maximum Packet Size for HID Endpoint */
#define EP6_MAX_PKT_SIZE                            16

#define SETUP_BUF_BASE                              0
#define SETUP_BUF_LEN                               8
#define EP0_BUF_BASE                                (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN                                 (EP0_MAX_PKT_SIZE)
#define EP1_BUF_BASE                                (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN                                 (EP1_MAX_PKT_SIZE)
#define EP2_BUF_BASE                                (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN                                 (EP2_MAX_PKT_SIZE)
#define EP3_BUF_BASE                                (EP2_BUF_BASE + EP2_BUF_LEN)
#define EP3_BUF_LEN                                 (EP3_MAX_PKT_SIZE)
#define EP4_BUF_BASE                                (EP3_BUF_BASE + EP3_BUF_LEN)
#define EP4_BUF_LEN                                 (EP4_MAX_PKT_SIZE)
#define EP5_BUF_BASE                                (EP4_BUF_BASE + EP4_BUF_LEN)
#define EP5_BUF_LEN                                 (EP5_MAX_PKT_SIZE)
#define EP6_BUF_BASE                                (EP5_BUF_BASE + EP5_BUF_LEN)
#define EP6_BUF_LEN                                 (EP6_MAX_PKT_SIZE)
#define EP7_BUF_BASE                                (EP6_BUF_BASE + EP6_BUF_LEN)
#define EP7_BUF_LEN                                 (EP7_MAX_PKT_SIZE)


/* Audio Interface Subclass Codes */
#define AUDIO_SUBCLASS_UNDEFINED                    0x00
#define AUDIO_SUBCLASS_AUDIOCONTROL                 0x01
#define AUDIO_SUBCLASS_AUDIOSTREAMING               0x02
#define AUDIO_SUBCLASS_MIDISTREAMING                0x03

/* Audio Interface Protocol Codes */
#define AUDIO_PROTOCOL_UNDEFINED                    0x00

/* Audio Class-Specific Descriptor Types */
#define AUDIO_CS_UNDEFINED                          0x20
#define AUDIO_CS_DEVICE                             0x21
#define AUDIO_CS_CONFIGURATION                      0x22
#define AUDIO_CS_STRING                             0x23
#define AUDIO_CS_INTERFACE                          0x24
#define AUDIO_CS_ENDPOINT                           0x25

/* Audio Class-Specific AC Interface Descriptor Subtypes */
#define AUDIO_AC_DESCRIPTOR_INDEFINED               0x00
#define AUDIO_HEADER                                0x01
#define AUDIO_INPUT_TERMINAL                        0x02
#define AUDIO_OUTPUT_TERMINAL                       0x03
#define AUDIO_MIXER_UNIT                            0x04
#define AUDIO_SELECTOR_UNIT                         0x05
#define AUDIO_FEATURE_UNIT                          0x06
#define AUDIO_PROCESSING_UNIT                       0x07
#define AUDIO_EXTENSION_UNIT                        0x08

/* Audio Class-Specific AS Interface Descriptor Subtypes */
#define AUDIO_AS_DESCRIPTOR_INDEFINED               0x00
#define AUDIO_AS_GENERAL                            0x01
#define AUDIO_FORMAT_TYPE                           0x02
#define AUDIO_SPECIFIC                              0x03

/* bmAttributes in Configuration Descriptor */
#define USB_CONFIG_POWERED_MASK                     0x40
#define USB_CONFIG_BUS_POWERED                      0x80
#define USB_CONFIG_SELF_POWERED                     0xC0
#define USB_CONFIG_REMOTE_WAKEUP                    0x20

/* bMaxPower in Configuration Descriptor */
#define USB_CONFIG_POWER_MA(mA)                     ((mA)/2)


/********************************************/
/* Audio Class Current State                */
/********************************************/
/*!<Define Audio Class Current State */
#define UAC_PLAYBACK_STOP                           0x0
#define UAC_PLAYBACK_START                          0x1
#define UAC_PLAYBACK_BUSY                           0x2

#define UAC_RECORD_STOP                             0x0
#define UAC_RECORD_START                            0x1
#define UAC_RECORD_BUSY                             0x2

/***************************************************/
/*      Audio Class-Specific Request Codes         */
/***************************************************/
/*!<Define Audio Class Specific Request */
#define UAC_REQUEST_CODE_UNDEFINED                  0x00
#define UAC_SET_CUR                                 0x01
#define UAC_GET_CUR                                 0x81
#define UAC_SET_MIN                                 0x02
#define UAC_GET_MIN                                 0x82
#define UAC_SET_MAX                                 0x03
#define UAC_GET_MAX                                 0x83
#define UAC_SET_RES                                 0x04
#define UAC_GET_RES                                 0x84
#define UAC_SET_MEM                                 0x05
#define UAC_GET_MEM                                 0x85
#define UAC_GET_STAT                                0xFF

#define MUTE_CONTROL                                0x01
#define VOLUME_CONTROL                              0x02

/* Endpoint Control Selectors */
#define EP_CONTROL_UNDEFINED                        0x00
#define SAMPLING_FREQ_CONTROL                       0x01


/*!<Define HID Class Specific Request */
#define GET_REPORT                                  0x01
#define GET_IDLE                                    0x02
#define GET_PROTOCOL                                0x03
#define SET_REPORT                                  0x09
#define SET_IDLE                                    0x0A
#define SET_PROTOCOL                                0x0B

/*-------------------------------------------------------------*/
void UAC_Init(void);
void UAC_ClassRequest(void);
void UAC_SetInterface(void);

/*-------------------------------------------------------------*/
void EP2_Handler(void);
void EP3_Handler(void);
void EP4_Handler(void);

void PlaybackBuffInit(void);
void RecordBuffInit(void);

void USBPlaybackPauseDetect(void);
void USBPlaybackPauseDetectReset(void);
    
extern volatile uint8_t g_au8PlaybackRingBuff[];
extern volatile uint8_t g_au8RecordRingBuff[];

extern volatile uint8_t g_usbd_Configured; 
extern volatile uint8_t g_usbd_PlayMute;
extern volatile uint8_t g_usbd_RecMute;
extern volatile int16_t g_usbd_RecVolumeL;
extern volatile int16_t g_usbd_RecVolumeR;

extern volatile uint16_t g_u16Record_Read_Ptr;
extern volatile uint16_t g_u16Record_Write_Ptr;

extern volatile uint16_t g_u16PlayBack_Read_Ptr;
extern volatile uint16_t g_u16PlayBack_Write_Ptr;

extern volatile uint32_t g_usbd_PlaySampleRate;
extern volatile uint32_t g_usbd_RecSampleRate;

extern volatile uint8_t g_u8USBPlayEn;
extern volatile uint8_t g_u8USBRecEn;

extern volatile uint16_t g_u16PlaybackRingBuffLen;
extern volatile uint16_t g_u16RecordRingBuffLen;


#endif //__USBD_AUDIO_H__

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
