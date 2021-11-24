/**************************************************************************//**
 * @file     descriptors.c
 * @brief    ISD series USBD driver source file
 * @version  1.0.0
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "Platform.h"
#include "usbd_audio.h"

#define WBVAL(x) (x&0xFF), ((x >>8) & 0xFF) 
#define B3VAL(x) (x&0xFF), ((x >>8) & 0xFF), ((x >>16) & 0xFF)  

const uint8_t gu8HidReportDesc[67] = 
{
    0x05, 0x0C,      // Usage Page (Consumer)
    0x09, 0x01,      // Usage(Consumer Control)
    0xA1, 0x01,      // Collection(Application )
    0x15, 0x00,      // Logical Minimum(0x0 )
    0x25, 0x01,      // Logical Maximum(0x1 )
    0x09, 0xE2,      // Usage(Mute)
    0x09, 0xE9,      // Usage(Volume Increment)
    0x09, 0xEA,      // Usage(Volume Decrement)
    0x75, 0x01,      // Report Size(0x1 )
    0x95, 0x03,      // Report Count(0x3 )
    0x81, 0x02,      // Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field)
    0x75, 0x01,      // Report Size(0x1 )
    0x95, 0x05,      // Report Count(0x5 )
    0x81, 0x03,      // Input(Constant, Variable, Absolute) - Padding 
	
    0x09, 0xB0,      // Usage(Play)
    0x09, 0xB7,      // Usage(Stop)
    0x09, 0xCD,      // Usage(Play/Pause)
    0x09, 0xB5,      // Usage(Scan Next Track)
    0x09, 0xB6,      // Usage(Scan Previous Track)
    0x09, 0xB2,      // Usage(Record)
    0x09, 0xB4,      // Usage(Rewind)
    0x09, 0xB3,      // Usage(Fast Forward)
    0x75, 0x01,      // Report Size(0x1 )
    0x95, 0x08,      // Report Count(0x8 )
    0x81, 0x02,      // Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field)
		
    0x09, 0x00,      // Usage(Undefined)
    0x75, 0x08,      // Report Size(0x8 )
    0x95, 0x06,      // Report Count(0x6 )
    0x81, 0x02,      // Input(Data, Variable, Absolute, No Wrap, Linear, Preferred State, No Null Position, Bit Field)
		
    0x09, 0x00,      // Usage(Undefined)
    0x75, 0x08,      // Report Size(0x8 )
    0x95, 0x08,      // Report Count(0x8 )
    0x91, 0x00,
    0xC0
};


/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
const uint8_t gu8DeviceDescriptor[] = 
{
    LEN_DEVICE,                         /* bLength */
    DESC_DEVICE,                        /* bDescriptorType */
    WBVAL(0x0200),                      /* bcdUSB */
    0x00,                               /* bDeviceClass */
    0x00,                               /* bDeviceSubClass */
    0x00,                               /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE,                   /* bMaxPacketSize0 */
    WBVAL(USBD_VID),                    /* idVendor */
    WBVAL(USBD_PID),                    /* idProduct */ 
    0x00, 0x00,                         /* bcdDevice */
    0x01,                               /* iManufacture */
    0x02,                               /* iProduct */
    0x03,                               /* iSerialNumber
                                            NOTE: The serial number must be different between each MassStorage device. */
    0x01                                /* bNumConfigurations */
};

#define HID_REPORT_DESCRIPTOR_SIZE  sizeof(gu8HidReportDesc) 

/*!<USB Configure Descriptor */
const uint8_t gu8ConfigDescriptor[] = 
{
    LEN_CONFIG,                         /* bLength */
    DESC_CONFIG,                        /* bDescriptorType */
#if (REC_CHANNELS == 1)
#if __FEEDBACK__
    WBVAL(227),                         /* wTotalLength */
#else
    WBVAL(218),                         /* wTotalLength */
#endif 
#elif (REC_CHANNELS == 2)
#if __FEEDBACK__
    WBVAL(228),                         /* wTotalLength */
#else
    WBVAL(219),                         /* wTotalLength */
#endif      
#endif
    0x04,                               /* bNumInterfaces */
    0x01,                               /* bConfigurationValue */
    0x00,                               /* iConfiguration */
    USB_CONFIG_BUS_POWERED,             /* bmAttributes */ 
    USB_CONFIG_POWER_MA(100),           /* Max power */
	
    /* Interface Descriptor (Audio Class) */
    LEN_INTERFACE,                      /* bLength */
    DESC_INTERFACE,                     /* bDescriptorType */
    0x00,                               /* bInterfaceNumber */
    0x00,                               /* bAlternateSetting */
    0x00,                               /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,             /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOCONTROL,        /* bInterfaceSubClass:AUDIOCONTROL */
    0x00,                               /* bInterfaceProtocol */
    0x00,                               /* iInterface */

    /* Audio Control Interface Header Descriptor */
    0x0A,                               /* bLength */
    AUDIO_CS_INTERFACE,                 /* bDescriptorType:CS_INTERFACE */
    AUDIO_HEADER,                       /* bDescriptorSubType:HEADER */
    WBVAL(0x0100),                      /* bcdADC:1.0 */
#if (REC_CHANNELS == 1)
    WBVAL(0x47),                        /* wTotalLength */
#elif (REC_CHANNELS == 2)
    WBVAL(0x48), 
#endif
    0x02,                               /* bInCollection */
    0x01,                               /* baInterfaceNr(1) - Record */
    0x02,                               /* baInterfaceNr(2) - Playback */

    /* Audio Control Input Terminal Descriptor - Record */
    0x0C,                               /* bLength */
    AUDIO_CS_INTERFACE,                 /* bDescriptorType:CS_INTERFACE */
    AUDIO_INPUT_TERMINAL,               /* bDescriptorSubType:INPUT_TERMINAL*/
    REC_IT_UNITID,                      /* bTerminalID*/
    WBVAL(0x201),                       /* wTerminalType: 0x0201 microphone*/
    0x00,                               /* bAssocTerminal*/
    REC_CHANNELS,                       /* bNrChannels*/
    WBVAL(REC_CH_CFG),                  /* wChannelConfig*/
    0x00,                               /* iChannelNames*/
    0x00,                               /* iTerminal*/
		
    /* Audio Control Feature Unit Descriptor - Record */
#if (REC_CHANNELS == 1)
    0x09,
#elif (REC_CHANNELS == 2)
    0x0A,
#endif
    AUDIO_CS_INTERFACE,                 /* bDescriptorType */
    AUDIO_FEATURE_UNIT,                 /* bDescriptorSubType */
    REC_FEATURE_UNITID,                 /* bUnitID */
    REC_IT_UNITID,                      /* bSourceID */
    0x01,                               /* bControlSize */
    0x03,                               /* bmaControls(0) */ 
    0x00,                               /* bmaControls(1) */ 
#if (REC_CHANNELS == 2)
    0x00,                               /* bmaControls(2) */
#endif
    0x00,               	            /* iFeature */
		
    /* Audio Control Output Terminal Descriptor - Record */
    0x09,               	            /* bLength */
    AUDIO_CS_INTERFACE,                 /* bDescriptorType:CS_INTERFACE */
    AUDIO_OUTPUT_TERMINAL,              /* bDescriptorSubType:OUTPUT_TERMINAL */
    REC_OT_UNITID,                      /* bTerminalID */
    WBVAL(0x0101),                      /* wTerminalType */
    0x00,                               /* bAssocTerminal */
    REC_FEATURE_UNITID,                 /* bSourceID */
    0x00,                               /* iTerminal */
    
    /* Audio Control Input Terminal Descriptor - Playback */
    0x0C,                               /* bLength */
    AUDIO_CS_INTERFACE,                 /* bDescriptorType:CS_INTERFACE */
    AUDIO_INPUT_TERMINAL,               /* bDescriptorSubType:INPUT_TERMINAL */
    PLAY_IT_UNITID,                     /* bTerminalID */
    WBVAL(0x0101),                      /* wTerminalType */
    0x00,                               /* bAssocTerminal */
    PLAY_CHANNELS,                      /* bNrChannels */
    WBVAL(PLAY_CH_CFG),                 /* wChannelConfig */
    0x00,                               /* iChannelNames */
    0x00,                               /* iTerminal */
	
    /* Audio Control Feature Unit Descriptor - Playback */
    0x0A,                               /* bLength */
    AUDIO_CS_INTERFACE,                 /* bDescriptorType */
    AUDIO_FEATURE_UNIT,                 /* bDescriptorSubType */
    PLAY_FEATURE_UNITID,                /* bUnitID */
    PLAY_IT_UNITID,                     /* bSourceID */
    0x01,                               /* bControlSize */
    0x01,                               /* bmaControls(0) */
    0x00,                               /* bmaControls(1) */
    0x00,                               /* bmaControls(2) */
    0x00,                               /* iFeature */
		
    /* Audio Control Output Terminal Descriptor - Playback */
    0x09,                               /* bLength*/
    AUDIO_CS_INTERFACE,                 /* bDescriptorType:CS_INTERFACE*/
    AUDIO_OUTPUT_TERMINAL,              /* bDescriptorSubType:OUTPUT_TERMINAL*/
    PLAY_OT_UNITID,                     /* bTerminalID*/
    WBVAL(0x0301),                      /* wTerminalType: 0x0301 speaker*/
    0x00,                               /* bAssocTerminal*/
    PLAY_FEATURE_UNITID,                /* bSourceID*/
    0x00,                               /* iTerminal*/
       
    /* Interface Descriptor - Interface 1, alternate 0 */
    LEN_INTERFACE,                      /* bLength */
    DESC_INTERFACE,                     /* bDescriptorType */
    0x01,                               /* bInterfaceNumber */
    0x00,                               /* bAlternateSetting */
    0x00,                               /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,             /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOSTREAMING,      /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,                               /* bInterfaceProtocol */
    0x00,                               /* iInterface */

    /* Interface Descriptor - Interface 1, alternate 1 */
    LEN_INTERFACE,                      /* bLength */
    DESC_INTERFACE,                     /* bDescriptorType */
    0x01,                               /* bInterfaceNumber */
    0x01,                               /* bAlternateSetting */
    0x01,                               /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,             /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOSTREAMING,      /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,                               /* bInterfaceProtocol */
    0x00,                               /* iInterface */

    /* Audio Streaming Class Specific Interface Descriptor */
    0x07,                               /* bLength */
    AUDIO_CS_INTERFACE,                 /* bDescriptorType:CS_INTERFACE */
    AUDIO_AS_GENERAL,                   /* bDescriptorSubType:AS_GENERAL */
    REC_OT_UNITID,                      /* bTernimalLink */
    0x01,                               /* bDelay */
    WBVAL(0x0001),                      /* wFormatTag */

    /* Audio Streaming Format Type Descriptor */
    0x0B,                               /* bLength */
    AUDIO_CS_INTERFACE,                 /* bDescriptorType:CS_INTERFACE */
    AUDIO_FORMAT_TYPE,                  /* bDescriptorSubType:FORMAT_TYPE */
    0x01,                               /* bFormatType:FORMAT_TYPE_I */
    REC_CHANNELS,                       /* bNrChannels */
    0x02,                               /* bSubFrameSize */
    REC_RES_MAX,                        /* bBitResolution */
    0x01,                               /* bSamFreqType */
    B3VAL(SAMPLE_RATE_48K),             /* tSamFreq[1] */

    /* Endpoint Descriptor (ISO IN Audio Data Endpoint - alternate 1) */
    0x09,                               /* bLength */
    DESC_ENDPOINT,                      /* bDescriptorType */
    (EP2 | EP_INPUT),                   /* bEndpointAddress */
    0x09,                               /* bmAttributes: Adaptive*/
    WBVAL(EP2_MAX_PKT_SIZE),            /* wMaxPacketSize */
    0x01,                               /* bInterval*/
    0x00,                               /* bRefresh*/
    0x00,                               /* bSynchAddress*/

    /* Audio Streaming Class Specific Audio Data Endpoint Descriptor */
    0x07,                               /* bLength */
    AUDIO_CS_ENDPOINT,                  /* bDescriptorType:CS_ENDPOINT */
    0x01,                               /* bDescriptorSubType:EP_GENERAL */
    0x00,                               /* bmAttributes */
    0x00,                               /* bLockDelayUnits */
    WBVAL(0x0000),                      /* wLockDelay */
		
    /* Interface Descriptor - Interface 2, alternate 0 */
    LEN_INTERFACE,                      /* bLength */
    DESC_INTERFACE,                     /* bDescriptorType */
    0x02,                               /* bInterfaceNumber */
    0x00,                               /* bAlternateSetting */
    0x00,                               /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,             /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOSTREAMING,      /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,                               /* bInterfaceProtocol */
    0x00,                               /* iInterface */

    /* Interface Descriptor - Interface 2, alternate 1 */
    LEN_INTERFACE,                      /* bLength */
    DESC_INTERFACE,                     /* bDescriptorType */
    0x02,                               /* bInterfaceNumber */    
    0x01,                               /* bAlternateSetting */    
#if __FEEDBACK__
    0x02,                               /* bNumEndpoints : ISO OUT Endpoint for play & ISO IN Endpoint for Feedback */
#else
    0x01,                               /* bNumEndpoints : ISO OUT Endpoint for play */
#endif
    USB_DEVICE_CLASS_AUDIO,             /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOSTREAMING,      /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,                               /* bInterfaceProtocol */
    0x00,                               /* iInterface */

    /* Audio Streaming Class Specific Interface Descriptor */
    0x07,                               /* bLength */
    AUDIO_CS_INTERFACE,                 /* bDescriptorType:CS_INTERFACE */
    0x01,                               /* bDescriptorSubType:AS_GENERAL */
    PLAY_IT_UNITID,                     /* bTernimalLink */
    0x01,                               /* bDelay */
    WBVAL(0x0001),                      /* wFormatTag */

    /* Audio Streaming Format Type Descriptor */
    0x0B,                               /* bLength */
    AUDIO_CS_INTERFACE,                 /* bDescriptorType:CS_INTERFACE */
    0x02,                               /* bDescriptorSubType:FORMAT_TYPE */
    0x01,                               /* bFormatType:FORMAT_TYPE_I */
    PLAY_CHANNELS,                      /* bNrChannels */
    0x02,                               /* bSubFrameSize */
    PLAY_RES_MAX,                       /* bBitResolution */
    0x01,                               /* bSamFreqType */
    B3VAL(SAMPLE_RATE_48K),             /* tSamFreq[1] */

    /* Endpoint Descriptor (ISO OUT Audio Data Endpoint - alternate 1) */
    0x09,                               /* bLength */
    DESC_ENDPOINT,                      /* bDescriptorType */
    (EP3 | EP_OUTPUT),                  /* bEndpointAddress */
#if __FEEDBACK__
    0x05,                               /* bmAttributes: Asynchronous*/
#else
    0x09,                               /* bmAttributes: Adaptive*/
#endif
    WBVAL(EP3_MAX_PKT_SIZE),            /* wMaxPacketSize */
    0x01,                               /* bInterval */
    0x00,                               /* bRefresh */
#if __FEEDBACK__
    (EP4 | EP_INPUT),                   /* bSynchAddress */
#else
    0x00,                               /* bSynchAddress */
#endif

    /* Audio Streaming Class Specific Audio Data Endpoint Descriptor */
    0x07,                               /* bLength */
    AUDIO_CS_ENDPOINT,                  /* bDescriptorType:CS_ENDPOINT */
    0x01,                               /* bDescriptorSubType:EP_GENERAL */
    0x01,                               /* bmAttributes */
    0x00,                               /* bLockDelayUnits */
    WBVAL(0x0000),                      /* wLockDelay */

#if __FEEDBACK__
    /* Feedback Endpoint */
    0x09,                               /* bLength */ 
    DESC_ENDPOINT,                      /* bDescriptorType */
    (EP4 | EP_INPUT),                   /* bEndpointAddress */
    0x11,                               /* bmAttributes */
    WBVAL(EP4_MAX_PKT_SIZE),            /* wMaxPacketSize */
    0x01,                               /* bInterval */
    0x02,                               /* bRefresh */	  
    0x00,                               /* bSynchAddress */
#endif

    /* I/F descr: HID */
    LEN_INTERFACE,                      /* bLength */
    DESC_INTERFACE,                     /* bDescriptorType */
    0x03,                               /* bInterfaceNumber */ 
    0x00,                               /* bAlternateSetting */
    0x01,                               /* bNumEndpoints */
    USB_DEVICE_CLASS_HUMAN_INTERFACE,   /* bInterfaceClass : HID */
    0x01,                               /* bInterfaceSubClass */
    0x01,                               /* bInterfaceProtocol */
    0x00,                               /* iInterface */

    // HID Descriptor
    LEN_HID,                            /* Size of this descriptor in UINT8s. */
    DESC_HID,                           /* HID descriptor type. */
    WBVAL(0x0110),                      /* HID Class Spec. release number. */
    0x00,                               /* H/W target country. */
    0x01,                               /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,                       /* Dscriptor type. */
    WBVAL(HID_REPORT_DESCRIPTOR_SIZE),	/* Total length of report descriptor */
		
    /* EP Descriptor: interrupt in */
    LEN_ENDPOINT,                       /* bLength */
    DESC_ENDPOINT,                      /* bDescriptorType */
    (EP5 | EP_INPUT),                   /* bEndpointAddress */
    EP_INT,                             /* bmAttributes */   
    WBVAL(EP5_MAX_PKT_SIZE),            /* wMaxPacketSize */
    10                                  /* bInterval */
};

/*!<USB Language String Descriptor */
const uint8_t gu8StringLang[4] = 
{
    4,                                  /* bLength */
    DESC_STRING,                        /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
const uint8_t gu8VendorStringDesc[16] = 
{
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
const uint8_t gu8ProductStringDesc[22] = {
    22,                                 /* bLength          */
    DESC_STRING,                        /* bDescriptorType  */
    'U', 0, 
    'A', 0, 
    'C', 0, 
    ' ', 0, 
    'D', 0, 
    'e', 0, 
    'm', 0, 
    'o', 0, 
    ' ', 0, 
    ' ', 0
};


uint8_t gu8StringSerial[26] = 
{
    26,                                 // bLength
    DESC_STRING,                        // bDescriptorType
    'A', 0, '0', 0, '0', 0, '0', 0, '0', 0, '8', 0, '0', 0, '4', 0, '0', 0, '1', 0, '1', 0, '5', 0

};

/*!<USB BOS Descriptor */
uint8_t gu8BOSDescriptor[] = 
{
    LEN_BOS,                            /* bLength */
    DESC_BOS,                           /* bDescriptorType */
    /* wTotalLength */
    0x0C & 0x00FF,
    (0x0C & 0xFF00) >> 8,
    0x01,                               /* bNumDeviceCaps */

    /* Device Capability */
    0x7,                                /* bLength */
    DESC_CAPABILITY,                    /* bDescriptorType */
    CAP_USB20_EXT,                      /* bDevCapabilityType */
    0x02, 0x00, 0x00, 0x00              /* bmAttributes */
};


const uint8_t *gpu8UsbString[4] = {
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    gu8StringSerial
};

const uint8_t *gu8UsbHidReport[6] = {
    NULL,
    NULL,
    NULL,
    gu8HidReportDesc,
    NULL,
    NULL 
};

const uint32_t gu32UsbHidReportLen[6] = {
    0,
    0,
    0,
    sizeof(gu8HidReportDesc),
    0,
    0
};

const uint32_t gu32ConfigHidDescIdx[4] = {
    0,
    0,
    0,
    (sizeof(gu8ConfigDescriptor) - LEN_ENDPOINT - LEN_HID)
};


const S_USBD_INFO_T gsInfo = 
{
    (uint8_t *)gu8DeviceDescriptor,
    (uint8_t *)gu8ConfigDescriptor,
    (const uint8_t **)gpu8UsbString,
    (const uint8_t **)gu8UsbHidReport,
    (uint8_t *)gu8BOSDescriptor,
    (uint32_t *)gu32UsbHidReportLen,
    (uint32_t *)gu32ConfigHidDescIdx
};

