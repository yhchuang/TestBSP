/**************************************************************************//**
 * @file     peripheral.c
 * @brief    Peripheral configure Sample file
 * @version  1.0.0
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <string.h> 
#include "Platform.h"
#include "usbd_audio.h"
#include "peripheral.h"
#include "audio_stream.h"

//----------------------------------------------------------------------------
//  Global variables for Control Pipe
//---------------------------------------------------------------------------- 

volatile uint8_t g_au8RecordPingpongBuff[(REC_RATE_MAX / 1000) * REC_CHANNELS * REC_RES_BYTE * 2];
volatile uint8_t g_au8PlaybackPingpongBuff[(PLAY_RATE_MAX / 1000) * PLAY_CHANNELS * PLAY_RES_BYTE * 2];

volatile uint8_t g_u8PlaybackDeviceEn = 0;
volatile uint8_t g_u8RecordDeviceEn = 0;

volatile int16_t g_i16PreRecVolume = 0;

volatile uint16_t g_u16TrimedValue;

/*--------------------------------------------------------------------------*/
/**
 * @brief       PDMA Interrupt Service Routine
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is the PDMA ISR
 */
void PDMA_IRQHandler(void)
{
	uint32_t u32global = PDMA_GET_INT_STATUS(); 
		        	
	// Playback
	if(u32global & (1 << PLAY_PDMA_CH))      
	{		
        if (PDMA_GET_CH_INT_STS(PLAY_PDMA_CH) & PDMA_HALF_WRAP_FLAG) //Current transfer half complete flag
		{	
            PDMA_CLR_CH_INT_FLAG(PLAY_PDMA_CH, PDMA_HALF_WRAP_FLAG); //Clear interrupt
            PlaybackPingPongBuffProcess(0);
		}
		else //Current transfer finished flag 
		{			
            PDMA_CLR_CH_INT_FLAG(PLAY_PDMA_CH, PDMA_FULL_WRAP_FLAG); //Clear interrupt	
            PlaybackPingPongBuffProcess(1);            
		}
        USBPlaybackPauseDetect();
	}
    // Record
	if(u32global & (1 << REC_PDMA_CH))
	{
        if (PDMA_GET_CH_INT_STS(REC_PDMA_CH) & PDMA_HALF_WRAP_FLAG) //Current transfer half complete flag
		{	
            PDMA_CLR_CH_INT_FLAG(REC_PDMA_CH, PDMA_HALF_WRAP_FLAG); //Clear interrupt
            RecordPingPongBuffProcess(0);
		}
		else //Current transfer finished flag 
		{		
            PDMA_CLR_CH_INT_FLAG(REC_PDMA_CH, PDMA_FULL_WRAP_FLAG); //Clear interrupt
            RecordPingPongBuffProcess(1);  	
		}		
	}
}

/**
 * @brief       Playback ping pong buffer clear
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to clear content of playback ping pong buffer
 */
void PlaybackPingpongBuffClear(void)
{
    uint32_t i;
    
    for(i=0; i<sizeof(g_au8PlaybackPingpongBuff); i++)
    {
        g_au8PlaybackPingpongBuff[i] = 0;
    }
}

/**
 * @brief       Record ping pong buffer clear
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to clear content of record ping pong buffer
 */
void RecordPingpongBuffClear(void)
{
    uint32_t i;
    
    for(i=0; i<sizeof(g_au8RecordPingpongBuff); i++)
    {
        g_au8RecordPingpongBuff[i] = 0;
    }
}

/**
 * @brief       Clock initialization
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to initialize clock domain
 */
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg(); 
    /* Enable Internal HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
#if (CRYSTAL_LESS == 1)
    /*Set HIRC 48M */
    CLK_SetHIRCFrequency(CLK_CLKSEL0_HIRCSEL_48M_VCC33);
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLLFOUT, CLK_CLKDIV_HCLK(1));
#else	
    /*Set HIRC 49M */
    CLK_SetHIRCFrequency(CLK_CLKSEL0_HIRCSEL_49M_VCC33);    
    /* Enable External HXT */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);     
    /* Enable PLL */
    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, 48000000);
    /* Waiting for PLL clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);
    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLLFOUT, CLK_CLKDIV_HCLK(1));	
#endif
    /* Get HIRC default trim value */
    g_u16TrimedValue = SYS_GET_OSCTRIM_VALUE(); 
}

/**
 * @brief       USBD initialization
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to initialize USBD IP
 */
void USBD_Init(void)
{         
	// Reset USBD module.
    SYS_ResetModule(USBD_RST);  
    // Enable USBD module clock 
    CLK_EnableModuleClock(USBD_MODULE);   
#if (CRYSTAL_LESS == 1) 
    // Set USBD clock divid 
    CLK_SetModuleClock(USBD_MODULE,CLK_CLKSEL2_USBDSEL_HIRC,CLK_CLKDIV_USBD(1));	
#else    
    // Set USBD clock divid 
    CLK_SetModuleClock(USBD_MODULE,CLK_CLKSEL2_USBDSEL_PLLFOUT,CLK_CLKDIV_USBD(1));	
#endif	
    // Initiate USBD hardware IP and input HID request for hand-shake.
    USBD_Open(&gsInfo, UAC_ClassRequest, (SET_INTERFACE_REQ)UAC_SetInterface);
    // Initiate AUDIO for endpoint configuration 
    UAC_Init();
    // Set USBD Priority
    NVIC_SetPriority(PDMA_IRQn, 1); 
    // Enable USB IRQ
    NVIC_EnableIRQ(USBD_IRQn);
    // Start USBD for processing.
    USBD_Start();
    while(g_usbd_Configured == 0);
}

/**
 * @brief       DAC initialization
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to initialize DAC IP
 */
void DAC_Init(void)
{
    /* Enable DAC clock. */
    CLK_EnableModuleClock(DAC_MODULE);
    CLK_SetModuleClock(DAC_MODULE,CLK_CLKSEL2_DACSEL_HIRC,CLK_CLKDIV_DAC(1));
    /* Enable DAC FIFO */
	DAC_ENABLE_FIFO(DAC);
	/* Enable channel */
	DAC_ENABLE_RIGHT_CHANNEL(DAC);
	DAC_ENABLE_LEFT_CHANNEL(DAC);
	/* Set channel volume */
	DAC_SET_RIGHT_CHANNEL_VOL(DAC,DAC_DVOL_0DB);
	DAC_SET_LEFT_CHANNEL_VOL(DAC,DAC_DVOL_0DB);
	/* Set fifo data width */
	DAC_SET_DATAWIDTH(DAC,DAC_CTL0_DATAWIDTH_16BIT);
    DAC_SET_HP_VOL(DAC,DAC_BV1P5_NEG12DB);
    /* Initialize analog for dac */
    DAC_EnableAnalog();    
}

/**
 * @brief       DAC start to operate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to start DAC
 */
void DAC_Start(void)
{
	DAC_SetSampleRate(g_usbd_PlaySampleRate); //Set sample rate	
    DAC_CLEAR_FIFO(DAC);
	DAC_ENABLE_RIGHT_CHANNEL(DAC);
	DAC_ENABLE_LEFT_CHANNEL(DAC);
	DAC_ENABLE_PDMA(DAC);
    PDMA_DAC_Init();    //PDMA init  	
}

/**
 * @brief       DAC stop to operate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to stop DAC
 */
void DAC_Stop(void)
{
    DAC_CLEAR_FIFO(DAC);
    DAC_DISABLE_PDMA(DAC);
    DAC_DISABLE_LEFT_CHANNEL(DAC);
    DAC_DISABLE_RIGHT_CHANNEL(DAC);	
    PDMA_STOP(PLAY_PDMA_CH);
}

/**
 * @brief       PDMA configure for DAC
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure PDMA for DAC
 */
void PDMA_DAC_Init(void)
{
    PDMA_SoftwareReset(PLAY_PDMA_CH);     
    PDMA_Open( 1 << PLAY_PDMA_CH );  	// Open Channel
    PDMA_SetTransferMode(PLAY_PDMA_CH, PDMA_DAC);
    PDMA_SetTransferCnt(PLAY_PDMA_CH, PDMA_WIDTH_32, 0);
    PDMATX->BCR = ((g_usbd_PlaySampleRate/1000) * PLAY_CHANNELS * PLAY_RES_BYTE * 2);
    PDMA_SetTransferAddr(PLAY_PDMA_CH, (uint32_t)&g_au8PlaybackPingpongBuff, PDMA_SAR_WRA, (uint32_t)&DAC->DAT, PDMA_DAR_FIX);
    PDMA_SetTransferDirection(PLAY_PDMA_CH,PDMA_SRAM_APB);	     
    PDMA_WrapIntSelect(PLAY_PDMA_CH,PDMA_BOTH_WRAP_MODE); //Both half and done interrupts generated     
    PDMA_EnableInt(PLAY_PDMA_CH, PDMA_IER_WAR_IE_Msk);  
    PDMA_Trigger(PLAY_PDMA_CH);      
}

/* BIQ coefficient for record sample rate 16 kHz*/
uint32_t biq_lpf_stage6_OSR_128_SincOrder_64_Fs_16000_2x[30] ={
	//Stage = 1 
	0x031cc,	//B0=0.194519
	0x00078,	//B1=0.001831
	0x031ca,	//B2=0.194489
	0x7cdcb,	//A1=-0.196121
	0x0f5c6,	//A2=0.960052
	
	//Stage = 2 
	0x0799c,	//B0=0.475037
	0x00b62,	//B1=0.044464
	0x0791d,	//B2=0.473099
	0x7c5ab,	//A1=-0.227859
	0x0dd0c,	//A2=0.863464
	
	//Stage = 3 
	0x09ae0,	//B0=0.604980
	0x030c5,	//B1=0.190506
	0x0994a,	//B2=0.598785
	0x7b22e,	//A1=-0.303986
	0x0b5d9,	//A2=0.710342
	
	//Stage = 4 
	0x09e4d,	//B0=0.618362
	0x077c0,	//B1=0.467773
	0x08bb1,	//B2=0.545670
	0x7968d,	//A1=-0.411911
	0x07875,	//A2=0.470535
	
	//Stage = 5 
	0x0a2b1,	//B0=0.635513
	0x0b7d7,	//B1=0.718124
	0x0455b,	//B2=0.270920
	0x7964c,	//A1=-0.412903
	0x02d3d,	//A2=0.176712
	
	//Stage = 6 
	0x0b7e2,	//B0=0.718292
	0x0cfde,	//B1=0.811981
	0x05d88,	//B2=0.365356
	0x7d084,	//A1=-0.185486
	0x00803 	//A2=0.031296
};

uint32_t biq_lpf_stage5_OSR_128_SincOrder_64_Fs_16000_2x[30] ={
//Stage = 1 
0x034f8,	//B0=0.206909
0x0009a,	//B1=0.002350
0x034f3,	//B2=0.206833
0x7cd42,	//A1=-0.198212
0x0f500,	//A2=0.957031

//Stage = 2 
0x07cf0,	//B0=0.488037
0x00e82,	//B1=0.056671
0x07cd6,	//B2=0.487640
0x7c2ed,	//A1=-0.238571
0x0d9ac,	//A2=0.850281

//Stage = 3 
0x09a74,	//B0=0.603333
0x03d43,	//B1=0.239304
0x094f6,	//B2=0.581879
0x7a922,	//A1=-0.339325
0x0ad43,	//A2=0.676804

//Stage = 4 
0x08b66,	//B0=0.544525
0x09439,	//B1=0.578995
0x08805,	//B2=0.531326
0x77fa6,	//A1=-0.501373
0x06ae9,	//A2=0.417618

//Stage = 5 
0x0a818,	//B0=0.656616
0x0d7ca,	//B1=0.842926
0x04bdc,	//B2=0.296326
0x75f42,	//A1=-0.627899
0x027c3, 	//A2=0.155319

//Stage = 6  //HPF
0x0FAB8,	//B0:0.9793701172
0x60A90,	//B1:-1.9587402344
0x0FAB8,	//B2:0.9793701172
0x60AA9,	//A1:-1.9583538097
0x0F58E	//A2:0.9592034996
};

/* BIQ coefficient for record sample rate 48 kHz*/
uint32_t biq_lpf_stage6_OSR_64_SincOrder_32_Fs_48000_2x[30] ={
    //Stage = 1 
    0x031eb,      //B0=0.194992
    0x00078,      //B1=0.001831
    0x031e9,      //B2=0.194962
    0x7cdcb,      //A1=-0.196121
    0x0f5c6,       //A2=0.960052

    //Stage = 2 
    0x079e7,      //B0=0.476181
    0x00b69,      //B1=0.044571
    0x07968,      //B2=0.474243
    0x7c5ab,      //A1=-0.227859
    0x0dd0c,      //A2=0.863464

    //Stage = 3 
    0x09b40,      //B0=0.606445
    0x030e3,      //B1=0.190964
    0x099a9,      //B2=0.600235
    0x7b22e,      //A1=-0.303986
    0x0b5d9,      //A2=0.710342

    //Stage = 4 
    0x09eaf,       //B0=0.619858
    0x0780a,      //B1=0.468903
    0x08c07,      //B2=0.546982
    0x7968d,      //A1=-0.411911
    0x07875,      //A2=0.470535

    //Stage = 5 
    0x0a316,      //B0=0.637054
    0x0b849,      //B1=0.719864
    0x04586,      //B2=0.271576
    0x7964c,      //A1=-0.412903
    0x02d3d,      //A2=0.176712

    //Stage = 6 
    0x0b854,      //B0=0.720032
    0x0d05e,      //B1=0.813934
    0x05dc2,      //B2=0.366241
    0x7d084,      //A1=-0.185486
    0x00803       //A2=0.031296
};

uint32_t biq_lpf_stage5_OSR_64_SincOrder_32_Fs_48000_2x[30] ={
//Stage = 1 
0x034f8,	//B0=0.206909
0x0009a,	//B1=0.002350
0x034f3,	//B2=0.206833
0x7cd42,	//A1=-0.198212
0x0f500,	//A2=0.957031

//Stage = 2 
0x07cf0,	//B0=0.488037
0x00e82,	//B1=0.056671
0x07cd6,	//B2=0.487640
0x7c2ed,	//A1=-0.238571
0x0d9ac,	//A2=0.850281

//Stage = 3 
0x09a74,	//B0=0.603333
0x03d43,	//B1=0.239304
0x094f6,	//B2=0.581879
0x7a922,	//A1=-0.339325
0x0ad43,	//A2=0.676804

//Stage = 4 
0x08b66,	//B0=0.544525
0x09439,	//B1=0.578995
0x08805,	//B2=0.531326
0x77fa6,	//A1=-0.501373
0x06ae9,	//A2=0.417618

//Stage = 5 
0x0a818,	//B0=0.656616
0x0d7ca,	//B1=0.842926
0x04bdc,	//B2=0.296326
0x75f42,	//A1=-0.627899
0x027c3, 	//A2=0.155319

//Stage = 6  //HPF
0x0FE38,	//B0:0.9930419922  //cutoff 150Hz
0x60390,	//B1:-1.9860839844
0x0FE38,	//B2:0.9930419922
0x6038E,	//A1:-1.9861162115
0x0FC78	//A2:0.9862119291

};

/**
 * @brief       BIQ initialization
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to initialize BIQ IP
 */
void BIQ_Init(void)
{
	uint32_t u32ms = 1000 ;
	/* Enable BIQ module */
	CLK_EnableModuleClock(BIQ_MODULE);
	/* Reset module */
	SYS_ResetModule(BIQ_RST);
	/* Stage 6/ 12th order */
	BIQ_SET_STAGE_NUM(BIQ,BIQ_CTL_6STAGES_NUM);
	/* HP off */
	BIQ_DISABLE_HIGHPASS_FILTER(BIQ);
	/* Set Coeffdat. */
    if(g_usbd_RecSampleRate == SAMPLE_RATE_16K)
//        BIQ_SetCoeff(biq_lpf_stage6_OSR_128_SincOrder_64_Fs_16000_2x);
		    BIQ_SetCoeff(biq_lpf_stage5_OSR_128_SincOrder_64_Fs_16000_2x);
    else if(g_usbd_RecSampleRate == SAMPLE_RATE_48K)
//        BIQ_SetCoeff(biq_lpf_stage6_OSR_64_SincOrder_32_Fs_48000_2x);
		    BIQ_SetCoeff(biq_lpf_stage5_OSR_64_SincOrder_32_Fs_48000_2x);
	
	/* Delay for setting */
	while(u32ms>0) u32ms--;
	/* Start biq. */
	BIQ_START_RUN(BIQ);
}
	
/**
 * @brief       SDADC initialization
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to initialize SDADC IP
 */
void SDADC_Init (void)
{
	/* Enable SDADC clock. */
	CLK_EnableModuleClock(SDADC_MODULE);
	/* Select SDADC clock source */
    CLK_SetModuleClock(SDADC_MODULE,CLK_CLKSEL2_SDADCSEL_HIRC, CLK_CLKDIV_SDADC(1));
	/* Set fifo bits. */
	SDADC_SET_FIFODATABITS(SDADC, SDADC_FIFODATA_16BITS);	
	/* Enable BST & PGA */
	SDADC_ENABLE_BST(SDADC);
	SDADC_ENABLE_PGA(SDADC);
	/* PGA BST unmute */
	SDADC_MUTEOFF_BST(SDADC);
	SDADC_MUTEOFF_PGA(SDADC);
    /* PGA Common mode Threshold lock adjust enable */
    SDADC->ANA0 &= ~SDADC_ANA0_PGA_CMLCK_Msk;
	/* CMLCKEN disale */
	SDADC->ANA1 &= ~SDADC_ANA1_CMLCKEN_Msk; 
	/* Set BST gain = 0dB */
	SDADC_SET_BST(SDADC, SDADC_BST_GAIN_0DB); 
	/* Set PGA gain = 0dB */
	SDADC_SET_PGA(SDADC, SDADC_PGACTL_GAIN_0DB);
	/* SDADC power on */
	SDADC_ANALOG_POWERON(SDADC);	  
}

/**
 * @brief       SDADC start to operate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to start SDADC
 */
void SDADC_Start(void)
{   
    // Initiate BIQ filter.
	BIQ_Init();
	// Set biq to SDADC.
	BIQ_SET_SDADCPATH(BIQ,BIQ_CTL_SDADC_DOWNSAMPLE2X);
	// Set down sample rate.
    if(g_usbd_RecSampleRate == SAMPLE_RATE_16K)
        SDADC_SET_DSRATIO(SDADC,SDADC_DS_RATION_64);			
    else if(g_usbd_RecSampleRate == SAMPLE_RATE_48K)    
        SDADC_SET_DSRATIO(SDADC,SDADC_DS_RATION_32);			
	// Set SDADC sample rate.
	SDADC_SetAudioSampleRate(g_usbd_RecSampleRate);
    SDADC_START_CONV(SDADC); 
    PDMA_SDADC_Init();	   
	SDADC_ENABLE_PDMA(SDADC);
}

/**
 * @brief       SDADC stop to operate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to stop SDADC
 */
void SDADC_Stop(void)
{
	SDADC_STOP_CONV(SDADC);
    SDADC_DISABLE_PDMA(SDADC);
}

/**
 * @brief       PDMA configure for SDADC
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure PDMA for SDADC
 */
void PDMA_SDADC_Init(void)
{  
    PDMA_SoftwareReset(REC_PDMA_CH);  
	PDMA_Open(1 << REC_PDMA_CH);      
    PDMA_SetTransferMode(REC_PDMA_CH, PDMA_SDADC);
    PDMA_SetTransferCnt(REC_PDMA_CH, PDMA_WIDTH_16, 0);	
    PDMARX->BCR = ((g_usbd_RecSampleRate/1000) * REC_CHANNELS * REC_RES_BYTE * 2);
    PDMA_SetTransferAddr(REC_PDMA_CH, (uint32_t)&SDADC->DAT, PDMA_SAR_FIX, (uint32_t)&g_au8RecordPingpongBuff, PDMA_DAR_WRA);	
    PDMA_SetTransferDirection(REC_PDMA_CH,PDMA_APB_SRAM);	// Set PDMA channel 1 as SRAM to APB data transfer
    PDMA_WrapIntSelect(REC_PDMA_CH,PDMA_BOTH_WRAP_MODE); //Both half and done interrupts generated     
    PDMA_EnableInt(REC_PDMA_CH, PDMA_IER_WAR_IE_Msk);  
	PDMA_Trigger(REC_PDMA_CH); //Start PDMA transfer
}

/**
 * @brief       Record volume control
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure SDADC PGA
 */
void RecordVolumeControl(void)
{
    if(g_i16PreRecVolume != g_usbd_RecVolumeR)
    {        
        g_i16PreRecVolume = g_usbd_RecVolumeR;
        SDADC_SET_PGA(SDADC, (((g_usbd_RecVolumeR >> 8) / 3) * 2 + 8));    
    }
}

/**
 * @brief       Playback and record on/off control
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to control playback and record enable
 */
void AudioDeviceCtrl(void)
{
    if(g_u8USBPlayEn)
    {
        if(!g_u8PlaybackDeviceEn)
        {
            g_u8PlaybackDeviceEn = 1;
            PlaybackPingpongBuffClear();
            DAC_Start();
        }
    }
    else
    {
        if(g_u8PlaybackDeviceEn)
        {
            g_u8PlaybackDeviceEn = 0;   
            DAC_Stop();
        }      
    }
    if(g_u8USBRecEn)
    {
        if(!g_u8RecordDeviceEn)
        {
            g_u8RecordDeviceEn = 1;
            RecordPingpongBuffClear();
            SDADC_Start();
        }
    }
    else
    {
        if(g_u8RecordDeviceEn)
        {
            g_u8RecordDeviceEn = 0;
            SDADC_Stop();
        }      
    }
}

/**
 * @brief       PDMA initialization
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to initialize PDMA IP
 */
void PDMA_Init (void)
{
	CLK_EnableModuleClock(PDMA_MODULE);
	SYS_ResetModule(PDMA_RST);
	NVIC_ClearPendingIRQ(PDMA_IRQn);
    NVIC_EnableIRQ(PDMA_IRQn);
	NVIC_EnableIRQ(PDMA_IRQn);
}

/**
 * @brief       Analog initialization
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to initialize Analog IP
 */
void ANA_Init (void)
{
    ANA_VMID_Enable();
    ANA_MICBIAS_Enable(ANA_MICBCTR_MICBVSEL_2P4V);
}
