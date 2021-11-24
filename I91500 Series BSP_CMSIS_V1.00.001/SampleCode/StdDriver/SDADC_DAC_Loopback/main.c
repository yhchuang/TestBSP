/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/01/23 2:52p $
 * @brief    Demo Aanlog or Digital Mic input to SDADC
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Platform.h"


#define SAMPLE_RATE_16K                             16000
#define SAMPLE_RATE_48K                             48000
uint32_t g_SampleRate ;

#define SDADC_INT_SAMPLES	(4)			//There are 4 words data in FIFO buffer, SDADC threshold will generate a interrupt


void SDADC_IRQHandler(void)
{
	uint8_t u8i = SDADC_INT_SAMPLES;
	uint32_t Mono2stereo;
	
	if (SDADC_GetIntFlag(SDADC_FIFO_INT))
	{
		do
		{
			while(DAC_IS_FIFOFULL(DAC)); // check DPWM FIFO whether full
			
			Mono2stereo = SDADC_GET_FIFODATA(SDADC);
			Mono2stereo = Mono2stereo + (Mono2stereo<<16);
			DAC->DAT = Mono2stereo;
		}while(--u8i>0);
	}

}

void System_Initiate(void)
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
	  // Set HIRC frequency = 49M
	  CLK_SetHIRCFrequency(CLK_CLKSEL0_HIRCSEL_49M_VCC33);
    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC,  CLK_CLKDIV_HCLK(1));		  
}

void ANA_Init (void)
{
    ANA_VMID_Enable();
    ANA_MICBIAS_Enable(ANA_MICBCTR_MICBVSEL_90VCCA);
}

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

void DAC_Start(void)
{
	DAC_SetSampleRate(g_SampleRate); //Set sample rate	
    DAC_CLEAR_FIFO(DAC);
	DAC_ENABLE_RIGHT_CHANNEL(DAC);
	DAC_ENABLE_LEFT_CHANNEL(DAC);
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
    if(g_SampleRate == SAMPLE_RATE_16K)
//        BIQ_SetCoeff(biq_lpf_stage6_OSR_128_SincOrder_64_Fs_16000_2x);
		    BIQ_SetCoeff(biq_lpf_stage5_OSR_128_SincOrder_64_Fs_16000_2x);
    else if(g_SampleRate == SAMPLE_RATE_48K)
//        BIQ_SetCoeff(biq_lpf_stage6_OSR_64_SincOrder_32_Fs_48000_2x);
		    BIQ_SetCoeff(biq_lpf_stage5_OSR_64_SincOrder_32_Fs_48000_2x);
	
	/* Delay for setting */
	while(u32ms>0) u32ms--;
	/* Start biq. */
	BIQ_START_RUN(BIQ);
}

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
	/* Set PGA gain = 24dB */
	SDADC_SET_PGA(SDADC, SDADC_PGACTL_GAIN_plus_24DB);
	/* SDADC power on */
	SDADC_ANALOG_POWERON(SDADC);	  
}

void SDADC_Start(void)
{   
  // Initiate BIQ filter.
	BIQ_Init();
	// Set biq to SDADC.
	BIQ_SET_SDADCPATH(BIQ,BIQ_CTL_SDADC_DOWNSAMPLE2X);
	// Set down sample rate.
    if(g_SampleRate == SAMPLE_RATE_16K)
        SDADC_SET_DSRATIO(SDADC,SDADC_DS_RATION_64);			
    else if(g_SampleRate == SAMPLE_RATE_48K)    
        SDADC_SET_DSRATIO(SDADC,SDADC_DS_RATION_32);			
	// Set SDADC sample rate.
	SDADC_SetAudioSampleRate(g_SampleRate);
	// Set Fifo interrupt level.
	SDADC_SET_FIFOINTLEVEL(SDADC, SDADC_INT_SAMPLES);	
	// Enable fifo interrupt
	SDADC_EnableInt(SDADC_FIFO_INT);
	
	NVIC_EnableIRQ(SDADC_IRQn);
	
	printf("SDADC Sample rate %d Hz.\n\n",SDADC_GetAudioSampleRate());
		
	SDADC_START_CONV(SDADC);
}


void UART0_Init(void)
{
    // Enable UART Module Clock 
    CLK_EnableModuleClock(UART0_MODULE);
    // Configure UART0 and Set UART0 Baud Rate 
    UART_Open(UART0, 115200);
    // Set GPD Multi-function Pins for UART0 TXD(PD.8) and RXD(PD.9) 
    SYS->GPD_MFP  = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD8MFP_Msk|SYS_GPD_MFP_PD9MFP_Msk) ) | (SYS_GPD_MFP_PD8MFP_UART0_TX|SYS_GPD_MFP_PD9MFP_UART0_RX);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
	int8_t item;
	System_Initiate();
	UART0_Init();
	
	printf("\r\n\r\nCPU @ %d Hz\r\n", SystemCoreClock);
	printf("+-------------------------------------------------+\r\n");
	printf("|  Demo Aanlog Mic input to SDADC                 |\r\n");
	printf("|   (Direct output to Headphone)                  |\r\n");
	printf("+-------------------------------------------------+\r\n");
	
	
	printf("+Press 'Enter' to START.\r\n");
	getchar();
	
	ANA_Init();
	SDADC_Init();
	DAC_Init();
	
		printf("+------------------------------------------------+\r\n");
		printf("|  Select audio sample rate                      |\r\n");
		printf("+------------------------------------------------+\r\n");
		printf("|  1.Sample rate = 48k Hz                        |\r\n");
		printf("|  2.Sample rate = 16k Hz                        |\r\n");
		printf("+------------------------------------------------+\r\n");
		
		item=getchar();
		
		switch(item)
		{
			case'1':
				g_SampleRate = SAMPLE_RATE_48K;
			break;
			
			case'2':
				g_SampleRate = SAMPLE_RATE_16K;
			break;
			
			default:
				g_SampleRate = SAMPLE_RATE_48K;
			break;
		}

		DAC_Start();
		SDADC_Start();

		while(1);
 
}
