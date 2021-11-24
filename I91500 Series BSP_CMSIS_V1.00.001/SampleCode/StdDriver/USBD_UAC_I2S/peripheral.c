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
#include "nau88l21_drv.h"

//----------------------------------------------------------------------------
//  Global variables for Control Pipe
//---------------------------------------------------------------------------- 

volatile uint8_t g_au8RecordPingpongBuff[(REC_RATE_MAX / 1000) * PLAY_CHANNELS * PLAY_RES_BYTE * 2];
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
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLLFOUT,  CLK_CLKDIV_HCLK(1));		
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
    // Set USBD clock divid 
    CLK_SetModuleClock(USBD_MODULE,CLK_CLKSEL2_USBDSEL_PLLFOUT,CLK_CLKDIV_USBD(1));		
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
 * @brief       I2S initialization
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to initialize I2S IP
 */
void I2S0_Init(void)
{
	// Set I2S0 MFP
	SYS->GPC_MFP = (SYS->GPC_MFP&~(SYS_GPC_MFP_PC14MFP_Msk|SYS_GPC_MFP_PC13MFP_Msk|SYS_GPC_MFP_PC12MFP_Msk|SYS_GPC_MFP_PC11MFP_Msk|SYS_GPC_MFP_PC10MFP_Msk))|
                   (SYS_GPC_MFP_PC14MFP_I2S0_DI|SYS_GPC_MFP_PC13MFP_I2S0_DO|SYS_GPC_MFP_PC12MFP_I2S0_BCLK|SYS_GPC_MFP_PC11MFP_I2S0_LRCK|SYS_GPC_MFP_PC10MFP_I2S0_MCLK);
	// Enable I2S0 clock.
	CLK_EnableModuleClock(I2S0_MODULE);
	// Select I2S0 clock.
    CLK_SetModuleClock(I2S0_MODULE, CLK_CLKSEL1_I2S0SEL_HIRC, NULL);

	// Open I2S0 hardware IP
	I2S_Open(I2S0, I2S_MASTER, PLAY_RATE_MAX, I2S_CHWIDTH_32, I2S_STEREO, I2S_FORMAT_I2S);
	I2S0->CTL0 = (I2S0->CTL0 & ~I2S_CTL0_DATWIDTH_Msk) | I2S_DATABIT_16;
    // I2S0 Configuration
	I2S_SET_PCMSYNC(I2S0, I2S_PCMSYNC_BCLK);
	I2S_SET_MONO_RX_CHANNEL(I2S0, I2S_MONO_RX_RIGHT);
	I2S_SET_STEREOORDER(I2S0, I2S_ORDER_EVENLOW);  
    I2S_EnableMCLK(I2S0, (PLAY_RATE_MAX * 256));
	// Set channel width.
	I2S_SET_CHWIDTH(I2S0, I2S_CHWIDTH_32);

    // Enable I2S
    I2S_ENABLE(I2S0);
}

/**
 * @brief       I2S0 TX start to operate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to start I2S0 TX
 */
void I2S0_TX_Start(void)
{
    I2S_CLR_TX_FIFO(I2S0);
    I2S_ENABLE_TXDMA(I2S0);
    I2S_ENABLE_TX(I2S0);
}

/**
 * @brief       I2S0 TX stop to operate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to stop I2S0 TX
 */
void I2S0_TX_Stop(void)
{
    I2S_CLR_TX_FIFO(I2S0);
    I2S_DISABLE_TXDMA(I2S0);
	I2S_DISABLE_TX(I2S0);
}

/**
 * @brief       I2S0 RX start to operate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to start I2S0 RX
 */
void I2S0_RX_Start(void)
{
	I2S_CLR_RX_FIFO(I2S0);
    I2S_ENABLE_RXDMA(I2S0);
	I2S_ENABLE_RX(I2S0);
}

/**
 * @brief       I2S0 RX stop to operate
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to stop I2S0 RX
 */
void I2S0_RX_Stop(void)
{
    I2S_CLR_RX_FIFO(I2S0);
	I2S_DISABLE_RXDMA(I2S0);
    I2S_DISABLE_RX(I2S0);  
}

/**
 * @brief       PDMA configure for I2S TX
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure PDMA for TX
 */
void PDMA_I2S0_TX_Init(void)
{
    PDMA_SoftwareReset(PLAY_PDMA_CH);     
    PDMA_Open( 1 << PLAY_PDMA_CH );  	// Open Channel
    PDMA_SetTransferMode(PLAY_PDMA_CH, PDMA_I2S_TX);
    PDMA_SetTransferCnt(PLAY_PDMA_CH, PDMA_WIDTH_32, 0);
    PDMATX->BCR = ((PLAY_RATE_MAX/1000) * PLAY_CHANNELS * PLAY_RES_BYTE * 2);
    PDMA_SetTransferAddr(PLAY_PDMA_CH, (uint32_t)&g_au8PlaybackPingpongBuff, PDMA_SAR_WRA, (uint32_t)&I2S0->TXFIFO, PDMA_DAR_FIX);
    PDMA_SetTransferDirection(PLAY_PDMA_CH,PDMA_SRAM_APB);	     
    PDMA_WrapIntSelect(PLAY_PDMA_CH,PDMA_BOTH_WRAP_MODE); //Both half and done interrupts generated     
    PDMA_EnableInt(PLAY_PDMA_CH, PDMA_IER_WAR_IE_Msk);  
    PDMA_Trigger(PLAY_PDMA_CH);      
}

/**
 * @brief       PDMA configure for I2S RX
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure PDMA for I2S RX
 */
void PDMA_I2S0_RX_Init(void)
{  
    PDMA_SoftwareReset(REC_PDMA_CH);  
	PDMA_Open(1 << REC_PDMA_CH);      
    PDMA_SetTransferMode(REC_PDMA_CH, PDMA_I2S_RX);
    PDMA_SetTransferCnt(REC_PDMA_CH, PDMA_WIDTH_32, 0);	
    PDMARX->BCR = ((REC_RATE_MAX/1000) * REC_CHANNELS * REC_RES_BYTE * 2);
    PDMA_SetTransferAddr(REC_PDMA_CH, (uint32_t)&I2S0->RXFIFO, PDMA_SAR_FIX, (uint32_t)&g_au8RecordPingpongBuff, PDMA_DAR_WRA);	
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
 * @details     This function is used to configure audio codec PGA
 */
void RecordVolumeControl(void)
{
    if(g_i16PreRecVolume != g_usbd_RecVolumeR)
    {        
        g_i16PreRecVolume = g_usbd_RecVolumeR;
        
        nau88l21_pga_control((uint8_t)(g_usbd_RecVolumeR >> 8), (uint8_t)(g_usbd_RecVolumeR >> 8));  
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
            PDMA_I2S0_TX_Init();
            I2S0_TX_Start();
        }
    }
    else
    {
        if(g_u8PlaybackDeviceEn)
        {
            g_u8PlaybackDeviceEn = 0; 
            I2S0_TX_Stop();            
        }      
    }
    if(g_u8USBRecEn)
    {
        if(!g_u8RecordDeviceEn)
        {
            g_u8RecordDeviceEn = 1;
            RecordPingpongBuffClear();
            PDMA_I2S0_RX_Init();
            I2S0_RX_Start();
        }
    }
    else
    {
        if(g_u8RecordDeviceEn)
        {
            g_u8RecordDeviceEn = 0;
            I2S0_RX_Stop();
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
 * @brief       I2C initialization
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to initialize I2C IP
 */
void I2C0_Init(void)
{
    // Reset module. 
    SYS_ResetModule(I2C0_RST);
    // Enable I2C0 module clock. 
    CLK_EnableModuleClock(I2C0_MODULE);
    // Open I2C module and set bus clock. 
    I2C_Open(I2C0, 400000);

    /* GPIO multi-function. */
    SYS->GPC_MFP  = (SYS->GPC_MFP & ~(SYS_GPC_MFP_PC6MFP_Msk)) | SYS_GPC_MFP_PC6MFP_I2C0_SCL;
    SYS->GPC_MFP  = (SYS->GPC_MFP & ~(SYS_GPC_MFP_PC7MFP_Msk)) | SYS_GPC_MFP_PC7MFP_I2C0_SDA;
}


/**
  * @brief      Write multi bytes to Slave
  *
  * @param[in]  u8dev           Access Slave address(7-bit)
  * @param[in]  *pu8Buf         Pointer to array to write data to Slave
  * @param[in]  u32cnt          How many bytes need to write to Slave
  * 
  * @retval     0               Write data success
  * @retval     -1              Write data fail, or bus occurs error events
  * @retval     -2              i2c timeout, may cause by SDA is hold low by slave device
  *
  * @details    The function is used for I2C Master write multi bytes data to Slave. 
  *
  */
int32_t i2c_master_send(I2C_T *i2c, uint8_t u8dev, uint8_t *pu8Buf, uint32_t u32cnt)
{
    uint8_t u8Xfering = 1, u8Err = 0;
    uint32_t u32txLen = 0;
    uint32_t u32toutcnt = 0;
    
#if I2C_TOUT_EN
    I2C_ClearTimeoutFlag(i2c);
    I2C_EnableTimeout(i2c, 1);
#endif
    I2C_SET_CONTROL_REG(i2c, I2C_CTL_STA_SI);                      /* Send START */    
    while(u8Xfering && (u8Err == 0))
    {
//        I2C_WAIT_READY(i2c);
        while(!((i2c)->CTL & I2C_CTL_SI_Msk))
        {
#if I2C_TOUT_EN
            if(i2c->TOCTL & I2C_TOCTL_TOIF_Msk)
            {
                I2C_ClearTimeoutFlag(i2c);
                I2C_DisableTimeout(i2c);
                return -2;           
            }
#else
            u32toutcnt++;
            if(u32toutcnt > I2C_TOUT_CNT)
                return -2;
#endif
        }
        switch(I2C_GET_STATUS(i2c))
        {
            case 0x08:                                              /* START */ 
                I2C_SET_DATA(i2c, (u8dev << 1));              /* Send Slave address with write bit */
                I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);               /* Clear SI */
                break;
            case 0x18:                                              /* Master transmit address ACK */
                I2C_SET_DATA(i2c, pu8Buf[u32txLen++]);               /* Write Low byte address of register */
                I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);            
                break;
            case 0x20:                                              /* Master transmit address NACK */
            case 0x30:                                              /* Master transmit data NACK */                                                                                       
                I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);           /* Clear SI and send STOP */          
                u8Err = 1;     
                break;             
            case 0x28:                                              /* Master transmit data ACK */
                if(u32txLen < u32cnt)
                {
                    I2C_SET_DATA(i2c, pu8Buf[u32txLen++]);
                    I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI); 
                }
                else
                {
                    I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);       /* Clear SI and send STOP */
                    u8Xfering = 0;
                }                 
                break;
            default:                                                /* Unknow status */
                I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);            /* Clear SI and send STOP */             
                u8Err = 1;
                break;
        }     
    }
#if I2C_TOUT_EN
    I2C_ClearTimeoutFlag(i2c);
    I2C_DisableTimeout(i2c);  
#endif
    if(u8Err == 0)
        return 0;
    else
        return -1;                                  /* return (Success)/(Fail) status */
}

/**
  * @brief      Read multi bytes from Slave
  *
  * @param[in]  u8dev           Access Slave address(7-bit)
  * @param[out] *pu8Buf         Point to array to store data from Slave 
  * @param[in]  u32cnt          How many bytes need to read from Slave
  * 
  * @retval     0               Write data success
  * @retval     -1              Write data fail, or bus occurs error events
  * @retval     -2              i2c timeout, may cause by SDA is hold low by slave device
  *
  * @details    The function is used for I2C Master to read multi data bytes from Slave. 
  *
  *
  */
int32_t i2c_master_recv(I2C_T *i2c, uint8_t u8dev, uint8_t *pu8Buf, uint32_t u32cnt)
{
    uint8_t u8Xfering = 1, u8Err = 0;
    uint32_t u32txLen = 0;
    uint32_t u32toutcnt = 0;

#if I2C_TOUT_EN
    I2C_ClearTimeoutFlag(i2c);
    I2C_EnableTimeout(i2c, 1);
#endif
    I2C_SET_CONTROL_REG(i2c, I2C_CTL_STA_SI);                        /* Send START */
    while(u8Xfering && (u8Err == 0))
    { 
//        I2C_WAIT_READY(i2c);
        while(!((i2c)->CTL & I2C_CTL_SI_Msk))
        {
#if I2C_TOUT_EN
            if(i2c->TOCTL & I2C_TOCTL_TOIF_Msk)
            {
                I2C_ClearTimeoutFlag(i2c);
                I2C_DisableTimeout(i2c);
                return -2;           
            }
#else
            u32toutcnt++;
            if(u32toutcnt > I2C_TOUT_CNT)
                return -2;            
#endif
        }
        switch(I2C_GET_STATUS(i2c))
        {
            case 0x08:                                               /* START */ 
                I2C_SET_DATA(i2c, ((u8dev << 1) | 0x01));      /* Write SLA+R to Register I2CDAT */            	
                I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);                /* Clear SI */              
                break;
            case 0x40:                                               /* SLA+R has been transmitted and ACK has been received */
                I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);             /* Clear SI + AA*/  
                break;             
            case 0x48:                                               /* Master receive address NACK */                 
                I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);            /* Clear SI and send STOP */    
                u8Err = 1; 
                break;            
            case 0x50:                                               /* Master receive data ACK */
                pu8Buf[u32txLen++] = I2C_GET_DATA(i2c);               /* Receive Data */
                if(u32txLen < (u32cnt - 1))
                {   
                    I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);         /* Clear SI + AA*/                  
                }
                else
                {
                    I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);            /* Clear SI*/   
                }
                break;
            case 0x58:                                               /* Master receive data NACK */
                pu8Buf[u32txLen++] = I2C_GET_DATA(i2c);               /* Receive Data */
                I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);            /* Clear SI and send STOP */
                u8Xfering = 0;
                break;
            default:                                                 /* Unknow status */             
                I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);            /* Clear SI and send STOP */    
                u8Err = 1;  
                break;                
        }
    }    
#if I2C_TOUT_EN    
    I2C_ClearTimeoutFlag(i2c);
    I2C_DisableTimeout(i2c);  
#endif    
    if(u8Err == 0)
        return 0;
    else
        return -1;                                  /* return (Success)/(Fail) status */   
    
}

/**
  * @brief      i2c register write 16-bit
  *
  * @param[in]  u8dev           Specify i2c device id
  * @param[in]  u16reg          register address (16-bit)
  * @param[in]  u16val          data (16-bit)
  *
  * @retval     0               Write data success
  * @retval     -1              Write data fail, or bus occurs error events
  *
  * @details    This function can write i2c device 16-bit address and 16-bit data register.
  *
  */
int32_t i2c_reg_write_16b(I2C_T *i2c, uint8_t u8dev, uint16_t u16reg, uint16_t u16val)
{
	uint8_t au8buf[4], u8cnt = 0;
	int32_t i32ret;

	au8buf[u8cnt++] = u16reg >> 8;
	au8buf[u8cnt++] = u16reg;
    au8buf[u8cnt++] = u16val >> 8;
    au8buf[u8cnt++] = u16val;
   
    i32ret = i2c_master_send(i2c, u8dev, au8buf, u8cnt);     
    
    return i32ret;
}

/**
  * @brief      i2c register read 16-bit
  *
  * @param[in]  u8dev           Specify i2c device id
  * @param[in]  u16reg          register address (16-bit)
  * @param[in]  *pu16val        point to data address (16-bit)
  *
  * @retval     0               Write data success
  * @retval     -1              Write data fail, or bus occurs error events
  *
  * @details    This function can read i2c device 16-bit data register.
  *
  */
int32_t i2c_reg_read_16b(I2C_T *i2c, uint8_t u8dev, uint16_t u16reg, uint16_t *pu16val)
{
	uint8_t au8buf[4], u8cnt = 0;
	int32_t i32ret;
    
	au8buf[u8cnt++] = u16reg >> 8;
	au8buf[u8cnt++] = u16reg;
    
    i32ret = i2c_master_send(i2c, u8dev, au8buf, u8cnt);
    if (i32ret < 0)
        return i32ret;    

    u8cnt = 2;   
    i32ret = i2c_master_recv(i2c, u8dev, au8buf, u8cnt);
	if (i32ret < 0)
		return i32ret;

    *pu16val = au8buf[1];
    *pu16val |= ((uint8_t)au8buf[0]) << 8;
    
	return i32ret;
}
