/**************************************************************************//**
 * @file     clk.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/10/12 2:35p $
 * @brief    I91500 CLK driver source file
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "I91500.h"

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_CLK_Driver CLK Driver
  @{
*/

/** @addtogroup I91500_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/
#define FREQ_4MHZ          4000000UL
#define FREQ_8MHZ          8000000UL
#define FREQ_16MHZ         16000000UL
#define FREQ_50MHZ         50000000UL
#define FREQ_64MHZ         64000000UL
#define FREQ_100MHZ        100000000UL

// Cal abs a and b.
#define CLK_ABS(a,b)                    ((a>=b)?(a-b):(b-a))

/**
  * @brief  Enters device into Deep Power Down mode and selects the acceptable causes for wakeup.
  * @param  u32DPDWakeupMode  PDP wake up mode.
  *         - \ref  CLK_DPDWAKEUP_PINOSC10K
  *         - \ref  CLK_DPDWAKEUP_PIN
  *         - \ref  CLK_DPDWAKEUP_OSC10K
  *         - \ref  CLK_DPDWAKEUP_POR
  * @param  u32TimerSel  Wakeup time selection from OSC 10k.
  *         - \ref  CLK_DPDWAKETIME_12ms
  *         - \ref  CLK_DPDWAKETIME_25ms
  *         - \ref  CLK_DPDWAKETIME_50ms
  *         - \ref  CLK_DPDWAKETIME_100ms
	*         - \ref  CLK_DPDWAKETIME_400ms  
	*         - \ref  CLK_DPDWAKETIME_800ms  
	*         - \ref  CLK_DPDWAKETIME_16000ms
	*         - \ref  CLK_DPDWAKETIME_65000ms
  * @return None
  */
void CLK_DeepPowerDown(uint32_t u32DPDWakeupMode, uint32_t u32TimerSel)
{
	uint8_t u8Lock = SYS_Unlock();
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	CLK->PWRCTL |= CLK_PWRCTL_DPDEN_Msk; //Go into Deep Power Down upon WFI/WFE command
	CLK->PWRCTL |= u32TimerSel;	 //Sets wakeup timer time

	switch(u32DPDWakeupMode)
	{
		case CLK_DPDWAKEUP_PINOSC10K:	   //Wakeup by Pin or OSC10k count 
			break;                         //Pin and OSC10K are enabled above
		    
		case CLK_DPDWAKEUP_PIN:		                           //Wakeup by Pin 
			CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_WK10KEN_Pos;    //Disable OSC10K Wakeup
			break;

		case CLK_DPDWAKEUP_OSC10K:	                           //Wakeup by OSC10k count 
 			CLK->PWRCTL |= 0x1ul << CLK_PWRCTL_WKPINEN_Pos;    //Disable PIN Wakeup
			break;
			
		case CLK_DPDWAKEUP_POR:          	                                   //Wakeup by Power On Wakeup only  
		    CLK->PWRCTL |= (CLK_PWRCTL_WK10KEN_Msk|CLK_PWRCTL_WKPINEN_Msk);    //Disable PIN Wakeup and OSC10K Wakeup
		    break;
		
	}
	SYS_Lock(u8Lock);
}

/**
  * @brief  Entering in Standby Power Down mode.
  * @return None
  */
void CLK_StandbyPowerDown(void)
{
	uint8_t u8Lock = SYS_Unlock();
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	CLK->PWRCTL = (CLK->PWRCTL&(~CLK_PWRCTL_FLASHEN_Msk))|(0x02<<CLK_PWRCTL_FLASHEN_Pos);
	CLK->PWRCTL |= CLK_PWRCTL_STOPEN_Msk;
	SYS_Lock(u8Lock);
}

/**
  * @brief  Entering in Deep sleep mode.
  * @return None
  */
void CLK_DeepSleep(void)
{
	uint8_t u8Lock = SYS_Unlock();
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	SYS_Lock(u8Lock);
}

/**
  * @brief  Entering in Sleep mode.
  * @return None
  */
void CLK_Sleep(void)
{
	uint8_t u8Lock = SYS_Unlock();
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function enable clock source
  * @param  u32ClkMask is clock source mask:
  *         - \ref CLK_PWRCTL_HXTEN_Msk
  *         - \ref CLK_PWRCTL_HIRCEN_Msk
  *         - \ref CLK_PWRCTL_LIRCEN_Msk
  * @return None
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL |= u32ClkMask;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function disable clock source
  * @param  u32ClkMask is clock source mask:
  *         - \ref CLK_PWRCTL_HXTEN_Msk
  *         - \ref CLK_PWRCTL_HIRCEN_Msk
  *         - \ref CLK_PWRCTL_LIRCEN_Msk
  * @return None
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->PWRCTL &= ~u32ClkMask;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function set HCLK clock source and HCLK clock dividerO
  * @param  u32ClkSrc is HCLK clock source :
  *         - \ref CLK_CLKSEL0_HCLKSEL_HXT
  *         - \ref CLK_CLKSEL0_HCLKSEL_PLLFOUT
  *         - \ref CLK_CLKSEL0_HCLKSEL_LIRC
  *         - \ref CLK_CLKSEL0_HCLKSEL_HIRC
  * @param  u32ClkDiv is HCLK clock divider:
  *         - \ref CLK_CLKDIV_HCLK(x)
  * @return None
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | u32ClkSrc;
	CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | u32ClkDiv;
	SYS_Lock(u8Lock);
	SystemCoreClockUpdate();
}

/**
  * @brief      Get HCLK frequency  
  * @return     HCLK frequency
  * @details    This function get HCLK frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHCLKFreq(void)
{
	SystemCoreClockUpdate();
	return SystemCoreClock;
}

/**
  * @brief     Set HIRC frequency.
  * @param[in] u32Frequency is selected frequency of HIRC
  *            - \ref CLK_CLKSEL0_HIRCSEL_49M_VCC33
  *            - \ref CLK_CLKSEL0_HIRCSEL_48M_VCC33
  *            - \ref CLK_CLKSEL0_HIRCSEL_49M_VCC18  
  * @return None
  */
void CLK_SetHIRCFrequency(uint32_t u32Frequency)
{
	uint8_t u8Lock = SYS_Unlock();
	CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_OSCFSEL_Msk) | u32Frequency;
	SYS_Lock(u8Lock);
	if( u32Frequency == CLK_CLKSEL0_HIRCSEL_48M_VCC33 )
		g_u32HIRC = 48000000UL; 
	else
		g_u32HIRC = 49152000UL;
	SystemCoreClockUpdate();
}


/**
  * @brief     Set MCLKI frequency.
  * @param[in] u32Frequency is setting frequency of MCLKI
  * @return None
  */
void CLK_SetMCLKIFrequency(uint32_t u32Frequency)
{
	// Set frequency into global variable.
	g_u32MCLKI = u32Frequency;
	
	SystemCoreClockUpdate();
}

/**
  * @brief     Get MCLKI frequency.
  * @return     MCLKI frequency
  * @details    This function get MCLKI frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetMCLKIFrequency(void)
{
	// Set frequency into global variable.
	return g_u32MCLKI;
}

/**
  * @brief     Set XCLK frequency.
  * @param[in] u32SelCLK is selected clock source of XCLK.
  *            - \ref CLK_CLKSEL2_XCLKSEL_MCLKI
  *            - \ref CLK_CLKSEL2_XCLKSEL_I2SBCLK
  * @param[in] u32SourceFrequency is frequency of clock source(BCLK or MCLKI) for XCLK
  * @param[in] u32Multi is selected multi of XCLK.
  *            - \ref CLK_XCLKCTL_XCLKMUL_x1
  *            - \ref CLK_XCLKCTL_XCLKMUL_x2
  *            - \ref CLK_XCLKCTL_XCLKMUL_x4
  *            - \ref CLK_XCLKCTL_XCLKMUL_x8
  * @return None
  */
void CLK_SetXCLKFrequency(uint32_t u32SelCLK, uint32_t u32SourceFrequency, uint32_t u32Multi)
{
	
	if( u32SelCLK == CLK_CLKSEL2_XCLKSEL_I2SBCLK )
	{
		CLK->CLKSEL2 = (CLK->CLKSEL2&~CLK_CLKSEL2_XCLKSEL_Msk)|CLK_CLKSEL2_XCLKSEL_I2SBCLK;
		// Set frequency into global variable.
		g_u32BCLK = u32SourceFrequency;
	}
	else
	{
		CLK->CLKSEL2 = (CLK->CLKSEL2&~CLK_CLKSEL2_XCLKSEL_Msk)|CLK_CLKSEL2_XCLKSEL_MCLKI;
		// Set frequency into global variable.
		g_u32MCLKI = u32SourceFrequency;
	}

	CLK->XCLKCTL = (CLK->XCLKCTL&~CLK_XCLKCTL_XCLKMUL_Msk)|u32Multi;
	
	SystemCoreClockUpdate();
}

/**
  * @brief     Get XCLK frequency.
  * @return     XCLK frequency
  * @details    This function get XCLK frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetXCLKFrequency(void)
{
	uint32_t u32Freq, u32MultiTalbe[] = {1,2,4,8};
	
	if( (CLK->CLKSEL2&CLK_CLKSEL2_XCLKSEL_Msk) == CLK_CLKSEL2_XCLKSEL_I2SBCLK )
		u32Freq = g_u32BCLK;
	else
		u32Freq = g_u32MCLKI;
	
	u32Freq *= u32MultiTalbe[(CLK->XCLKCTL&CLK_XCLKCTL_XCLKMUL_Msk)>>CLK_XCLKCTL_XCLKMUL_Pos];
	
	// Set frequency into global variable.
	return u32Freq;
}


/**
  * @brief  This function enable module clock
  * @param  u32ModuleIdx is module index :
  *         - \ref PDMA_MODULE
  *         - \ref CPD_MODULE
  *         - \ref ISP_MODULE
  *         - \ref WDT_MODULE
  *         - \ref TMR0_MODULE
  *         - \ref TMR1_MODULE
  *         - \ref TMR2_MODULE
  *         - \ref I2C0_MODULE
  *         - \ref I2C1_MODULE
  *         - \ref SPI0_MODULE
  *         - \ref SPI1_MODULE
  *         - \ref I2S0_MODULE
  *         - \ref UART0_MODULE
  *         - \ref UART1_MODULE
  *         - \ref BIQ_MODULE
  *         - \ref PWM0_MODULE
  *         - \ref PWM1_MODULE
  *         - \ref USBD_MODULE
  *         - \ref SARADC_MODULE
  *         - \ref SDADC_MODULE
  *         - \ref DAC_MODULE
  *         - \ref ANA_MODULE
  * @return None
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
	uint8_t u8Lock = SYS_Unlock();
	*(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_AHPBCLK(u32ModuleIdx)*4))  |= 1<<MODULE_IP_EN_Pos(u32ModuleIdx);
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function disable module clock
  * @param  u32ModuleIdx is module index :
  *         - \ref PDMA_MODULE
  *         - \ref CPD_MODULE
  *         - \ref ISP_MODULE
  *         - \ref WDT_MODULE
  *         - \ref TMR0_MODULE
  *         - \ref TMR1_MODULE
  *         - \ref TMR2_MODULE
  *         - \ref I2C0_MODULE
  *         - \ref I2C1_MODULE
  *         - \ref SPI0_MODULE
  *         - \ref SPI1_MODULE
  *         - \ref I2S0_MODULE
  *         - \ref UART0_MODULE
  *         - \ref UART1_MODULE
  *         - \ref BIQ_MODULE
  *         - \ref PWM0_MODULE
  *         - \ref PWM1_MODULE
  *         - \ref USBD_MODULE
  *         - \ref SARADC_MODULE
  *         - \ref SDADC_MODULE
  *         - \ref DAC_MODULE
  *         - \ref ANA_MODULE
  * @return None
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
	uint8_t u8Lock = SYS_Unlock();
	*(volatile uint32_t *)((uint32_t)&CLK->AHBCLK+(MODULE_AHPBCLK(u32ModuleIdx)*4))  &= ~(1<<MODULE_IP_EN_Pos(u32ModuleIdx));
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function set selected module clock source and module clock divider
  * @param  u32ModuleIdx is module index.
  * @param  u32ClkSrc is module clock source.
  * @param  u32ClkDiv is module clock divider.
  * @return None
  * @details Valid parameter combinations listed in following table:
  *
  * |Module index          |Clock source                                 |Divider                       |
  * | :------------------- | :------------------------------------------ | :-------------------------   |
  * |\ref WDT_MODULE       |\ref CLK_CLKSEL1_WDTSEL_LIRC                 | x                            |
  * |\ref WDT_MODULE       |\ref CLK_CLKSEL1_WDTSEL_HCLK_DIV2048         | x                            |
  * |\ref SARADC_MODULE    |\ref CLK_CLKSEL1_SARADCSEL_HXT               |\ref CLK_CLKDIV_SARADC(x)     |
  * |\ref SARADC_MODULE    |\ref CLK_CLKSEL1_SARADCSEL_PLLFOUT           |\ref CLK_CLKDIV_SARADC(x)     |
  * |\ref SARADC_MODULE    |\ref CLK_CLKSEL1_SARADCSEL_PCLK              |\ref CLK_CLKDIV_SARADC(x)     |
  * |\ref SARADC_MODULE    |\ref CLK_CLKSEL1_SARADCSEL_HIRC              |\ref CLK_CLKDIV_SARADC(x)     |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_HXT                 | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_PCLK                | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_EXT                 | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_LIRC                | x                            |
  * |\ref TMR0_MODULE      |\ref CLK_CLKSEL1_TMR0SEL_HIRC                | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_HXT                 | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_PCLK                | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_EXT                 | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_LIRC                | x                            |
  * |\ref TMR1_MODULE      |\ref CLK_CLKSEL1_TMR1SEL_HIRC                | x                            |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_HXT                 | x                            |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_PCLK                | x                            |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_EXT                 | x                            |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_LIRC                | x                            |
  * |\ref TMR2_MODULE      |\ref CLK_CLKSEL1_TMR2SEL_HIRC                | x                            |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL1_I2S0SEL_HXT                  | x                            |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL1_I2S0SEL_PLLFOUT              | x                            |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL1_I2S0SEL_PCLK                 | x                            |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL1_I2S0SEL_HIRC                 | x                            |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL1_I2S0SEL_MCLKI                | x                            |
  * |\ref I2S0_MODULE      |\ref CLK_CLKSEL1_I2S0SEL_XLCK                 | x                            |
  * |\ref UART0_MODULE     |\ref CLK_CLKSEL1_UART0SEL_HXT                |\ref CLK_CLKDIV_UART0(x)      |
  * |\ref UART0_MODULE     |\ref CLK_CLKSEL1_UART0SEL_PLLFOUT            |\ref CLK_CLKDIV_UART0(x)      |
  * |\ref UART0_MODULE     |\ref CLK_CLKSEL1_UART0SEL_HIRC               |\ref CLK_CLKDIV_UART0(x)      |
  * |\ref UART1_MODULE     |\ref CLK_CLKSEL1_UART1SEL_HXT                |\ref CLK_CLKDIV_UART1(x)      |
  * |\ref UART1_MODULE     |\ref CLK_CLKSEL1_UART1SEL_PLLFOUT            |\ref CLK_CLKDIV_UART1(x)      |
  * |\ref UART1_MODULE     |\ref CLK_CLKSEL1_UART1SEL_HIRC               |\ref CLK_CLKDIV_UART1(x)      |
  * |\ref PWM0_MODULE      |\ref CLK_CLKSEL1_PWM0SEL_HXT                 | x                            |
  * |\ref PWM0_MODULE      |\ref CLK_CLKSEL1_PWM0SEL_PLLFOUT             | x                            |
  * |\ref PWM0_MODULE      |\ref CLK_CLKSEL1_PWM0SEL_PCLK                | x                            |
  * |\ref PWM0_MODULE      |\ref CLK_CLKSEL1_PWM0SEL_HIRC                | x                            |
  * |\ref PWM1_MODULE      |\ref CLK_CLKSEL1_PWM1SEL_HXT                 | x                            |
  * |\ref PWM1_MODULE      |\ref CLK_CLKSEL1_PWM1SEL_PLLFOUT             | x                            |
  * |\ref PWM1_MODULE      |\ref CLK_CLKSEL1_PWM1SEL_PCLK                | x                            |
  * |\ref PWM1_MODULE      |\ref CLK_CLKSEL1_PWM1SEL_HIRC                | x                            |
  * |\ref USBD_MODULE      |\ref CLK_CLKSEL2_USBDSEL_HIRC                |\ref CLK_CLKDIV_USBD(x)       |
  * |\ref USBD_MODULE      |\ref CLK_CLKSEL2_USBDSEL_PLLFOUT             |\ref CLK_CLKDIV_USBD(x)       |
  * |\ref DAC_MODULE       |\ref CLK_CLKSEL2_DACSEL_HXT                  |\ref CLK_CLKDIV_DAC(x)        |
  * |\ref DAC_MODULE       |\ref CLK_CLKSEL2_DACSEL_PLLFOUT              |\ref CLK_CLKDIV_DAC(x)        |
  * |\ref DAC_MODULE       |\ref CLK_CLKSEL2_DACSEL_PCLK                 |\ref CLK_CLKDIV_DAC(x)        |
  * |\ref DAC_MODULE       |\ref CLK_CLKSEL2_DACSEL_HIRC                 |\ref CLK_CLKDIV_DAC(x)        |
  * |\ref DAC_MODULE       |\ref CLK_CLKSEL2_DACSEL_MCLKI                |\ref CLK_CLKDIV_DAC(x)        |
  * |\ref DAC_MODULE       |\ref CLK_CLKSEL2_DACSEL_XCLK                 |\ref CLK_CLKDIV_DAC(x)        |
  * |\ref SDADC_MODULE     |\ref CLK_CLKSEL2_SDADCSEL_HXT                |\ref CLK_CLKDIV_SDADC(x)      |
  * |\ref SDADC_MODULE     |\ref CLK_CLKSEL2_SDADCSEL_PLLFOUT            |\ref CLK_CLKDIV_SDADC(x)      |
  * |\ref SDADC_MODULE     |\ref CLK_CLKSEL2_SDADCSEL_PCLK               |\ref CLK_CLKDIV_SDADC(x)      |
  * |\ref SDADC_MODULE     |\ref CLK_CLKSEL2_SDADCSEL_HIRC               |\ref CLK_CLKDIV_SDADC(x)      |
  * |\ref SDADC_MODULE     |\ref CLK_CLKSEL2_SDADCSEL_MCLKI              |\ref CLK_CLKDIV_SDADC(x)      |
  * |\ref SDADC_MODULE     |\ref CLK_CLKSEL2_SDADCSEL_XCLK               |\ref CLK_CLKDIV_SDADC(x)      |
  */
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
	uint32_t u32tmp=0,u32sel=0,u32div=0;
  
	uint8_t u8Lock = SYS_Unlock();
	
	if(MODULE_CLKDIV_Msk(u32ModuleIdx)!=MODULE_NoMsk)
	{
		u32div =(uint32_t)&CLK->CLKDIV0+((MODULE_CLKDIV(u32ModuleIdx))*4);
		u32tmp = *(volatile uint32_t *)(u32div);
		u32tmp = ( u32tmp & ~(MODULE_CLKDIV_Msk(u32ModuleIdx)<<MODULE_CLKDIV_Pos(u32ModuleIdx)) ) | u32ClkDiv;
		*(volatile uint32_t *)(u32div) = u32tmp;
    }	

	if(MODULE_CLKSEL_Msk(u32ModuleIdx)!=MODULE_NoMsk)
	{
		if( MODULE_CLKSEL(u32ModuleIdx) == 2 )
			u32sel = (uint32_t)&CLK->CLKSEL2;
		else
			u32sel = (uint32_t)&CLK->CLKSEL0+((MODULE_CLKSEL(u32ModuleIdx))*4);
		u32tmp = *(volatile uint32_t *)(u32sel);
		u32tmp = ( u32tmp & ~(MODULE_CLKSEL_Msk(u32ModuleIdx)<<MODULE_CLKSEL_Pos(u32ModuleIdx)) ) | u32ClkSrc;
		*(volatile uint32_t *)(u32sel) = u32tmp;
	}
	
	SYS_Lock(u8Lock);
}

/**
  * @brief      Disable PLL
  * @param      None
  * @return     None
  * @details    This function set PLL in Power-down mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisablePLL(void)
{
	if((CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk) == CLK_CLKSEL0_HCLKSEL_PLLFOUT)
		return ;
    else
		CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;
}

/**
  * @brief      Get PLL frequency
  * @param[in]  u32PllClkSrc is PLL clock source. Including :
  *             - \ref CLK_PLLCTL_PLLSRC_HXT
  *             - \ref CLK_PLLCTL_PLLSRC_HIRC
  * @param[in]  u32PllFreq is PLL frequency.
  * @return     PLL frequency
  * @details    This function is used to configure PLLCTL register to set specified PLL frequency. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_GetPLLFreq(void)
{
	uint32_t u32ClkSrc;
    uint8_t au8NoTbl[4] = {1, 2, 4, 4};
	
	if( (CLK->PLLCTL&CLK_PLLCTL_PLLSRC_Msk)==CLK_PLLCTL_PLLSRC_HIRC )
		u32ClkSrc = g_u32HIRC;
	else
		u32ClkSrc = __HXT;
	
	return (u32ClkSrc * ((CLK->PLLCTL&CLK_PLLCTL_FBDIV_Msk)>>CLK_PLLCTL_FBDIV_Pos)) / (au8NoTbl[((CLK->PLLCTL&CLK_PLLCTL_OUTDIV_Msk)>>CLK_PLLCTL_OUTDIV_Pos)] * ((CLK->PLLCTL&CLK_PLLCTL_INDIV_Msk)>>CLK_PLLCTL_INDIV_Pos));
}

/**
  * @brief      Enable PLL frequency
  * @param[in]  u32PllClkSrc is PLL clock source. Including :
  *             - \ref CLK_PLLCTL_PLLSRC_HXT
  *             - \ref CLK_PLLCTL_PLLSRC_HIRC
  * @param[in]  u32PllFreq is PLL frequency.
  * @return     PLL frequency
  * @details    This function is used to configure PLLCTL register to set specified PLL frequency. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq)
{
	uint32_t u32PllSrcClk, u32NR, u32NF, u32NO, u32CLK_SRC;
	uint32_t u32Fref,u32Fvco;
	uint32_t u32Tmp1, u32Tmp2, u32Tmp3, u32Min=0, u32MinNF=0, u32MinNR=0, u32MinNO=0;

	if((CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk) == CLK_CLKSEL0_HCLKSEL_PLLFOUT)
		return 0;
	else
		/* Disable PLL first to avoid unstable when setting PLL */
		CLK_DisablePLL();
	
	if(u32PllFreq > FREQ_50MHZ ) 
		u32PllFreq = FREQ_50MHZ;
	else if(u32PllFreq < FREQ_16MHZ)
		u32PllFreq = FREQ_16MHZ;

	/* PLL source clock is from HXT */
	if(u32PllClkSrc == CLK_PLLCTL_PLLSRC_HXT) 
	{
		/* Enable HXT clock */
		CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;
		/* Wait for HXT clock ready */
		CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
		/* Select PLL source clock from HXT */
		u32CLK_SRC = CLK_PLLCTL_PLLSRC_HXT;
		/* Input frequency */
		u32PllSrcClk = __HXT;
	}
	/* PLL source clock is from XCLK */
	else if(u32PllClkSrc == CLK_PLLCTL_PLLSRC_HXT) 
	{
		u32PllSrcClk = CLK_GetXCLKFrequency();
		
		if( u32PllSrcClk==0 )
			return 0;
		else
			u32CLK_SRC = CLK_PLLCTL_PLLSRC_XCLK;
	}
	/* PLL source clock is from HIRC */
	else 
	{
		/* Enable HIRC clock */
		CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
		/* Wait for HIRC clock ready */
		CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
		/* Select PLL source clock from HIRC */
		u32CLK_SRC = CLK_PLLCTL_PLLSRC_HIRC;
		/* Input frequency */
		u32PllSrcClk = g_u32HIRC;
	}	
	
	for( u32NR=1; u32NR<=16; u32NR++ )
	{
		u32Fref = u32PllSrcClk/u32NR;
		
		if( u32Fref >= FREQ_4MHZ && u32Fref <= FREQ_8MHZ)
		{
			for( u32NF=1; u32NF<=64; u32NF++ )
			{
				u32Fvco = (u32PllSrcClk*u32NF)/u32NR;
				
				if( u32Fvco >= FREQ_64MHZ && u32Fvco <= FREQ_100MHZ )
				{
					for( u32NO=1; u32NO<=4; u32NO*=2 )
					{
						u32Tmp1=u32PllFreq*u32NR*u32NO;
						u32Tmp2=u32PllSrcClk*u32NF;
						
						if( u32Tmp1==u32Tmp2 )
						{
							/* Enable and apply new PLL setting. */
							CLK->PLLCTL = u32CLK_SRC | ((u32NO - 1UL) << CLK_PLLCTL_OUTDIV_Pos) | (u32NR << CLK_PLLCTL_INDIV_Pos) | (u32NF << CLK_PLLCTL_FBDIV_Pos); 
							/* Wait for PLL clock stable */
							CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);
							/* Actual PLL output clock frequency */
							return CLK_GetPLLFreq();				
						}
						else
						{
							u32Tmp3 = CLK_ABS(u32Tmp1,u32Tmp2);
							
							if( u32Tmp3<u32Min )
							{
								u32Min = u32Tmp3;
								u32MinNF = u32NF;
								u32MinNO = u32NO;
								u32MinNR = u32NR;
							}
						}
					}
				}
			}
		}
	}
	
	/* Enable and apply new PLL setting. */
	CLK->PLLCTL = u32CLK_SRC | ((u32MinNO - 1UL) << CLK_PLLCTL_OUTDIV_Pos) | (u32MinNR << CLK_PLLCTL_INDIV_Pos) | (u32MinNF << CLK_PLLCTL_FBDIV_Pos); 
	/* Wait for PLL clock stable */
	CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);
	/* Actual PLL output clock frequency */
	return CLK_GetPLLFreq();
}

/**
  * @brief      Set SysTick clock source
  * @param[in]  u32ClkSrc is module clock source. Including:
  *             - \ref CLK_CLKSEL0_STICKSEL_HXT
  *             - \ref CLK_CLKSEL0_STICKSEL_HXT_DIV2
  *             - \ref CLK_CLKSEL0_STICKSEL_HCLK_DIV2
  *             - \ref CLK_CLKSEL0_STICKSEL_HIRC_DIV2
  * @return     None
  * @details    This function set SysTick clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc)
{
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STICKSEL_Msk) | u32ClkSrc;
}

/**
  * @brief      Enable System Tick counter
  * @param[in]  u32ClkSrc is System Tick clock source. Including:
  *             - \ref CLK_CLKSEL0_STICKSEL_HXT
  *             - \ref CLK_CLKSEL0_STICKSEL_HXT_DIV2
  *             - \ref CLK_CLKSEL0_STICKSEL_HCLK_DIV2
  *             - \ref CLK_CLKSEL0_STICKSEL_HIRC_DIV2
  * @param[in]  u32Count is System Tick reload value. It could be 0~0xFFFFFF.
  * @return     None
  * @details    This function is to set System Tick clock source, reload value, enable System Tick counter and interrupt. \n
  *             The register write-protection function should be disabled before using this function. 
  */
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count) 
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0;    

    /* Set System Tick clock source */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STICKSEL_Msk) | u32ClkSrc; 

    /* Set System Tick reload value */
    SysTick->LOAD = u32Count;   
    
    /* Clear System Tick current value and counter flag */
    SysTick->VAL = 0;           
    
    /* Set System Tick interrupt enabled and counter enabled */    
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;       
}

/**
  * @brief      Disable System Tick counter
  * @param      None 
  * @return     None
  * @details    This function is to disable System Tick counter.
  */
void CLK_DisableSysTick(void) 
{    
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0;    
}

/**
  * @brief      This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including :
  *             - \ref CLK_STATUS_HXTSTB_Msk
  *             - \ref CLK_STATUS_HIRCSTB_Msk
  *             - \ref CLK_STATUS_LIRCSTB_Msk
  *             - \ref CLK_STATUS_PLLSTB_Msk 
  *             - \ref CLK_STATUS_XCLKSTB_Msk
  * @retval     0  clock is not stable
  * @retval     1  clock is stable
  * @details    To wait for clock ready by specified clock source stable flag or timeout (~300ms)
  */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask)
{    
    int32_t i32TimeOutCnt = 2160000;

    while((CLK->STATUS & u32ClkMask) != u32ClkMask)
    {
        if(i32TimeOutCnt-- <= 0)
            return 0;
    }

    return 1;
}

/*@}*/ /* end of group I91500_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_CLK_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
