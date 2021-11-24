/**************************************************************************//**
 * @file     DAC.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 20/11/26 07:27p $
 * @brief    I91500 DAC driver source file
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include  "Platform.h"

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_DAC_Driver DAC Driver
  @{
*/

/** @addtogroup I91500_DAC_EXPORTED_FUNCTIONS DAC Exported Functions
  @{
*/

#define DAC_MAX_SD_CLOCK_FREQ                  (6144000)   

/**
  * @brief  This function is used to get dac source clock frequency.
  * @param[in]  i2s The pointer of the specified I2S module.
  * @return I2S source clock frequency (Hz).
  * @details Return the source clock frequency according to the setting of SPI1SEL (CLKSEL2[5:4]) or SPI2SEL (CLKSEL2[7:6]).
  */
static uint32_t DAC_GetSourceClockFreq(void)
{
    switch( CLK->CLKSEL2&CLK_CLKSEL2_DACSEL_Msk )
	{
		case CLK_CLKSEL2_DACSEL_HXT :            return __HXT;
		case CLK_CLKSEL2_DACSEL_PLLFOUT :        return CLK_GetPLLFreq();
		case CLK_CLKSEL2_DACSEL_PCLK:            return CLK_GetHCLKFreq();
		case CLK_CLKSEL2_DACSEL_XCLK:            return CLK_GetXCLKFrequency();
		case CLK_CLKSEL2_DACSEL_MCLKI:           return CLK_GetMCLKIFrequency();
		case CLK_CLKSEL2_DACSEL_HIRC :           return g_u32HIRC; 
	}
	return 0;
}

void DAC_EnableAnalog(void)
{
    DAC->ANA0 = ((DAC->ANA0 & ~DAC_ANA0_VREFSEL_Msk) | (2 << DAC_ANA0_VREFSEL_Pos));
    DAC->ANA1 &= ~(DAC_ANA1_PDFLITSM2_Msk | DAC_ANA1_PDFLITSM1_Msk | DAC_ANA1_PDIBGEN_Msk | DAC_ANA1_PDVBUF2_Msk | DAC_ANA1_PDVBUF1_Msk);
	// DAC Vref buffer power & DAC CLOCK enable & DAC enable
    DAC->ANA1 |= ( DAC_ANA1_PDBDAC2_Msk | DAC_ANA1_PDBDAC1_Msk | DAC_ANA1_VOLEN2_Msk | DAC_ANA1_VOLEN1_Msk | DAC_ANA1_ENHP2_Msk | DAC_ANA1_ENHP1_Msk | DAC_ANA1_ENDAC2_Msk | DAC_ANA1_ENDAC1_Msk | DAC_ANA1_ENCLK2_Msk | DAC_ANA1_ENCLK1_Msk );	
}

void DAC_DisableAnalog(void)
{
    DAC->ANA0 &= ~DAC_ANA0_VREFSEL_Msk;
    DAC->ANA1 |= ( DAC_ANA1_PDFLITSM2_Msk | DAC_ANA1_PDFLITSM1_Msk );
	// DAC Vref buffer power & DAC CLOCK enable & DAC enable
    DAC->ANA1 &= ~( DAC_ANA1_VOLEN2_Msk | DAC_ANA1_VOLEN1_Msk | DAC_ANA1_ENHP2_Msk | DAC_ANA1_ENHP1_Msk | DAC_ANA1_ENDAC2_Msk | DAC_ANA1_ENDAC1_Msk | DAC_ANA1_ENCLK2_Msk | DAC_ANA1_ENCLK1_Msk );	
}

/**
  * @brief      Set the sample Rate of data 
  * @param      u32SampleRate is sample Rate of data.
  * @return     Real sample rate.
  */
uint32_t DAC_SetSampleRate(uint32_t u32SampleRate)
{
	uint32_t u32ClockSource, u32DacClock, u32ClockDiv;
	
	u32ClockSource = DAC_GetSourceClockFreq();

	if( u32ClockSource != 0 && u32SampleRate != 0 )
	{
		if( ((u32ClockSource/1000) % 256) == 0)
		{
			DAC_SET_CLKSEL(DAC,DAC_CTL0_CLKSEL_256K);
			u32DacClock = u32SampleRate * 256;
			// Process OSR
			DAC->CTL1 &= ~(DAC_CTL1_MIPS500_Msk|DAC_CTL1_OSR100_Msk|DAC_CTL1_OSRDIV_Msk);
			if(u32DacClock > DAC_MAX_SD_CLOCK_FREQ)
				DAC->CTL1 |= (0x2<<DAC_CTL1_OSRDIV_Pos);
			else
				DAC->CTL1 |= (0x4<<DAC_CTL1_OSRDIV_Pos);
		}
		else 
		{
			DAC_SET_CLKSEL(DAC,DAC_CTL0_CLKSEL_250K);
			u32DacClock = u32SampleRate * 250;		
			// Process OSR.
			DAC->CTL1 &= ~DAC_CTL1_OSRDIV_Msk;
			DAC->CTL1 |= (DAC_CTL1_MIPS500_Msk|DAC_CTL1_OSR100_Msk);
		}
		u32ClockDiv = (u32ClockSource / u32DacClock) - 1;
		
		CLK->CLKDIV1 = (CLK->CLKDIV1 & ~(CLK_CLKDIV1_DACDIV_Msk)) | (u32ClockDiv << CLK_CLKDIV1_DACDIV_Pos);
		
		return DAC_GetSampleRate();
	}
	return 0;
}

/**
  * @brief      Get the sample Rate of data 
  * @return     Real sample rate.
  */
uint32_t DAC_GetSampleRate(void)
{
	uint32_t u32DacClock = DAC_GetSourceClockFreq() / (((CLK->CLKDIV1 & CLK_CLKDIV1_DACDIV_Msk)>>CLK_CLKDIV1_DACDIV_Pos) + 1);
	
	return (u32DacClock/((DAC->CTL0&DAC_CTL0_CLKSET_Msk)?250:256));
}

/**
  * @brief      Write to DAC FIFO for transmit
  * @param      pi32Stream is pointer of input data stream for transmit.
  * @param      u32count is transmit sample count.  
  * @return     None
  * @details    If all channels of PDMA are occupied, programmer can call this 
  *             function to write FIFO by CPU. But programmer needs to 
  *             handle suitable time for writing.  
  */
void DAC_WriteFIFO(int32_t *pi32Stream, uint32_t u32count)
{
	while(u32count > 0)	
	{
		while(DAC_IS_FIFOFULL(DAC)); //FIFO is full
		DAC->DAT = *pi32Stream++;
		u32count--;	
	}
}

/*@}*/ /* end of group I91500_DAC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_DAC_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/

