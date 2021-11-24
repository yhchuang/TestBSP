/**************************************************************************//**
 * @file     sdadc.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 16/08/22 13:27p $
 * @brief    I91500 SDADC driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "Platform.h"

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_SDADC_Driver SDADC Driver
  @{
*/


/** @addtogroup I91500_SDADC_EXPORTED_FUNCTIONS SDADC Exported Functions
  @{
*/

#define SDADC_MAX_SD_CLOCK_FREQ                  (6144000)   


/**
  * @brief  This function is used to get sdadc source clock frequency.
  * @return SDADC source clock frequency (Hz).
  * @details Return the source clock frequency according to the setting of SPI1SEL (CLKSEL2[5:4]) or SPI2SEL (CLKSEL2[7:6]).
  */
static uint32_t SDADC_GetSourceClockFreq(void)
{
    switch( CLK->CLKSEL2&CLK_CLKSEL2_SDADCSEL_Msk )
	{
		case CLK_CLKSEL2_SDADCSEL_HXT :            return __HXT;
		case CLK_CLKSEL2_SDADCSEL_PLLFOUT :        return CLK_GetPLLFreq();
		case CLK_CLKSEL2_SDADCSEL_PCLK:            return CLK_GetHCLKFreq();
		case CLK_CLKSEL2_SDADCSEL_XCLK :           return CLK_GetXCLKFrequency();
		case CLK_CLKSEL2_SDADCSEL_MCLKI:   		   return CLK_GetMCLKIFrequency();
		case CLK_CLKSEL2_SDADCSEL_HIRC :           return g_u32HIRC; 
	}
	return 0;
}


/**
  * @brief      Set SDADC sample rate for audio path
  * @return     Real sample rate, unit is Hz.
  * @details    The sample rate is calculated by: Fs = MCLK กา CLK_DIV กา DSR
  *             The vaild sample rate range: 48kHz~8kHz.
  *             If BIQ modile is enabled, this function must be called after BIQ initialization
  */
int32_t SDADC_SetAudioSampleRate(uint32_t u32SampleRate)
{
	uint32_t u32ClockSource, u32SDClock, u32ClockDiv, u32Biq, u32SDADC_Div;
	
	float fDSR;

	float fDSRTable[] = {1, 16, 32, 64};
	
	if( BIQ->CTL&BIQ_CTL_BIQEN_Msk )
        u32Biq = (BIQ->CTL&BIQ_CTL_SDADCWNSR_Msk)>>BIQ_CTL_SDADCWNSR_Pos;
    else
        u32Biq = 1;
	
	u32ClockSource = SDADC_GetSourceClockFreq();
		
	if(u32ClockSource <= 12000000)
        return -1;
	
	if( u32ClockSource != 0 && u32SampleRate != 0 )
	{
		if( ((u32ClockSource/1000) % 256) == 0)
		{
			if(u32ClockSource >= 24576000)
				SDADC_SET_DSRATIO(SDADC,SDADC_DS_RATION_64);
			
			SDADC_SET_CLKSEL(SDADC,SDADC_CTL_CLKSEL_256K);
			// Get DSR from table.
			fDSR = fDSRTable[(SDADC->CTL&SDADC_CTL_DSRATE_Msk)>>SDADC_CTL_DSRATE_Pos];
			
			// Get SD clock value.
            u32SDClock = u32SampleRate * fDSR * u32Biq;	
            // Cal clock divider.
            u32ClockDiv = u32ClockSource / u32SDClock;
			      u32SDADC_Div = 1;
		}
		else 
		{
			SDADC_SET_CLKSEL(SDADC,SDADC_CTL_CLKSEL_250K);
			// If clock source is selected 250K, DSR is always 62.5.
			SDADC_SET_DSRATIO(SDADC,SDADC_DS_RATION_64);
			fDSR = 62.5;
			u32ClockDiv = 8;
			// Get SD clock value.
			u32SDClock = u32SampleRate * fDSR * u32Biq;	
			// Cal clock divider.
			u32SDADC_Div = u32ClockSource / u32SDClock / u32ClockDiv;
		}

		if( u32ClockDiv < 4 )			//Error condition
			return 0;
			
		SDADC->CLKDIV = u32ClockDiv;
		
		CLK->CLKDIV1 = (CLK->CLKDIV1 & ~CLK_CLKDIV1_SDADCDIV_Msk) | ((u32SDADC_Div-1)<<CLK_CLKDIV1_SDADCDIV_Pos);
		
		return SDADC_GetAudioSampleRate();
	}
	return 0;
}

/**
  * @brief      Get sample rate of SDADC audio path
  * @return     Sample rate, unit is Hz.
  * @details    This function calculate sample rate according to SDCLK 
  *             divisor and over sampling ration.
  *             Formula: Fs = HCLK/CLKDIV/OSR.
  */
uint32_t SDADC_GetAudioSampleRate(void)
{
  float fDSRTable[4] = {1, 16, 32, 64};
	float fDSR;
	uint32_t u32SDADC_Div;

	if( SDADC->CTL&SDADC_CTL_SPDS_Msk )
		fDSR = 62.5;
	else
		fDSR = fDSRTable[(SDADC->CTL&SDADC_CTL_DSRATE_Msk)>>SDADC_CTL_DSRATE_Pos];
		
	if (BIQ->CTL&BIQ_CTL_BIQEN_Msk)
		fDSR *= ((BIQ->CTL&BIQ_CTL_SDADCWNSR_Msk) >> BIQ_CTL_SDADCWNSR_Pos);
	
	u32SDADC_Div = ((CLK->CLKDIV1 & CLK_CLKDIV1_SDADCDIV_Msk) >> CLK_CLKDIV1_SDADCDIV_Pos) + 1 ;
	
    return ((SDADC_GetSourceClockFreq()/u32SDADC_Div/(SDADC->CLKDIV))/fDSR);
}

/**
  * @brief      Enable SDADC interrupts
  * @param      u32Mask is interrupt flags.
  *             - \ref SDADC_FIFO_INT 
  *             - \ref SDADC_CMP0_INT 
  *             - \ref SDADC_CMP1_INT
  *             - \ref SDADC_ALC_PLMT_INT
  *             - \ref SDADC_ALC_NG_INT
  *             - \ref SDADC_ALC_GINC_INT
  *             - \ref SDADC_ALC_GDEC_INT
  *             - \ref SDADC_ALC_GMAX_INT
  *             - \ref SDADC_ALC_GMIN_INT
  * @return     None.
  * @details    SDADC_FIFO_INT is generated whenever FIFO level exceeds
  *             that set in u8Level of SDADC_SET_FIFOINTLEVEL macro.
  *             SDADC_CMP0_INT is generated whenever compared 0 matches
  *             the setting of count and condition.
  *             SDADC_CMP1_INT is generated whenever compared 1 matches
  *             the setting of count and condition.             
  */
void SDADC_EnableInt(uint32_t u32Mask)
{
	if (u32Mask&SDADC_FIFO_INT)
	{
		/* ADC FIFO interrupt and PDMA don't coexist */
		SDADC->PDMACTL = 0;
		SDADC->CTL |= SDADC_CTL_FIFOTHIE_Msk;
	}
	
	if (u32Mask&SDADC_CMP0_INT)
	{
		/* clear the CMP interrupt flags for safe */
		SDADC->CMPR0 |= (SDADC_CMPR0_CMPF_Msk);
		SDADC->CMPR0 |= (SDADC_CMPR0_CMPIE_Msk);
	}
	
	if (u32Mask&SDADC_CMP1_INT)
	{
		/* clear the CMP interrupt flags for safe */
		SDADC->CMPR1 |= (SDADC_CMPR1_CMPF_Msk);
		SDADC->CMPR1 |= (SDADC_CMPR1_CMPIE_Msk);
	}
}

/**
  * @brief      Disable ADC interrupts
  * @param      u32Mask is interrupt flags.
  *             - \ref SDADC_FIFO_INT 
  *             - \ref SDADC_CMP0_INT 
  *             - \ref SDADC_CMP1_INT
  *             - \ref SDADC_ALC_PLMT_INT
  *             - \ref SDADC_ALC_NG_INT
  *             - \ref SDADC_ALC_GINC_INT
  *             - \ref SDADC_ALC_GDEC_INT
  *             - \ref SDADC_ALC_GMAX_INT
  *             - \ref SDADC_ALC_GMIN_INT
  * @return     None.
  * @details    ADC_FIFO_INT is generated whenever FIFO level exceeds
  *             that set in u8Level of ADC_SET_FIFOINTLEVEL macro.
  *             ADC_CMP0_INT is generated whenever compared 0 matches
  *             the setting of count and condition.
  *             ADC_CMP1_INT is generated whenever compared 1 matches
  *             the setting of count and condition.             
  */
void SDADC_DisableInt(uint32_t u32Mask)
{
	if (u32Mask&SDADC_FIFO_INT)
		SDADC->CTL &= (~SDADC_CTL_FIFOTHIE_Msk);
		
	if (u32Mask&SDADC_CMP0_INT)
		SDADC->CMPR0 &= (~SDADC_CMPR0_CMPIE_Msk);
	
	if (u32Mask&SDADC_CMP1_INT)
		SDADC->CMPR1 &= (~SDADC_CMPR1_CMPIE_Msk);
}

/**
  * @brief     Get the interrupt status bits
  * @param     u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *             - \ref SDADC_FIFO_INT 
  *             - \ref SDADC_CMP0_INT 
  *             - \ref SDADC_CMP1_INT
  *             - \ref SDADC_ALC_PLMT_INT
  *             - \ref SDADC_ALC_NG_INT
  *             - \ref SDADC_ALC_GINC_INT
  *             - \ref SDADC_ALC_GDEC_INT
  *             - \ref SDADC_ALC_GMAX_INT
  *             - \ref SDADC_ALC_GMIN_INT
  * @return    interrupt status bits
  */
uint32_t SDADC_GetIntFlag(uint32_t u32Mask)
{
	uint32_t u32Flag = 0;
	
	if (u32Mask&SDADC_FIFO_INT)
		u32Flag |= (SDADC->FIFOSTS&SDADC_FIFOSTS_THIF_Msk)?SDADC_FIFO_INT:0;
	
	if (u32Mask&SDADC_CMP0_INT)
		u32Flag |= (SDADC->CMPR0&SDADC_CMPR0_CMPF_Msk)?SDADC_CMP0_INT:0;
	
	if (u32Mask&SDADC_CMP1_INT)
		u32Flag |= (SDADC->CMPR1&SDADC_CMPR1_CMPF_Msk)?SDADC_CMP1_INT:0;
	
	return u32Flag;
}

/**
  * @brief     Clear the selected interrupt status bits
  * @param     u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *             - \ref SDADC_CMP0_INT 
  *             - \ref SDADC_CMP1_INT
  *             - \ref SDADC_ALC_PLMT_INT
  *             - \ref SDADC_ALC_NG_INT
  *             - \ref SDADC_ALC_GINC_INT
  *             - \ref SDADC_ALC_GDEC_INT
  *             - \ref SDADC_ALC_GMAX_INT
  *             - \ref SDADC_ALC_GMIN_INT
  * @return    None
  * @details   ADC_FIFO_INT don't need to clear.
  */
void SDADC_ClearIntFlag(uint32_t u32Mask)
{
	if (u32Mask&SDADC_CMP0_INT)
		SDADC->CMPR0 |= SDADC_CMPR0_CMPF_Msk;
	
	if (u32Mask&SDADC_CMP1_INT)
		SDADC->CMPR1 |= SDADC_CMPR1_CMPF_Msk;
}

/*@}*/ /* end of group I91500_SDADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_SDADC_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
