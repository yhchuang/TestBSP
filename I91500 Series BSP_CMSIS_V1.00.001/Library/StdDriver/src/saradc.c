/**************************************************************************//**
 * @file     saradc.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/11/23 2:35p $
 * @brief    I91500 SARADC driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

#define SARADC_SHCLK       (1250*1000) //1.25MHz

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/ 

/** @addtogroup I91500_SARADC_Driver SARADC Driver
  @{
*/


/** @addtogroup I91500_SARADC_EXPORTED_FUNCTIONS SARADC Exported Functions
  @{
*/

/**
  * @brief      SARADC open function
  * @return     None
  * @details    
  */
void SARADC_Open(void)
{

}

/**
  * @brief      SARADC close function
  * @return     None
  * @details   
  */
void SARADC_Close(void)
{   
      
}

/**
  * @brief      Enable SARADC interrupts
  * @param[in]  u32Mask is interrupt enabled bits according to defined int falg
  *             - \ref SARADC_ADF_INT 
  *             - \ref SARADC_CMP0_INT 
  *             - \ref SARADC_CMP1_INT
  * @return     None.
  * @details    SARADC_ADF_INT is generated whenever A/D conversion end.
  *             SARADC_CMP0_INT is generated whenever compared 0 matches
  *             the setting of count and condition.
  *             SARADC_CMP1_INT is generated whenever compared 1 matches
  *             the setting of count and condition.             
  */
void SARADC_EnableInt(uint32_t u32Mask)
{
	if (u32Mask&SARADC_ADF_INT)
		SARADC->CTL |= SARADC_CTL_ADCIE_Msk;

	if (u32Mask&SARADC_CMP0_INT)
		SARADC->CMP[0] |= SARADC_CMP_ADCMPIE_Msk;
	
	if (u32Mask&SARADC_CMP1_INT)
		SARADC->CMP[1] |= SARADC_CMP_ADCMPIE_Msk;
}  

/**
  * @brief      Disable SARADC interrupts
  * @param[in]  u32Mask is interrupt flags.
  *             - \ref SARADC_ADF_INT 
  *             - \ref SARADC_CMP0_INT 
  *             - \ref SARADC_CMP1_INT
	* @return     None.
  * @details    SARADC_ADF_INT is generated whenever A/D conversion end.
  *             SARADC_CMP0_INT is generated whenever compared 0 matches
  *             the setting of count and condition.
  *             SARADC_CMP1_INT is generated whenever compared 1 matches
  *             the setting of count and condition.             
  */
void SARADC_DisableInt(uint32_t u32Mask)
{
	if (u32Mask&SARADC_ADF_INT)
		SARADC->CTL &= ~SARADC_CTL_ADCIE_Msk;

	if (u32Mask&SARADC_CMP0_INT)
		SARADC->CMP[0] &= ~SARADC_CMP_ADCMPIE_Msk;
	
	if (u32Mask&SARADC_CMP1_INT)
		SARADC->CMP[1] &= ~SARADC_CMP_ADCMPIE_Msk;
}

/**
  * @brief      Get SARADC module clock source
  * @return     SARADC clock source.            
  */
uint32_t SARADC_GetClockSrc(void)
{
	uint32_t u32ClkSrc; 
	
	switch (CLK->CLKSEL1&CLK_CLKSEL1_SARADCSEL_Msk)
	{
		case CLK_CLKSEL1_SARADCSEL_HIRC:
			u32ClkSrc = g_u32HIRC;
			break;
		case CLK_CLKSEL1_SARADCSEL_PCLK:
			u32ClkSrc = CLK_GetHCLKFreq();
			break;
		case CLK_CLKSEL1_SARADCSEL_PLLFOUT:
			u32ClkSrc = CLK_GetPLLFreq();
			break;
		case CLK_CLKSEL1_SARADCSEL_HXT:
			u32ClkSrc = __HXT;
			break;
		default:
			u32ClkSrc = 0;
			break;
	}
	
	return u32ClkSrc;
}

/**
  * @brief      Set SARADC Sample rate
  * @param[in]  u32SampleRate is set sample rate.
  * @return     Real sample rate.            
  */
uint32_t SARADC_SetSampleRate(uint32_t u32SampleRate)
{
	uint32_t u32ClkSrc, u32ClkDiv, SARADC_CONVERSION_CYCLE , SHCLKN;
		
	u32ClkSrc = SARADC_GetClockSrc();
	
	if(u32ClkSrc >= 48000000)
	{
		u32ClkDiv = 4;
	}
	else if(u32ClkSrc < 48000000 && u32ClkSrc >= 24000000)
	{
		u32ClkDiv = 2;
	}
	else
	{
		u32ClkDiv = 1;
	}
	
	SHCLKN =  ((u32ClkSrc/u32ClkDiv) + (SARADC_SHCLK>>1)) / (SARADC_SHCLK) ;
	
	if ((SHCLKN == 0) || (SHCLKN>0x40))
		return 0;
	
	SARADC->HWPARA = (SARADC->HWPARA & ~SARADC_HWPARA_SHCLKN_Msk) | ((SHCLKN-1)<<SARADC_HWPARA_SHCLKN_Pos);
	
	SARADC_CONVERSION_CYCLE = ((u32ClkSrc / u32ClkDiv) + (u32SampleRate>>1)) / (u32SampleRate) ; 
	
	SARADC->HWPARA = (SARADC->HWPARA & ~SARADC_HWPARA_CONVN_Msk) | ((SARADC_CONVERSION_CYCLE-1)<<SARADC_HWPARA_CONVN_Pos);
	
	CLK->CLKDIV0 = (CLK->CLKDIV0&~CLK_CLKDIV0_SARADCDIV_Msk)|((u32ClkDiv - 1)<<CLK_CLKDIV0_SARADCDIV_Pos);
	
	return(u32ClkSrc/u32ClkDiv/SARADC_CONVERSION_CYCLE);
}

/**
  * @brief      Get SARADC Sample rate
  * @return     Real sample rate.            
  */
uint32_t SARADC_GetSampleRate(void)
{
	uint32_t u32ClkSrc ,SARADC_CONVERSION_CYCLE;
	u32ClkSrc = SARADC_GetClockSrc();
	SARADC_CONVERSION_CYCLE = ((SARADC->HWPARA & SARADC_HWPARA_CONVN_Msk) >> SARADC_HWPARA_CONVN_Pos) +1 ;
	return((u32ClkSrc/(((CLK->CLKDIV0&CLK_CLKDIV0_SARADCDIV_Msk)>>CLK_CLKDIV0_SARADCDIV_Pos) + 1))/SARADC_CONVERSION_CYCLE);
}

void SARADC_EnableChannelSequence(uint32_t u32Sequence, uint32_t u32Channel)
{
	UINT32 u32SeqReg = 0;
	
	if( u32Sequence>SARADC_SEL_SEQ13 || u32Channel>SARADC_SEL_CH11 )
		return;
	
	u32SeqReg = (uint32_t)(&SARADC->CHSEQ0+((u32Sequence>=SARADC_SEL_SEQ8)?4:0));
	
	if( u32Sequence >= SARADC_SEL_SEQ8 )
		u32Sequence -=8 ;
	
	*(volatile uint32_t *)u32SeqReg  &=  ~(SARADC_CHSEQ0_CHSEQ0_Msk<<(u32Sequence*4));
	*(volatile uint32_t *)u32SeqReg  |=  (u32Channel<<(u32Sequence*4));		
}

/*@}*/ /* end of group I91500_SARADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_SARADC_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
