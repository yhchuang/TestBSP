/**************************************************************************//**
 * @file     clk.h
 * @version  V1.00
 * $Revision: 1$
 * $Date: 20/10/13 2:35p $
 * @brief    I91500 series CLK driver header file
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __CLK_H__
#define __CLK_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_CLK_Driver CLK Driver
  @{
*/

/** @addtogroup I91500_CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  PWRCON constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_DPDWAKEUP_PINOSC10K   0      /*!< DPD wake up from WAKEUP pin and OSC 10k */  
#define CLK_DPDWAKEUP_PIN         1      /*!< DPD wake up from WAKEUP pin */
#define CLK_DPDWAKEUP_OSC10K      2      /*!< DPD wake up from OSC 10k */
#define CLK_DPDWAKEUP_POR         3      /*!< DPD wake up from POR event trigger */

#define CLK_DPDWAKETIME_12ms    (0x0ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 128 OSC10K clocks (12.8 ms) */
#define CLK_DPDWAKETIME_25ms    (0x1ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 256 OSC10K clocks (25.6 ms) */
#define CLK_DPDWAKETIME_50ms    (0x2ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 512 OSC10K clocks (51.2 ms) */
#define CLK_DPDWAKETIME_100ms   (0x3ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 1024 OSC10K clocks (102.4 ms) */
#define CLK_DPDWAKETIME_400ms   (0x4ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 4096 OSC10K clocks (409.2 ms) */
#define CLK_DPDWAKETIME_800ms   (0x5ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 8192 OSC10K clocks (819.2 ms) */
#define CLK_DPDWAKETIME_16000ms (0x6ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 16384 OSC10K clocks (1638.4 ms) */
#define CLK_DPDWAKETIME_65000ms (0x7ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 65536 OSC10K clocks (6553.6 ms) */
	
/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL0 constant definitions.  (Write-protection)                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL0_HIRCSEL_49M_VCC33    (0x0UL<<CLK_CLKSEL0_OSCFSEL_Pos)	/*!< Setting HIRC select as 49.152M and VCC 3.3 \hideinitializer */
#define CLK_CLKSEL0_HIRCSEL_48M_VCC33    (0x1UL<<CLK_CLKSEL0_OSCFSEL_Pos)	/*!< Setting HIRC select as 48M and VCC 3.3 \hideinitializer */
#define CLK_CLKSEL0_HIRCSEL_49M_VCC18    (0x2UL<<CLK_CLKSEL0_OSCFSEL_Pos)	/*!< Setting HIRC select as 49.152M and VCC 1.8 \hideinitializer */

#define CLK_CLKSEL0_HCLKSEL_HXT          (0x0UL<<CLK_CLKSEL0_HCLKSEL_Pos)	/*!< Setting clock source as HXT  \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_PLLFOUT      (0x1UL<<CLK_CLKSEL0_HCLKSEL_Pos)	/*!< Setting clock source as PLLFOUT  \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_LIRC         (0x2UL<<CLK_CLKSEL0_HCLKSEL_Pos)	/*!< Setting clock source as LIRC  \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_HIRC         (0x3UL<<CLK_CLKSEL0_HCLKSEL_Pos)   /*!< Setting clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL0_STICKSEL_HXT         (0x0UL<<CLK_CLKSEL0_STICKSEL_Pos)	/*!< Setting SYS_TICK Clock source as HXT  \hideinitializer */
#define CLK_CLKSEL0_STICKSEL_HXT_DIV2    (0x1UL<<CLK_CLKSEL0_STICKSEL_Pos)	/*!< Setting SYS_TICK Clock source as HXT/2  \hideinitializer */
#define CLK_CLKSEL0_STICKSEL_HCLK_DIV2   (0x2UL<<CLK_CLKSEL0_STICKSEL_Pos)	/*!< Setting SYS_TICK Clock source as HCLK/2  \hideinitializer */
#define CLK_CLKSEL0_STICKSEL_HIRC_DIV2   (0x3UL<<CLK_CLKSEL0_STICKSEL_Pos)	/*!< Setting SYS_TICK Clock source as HIRC/2  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL1 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL1_WDTSEL_LIRC          (0x0UL<<CLK_CLKSEL1_WDTSEL_Pos)       /*!< Setting WDT clock source as LIRC 10KHz  \hideinitializer */
#define CLK_CLKSEL1_WDTSEL_HCLK_DIV2048  (0x1UL<<CLK_CLKSEL1_WDTSEL_Pos)       /*!< Setting WDT clock source as HCLK/2048  \hideinitializer */

#define CLK_CLKSEL1_SARADCSEL_HXT        (0x0UL<<CLK_CLKSEL1_SARADCSEL_Pos)    /*!< Setting SARADC clock source as HXT  \hideinitializer */
#define CLK_CLKSEL1_SARADCSEL_PLLFOUT    (0x1UL<<CLK_CLKSEL1_SARADCSEL_Pos)    /*!< Setting SARADC clock source as PLLFOUT  \hideinitializer */
#define CLK_CLKSEL1_SARADCSEL_PCLK       (0x2UL<<CLK_CLKSEL1_SARADCSEL_Pos)    /*!< Setting SARADC clock source as PCLK  \hideinitializer */
#define CLK_CLKSEL1_SARADCSEL_HIRC       (0x3UL<<CLK_CLKSEL1_SARADCSEL_Pos)    /*!< Setting SARADC clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_TMR0SEL_HXT          (0x0UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as HXT  \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_PCLK         (0x1UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as PCLK  \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_EXT          (0x2UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as external trigger  \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_LIRC         (0x3UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as LIRC  \hideinitializer */    
#define CLK_CLKSEL1_TMR0SEL_HIRC         (0x4UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_TMR1SEL_HXT          (0x0UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as HXT  \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_PCLK         (0x1UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as PCLK  \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_EXT          (0x2UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as external trigger  \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_LIRC         (0x3UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as LIRC   \hideinitializer */    
#define CLK_CLKSEL1_TMR1SEL_HIRC         (0x4UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_TMR2SEL_HXT          (0x0UL<<CLK_CLKSEL1_TMR2SEL_Pos)      /*!< Setting Timer 2 clock source as HXT  \hideinitializer */
#define CLK_CLKSEL1_TMR2SEL_PCLK         (0x1UL<<CLK_CLKSEL1_TMR2SEL_Pos)      /*!< Setting Timer 2 clock source as PCLK  \hideinitializer */
#define CLK_CLKSEL1_TMR2SEL_EXT          (0x2UL<<CLK_CLKSEL1_TMR2SEL_Pos)      /*!< Setting Timer 2 clock source as external trigger  \hideinitializer */
#define CLK_CLKSEL1_TMR2SEL_LIRC         (0x3UL<<CLK_CLKSEL1_TMR2SEL_Pos)      /*!< Setting Timer 2 clock source as LIRC  \hideinitializer */
#define CLK_CLKSEL1_TMR2SEL_HIRC         (0x4UL<<CLK_CLKSEL1_TMR2SEL_Pos)      /*!< Setting Timer 2 clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_I2S0SEL_HXT          (0x0UL<<CLK_CLKSEL1_I2S0SEL_Pos)      /*!< Setting I2S clock source as HXT  \hideinitializer */
#define CLK_CLKSEL1_I2S0SEL_PLLFOUT      (0x1UL<<CLK_CLKSEL1_I2S0SEL_Pos)      /*!< Setting I2S clock source as PLLFOUT  \hideinitializer */
#define CLK_CLKSEL1_I2S0SEL_PCLK         (0x2UL<<CLK_CLKSEL1_I2S0SEL_Pos)      /*!< Setting I2S clock source as LIRC  \hideinitializer */
#define CLK_CLKSEL1_I2S0SEL_HIRC         (0x3UL<<CLK_CLKSEL1_I2S0SEL_Pos)      /*!< Setting I2S clock source as PCLK  \hideinitializer */
#define CLK_CLKSEL1_I2S0SEL_MCLKI        (0x4UL<<CLK_CLKSEL1_I2S0SEL_Pos)      /*!< Setting I2S clock source as MCLKI  \hideinitializer */
#define CLK_CLKSEL1_I2S0SEL_XLCK         (0x5UL<<CLK_CLKSEL1_I2S0SEL_Pos)      /*!< Setting I2S clock source as XLCK  \hideinitializer */

#define CLK_CLKSEL1_UART0SEL_HXT         (0x0UL<<CLK_CLKSEL1_UART0SEL_Pos)     /*!< Setting UART 0 clock source as HXT  \hideinitializer */
#define CLK_CLKSEL1_UART0SEL_PLLFOUT     (0x1UL<<CLK_CLKSEL1_UART0SEL_Pos)     /*!< Setting UART 0 clock source as PLLFOUT  \hideinitializer */
#define CLK_CLKSEL1_UART0SEL_HIRC        (0x2UL<<CLK_CLKSEL1_UART0SEL_Pos)     /*!< Setting UART 0 clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_UART1SEL_HXT         (0x0UL<<CLK_CLKSEL1_UART1SEL_Pos)     /*!< Setting UART 1 clock source as HXT  \hideinitializer */
#define CLK_CLKSEL1_UART1SEL_PLLFOUT     (0x1UL<<CLK_CLKSEL1_UART1SEL_Pos)     /*!< Setting UART 1 clock source as PLLFOUT  \hideinitializer */
#define CLK_CLKSEL1_UART1SEL_HIRC        (0x2UL<<CLK_CLKSEL1_UART1SEL_Pos)     /*!< Setting UART 1 clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_PWM0SEL_HXT          (0x0UL<<CLK_CLKSEL1_PWM0SEL_Pos)      /*!< Setting PWM 0 clock source as HXT  \hideinitializer */
#define CLK_CLKSEL1_PWM0SEL_PLLFOUT      (0x1UL<<CLK_CLKSEL1_PWM0SEL_Pos)      /*!< Setting PWM 0 clock source as PLLFOUT  \hideinitializer */
#define CLK_CLKSEL1_PWM0SEL_PCLK         (0x2UL<<CLK_CLKSEL1_PWM0SEL_Pos)      /*!< Setting PWM 0 clock source as PCLK  \hideinitializer */
#define CLK_CLKSEL1_PWM0SEL_HIRC         (0x3UL<<CLK_CLKSEL1_PWM0SEL_Pos)      /*!< Setting PWM 0 clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL1_PWM1SEL_HXT          (0x0UL<<CLK_CLKSEL1_PWM1SEL_Pos)      /*!< Setting PWM 1 clock source as HXT  \hideinitializer */
#define CLK_CLKSEL1_PWM1SEL_PLLFOUT      (0x1UL<<CLK_CLKSEL1_PWM1SEL_Pos)      /*!< Setting PWM 1 clock source as PLLFOUT  \hideinitializer */
#define CLK_CLKSEL1_PWM1SEL_PCLK         (0x2UL<<CLK_CLKSEL1_PWM1SEL_Pos)      /*!< Setting PWM 1 clock source as PCLK  \hideinitializer */
#define CLK_CLKSEL1_PWM1SEL_HIRC         (0x3UL<<CLK_CLKSEL1_PWM1SEL_Pos)      /*!< Setting PWM 1 clock source as HIRC  \hideinitializer */

#define CLK_CLKSEL2_USBDSEL_HIRC         (0x0UL<<CLK_CLKSEL2_USBSEL_Pos)       /*!< Setting USBD clock source as HIRC  \hideinitializer */
#define CLK_CLKSEL2_USBDSEL_PLLFOUT      (0x1UL<<CLK_CLKSEL2_USBSEL_Pos)       /*!< Setting USBD clock source as PLLFOUT  \hideinitializer */

#define CLK_CLKSEL2_XCLKSEL_MCLKI        (0x0UL<<CLK_CLKSEL2_XCLKSEL_Pos)      /*!< Setting Xtal CLK clock source as MCLKI  \hideinitializer */
#define CLK_CLKSEL2_XCLKSEL_I2SBCLK      (0x1UL<<CLK_CLKSEL2_XCLKSEL_Pos)      /*!< Setting Xtal CLK clock source as I2S's BCLK  \hideinitializer */

#define CLK_CLKSEL2_DACSEL_HXT           (0x0UL<<CLK_CLKSEL2_DACSEL_Pos)       /*!< Setting DAC clock source as HXT  \hideinitializer */
#define CLK_CLKSEL2_DACSEL_PLLFOUT       (0x1UL<<CLK_CLKSEL2_DACSEL_Pos)       /*!< Setting DAC clock source as PLLFOUT  \hideinitializer */
#define CLK_CLKSEL2_DACSEL_PCLK          (0x2UL<<CLK_CLKSEL2_DACSEL_Pos)       /*!< Setting DAC clock source as PCLK  \hideinitializer */
#define CLK_CLKSEL2_DACSEL_HIRC          (0x3UL<<CLK_CLKSEL2_DACSEL_Pos)       /*!< Setting DAC clock source as HIRC  \hideinitializer */
#define CLK_CLKSEL2_DACSEL_MCLKI         (0x4UL<<CLK_CLKSEL2_DACSEL_Pos)       /*!< Setting DAC clock source as MCLKI  \hideinitializer */
#define CLK_CLKSEL2_DACSEL_XCLK          (0x5UL<<CLK_CLKSEL2_DACSEL_Pos)       /*!< Setting DAC clock source as XCLK  \hideinitializer */

#define CLK_CLKSEL2_SDADCSEL_HXT         (0x0UL<<CLK_CLKSEL2_SDADCSEL_Pos)     /*!< Setting SDADC clock source as HXT  \hideinitializer */
#define CLK_CLKSEL2_SDADCSEL_PLLFOUT     (0x1UL<<CLK_CLKSEL2_SDADCSEL_Pos)     /*!< Setting SDADC clock source as PLLFOUT  \hideinitializer */
#define CLK_CLKSEL2_SDADCSEL_PCLK        (0x2UL<<CLK_CLKSEL2_SDADCSEL_Pos)     /*!< Setting SDADC clock source as PCLK  \hideinitializer */
#define CLK_CLKSEL2_SDADCSEL_HIRC        (0x3UL<<CLK_CLKSEL2_SDADCSEL_Pos)     /*!< Setting SDADC clock source as HIRC  \hideinitializer */
#define CLK_CLKSEL2_SDADCSEL_MCLKI       (0x4UL<<CLK_CLKSEL2_SDADCSEL_Pos)     /*!< Setting SDADC clock source as MCLKI  \hideinitializer */
#define CLK_CLKSEL2_SDADCSEL_XCLK        (0x5UL<<CLK_CLKSEL2_SDADCSEL_Pos)     /*!< Setting SDADC clock source as XCLK  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKDIV constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKDIV_HCLK(x)     ((((x)-1)<<CLK_CLKDIV0_HCLKDIV_Pos)&CLK_CLKDIV0_HCLKDIV_Msk)         /*!< CLKDIV Setting for HCLK clock divider. It could be 1~16  \hideinitializer */
#define CLK_CLKDIV_UART0(x)    ((((x)-1)<<CLK_CLKDIV0_UART0DIV_Pos)&CLK_CLKDIV0_UART0DIV_Msk)       /*!< CLKDIV Setting for UART0 clock divider. It could be 1~16 \hideinitializer */
#define CLK_CLKDIV_UART1(x)    ((((x)-1)<<CLK_CLKDIV0_UART1DIV_Pos)&CLK_CLKDIV0_UART1DIV_Msk)       /*!< CLKDIV Setting for UART1 clock divider. It could be 1~16 \hideinitializer */
#define CLK_CLKDIV_USBD(x)     ((((x)-1)<<CLK_CLKDIV0_USBDIV_Pos)&CLK_CLKDIV0_USBDIV_Msk)           /*!< CLKDIV Setting for USBD clock divider. It could be 1~16 \hideinitializer */
#define CLK_CLKDIV_SARADC(x)   ((((x)-1)<<CLK_CLKDIV0_SARADCDIV_Pos)&CLK_CLKDIV0_SARADCDIV_Msk)     /*!< CLKDIV Setting for SARADC clock divider. It could be 1~128 \hideinitializer */
#define CLK_CLKDIV_DAC(x)      ((((x)-1)<<CLK_CLKDIV1_DACDIV_Pos)&CLK_CLKDIV1_DACDIV_Msk)           /*!< CLKDIV Setting for DAC clock divider. It could be 1~256 \hideinitializer */
#define CLK_CLKDIV_BIQ(x)      ((((x)-1)<<CLK_CLKDIV1_BIQDIV_Pos)&CLK_CLKDIV1_BIQDIV_Msk)           /*!< CLKDIV Setting for BIQ clock divider. It could be 1~16 \hideinitializer */
#define CLK_CLKDIV_SDADC(x)    ((((x)-1)<<CLK_CLKDIV1_SDADCDIV_Pos)&CLK_CLKDIV1_SDADCDIV_Msk)      /*!< CLKDIV Setting for SDADC clock divider. It could be 1~256 \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  PLLCTL constant definitions. PLL = FIN * 2*NF / NR / NO                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PLLCTL_PLLSRC_HXT   (0x0UL<<CLK_PLLCTL_PLLSRC_Pos)    /*!< For PLL clock source is HXT. */
#define CLK_PLLCTL_PLLSRC_XCLK  (0x1UL<<CLK_PLLCTL_PLLSRC_Pos)    /*!< For PLL clock source is XCLK. */
#define CLK_PLLCTL_PLLSRC_HIRC  (0x3UL<<CLK_PLLCTL_PLLSRC_Pos)    /*!< For PLL clock source is HIRC. 48MHz */

/*---------------------------------------------------------------------------------------------------------*/
/*  XCLKCTL constant definitions.                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_XCLKCTL_XCLKMUL_x1   (0x0UL<<CLK_XCLKCTL_XCLKMUL_Pos)    /*!< Output frequency multiply by 1 (Bypass). */
#define CLK_XCLKCTL_XCLKMUL_x2   (0x1UL<<CLK_XCLKCTL_XCLKMUL_Pos)    /*!< Output frequency multiply by 2 */
#define CLK_XCLKCTL_XCLKMUL_x4   (0x2UL<<CLK_XCLKCTL_XCLKMUL_Pos)    /*!< Output frequency multiply by 4 */
#define CLK_XCLKCTL_XCLKMUL_x8   (0x3UL<<CLK_XCLKCTL_XCLKMUL_Pos)    /*!< Output frequency multiply by 8 */

/*---------------------------------------------------------------------------------------------------------*/
/*  MODULE constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define MODULE_AHPBCLK(x)                  ((x >>31) & 0x1)    /*!< Calculate AHBCLK/APBCLK offset on MODULE index  \hideinitializer */
#define MODULE_CLKSEL(x)                   ((x >>26) & 0x3)    /*!< Calculate CLKSEL0~2 register offset by MODULE index  \hideinitializer */
#define MODULE_CLKSEL_Msk(x)               ((x >>23) & 0x7)    /*!< Calculate mask bits of CLKSEL0~2 by MODULE index  \hideinitializer */
#define MODULE_CLKSEL_Pos(x)               ((x >>18) & 0x1f)   /*!< Calculate mask bits offset of CLKSEL0~2 by MODULE index  \hideinitializer */
#define MODULE_CLKDIV(x)                   ((x >>17) & 0x1)    /*!< Calculate APBCLK CLKDIV0~1 by MODULE index  \hideinitializer */
#define MODULE_CLKDIV_Msk(x)               ((x >>10) & 0x7f)   /*!< Calculate mask bits of CLKDIV0~1 by MODULE index  \hideinitializer */
#define MODULE_CLKDIV_Pos(x)               ((x >>5 ) & 0x1f)   /*!< Calculate ask bits offset of CLKDIV0~1 by MODULE index  \hideinitializer */
#define MODULE_IP_EN_Pos(x)                ((x >>0 ) & 0x1f)   /*!< Calculate APBCLK enabled offset by MODULE index  \hideinitializer */

#define MODULE_NoMsk                       0x0                 /*!< Not mask by MODULE index  \hideinitializer */
#define MODULE_CFG(CLKType,CLKSel,CLKMsk,CLKPos,DIVSel,DIVMsk,DIVPos,IPEn)  \
(       (CLKType<<31)      |   (CLKSel<<26)   |     (CLKMsk<<23)     |     (CLKPos<<18)     |   (DIVSel<<17)   |     (DIVMsk<<10)     |     (DIVPos<<5)     |       IPEn       )
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/* AHBCLK/APBCLK(BIT31)(1) | CLKSEL(BIT26)(2) | CLKSEL_Msk(BIT23)(3) | CLKSEL_Pos(BIT18)(5) | CLKDIV(BIT17)(1) | CLKDIV_Msk(BIT10)(7) | CLKDIV_Pos(BIT5)(5) | IP_EN_Pos(BIT0)(5) */
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#define PDMA_MODULE    MODULE_CFG(0x0UL,0,                                        MODULE_NoMsk,                        0,0,                                        MODULE_NoMsk,                        0,CLK_AHBCLK_PDMACKEN_Pos)    /*!< PDMA Module  \hideinitializer */ 
#define CPD_MODULE     MODULE_CFG(0x0UL,0,                                        MODULE_NoMsk,                        0,0,                                        MODULE_NoMsk,                        0,CLK_AHBCLK_CPDCKEN_Pos)     /*!< CPD Module  \hideinitializer */ 
#define ISP_MODULE     MODULE_CFG(0x0UL,0,                                        MODULE_NoMsk,                        0,0,                                        MODULE_NoMsk,                        0,CLK_AHBCLK_ISPCKEN_Pos)     /*!< ISP Module  \hideinitializer */
#define WDT_MODULE     MODULE_CFG(0x1UL,1,      CLK_CLKSEL1_WDTSEL_Msk>>CLK_CLKSEL1_WDTSEL_Pos,   CLK_CLKSEL1_WDTSEL_Pos,0,                                        MODULE_NoMsk,                        0,CLK_APBCLK_WDTEN_Pos)       /*!< WDT Module  \hideinitializer */
#define TMR0_MODULE    MODULE_CFG(0x1UL,1,    CLK_CLKSEL1_TMR0SEL_Msk>>CLK_CLKSEL1_TMR0SEL_Pos,  CLK_CLKSEL1_TMR0SEL_Pos,0,                                        MODULE_NoMsk,                        0,CLK_APBCLK_TMR0EN_Pos)      /*!< TMR0 Module  \hideinitializer */
#define TMR1_MODULE    MODULE_CFG(0x1UL,1,    CLK_CLKSEL1_TMR1SEL_Msk>>CLK_CLKSEL1_TMR1SEL_Pos,  CLK_CLKSEL1_TMR1SEL_Pos,0,                                        MODULE_NoMsk,                        0,CLK_APBCLK_TMR1EN_Pos)      /*!< TMR1 Module  \hideinitializer */
#define TMR2_MODULE    MODULE_CFG(0x1UL,1,    CLK_CLKSEL1_TMR2SEL_Msk>>CLK_CLKSEL1_TMR2SEL_Pos,  CLK_CLKSEL1_TMR2SEL_Pos,0,                                        MODULE_NoMsk,                        0,CLK_APBCLK_TMR2EN_Pos)      /*!< TMR2 Module  \hideinitializer */
#define I2C0_MODULE    MODULE_CFG(0x1UL,0,                                        MODULE_NoMsk,                        0,0,                                        MODULE_NoMsk,                        0,CLK_APBCLK_I2C0EN_Pos)      /*!< I2C0 Module  \hideinitializer */
#define I2C1_MODULE    MODULE_CFG(0x1UL,0,                                        MODULE_NoMsk,                        0,0,                                        MODULE_NoMsk,                        0,CLK_APBCLK_I2C1EN_Pos)      /*!< I2C1 Module  \hideinitializer */
#define SPI0_MODULE    MODULE_CFG(0x1UL,0,                                        MODULE_NoMsk,                        0,0,                                        MODULE_NoMsk,                        0,CLK_APBCLK_SPI0EN_Pos)      /*!< SPI0 Module  \hideinitializer */
#define SPI1_MODULE    MODULE_CFG(0x1UL,0,                                        MODULE_NoMsk,                        0,0,                                        MODULE_NoMsk,                        0,CLK_APBCLK_SPI1EN_Pos)      /*!< SPI1 Module  \hideinitializer */
#define I2S0_MODULE    MODULE_CFG(0x1UL,1,      CLK_CLKSEL1_I2S0SEL_Msk>>CLK_CLKSEL1_I2S0SEL_Pos,   CLK_CLKSEL1_I2S0SEL_Pos,0,                                        MODULE_NoMsk,                        0,CLK_APBCLK_I2S0EN_Pos)       /*!< I2S0 Module  \hideinitializer */
#define UART0_MODULE   MODULE_CFG(0x1UL,1,  CLK_CLKSEL1_UART0SEL_Msk>>CLK_CLKSEL1_UART0SEL_Pos, CLK_CLKSEL1_UART0SEL_Pos,0,  CLK_CLKDIV0_UART0DIV_Msk>>CLK_CLKDIV0_UART0DIV_Pos, CLK_CLKDIV0_UART0DIV_Pos,CLK_APBCLK_UART0EN_Pos)     /*!< UART0 Module  \hideinitializer */
#define UART1_MODULE   MODULE_CFG(0x1UL,1,  CLK_CLKSEL1_UART1SEL_Msk>>CLK_CLKSEL1_UART1SEL_Pos, CLK_CLKSEL1_UART1SEL_Pos,0,  CLK_CLKDIV0_UART1DIV_Msk>>CLK_CLKDIV0_UART1DIV_Pos, CLK_CLKDIV0_UART1DIV_Pos,CLK_APBCLK_UART1EN_Pos)     /*!< UART1 Module  \hideinitializer */
#define BIQ_MODULE     MODULE_CFG(0x1UL,0,                                        MODULE_NoMsk,                        0,1,      CLK_CLKDIV1_BIQDIV_Msk>>CLK_CLKDIV1_BIQDIV_Pos,   CLK_CLKDIV1_BIQDIV_Pos,CLK_APBCLK_BIQEN_Pos)       /*!< BIQ Module  \hideinitializer */                  
#define PWM0_MODULE    MODULE_CFG(0x1UL,1,    CLK_CLKSEL1_PWM0SEL_Msk>>CLK_CLKSEL1_PWM0SEL_Pos,  CLK_CLKSEL1_PWM0SEL_Pos,0,                                        MODULE_NoMsk,                        0,CLK_APBCLK_PWM0EN_Pos)      /*!< PWM0 Module  \hideinitializer */                
#define PWM1_MODULE    MODULE_CFG(0x1UL,1,    CLK_CLKSEL1_PWM1SEL_Msk>>CLK_CLKSEL1_PWM1SEL_Pos,  CLK_CLKSEL1_PWM1SEL_Pos,0,                                        MODULE_NoMsk,                        0,CLK_APBCLK_PWM1EN_Pos)      /*!< PWM1 Module  \hideinitializer */    
#define USBD_MODULE    MODULE_CFG(0x1UL,2,      CLK_CLKSEL2_USBSEL_Msk>>CLK_CLKSEL2_USBSEL_Pos,   CLK_CLKSEL2_USBSEL_Pos,0,      CLK_CLKDIV0_USBDIV_Msk>>CLK_CLKDIV0_USBDIV_Pos,   CLK_CLKDIV0_USBDIV_Pos,CLK_APBCLK_USBEN_Pos)       /*!< USBD Module  \hideinitializer */    
#define SARADC_MODULE  MODULE_CFG(0x1UL,1,CLK_CLKSEL1_SARADCSEL_Msk>>CLK_CLKSEL1_SARADCSEL_Pos,CLK_CLKSEL1_SARADCSEL_Pos,0,CLK_CLKDIV0_SARADCDIV_Msk>>CLK_CLKDIV0_SARADCDIV_Pos,CLK_CLKDIV0_SARADCDIV_Pos,CLK_APBCLK_SARADCEN_Pos)    /*!< SARADC Module  \hideinitializer */    
#define SDADC_MODULE   MODULE_CFG(0x1UL,2,  CLK_CLKSEL2_SDADCSEL_Msk>>CLK_CLKSEL2_SDADCSEL_Pos, CLK_CLKSEL2_SDADCSEL_Pos,1,  CLK_CLKDIV1_SDADCDIV_Msk>>CLK_CLKDIV1_SDADCDIV_Pos, CLK_CLKDIV1_SDADCDIV_Pos,CLK_APBCLK_SDADCEN_Pos)     /*!< SDADC Module  \hideinitializer */    
#define DAC_MODULE     MODULE_CFG(0x1UL,2,      CLK_CLKSEL2_DACSEL_Msk>>CLK_CLKSEL2_DACSEL_Pos,   CLK_CLKSEL2_DACSEL_Pos,1,      CLK_CLKDIV1_DACDIV_Msk>>CLK_CLKDIV1_DACDIV_Pos,   CLK_CLKDIV1_DACDIV_Pos,CLK_APBCLK_DACEN_Pos)       /*!< DAC Module  \hideinitializer */    
#define ANA_MODULE     MODULE_CFG(0x1UL,0,                                        MODULE_NoMsk,                        0,0,                                        MODULE_NoMsk,                        0,CLK_APBCLK_ANAEN_Pos)       /*!< ANA Module  \hideinitializer */    

/*@}*/ /* end of group I91500_CLK_EXPORTED_CONSTANTS */


/** @addtogroup I91500_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief     Get power down state.
  * @param[in] clk Base address of CLK module.
  * @param[in] u8Flag is power state flag.
  *            - \ref CLK_PFLAG_STOPF_Msk
  *            - \ref CLK_PFLAG_DSF_Msk
  * @return    power down flag
  */
#define CLK_GET_POWERDOWNFLAG(clk, u8Flag)		(clk->PFLAG&u8Flag)

/**
  * @brief     Clear power down state.
  * @param[in] clk Base address of CLK module.
  * @param[in] u8Flag is power state flag.
  *            - \ref CLK_PFLAG_STOPF_Msk
  *            - \ref CLK_PFLAG_DSF_Msk
  * @return    None.
  */
#define CLK_CLEAR_POWERDOWNFLAG(clk, u8Flag)	(clk->PFLAG |= u8Flag)

/**
  * @brief     Set DPD State Register.
  * @param[in] clk Base address of CLK module.
  * @param[in] u8Flag is a one-byte register is available PD_STATE[7:0] that can be loaded with a value to be preserved before issuing a DPD request.
  *            - \ref 0x00~0xFF
  * @return    None.
  */
#define CLK_SET_DPD_PD_STATE(clk, u8Flag)	(clk->DPDFLR |= u8Flag)

/**
  * @brief     Get current value of PD_STATE_RB register.
  * @param[in] clk Base address of CLK module.
  * @return    Current value of PD_STATE register.
  */
#define CLK_GET_DPD_PD_STATE(clk)							((clk->DPDFLR &= 0x0000FF00) >> 8)

/**
  * @brief      This function execute delay function.
  * @param      us  Delay time. The Max value is 2^24 / CPU Clock(MHz). Ex:
  *                             72MHz => 233016us, 50MHz => 335544us,
  *                             48MHz => 349525us, 49MHz => 341333us,
  *                           	28MHz => 699050us ...
  * @return     None
  * @details    Use the SysTick to generate the delay time and the unit is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  */
__STATIC_INLINE void CLK_SysTickDelay(uint32_t us)
{
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    
    /* Disable SysTick counter */
    SysTick->CTRL = 0;
}

/**
  * @brief     Enable STOP mode fast wakeup.
  * @param[in] clk Base address of CLK module.
  * @return    None.
  */
#define CLK_ENABLE_FAST_WAKEUP(clk)   (clk->PWRCTL |= CLK_PWRCTL_FWKEN_Msk)				        /*!< This macro    \hideinitializer */

/**
  * @brief     Disable STOP mode fast wakeup.
  * @param[in] clk Base address of CLK module.
  * @return    None.
  */
#define CLK_DISABLE_FAST_WAKEUP(clk)   (clk->PWRCTL &= ~CLK_PWRCTL_FWKEN_Msk)             /*!< This macro    \hideinitializer */

/**
  * @brief     Enable IO pin fast wakeup in STOP/DeepSleep mode.
  * @param[in] clk Base address of CLK module.
  * @return    None.
  */
#define CLK_ENABLE_IO_FASTWAKEUP(clk)   (clk->PWRCTL |= CLK_PWRCTL_IOFWK_Msk)				        /*!< This macro    \hideinitializer */

/**
  * @brief     Disable IO pin fast wakeup in STOP/DeepSleep mode.
  * @param[in] clk Base address of CLK module.
  * @return    None.
  */
#define CLK_DISABLE_IO_FASTWAKEUP(clk)   (clk->PWRCTL &= ~CLK_PWRCTL_IOFWK_Msk)            		/*!< This macro    \hideinitializer */

/**
  * @brief     Power down internel LDO
  * @param[in] clk Base address of CLK module.
  * @return    None.
  */
#define CLK_POWERDOWN_LDO(clk)   (clk->ILDOCTL |= (CLK_ILDOCTL_PD_Msk | CLK_ILDOCTL_SW_Msk))	/*!< This macro    \hideinitializer */

/**
  * @brief     Wake up internel LDO
  * @param[in] clk Base address of CLK module.
  * @return    None.
  */
#define CLK_WAKEUP_LDO(clk)   (clk->ILDOCTL =0)				        							/*!< This macro    \hideinitializer */

void CLK_DeepPowerDown(uint32_t u32DPDWakeupMode, uint32_t u32TimerSel);
void CLK_StandbyPowerDown(void);
void CLK_DeepSleep(void);
void CLK_Sleep(void);

void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
uint32_t CLK_GetHCLKFreq(void);
void CLK_SetHIRCFrequency(uint32_t u32Frequency);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);

void CLK_EnableX32Filter(void);
void CLK_DisableX32Filter(void);

void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);

uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);

uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
uint32_t CLK_GetPLLFreq(void);

void CLK_SetXCLKFrequency(uint32_t u32SelCLK, uint32_t u32SourceFrequency, uint32_t u32Multi);
uint32_t CLK_GetXCLKFrequency(void);

void CLK_SetMCLKIFrequency(uint32_t u32Frequency);
uint32_t CLK_GetMCLKIFrequency(void);

/*@}*/ /* end of group I91500_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_CLK_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__CLK_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
