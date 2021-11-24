/**************************************************************************//**
 * @file     sys.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/10/13 10:00a $
 * @brief    I91500 Series SYS Header File
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
#ifndef __SYS_H__
#define __SYS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_SYS_Driver SYS Driver
  @{
*/

/** @addtogroup I91500_SYS_EXPORTED_CONSTANTS SYS Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  RSTSTS constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define  SYS_RSTSTS_PORWK         (SYS_RSTSTS_PORWK_Msk)      /*!<Wakeup from DPD From POR  \hideinitializer */
#define  SYS_RSTSTS_TIMWK         (SYS_RSTSTS_TIMWK_Msk)      /*!<Wakeup from DPD From TIMER  \hideinitializer */
#define  SYS_RSTSTS_PINWK         (SYS_RSTSTS_PINWK_Msk)      /*!<Wakeup from DPD From PIN  \hideinitializer */
#define  SYS_RSTSTS_PMURSTF       (SYS_RSTSTS_PMURSTF_Msk)    /*!<Reset Source From PMU  \hideinitializer */
#define  SYS_RSTSTS_BODF          (SYS_RSTSTS_BODRF_Msk)      /*!<BOD Reset Flag  \hideinitializer */
#define  SYS_RSTSTS_LVRF          (SYS_RSTSTS_LVRF_Msk)       /*!<LVR Reset Flag  \hideinitializer */
#define  SYS_RSTSTS_WDTRF         (SYS_RSTSTS_WDTRF_Msk)      /*!<Reset Source From WDG  \hideinitializer */
#define  SYS_RSTSTS_PINRF         (SYS_RSTSTS_PINRF_Msk)      /*!<Reset Pin Reset Flag  \hideinitializer */
#define  SYS_RSTSTS_PORF          (SYS_RSTSTS_PORF_Msk)       /*!<POR Reset Flag  \hideinitializer */
#define  SYS_RSTSTS_ALL           (SYS_RSTSTS_PORF | SYS_RSTSTS_PINRF | SYS_RSTSTS_WDTRF | SYS_RSTSTS_LVRF | SYS_RSTSTS_BODF | SYS_RSTSTS_PMURSTF | SYS_RSTSTS_PINWK | SYS_RSTSTS_TIMWK | SYS_RSTSTS_PORWK)      /*!<Combine with ALL Reset Flags \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define CHIP_RST    ((0x0<<24) | SYS_IPRST0_CHIPRST_Pos  ) /*!<Reset CHIP   \hideinitializer */
#define CPU_RST     ((0x0<<24) | SYS_IPRST0_CPURST_Pos   ) /*!<Reset CPU   \hideinitializer  */
#define GPIO_RST    ((0x4<<24) | SYS_IPRST1_GPIORST_Pos  ) /*!<Reset GPIO  \hideinitializer  */
#define TMR0_RST    ((0x4<<24) | SYS_IPRST1_TMR0RST_Pos  ) /*!<Reset TMR0  \hideinitializer  */
#define TMR1_RST    ((0x4<<24) | SYS_IPRST1_TMR1RST_Pos  ) /*!<Reset TMR1  \hideinitializer  */
#define TMR2_RST    ((0x4<<24) | SYS_IPRST1_TMR2RST_Pos  ) /*!<Reset TMR2  \hideinitializer  */
#define CPD_RST     ((0x4<<24) | SYS_IPRST1_CPDRST_Pos   ) /*!<Reset CPD   \hideinitializer  */
#define PDMA_RST    ((0x4<<24) | SYS_IPRST1_PDMARST_Pos  ) /*!<Reset PDMA  \hideinitializer  */
#define I2C0_RST    ((0x4<<24) | SYS_IPRST1_I2C0RST_Pos  ) /*!<Reset I2C0  \hideinitializer  */
#define I2C1_RST    ((0x4<<24) | SYS_IPRST1_I2C1RST_Pos  ) /*!<Reset I2C1  \hideinitializer  */
#define SPI1_RST    ((0x4<<24) | SYS_IPRST1_SPI1RST_Pos  ) /*!<Reset SPI1  \hideinitializer  */
#define SPI0_RST    ((0x4<<24) | SYS_IPRST1_SPI0RST_Pos  ) /*!<Reset SPI0  \hideinitializer  */
#define I2S0_RST    ((0x4<<24) | SYS_IPRST1_I2S0RST_Pos  ) /*!<Reset I2S0  \hideinitializer   */
#define UART0_RST   ((0x4<<24) | SYS_IPRST1_UART0RST_Pos ) /*!<Reset UART0  \hideinitializer */
#define UART1_RST   ((0x4<<24) | SYS_IPRST1_UART1RST_Pos ) /*!<Reset UART1  \hideinitializer */
#define BIQ_RST     ((0x4<<24) | SYS_IPRST1_BIQRST_Pos   ) /*!<Reset BIQ  \hideinitializer   */
#define PWM0_RST    ((0x4<<24) | SYS_IPRST1_PWM0RST_Pos  ) /*!<Reset PWM0  \hideinitializer  */
#define PWM1_RST    ((0x4<<24) | SYS_IPRST1_PWM1RST_Pos  ) /*!<Reset PWM1  \hideinitializer  */
#define USBD_RST    ((0x4<<24) | SYS_IPRST1_USBRST_Pos   ) /*!<Reset USBD  \hideinitializer  */
#define SARADC_RST  ((0x4<<24) | SYS_IPRST1_SARADCRST_Pos) /*!<Reset SARADC \hideinitializer */
#define DAC_RST     ((0x4<<24) | SYS_IPRST1_DACRST_Pos   ) /*!<Reset DAC  \hideinitializer   */
#define SDADC_RST   ((0x4<<24) | SYS_IPRST1_SDADCRST_Pos ) /*!<Reset SDADC  \hideinitializer */
#define ANA_RST     ((0x4<<24) | SYS_IPRST1_ANARST_Pos   ) /*!<Reset ANA  \hideinitializer   */

/*---------------------------------------------------------------------------------------------------------*/
/*  IRCTCTL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define  SYS_IRCTCTL_FREQSEL_48M   (0x1UL<<SYS_IRCTCTL_FREQSEL_Pos)    /*!<Enable HIRC auto trim function and trim HIRC to 48 MHz. \hideinitializer */
#define  SYS_IRCTCTL_FREQSEL_49M   (0x3UL<<SYS_IRCTCTL_FREQSEL_Pos)    /*!<Enable HIRC auto trim function and trim HIRC to 49.152 MHz. \hideinitializer */

#define  SYS_IRCTCTL_LOOPSEL_4     (0x0UL<<SYS_IRCTCTL_LOOPSEL_Pos)    /*!<Trim value calculation is based on average difference in 4 clocks of reference clock.  \hideinitializer */
#define  SYS_IRCTCTL_LOOPSEL_8     (0x1UL<<SYS_IRCTCTL_LOOPSEL_Pos)    /*!<Trim value calculation is based on average difference in 8 clocks of reference clock. \hideinitializer */
#define  SYS_IRCTCTL_LOOPSEL_16    (0x2UL<<SYS_IRCTCTL_LOOPSEL_Pos)    /*!<Trim value calculation is based on average difference in 16 clocks of reference clock.  \hideinitializer */
#define  SYS_IRCTCTL_LOOPSEL_32    (0x3UL<<SYS_IRCTCTL_LOOPSEL_Pos)    /*!<Trim value calculation is based on average difference in 32 clocks of reference clock. \hideinitializer */

#define  SYS_IRCTCTL_RETRYCNT_64   (0x0UL<<SYS_IRCTCTL_RETRYCNT_Pos)   /*!<Trim value calculation is based on average difference in 4 clocks of reference clock.  \hideinitializer */
#define  SYS_IRCTCTL_RETRYCNT_128  (0x1UL<<SYS_IRCTCTL_RETRYCNT_Pos)   /*!<Trim value calculation is based on average difference in 8 clocks of reference clock. \hideinitializer */
#define  SYS_IRCTCTL_RETRYCNT_256  (0x2UL<<SYS_IRCTCTL_RETRYCNT_Pos)   /*!<Trim value calculation is based on average difference in 16 clocks of reference clock.  \hideinitializer */
#define  SYS_IRCTCTL_RETRYCNT_512  (0x3UL<<SYS_IRCTCTL_RETRYCNT_Pos)   /*!<Trim value calculation is based on average difference in 32 clocks of reference clock. \hideinitializer */

#define  SYS_IRCTCTL_REFCKSEL_HXT       (0x1UL<<SYS_IRCTCTL_REFCKSEL_Pos)    /*!<Reference Clock Selection from HXT. \hideinitializer */
#define  SYS_IRCTCTL_REFCKSEL_USBD_SOF  (0x3UL<<SYS_IRCTCTL_REFCKSEL_Pos)    /*!<Reference Clock Selection from USBD SOF. \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  IRCTIEN constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define  SYS_IRCTIEN_TRIMFAIL_INT_MASK  (SYS_IRCTIEN_TFAILIEN_Msk)    /*!<Enable trim fail interrupt function. \hideinitializer */
#define  SYS_IRCTIEN_CLKERROR_INT_MASK  (SYS_IRCTIEN_CLKEIEN_Msk)     /*!<Enable clock error interrupt function. \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  IRCTISTS constant definitions.                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define  SYS_IRCTISTS_TRIMFAIL_INT_FLAG (SYS_IRCTISTS_TFAILIF_Msk)    /*!< Trim Fail Interrupt Flag.  \hideinitializer */
#define  SYS_IRCTISTS_CLKERROR_INT_FLAG (SYS_IRCTISTS_CLKERRIF_Msk)   /*!< Clock Error Interrupt Flag.  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Multi-Function constant definitions.                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
/* How to use below #define?
Example 1: If user want to set PA.0 as PWM00 in initial function,
           user can issue following command to achieve it.

           SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA0MFP_Msk) ) | SYS_GPA_MFP_PA0MFP_PWM00  ;

*/

//GPA_MFP_PA0MFP
#define SYS_GPA_MFP_PA0MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA0MFP_I2S0_MCLK   (0x1UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for I2S0_MCLK  \hideinitializer */
#define SYS_GPA_MFP_PA0MFP_IROUT       (0x2UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for IROUT      \hideinitializer */
#define SYS_GPA_MFP_PA0MFP_SPI1_MISO1  (0x3UL<<SYS_GPA_MFP_PA0MFP_Pos)           /*!< GPA_MFP PA0 setting for SPI1_MISO1 \hideinitializer */

//GPA_MFP_PA1MFP
#define SYS_GPA_MFP_PA1MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA1MFP_I2S0_LRCK   (0x1UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for I2S0_LRCK  \hideinitializer */
#define SYS_GPA_MFP_PA1MFP_I2C0_SCL    (0x2UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for I2C0_SCL   \hideinitializer */
#define SYS_GPA_MFP_PA1MFP_SPI1_MOSI1  (0x3UL<<SYS_GPA_MFP_PA1MFP_Pos)           /*!< GPA_MFP PA1 setting for SPI1_MOSI1 \hideinitializer */

//GPA_MFP_PA2MFP
#define SYS_GPA_MFP_PA2MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA2MFP_I2S0_BCLK   (0x1UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for I2S0_BCLK  \hideinitializer */
#define SYS_GPA_MFP_PA2MFP_I2C0_SDA    (0x2UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for I2C0_SDA   \hideinitializer */
#define SYS_GPA_MFP_PA2MFP_SPI1_MOSI0  (0x3UL<<SYS_GPA_MFP_PA2MFP_Pos)           /*!< GPA_MFP PA2 setting for SPI1_MOSI0 \hideinitializer */

//GPA_MFP_PA3MFP
#define SYS_GPA_MFP_PA3MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA3MFP_I2S0_DO     (0x1UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for I2S0_DO    \hideinitializer */
#define SYS_GPA_MFP_PA3MFP_UART1_TX    (0x2UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for UART1_TX   \hideinitializer */
#define SYS_GPA_MFP_PA3MFP_SPI1_CLK    (0x3UL<<SYS_GPA_MFP_PA3MFP_Pos)           /*!< GPA_MFP PA3 setting for SPI1_CLK   \hideinitializer */

//GPA_MFP_PA4MFP
#define SYS_GPA_MFP_PA4MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA4MFP_I2S0_DI     (0x1UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for I2S0_DI    \hideinitializer */
#define SYS_GPA_MFP_PA4MFP_UART1_RX    (0x2UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for UART1_RX   \hideinitializer */
#define SYS_GPA_MFP_PA4MFP_SPI1_MISO0  (0x3UL<<SYS_GPA_MFP_PA4MFP_Pos)           /*!< GPA_MFP PA4 setting for SPI1_MISO0 \hideinitializer */

//eGPA_MFP_PA5MFP
#define SYS_GPA_MFP_PA5MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA5MFP_MCLKI       (0x1UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for MCLKI      \hideinitializer */
#define SYS_GPA_MFP_PA5MFP_SPI1_SS0    (0x3UL<<SYS_GPA_MFP_PA5MFP_Pos)           /*!< GPA_MFP PA5 setting for SPI1_SS0   \hideinitializer */

//GPA_MFP_PA6MFP
#define SYS_GPA_MFP_PA6MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA6MFP_TMR0        (0x1UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for TMR0       \hideinitializer */
#define SYS_GPA_MFP_PA6MFP_CAP1        (0x2UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for CAP1       \hideinitializer */
#define SYS_GPA_MFP_PA6MFP_PWM10       (0x3UL<<SYS_GPA_MFP_PA6MFP_Pos)           /*!< GPA_MFP PA6 setting for PWM10      \hideinitializer */

//GPA_MFP_PA7MFP
#define SYS_GPA_MFP_PA7MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA7MFP_TMR1        (0x1UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for TMR1       \hideinitializer */
#define SYS_GPA_MFP_PA7MFP_SPI1_SS1    (0x2UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for SPI1_SS1   \hideinitializer */
#define SYS_GPA_MFP_PA7MFP_PWM11       (0x3UL<<SYS_GPA_MFP_PA7MFP_Pos)           /*!< GPA_MFP PA7 setting for PWM11      \hideinitializer */

//GPA_MFP_PA8MFP
#define SYS_GPA_MFP_PA8MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA8MFP_TMR2        (0x1UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for TMR2       \hideinitializer */
#define SYS_GPA_MFP_PA8MFP_UART0_TX    (0x2UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for UART0_TX   \hideinitializer */
#define SYS_GPA_MFP_PA8MFP_PWM12       (0x3UL<<SYS_GPA_MFP_PA8MFP_Pos)           /*!< GPA_MFP PA8 setting for PWM12      \hideinitializer */

//GPA_MFP_PA9MFP
#define SYS_GPA_MFP_PA9MFP_GPIO        (0x0UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for GPIO       \hideinitializer */
#define SYS_GPA_MFP_PA9MFP_SPI0_SS1    (0x1UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for SPI0_SS1   \hideinitializer */
#define SYS_GPA_MFP_PA9MFP_UART0_RX    (0x2UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for UART0_RX   \hideinitializer */
#define SYS_GPA_MFP_PA9MFP_PWM13       (0x3UL<<SYS_GPA_MFP_PA9MFP_Pos)           /*!< GPA_MFP PA9 setting for PWM13      \hideinitializer */

//GPA_MFP_PA10MFP
#define SYS_GPA_MFP_PA10MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for GPIO      \hideinitializer */
#define SYS_GPA_MFP_PA10MFP_SPI0_MISO1 (0x1UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for SPI0_MISO1 \hideinitializer */
#define SYS_GPA_MFP_PA10MFP_MCLKI      (0x2UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for MCLKI     \hideinitializer */
#define SYS_GPA_MFP_PA10MFP_UART0_TX   (0x3UL<<SYS_GPA_MFP_PA10MFP_Pos)          /*!< GPA_MFP PA10 setting for UART0_TX  \hideinitializer */

//GPA_MFP_PA11MFP
#define SYS_GPA_MFP_PA11MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for GPIO      \hideinitializer */
#define SYS_GPA_MFP_PA11MFP_SPI0_MOSI1 (0x1UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for SPI0_MOSI1 \hideinitializer */
#define SYS_GPA_MFP_PA11MFP_I2C1_SCL   (0x2UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for I2C1_SCL  \hideinitializer */
#define SYS_GPA_MFP_PA11MFP_UART0_RX   (0x3UL<<SYS_GPA_MFP_PA11MFP_Pos)          /*!< GPA_MFP PA11 setting for UART0_RX  \hideinitializer */

//GPA_MFP_PA12MFP
#define SYS_GPA_MFP_PA12MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for GPIO      \hideinitializer */
#define SYS_GPA_MFP_PA12MFP_SPI0_MOSI0 (0x1UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for SPI0_MOSI0 \hideinitializer */
#define SYS_GPA_MFP_PA12MFP_I2C1_SDA   (0x2UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for I2C1_SDA  \hideinitializer */
#define SYS_GPA_MFP_PA12MFP_UART0_CTS  (0x3UL<<SYS_GPA_MFP_PA12MFP_Pos)          /*!< GPA_MFP PA12 setting for UART0_CTS \hideinitializer */

//GPA_MFP_PA13MFP
#define SYS_GPA_MFP_PA13MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for GPIO      \hideinitializer */
#define SYS_GPA_MFP_PA13MFP_SPI0_CLK   (0x1UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for SPI0_CLK  \hideinitializer */
#define SYS_GPA_MFP_PA13MFP_UART0_RTS  (0x3UL<<SYS_GPA_MFP_PA13MFP_Pos)          /*!< GPA_MFP PA13 setting for UART0_RTS \hideinitializer */

//GPA_MFP_PA14MFP
#define SYS_GPA_MFP_PA14MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for GPIO      \hideinitializer */
#define SYS_GPA_MFP_PA14MFP_SPI0_MISO0 (0x1UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for SPI0_MISO0 \hideinitializer */
#define SYS_GPA_MFP_PA14MFP_UART1_TX   (0x2UL<<SYS_GPA_MFP_PA14MFP_Pos)          /*!< GPA_MFP PA14 setting for UART1_TX  \hideinitializer */

//GPA_MFP_PA15MFP
#define SYS_GPA_MFP_PA15MFP_GPIO       (0x0UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for GPIO      \hideinitializer */
#define SYS_GPA_MFP_PA15MFP_SPI0_SS0   (0x1UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for SPI0_SS0  \hideinitializer */
#define SYS_GPA_MFP_PA15MFP_UART1_RX   (0x2UL<<SYS_GPA_MFP_PA15MFP_Pos)          /*!< GPA_MFP PA15 setting for UART1_RX  \hideinitializer */

//GPB_MFP_PB0MFP
#define SYS_GPB_MFP_PB0MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB0MFP_Pos)           /*!< GPB_MFP PB0 setting for GPIO       \hideinitializer */
#define SYS_GPB_MFP_PB0MFP_I2C1_SCL    (0x1UL<<SYS_GPB_MFP_PB0MFP_Pos)           /*!< GPB_MFP PB0 setting for I2C1_SCL   \hideinitializer */

//GPB_MFP_PB1MFP
#define SYS_GPB_MFP_PB1MFP_GPIO        (0x0UL<<SYS_GPB_MFP_PB1MFP_Pos)           /*!< GPB_MFP PB1 setting for GPIO       \hideinitializer */
#define SYS_GPB_MFP_PB1MFP_I2C1_SDA    (0x1UL<<SYS_GPB_MFP_PB1MFP_Pos)           /*!< GPB_MFP PB1 setting for I2C1_SDA   \hideinitializer */

//GPC_MFP_PC0MFP
#define SYS_GPC_MFP_PC0MFP_GPIO        (0x0UL<<SYS_GPC_MFP_PC0MFP_Pos)           /*!< GPC_MFP PC0 setting for GPIO       \hideinitializer */
#define SYS_GPC_MFP_PC0MFP_XT1_OUT     (0x1UL<<SYS_GPC_MFP_PC0MFP_Pos)           /*!< GPC_MFP PC0 setting for XT1_OUT    \hideinitializer */
#define SYS_GPC_MFP_PC0MFP_I2C0_SCL    (0x3UL<<SYS_GPC_MFP_PC0MFP_Pos)           /*!< GPC_MFP PC0 setting for I2C0_SCL   \hideinitializer */

//GPC_MFP_PC1MFP
#define SYS_GPC_MFP_PC1MFP_GPIO        (0x0UL<<SYS_GPC_MFP_PC1MFP_Pos)           /*!< GPC_MFP PC1 setting for GPIO       \hideinitializer */
#define SYS_GPC_MFP_PC1MFP_XT1_IN      (0x1UL<<SYS_GPC_MFP_PC1MFP_Pos)           /*!< GPC_MFP PC1 setting for XT1_IN     \hideinitializer */
#define SYS_GPC_MFP_PC1MFP_I2C0_SDA    (0x3UL<<SYS_GPC_MFP_PC1MFP_Pos)           /*!< GPC_MFP PC1 setting for I2C0_SDA   \hideinitializer */

//GPC_MFP_PC2MFP
#define SYS_GPC_MFP_PC2MFP_GPIO        (0x0UL<<SYS_GPC_MFP_PC2MFP_Pos)           /*!< GPC_MFP PC2 setting for GPIO       \hideinitializer */
#define SYS_GPC_MFP_PC2MFP_UART0_CTS   (0x1UL<<SYS_GPC_MFP_PC2MFP_Pos)           /*!< GPC_MFP PC2 setting for UART0_CTS  \hideinitializer */
#define SYS_GPC_MFP_PC2MFP_I2S0_LRCK   (0x2UL<<SYS_GPC_MFP_PC2MFP_Pos)           /*!< GPC_MFP PC2 setting for I2S0_LRCK  \hideinitializer */
#define SYS_GPC_MFP_PC2MFP_PWM00       (0x3UL<<SYS_GPC_MFP_PC2MFP_Pos)           /*!< GPC_MFP PC2 setting for PWM00      \hideinitializer */

//GPC_MFP_PC3MFP
#define SYS_GPC_MFP_PC3MFP_GPIO        (0x0UL<<SYS_GPC_MFP_PC3MFP_Pos)           /*!< GPC_MFP PC3 setting for GPIO       \hideinitializer */
#define SYS_GPC_MFP_PC3MFP_UART0_RTS   (0x1UL<<SYS_GPC_MFP_PC3MFP_Pos)           /*!< GPC_MFP PC3 setting for UART0_RTS  \hideinitializer */
#define SYS_GPC_MFP_PC3MFP_I2S0_BCLK   (0x2UL<<SYS_GPC_MFP_PC3MFP_Pos)           /*!< GPC_MFP PC3 setting for I2S0_BCLK  \hideinitializer */
#define SYS_GPC_MFP_PC3MFP_PWM01       (0x3UL<<SYS_GPC_MFP_PC3MFP_Pos)           /*!< GPC_MFP PC3 setting for PWM01      \hideinitializer */

//GPC_MFP_PC4MFP
#define SYS_GPC_MFP_PC4MFP_GPIO        (0x0UL<<SYS_GPC_MFP_PC4MFP_Pos)           /*!< GPC_MFP PC4 setting for GPIO       \hideinitializer */
#define SYS_GPC_MFP_PC4MFP_UART0_TX    (0x1UL<<SYS_GPC_MFP_PC4MFP_Pos)           /*!< GPC_MFP PC4 setting for UART0_TX   \hideinitializer */
#define SYS_GPC_MFP_PC4MFP_I2S0_DO     (0x2UL<<SYS_GPC_MFP_PC4MFP_Pos)           /*!< GPC_MFP PC4 setting for I2S0_DO    \hideinitializer */
#define SYS_GPC_MFP_PC4MFP_PWM02       (0x3UL<<SYS_GPC_MFP_PC4MFP_Pos)           /*!< GPC_MFP PC4 setting for PWM02      \hideinitializer */

//GPC_MFP_PC5MFP
#define SYS_GPC_MFP_PC5MFP_GPIO        (0x0UL<<SYS_GPC_MFP_PC5MFP_Pos)           /*!< GPC_MFP PC5 setting for GPIO       \hideinitializer */
#define SYS_GPC_MFP_PC5MFP_UART0_RX    (0x1UL<<SYS_GPC_MFP_PC5MFP_Pos)           /*!< GPC_MFP PC5 setting for UART0_RX   \hideinitializer */
#define SYS_GPC_MFP_PC5MFP_I2S0_DI     (0x2UL<<SYS_GPC_MFP_PC5MFP_Pos)           /*!< GPC_MFP PC5 setting for I2S0_DI    \hideinitializer */
#define SYS_GPC_MFP_PC5MFP_PWM03       (0x3UL<<SYS_GPC_MFP_PC5MFP_Pos)           /*!< GPC_MFP PC5 setting for PWM03      \hideinitializer */

//GPC_MFP_PC6MFP
#define SYS_GPC_MFP_PC6MFP_GPIO        (0x0UL<<SYS_GPC_MFP_PC6MFP_Pos)           /*!< GPC_MFP PC6 setting for GPIO       \hideinitializer */
#define SYS_GPC_MFP_PC6MFP_I2C0_SCL    (0x1UL<<SYS_GPC_MFP_PC6MFP_Pos)           /*!< GPC_MFP PC6 setting for I2C0_SCL   \hideinitializer */
#define SYS_GPC_MFP_PC6MFP_SPI1_SS0    (0x2UL<<SYS_GPC_MFP_PC6MFP_Pos)           /*!< GPC_MFP PC6 setting for SPI1_SS0   \hideinitializer */
#define SYS_GPC_MFP_PC6MFP_PWM10       (0x3UL<<SYS_GPC_MFP_PC6MFP_Pos)           /*!< GPC_MFP PC6 setting for PWM10      \hideinitializer */

//GPC_MFP_PC7MFP
#define SYS_GPC_MFP_PC7MFP_GPIO        (0x0UL<<SYS_GPC_MFP_PC7MFP_Pos)           /*!< GPC_MFP PC7 setting for GPIO       \hideinitializer */
#define SYS_GPC_MFP_PC7MFP_I2C0_SDA    (0x1UL<<SYS_GPC_MFP_PC7MFP_Pos)           /*!< GPC_MFP PC7 setting for I2C0_SDA   \hideinitializer */
#define SYS_GPC_MFP_PC7MFP_SPI1_SS1    (0x2UL<<SYS_GPC_MFP_PC7MFP_Pos)           /*!< GPC_MFP PC7 setting for SPI1_SS1   \hideinitializer */
#define SYS_GPC_MFP_PC7MFP_PWM11       (0x3UL<<SYS_GPC_MFP_PC7MFP_Pos)           /*!< GPC_MFP PC7 setting for PWM11      \hideinitializer */

//GPC_MFP_PC8MFP
#define SYS_GPC_MFP_PC8MFP_GPIO        (0x0UL<<SYS_GPC_MFP_PC8MFP_Pos)           /*!< GPC_MFP PC8 setting for GPIO       \hideinitializer */
#define SYS_GPC_MFP_PC8MFP_SPI0_SS1    (0x1UL<<SYS_GPC_MFP_PC8MFP_Pos)           /*!< GPC_MFP PC8 setting for SPI0_SS1   \hideinitializer */
#define SYS_GPC_MFP_PC8MFP_I2S0_MCLK   (0x2UL<<SYS_GPC_MFP_PC8MFP_Pos)           /*!< GPC_MFP PC8 setting for I2S0_MCLK  \hideinitializer */
#define SYS_GPC_MFP_PC8MFP_PWM12       (0x3UL<<SYS_GPC_MFP_PC8MFP_Pos)           /*!< GPC_MFP PC8 setting for PWM12      \hideinitializer */

//GPC_MFP_PC9MFP
#define SYS_GPC_MFP_PC9MFP_GPIO        (0x0UL<<SYS_GPC_MFP_PC9MFP_Pos)           /*!< GPC_MFP PC9 setting for GPIO       \hideinitializer */
#define SYS_GPC_MFP_PC9MFP_SPI0_MISO1  (0x1UL<<SYS_GPC_MFP_PC9MFP_Pos)           /*!< GPC_MFP PC9 setting for SPI0_MISO1 \hideinitializer */
#define SYS_GPC_MFP_PC9MFP_PWM13       (0x3UL<<SYS_GPC_MFP_PC9MFP_Pos)           /*!< GPC_MFP PC9 setting for PWM13      \hideinitializer */

//GPC_MFP_PC10MFP
#define SYS_GPC_MFP_PC10MFP_GPIO       (0x0UL<<SYS_GPC_MFP_PC10MFP_Pos)          /*!< GPC_MFP PC10 setting for GPIO      \hideinitializer */
#define SYS_GPC_MFP_PC10MFP_SPI0_MOSI1 (0x1UL<<SYS_GPC_MFP_PC10MFP_Pos)          /*!< GPC_MFP PC10 setting for SPI0_SS1  \hideinitializer */
#define SYS_GPC_MFP_PC10MFP_I2S0_MCLK  (0x2UL<<SYS_GPC_MFP_PC10MFP_Pos)          /*!< GPC_MFP PC10 setting for I2S0_MCLK \hideinitializer */
#define SYS_GPC_MFP_PC10MFP_MCLKI      (0x3UL<<SYS_GPC_MFP_PC10MFP_Pos)          /*!< GPC_MFP PC10 setting for PWM12     \hideinitializer */

//GPC_MFP_PC11MFP
#define SYS_GPC_MFP_PC11MFP_GPIO       (0x0UL<<SYS_GPC_MFP_PC11MFP_Pos)          /*!< GPC_MFP PC11 setting for GPIO      \hideinitializer */
#define SYS_GPC_MFP_PC11MFP_SPI0_MOSI0 (0x1UL<<SYS_GPC_MFP_PC11MFP_Pos)          /*!< GPC_MFP PC11 setting for SPI0_MOSI0 \hideinitializer */
#define SYS_GPC_MFP_PC11MFP_I2S0_LRCK  (0x2UL<<SYS_GPC_MFP_PC11MFP_Pos)          /*!< GPC_MFP PC11 setting for I2S0_LRCK \hideinitializer */
#define SYS_GPC_MFP_PC11MFP_UART1_CTS  (0x3UL<<SYS_GPC_MFP_PC11MFP_Pos)          /*!< GPC_MFP PC11 setting for UART1_CTS \hideinitializer */

//GPC_MFP_PC12MFP
#define SYS_GPC_MFP_PC12MFP_GPIO       (0x0UL<<SYS_GPC_MFP_PC12MFP_Pos)          /*!< GPC_MFP PC12 setting for GPIO      \hideinitializer */
#define SYS_GPC_MFP_PC12MFP_SPI0_CLK   (0x1UL<<SYS_GPC_MFP_PC12MFP_Pos)          /*!< GPC_MFP PC12 setting for SPI0_CLK  \hideinitializer */
#define SYS_GPC_MFP_PC12MFP_I2S0_BCLK  (0x2UL<<SYS_GPC_MFP_PC12MFP_Pos)          /*!< GPC_MFP PC12 setting for I2S0_BCLK \hideinitializer */
#define SYS_GPC_MFP_PC12MFP_UART1_RTS  (0x3UL<<SYS_GPC_MFP_PC12MFP_Pos)          /*!< GPC_MFP PC12 setting for UART1_RTS \hideinitializer */

//GPC_MFP_PC13MFP
#define SYS_GPC_MFP_PC13MFP_GPIO       (0x0UL<<SYS_GPC_MFP_PC13MFP_Pos)          /*!< GPC_MFP PC13 setting for GPIO      \hideinitializer */
#define SYS_GPC_MFP_PC13MFP_SPI0_MISO0 (0x1UL<<SYS_GPC_MFP_PC13MFP_Pos)          /*!< GPC_MFP PC13 setting for SPI0_MISO0 \hideinitializer */
#define SYS_GPC_MFP_PC13MFP_I2S0_DO    (0x2UL<<SYS_GPC_MFP_PC13MFP_Pos)          /*!< GPC_MFP PC13 setting for I2S0_DO   \hideinitializer */
#define SYS_GPC_MFP_PC13MFP_UART1_TX   (0x3UL<<SYS_GPC_MFP_PC13MFP_Pos)          /*!< GPC_MFP PC13 setting for UART1_TX  \hideinitializer */

//GPC_MFP_PC14MFP
#define SYS_GPC_MFP_PC14MFP_GPIO       (0x0UL<<SYS_GPC_MFP_PC14MFP_Pos)          /*!< GPC_MFP PC14 setting for GPIO      \hideinitializer */
#define SYS_GPC_MFP_PC14MFP_SPI0_SS0   (0x1UL<<SYS_GPC_MFP_PC14MFP_Pos)          /*!< GPC_MFP PC14 setting for SPI0_SS0  \hideinitializer */
#define SYS_GPC_MFP_PC14MFP_I2S0_DI    (0x2UL<<SYS_GPC_MFP_PC14MFP_Pos)          /*!< GPC_MFP PC14 setting for I2S0_DI   \hideinitializer */
#define SYS_GPC_MFP_PC14MFP_UART1_RX   (0x3UL<<SYS_GPC_MFP_PC14MFP_Pos)          /*!< GPC_MFP PC14 setting for UART1_RX  \hideinitializer */

//GPC_MFP_PC15MFP
#define SYS_GPC_MFP_PC15MFP_GPIO       (0x0UL<<SYS_GPC_MFP_PC15MFP_Pos)          /*!< GPC_MFP PC15 setting for GPIO      \hideinitializer */
#define SYS_GPC_MFP_PC15MFP_MCLKI      (0x2UL<<SYS_GPC_MFP_PC15MFP_Pos)          /*!< GPC_MFP PC15 setting for MCLKI     \hideinitializer */

//GPD_MFP_PD0MFP
#define SYS_GPD_MFP_PD0MFP_GPIO        (0x0UL<<SYS_GPD_MFP_PD0MFP_Pos)           /*!< GPD_MFP PD0 setting for GPIO       \hideinitializer */
#define SYS_GPD_MFP_PD0MFP_UART1_CTS   (0x1UL<<SYS_GPD_MFP_PD0MFP_Pos)           /*!< GPD_MFP PD0 setting for UART1_CTS  \hideinitializer */
#define SYS_GPD_MFP_PD0MFP_PWM00       (0x2UL<<SYS_GPD_MFP_PD0MFP_Pos)           /*!< GPD_MFP PD0 setting for PWM00      \hideinitializer */
#define SYS_GPD_MFP_PD0MFP_I2S0_MCLK   (0x3UL<<SYS_GPD_MFP_PD0MFP_Pos)           /*!< GPD_MFP PD0 setting for I2S0_MCLK  \hideinitializer */

//GPD_MFP_PD1MFP
#define SYS_GPD_MFP_PD1MFP_GPIO        (0x0UL<<SYS_GPD_MFP_PD1MFP_Pos)           /*!< GPD_MFP PD1 setting for GPIO       \hideinitializer */
#define SYS_GPD_MFP_PD1MFP_UART1_RTS   (0x1UL<<SYS_GPD_MFP_PD1MFP_Pos)           /*!< GPD_MFP PD1 setting for UART1_RTS  \hideinitializer */
#define SYS_GPD_MFP_PD1MFP_PWM01       (0x2UL<<SYS_GPD_MFP_PD1MFP_Pos)           /*!< GPD_MFP PD1 setting for PWM01      \hideinitializer */
#define SYS_GPD_MFP_PD1MFP_I2S0_LRCK   (0x3UL<<SYS_GPD_MFP_PD1MFP_Pos)           /*!< GPD_MFP PD1 setting for I2S0_LRCK  \hideinitializer */

//GPD_MFP_PD2MFP
#define SYS_GPD_MFP_PD2MFP_GPIO        (0x0UL<<SYS_GPD_MFP_PD2MFP_Pos)           /*!< GPD_MFP PD2 setting for GPIO       \hideinitializer */
#define SYS_GPD_MFP_PD2MFP_UART1_TX    (0x1UL<<SYS_GPD_MFP_PD2MFP_Pos)           /*!< GPD_MFP PD2 setting for UART1_TX   \hideinitializer */
#define SYS_GPD_MFP_PD2MFP_PWM02       (0x2UL<<SYS_GPD_MFP_PD2MFP_Pos)           /*!< GPD_MFP PD2 setting for PWM02      \hideinitializer */
#define SYS_GPD_MFP_PD2MFP_I2S0_BCLK   (0x3UL<<SYS_GPD_MFP_PD2MFP_Pos)           /*!< GPD_MFP PD2 setting for I2S0_BCLK  \hideinitializer */

//GPD_MFP_PD3MFP
#define SYS_GPD_MFP_PD3MFP_GPIO        (0x0UL<<SYS_GPD_MFP_PD3MFP_Pos)           /*!< GPD_MFP PD3 setting for GPIO       \hideinitializer */
#define SYS_GPD_MFP_PD3MFP_UART1_RX    (0x1UL<<SYS_GPD_MFP_PD3MFP_Pos)           /*!< GPD_MFP PD3 setting for UART1_RX   \hideinitializer */
#define SYS_GPD_MFP_PD3MFP_PWM03       (0x2UL<<SYS_GPD_MFP_PD3MFP_Pos)           /*!< GPD_MFP PD3 setting for PWM03      \hideinitializer */
#define SYS_GPD_MFP_PD3MFP_I2S0_DO     (0x3UL<<SYS_GPD_MFP_PD3MFP_Pos)           /*!< GPD_MFP PD3 setting for I2S0_DO    \hideinitializer */

//GPD_MFP_PD4MFP
#define SYS_GPD_MFP_PD4MFP_GPIO        (0x0UL<<SYS_GPD_MFP_PD4MFP_Pos)           /*!< GPD_MFP PD4 setting for GPIO       \hideinitializer */
#define SYS_GPD_MFP_PD4MFP_PWM00       (0x1UL<<SYS_GPD_MFP_PD4MFP_Pos)           /*!< GPD_MFP PD4 setting for PWM00      \hideinitializer */
#define SYS_GPD_MFP_PD4MFP_CAP0        (0x2UL<<SYS_GPD_MFP_PD4MFP_Pos)           /*!< GPD_MFP PD4 setting for CAP0       \hideinitializer */
#define SYS_GPD_MFP_PD4MFP_I2S0_DI     (0x3UL<<SYS_GPD_MFP_PD4MFP_Pos)           /*!< GPD_MFP PD4 setting for I2S0_DI    \hideinitializer */

//GPD_MFP_PD5MFP
#define SYS_GPD_MFP_PD5MFP_GPIO        (0x0UL<<SYS_GPD_MFP_PD5MFP_Pos)           /*!< GPD_MFP PD5 setting for GPIO       \hideinitializer */
#define SYS_GPD_MFP_PD5MFP_PWM01       (0x1UL<<SYS_GPD_MFP_PD5MFP_Pos)           /*!< GPD_MFP PD5 setting for PWM01      \hideinitializer */
#define SYS_GPD_MFP_PD5MFP_SPI1_MOSI0  (0x3UL<<SYS_GPD_MFP_PD5MFP_Pos)           /*!< GPD_MFP PD5 setting for SPI1_MOSI0 \hideinitializer */

//GPD_MFP_PD6MFP
#define SYS_GPD_MFP_PD6MFP_GPIO        (0x0UL<<SYS_GPD_MFP_PD6MFP_Pos)           /*!< GPD_MFP PD6 setting for GPIO       \hideinitializer */
#define SYS_GPD_MFP_PD6MFP_PWM02       (0x1UL<<SYS_GPD_MFP_PD6MFP_Pos)           /*!< GPD_MFP PD6 setting for PWM02      \hideinitializer */
#define SYS_GPD_MFP_PD6MFP_SPI1_CLK    (0x3UL<<SYS_GPD_MFP_PD6MFP_Pos)           /*!< GPD_MFP PD6 setting for SPI1_CLK   \hideinitializer */

//GPD_MFP_PD7MFP
#define SYS_GPD_MFP_PD7MFP_GPIO        (0x0UL<<SYS_GPD_MFP_PD7MFP_Pos)           /*!< GPD_MFP PD7 setting for GPIO       \hideinitializer */
#define SYS_GPD_MFP_PD7MFP_PWM03       (0x1UL<<SYS_GPD_MFP_PD7MFP_Pos)           /*!< GPD_MFP PD7 setting for PWM03      \hideinitializer */
#define SYS_GPD_MFP_PD7MFP_SPI1_CLK    (0x3UL<<SYS_GPD_MFP_PD7MFP_Pos)           /*!< GPD_MFP PD7 setting for SPI1_CLK   \hideinitializer */

//GPD_MFP_PD8MFP
#define SYS_GPD_MFP_PD8MFP_GPIO        (0x0UL<<SYS_GPD_MFP_PD8MFP_Pos)           /*!< GPD_MFP PD8 setting for GPIO       \hideinitializer */
#define SYS_GPD_MFP_PD8MFP_PWM10       (0x1UL<<SYS_GPD_MFP_PD8MFP_Pos)           /*!< GPD_MFP PD8 setting for PWM10      \hideinitializer */
#define SYS_GPD_MFP_PD8MFP_SPI0_SS0    (0x2UL<<SYS_GPD_MFP_PD8MFP_Pos)           /*!< GPD_MFP PD8 setting for SPI0_SS0   \hideinitializer */
#define SYS_GPD_MFP_PD8MFP_UART0_TX    (0x3UL<<SYS_GPD_MFP_PD8MFP_Pos)           /*!< GPD_MFP PD8 setting for UART0_TX   \hideinitializer */

//GPD_MFP_PD9MFP
#define SYS_GPD_MFP_PD9MFP_GPIO        (0x0UL<<SYS_GPD_MFP_PD9MFP_Pos)           /*!< GPD_MFP PD9 setting for GPIO       \hideinitializer */
#define SYS_GPD_MFP_PD9MFP_PWM11       (0x1UL<<SYS_GPD_MFP_PD9MFP_Pos)           /*!< GPD_MFP PD9 setting for PWM11      \hideinitializer */
#define SYS_GPD_MFP_PD9MFP_SPI0_MISO0  (0x2UL<<SYS_GPD_MFP_PD9MFP_Pos)           /*!< GPD_MFP PD9 setting for SPI0_MISO0 \hideinitializer */
#define SYS_GPD_MFP_PD9MFP_UART0_RX    (0x3UL<<SYS_GPD_MFP_PD9MFP_Pos)           /*!< GPD_MFP PD9 setting for UART0_RX   \hideinitializer */

//GPD_MFP_PD10MFP
#define SYS_GPD_MFP_PD10MFP_GPIO       (0x0UL<<SYS_GPD_MFP_PD10MFP_Pos)          /*!< GPD_MFP PD10 setting for GPIO      \hideinitializer */
#define SYS_GPD_MFP_PD10MFP_PWM12      (0x1UL<<SYS_GPD_MFP_PD10MFP_Pos)          /*!< GPD_MFP PD10 setting for PWM12     \hideinitializer */
#define SYS_GPD_MFP_PD10MFP_SPI0_CLK   (0x2UL<<SYS_GPD_MFP_PD10MFP_Pos)          /*!< GPD_MFP PD10 setting for SPI0_CLK  \hideinitializer */
#define SYS_GPD_MFP_PD10MFP_I2C1_SCL   (0x3UL<<SYS_GPD_MFP_PD10MFP_Pos)          /*!< GPD_MFP PD10 setting for I2C1_SCL  \hideinitializer */

//GPD_MFP_PD11MFP
#define SYS_GPD_MFP_PD11MFP_GPIO       (0x0UL<<SYS_GPD_MFP_PD11MFP_Pos)          /*!< GPD_MFP PD11 setting for GPIO      \hideinitializer */
#define SYS_GPD_MFP_PD11MFP_PWM13      (0x1UL<<SYS_GPD_MFP_PD11MFP_Pos)          /*!< GPD_MFP PD11 setting for PWM13     \hideinitializer */
#define SYS_GPD_MFP_PD11MFP_SPI0_MOSI0 (0x2UL<<SYS_GPD_MFP_PD11MFP_Pos)          /*!< GPD_MFP PD11 setting for SPI0_MOSI0 \hideinitializer */
#define SYS_GPD_MFP_PD11MFP_I2C1_SDA   (0x3UL<<SYS_GPD_MFP_PD11MFP_Pos)          /*!< GPD_MFP PD11 setting for I2C1_SDA  \hideinitializer */

//GPD_MFP_PD12MFP
#define SYS_GPD_MFP_PD12MFP_GPIO       (0x0UL<<SYS_GPD_MFP_PD12MFP_Pos)          /*!< GPD_MFP PD12 setting for GPIO      \hideinitializer */
#define SYS_GPD_MFP_PD12MFP_I2C1_SCL   (0x1UL<<SYS_GPD_MFP_PD12MFP_Pos)          /*!< GPD_MFP PD12 setting for I2C1_SCL  \hideinitializer */
#define SYS_GPD_MFP_PD12MFP_SPI0_MOSI1 (0x2UL<<SYS_GPD_MFP_PD12MFP_Pos)          /*!< GPD_MFP PD12 setting for SPI0_MOSI1 \hideinitializer */
#define SYS_GPD_MFP_PD12MFP_ICE_CLK    (0x3UL<<SYS_GPD_MFP_PD12MFP_Pos)          /*!< GPD_MFP PD12 setting for ICE_CLK   \hideinitializer */

//GPD_MFP_PD13MFP
#define SYS_GPD_MFP_PD13MFP_GPIO       (0x0UL<<SYS_GPD_MFP_PD12MFP_Pos)          /*!< GPD_MFP PD12 setting for GPIO      \hideinitializer */
#define SYS_GPD_MFP_PD13MFP_I2C1_SCL   (0x1UL<<SYS_GPD_MFP_PD12MFP_Pos)          /*!< GPD_MFP PD12 setting for I2C1_SCL  \hideinitializer */
#define SYS_GPD_MFP_PD13MFP_SPI0_MOSI1 (0x2UL<<SYS_GPD_MFP_PD12MFP_Pos)          /*!< GPD_MFP PD12 setting for SPI0_MOSI1 \hideinitializer */
#define SYS_GPD_MFP_PD13MFP_ICE_CLK    (0x3UL<<SYS_GPD_MFP_PD12MFP_Pos)          /*!< GPD_MFP PD12 setting for ICE_CLK   \hideinitializer */


/*@}*/ /* end of group I91500_SYS_EXPORTED_CONSTANTS */

/** @addtogroup I91500_SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

#define SYS_IS_POR_RST()                   (SYS->RSTSTS & SYS_RSTSTS_PORF_Msk)    /*!< This macro get previous reset source is from Power-on Reset   \hideinitializer */
#define SYS_IS_PIN_RST()                   (SYS->RSTSTS & SYS_RSTSTS_PINRF_Msk)   /*!< This macro get previous reset source is from Pin reset   \hideinitializer */
#define SYS_IS_WDT_RST()                   (SYS->RSTSTS & SYS_RSTSTS_WDTRF_Msk)   /*!< This macro get previous reset source is from Watch dog reset   \hideinitializer */
#define SYS_IS_LV_RST()                    (SYS->RSTSTS & SYS_RSTSTS_LVRF_Msk)    /*!< This macro get previous reset source is from Lower voltage reset   \hideinitializer */
#define SYS_IS_PMU_RST()                   (SYS->RSTSTS & SYS_RSTSTS_PMURSTF_Msk) /*!< This macro get previous reset source is from Power manager unit reset   \hideinitializer */

#define SYS_CLEAR_RST_SOURCE(u32RSTSTS)    (SYS->RSTSTS = u32RSTSTS)              /*!< This macro clears reset source   \hideinitializer */

#define SYS_DISABLE_POR()                  SYS_DisablePOR()                             /*!< This macro disable Power-on Reset function \hideinitializer */
#define SYS_ENABLE_POR()                   SYS_EnablePOR()                              /*!< This macro enable Power-on Reset function \hideinitializer */

#define SYS_SET_GPIO_INPUT_TYPE_SMITT(u32PinMask)       (SYS->GPIO_INTP |= u32PinMask)  /*!< This macro set GPIOA high speed transition to 50MHz   \hideinitializer */
#define SYS_SET_GPIO_INPUT_TYPE_CMOS(u32PinMask)        (SYS->GPIO_INTP &= ~u32PinMask) /*!< This macro set GPIOA normal speed transition less than 25MHz.   \hideinitializer */

#define SYS_SET_GPIO_OUTPUT_HIGH_SLEW_RATE(u32PinMask)  (SYS->GPIO_INTP |= (u32PinMask<<1))         /*!< This macro set GPIOB high speed transition to 50MHz   \hideinitializer */
#define SYS_SET_GPIO_OUTPUT_LOW_SLEW_RATE(u32PinMask)   (SYS->GPIO_INTP &= ~(u32PinMask<<1))        /*!< This macro set GPIOB normal speed transition less than 25MHz.   \hideinitializer */

#define SYS_ENABLE_GPIOA_PULL_UP(u32PinMask)  (SYS->GPA_PULL |= (u32PinMask&0xFFFF))       /*!< This macro enable GPIOA input mode pull-up   \hideinitializer */
#define SYS_DISABLE_GPIOA_PULL_UP(u32PinMask) (SYS->GPA_PULL &= ~(u32PinMask&0xFFFF))      /*!< This macro disable GPIOA input mode pull-up   \hideinitializer */
#define SYS_ENABLE_GPIOB_PULL_UP(u32PinMask)  (SYS->GPB_PULL |= (u32PinMask&0x3F))         /*!< This macro enable GPIOB input mode pull-up   \hideinitializer */
#define SYS_DISABLE_GPIOB_PULL_UP(u32PinMask) (SYS->GPB_PULL &= ~(u32PinMask&0x3F))        /*!< This macro disable GPIOB input mode pull-up   \hideinitializer */

#define SYS_ENABLE_GPIOA_DIGITAL_INPUT_BUF(u32PinMask)  (SYS->GPA_IEN &= ~(u32PinMask&0xFFFF))     /*!< This macro enable GPIOA digital input buffer   \hideinitializer */
#define SYS_DISABLE_GPIOA_DIGITAL_INPUT_BUF(u32PinMask) (SYS->GPA_IEN |= (u32PinMask&0xFFFF))      /*!< This macro disable GPIOA digital input buffer   \hideinitializer */
#define SYS_ENABLE_GPIOB_DIGITAL_INPUT_BUF(u32PinMask)  (SYS->GPB_IEN &= ~(u32PinMask&0x3F))       /*!< This macro enable GPIOB digital input buffer   \hideinitializer */
#define SYS_DISABLE_GPIOB_DIGITAL_INPUT_BUF(u32PinMask) (SYS->GPB_IEN |= (u32PinMask&0x3F))        /*!< This macro disable GPIOB digital input buffer   \hideinitializer */

#define SYS_READ_ROMMAP0()                 (*((uint32_t*)&SYS->IMGMAP0))                /*!< This macro read ROMMAP0 image data   \hideinitializer */
#define SYS_READ_ROMMAP1()                 (*((uint32_t*)&SYS->IMGMAP1))                /*!< This macro read ROMMAP1 image data   \hideinitializer */
#define SYS_READ_ROMMAP2()                 (*((uint32_t*)&SYS->IMGMAP2))                /*!< This macro read ROMMAP2 image data   \hideinitializer */
#define SYS_READ_ROMMAP3()                 (*((uint32_t*)&SYS->IMGMAP3))                /*!< This macro read ROMMAP3 image data   \hideinitializer */

#define SYS_ENABLE_ICE_PIN()               SYS_EnableICEPin()					        /*!< This macro set ICE_TCK and ICE_TDA to be ICE CLCOK/ ICE DIO,for debug purpose.   \hideinitializer */
#define SYS_DISABLE_ICE_PIN()              SYS_DisableICEPin()                          /*!< This macro set ICE_TCK and ICE_TDA to be GPB4 and GPB5,for general IO purpose.   \hideinitializer */

#define SYS_SET_PA_DC_OFFSET(u8AdjValue)   SYS_SetPADCOffset(u8AdjValue)			    /*!< This macro turn the DC offset voltage between SPK+/SPK-.   \hideinitializer */

#define SYS_GET_OSCTRIM_VALUE()            (SYS->OSCTRIM & SYS_OSCTRIM_TRIM_Msk)        /*!< This macro get the TRIM Value from OSCTRIM.   \hideinitializer */

/**
  * @brief      Enable Trim HIRC
  * @param      Enable interrupt type(SYS_IRCTIEN_TRIMFAIL_INT_MASK or SYS_IRCTIEN_CLKERROR_INT_MASK)
  * @return     None
  * @details    This macro enable trim HIRC interrupt function.(clock error & trim fail)
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_ENABLE_TRIMHIRC_INT(u32Mask)     (SYS->IRCTIEN = (SYS->IRCTIEN&~(SYS_IRCTIEN_CLKEIEN_Msk|SYS_IRCTIEN_TFAILIEN_Msk))|u32Mask)

/**
  * @brief      Disable Trim HIRC
  * @param      None
  * @return     None
  * @details    This macro disable trim HIRC interrupt function.(clock error & trim fail)
  *             The register write-protection function should be disabled before using this macro.
  */
#define SYS_DISABLE_TRIMHIRC_INT()           (SYS->IRCTIEN = 0)

/**
  * @brief      Get trim HIRC interrupt flag
  * @param      Check triggered interrupt flag mask.
  *             - \ref SYS_IRCTISTS_TRIMFAIL_INT_FLAG
  *             - \ref SYS_IRCTISTS_CLKERROR_INT_FLAG
  * @retval     Triggered interrupt flag.
  * @details    This macro get trim HIRC interrupt flag.
  */
#define SYS_GET_TRIMHIRC_INT_FLAG(u32Mask)   (SYS->IRCTISTS & (u32Mask))

/**
  * @brief      Clear trim HIRC interrupt flag
  * @param      Clear interrupt flag mask.
  *             - \ref SYS_IRCTISTS_TRIMFAIL_INT_FLAG
  *             - \ref SYS_IRCTISTS_CLKERROR_INT_FLAG
  * @return     None
  * @details    This macro clear trim HIRC interrupt flag.
  */
#define SYS_CLEAR_TRIMHIRC_INT_FLAG(u32Mask) (SYS->IRCTISTS |= (u32Mask))

/**
  * @brief      Trim HIRC is done(lock set frequency)
  * @param      None
  * @return     0   The internal high-speed oscillator frequency doesn't lock at setting frequency yest 
  * @retval     >=1 The internal high-speed oscillator frequency locked lock at setting frequency. 
  * @details    This macro get Trim HIRC is done status.
  */
#define SYS_IS_TRIMHIRC_DONE()               (SYS->IRCTISTS & SYS_IRCTISTS_FREQLOCK_Msk)

#define SYS_SET_TRIMHIRC_LOOPSEL(u32LoopSel)     (SYS->IRCTCTL = (SYS->IRCTCTL&~SYS_IRCTCTL_LOOPSEL_Msk) | (u32LoopSel))
#define SYS_SET_TRIMHIRC_RETRYCNT(u32RetryCnt)   (SYS->IRCTCTL = (SYS->IRCTCTL&~SYS_IRCTCTL_RETRYCNT_Msk) | (u32RetryCnt))
#define SYS_ENABLE_TRIMHIRC_CLKERRSTOP()         (SYS->IRCTCTL |= SYS_IRCTCTL_CESTOPEN_Msk)
#define SYS_DISABLE_TRIMHIRC_CLKERRSTOP()        (SYS->IRCTCTL &= ~SYS_IRCTCTL_CESTOPEN_Msk)
#define SYS_SET_TRIMHIRC_REFCLK(u32RefClk)       (SYS->IRCTCTL = (SYS->IRCTCTL&~SYS_IRCTCTL_REFCKSEL_Msk) | (u32RefClk))

/*---------------------------------------------------------------------------------------------------------*/
/* functions                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
void SYS_LockReg(void);
void SYS_Lock(uint8_t u8Lock);
void SYS_UnlockReg(void);
uint8_t SYS_Unlock(void);
uint32_t SYS_ReadPDID(void);
uint32_t SYS_ReadDeviceID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_DisablePOR(void);
void SYS_EnablePOR(void);
void SYS_SetPADCOffset(uint8_t u8AdjValue);
void SYS_DisableICEPin(void);
void SYS_EnableICEPin(void);

void SYS_OSCTRIM_Increase(void);
void SYS_OSCTRIM_Decrease(void);
void SYS_EnableTrimHIRC(uint32_t u32FreqSel);
void SYS_DisableTrimHIRC(void);

/*@}*/ /* end of group I91500_SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_SYS_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif 

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
