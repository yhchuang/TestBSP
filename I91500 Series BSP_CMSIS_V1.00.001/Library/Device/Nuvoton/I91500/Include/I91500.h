/**************************************************************************//**
 * @file     I91500.h
 * @version  V1.0
 * $Revision: 1 $
 * $Date: 20/07/15 11:06a $
 * @brief    I91500 Series Peripheral Access Layer Header File
 *
 * @note
 *
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/**
  \mainpage Introduction
  *
  *
  * This user manual describes the usage of I91500 Series MCU device driver
  *
  * <b>Disclaimer</b>
  *
  * The Software is furnished "AS IS", without warranty as to performance or results, and
  * the entire risk as to performance or results is assumed by YOU. Nuvoton disclaims all
  * warranties, express, implied or otherwise, with regard to the Software, its use, or
  * operation, including without limitation any and all warranties of merchantability, fitness
  * for a particular purpose, and non-infringement of intellectual property rights.
  *
  * <b>Copyright Notice</b>
  *
  * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
  */

/**
  * \page PG_DIR Directory Structure
  * Please refer to Readme.pdf under BSP root directory for the BSP directory structure. 
  *
  *
  * \page PG_REV Revision History
  *
*/

#ifndef __I91500_H__
#define __I91500_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
*/

/** @addtogroup I91500_CMSIS Device Definitions for CMSIS
  I91500 Interrupt Number Definition and Configurations for CMSIS
  @{
*/

/**
 * @details  Interrupt Number Definition. The maximum of 32 Specific Interrupts are possible.
 */

typedef	enum IRQn
{
/******	 Cortex-M0 Processor Exceptions	Numbers	***************************************************/
	NonMaskableInt_IRQn		= -14,		/*!< 2 Non Maskable	Interrupt							  */
	HardFault_IRQn			= -13,		/*!< 3 Cortex-M0 Hard Fault	Interrupt					  */
	SVCall_IRQn				= -5,		/*!< 11	Cortex-M0 SV Call Interrupt						  */
	PendSV_IRQn				= -2,		/*!< 14	Cortex-M0 Pend SV Interrupt						  */
	SysTick_IRQn			= -1,		/*!< 15	Cortex-M0 System Tick Interrupt					  */

/******	 I91500 specific Interrupt Numbers ********************************************************/
	WDT_IRQn				= 0,
	DAC_IRQn				= 1,
	SARADC_IRQn				= 2,
	SDADC_IRQn				= 3,
	I2S0_IRQn				= 4,
	TMR0_IRQn				= 5,
	TMR1_IRQn				= 6,
	TMR2_IRQn				= 7,
	GPA_IRQn				= 8,
	GPB_IRQn				= 9,
	GPC_IRQn				= 10,
	GPD_IRQn				= 11,
	SPI0_IRQn				= 12,
	PWM0_IRQn				= 13,
	PWM1_IRQn				= 14,
	PDMA_IRQn               = 15,
	I2C0_IRQn				= 16,
	I2C1_IRQn				= 17,
	BOD_IRQn                = 18,
	UART0_IRQn              = 20,
	UART1_IRQn              = 21,
	IRCTRIM_IRQn			= 22,
	USBD_IRQn				= 23,
	CPD_IRQn				= 24,	
	XCLKF_IRQn				= 25,
	SPI1_IRQn				= 26		/*!< maximum of	32 Interrupts are possible				  */
} IRQn_Type;

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT           0       /*!< armikcmu does not provide a MPU present or not       */
#define __NVIC_PRIO_BITS        2       /*!< armikcmu Supports 2 Bits for the Priority Levels     */
#define __Vendor_SysTickConfig  0       /*!< Set to 1 if different SysTick Config is used         */
/*@}*/ /* end of group I91500_CMSIS */

#include "core_cm0.h"                   /* Cortex-M0 processor and core peripherals               */
#include "system_I91500.h"              /* I91500 System include file                             */

#if defined ( __CC_ARM   )
#pragma anon_unions
// IAR C compiler detected
#elif  ( defined (__ICCARM__) )
  #define __wfi       __WFI
  #ifndef __STATIC_INLINE
    #define __STATIC_INLINE  static inline
  #endif
/*
Usage of #define
  #define A(x)  T_##x
  #define B(x) #@x
  #define C(x) #x

  A(1)------>T_1
  B(1)------>'1'
  C(1)------>"1"
*/
  #define __quote(n)      #n
  #define __iar_align(n)  __quote(data_alignment=##n)
  #define __align(n)      _Pragma(__iar_align(n))
#elif ( defined(__GNUC__) )
// GNU C compiler detected
  #define __inline      inline
  #define __isb(n)      __ISB()
  #define __wfi         __WFI
  #define __weak	    __attribute__((weak))
  #define __align(n)    __attribute__ ((aligned (n))) 
#endif

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup REGISTER Control Register

  @{

*/

/*---------------------- Analog Functional Blocks -------------------------*/
/**
    @addtogroup ANA Analog Functional Blocks(ANA)
    Memory Mapped Structure for ANA Controller
@{ */
 
typedef struct
{


/**
 * @var ANA_T::VMID
 * Offset: 0x00  VMID Reference Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VMIDHPD   |VMIDH Pulldown
 * |        |          |0= Release VMIDH pin for reference operation.
 * |        |          |1= Pull VMIDH pin to ground. Default power down and reset condition.
 * |[1]     |VMIDHRL   |Power Down VMIDH Low (4.8kOhm) Resistance Reference
 * |        |          |0= Connect the Low Resistance reference to VMIDH
 * |        |          |Use this setting for fast power up of VMIDH
 * |        |          |Can be turned off after 50ms to save power.
 * |        |          |1= The Low Resistance reference is disconnected from VMIDH. Default power down and reset condition.
 * |[2]     |VMIDHRH   |Power Down VMIDH High (360kOhm) Resistance Reference
 * |        |          |0= Connect the High Resistance reference to VMIDH. Use this setting for minimum power consumption.
 * |        |          |1= The High Resistance reference is disconnected from VMIDH. Default power down and reset condition.
 * |[4]     |VMIDLPD   |VMIDH Pulldown
 * |        |          |0= Release VMIDL pin for reference operation.
 * |        |          |1= Pull VMIDL pin to ground. Default power down and reset condition.
 * |[5]     |VMIDLRL   |Power Down VMIDL Low (4.8kOhm) Resistance Reference
 * |        |          |0= Connect the Low Resistance reference to VMIDL
 * |        |          |Use this setting for fast power up of VMIDH
 * |        |          |Can be turned off after 50ms to save power.
 * |        |          |1= The Low Resistance reference is disconnected from VMIDL. Default power down and reset condition.
 * |[6]     |VMIDLRH   |Power Down VMIDL High (360kOhm) Resistance Reference
 * |        |          |0= Connect the High Resistance reference to VMIDL. Use this setting for minimum power consumption.
 * |        |          |1= The High Resistance reference is disconnected from VMIDL. Default power down and reset condition.
 * @var ANA_T::MICBCTR
 * Offset: 0x04  Microphone Bias Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |MICBEN    |MICBIAS enable
 * |        |          |0: Disable MIC_BIAS
 * |        |          |1: Enable MIC_BIAS
 * |[2:1]   |MICBVSEL  |Select Microphone Bias Voltage
 * |        |          |MICBMODE = 0
 * |        |          |0: 90% VCCA
 * |        |          |1: 65% VCCA
 * |        |          |2: 75% VCCA
 * |        |          |3: 50% VCCA
 * |        |          |MICBMODE = 1
 * |        |          |0: 2.4V
 * |        |          |1: 1.7V
 * |        |          |2: 2.0V
 * |        |          |3: 1.3V
 * |[3]     |MICBMODE  |Select Reference Source For MICBIAS Generator
 * |        |          |VMID provides superior noise performance for MICBIAS generation and should be used unless fixed voltage is absolutely necessary, then noise performance can be sacrificed and bandgap voltage used as reference.
 * |        |          |0= VMID ( VCCA/2) is reference source.
 * |        |          |1= VBG (bandgap voltage reference) is reference source.
 * |        |          |Select MICBIAS Generator Mode
 * |        |          |This bit can select MICBIAS as a fixed output or ratio of VCCA.
 * |        |          |0= MICBIAS output ratio of VCCA
 * |        |          |1= MICBIAS output fixed DC voltage
 */
    __IO uint32_t VMID;                  /*!< [0x0000] VMID Reference Control Register                                  */
    __IO uint32_t MICBCTR;               /*!< [0x0004] Microphone Bias Control Register                                 */

} ANA_T;

/**
    @addtogroup ANA_CONST ANA Bit Field Definition
    Constant Definitions for ANA Controller
@{ */

#define ANA_VMID_VMIDHPD_Pos             (0)                                               /*!< ANA_T::VMID: VMIDHPD Position          */
#define ANA_VMID_VMIDHPD_Msk             (0x1ul << ANA_VMID_VMIDHPD_Pos)                   /*!< ANA_T::VMID: VMIDHPD Mask              */

#define ANA_VMID_VMIDHRL_Pos             (1)                                               /*!< ANA_T::VMID: VMIDHRL Position          */
#define ANA_VMID_VMIDHRL_Msk             (0x1ul << ANA_VMID_VMIDHRL_Pos)                   /*!< ANA_T::VMID: VMIDHRL Mask              */

#define ANA_VMID_VMIDHRH_Pos             (2)                                               /*!< ANA_T::VMID: VMIDHRH Position          */
#define ANA_VMID_VMIDHRH_Msk             (0x1ul << ANA_VMID_VMIDHRH_Pos)                   /*!< ANA_T::VMID: VMIDHRH Mask              */

#define ANA_VMID_VMIDLPD_Pos             (4)                                               /*!< ANA_T::VMID: VMIDLPD Position          */
#define ANA_VMID_VMIDLPD_Msk             (0x1ul << ANA_VMID_VMIDLPD_Pos)                   /*!< ANA_T::VMID: VMIDLPD Mask              */

#define ANA_VMID_VMIDLRL_Pos             (5)                                               /*!< ANA_T::VMID: VMIDLRL Position          */
#define ANA_VMID_VMIDLRL_Msk             (0x1ul << ANA_VMID_VMIDLRL_Pos)                   /*!< ANA_T::VMID: VMIDLRL Mask              */

#define ANA_VMID_VMIDLRH_Pos             (6)                                               /*!< ANA_T::VMID: VMIDLRH Position          */
#define ANA_VMID_VMIDLRH_Msk             (0x1ul << ANA_VMID_VMIDLRH_Pos)                   /*!< ANA_T::VMID: VMIDLRH Mask              */

#define ANA_MICBCTR_MICBEN_Pos           (0)                                               /*!< ANA_T::MICBCTR: MICBEN Position        */
#define ANA_MICBCTR_MICBEN_Msk           (0x1ul << ANA_MICBCTR_MICBEN_Pos)                 /*!< ANA_T::MICBCTR: MICBEN Mask            */

#define ANA_MICBCTR_MICBVSEL_Pos         (1)                                               /*!< ANA_T::MICBCTR: MICBVSEL Position      */
#define ANA_MICBCTR_MICBVSEL_Msk         (0x3ul << ANA_MICBCTR_MICBVSEL_Pos)               /*!< ANA_T::MICBCTR: MICBVSEL Mask          */

#define ANA_MICBCTR_MICBMODE_Pos         (3)                                               /*!< ANA_T::MICBCTR: MICBMODE Position      */
#define ANA_MICBCTR_MICBMODE_Msk         (0x1ul << ANA_MICBCTR_MICBMODE_Pos)               /*!< ANA_T::MICBCTR: MICBMODE Mask          */

/**@}*/ /* ANA_CONST */
/**@}*/ /* end of ANA register group */


/*---------------------- Biquad Filter -------------------------*/
/**
    @addtogroup BIQ Biquad Filter(BIQ)
    Memory Mapped Structure for BIQ Controller
@{ */
 
typedef struct
{


/**
 * @var BIQ_T::COEFF0
 * Offset: 0x00  Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF1
 * Offset: 0x04  Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF2
 * Offset: 0x08  Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF3
 * Offset: 0x0C  Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF4
 * Offset: 0x10  Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF5
 * Offset: 0x14  Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF6
 * Offset: 0x18  Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF7
 * Offset: 0x1C  Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF8
 * Offset: 0x20  Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF9
 * Offset: 0x24  Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF10
 * Offset: 0x28  Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF11
 * Offset: 0x2C  Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF12
 * Offset: 0x30  Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF13
 * Offset: 0x34  Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF14
 * Offset: 0x38  Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF15
 * Offset: 0x3C  Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF16
 * Offset: 0x40  Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF17
 * Offset: 0x44  Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF18
 * Offset: 0x48  Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF19
 * Offset: 0x4C  Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF20
 * Offset: 0x50  Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF21
 * Offset: 0x54  Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF22
 * Offset: 0x58  Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF23
 * Offset: 0x5C  Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF24
 * Offset: 0x60  Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF25
 * Offset: 0x64  Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF26
 * Offset: 0x68  Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF27
 * Offset: 0x6C  Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF28
 * Offset: 0x70  Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF29
 * Offset: 0x74  Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::CTL
 * Offset: 0x80  BIQ Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BIQEN     |BIQ Filter Start to Run
 * |        |          |0 = BIQ filter is not processing.
 * |        |          |1 = BIQ filter is on.
 * |[1]     |HPFON     |High Pass Filter On
 * |        |          |0 = disable high pass filter.
 * |        |          |1 = enable high pass filter.
 * |        |          |Note :
 * |        |          |If this register is on, BIQ only 5 stage left.
 * |        |          |SDADC path sixth stage coefficient is for HPF filter coefficient.
 * |        |          |DAC path first stage coefficient is for HPF filter coefficient. 
 * |[2]     |PATHSEL   |AC Path Selection for BIQ
 * |        |          |0 = used in SDADC path.
 * |        |          |1 = used in DAC path.
 * |[3]     |DLCOEFF   |Move BIQ Out of Reset State
 * |        |          |0 = BIQ filter is in reset state.
 * |        |          |1 = When this bit is on, the default coefficients will be downloaded to the coefficient ram automatically in 32 internal system clocks
 * |        |          |Processor must delay enough time before changing the coefficients or turn the BIQ on.
 * |[6:4]   |SDADCWNSR |SDADC Down Sample
 * |        |          |001--- 1x (no down sample)
 * |        |          |010 --- 2x
 * |        |          |011 --- 3x
 * |        |          |100 --- 4x
 * |        |          |11 0--- 6x
 * |        |          |Others reserved
 * |[7]     |PRGCOEFF  |Programming Mode Coefficient Control Bit
 * |        |          |0 = Coefficient RAM is in normal mode.
 * |        |          |1 = coefficient RAM is under programming mode.
 * |        |          |This bit must be turned off when BIQEN in on.
 * |[11]    |STAGE     |BIQ Stage Number Control
 * |        |          |0 = 6 stage.
 * |        |          |1 = 5 stage.
 * @var BIQ_T::STS
 * Offset: 0x84  BIQ Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BIST1EN   |RAM BIST1 testing Enable for internal use
 * |        |          |(Only test load coeficient and verify)
 * |[1]     |BIST1F    |RAM BIST1 testing FAILED indicator for internal use
 * |[2]     |BIST1D    |RAM BIST1 testing DONE flag for internal use
 * |[8]     |BIST2EN   |RAM BIST2 testing Enable for internal use
 * |        |          |(Test RAM write/read with several values)
 * |[9]     |BIST2F    |RAM BIST2 testing FAILED indicator for internal use
 * |[10]    |BIST2D    |RAM BIST2 testing DONE flag for internal use
 * |[31]    |RAMINITF  |Coefficient Ram Initial Default Done Flag
 * |        |          |0 = initial default value done.
 * |        |          |1 = still working on.
 */
    __IO uint32_t COEFF0;                /*!< [0x0000] Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients */
    __IO uint32_t COEFF1;                /*!< [0x0004] Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients */
    __IO uint32_t COEFF2;                /*!< [0x0008] Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients */
    __IO uint32_t COEFF3;                /*!< [0x000c] Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients */
    __IO uint32_t COEFF4;                /*!< [0x0010] Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients */
    __IO uint32_t COEFF5;                /*!< [0x0014] Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients */
    __IO uint32_t COEFF6;                /*!< [0x0018] Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients */
    __IO uint32_t COEFF7;                /*!< [0x001c] Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients */
    __IO uint32_t COEFF8;                /*!< [0x0020] Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients */
    __IO uint32_t COEFF9;                /*!< [0x0024] Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients */
    __IO uint32_t COEFF10;               /*!< [0x0028] Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients */
    __IO uint32_t COEFF11;               /*!< [0x002c] Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients */
    __IO uint32_t COEFF12;               /*!< [0x0030] Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients */
    __IO uint32_t COEFF13;               /*!< [0x0034] Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients */
    __IO uint32_t COEFF14;               /*!< [0x0038] Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients */
    __IO uint32_t COEFF15;               /*!< [0x003c] Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients */
    __IO uint32_t COEFF16;               /*!< [0x0040] Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients */
    __IO uint32_t COEFF17;               /*!< [0x0044] Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients */
    __IO uint32_t COEFF18;               /*!< [0x0048] Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients */
    __IO uint32_t COEFF19;               /*!< [0x004c] Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients */
    __IO uint32_t COEFF20;               /*!< [0x0050] Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients */
    __IO uint32_t COEFF21;               /*!< [0x0054] Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients */
    __IO uint32_t COEFF22;               /*!< [0x0058] Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients */
    __IO uint32_t COEFF23;               /*!< [0x005c] Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients */
    __IO uint32_t COEFF24;               /*!< [0x0060] Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients */
    __IO uint32_t COEFF25;               /*!< [0x0064] Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients */
    __IO uint32_t COEFF26;               /*!< [0x0068] Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients */
    __IO uint32_t COEFF27;               /*!< [0x006c] Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients */
    __IO uint32_t COEFF28;               /*!< [0x0070] Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients */
    __IO uint32_t COEFF29;               /*!< [0x0074] Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients */
    __I  uint32_t RESERVE0[2];
    __IO uint32_t CTL;                   /*!< [0x0080] BIQ Control Register                                             */
    __IO uint32_t STS;                   /*!< [0x0084] BIQ Status Register                                              */

} BIQ_T;

/**
    @addtogroup BIQ_CONST BIQ Bit Field Definition
    Constant Definitions for BIQ Controller
@{ */

#define BIQ_COEFF0_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF0: COEFFDAT Position       */
#define BIQ_COEFF0_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF0_COEFFDAT_Pos)         /*!< BIQ_T::COEFF0: COEFFDAT Mask           */

#define BIQ_COEFF1_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF1: COEFFDAT Position       */
#define BIQ_COEFF1_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF1_COEFFDAT_Pos)         /*!< BIQ_T::COEFF1: COEFFDAT Mask           */

#define BIQ_COEFF2_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF2: COEFFDAT Position       */
#define BIQ_COEFF2_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF2_COEFFDAT_Pos)         /*!< BIQ_T::COEFF2: COEFFDAT Mask           */

#define BIQ_COEFF3_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF3: COEFFDAT Position       */
#define BIQ_COEFF3_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF3_COEFFDAT_Pos)         /*!< BIQ_T::COEFF3: COEFFDAT Mask           */

#define BIQ_COEFF4_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF4: COEFFDAT Position       */
#define BIQ_COEFF4_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF4_COEFFDAT_Pos)         /*!< BIQ_T::COEFF4: COEFFDAT Mask           */

#define BIQ_COEFF5_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF5: COEFFDAT Position       */
#define BIQ_COEFF5_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF5_COEFFDAT_Pos)         /*!< BIQ_T::COEFF5: COEFFDAT Mask           */

#define BIQ_COEFF6_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF6: COEFFDAT Position       */
#define BIQ_COEFF6_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF6_COEFFDAT_Pos)         /*!< BIQ_T::COEFF6: COEFFDAT Mask           */

#define BIQ_COEFF7_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF7: COEFFDAT Position       */
#define BIQ_COEFF7_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF7_COEFFDAT_Pos)         /*!< BIQ_T::COEFF7: COEFFDAT Mask           */

#define BIQ_COEFF8_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF8: COEFFDAT Position       */
#define BIQ_COEFF8_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF8_COEFFDAT_Pos)         /*!< BIQ_T::COEFF8: COEFFDAT Mask           */

#define BIQ_COEFF9_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF9: COEFFDAT Position       */
#define BIQ_COEFF9_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF9_COEFFDAT_Pos)         /*!< BIQ_T::COEFF9: COEFFDAT Mask           */

#define BIQ_COEFF10_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF10: COEFFDAT Position      */
#define BIQ_COEFF10_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF10_COEFFDAT_Pos)        /*!< BIQ_T::COEFF10: COEFFDAT Mask          */

#define BIQ_COEFF11_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF11: COEFFDAT Position      */
#define BIQ_COEFF11_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF11_COEFFDAT_Pos)        /*!< BIQ_T::COEFF11: COEFFDAT Mask          */

#define BIQ_COEFF12_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF12: COEFFDAT Position      */
#define BIQ_COEFF12_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF12_COEFFDAT_Pos)        /*!< BIQ_T::COEFF12: COEFFDAT Mask          */

#define BIQ_COEFF13_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF13: COEFFDAT Position      */
#define BIQ_COEFF13_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF13_COEFFDAT_Pos)        /*!< BIQ_T::COEFF13: COEFFDAT Mask          */

#define BIQ_COEFF14_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF14: COEFFDAT Position      */
#define BIQ_COEFF14_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF14_COEFFDAT_Pos)        /*!< BIQ_T::COEFF14: COEFFDAT Mask          */

#define BIQ_COEFF15_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF15: COEFFDAT Position      */
#define BIQ_COEFF15_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF15_COEFFDAT_Pos)        /*!< BIQ_T::COEFF15: COEFFDAT Mask          */

#define BIQ_COEFF16_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF16: COEFFDAT Position      */
#define BIQ_COEFF16_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF16_COEFFDAT_Pos)        /*!< BIQ_T::COEFF16: COEFFDAT Mask          */

#define BIQ_COEFF17_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF17: COEFFDAT Position      */
#define BIQ_COEFF17_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF17_COEFFDAT_Pos)        /*!< BIQ_T::COEFF17: COEFFDAT Mask          */

#define BIQ_COEFF18_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF18: COEFFDAT Position      */
#define BIQ_COEFF18_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF18_COEFFDAT_Pos)        /*!< BIQ_T::COEFF18: COEFFDAT Mask          */

#define BIQ_COEFF19_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF19: COEFFDAT Position      */
#define BIQ_COEFF19_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF19_COEFFDAT_Pos)        /*!< BIQ_T::COEFF19: COEFFDAT Mask          */

#define BIQ_COEFF20_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF20: COEFFDAT Position      */
#define BIQ_COEFF20_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF20_COEFFDAT_Pos)        /*!< BIQ_T::COEFF20: COEFFDAT Mask          */

#define BIQ_COEFF21_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF21: COEFFDAT Position      */
#define BIQ_COEFF21_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF21_COEFFDAT_Pos)        /*!< BIQ_T::COEFF21: COEFFDAT Mask          */

#define BIQ_COEFF22_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF22: COEFFDAT Position      */
#define BIQ_COEFF22_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF22_COEFFDAT_Pos)        /*!< BIQ_T::COEFF22: COEFFDAT Mask          */

#define BIQ_COEFF23_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF23: COEFFDAT Position      */
#define BIQ_COEFF23_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF23_COEFFDAT_Pos)        /*!< BIQ_T::COEFF23: COEFFDAT Mask          */

#define BIQ_COEFF24_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF24: COEFFDAT Position      */
#define BIQ_COEFF24_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF24_COEFFDAT_Pos)        /*!< BIQ_T::COEFF24: COEFFDAT Mask          */

#define BIQ_COEFF25_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF25: COEFFDAT Position      */
#define BIQ_COEFF25_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF25_COEFFDAT_Pos)        /*!< BIQ_T::COEFF25: COEFFDAT Mask          */

#define BIQ_COEFF26_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF26: COEFFDAT Position      */
#define BIQ_COEFF26_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF26_COEFFDAT_Pos)        /*!< BIQ_T::COEFF26: COEFFDAT Mask          */

#define BIQ_COEFF27_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF27: COEFFDAT Position      */
#define BIQ_COEFF27_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF27_COEFFDAT_Pos)        /*!< BIQ_T::COEFF27: COEFFDAT Mask          */

#define BIQ_COEFF28_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF28: COEFFDAT Position      */
#define BIQ_COEFF28_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF28_COEFFDAT_Pos)        /*!< BIQ_T::COEFF28: COEFFDAT Mask          */

#define BIQ_COEFF29_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF29: COEFFDAT Position      */
#define BIQ_COEFF29_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF29_COEFFDAT_Pos)        /*!< BIQ_T::COEFF29: COEFFDAT Mask          */

#define BIQ_CTL_BIQEN_Pos                (0)                                               /*!< BIQ_T::CTL: BIQEN Position             */
#define BIQ_CTL_BIQEN_Msk                (0x1ul << BIQ_CTL_BIQEN_Pos)                      /*!< BIQ_T::CTL: BIQEN Mask                 */

#define BIQ_CTL_HPFON_Pos                (1)                                               /*!< BIQ_T::CTL: HPFON Position             */
#define BIQ_CTL_HPFON_Msk                (0x1ul << BIQ_CTL_HPFON_Pos)                      /*!< BIQ_T::CTL: HPFON Mask                 */

#define BIQ_CTL_PATHSEL_Pos              (2)                                               /*!< BIQ_T::CTL: PATHSEL Position           */
#define BIQ_CTL_PATHSEL_Msk              (0x1ul << BIQ_CTL_PATHSEL_Pos)                    /*!< BIQ_T::CTL: PATHSEL Mask               */

#define BIQ_CTL_DLCOEFF_Pos              (3)                                               /*!< BIQ_T::CTL: DLCOEFF Position           */
#define BIQ_CTL_DLCOEFF_Msk              (0x1ul << BIQ_CTL_DLCOEFF_Pos)                    /*!< BIQ_T::CTL: DLCOEFF Mask               */

#define BIQ_CTL_SDADCWNSR_Pos            (4)                                               /*!< BIQ_T::CTL: SDADCWNSR Position         */
#define BIQ_CTL_SDADCWNSR_Msk            (0x7ul << BIQ_CTL_SDADCWNSR_Pos)                  /*!< BIQ_T::CTL: SDADCWNSR Mask             */

#define BIQ_CTL_PRGCOEFF_Pos             (7)                                               /*!< BIQ_T::CTL: PRGCOEFF Position          */
#define BIQ_CTL_PRGCOEFF_Msk             (0x1ul << BIQ_CTL_PRGCOEFF_Pos)                   /*!< BIQ_T::CTL: PRGCOEFF Mask              */

#define BIQ_CTL_STAGE_Pos                (11)                                              /*!< BIQ_T::CTL: STAGE Position             */
#define BIQ_CTL_STAGE_Msk                (0x1ul << BIQ_CTL_STAGE_Pos)                      /*!< BIQ_T::CTL: STAGE Mask                 */

#define BIQ_STS_BIST1EN_Pos              (0)                                               /*!< BIQ_T::STS: BIST1EN Position           */
#define BIQ_STS_BIST1EN_Msk              (0x1ul << BIQ_STS_BIST1EN_Pos)                    /*!< BIQ_T::STS: BIST1EN Mask               */

#define BIQ_STS_BIST1F_Pos               (1)                                               /*!< BIQ_T::STS: BIST1F Position            */
#define BIQ_STS_BIST1F_Msk               (0x1ul << BIQ_STS_BIST1F_Pos)                     /*!< BIQ_T::STS: BIST1F Mask                */

#define BIQ_STS_BIST1D_Pos               (2)                                               /*!< BIQ_T::STS: BIST1D Position            */
#define BIQ_STS_BIST1D_Msk               (0x1ul << BIQ_STS_BIST1D_Pos)                     /*!< BIQ_T::STS: BIST1D Mask                */

#define BIQ_STS_BIST2EN_Pos              (8)                                               /*!< BIQ_T::STS: BIST2EN Position           */
#define BIQ_STS_BIST2EN_Msk              (0x1ul << BIQ_STS_BIST2EN_Pos)                    /*!< BIQ_T::STS: BIST2EN Mask               */

#define BIQ_STS_BIST2F_Pos               (9)                                               /*!< BIQ_T::STS: BIST2F Position            */
#define BIQ_STS_BIST2F_Msk               (0x1ul << BIQ_STS_BIST2F_Pos)                     /*!< BIQ_T::STS: BIST2F Mask                */

#define BIQ_STS_BIST2D_Pos               (10)                                              /*!< BIQ_T::STS: BIST2D Position            */
#define BIQ_STS_BIST2D_Msk               (0x1ul << BIQ_STS_BIST2D_Pos)                     /*!< BIQ_T::STS: BIST2D Mask                */

#define BIQ_STS_RAMINITF_Pos             (31)                                              /*!< BIQ_T::STS: RAMINITF Position          */
#define BIQ_STS_RAMINITF_Msk             (0x1ul << BIQ_STS_RAMINITF_Pos)                   /*!< BIQ_T::STS: RAMINITF Mask              */

/**@}*/ /* BIQ_CONST */
/**@}*/ /* end of BIQ register group */


/*---------------------- System Clock Controller -------------------------*/
/**
    @addtogroup CLK System Clock Controller(CLK)
    Memory Mapped Structure for CLK Controller
@{ */
 
typedef struct
{


/**
 * @var CLK_T::PWRCTL
 * Offset: 0x00  System Power Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FWKEN     |STOP/DeepSleep mode fast wakeup enable control
 * |        |          |0 = normal Normal wake up
 * |        |          |1 = fast Fast wake up (default)
 * |        |          |Note: Normal wake up will count 2 LIRC first, and then switch to original HCLK source
 * |        |          |Fast wakeup will direct switch to original HCLK source without any LIRC counting.
 * |[1]     |HXTEN     |External high speed 12MHz Crystal Oscillator Control
 * |        |          |After reset, this bit is u201C0u201D.
 * |        |          |0 = External high speed 12MHz crystal oscillation Disabled.
 * |        |          |1 = External high speed 12MHz crystal oscillation Enabled.
 * |[2]     |HIRCEN    |Internal 49.152 MHzhigh speed RC Oscillator Control
 * |        |          |After reset, this bit is u201C1u201D.
 * |        |          |0 = 49.152 MHzInternal high speed oscillation Disabled.
 * |        |          |1 = Internal high speed49.152 MHz oscillation Enabled.
 * |[5:3]   |HXTGAIN   |HXT Gain Control Bit
 * |        |          |This is a protected register. Please refer to open lock sequence to program it.
 * |        |          |Gain control is used to enlarge the gain of crystal to make sure crystal work normally
 * |        |          |If gain control is enabled, crystal will consume more power than gain control off.
 * |        |          |000 = HXT frequency is from 1 MHz to4 MHz.
 * |        |          |001 = HXT frequency is lower than from 4 MHz to 8 MHz.
 * |        |          |010 = HXT frequency is from 8 MHz to 12 MHz.
 * |        |          |10 011 = HXT frequency is from 12 MHz to 16 MHz.
 * |        |          |11 100= HXT frequency is higher than 16 MHz.
 * |        |          |Others = Reserved(Optional from 12 M Hz to 24 MHz)
 * |[6]     |HXTTBEN   |HXT Crystal TURBO Mode (Write Protect)
 * |        |          |This is a protected register. Please refer to open lock sequence to program it.
 * |        |          |0 = HXT Crystal TURBO mode disabled.
 * |        |          |1 = HXT Crystal TURBO mode enabled.
 * |[7]     |IOFWK     |All IO pin is enabled fast wakeup in STOP/DeepSleep mode.
 * |        |          |When this bit set 0u2019b, trigger IO will delay 3 LIRC and then trigger wakeup from STOP/DeepSleep mode
 * |        |          |When this bit set 1u2019b, trigger IO will wakeup from STOP/DeepSleep mode immediately.
 * |        |          |0= slow Slow wakeup
 * |        |          |1= fast Fast wakeup
 * |[8]     |CLKRDDLY  |Enable the Wake-upClock Ready Delay Counter
 * |        |          |When the chip wakes up from DeepSleep and STOP modeWhen HXT enable, the clock control will delay certain clock cycles to wait system clock stable.
 * |        |          |The delayed clock cycle is 4096 clock cycles when chip works at external high speed crystal oscillator (HXT) enable, and 128 clock cycles when chip works at internal high speed RC oscillator (HIRC).
 * |        |          |0 = Clock cycles delay Disabled.
 * |        |          |1 = Clock cycles delay Enabled.
 * |[9]     |STOPEN    |STOP mode bit. Set to u20181u2019 and issue WFI/WFE instruction to enter STOP mode.
 * |[10]    |DPDEN     |Deep Power Down (DPD) bit. Set to u20181u2019 and issue WFI/WFE instruction to enter DPD mode.
 * |[12]    |LIRCEN    |Internal 10kHz Oscillator Control
 * |        |          |After reset, this bit is u201C0u201D.
 * |        |          |0 = Internal 10 KHz oscillator Disabled.
 * |        |          |1 = Internal 10 KHz oscillator Enabled.
 * |[15:13] |VSET      |Adjusts the digital supply voltage. Should be left as default.
 * |[16]    |WKPINEN   |Determines whether WAKEUP pin(PA15) is enabled in DPD mode.
 * |        |          |0=enabledEnabled
 * |        |          |1=disabledDisabled.
 * |[17]    |WK10KEN   |Determines whether OSC10K is enabled in DPD mode.
 * |        |          |0= eEnabled, in DPD
 * |        |          |1= disabled Disabled in DPD.
 * |        |          |Note: If OSC10K WK10KEN is disabled, device cannot wake from DPD with SELWKTMR delay.
 * |[19:18] |FLASHEN   |Determine whether FLASH memory enters deep power down.
 * |        |          |FLASHEN [0]: ]= 1: flash enters deep power down upon DEEP_SLEEP
 * |        |          |FLASHEN [1]: ]= 1: flash enters deep power down upon STOP mode.
 * |        |          |If FLASHEN is selected for a power state mode, current consumption is reduced, but a 10us wakeup time must be added to the wakeup sequence
 * |        |          |Trade-off is wakeup time for standby power.
 * |[22:20] |SELWKTMR  |Select WAKEUP Timer:
 * |        |          |000 = Time-out interval is 128 LIRC clocks (About 12.8 ms).
 * |        |          |001 = Time-out interval is 256 LIRC clocks (About 25.6 ms).
 * |        |          |010 = Time-out interval is 512 LIRC clocks (About 51.2 ms).
 * |        |          |011 = Time-out interval is 1024 LIRC clocks (About 102.4ms).
 * |        |          |100 = Time-out interval is 4096 LIRC clocks (About 409.6ms).
 * |        |          |101 = Time-out interval is 8192 LIRC clocks (About 819.2ms).
 * |        |          |110 = Time-out interval is 16384 LIRC clocks (About 1638.4ms).
 * |        |          |111 = Time-out interval is 65536 LIRC clocks (About 6553.6ms).
 * |[24]    |WKPINWKF  |Read Only. This flag indicates that wakeup of device was requested with a high to low transition of the WAKEUP pin. Flag is cleared when DPD mode is entered or any of the DPD bits of RSTSRC register (RSTSRC[10:8]) are cleared.
 * |[25]    |TMRWKF    |Read Only. This flag indicates that wakeup of device was requested with TIMER count of the 10Khz oscillator. Flag is cleared when DPD mode is entered or any of the DPD bits of RSTSRC register (RSTSRC [10:8]) are cleared.
 * |[27]    |WKPUEN    |Wakeup Pin Pull-up Control
 * |        |          |This signal is latched in deep power down and preserved.
 * |        |          |0 = pull-up enable.
 * |        |          |1 = tri-state (default).
 * |[31:28] |WKTMRSTS  |Read-Only. Read back of the current WAKEUP timer setting. This value is updated with SELWKTMR upon entering DPD mode. 
 * @var CLK_T::AHBCLK
 * Offset: 0x04  AHB Device Clock Enable Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDMACKEN  |PDMA Clock Enable Control
 * |        |          |0 = PDMA engine clock Disabled.
 * |        |          |1 = PDMA engine clock Enabled.
 * |[1]     |CPDCKEN   |Companding Clock Enable Control
 * |        |          |0 = Companding engine clock Disabled.
 * |        |          |1 = Companding engine clock Enabled.
 * |[2]     |ISPCKEN   |Flash ISP Controller Clock Enable Control.
 * |        |          |The Flash ISP engine clock always is from 49 MHz RC oscillator.
 * |        |          |0 = Flash ISP engine clock Disabled.
 * |        |          |1 = Flash ISP engine clock Enabled.
 * @var CLK_T::APBCLK
 * Offset: 0x08  APB Device Clock Enable Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WDTEN     |Watchdog Clock Enable Control
 * |        |          |This bit is the protected bit
 * |        |          |To program this bit needs an open lock sequence, write u201C59hu201D, u201C16hu201D, u201C88hu201D to register SYS_REGLCTL to un-lock this bit
 * |        |          |Refer to the register SYS_REGLCTL at address SYS_BA+0x100.
 * |        |          |0 = WDT clock Disabled.
 * |        |          |1 = WDT clock Enabled.
 * |[2]     |TMR0EN    |Timer0 Clock Enable Control
 * |        |          |0 = Timer0 clock Disabled.
 * |        |          |1 = Timer0 clock Enabled.
 * |[3]     |TMR1EN    |Timer1 Clock Enable Control
 * |        |          |0 = Timer1 clock Disabled.
 * |        |          |1 = Timer1 clock Enabled.
 * |[4]     |TMR2EN    |Timer2 Clock Enable Control
 * |        |          |0 = Timer2 clock Disabled.
 * |        |          |1 = Timer2 clock Enabled.
 * |[8]     |I2C0EN    |I2C0 Clock Enable Control
 * |        |          |0 = I2C0 clock Disabled.
 * |        |          |1 = I2C0 clock Enabled.
 * |[9]     |I2C1EN    |I2C1 Clock Enable Control
 * |        |          |0 = I2C1 clock Disabled.
 * |        |          |1 = I2C1 clock Enabled.
 * |[11]    |SPI1EN    |SPI1 Clock Enable Control
 * |        |          |0 = SPI1 clock Disabled.
 * |        |          |1 = SPI1 clock Enabled.Reserved
 * |[12]    |SPI0EN    |SPI0 Clock Enable Control
 * |        |          |0 = SPI0 clock Disabled.
 * |        |          |1 = SPI0 clock Enabled.
 * |[13]    |I2S0EN    |I2S0 Clock Enable Control
 * |        |          |0 = I2S0 clock Disabled.
 * |        |          |1 = I2S0 clock Enabled.
 * |[16]    |UART0EN   |UART0 Block Clock Enable Control
 * |        |          |0 = UART0 clock Disabled.
 * |        |          |1 = UART0 clock Enabled.
 * |[17]    |UART1EN   |UART1 Block Clock Enable Control
 * |        |          |0 = UART1 clock Disabled.
 * |        |          |1 = UART1 clock Enabled.
 * |[18]    |BIQEN     |Biquad Filter(BIQ) Block Clock Enable Control
 * |        |          |0 = BIQ clock Disabled.
 * |        |          |1 = BIQ clock Enabled.
 * |[20]    |PWM0EN    |PWM0 Block Clock Enable Control
 * |        |          |0 = PWM0 clock Disabled.
 * |        |          |1 = PWM0 clock Enabled.
 * |[21]    |PWM1EN    |PWM1 Block Clock Enable Control
 * |        |          |0 = PWM1 clock Disabled.
 * |        |          |1 = PWM1 clock Enabled.
 * |[24]    |USBEN     |USB Clock Enable Control
 * |        |          |0 = USB clock Disabled.
 * |        |          |1 = USB clock Enabled.
 * |[28]    |SARADCEN  |Analog-Digital-Converter (SARADC) Clock Enable Control
 * |        |          |0 = SARADC clock Disabled.
 * |        |          |1 = SARADC clock Enabled.
 * |[29]    |DACEN     |DAC Clock Enable Control
 * |        |          |0 = DAC clock Disabled.
 * |        |          |1 = DAC clock Enabled.
 * |[30]    |SDADCEN   |SDADC Clock Enable Control
 * |        |          |0 = SDADC clock Disabled
 * |        |          |1 = SDADC clock Enabled
 * |[31]    |ANAEN     |Analog Block Clock Enable Control
 * |        |          |0 = Analog block clock Disabled
 * |        |          |1 = Analog block clock Enabled
 * @var CLK_T::DPDFLR
 * Offset: 0x0C  DPD State Register and Flash Regulator Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |PD_STATE  |An 8bit register that is preserved when DPD (Deep Power Down) state is entered and after wakeup is available by reading PD_STATE_RB.
 * |[15:8]  |PD_STATE_RB|Current values of PD_STATE register.
 * @var CLK_T::CLKSEL0
 * Offset: 0x10  Clock Source Select Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |HCLKSEL   |HCLK Clock Source Select
 * |        |          |00 = clock source from HXT
 * |        |          |01 = clock source from PLLFOUT
 * |        |          |10 = clock source from LIRC
 * |        |          |11 = clock source from HIRC
 * |        |          |Note:
 * |        |          |1. When power on, HIRC is selected as HCLK clock source.
 * |        |          |2. Before clock switch, the related clock sources (pre-select and new-select) must be turned on.
 * |[4:3]   |STICKSEL  |SYS_TICK Clock Source Select
 * |        |          |00 = clock source from HXT
 * |        |          |01 = clock source from HXT/2
 * |        |          |10 = clock source from HCLK/2
 * |        |          |11 = clock source from HIRC/2
 * |        |          |Note:
 * |        |          |1. When power on, HIRC is selected as HCLK clock source.
 * |        |          |2. Before clock switch, the related clock sources (pre-select and new-select) must be turned on.
 * |        |          |3. SysTick clock source must less than or equal to HCLK/2.
 * |[7:6]   |OSCFSEL   |HIRC Frequency Selection register
 * |        |          |These bits are protected, to write to bits first perform the unlock sequence (see Register Lock Control Register (SYS_REGLCTL))
 * |        |          |00 = Trim for 49.152MHz@VCC=3.3V selected.
 * |        |          |01 = Trim for 49.152MHz@VCC=1.8V selected.
 * |        |          |1x= Trim for 48MHz@VCC=3.3V selected.
 * |[18:16] |FCLK_MUX_STATE|These register state shows the current HCLK is from which source clock
 * |        |          |000 = clock source from HXT
 * |        |          |001 = clock source from PLLFOUT
 * |        |          |010 = clock source from LIRC
 * |        |          |011 = clock source from HIRC
 * |        |          |Others reserved.
 * @var CLK_T::CLKSEL1
 * Offset: 0x14  Clock Source Select Control Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WDTSEL    |Watchdog Timer Clock Source Selection (Write Protect)
 * |        |          |These bits are protected bits
 * |        |          |To program these bits needs an open lock sequence, write u201C59hu201D, u201C16hu201D, u201C88hu201D to SYS_REGLCTL to un-lock these bits
 * |        |          |Refer to the register SYS_REGLCTL at address SYS_BA+0x100..
 * |        |          |0= Clock source from LIRC.
 * |        |          |1= Clock source from HCLK/2048. 
 * |[3:2]   |SARADCSEL |SARADC Clock Source Select
 * |        |          |00 = Clock source from HXT.
 * |        |          |01 = Clock source from PLLFOUT.
 * |        |          |10 = Clock source from PCLK.
 * |        |          |11 = Clock source from HIRC.
 * |[5:4]   |SPI0SEL   |SPI0 Clock Source Select
 * |        |          |00 = Clock source from HXT.
 * |        |          |01 = Clock source from PLLFOUT.
 * |        |          |10 = Clock source from PCLK.
 * |        |          |11 = Clock source from HIRC.
 * |        |          |Note: SPI0 engine clock must be same clock source as PCLK
 * |[7:6]   |SPI1SEL   |SPI1 Clock Source Select
 * |        |          |00 = Clock source from HXT.
 * |        |          |01 = Clock source from PLLFOUT.
 * |        |          |10 = Clock source from PCLK.
 * |        |          |11 = Clock source from HIRC.
 * |        |          |Note: SPI1 engine clock must be same clock source as PCLK Reserved
 * |[10:8]  |TMR0SEL   |Timer0 Clock Source Select
 * |        |          |000 = Clock source from HXT.
 * |        |          |001 = Clock source from PCLK.
 * |        |          |010 = Clock source from External Trigger.
 * |        |          |011 = Clock source from LIRC.
 * |        |          |100 = Clock source from HIRC.
 * |        |          |Others = Equivalent with u201C100u201D.
 * |[14:12] |TMR1SEL   |Timer1 Clock Source Select
 * |        |          |000 = Clock source from HXT.
 * |        |          |001 = Clock source from PCLK.
 * |        |          |010 = Clock source from External Trigger.
 * |        |          |011 = Clock source from LIRC.
 * |        |          |100 = Clock source from HIRC.
 * |        |          |Others = Equivalent with u201C100u201D.
 * |[18:16] |TMR2SEL   |Timer2 Clock Source Select
 * |        |          |000 = Clock source from HXT.
 * |        |          |001 = Clock source from PCLK.
 * |        |          |010 = Clock source from External Trigger.
 * |        |          |011 = Clock source from LIRC.
 * |        |          |100 = Clock source from HIRC.
 * |        |          |Others = Equivalent with u201C100u201D.
 * |[22:20] |I2S0SEL   |I2S0 Clock Source Select
 * |        |          |000 = Clock source from HXT.
 * |        |          |001 = Clock source from PLLFOUT.
 * |        |          |010 = Clock source from PCLK.
 * |        |          |011 = Clock source from HIRC.
 * |        |          |100 = Clock source from MCLKI.
 * |        |          |101 = Clock source from XCLK.
 * |        |          |Others = Equivalent with u201C011u201D.
 * |[25:24] |UART0SEL  |UART0 Clock Source Select
 * |        |          |00 = Clock source from HXT.
 * |        |          |01 = Clock source from PLLFOUT.
 * |        |          |10 = Clock source from HIRC.
 * |        |          |11 = Clock source from HIRC.
 * |[27:26] |UART1SEL  |UART1 Clock Source Select
 * |        |          |00 = Clock source from HXT.
 * |        |          |01 = Clock source from PLLFOUT.
 * |        |          |10 = Clock source from HIRC.
 * |        |          |11 = Clock source from HIRC.
 * |[29:28] |PWM0SEL   |PWM Timer Clock Source Select
 * |        |          |00 = Clock source from HXT.
 * |        |          |01 = Clock source from PLLFOUT.
 * |        |          |10 = Clock source from PCLK.
 * |        |          |11 = Clock source from HIRC.
 * |[31:30] |PWM1SEL   |PWM Timer Clock Source Select
 * |        |          |00 = Clock source from HXT.
 * |        |          |01 = Clock source from PLLFOUT.
 * |        |          |10 = Clock source from PCLK.
 * |        |          |11 = Clock source from HIRC.
 * @var CLK_T::CLKDIV0
 * Offset: 0x18  Clock Divider Number Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |HCLKDIV   |HCLK Clock Divide Number From HCLK Clock Source
 * |        |          |The HCLK clock frequency = (HCLK clock source frequency) / (HCLKDIV + 1).
 * |[7:4]   |UART0DIV  |UART0 Clock Divide Number From UART0 Clock Source
 * |        |          |The UART0 clock frequency = (UART0 clock source frequency) / (UART0DIV + 1).
 * |        |          |Note: UART0 engine clock must smaller or equal to PCLK.
 * |[11:8]  |UART1DIV  |UART1 Clock Divide Number From UART1 Clock Source
 * |        |          |The UART1 clock frequency = (UART1 clock source frequency) / (UART1DIV + 1).
 * |        |          |Note: UART1 engine clock must smaller or equal to PCLK.
 * |[15:12] |USBDIV    |USB Clock Divide Number From PLL Clock
 * |        |          |USB clock frequency = (PLL frequency) / (USBDIV + 1).
 * |[22:16] |SARADCDIV |SARADC Clock Divide Number From ADC Clock Source
 * |        |          |The SARADC clock frequency SARADCLK = (SARADC clock source frequency) / (SARADCDIV + 1).
 * |        |          |The ADC engine clock must meet the constraint: ADCLK  HCLK/2.
 * @var CLK_T::CLKDIV1
 * Offset: 0x1C  Clock Divider Number Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DACDIV    |DAC Clock Divide Number From DAC Clock Source
 * |        |          |The DAC clock frequency = (DAC clock source frequency) / (DACDIV + 1).
 * |[11:8]  |BIQDIV    |BIQ Clock Divide Number From HCLK
 * |        |          |The BIQ clock frequency = HCLK / (BIQDIV + 1).
 * |        |          |Note: BIQ clock frequency must keep PCLK/2
 * |[23:16] |SDADCDIV  |SDADC Clock Divide Number From SDADC Clock Source
 * |        |          |The SDADC clock frequency = (SDADC clock source frequency) / (SDADCDIV + 1).
 * @var CLK_T::STATUS
 * Offset: 0x20  Clock Status Monitor Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |HXTSTB    |HXT Clock Source Stable Flag (Read Only)
 * |        |          |0 = External high speed crystal oscillator (HXT) clock is not stable or disabled.
 * |        |          |1 = External high speed crystal oscillator (HXT) clock is stabled and enabled.
 * |[1]     |LIRCSTB   |LIRC Clock Source Stable Flag (Read Only)
 * |        |          |0 = Internal low speed RC oscillator (LIRC) clock is not stable or disabled.
 * |        |          |1 = Internal low speed RC oscillator (LIRC) clock is stable and enabled.
 * |[2]     |HIRCSTB   |HIRC clock source stable flag(Read only)
 * |        |          |0 = Internal high speed RC oscillator (HIRC) clock is not stable or disabled.
 * |        |          |1 = Internal high speed RC oscillator (HIRC) clock is stable and enabled.
 * |[3]     |PLLSTB    |Internal PLL Clock Source Stable Flag (Read Only)
 * |        |          |0 = Internal PLL clock is not stable or disabled.
 * |        |          |1 = Internal PLL clock is stable and enabled.
 * |[4]     |XCLKSTB   |XCLK Clock Source Stable Flag (Read Only)
 * |        |          |0 = Clock doubler (XCLK) clock is not stable or disabled.
 * |        |          |1 = Clock doubler (XCLK) clock is stable and enabled.
 * @var CLK_T::PFLAG
 * Offset: 0x24  Power Down Flag Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |DSF       |Deep Sleep Flag
 * |        |          |This flag is set if core logic was placed in Deep Sleep mode. Write u20181u2019 to clear flag.
 * |[1]     |STOPF     |Stop Flag
 * |        |          |This flag is set if core logic was stopped but not powered down. Write u20181u2019 to clear flag.
 * @var CLK_T::CLKSEL2
 * Offset: 0x28  Clock Source Select Control Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |USBSEL    |USB Clock Source Select
 * |        |          |0 = Clock source from HIRC.
 * |        |          |1 = Clock source from PLLFOUT.
 * |[4]     |XCLKSEL   |Clock Doubler Source Selection
 * |        |          |0 = Clock source from MCLK input (MCLKI).
 * |        |          |1 = Clock source from BCLK of I2S0 (I2S0_BCLK)
 * |[10:8]  |DACSEL    |DAC Clock Source Select
 * |        |          |000 = Clock source from HXT.
 * |        |          |001 = Clock source from PLLFOUT.
 * |        |          |010 = Clock source from PCLK.
 * |        |          |011 = Clock source from HIRC.
 * |        |          |100 = Clock source from MCLKI.
 * |        |          |101 = Clock source from XCLK.
 * |        |          |Others = Equivalent with u201C000u201D.
 * |[14:12] |SDADCSEL  |SDADC Clock Source Select
 * |        |          |000 = Clock source from HXT.
 * |        |          |001 = Clock source from PLLFOUT.
 * |        |          |010 = Clock source from PCLK.
 * |        |          |011 = Clock source from HIRC.
 * |        |          |100 = Clock source from MCLKI.
 * |        |          |101 = Clock source from XCLK.
 * |        |          |Others = Equivalent with u201C000u201D.
 * @var CLK_T::XCLKCTL
 * Offset: 0x2C  Clock Doubler Output Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |XCLKMUL   |Clock doubler Output Frequency Multiplication
 * |        |          |00 = Output frequency multiply by 1 (Bypass)
 * |        |          |01 = Output frequency multiply by 2
 * |        |          |10 = Output frequency multiply by 4
 * |        |          |11 = Output frequency multiply by 8
 * |[4]     |XCLKEN    |XCLK Enable Bit
 * |        |          |0 = Clock doubler (XCLK) Disabled.
 * |        |          |1 = Clock doubler (XCLK) Enabled.
 * |[5]     |RELOCK    |XCLK relock enable setting
 * |        |          |When write this bit to 1u2019b, the XCLK will execute relock action
 * |        |          |And this bit will auto clear to 0u2019b after relock finish & XCLK output clock stable.
 * |[12]    |XCLKFDEN  |XCLK Clock Fail Detector Enable Bit
 * |        |          |0 = Clock doubler clock (XCLK) fail detector Disabled.
 * |        |          |1 = Clock doubler clock (XCLK) fail detector Enabled.
 * |[13]    |XCLKFIEN  |XCLK Clock Fail Interrupt Enable Bit
 * |        |          |0 = Clock doubler clock (XCLK) fail interrupt Disabled.
 * |        |          |1 = Clock doubler clock (XCLK) fail interrupt Enabled.
 * |[14]    |XCLKFIF   |XCLK Clock Fail Interrupt Flag
 * |        |          |0 = Clock doubler clock (XCLK) clock is normal.
 * |        |          |1 = Clock doubler clock (XCLK) stops.
 * |        |          |Note: Write 1 to clear the bit to 0.
 * @var CLK_T::PLLCTL
 * Offset: 0x30  PLL Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:0]   |FBDIV     |PLL Feedback Divider Control (Write Protected)
 * |        |          |Refer to the formulas below the table.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[12:9]  |INDIV     |PLL Input Divider Control (Write Protected)
 * |        |          |Refer to the formulas below the table.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[15:14] |OUTDIV    |PLL Output Divider Control (Write Protected)
 * |        |          |Refer to the formulas below the table.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[16]    |PD        |Power-down Mode (Write Protected)
 * |        |          |If set the PDEN bit to 1 in CLK_PWRCTL register, the PLL will enter Power-down mode, too.
 * |        |          |0 = PLL is in normal mode.
 * |        |          |1 = PLL is in Power-down mode (default).
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[17]    |BP        |PLL Bypass Control (Write Protected)
 * |        |          |0 = PLL is in normal mode (default).
 * |        |          |1 = PLL clock output is same as PLL input clock FIN.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[18]    |OE        |PLL OE (FOUT Enable) Pin Control (Write Protected)
 * |        |          |0 = PLL FOUT Enabled.
 * |        |          |1 = PLL FOUT is fixed low.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[20:19] |PLLSRC    |PLL Source Clock Selection (Write Protected)
 * |        |          |00 = PLL source clock from external high-speed crystal oscillator (HXT).
 * |        |          |01 = PLL source clock from clock doubler output (XCLK).
 * |        |          |10 = Reserved. Do not use.
 * |        |          |11 = PLL source clock from internal high-speed oscillator (HIRC).
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[22]    |FTREN     |Fliter Enable Control
 * |        |          |0 = Disable Filter
 * |        |          |1 = Enable Filter
 * |[23]    |STBSEL    |PLL Stable Counter Selection (Write Protected)
 * |        |          |0 = PLL stable time is 1293 PLL source clock (suitable for source clock is equal to or less than 12 MHz).
 * |        |          |1 = PLL stable time is 12288 5044 PLL source clock (suitable for source clock is larger than 12 MHz).
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 */
    __IO uint32_t PWRCTL;                /*!< [0x0000] System Power Control Register                                    */
    __IO uint32_t AHBCLK;                /*!< [0x0004] AHB Device Clock Enable Control Register                         */
    __IO uint32_t APBCLK;                /*!< [0x0008] APB Device Clock Enable Control Register                         */
    __IO uint32_t DPDFLR;                /*!< [0x000c] DPD State Register and Flash Regulator Control                   */
    __IO uint32_t CLKSEL0;               /*!< [0x0010] Clock Source Select Control Register 0                           */
    __IO uint32_t CLKSEL1;               /*!< [0x0014] Clock Source Select Control Register 1                           */
    __IO uint32_t CLKDIV0;               /*!< [0x0018] Clock Divider Number Register 0                                  */
    __IO uint32_t CLKDIV1;               /*!< [0x001c] Clock Divider Number Register 1                                  */
    __I  uint32_t STATUS;                /*!< [0x0020] Clock Status Monitor Register                                    */
    __IO uint32_t PFLAG;                 /*!< [0x0024] Power Down Flag Register                                         */
    __IO uint32_t CLKSEL2;               /*!< [0x0028] Clock Source Select Control Register 2                           */
    __IO uint32_t XCLKCTL;               /*!< [0x002c] Clock Doubler Output Control Register                            */
    __IO uint32_t PLLCTL;                /*!< [0x0030] PLL Control Register                                             */
    __I  uint32_t RESERVE0[48];
	__IO uint32_t ILDOCTL;               /*!< [0x00F4] Internal LDO Control Register                                        */
} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

#define CLK_PWRCTL_FWKEN_Pos             (0)                                               /*!< CLK_T::PWRCTL: FWKEN Position          */
#define CLK_PWRCTL_FWKEN_Msk             (0x1ul << CLK_PWRCTL_FWKEN_Pos)                   /*!< CLK_T::PWRCTL: FWKEN Mask              */

#define CLK_PWRCTL_HXTEN_Pos             (1)                                               /*!< CLK_T::PWRCTL: HXTEN Position          */
#define CLK_PWRCTL_HXTEN_Msk             (0x1ul << CLK_PWRCTL_HXTEN_Pos)                   /*!< CLK_T::PWRCTL: HXTEN Mask              */

#define CLK_PWRCTL_HIRCEN_Pos            (2)                                               /*!< CLK_T::PWRCTL: HIRCEN Position         */
#define CLK_PWRCTL_HIRCEN_Msk            (0x1ul << CLK_PWRCTL_HIRCEN_Pos)                  /*!< CLK_T::PWRCTL: HIRCEN Mask             */

#define CLK_PWRCTL_HXTGAIN_Pos           (3)                                               /*!< CLK_T::PWRCTL: HXTGAIN Position        */
#define CLK_PWRCTL_HXTGAIN_Msk           (0x7ul << CLK_PWRCTL_HXTGAIN_Pos)                 /*!< CLK_T::PWRCTL: HXTGAIN Mask            */

#define CLK_PWRCTL_HXTTBEN_Pos           (6)                                               /*!< CLK_T::PWRCTL: HXTTBEN Position        */
#define CLK_PWRCTL_HXTTBEN_Msk           (0x1ul << CLK_PWRCTL_HXTTBEN_Pos)                 /*!< CLK_T::PWRCTL: HXTTBEN Mask            */

#define CLK_PWRCTL_IOFWK_Pos             (7)                                               /*!< CLK_T::PWRCTL: IOFWK Position          */
#define CLK_PWRCTL_IOFWK_Msk             (0x1ul << CLK_PWRCTL_IOFWK_Pos)                   /*!< CLK_T::PWRCTL: IOFWK Mask              */

#define CLK_PWRCTL_CLKRDDLY_Pos          (8)                                               /*!< CLK_T::PWRCTL: CLKRDDLY Position       */
#define CLK_PWRCTL_CLKRDDLY_Msk          (0x1ul << CLK_PWRCTL_CLKRDDLY_Pos)                /*!< CLK_T::PWRCTL: CLKRDDLY Mask           */

#define CLK_PWRCTL_STOPEN_Pos            (9)                                               /*!< CLK_T::PWRCTL: STOPEN Position         */
#define CLK_PWRCTL_STOPEN_Msk            (0x1ul << CLK_PWRCTL_STOPEN_Pos)                  /*!< CLK_T::PWRCTL: STOPEN Mask             */

#define CLK_PWRCTL_DPDEN_Pos             (10)                                              /*!< CLK_T::PWRCTL: DPDEN Position          */
#define CLK_PWRCTL_DPDEN_Msk             (0x1ul << CLK_PWRCTL_DPDEN_Pos)                   /*!< CLK_T::PWRCTL: DPDEN Mask              */

#define CLK_PWRCTL_LIRCEN_Pos            (12)                                              /*!< CLK_T::PWRCTL: LIRCEN Position         */
#define CLK_PWRCTL_LIRCEN_Msk            (0x1ul << CLK_PWRCTL_LIRCEN_Pos)                  /*!< CLK_T::PWRCTL: LIRCEN Mask             */

#define CLK_PWRCTL_VSET_Pos              (13)                                              /*!< CLK_T::PWRCTL: VSET Position           */
#define CLK_PWRCTL_VSET_Msk              (0x7ul << CLK_PWRCTL_VSET_Pos)                    /*!< CLK_T::PWRCTL: VSET Mask               */

#define CLK_PWRCTL_WKPINEN_Pos           (16)                                              /*!< CLK_T::PWRCTL: WKPINEN Position        */
#define CLK_PWRCTL_WKPINEN_Msk           (0x1ul << CLK_PWRCTL_WKPINEN_Pos)                 /*!< CLK_T::PWRCTL: WKPINEN Mask            */

#define CLK_PWRCTL_WK10KEN_Pos           (17)                                              /*!< CLK_T::PWRCTL: WK10KEN Position        */
#define CLK_PWRCTL_WK10KEN_Msk           (0x1ul << CLK_PWRCTL_WK10KEN_Pos)                 /*!< CLK_T::PWRCTL: WK10KEN Mask            */

#define CLK_PWRCTL_FLASHEN_Pos           (18)                                              /*!< CLK_T::PWRCTL: FLASHEN Position        */
#define CLK_PWRCTL_FLASHEN_Msk           (0x3ul << CLK_PWRCTL_FLASHEN_Pos)                 /*!< CLK_T::PWRCTL: FLASHEN Mask            */

#define CLK_PWRCTL_SELWKTMR_Pos          (20)                                              /*!< CLK_T::PWRCTL: SELWKTMR Position       */
#define CLK_PWRCTL_SELWKTMR_Msk          (0x7ul << CLK_PWRCTL_SELWKTMR_Pos)                /*!< CLK_T::PWRCTL: SELWKTMR Mask           */

#define CLK_PWRCTL_WKPINWKF_Pos          (24)                                              /*!< CLK_T::PWRCTL: WKPINWKF Position       */
#define CLK_PWRCTL_WKPINWKF_Msk          (0x1ul << CLK_PWRCTL_WKPINWKF_Pos)                /*!< CLK_T::PWRCTL: WKPINWKF Mask           */

#define CLK_PWRCTL_TMRWKF_Pos            (25)                                              /*!< CLK_T::PWRCTL: TMRWKF Position         */
#define CLK_PWRCTL_TMRWKF_Msk            (0x1ul << CLK_PWRCTL_TMRWKF_Pos)                  /*!< CLK_T::PWRCTL: TMRWKF Mask             */

#define CLK_PWRCTL_WKPUEN_Pos            (27)                                              /*!< CLK_T::PWRCTL: WKPUEN Position         */
#define CLK_PWRCTL_WKPUEN_Msk            (0x1ul << CLK_PWRCTL_WKPUEN_Pos)                  /*!< CLK_T::PWRCTL: WKPUEN Mask             */

#define CLK_PWRCTL_WKTMRSTS_Pos          (28)                                              /*!< CLK_T::PWRCTL: WKTMRSTS Position       */
#define CLK_PWRCTL_WKTMRSTS_Msk          (0xful << CLK_PWRCTL_WKTMRSTS_Pos)                /*!< CLK_T::PWRCTL: WKTMRSTS Mask           */

#define CLK_AHBCLK_PDMACKEN_Pos          (0)                                               /*!< CLK_T::AHBCLK: PDMACKEN Position       */
#define CLK_AHBCLK_PDMACKEN_Msk          (0x1ul << CLK_AHBCLK_PDMACKEN_Pos)                /*!< CLK_T::AHBCLK: PDMACKEN Mask           */

#define CLK_AHBCLK_CPDCKEN_Pos           (1)                                               /*!< CLK_T::AHBCLK: CPDCKEN Position        */
#define CLK_AHBCLK_CPDCKEN_Msk           (0x1ul << CLK_AHBCLK_CPDCKEN_Pos)                 /*!< CLK_T::AHBCLK: CPDCKEN Mask            */

#define CLK_AHBCLK_ISPCKEN_Pos           (2)                                               /*!< CLK_T::AHBCLK: ISPCKEN Position        */
#define CLK_AHBCLK_ISPCKEN_Msk           (0x1ul << CLK_AHBCLK_ISPCKEN_Pos)                 /*!< CLK_T::AHBCLK: ISPCKEN Mask            */

#define CLK_APBCLK_WDTEN_Pos             (0)                                               /*!< CLK_T::APBCLK: WDTEN Position          */
#define CLK_APBCLK_WDTEN_Msk             (0x1ul << CLK_APBCLK_WDTEN_Pos)                   /*!< CLK_T::APBCLK: WDTEN Mask              */

#define CLK_APBCLK_TMR0EN_Pos            (2)                                               /*!< CLK_T::APBCLK: TMR0EN Position         */
#define CLK_APBCLK_TMR0EN_Msk            (0x1ul << CLK_APBCLK_TMR0EN_Pos)                  /*!< CLK_T::APBCLK: TMR0EN Mask             */

#define CLK_APBCLK_TMR1EN_Pos            (3)                                               /*!< CLK_T::APBCLK: TMR1EN Position         */
#define CLK_APBCLK_TMR1EN_Msk            (0x1ul << CLK_APBCLK_TMR1EN_Pos)                  /*!< CLK_T::APBCLK: TMR1EN Mask             */

#define CLK_APBCLK_TMR2EN_Pos            (4)                                               /*!< CLK_T::APBCLK: TMR2EN Position         */
#define CLK_APBCLK_TMR2EN_Msk            (0x1ul << CLK_APBCLK_TMR2EN_Pos)                  /*!< CLK_T::APBCLK: TMR2EN Mask             */

#define CLK_APBCLK_I2C0EN_Pos            (8)                                               /*!< CLK_T::APBCLK: I2C0EN Position         */
#define CLK_APBCLK_I2C0EN_Msk            (0x1ul << CLK_APBCLK_I2C0EN_Pos)                  /*!< CLK_T::APBCLK: I2C0EN Mask             */

#define CLK_APBCLK_I2C1EN_Pos            (9)                                               /*!< CLK_T::APBCLK: I2C1EN Position         */
#define CLK_APBCLK_I2C1EN_Msk            (0x1ul << CLK_APBCLK_I2C1EN_Pos)                  /*!< CLK_T::APBCLK: I2C1EN Mask             */

#define CLK_APBCLK_SPI1EN_Pos            (11)                                              /*!< CLK_T::APBCLK: SPI1EN Position         */
#define CLK_APBCLK_SPI1EN_Msk            (0x1ul << CLK_APBCLK_SPI1EN_Pos)                  /*!< CLK_T::APBCLK: SPI1EN Mask             */

#define CLK_APBCLK_SPI0EN_Pos            (12)                                              /*!< CLK_T::APBCLK: SPI0EN Position         */
#define CLK_APBCLK_SPI0EN_Msk            (0x1ul << CLK_APBCLK_SPI0EN_Pos)                  /*!< CLK_T::APBCLK: SPI0EN Mask             */

#define CLK_APBCLK_I2S0EN_Pos            (13)                                              /*!< CLK_T::APBCLK: I2S0EN Position          */
#define CLK_APBCLK_I2S0EN_Msk            (0x1ul << CLK_APBCLK_I2S0EN_Pos)                   /*!< CLK_T::APBCLK: I2S0EN Mask              */

#define CLK_APBCLK_UART0EN_Pos           (16)                                              /*!< CLK_T::APBCLK: UART0EN Position        */
#define CLK_APBCLK_UART0EN_Msk           (0x1ul << CLK_APBCLK_UART0EN_Pos)                 /*!< CLK_T::APBCLK: UART0EN Mask            */

#define CLK_APBCLK_UART1EN_Pos           (17)                                              /*!< CLK_T::APBCLK: UART1EN Position        */
#define CLK_APBCLK_UART1EN_Msk           (0x1ul << CLK_APBCLK_UART1EN_Pos)                 /*!< CLK_T::APBCLK: UART1EN Mask            */

#define CLK_APBCLK_BIQEN_Pos             (18)                                              /*!< CLK_T::APBCLK: BIQEN Position          */
#define CLK_APBCLK_BIQEN_Msk             (0x1ul << CLK_APBCLK_BIQEN_Pos)                   /*!< CLK_T::APBCLK: BIQEN Mask              */

#define CLK_APBCLK_PWM0EN_Pos            (20)                                              /*!< CLK_T::APBCLK: PWM0EN Position         */
#define CLK_APBCLK_PWM0EN_Msk            (0x1ul << CLK_APBCLK_PWM0EN_Pos)                  /*!< CLK_T::APBCLK: PWM0EN Mask             */

#define CLK_APBCLK_PWM1EN_Pos            (21)                                              /*!< CLK_T::APBCLK: PWM1EN Position         */
#define CLK_APBCLK_PWM1EN_Msk            (0x1ul << CLK_APBCLK_PWM1EN_Pos)                  /*!< CLK_T::APBCLK: PWM1EN Mask             */

#define CLK_APBCLK_USBEN_Pos             (24)                                              /*!< CLK_T::APBCLK: USBEN Position          */
#define CLK_APBCLK_USBEN_Msk             (0x1ul << CLK_APBCLK_USBEN_Pos)                   /*!< CLK_T::APBCLK: USBEN Mask              */

#define CLK_APBCLK_SARADCEN_Pos          (28)                                              /*!< CLK_T::APBCLK: SARADCEN Position       */
#define CLK_APBCLK_SARADCEN_Msk          (0x1ul << CLK_APBCLK_SARADCEN_Pos)                /*!< CLK_T::APBCLK: SARADCEN Mask           */

#define CLK_APBCLK_DACEN_Pos             (29)                                              /*!< CLK_T::APBCLK: DACEN Position          */
#define CLK_APBCLK_DACEN_Msk             (0x1ul << CLK_APBCLK_DACEN_Pos)                   /*!< CLK_T::APBCLK: DACEN Mask              */

#define CLK_APBCLK_SDADCEN_Pos           (30)                                              /*!< CLK_T::APBCLK: SDADCEN Position        */
#define CLK_APBCLK_SDADCEN_Msk           (0x1ul << CLK_APBCLK_SDADCEN_Pos)                 /*!< CLK_T::APBCLK: SDADCEN Mask            */

#define CLK_APBCLK_ANAEN_Pos             (31)                                              /*!< CLK_T::APBCLK: ANAEN Position          */
#define CLK_APBCLK_ANAEN_Msk             (0x1ul << CLK_APBCLK_ANAEN_Pos)                   /*!< CLK_T::APBCLK: ANAEN Mask              */

#define CLK_DPDFLR_PD_STATE_Pos          (0)                                               /*!< CLK_T::DPDFLR: PD_STATE Position       */
#define CLK_DPDFLR_PD_STATE_Msk          (0xfful << CLK_DPDFLR_PD_STATE_Pos)               /*!< CLK_T::DPDFLR: PD_STATE Mask           */

#define CLK_DPDFLR_PD_STATE_RB_Pos       (8)                                               /*!< CLK_T::DPDFLR: PD_STATE_RB Position    */
#define CLK_DPDFLR_PD_STATE_RB_Msk       (0xfful << CLK_DPDFLR_PD_STATE_RB_Pos)            /*!< CLK_T::DPDFLR: PD_STATE_RB Mask        */

#define CLK_CLKSEL0_HCLKSEL_Pos          (0)                                               /*!< CLK_T::CLKSEL0: HCLKSEL Position       */
#define CLK_CLKSEL0_HCLKSEL_Msk          (0x3ul << CLK_CLKSEL0_HCLKSEL_Pos)                /*!< CLK_T::CLKSEL0: HCLKSEL Mask           */

#define CLK_CLKSEL0_STICKSEL_Pos         (3)                                               /*!< CLK_T::CLKSEL0: STICKSEL Position      */
#define CLK_CLKSEL0_STICKSEL_Msk         (0x3ul << CLK_CLKSEL0_STICKSEL_Pos)               /*!< CLK_T::CLKSEL0: STICKSEL Mask          */

#define CLK_CLKSEL0_OSCFSEL_Pos          (6)                                               /*!< CLK_T::CLKSEL0: OSCFSEL Position       */
#define CLK_CLKSEL0_OSCFSEL_Msk          (0x3ul << CLK_CLKSEL0_OSCFSEL_Pos)                /*!< CLK_T::CLKSEL0: OSCFSEL Mask           */

#define CLK_CLKSEL0_FCLK_MUX_STATE_Pos   (16)                                              /*!< CLK_T::CLKSEL0: FCLK_MUX_STATE Position*/
#define CLK_CLKSEL0_FCLK_MUX_STATE_Msk   (0x7ul << CLK_CLKSEL0_FCLK_MUX_STATE_Pos)         /*!< CLK_T::CLKSEL0: FCLK_MUX_STATE Mask    */

#define CLK_CLKSEL1_WDTSEL_Pos           (0)                                               /*!< CLK_T::CLKSEL1: WDTSEL Position        */
#define CLK_CLKSEL1_WDTSEL_Msk           (0x1ul << CLK_CLKSEL1_WDTSEL_Pos)                 /*!< CLK_T::CLKSEL1: WDTSEL Mask            */

#define CLK_CLKSEL1_SARADCSEL_Pos        (2)                                               /*!< CLK_T::CLKSEL1: SARADCSEL Position     */
#define CLK_CLKSEL1_SARADCSEL_Msk        (0x3ul << CLK_CLKSEL1_SARADCSEL_Pos)              /*!< CLK_T::CLKSEL1: SARADCSEL Mask         */

#define CLK_CLKSEL1_TMR0SEL_Pos          (8)                                               /*!< CLK_T::CLKSEL1: TMR0SEL Position       */
#define CLK_CLKSEL1_TMR0SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR0SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR0SEL Mask           */

#define CLK_CLKSEL1_TMR1SEL_Pos          (12)                                              /*!< CLK_T::CLKSEL1: TMR1SEL Position       */
#define CLK_CLKSEL1_TMR1SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR1SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR1SEL Mask           */

#define CLK_CLKSEL1_TMR2SEL_Pos          (16)                                              /*!< CLK_T::CLKSEL1: TMR2SEL Position       */
#define CLK_CLKSEL1_TMR2SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR2SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR2SEL Mask           */

#define CLK_CLKSEL1_I2S0SEL_Pos          (20)                                              /*!< CLK_T::CLKSEL1: I2S0SEL Position        */
#define CLK_CLKSEL1_I2S0SEL_Msk          (0x7ul << CLK_CLKSEL1_I2S0SEL_Pos)                 /*!< CLK_T::CLKSEL1: I2S0SEL Mask            */

#define CLK_CLKSEL1_UART0SEL_Pos         (24)                                              /*!< CLK_T::CLKSEL1: UART0SEL Position      */
#define CLK_CLKSEL1_UART0SEL_Msk         (0x3ul << CLK_CLKSEL1_UART0SEL_Pos)               /*!< CLK_T::CLKSEL1: UART0SEL Mask          */

#define CLK_CLKSEL1_UART1SEL_Pos         (26)                                              /*!< CLK_T::CLKSEL1: UART1SEL Position      */
#define CLK_CLKSEL1_UART1SEL_Msk         (0x3ul << CLK_CLKSEL1_UART1SEL_Pos)               /*!< CLK_T::CLKSEL1: UART1SEL Mask          */

#define CLK_CLKSEL1_PWM0SEL_Pos          (28)                                              /*!< CLK_T::CLKSEL1: PWM0SEL Position       */
#define CLK_CLKSEL1_PWM0SEL_Msk          (0x3ul << CLK_CLKSEL1_PWM0SEL_Pos)                /*!< CLK_T::CLKSEL1: PWM0SEL Mask           */

#define CLK_CLKSEL1_PWM1SEL_Pos          (30)                                              /*!< CLK_T::CLKSEL1: PWM1SEL Position       */
#define CLK_CLKSEL1_PWM1SEL_Msk          (0x3ul << CLK_CLKSEL1_PWM1SEL_Pos)                /*!< CLK_T::CLKSEL1: PWM1SEL Mask           */

#define CLK_CLKDIV0_HCLKDIV_Pos          (0)                                               /*!< CLK_T::CLKDIV0: HCLKDIV Position       */
#define CLK_CLKDIV0_HCLKDIV_Msk          (0xful << CLK_CLKDIV0_HCLKDIV_Pos)                /*!< CLK_T::CLKDIV0: HCLKDIV Mask           */

#define CLK_CLKDIV0_UART0DIV_Pos         (4)                                               /*!< CLK_T::CLKDIV0: UART0DIV Position      */
#define CLK_CLKDIV0_UART0DIV_Msk         (0xful << CLK_CLKDIV0_UART0DIV_Pos)               /*!< CLK_T::CLKDIV0: UART0DIV Mask          */

#define CLK_CLKDIV0_UART1DIV_Pos         (8)                                               /*!< CLK_T::CLKDIV0: UART1DIV Position      */
#define CLK_CLKDIV0_UART1DIV_Msk         (0xful << CLK_CLKDIV0_UART1DIV_Pos)               /*!< CLK_T::CLKDIV0: UART1DIV Mask          */

#define CLK_CLKDIV0_USBDIV_Pos           (12)                                              /*!< CLK_T::CLKDIV0: USBDIV Position        */
#define CLK_CLKDIV0_USBDIV_Msk           (0xful << CLK_CLKDIV0_USBDIV_Pos)                 /*!< CLK_T::CLKDIV0: USBDIV Mask            */

#define CLK_CLKDIV0_SARADCDIV_Pos        (16)                                              /*!< CLK_T::CLKDIV0: SARADCDIV Position     */
#define CLK_CLKDIV0_SARADCDIV_Msk        (0x7ful << CLK_CLKDIV0_SARADCDIV_Pos)             /*!< CLK_T::CLKDIV0: SARADCDIV Mask         */

#define CLK_CLKDIV1_DACDIV_Pos           (0)                                               /*!< CLK_T::CLKDIV1: DACDIV Position        */
#define CLK_CLKDIV1_DACDIV_Msk           (0xfful << CLK_CLKDIV1_DACDIV_Pos)                /*!< CLK_T::CLKDIV1: DACDIV Mask            */

#define CLK_CLKDIV1_BIQDIV_Pos           (8)                                               /*!< CLK_T::CLKDIV1: BIQDIV Position        */
#define CLK_CLKDIV1_BIQDIV_Msk           (0xful << CLK_CLKDIV1_BIQDIV_Pos)                 /*!< CLK_T::CLKDIV1: BIQDIV Mask            */

#define CLK_CLKDIV1_SDADCDIV_Pos         (16)                                              /*!< CLK_T::CLKDIV1: SDADCDIV Position      */
#define CLK_CLKDIV1_SDADCDIV_Msk         (0xfful << CLK_CLKDIV1_SDADCDIV_Pos)              /*!< CLK_T::CLKDIV1: SDADCDIV Mask          */

#define CLK_STATUS_HXTSTB_Pos            (0)                                               /*!< CLK_T::STATUS: HXTSTB Position         */
#define CLK_STATUS_HXTSTB_Msk            (0x1ul << CLK_STATUS_HXTSTB_Pos)                  /*!< CLK_T::STATUS: HXTSTB Mask             */

#define CLK_STATUS_LIRCSTB_Pos           (1)                                               /*!< CLK_T::STATUS: LIRCSTB Position        */
#define CLK_STATUS_LIRCSTB_Msk           (0x1ul << CLK_STATUS_LIRCSTB_Pos)                 /*!< CLK_T::STATUS: LIRCSTB Mask            */

#define CLK_STATUS_HIRCSTB_Pos           (2)                                               /*!< CLK_T::STATUS: HIRCSTB Position        */
#define CLK_STATUS_HIRCSTB_Msk           (0x1ul << CLK_STATUS_HIRCSTB_Pos)                 /*!< CLK_T::STATUS: HIRCSTB Mask            */

#define CLK_STATUS_PLLSTB_Pos            (3)                                               /*!< CLK_T::STATUS: PLLSTB Position         */
#define CLK_STATUS_PLLSTB_Msk            (0x1ul << CLK_STATUS_PLLSTB_Pos)                  /*!< CLK_T::STATUS: PLLSTB Mask             */

#define CLK_STATUS_XCLKSTB_Pos           (4)                                               /*!< CLK_T::STATUS: XCLKSTB Position        */
#define CLK_STATUS_XCLKSTB_Msk           (0x1ul << CLK_STATUS_XCLKSTB_Pos)                 /*!< CLK_T::STATUS: XCLKSTB Mask            */

#define CLK_PFLAG_DSF_Pos                (0)                                               /*!< CLK_T::PFLAG: DSF Position             */
#define CLK_PFLAG_DSF_Msk                (0x1ul << CLK_PFLAG_DSF_Pos)                      /*!< CLK_T::PFLAG: DSF Mask                 */

#define CLK_PFLAG_STOPF_Pos              (1)                                               /*!< CLK_T::PFLAG: STOPF Position           */
#define CLK_PFLAG_STOPF_Msk              (0x1ul << CLK_PFLAG_STOPF_Pos)                    /*!< CLK_T::PFLAG: STOPF Mask               */

#define CLK_CLKSEL2_USBSEL_Pos           (0)                                               /*!< CLK_T::CLKSEL2: USBSEL Position        */
#define CLK_CLKSEL2_USBSEL_Msk           (0x1ul << CLK_CLKSEL2_USBSEL_Pos)                 /*!< CLK_T::CLKSEL2: USBSEL Mask            */

#define CLK_CLKSEL2_XCLKSEL_Pos          (4)                                               /*!< CLK_T::CLKSEL2: XCLKSEL Position       */
#define CLK_CLKSEL2_XCLKSEL_Msk          (0x1ul << CLK_CLKSEL2_XCLKSEL_Pos)                /*!< CLK_T::CLKSEL2: XCLKSEL Mask           */

#define CLK_CLKSEL2_DACSEL_Pos           (8)                                               /*!< CLK_T::CLKSEL2: DACSEL Position        */
#define CLK_CLKSEL2_DACSEL_Msk           (0x7ul << CLK_CLKSEL2_DACSEL_Pos)                 /*!< CLK_T::CLKSEL2: DACSEL Mask            */

#define CLK_CLKSEL2_SDADCSEL_Pos         (12)                                              /*!< CLK_T::CLKSEL2: SDADCSEL Position      */
#define CLK_CLKSEL2_SDADCSEL_Msk         (0x7ul << CLK_CLKSEL2_SDADCSEL_Pos)               /*!< CLK_T::CLKSEL2: SDADCSEL Mask          */

#define CLK_XCLKCTL_XCLKMUL_Pos          (0)                                               /*!< CLK_T::XCLKCTL: XCLKMUL Position       */
#define CLK_XCLKCTL_XCLKMUL_Msk          (0x3ul << CLK_XCLKCTL_XCLKMUL_Pos)                /*!< CLK_T::XCLKCTL: XCLKMUL Mask           */

#define CLK_XCLKCTL_XCLKEN_Pos           (4)                                               /*!< CLK_T::XCLKCTL: XCLKEN Position        */
#define CLK_XCLKCTL_XCLKEN_Msk           (0x1ul << CLK_XCLKCTL_XCLKEN_Pos)                 /*!< CLK_T::XCLKCTL: XCLKEN Mask            */

#define CLK_XCLKCTL_RELOCK_Pos           (5)                                               /*!< CLK_T::XCLKCTL: RELOCK Position        */
#define CLK_XCLKCTL_RELOCK_Msk           (0x1ul << CLK_XCLKCTL_RELOCK_Pos)                 /*!< CLK_T::XCLKCTL: RELOCK Mask            */

#define CLK_XCLKCTL_XCLKFDEN_Pos         (12)                                              /*!< CLK_T::XCLKCTL: XCLKFDEN Position      */
#define CLK_XCLKCTL_XCLKFDEN_Msk         (0x1ul << CLK_XCLKCTL_XCLKFDEN_Pos)               /*!< CLK_T::XCLKCTL: XCLKFDEN Mask          */

#define CLK_XCLKCTL_XCLKFIEN_Pos         (13)                                              /*!< CLK_T::XCLKCTL: XCLKFIEN Position      */
#define CLK_XCLKCTL_XCLKFIEN_Msk         (0x1ul << CLK_XCLKCTL_XCLKFIEN_Pos)               /*!< CLK_T::XCLKCTL: XCLKFIEN Mask          */

#define CLK_XCLKCTL_XCLKFIF_Pos          (14)                                              /*!< CLK_T::XCLKCTL: XCLKFIF Position       */
#define CLK_XCLKCTL_XCLKFIF_Msk          (0x1ul << CLK_XCLKCTL_XCLKFIF_Pos)                /*!< CLK_T::XCLKCTL: XCLKFIF Mask           */

#define CLK_PLLCTL_FBDIV_Pos             (0)                                               /*!< CLK_T::PLLCTL: FBDIV Position          */
#define CLK_PLLCTL_FBDIV_Msk             (0x3ful << CLK_PLLCTL_FBDIV_Pos)                  /*!< CLK_T::PLLCTL: FBDIV Mask              */

#define CLK_PLLCTL_INDIV_Pos             (9)                                               /*!< CLK_T::PLLCTL: INDIV Position          */
#define CLK_PLLCTL_INDIV_Msk             (0xful << CLK_PLLCTL_INDIV_Pos)                   /*!< CLK_T::PLLCTL: INDIV Mask              */

#define CLK_PLLCTL_OUTDIV_Pos            (14)                                              /*!< CLK_T::PLLCTL: OUTDIV Position         */
#define CLK_PLLCTL_OUTDIV_Msk            (0x3ul << CLK_PLLCTL_OUTDIV_Pos)                  /*!< CLK_T::PLLCTL: OUTDIV Mask             */

#define CLK_PLLCTL_PD_Pos                (16)                                              /*!< CLK_T::PLLCTL: PD Position             */
#define CLK_PLLCTL_PD_Msk                (0x1ul << CLK_PLLCTL_PD_Pos)                      /*!< CLK_T::PLLCTL: PD Mask                 */

#define CLK_PLLCTL_BP_Pos                (17)                                              /*!< CLK_T::PLLCTL: BP Position             */
#define CLK_PLLCTL_BP_Msk                (0x1ul << CLK_PLLCTL_BP_Pos)                      /*!< CLK_T::PLLCTL: BP Mask                 */

#define CLK_PLLCTL_OE_Pos                (18)                                              /*!< CLK_T::PLLCTL: OE Position             */
#define CLK_PLLCTL_OE_Msk                (0x1ul << CLK_PLLCTL_OE_Pos)                      /*!< CLK_T::PLLCTL: OE Mask                 */

#define CLK_PLLCTL_PLLSRC_Pos            (19)                                              /*!< CLK_T::PLLCTL: PLLSRC Position         */
#define CLK_PLLCTL_PLLSRC_Msk            (0x3ul << CLK_PLLCTL_PLLSRC_Pos)                  /*!< CLK_T::PLLCTL: PLLSRC Mask             */

#define CLK_PLLCTL_FTREN_Pos             (22)                                              /*!< CLK_T::PLLCTL: FTREN Position          */
#define CLK_PLLCTL_FTREN_Msk             (0x1ul << CLK_PLLCTL_FTREN_Pos)                   /*!< CLK_T::PLLCTL: FTREN Mask              */

#define CLK_PLLCTL_STBSEL_Pos            (23)                                              /*!< CLK_T::PLLCTL: STBSEL Position         */
#define CLK_PLLCTL_STBSEL_Msk            (0x1ul << CLK_PLLCTL_STBSEL_Pos)                  /*!< CLK_T::PLLCTL: STBSEL Mask             */

#define CLK_ILDOCTL_PD_Pos               (0)                                               /*!< CLK_T::PLLTEST: IVCO Position          */
#define CLK_ILDOCTL_PD_Msk               (0x1ul << CLK_ILDOCTL_PD_Pos)                     /*!< CLK_T::PLLTEST: IVCO Mask              */

#define CLK_ILDOCTL_SW_Pos               (1)                                              /*!< CLK_T::PLLTEST: IVCO Position          */
#define CLK_ILDOCTL_SW_Msk               (0x1ul << CLK_ILDOCTL_SW_Pos)                    /*!< CLK_T::PLLTEST: IVCO Mask              */


/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */


/*---------------------- Companding Control Registers -------------------------*/
/**
    @addtogroup CPD Companding Control Registers(CPD)
    Memory Mapped Structure for CPD Controller
@{ */
 
typedef struct
{


/**
 * @var CPD_T::CTRL
 * Offset: 0x00  CPD Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |EN        |CPD enable control
 * |        |          |0 = CPD disable
 * |        |          |1 = CPD enable
 * |[1]     |MODE      |CPD encode/decode algorithm select
 * |        |          |0 = ADPCM (G.726)
 * |        |          |1 = A-law / u-law (G.711)
 * |[2]     |TYPE      |CPD encoder input and decoder output type select
 * |        |          |0 = A-law / u-law
 * |        |          |1 = PCM
 * |[3]     |LAW       |CPD A-law / u-law select
 * |        |          |0 = u-law
 * |        |          |1 = A-law
 * |[5:4]   |BITRATE   |CPD ADPCM bitrate select
 * |        |          |0 = 16K bit/s (2 bits per sample)
 * |        |          |1 = 24K bit/s (3 bits per sample)
 * |        |          |2 = 32K bit/s (4 bits per sample)
 * |        |          |3 = 40K bit/s (5 bits per sample)
 * |[8]     |ENCRST    |Encoder reset register
 * |        |          |Write 1 to this bit will reset encoder state machine and clear input/output FIFO.
 * |        |          |This bit will auto change to 0 after reset done.
 * |[9]     |DECRST    |Decoder reset register
 * |        |          |Write 1 to this bit will reset decoder state machine and clear input/output FIFO.
 * |        |          |This bit will auto change to 0 after reset done.
 * |[18:16] |EITH      |Encoder input FIFO Threshold Level
 * |        |          |If the valid data count of the FIFO data buffer is less than or equal to EITH (CPD_CTRL[18:16]) setting, the EITHIF (CPD_STS[3]) will set to 1, else the DITHIF (CPD_STS[3]) will be cleared to 0.
 * |[19]    |EITHIE    |Encoder input FIFO Threshold Interrupt
 * |        |          |0 =Encoder input FIFO threshold interrupt Disabled
 * |        |          |1 =Encoder input FIFO threshold interrupt Enabled.
 * |[22:20] |EOTH      |Encoder output FIFO Threshold Level
 * |        |          |If the valid data count of the FIFO data buffer is larger than or equal to EOTH (CPD_CTRL[22:20]) setting, the EOTHIF (CPD_STS[11]) will set to 1, else the EOTHIF (CPD_STS[11]) will be cleared to 0.
 * |[23]    |EOTHIE    |Encoder output FIFO Threshold Interrupt
 * |        |          |0 =Encoder output FIFO threshold interrupt Disabled
 * |        |          |1 =Encoder output FIFO threshold interrupt Enabled.
 * |[26:24] |DITH      |Decoder input FIFO Threshold Level
 * |        |          |If the valid data count of the FIFO data buffer is less than or equal to DITH (CPD_CTRL[26:24]) setting, the DITHIF (CPD_STS[19]) will set to 1, else the DITHIF (CPD_STS[19]) will be cleared to 0.
 * |[27]    |DITHIE    |Decoder input FIFO Threshold Interrupt
 * |        |          |0 =Decoder input FIFO threshold interrupt Disabled
 * |        |          |1 =Decoder input FIFO threshold interrupt Enabled.
 * |[30:28] |DOTH      |Decoder output FIFO Threshold Level
 * |        |          |If the valid data count of the FIFO data buffer is larger than or equal to DOTH (CPD_CTRL[30:28]) setting, the DOTHIF (CPD_STS[27]) will set to 1, else the DOTHIF (CPD_STS[27]) will be cleared to 0.
 * |[31]    |DOTHIE    |Decoder output FIFO Threshold Interrupt
 * |        |          |0 =Decoder output FIFO threshold interrupt Disabled
 * |        |          |1 =Decoder output FIFO threshold interrupt Enabled.
 * @var CPD_T::STS
 * Offset: 0x04  CPD FIFO Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |EIF       |CPD encoder input FIFO full flag
 * |        |          |0 = CPD encoder input FIFO is NOT full
 * |        |          |1 = CPD encoder input FIFO is full
 * |[1]     |EIE       |CPD encoder input FIFO empty flag
 * |        |          |0 = CPD encoder input FIFO is NOT empty
 * |        |          |1 = CPD encoder input FIFO is empty
 * |[2]     |EIOV      |CPD encoder input FIFO overflow flag
 * |        |          |If encoder input FIFO (CPD->CPD_ENC_IN) is full, and an additional data is written to the FIFO, an overflow condition will occur and set this bit to logic 1.
 * |[3]     |EITHIF    |CPD encoder input FIFO Threshold Interrupt Status (Read Only)
 * |        |          |0 = The valid data count within the FIFO data buffer is more than the setting value of EITH (CPD_CTL[18:16]).
 * |        |          |1 = The valid data count within the FIFO data buffer is less than or equal to the setting value of EITH (CPD_CTL[18:16]).
 * |[7:4]   |EIFPTR    |CPD encoder input FIFO Pointer (Read Only)
 * |        |          |The FULL (CPD_STS[0]) and FIFOPTR (CPD_STS[7:4]) indicates the field that the valid data count within the encoder input FIFO buffer.
 * |        |          |The maximum value shown in FIFOPTR is 8
 * |        |          |When the using level of encoder input FIFO buffer equal to 8, The FULL (CPD_STS[0]) is set to 1.
 * |[8]     |EOF       |CPD encoder output FIFO full flag
 * |        |          |0 = CPD encoder output FIFO is NOT full
 * |        |          |1 = CPD encoder output FIFO is full
 * |[9]     |EOE       |CPD encoder output FIFO empty flag
 * |        |          |0 = CPD encoder output FIFO is NOT empty
 * |        |          |1 = CPD encoder output FIFO is empty
 * |[10]    |EOOV      |CPD encoder output FIFO overflow flag
 * |        |          |If encoder output FIFO (CPD->CPD_ENC_OUT) is full, and an additional converted data is written to the FIFO, an overflow condition will occur and set this bit to logic 1.
 * |[11]    |EOTHIF    |CPD encoder output FIFO Threshold Interrupt Status (Read Only)
 * |        |          |0 = The valid data count within the FIFO data buffer is less than the setting value of EOTH (CPD_CTL[22:20]).
 * |        |          |1 = The valid data count within the FIFO data buffer is more than or equal to the setting value of EOTH (CPD_CTL[22:20]).
 * |[15:12] |EOFPTR    |CPD encoder output FIFO Pointer (Read Only)
 * |        |          |The FULL (CPD_STS[8]) and FIFOPTR (CPD_STS[15:12]) indicates the field that the valid data count within the encoder output FIFO buffer.
 * |        |          |The maximum value shown in FIFOPTR is 8
 * |        |          |When the using level of encoder output FIFO buffer equal to 8, The FULL (CPD_STS[8]) is set to 1.
 * |[16]    |DIF       |CPD decoder input FIFO full flag
 * |        |          |0 = CPD decoder input FIFO is NOT full
 * |        |          |1 = CPD decoder input FIFO is full
 * |[17]    |DIE       |CPD decoder input FIFO empty flag
 * |        |          |0 = CPD decoder input FIFO is NOT empty
 * |        |          |1 = CPD decoder input FIFO is empty
 * |[18]    |DIOV      |CPD decoder input FIFO overflow flag
 * |        |          |If decoder input FIFO (CPD->CPD_DEC_IN) is full, and an additional data is written to the FIFO, an overflow condition will occur and set this bit to logic 1.
 * |[19]    |DITHIF    |CPD decoder input FIFO Threshold Interrupt Status (Read Only)
 * |        |          |0 = The valid data count within the FIFO data buffer is more than the setting value of DITH (CPD_CTL[26:24]).
 * |        |          |1 = The valid data count within the FIFO data buffer is less than or equal to the setting value of DITH (CPD_CTL[26:24]).
 * |[23:20] |DIFPTR    |CPD decoder input FIFO Pointer (Read Only)
 * |        |          |The FULL (CPD_STS[16]) and FIFOPTR (CPD_STS[23:20]) indicates the field that the valid data count within the decoder input FIFO buffer.
 * |        |          |The maximum value shown in FIFOPTR is 8
 * |        |          |When the using level of decoder input FIFO buffer equal to 8, The FULL (CPD_STS[16]) is set to 1.
 * |[24]    |DOF       |CPD decoder output FIFO full flag
 * |        |          |0 = CPD decoder output FIFO is NOT full
 * |        |          |1 = CPD decoder output FIFO is full
 * |[25]    |DOE       |CPD decoder output FIFO empty flag
 * |        |          |0 = CPD decoder output FIFO is NOT empty
 * |        |          |1 = CPD decoder output FIFO is empty
 * |[26]    |DOOV      |CPD decoder output FIFO overflow flag
 * |        |          |If decoder output FIFO (CPD->CPD_DEC_OUT) is full, and an additional converted data is written to the FIFO, an overflow condition will occur and set this bit to logic 1.
 * |[27]    |DOTHIF    |CPD decoder output FIFO Threshold Interrupt Status (Read Only)
 * |        |          |0 = The valid data count within the FIFO data buffer is less than the setting value of DOTH (CPD_CTL[30:28]).
 * |        |          |1 = The valid data count within the FIFO data buffer is more than or equal to the setting value of DOTH (CPD_CTL[30:28]).
 * |[31:28] |DOFPTR    |CPD decoder output FIFO Pointer (Read Only)
 * |        |          |The FULL (CPD_STS[24]) and FIFOPTR (CPD_STS[31:28]) indicates the field that the valid data count within the Decoder output FIFO buffer.
 * |        |          |The maximum value shown in FIFOPTR is 8
 * |        |          |When the using level of decoder output FIFO buffer equal to 8, The FULL (CPD_STS[24]) is set to 1.
 * @var CPD_T::ENCIN
 * Offset: 0x08  CPD Encoder Input FIFO
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |ENCIN     |CPD encoder input FIFO
 * |        |          |By writing to this register, encoder input data will be pushed onto the transmit FIFO
 * |        |          |CPD will start encoding if this FIFO is not empty.
 * @var CPD_T::ENCOUT
 * Offset: 0x0C  CPD Encoder Output FIFO
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ENCOUT    |CPD encoder output FIFO
 * |        |          |Reading this register will return data from encoder output data FIFO.
 * @var CPD_T::DECIN
 * Offset: 0x10  CPD Decoder Input FIFO
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DECIN     |CPD decoder input FIFO
 * |        |          |By writing to this register, decoder input data will be pushed onto the transmit FIFO
 * |        |          |CPD will start encoding if this FIFO is not empty.
 * @var CPD_T::DECOUT
 * Offset: 0x14  CPD Decoder Output FIFO
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |DECOUT    |CPD decoder output FIFO
 * |        |          |Reading this register will return data from decoder output data FIFO.
 */
    __IO uint32_t CTRL;                  /*!< [0x0000] CPD Control Register                                             */
    __IO uint32_t STS;                   /*!< [0x0004] CPD FIFO Status Register                                         */
    __O  uint32_t ENCIN;                 /*!< [0x0008] CPD Encoder Input FIFO                                           */
    __I  uint32_t ENCOUT;                /*!< [0x000c] CPD Encoder Output FIFO                                          */
    __O  uint32_t DECIN;                 /*!< [0x0010] CPD Decoder Input FIFO                                           */
    __I  uint32_t DECOUT;                /*!< [0x0014] CPD Decoder Output FIFO                                          */

} CPD_T;

/**
    @addtogroup CPD_CONST CPD Bit Field Definition
    Constant Definitions for CPD Controller
@{ */

#define CPD_CTRL_EN_Pos                  (0)                                               /*!< CPD_T::CTRL: EN Position               */
#define CPD_CTRL_EN_Msk                  (0x1ul << CPD_CTRL_EN_Pos)                        /*!< CPD_T::CTRL: EN Mask                   */

#define CPD_CTRL_MODE_Pos                (1)                                               /*!< CPD_T::CTRL: MODE Position             */
#define CPD_CTRL_MODE_Msk                (0x1ul << CPD_CTRL_MODE_Pos)                      /*!< CPD_T::CTRL: MODE Mask                 */

#define CPD_CTRL_TYPE_Pos                (2)                                               /*!< CPD_T::CTRL: TYPE Position             */
#define CPD_CTRL_TYPE_Msk                (0x1ul << CPD_CTRL_TYPE_Pos)                      /*!< CPD_T::CTRL: TYPE Mask                 */

#define CPD_CTRL_LAW_Pos                 (3)                                               /*!< CPD_T::CTRL: LAW Position              */
#define CPD_CTRL_LAW_Msk                 (0x1ul << CPD_CTRL_LAW_Pos)                       /*!< CPD_T::CTRL: LAW Mask                  */

#define CPD_CTRL_BITRATE_Pos             (4)                                               /*!< CPD_T::CTRL: BITRATE Position          */
#define CPD_CTRL_BITRATE_Msk             (0x3ul << CPD_CTRL_BITRATE_Pos)                   /*!< CPD_T::CTRL: BITRATE Mask              */

#define CPD_CTRL_ENCRST_Pos              (8)                                               /*!< CPD_T::CTRL: ENCRST Position           */
#define CPD_CTRL_ENCRST_Msk              (0x1ul << CPD_CTRL_ENCRST_Pos)                    /*!< CPD_T::CTRL: ENCRST Mask               */

#define CPD_CTRL_DECRST_Pos              (9)                                               /*!< CPD_T::CTRL: DECRST Position           */
#define CPD_CTRL_DECRST_Msk              (0x1ul << CPD_CTRL_DECRST_Pos)                    /*!< CPD_T::CTRL: DECRST Mask               */

#define CPD_CTRL_EITH_Pos                (16)                                              /*!< CPD_T::CTRL: EITH Position             */
#define CPD_CTRL_EITH_Msk                (0x7ul << CPD_CTRL_EITH_Pos)                      /*!< CPD_T::CTRL: EITH Mask                 */

#define CPD_CTRL_EITHIE_Pos              (19)                                              /*!< CPD_T::CTRL: EITHIE Position           */
#define CPD_CTRL_EITHIE_Msk              (0x1ul << CPD_CTRL_EITHIE_Pos)                    /*!< CPD_T::CTRL: EITHIE Mask               */

#define CPD_CTRL_EOTH_Pos                (20)                                              /*!< CPD_T::CTRL: EOTH Position             */
#define CPD_CTRL_EOTH_Msk                (0x7ul << CPD_CTRL_EOTH_Pos)                      /*!< CPD_T::CTRL: EOTH Mask                 */

#define CPD_CTRL_EOTHIE_Pos              (23)                                              /*!< CPD_T::CTRL: EOTHIE Position           */
#define CPD_CTRL_EOTHIE_Msk              (0x1ul << CPD_CTRL_EOTHIE_Pos)                    /*!< CPD_T::CTRL: EOTHIE Mask               */

#define CPD_CTRL_DITH_Pos                (24)                                              /*!< CPD_T::CTRL: DITH Position             */
#define CPD_CTRL_DITH_Msk                (0x7ul << CPD_CTRL_DITH_Pos)                      /*!< CPD_T::CTRL: DITH Mask                 */

#define CPD_CTRL_DITHIE_Pos              (27)                                              /*!< CPD_T::CTRL: DITHIE Position           */
#define CPD_CTRL_DITHIE_Msk              (0x1ul << CPD_CTRL_DITHIE_Pos)                    /*!< CPD_T::CTRL: DITHIE Mask               */

#define CPD_CTRL_DOTH_Pos                (28)                                              /*!< CPD_T::CTRL: DOTH Position             */
#define CPD_CTRL_DOTH_Msk                (0x7ul << CPD_CTRL_DOTH_Pos)                      /*!< CPD_T::CTRL: DOTH Mask                 */

#define CPD_CTRL_DOTHIE_Pos              (31)                                              /*!< CPD_T::CTRL: DOTHIE Position           */
#define CPD_CTRL_DOTHIE_Msk              (0x1ul << CPD_CTRL_DOTHIE_Pos)                    /*!< CPD_T::CTRL: DOTHIE Mask               */

#define CPD_STS_EIF_Pos                  (0)                                               /*!< CPD_T::STS: EIF Position               */
#define CPD_STS_EIF_Msk                  (0x1ul << CPD_STS_EIF_Pos)                        /*!< CPD_T::STS: EIF Mask                   */

#define CPD_STS_EIE_Pos                  (1)                                               /*!< CPD_T::STS: EIE Position               */
#define CPD_STS_EIE_Msk                  (0x1ul << CPD_STS_EIE_Pos)                        /*!< CPD_T::STS: EIE Mask                   */

#define CPD_STS_EIOV_Pos                 (2)                                               /*!< CPD_T::STS: EIOV Position              */
#define CPD_STS_EIOV_Msk                 (0x1ul << CPD_STS_EIOV_Pos)                       /*!< CPD_T::STS: EIOV Mask                  */

#define CPD_STS_EITHIF_Pos               (3)                                               /*!< CPD_T::STS: EITHIF Position            */
#define CPD_STS_EITHIF_Msk               (0x1ul << CPD_STS_EITHIF_Pos)                     /*!< CPD_T::STS: EITHIF Mask                */

#define CPD_STS_EIFPTR_Pos               (4)                                               /*!< CPD_T::STS: EIFPTR Position            */
#define CPD_STS_EIFPTR_Msk               (0xful << CPD_STS_EIFPTR_Pos)                     /*!< CPD_T::STS: EIFPTR Mask                */

#define CPD_STS_EOF_Pos                  (8)                                               /*!< CPD_T::STS: EOF Position               */
#define CPD_STS_EOF_Msk                  (0x1ul << CPD_STS_EOF_Pos)                        /*!< CPD_T::STS: EOF Mask                   */

#define CPD_STS_EOE_Pos                  (9)                                               /*!< CPD_T::STS: EOE Position               */
#define CPD_STS_EOE_Msk                  (0x1ul << CPD_STS_EOE_Pos)                        /*!< CPD_T::STS: EOE Mask                   */

#define CPD_STS_EOOV_Pos                 (10)                                              /*!< CPD_T::STS: EOOV Position              */
#define CPD_STS_EOOV_Msk                 (0x1ul << CPD_STS_EOOV_Pos)                       /*!< CPD_T::STS: EOOV Mask                  */

#define CPD_STS_EOTHIF_Pos               (11)                                              /*!< CPD_T::STS: EOTHIF Position            */
#define CPD_STS_EOTHIF_Msk               (0x1ul << CPD_STS_EOTHIF_Pos)                     /*!< CPD_T::STS: EOTHIF Mask                */

#define CPD_STS_EOFPTR_Pos               (12)                                              /*!< CPD_T::STS: EOFPTR Position            */
#define CPD_STS_EOFPTR_Msk               (0xful << CPD_STS_EOFPTR_Pos)                     /*!< CPD_T::STS: EOFPTR Mask                */

#define CPD_STS_DIF_Pos                  (16)                                              /*!< CPD_T::STS: DIF Position               */
#define CPD_STS_DIF_Msk                  (0x1ul << CPD_STS_DIF_Pos)                        /*!< CPD_T::STS: DIF Mask                   */

#define CPD_STS_DIE_Pos                  (17)                                              /*!< CPD_T::STS: DIE Position               */
#define CPD_STS_DIE_Msk                  (0x1ul << CPD_STS_DIE_Pos)                        /*!< CPD_T::STS: DIE Mask                   */

#define CPD_STS_DIOV_Pos                 (18)                                              /*!< CPD_T::STS: DIOV Position              */
#define CPD_STS_DIOV_Msk                 (0x1ul << CPD_STS_DIOV_Pos)                       /*!< CPD_T::STS: DIOV Mask                  */

#define CPD_STS_DITHIF_Pos               (19)                                              /*!< CPD_T::STS: DITHIF Position            */
#define CPD_STS_DITHIF_Msk               (0x1ul << CPD_STS_DITHIF_Pos)                     /*!< CPD_T::STS: DITHIF Mask                */

#define CPD_STS_DIFPTR_Pos               (20)                                              /*!< CPD_T::STS: DIFPTR Position            */
#define CPD_STS_DIFPTR_Msk               (0xful << CPD_STS_DIFPTR_Pos)                     /*!< CPD_T::STS: DIFPTR Mask                */

#define CPD_STS_DOF_Pos                  (24)                                              /*!< CPD_T::STS: DOF Position               */
#define CPD_STS_DOF_Msk                  (0x1ul << CPD_STS_DOF_Pos)                        /*!< CPD_T::STS: DOF Mask                   */

#define CPD_STS_DOE_Pos                  (25)                                              /*!< CPD_T::STS: DOE Position               */
#define CPD_STS_DOE_Msk                  (0x1ul << CPD_STS_DOE_Pos)                        /*!< CPD_T::STS: DOE Mask                   */

#define CPD_STS_DOOV_Pos                 (26)                                              /*!< CPD_T::STS: DOOV Position              */
#define CPD_STS_DOOV_Msk                 (0x1ul << CPD_STS_DOOV_Pos)                       /*!< CPD_T::STS: DOOV Mask                  */

#define CPD_STS_DOTHIF_Pos               (27)                                              /*!< CPD_T::STS: DOTHIF Position            */
#define CPD_STS_DOTHIF_Msk               (0x1ul << CPD_STS_DOTHIF_Pos)                     /*!< CPD_T::STS: DOTHIF Mask                */

#define CPD_STS_DOFPTR_Pos               (28)                                              /*!< CPD_T::STS: DOFPTR Position            */
#define CPD_STS_DOFPTR_Msk               (0xful << CPD_STS_DOFPTR_Pos)                     /*!< CPD_T::STS: DOFPTR Mask                */

#define CPD_ENCIN_ENCIN_Pos              (0)                                               /*!< CPD_T::ENCIN: ENCIN Position           */
#define CPD_ENCIN_ENCIN_Msk              (0xfffful << CPD_ENCIN_ENCIN_Pos)                 /*!< CPD_T::ENCIN: ENCIN Mask               */

#define CPD_ENCOUT_ENCOUT_Pos            (0)                                               /*!< CPD_T::ENCOUT: ENCOUT Position         */
#define CPD_ENCOUT_ENCOUT_Msk            (0xfful << CPD_ENCOUT_ENCOUT_Pos)                 /*!< CPD_T::ENCOUT: ENCOUT Mask             */

#define CPD_DECIN_DECIN_Pos              (0)                                               /*!< CPD_T::DECIN: DECIN Position           */
#define CPD_DECIN_DECIN_Msk              (0xfful << CPD_DECIN_DECIN_Pos)                   /*!< CPD_T::DECIN: DECIN Mask               */

#define CPD_DECOUT_DECOUT_Pos            (0)                                               /*!< CPD_T::DECOUT: DECOUT Position         */
#define CPD_DECOUT_DECOUT_Msk            (0xfffful << CPD_DECOUT_DECOUT_Pos)               /*!< CPD_T::DECOUT: DECOUT Mask             */

/**@}*/ /* CPD_CONST */
/**@}*/ /* end of CPD register group */


/*---------------------- Digital to Analog Converter -------------------------*/
/**
    @addtogroup DAC Digital to Analog Converter(DAC)
    Memory Mapped Structure for DAC Controller
@{ */
 
typedef struct
{


/**
 * @var DAC_T::DAT
 * Offset: 0x00  DAC FIFO Data Write Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |FIFO      |FIFO Data Input Register
 * |        |          |DAC contains 32 words (32x32 bit) data buffer for data transmit
 * |        |          |A write to this register pushes data onto the FIFO data buffer and increments the write pointer
 * |        |          |This is the address that CPU/PDMA writes audio data to
 * |        |          |The remaining word number is indicated by FIFOPTR (DAC_FIFOSTS[9:4]).
 * @var DAC_T::CTL0
 * Offset: 0x04  DAC Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |FIFOWIDTH |FIFO Data Width
 * |        |          |This bit field is used to define the bit-width of data word and valid bits in register DAC_DAT.
 * |        |          |00 = The bit-width of data word is 32-bit, valid bits is DAC_DAT[31:0].
 * |        |          |01 = The bit-width of data word is 16-bit, valid bits is DAC_DAT[15:0].
 * |        |          |10 = The bit-width of data word is 8-bit, valid bits is DAC_DAT[7:0].
 * |        |          |11 = The bit-width of data word is 24-bit, valid bits is DAC_DAT[23:0].
 * |[5:4]   |MODESEL   |Data Control in FIFO
 * |        |          |00 = Data is stereo format
 * |        |          |01 = Data is monaural format
 * |        |          |Others = reserved
 * |[7:6]   |FIFOEN    |DAC FIFO enable control
 * |        |          |00 = FIFO disable
 * |        |          |11 = FIFO enable
 * |        |          |Others = reserved
 * |[11]    |THIE      |FIFO Threshold Interrupt
 * |        |          |0 = FIFO threshold interrupt Disabled
 * |        |          |1 = FIFO threshold interrupt Enabled.
 * |[16:12] |TH        |FIFO Threshold Level
 * |        |          |If the valid data count of the FIFO data buffer is less than or equal to TH (DAC_CTL[16:12]) setting, the THIF (DAC_FIFOSTS[2]) will set to 1, else the THIF (DAC_FIFOSTS[2]) will be cleared to 0.
 * |[29:28] |FCLR      |FIFO Clear
 * |        |          |11 = Clear the FIFO.
 * |        |          |Others = Reserved. Do not use.
 * |        |          |Note 1: To clear the FIFO, need to write FCLR (DAC_CTL[29:28]) to 11b, and can read the EMPTY (DAC_FIFOSTS[1]) bit to make sure that the FIFO has been cleared.
 * |        |          |Note 2: This field is auto cleared by hardware.
 * |[30]    |SWRST     |State Machine Software Reset
 * |        |          |0 = State Machine normal operation
 * |        |          |1 = State Machine Reset
 * |[31]    |CLKSET    |Working Clock Selection
 * |        |          |0= the sampling rate is DACCLK/512
 * |        |          |1= the sampling rate is DACCLK/500
 * @var DAC_T::DVOL
 * Offset: 0x08  DAC Digital Volume Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DACLVOL   |DACL Digital Volume Control Register
 * |        |          |0xff = +6dB
 * |        |          |0xfe = +5.5dB
 * |        |          | u25BC
 * |        |          |0xf3 = 0dB
 * |        |          |0xf2= -0.5dBdB
 * |        |          | u25BC
 * |        |          |0x53= -80dB
 * |        |          |0x52= Reserved
 * |        |          | u25BC
 * |        |          |0x01= Reserved
 * |        |          |0x00 = Mute
 * |        |          |Note: Volume per step 0.5dB
 * |[15:8]  |DACRVOL   |DACR Digital Volume Control Register
 * |        |          |0xff = +6dB
 * |        |          |0xfe = +5.5dB
 * |        |          | u25BC
 * |        |          |0xf3 = 0dB
 * |        |          |0xf2= -0.5dBdB
 * |        |          | u25BC
 * |        |          |0x53= -80dB
 * |        |          |0x52= Reserved
 * |        |          | u25BC
 * |        |          |0x01= Reserved
 * |        |          |0x00 = Mute
 * |        |          |Note: Volume per step 0.5dB
 * @var DAC_T::FIFOSTS
 * Offset: 0x0C  DAC FIFO Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FULL      |FIFO Full (Read Only)
 * |        |          |0 = FIFO is not full.
 * |        |          |1 = FIFO is full.
 * |[1]     |EMPTY     |FIFO Empty (Read Only)
 * |        |          |0 = FIFO is not empty.
 * |        |          |1 = FIFO is empty.
 * |[2]     |THIF      |FIFO Threshold Interrupt Status (Read Only)
 * |        |          |0 = The valid data count within the FIFO data buffer is more than the setting value of TH (DAC_CTL[24:20]).
 * |        |          |1 = The valid data count within the FIFO data buffer is less than or equal to the setting value of TH (DAC_CTL[24:20]).
 * |[9:4]   |FIFOPTR   |FIFO Pointer (Read Only)
 * |        |          |The FULL (DAC_FIFOSTS[0]) and FIFOPTR (DAC_FIFOSTS[9:4]) indicates the field that the valid data count within the DAC FIFO buffer.
 * |        |          |The maximum value shown in FIFOPTR is 32
 * |        |          |When the using level of DAC FIFO buffer equal to 32, The FULL (DAC_FIFOSTS[0]) is set to 1.
 * |        |          |The minimum value shown in FIFOPTR is 0
 * |        |          |When the using level of DAC FIFO buffer equal to 0, The EMPTY (DAC_FIFOSTS[1]) is set to 1.
 * |[12]    |FIFOFAIL  |FIFO TEST Failed flag
 * |        |          |1 = memory bist fail, if set BISTEN[0] to u201C1u201D and FIFOEND is u201C1u201D
 * |        |          |0 = memory bist pass, if set BISTEN[0] to u201C1u201D and FIFOEND is u201C1u201D
 * |[13]    |FIFOEND   |FIFO TEST Finish flag
 * |        |          |1 = finished, if set BISTEN[0] to u201C1u201D
 * |        |          |0 = is not finishing, if set BISTEN[0] to u201C1u201D
 * |[14]    |RAMFAIL   |RAM TEST Failed Flag(internal use)
 * |        |          |1 = memory bist fail, if set BISTEN[1] to u201C1u201D and RAMEND is u201C1u201D
 * |        |          |0 = memory bist pass, if set BISTEN[1] to u201C1u201D and RAMEND is u201C1u201D 
 * |[15]    |RAMEND    |RAM TEST Finish Flag(internal use)
 * |        |          |1 = finish, if set BISTEN[1] to u201C1u201D
 * |        |          |0 = is not finishing, if set BISTEN[1] to u201C1u201D
 * |[28]    |FIFOTEST  |Enable FIFO test mode
 * |        |          |0 = Disable FIFO test, FIFO only write.
 * |        |          |1 = Enable FIFO test, FIFO can read and write.
 * |[30:29] |BISTEN    |BIST Enable(internal use)
 * |        |          |Bit[30]: Interpolator RAM BIST Mode control
 * |        |          |Bit[29]: FIFO BIST Mode control
 * |        |          |0 = Disable BIST testing.
 * |        |          |1 = Enable BIST testing.
 * |        |          |Note: FIFO can be testing by Cortex-M0
 * @var DAC_T::PDMACTL
 * Offset: 0x10  DAC PDMA Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDMAEN    |PDMA Transfer Enable Bit
 * |        |          |0 = PDMA data transfer Disabled.
 * |        |          |1 = PDMA data transfer Enabled.
 * @var DAC_T::HPVOL
 * Offset: 0x14  DAC Headphone Volume Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:0]   |HPLVOL    |HP Output Left Channel Volume Control Register
 * |        |          |0x3f = +6dB
 * |        |          |0x3e = +5dB
 * |        |          | u25BC
 * |        |          |0x39= 0dB
 * |        |          |0x38= -1dB
 * |        |          | u25BC
 * |        |          |0x01 = -56dB
 * |        |          |0x00 = -57dB
 * |        |          |Note: Volume per step 0.5dB
 * |[15]    |HPLM      |HP Output Left Channel Mute Control Register
 * |        |          |0 = Unmute
 * |        |          |1 = Mute
 * |[21:16] |HPRVOL    |HP Output Right Channel Volume Control Register
 * |        |          |0x3f = +6dB
 * |        |          |0x3e = +5dB
 * |        |          | u25BC
 * |        |          |0x39= 0dB
 * |        |          |0x38= -1dB
 * |        |          | u25BC
 * |        |          |0x01 = -56dB
 * |        |          |0x00 = -57dB
 * |        |          |Note: Volume per step 0.5dB
 * |[31]    |HPRM      |HP Output Right Channel Mute Control Register
 * |        |          |0 = Unmute
 * |        |          |1 = Mute
 * @var DAC_T::ZOHDIV
 * Offset: 0x18  DAC Zero Order Hold Division Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ZOHDIV    |Zero Order Hold, Down-sampling Divisor
 * |        |          |The input sample rate of the DPWM is set by DAC_CLK frequency and the divisor set in this register by the following formula:
 * |        |          |If CLKSET (DAC_CTL[31]) is 0, K = 128.
 * |        |          |If CLKSET (DAC_CTL[31]) is 1, K = 125.
 * |        |          |ZOHDIV = F_DAC_CLK / (Fs * K ).
 * |        |          |Where F_DAC_CLK is the frequency of DAC module working clock (DAC_CLK) and Fs is sampling rate.
 * |        |          |Note: The value of ZOHDIV must be >= 4
 * @var DAC_T::CTL1
 * Offset: 0x200  DAC Control Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |DACENL    |SDMOD enable control for left channel
 * |[1]     |DACENR    |SDMOD enable control for right channel
 * |[6:2]   |SDDITHER  |SDMOD dither control
 * |        |          |Number of bits of dithering on SD Modulator . Each level increments dithering by 1 bit
 * |        |          |0000 = No Dithering
 * |        |          |0001 = 1
 * |        |          | .
 * |        |          | ~
 * |        |          | .
 * |        |          |1111 = 15
 * |[10:7]  |DEMDITHER |DEM dither control
 * |        |          |Set Probability of DEM Dithering
 * |        |          |Set probability of first order DEM dithering. Each level increments probability by 1/16
 * |        |          |0000 = No dithering
 * |        |          |0001= 1/16
 * |        |          |0010 = 2/16
 * |        |          | .
 * |        |          | ~
 * |        |          | .
 * |        |          |1111 = 15/16
 * |[11]    |OSR100    |Reads u20181u2019 when OSR = 100x
 * |[12]    |MIPS500   |Indicates u20181u2019 when MCLK_SRC/FS = 500
 * |[13]    |DACOSR32  |DAC Oversample Rate 32 Selection
 * |[14]    |DACOSR128 |DAC Oversample Rate 128 Selection
 * |[15]    |DACOSR256 |DAC Oversample Rate 256 Selection
 * |[17]    |DISDEM    |Disable DEM (dynamic element matching)
 * |        |          |0 = Enable
 * |        |          |1 = Disable
 * @var DAC_T::CTL2
 * Offset: 0x204  DAC Control Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CICIADJ   |Digital filter control ???
 * |[16]    |CICCLPOFF |Digital filter control ???
 * |[19:17] |CICGADJ   |DAC Output Fine Tuning
 * |[20]    |COFFSEL   |Digital filter control ???
 * |[21]    |CLKSYNC   |Keep dault
 * |[31:24] |SDSEL     |Digital filter control ???
 * @var DAC_T::CTL12
 * Offset: 0x208  DAC Control Register 12 (All Reserved)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2]     |ACLKSEL   |Analog DAC clock source selection
 * |        |          |0 = provide engine clock for analog DAC
 * |        |          |1 = provide data clock for analog DAC
 * @var DAC_T::CTL3
 * Offset: 0x20C  DAC Control Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SMCTL     |Soft mute control
 * |        |          |0 = Gradually increase DAC volume to volume register setting
 * |        |          |1 = Gradually decrease DAC volume to zero
 * |[2:1]   |UNMUTECTL |Power-up soft unmute control
 * |        |          |x0 = No soft digital unmute on PWRUPEN and MUTEB events
 * |        |          |01 = 512 MCLK per step soft unmute
 * |        |          |11 = 32 MCLK per step soft unmute
 * |[3]     |DACENSM   |DACEN Soft Mute enable
 * |        |          |DAC volume ramping up of a channel on a rising edge of when it turned on.
 * |        |          |0 = Disable
 * |        |          |1 = Enable
 * |[4]     |ZCEN      |DAC zero cross enable
 * |        |          |0 = Disable
 * |        |          |1 = Enable
 * @var DAC_T::ANA0
 * Offset: 0x300  DAC Analog Block Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:0]   |BV1P5     |HP output Volume Control
 * |        |          |0u2019h = -57dB
 * |        |          | ~
 * |        |          |39u2019h = 0dB
 * |        |          |~
 * |        |          |3Fu2019h = +6dB
 * |        |          |1dB/step
 * |        |          |Default = 0u2019h( 39u2019h = 
 * |[7:6]   |CAPV1P5   |Bypass cap setting
 * |        |          |00 = 0C(default)
 * |        |          |01 = 1C
 * |        |          |10 = 2C
 * |        |          |11 = 3C
 * |[10:8]  |CKDLYV1P5 |Delay clock choice for DAC
 * |        |          |000 = clk_3(default)
 * |        |          |001 = clk_4
 * |        |          |010 = clk_5
 * |        |          |011 = clk_6
 * |        |          |100 = clk_7
 * |        |          |101 = clk_0
 * |        |          |110 = clk_1
 * |        |          |111 = clk_2
 * |[12:11] |IBADJV1P5 |BIAS current adjust control
 * |        |          |00 = 20uA(default)
 * |        |          |01 = 25uA
 * |        |          |10 = 17uA
 * |        |          |11 = 10.8uA
 * |[14:13] |VREFSEL   |DAC Vref select control
 * |        |          |00 = vccx(default)
 * |        |          |01 = 2.2V(@VCC=3.3V)
 * |        |          |10 = 2.4V(@VCC=3.3V)
 * |        |          |11 = 2.6V(@VCC=3.3V)
 * |[15]    |CLKINV    |Clock input inverse
 * |        |          |0 = not inverse(default)
 * |        |          |1 = inverse
 * @var DAC_T::ANA1
 * Offset: 0x304  DAC Analog Block Control Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ENCLK1    |Left channel DAC clock enable control
 * |        |          |0 = Disable(default)
 * |        |          |1 = Enable 
 * |[1]     |ENCLK2    |Right channel DAC clock enable control
 * |        |          |0 = Disable(default)
 * |        |          |1 = Enable 
 * |[2]     |ENDAC1    |Left channel DAC enable control
 * |        |          |0 = Disable(default)
 * |        |          |1 = Enable 
 * |[3]     |ENDAC2    |Right channel DAC enable control
 * |        |          |0 = Disable(default)
 * |        |          |1 = Enable
 * |[5:4]   |ENHP1     |Left channel headphone driver block enable control
 * |        |          |00 = Disable(default)
 * |[7:6]   |ENHP2     |Right channel headphone driver block enable control
 * |        |          |00 = Disable(default)
 * |[8]     |PDFLITSM1 |Left channel smooth filter block power down control
 * |        |          |0 = power on
 * |        |          |1 = power down(default)
 * |[9]     |PDFLITSM2 |Right channel smooth filter block power down control
 * |        |          |0 = power on
 * |        |          |1 = power down(default)
 * |[10]    |PDVBUF1   |Left channel VMID buffer block power down control
 * |        |          |0 = power on
 * |        |          |1 = power off(default)
 * |[11]    |PDVBUF2   |Right channel VMID buffer block power down control
 * |        |          |0 = power on
 * |        |          |1 = power off(default)
 * |[12]    |PDIBGEN   |IBGEN block power down control
 * |        |          |0 = power on
 * |        |          |1 = power off(default)
 * |[14:13] |PDBIAS    |11 = (default)
 * |[16:15] |PDBIAS2   |11 = (default)
 * |[17]    |PDBDAC1   |Left DAC power down control
 * |        |          |0 = power off(default)
 * |        |          |1= power on
 * |[18]    |PDBDAC2   |Right DAC power down control
 * |        |          |0 = power off(default)
 * |        |          |1= power on 
 * |[19]    |VOLEN1    |Left volume enable control
 * |        |          |0 = Disable(default)(Connect to VMID)
 * |        |          |1 = Enable (Connect to signal)
 * |[20]    |VOLEN2    |Right volume enable control
 * |        |          |0 = Disable(default)(Connect to VMID)
 * |        |          |1 = Enable (Connect to signal)
 * |[21]    |VOLMUTE   |Volume mute control
 * |        |          |0 = mute(default)
 * |        |          |1 = unmute
 * |[22]    |VROI      |VROI Control(for pop control)
 * |        |          |0 = (default)
 * |[23]    |TEST      |Test mode enable
 * |        |          |0 = Disable (default)
 * |        |          |1= Enable
 * |[25:24] |TESTDACIN |DAC input while in active mode
 * |        |          |00 = (default)
 */
    __IO uint32_t DAT;                   /*!< [0x0000] DAC FIFO Data Write Register                                     */
    __IO uint32_t CTL0;                  /*!< [0x0004] DAC Control Register 0                                           */
    __IO uint32_t DVOL;                  /*!< [0x0008] DAC Digital Volume Control Register                              */
    __IO uint32_t FIFOSTS;               /*!< [0x000c] DAC FIFO Status Register                                         */
    __IO uint32_t PDMACTL;               /*!< [0x0010] DAC PDMA Control Register                                        */
    __I  uint32_t RESERVE0[123];
    __IO uint32_t CTL1;                  /*!< [0x0200] DAC Control Register 1                                           */
    __IO uint32_t CTL2;                  /*!< [0x0204] DAC Control Register 2                                           */
    __IO uint32_t RESERVE1[1];        
    __IO uint32_t CTL3;                  /*!< [0x020c] DAC Control Register 3                                           */
    __I  uint32_t RESERVE2[60];
    __IO uint32_t ANA0;                  /*!< [0x0300] DAC Analog Block Control Register 0                              */
    __IO uint32_t ANA1;                  /*!< [0x0304] DAC Analog Block Control Register 1                              */

} DAC_T;

/**
    @addtogroup DAC_CONST DAC Bit Field Definition
    Constant Definitions for DAC Controller
@{ */

#define DAC_DAT_FIFO_Pos                 (0)                                               /*!< DAC_T::DAT: FIFO Position              */
#define DAC_DAT_FIFO_Msk                 (0xfffffffful << DAC_DAT_FIFO_Pos)                /*!< DAC_T::DAT: FIFO Mask                  */

#define DAC_CTL0_FIFOWIDTH_Pos           (0)                                               /*!< DAC_T::CTL0: FIFOWIDTH Position        */
#define DAC_CTL0_FIFOWIDTH_Msk           (0x3ul << DAC_CTL0_FIFOWIDTH_Pos)                 /*!< DAC_T::CTL0: FIFOWIDTH Mask            */

#define DAC_CTL0_MODESEL_Pos             (4)                                               /*!< DAC_T::CTL0: MODESEL Position          */
#define DAC_CTL0_MODESEL_Msk             (0x3ul << DAC_CTL0_MODESEL_Pos)                   /*!< DAC_T::CTL0: MODESEL Mask              */

#define DAC_CTL0_FIFOEN_Pos              (6)                                               /*!< DAC_T::CTL0: FIFOEN Position           */
#define DAC_CTL0_FIFOEN_Msk              (0x3ul << DAC_CTL0_FIFOEN_Pos)                    /*!< DAC_T::CTL0: FIFOEN Mask               */

#define DAC_CTL0_THIE_Pos                (11)                                              /*!< DAC_T::CTL0: THIE Position             */
#define DAC_CTL0_THIE_Msk                (0x1ul << DAC_CTL0_THIE_Pos)                      /*!< DAC_T::CTL0: THIE Mask                 */

#define DAC_CTL0_TH_Pos                  (12)                                              /*!< DAC_T::CTL0: TH Position               */
#define DAC_CTL0_TH_Msk                  (0x1ful << DAC_CTL0_TH_Pos)                       /*!< DAC_T::CTL0: TH Mask                   */

#define DAC_CTL0_FCLR_Pos                (28)                                              /*!< DAC_T::CTL0: FCLR Position             */
#define DAC_CTL0_FCLR_Msk                (0x3ul << DAC_CTL0_FCLR_Pos)                      /*!< DAC_T::CTL0: FCLR Mask                 */

#define DAC_CTL0_SWRST_Pos               (30)                                              /*!< DAC_T::CTL0: SWRST Position            */
#define DAC_CTL0_SWRST_Msk               (0x1ul << DAC_CTL0_SWRST_Pos)                     /*!< DAC_T::CTL0: SWRST Mask                */

#define DAC_CTL0_CLKSET_Pos              (31)                                              /*!< DAC_T::CTL0: CLKSET Position           */
#define DAC_CTL0_CLKSET_Msk              (0x1ul << DAC_CTL0_CLKSET_Pos)                    /*!< DAC_T::CTL0: CLKSET Mask               */

#define DAC_DVOL_DACLVOL_Pos             (0)                                               /*!< DAC_T::DVOL: DACLVOL Position          */
#define DAC_DVOL_DACLVOL_Msk             (0xfful << DAC_DVOL_DACLVOL_Pos)                  /*!< DAC_T::DVOL: DACLVOL Mask              */

#define DAC_DVOL_DACRVOL_Pos             (8)                                               /*!< DAC_T::DVOL: DACRVOL Position          */
#define DAC_DVOL_DACRVOL_Msk             (0xfful << DAC_DVOL_DACRVOL_Pos)                  /*!< DAC_T::DVOL: DACRVOL Mask              */

#define DAC_FIFOSTS_FULL_Pos             (0)                                               /*!< DAC_T::FIFOSTS: FULL Position          */
#define DAC_FIFOSTS_FULL_Msk             (0x1ul << DAC_FIFOSTS_FULL_Pos)                   /*!< DAC_T::FIFOSTS: FULL Mask              */

#define DAC_FIFOSTS_EMPTY_Pos            (1)                                               /*!< DAC_T::FIFOSTS: EMPTY Position         */
#define DAC_FIFOSTS_EMPTY_Msk            (0x1ul << DAC_FIFOSTS_EMPTY_Pos)                  /*!< DAC_T::FIFOSTS: EMPTY Mask             */

#define DAC_FIFOSTS_THIF_Pos             (2)                                               /*!< DAC_T::FIFOSTS: THIF Position          */
#define DAC_FIFOSTS_THIF_Msk             (0x1ul << DAC_FIFOSTS_THIF_Pos)                   /*!< DAC_T::FIFOSTS: THIF Mask              */

#define DAC_FIFOSTS_FIFOPTR_Pos          (4)                                               /*!< DAC_T::FIFOSTS: FIFOPTR Position       */
#define DAC_FIFOSTS_FIFOPTR_Msk          (0x3ful << DAC_FIFOSTS_FIFOPTR_Pos)               /*!< DAC_T::FIFOSTS: FIFOPTR Mask           */

#define DAC_PDMACTL_PDMAEN_Pos           (0)                                               /*!< DAC_T::PDMACTL: PDMAEN Position        */
#define DAC_PDMACTL_PDMAEN_Msk           (0x1ul << DAC_PDMACTL_PDMAEN_Pos)                 /*!< DAC_T::PDMACTL: PDMAEN Mask            */

#define DAC_CTL1_DACENL_Pos              (0)                                               /*!< DAC_T::CTL1: DACENL Position           */
#define DAC_CTL1_DACENL_Msk              (0x1ul << DAC_CTL1_DACENL_Pos)                    /*!< DAC_T::CTL1: DACENL Mask               */

#define DAC_CTL1_DACENR_Pos              (1)                                               /*!< DAC_T::CTL1: DACENR Position           */
#define DAC_CTL1_DACENR_Msk              (0x1ul << DAC_CTL1_DACENR_Pos)                    /*!< DAC_T::CTL1: DACENR Mask               */

#define DAC_CTL1_SDDITHER_Pos            (2)                                               /*!< DAC_T::CTL1: SDDITHER Position         */
#define DAC_CTL1_SDDITHER_Msk            (0x1ful << DAC_CTL1_SDDITHER_Pos)                 /*!< DAC_T::CTL1: SDDITHER Mask             */

#define DAC_CTL1_DEMDITHER_Pos           (7)                                               /*!< DAC_T::CTL1: DEMDITHER Position        */
#define DAC_CTL1_DEMDITHER_Msk           (0xful << DAC_CTL1_DEMDITHER_Pos)                 /*!< DAC_T::CTL1: DEMDITHER Mask            */

#define DAC_CTL1_OSR100_Pos              (11)                                              /*!< DAC_T::CTL1: OSR100 Position           */
#define DAC_CTL1_OSR100_Msk              (0x1ul << DAC_CTL1_OSR100_Pos)                    /*!< DAC_T::CTL1: OSR100 Mask               */

#define DAC_CTL1_MIPS500_Pos             (12)                                              /*!< DAC_T::CTL1: MIPS500 Position          */
#define DAC_CTL1_MIPS500_Msk             (0x1ul << DAC_CTL1_MIPS500_Pos)                   /*!< DAC_T::CTL1: MIPS500 Mask              */

#define DAC_CTL1_OSRDIV_Pos              (13)                                              /*!< DAC_T::CTL1: OSRDIV Position           */
#define DAC_CTL1_OSRDIV_Msk              (0x7ul << DAC_CTL1_OSRDIV_Pos)                    /*!< DAC_T::CTL1: OSRDIV Mask               */

#define DAC_CTL1_DISDEM_Pos              (17)                                              /*!< DAC_T::CTL1: DISDEM Position           */
#define DAC_CTL1_DISDEM_Msk              (0x1ul << DAC_CTL1_DISDEM_Pos)                    /*!< DAC_T::CTL1: DISDEM Mask               */

#define DAC_CTL3_SMCTL_Pos               (0)                                               /*!< DAC_T::CTL3: SMCTL Position            */
#define DAC_CTL3_SMCTL_Msk               (0x1ul << DAC_CTL3_SMCTL_Pos)                     /*!< DAC_T::CTL3: SMCTL Mask                */

#define DAC_CTL3_UNMUTECTL_Pos           (2)                                               /*!< DAC_T::CTL3: UNMUTECTL Position        */
#define DAC_CTL3_UNMUTECTL_Msk           (0x1ul << DAC_CTL3_UNMUTECTL_Pos)                 /*!< DAC_T::CTL3: UNMUTECTL Mask            */

#define DAC_CTL3_ZCEN_Pos                (4)                                               /*!< DAC_T::CTL3: ZCEN Position             */
#define DAC_CTL3_ZCEN_Msk                (0x1ul << DAC_CTL3_ZCEN_Pos)                      /*!< DAC_T::CTL3: ZCEN Mask                 */

#define DAC_ANA0_BV1P5_Pos               (0)                                               /*!< DAC_T::ANA0: BV1P5 Position            */
#define DAC_ANA0_BV1P5_Msk               (0x3ful << DAC_ANA0_BV1P5_Pos)                    /*!< DAC_T::ANA0: BV1P5 Mask                */

#define DAC_ANA0_CAPV1P5_Pos             (6)                                               /*!< DAC_T::ANA0: CAPV1P5 Position          */
#define DAC_ANA0_CAPV1P5_Msk             (0x3ul << DAC_ANA0_CAPV1P5_Pos)                   /*!< DAC_T::ANA0: CAPV1P5 Mask              */

#define DAC_ANA0_CKDLYV1P5_Pos           (8)                                               /*!< DAC_T::ANA0: CKDLYV1P5 Position        */
#define DAC_ANA0_CKDLYV1P5_Msk           (0x7ul << DAC_ANA0_CKDLYV1P5_Pos)                 /*!< DAC_T::ANA0: CKDLYV1P5 Mask            */

#define DAC_ANA0_IBADJV1P5_Pos           (11)                                              /*!< DAC_T::ANA0: IBADJV1P5 Position        */
#define DAC_ANA0_IBADJV1P5_Msk           (0x3ul << DAC_ANA0_IBADJV1P5_Pos)                 /*!< DAC_T::ANA0: IBADJV1P5 Mask            */

#define DAC_ANA0_VREFSEL_Pos             (13)                                              /*!< DAC_T::ANA0: VREFSEL Position          */
#define DAC_ANA0_VREFSEL_Msk             (0x3ul << DAC_ANA0_VREFSEL_Pos)                   /*!< DAC_T::ANA0: VREFSEL Mask              */

#define DAC_ANA0_CLKINV_Pos              (15)                                              /*!< DAC_T::ANA0: CLKINV Position           */
#define DAC_ANA0_CLKINV_Msk              (0x1ul << DAC_ANA0_CLKINV_Pos)                    /*!< DAC_T::ANA0: CLKINV Mask               */

#define DAC_ANA1_ENCLK1_Pos              (0)                                               /*!< DAC_T::ANA1: ENCLK1 Position           */
#define DAC_ANA1_ENCLK1_Msk              (0x1ul << DAC_ANA1_ENCLK1_Pos)                    /*!< DAC_T::ANA1: ENCLK1 Mask               */

#define DAC_ANA1_ENCLK2_Pos              (1)                                               /*!< DAC_T::ANA1: ENCLK2 Position           */
#define DAC_ANA1_ENCLK2_Msk              (0x1ul << DAC_ANA1_ENCLK2_Pos)                    /*!< DAC_T::ANA1: ENCLK2 Mask               */

#define DAC_ANA1_ENDAC1_Pos              (2)                                               /*!< DAC_T::ANA1: ENDAC1 Position           */
#define DAC_ANA1_ENDAC1_Msk              (0x1ul << DAC_ANA1_ENDAC1_Pos)                    /*!< DAC_T::ANA1: ENDAC1 Mask               */

#define DAC_ANA1_ENDAC2_Pos              (3)                                               /*!< DAC_T::ANA1: ENDAC2 Position           */
#define DAC_ANA1_ENDAC2_Msk              (0x1ul << DAC_ANA1_ENDAC2_Pos)                    /*!< DAC_T::ANA1: ENDAC2 Mask               */

#define DAC_ANA1_ENHP1_Pos               (4)                                               /*!< DAC_T::ANA1: ENHP1 Position            */
#define DAC_ANA1_ENHP1_Msk               (0x3ul << DAC_ANA1_ENHP1_Pos)                     /*!< DAC_T::ANA1: ENHP1 Mask                */

#define DAC_ANA1_ENHP2_Pos               (6)                                               /*!< DAC_T::ANA1: ENHP2 Position            */
#define DAC_ANA1_ENHP2_Msk               (0x3ul << DAC_ANA1_ENHP2_Pos)                     /*!< DAC_T::ANA1: ENHP2 Mask                */

#define DAC_ANA1_PDFLITSM1_Pos           (8)                                               /*!< DAC_T::ANA1: PDFLITSM1 Position        */
#define DAC_ANA1_PDFLITSM1_Msk           (0x1ul << DAC_ANA1_PDFLITSM1_Pos)                 /*!< DAC_T::ANA1: PDFLITSM1 Mask            */

#define DAC_ANA1_PDFLITSM2_Pos           (9)                                               /*!< DAC_T::ANA1: PDFLITSM2 Position        */
#define DAC_ANA1_PDFLITSM2_Msk           (0x1ul << DAC_ANA1_PDFLITSM2_Pos)                 /*!< DAC_T::ANA1: PDFLITSM2 Mask            */

#define DAC_ANA1_PDVBUF1_Pos             (10)                                              /*!< DAC_T::ANA1: PDVBUF1 Position          */
#define DAC_ANA1_PDVBUF1_Msk             (0x1ul << DAC_ANA1_PDVBUF1_Pos)                   /*!< DAC_T::ANA1: PDVBUF1 Mask              */

#define DAC_ANA1_PDVBUF2_Pos             (11)                                              /*!< DAC_T::ANA1: PDVBUF2 Position          */
#define DAC_ANA1_PDVBUF2_Msk             (0x1ul << DAC_ANA1_PDVBUF2_Pos)                   /*!< DAC_T::ANA1: PDVBUF2 Mask              */

#define DAC_ANA1_PDIBGEN_Pos             (12)                                              /*!< DAC_T::ANA1: PDIBGEN Position          */
#define DAC_ANA1_PDIBGEN_Msk             (0x1ul << DAC_ANA1_PDIBGEN_Pos)                   /*!< DAC_T::ANA1: PDIBGEN Mask              */

#define DAC_ANA1_PDBIAS_Pos              (13)                                              /*!< DAC_T::ANA1: PDBIAS Position           */
#define DAC_ANA1_PDBIAS_Msk              (0x3ul << DAC_ANA1_PDBIAS_Pos)                    /*!< DAC_T::ANA1: PDBIAS Mask               */

#define DAC_ANA1_PDBIAS2_Pos             (15)                                              /*!< DAC_T::ANA1: PDBIAS2 Position          */
#define DAC_ANA1_PDBIAS2_Msk             (0x3ul << DAC_ANA1_PDBIAS2_Pos)                   /*!< DAC_T::ANA1: PDBIAS2 Mask              */

#define DAC_ANA1_PDBDAC1_Pos             (17)                                              /*!< DAC_T::ANA1: PDBDAC1 Position          */
#define DAC_ANA1_PDBDAC1_Msk             (0x1ul << DAC_ANA1_PDBDAC1_Pos)                   /*!< DAC_T::ANA1: PDBDAC1 Mask              */

#define DAC_ANA1_PDBDAC2_Pos             (18)                                              /*!< DAC_T::ANA1: PDBDAC2 Position          */
#define DAC_ANA1_PDBDAC2_Msk             (0x1ul << DAC_ANA1_PDBDAC2_Pos)                   /*!< DAC_T::ANA1: PDBDAC2 Mask              */

#define DAC_ANA1_VOLEN1_Pos              (19)                                              /*!< DAC_T::ANA1: VOLEN1 Position           */
#define DAC_ANA1_VOLEN1_Msk              (0x1ul << DAC_ANA1_VOLEN1_Pos)                    /*!< DAC_T::ANA1: VOLEN1 Mask               */

#define DAC_ANA1_VOLEN2_Pos              (20)                                              /*!< DAC_T::ANA1: VOLEN2 Position           */
#define DAC_ANA1_VOLEN2_Msk              (0x1ul << DAC_ANA1_VOLEN2_Pos)                    /*!< DAC_T::ANA1: VOLEN2 Mask               */

#define DAC_ANA1_VOLMUTE_Pos             (21)                                              /*!< DAC_T::ANA1: VOLMUTE Position          */
#define DAC_ANA1_VOLMUTE_Msk             (0x1ul << DAC_ANA1_VOLMUTE_Pos)                   /*!< DAC_T::ANA1: VOLMUTE Mask              */

#define DAC_ANA1_VROI_Pos                (22)                                              /*!< DAC_T::ANA1: VROI Position             */
#define DAC_ANA1_VROI_Msk                (0x1ul << DAC_ANA1_VROI_Pos)                      /*!< DAC_T::ANA1: VROI Mask                 */

#define DAC_ANA1_TEST_Pos                (23)                                              /*!< DAC_T::ANA1: TEST Position             */
#define DAC_ANA1_TEST_Msk                (0x1ul << DAC_ANA1_TEST_Pos)                      /*!< DAC_T::ANA1: TEST Mask                 */

#define DAC_ANA1_TESTDACIN_Pos           (24)                                              /*!< DAC_T::ANA1: TESTDACIN Position        */
#define DAC_ANA1_TESTDACIN_Msk           (0x3ul << DAC_ANA1_TESTDACIN_Pos)                 /*!< DAC_T::ANA1: TESTDACIN Mask            */

/**@}*/ /* DAC_CONST */
/**@}*/ /* end of DAC register group */


/*---------------------- Flash Memory Controller -------------------------*/
/**
    @addtogroup FMC Flash Memory Controller(FMC)
    Memory Mapped Structure for FMC Controller
@{ */
 
typedef struct
{


/**
 * @var FMC_T::ISPCTL
 * Offset: 0x00  ISP Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ISPEN     |ISP Enable
 * |        |          |0 = Disable ISP function
 * |        |          |1 = Enable ISP function
 * |[1]     |BS        |Boot Select
 * |        |          |0: APROM
 * |        |          |1: LDROM
 * |        |          |This bit functions as MCU boot status flag, which can be used to check where MCU booted from
 * |        |          |This bit is initialized after power-on reset with the inverse of CBS in Config0; It is not reset for any other reset event.
 * |[3]     |APUWEN    |APU Write Enable
 * |        |          |0 = APROM canu2019t write itself. ISPFF with u201C1u201D
 * |        |          |1 = APROM write to itself. 
 * |[4]     |CFGUEN    |CONFIG Update Enable
 * |        |          |0 = Disable
 * |        |          |1 = Enable
 * |        |          |When enabled, ISP functions can access the CONFIG address space and modify device configuration area. 
 * |[5]     |LDUEN     |LDROM Update Enable
 * |        |          |LDROM update enable bit.
 * |        |          |0 = LDROM cannot be updated
 * |        |          |1 = LDROM can be updated when the MCU runs in APROM.
 * |[6]     |ISPFF     |ISP Fail Flag
 * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
 * |        |          |(1) APROM writes to itself.
 * |        |          |(2) LDROM writes to itself.
 * |        |          |(3) Destination address is illegal, such as over an available range.
 * |        |          |(4) BOD event happen
 * |        |          |Write 1 to clear.
 * |[18:16] |WAIT_CFG  |Flash Access Wait State Configuration
 * |        |          |0x3= Zero wait states. HCLK < 24 MHz
 * |        |          |0x2= One wait states. HCLK <= 50 MHz
 * |        |          |0x1= Two wait state.
 * |        |          |0x0= Three wait state.
 * |        |          |Before changing WAIT_CFG, ensure HCLK speed is < 25 MHz.
 * |[21]    |CACHE_DIS |Cache Disable
 * |        |          |When set to 1, caching of flash memory reads is disabled.
 * @var FMC_T::ISPADR
 * Offset: 0x04  ISP Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ISPADR    |ISP Address Register
 * |        |          |This is the memory address register that a subsequent ISP command will access
 * |        |          |ISP operation are carried out on 32bit words only, consequently ISPARD [1:0] must be 00b for correct ISP operation
 * @var FMC_T::ISPDAT
 * Offset: 0x08  ISP Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ISPDAT    |ISP Data Register
 * |        |          |Write data to this register before an ISP program operation.
 * |        |          |Read data from this register after an ISP read operation
 * @var FMC_T::ISPCMD
 * Offset: 0x0C  ISP Command Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:0]   |ISPCMD    |ISP Command
 * |        |          |Operation   Mode
 * |        |          |ISPCMD
 * |        |          |Standby
 * |        |          |0x3X
 * |        |          |Read
 * |        |          |0x00
 * |        |          |Program
 * |        |          |0x21
 * |        |          |Page Erase
 * |        |          |0x22
 * |        |          |Read CID
 * |        |          |0x0B
 * |        |          |Read DID
 * |        |          |0x0C
 * |        |          |Read UID
 * |        |          |0x04
 * @var FMC_T::ISPTRG
 * Offset: 0x10  ISP Trigger Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ISPGO     |ISP Start Trigger
 * |        |          |Write 1 to start ISP operation
 * |        |          |This will be cleared to 0 by hardware automatically when ISP operation is finished.
 * |        |          |0 = ISP operation is finished
 * |        |          |1 = ISP is on going
 * |        |          |After triggering an ISP function M0 instruction pipeline should be flushed with a ISB instruction to guarantee data integrity.
 * |        |          |This is a protected register, user must first follow the unlock sequence see Register Lock Control Register (SYS_REGLCTL)) to gain access.
 * @var FMC_T::DFBADR
 * Offset: 0x14  Data Flash Base Address
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |DFBA      |Data Flash Base Address
 * |        |          |This register reports the data flash starting address. It is a read only register.
 * |        |          |Data flash size is defined by user configuration, register content is loaded from Config1 when chip is reset.
 * @var FMC_T::ICPCON
 * Offset: 0x1C  ICP Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ICP_EN    |ICP control enable
 * |        |          |0 = disable
 * |        |          |1 = enable 
 * @var FMC_T::ICPRMP
 * Offset: 0x20  ICP ROM Map Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |ICPRMP    |ICP ROM MAP control enable
 * |        |          |Write 24u2019h716a40 to ICPRMP, force ICPRMP_EN =u201C1u201D return to bit0.
 * |        |          |When write ICP_EN to u201C0u201D, clear ICPRMP_EN to u201C0u201D
 * |        |          |If ICP_EN is u201C1u201D and ICPRMP_EN is u201C1u201D, ISP can access MAP
 * @var FMC_T::RMPRD
 * Offset: 0x24  MAP READ Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RMPRD_EN  |ROM MAP RD control enable
 * |        |          |0 = disable
 * |        |          |1 = enable ISP access map memory
 * |        |          |ICP_EN is u201C1u201D and ICPRMP_EN is u201C1u201D, ISP can access MAP
 */
    __IO uint32_t ISPCTL;                /*!< [0x0000] ISP Control Register                                             */
    __IO uint32_t ISPADDR;               /*!< [0x0004] ISP Address Register                                             */
    __IO uint32_t ISPDAT;                /*!< [0x0008] ISP Data Register                                                */
    __IO uint32_t ISPCMD;                /*!< [0x000c] ISP Command Register                                             */
    __IO uint32_t ISPTRG;                /*!< [0x0010] ISP Trigger Control Register                                     */
    __I  uint32_t DFBADR;                /*!< [0x0014] Data Flash Base Address                                          */

} FMC_T;

/**
    @addtogroup FMC_CONST FMC Bit Field Definition
    Constant Definitions for FMC Controller
@{ */

#define FMC_ISPCTL_ISPEN_Pos             (0)                                               /*!< FMC_T::ISPCTL: ISPEN Position          */
#define FMC_ISPCTL_ISPEN_Msk             (0x1ul << FMC_ISPCTL_ISPEN_Pos)                   /*!< FMC_T::ISPCTL: ISPEN Mask              */

#define FMC_ISPCTL_BS_Pos                (1)                                               /*!< FMC_T::ISPCTL: BS Position             */
#define FMC_ISPCTL_BS_Msk                (0x1ul << FMC_ISPCTL_BS_Pos)                      /*!< FMC_T::ISPCTL: BS Mask                 */

#define FMC_ISPCTL_APUWEN_Pos            (3)                                               /*!< FMC_T::ISPCTL: APUWEN Position         */
#define FMC_ISPCTL_APUWEN_Msk            (0x1ul << FMC_ISPCTL_APUWEN_Pos)                  /*!< FMC_T::ISPCTL: APUWEN Mask             */

#define FMC_ISPCTL_CFGUEN_Pos            (4)                                               /*!< FMC_T::ISPCTL: CFGUEN Position         */
#define FMC_ISPCTL_CFGUEN_Msk            (0x1ul << FMC_ISPCTL_CFGUEN_Pos)                  /*!< FMC_T::ISPCTL: CFGUEN Mask             */

#define FMC_ISPCTL_LDUEN_Pos             (5)                                               /*!< FMC_T::ISPCTL: LDUEN Position          */
#define FMC_ISPCTL_LDUEN_Msk             (0x1ul << FMC_ISPCTL_LDUEN_Pos)                   /*!< FMC_T::ISPCTL: LDUEN Mask              */

#define FMC_ISPCTL_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPCTL: ISPFF Position          */
#define FMC_ISPCTL_ISPFF_Msk             (0x1ul << FMC_ISPCTL_ISPFF_Pos)                   /*!< FMC_T::ISPCTL: ISPFF Mask              */

#define FMC_ISPCTL_WAIT_CFG_Pos          (16)                                              /*!< FMC_T::ISPCTL: WAIT_CFG Position       */
#define FMC_ISPCTL_WAIT_CFG_Msk          (0x7ul << FMC_ISPCTL_WAIT_CFG_Pos)                /*!< FMC_T::ISPCTL: WAIT_CFG Mask           */

#define FMC_ISPCTL_CACHE_DIS_Pos         (21)                                              /*!< FMC_T::ISPCTL: CACHE_DIS Position      */
#define FMC_ISPCTL_CACHE_DIS_Msk         (0x1ul << FMC_ISPCTL_CACHE_DIS_Pos)               /*!< FMC_T::ISPCTL: CACHE_DIS Mask          */

#define FMC_ISPADDR_ISPADDR_Pos          (0)                                               /*!< FMC_T::ISPADDR: ISPADDR Position       */
#define FMC_ISPADDR_ISPADDR_Msk          (0xfffffffful << FMC_ISPADDR_ISPADDR_Pos)         /*!< FMC_T::ISPADDR: ISPADDR Mask           */

#define FMC_ISPDAT_ISPDAT_Pos            (0)                                               /*!< FMC_T::ISPDAT: ISPDAT Position         */
#define FMC_ISPDAT_ISPDAT_Msk            (0xfffffffful << FMC_ISPDAT_ISPDAT_Pos)           /*!< FMC_T::ISPDAT: ISPDAT Mask             */

#define FMC_ISPCMD_ISPCMD_Pos            (0)                                               /*!< FMC_T::ISPCMD: ISPCMD Position         */
#define FMC_ISPCMD_ISPCMD_Msk            (0x3ful << FMC_ISPCMD_ISPCMD_Pos)                 /*!< FMC_T::ISPCMD: ISPCMD Mask             */

#define FMC_ISPTRG_ISPGO_Pos             (0)                                               /*!< FMC_T::ISPTRG: ISPGO Position          */
#define FMC_ISPTRG_ISPGO_Msk             (0x1ul << FMC_ISPTRG_ISPGO_Pos)                   /*!< FMC_T::ISPTRG: ISPGO Mask              */

#define FMC_DFBADR_DFBA_Pos              (0)                                               /*!< FMC_T::DFBADR: DFBA Position           */
#define FMC_DFBADR_DFBA_Msk              (0xfffffffful << FMC_DFBADR_DFBA_Pos)             /*!< FMC_T::DFBADR: DFBA Mask               */

/**@}*/ /* FMC_CONST */
/**@}*/ /* end of FMC register group */


/*---------------------- General Purpose Input/Output Controller -------------------------*/
/**
    @addtogroup GPIO General Purpose Input/Output Controller(GPIO)
    Memory Mapped Structure for GPIO Controller
@{ */
 
typedef struct
{


/**
 * @var GPIO_T::PA_MODE
 * Offset: 0x00  GPIO PA Pin I/O Mode Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |MODE0     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[3:2]   |MODE1     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[5:4]   |MODE2     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[7:6]   |MODE3     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[9:8]   |MODE4     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[11:10] |MODE5     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[13:12] |MODE6     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[15:14] |MODE7     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[17:16] |MODE8     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[19:18] |MODE9     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[21:20] |MODE10    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[23:22] |MODE11    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[25:24] |MODE12    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[27:26] |MODE13    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[29:28] |MODE14    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[31:30] |MODE15    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * @var GPIO_T::PA_DOUT
 * Offset: 0x08  GPIO PA Data Output Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |DOUT      |Port [A/B/C/D] Pin[N] Output Value
 * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output or open-drain mode.
 * |        |          |0 = GPIO port [A/B/C/D] Pin[n] will drive Low if the corresponding output mode bit is set.
 * |        |          |1 = GPIO port [A/B/C/D] Pin[n] will drive High if the corresponding output mode bit is set.
 * |        |          |Note: PB_DOUT [15:2] are reserved to 0.
 * @var GPIO_T::PA_PIN
 * Offset: 0x10  GPIO PA Pin Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PIN       |Port [A/B/C/D] Pin[N] Pin Values
 * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin
 * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
 * |        |          |Note: PB_PIN [15:2] are reserved to 0.
 * @var GPIO_T::PA_INTTYPE
 * Offset: 0x18  GPIO PA Interrupt Trigger Type
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |TYPE      |Port [A/B/C/D] Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
 * |        |          |TYPE[n] is used to control whether the interrupt mode is level triggered or edge triggered
 * |        |          |If the interrupt mode is level triggered, the input source is sampled by one HCLK clock to generate the interrupt
 * |        |          |0 = Edge triggered interrupt.
 * |        |          |1 = Level triggered interrupt.
 * |        |          |Note 1: If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register
 * |        |          |If both levels are set, the setting is ignored and no interrupt will occur
 * |        |          |Note 2: PB_INTTYPE [15:2] are reserved to 0.
 * @var GPIO_T::PA_INTEN
 * Offset: 0x1C  GPIO PA Interrupt Enable
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FLIEN0    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[1]     |FLIEN1    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[2]     |FLIEN2    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[3]     |FLIEN3    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[4]     |FLIEN4    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[5]     |FLIEN5    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[6]     |FLIEN6    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[7]     |FLIEN7    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[8]     |FLIEN8    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[9]     |FLIEN9    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[10]    |FLIEN10   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[11]    |FLIEN11   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[12]    |FLIEN12   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[13]    |FLIEN13   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[14]    |FLIEN14   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[15]    |FLIEN15   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[16]    |RHIEN0    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[17]    |RHIEN1    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[18]    |RHIEN2    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[19]    |RHIEN3    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[20]    |RHIEN4    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[21]    |RHIEN5    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[22]    |RHIEN6    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[23]    |RHIEN7    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[24]    |RHIEN8    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[25]    |RHIEN9    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[26]    |RHIEN10   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[27]    |RHIEN11   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[28]    |RHIEN12   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[29]    |RHIEN13   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[30]    |RHIEN14   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[31]    |RHIEN15   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * @var GPIO_T::PA_INTSRC
 * Offset: 0x20  GPIO PA Interrupt Source Flag
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |INTSRC    |Port [A/B/C/D] Interrupt Source Flag
 * |        |          |Read operation:
 * |        |          |0 = No interrupt from Px.n.
 * |        |          |1 = Px.n generated an interrupt.
 * |        |          |Write operation:
 * |        |          |0 = No action.
 * |        |          |1 = Clear the corresponding pending interrupt.
 * |        |          |Note: PB_INTSRC [15:2] are reserved to 0.
 * @var GPIO_T::PB_MODE
 * Offset: 0x40  GPIO PB Pin I/O Mode Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |MODE0     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[3:2]   |MODE1     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[5:4]   |MODE2     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[7:6]   |MODE3     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[9:8]   |MODE4     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[11:10] |MODE5     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[13:12] |MODE6     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[15:14] |MODE7     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[17:16] |MODE8     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[19:18] |MODE9     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[21:20] |MODE10    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[23:22] |MODE11    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[25:24] |MODE12    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[27:26] |MODE13    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[29:28] |MODE14    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[31:30] |MODE15    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * @var GPIO_T::PB_DOUT
 * Offset: 0x48  GPIO PB Data Output Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |DOUT      |Port [A/B/C/D] Pin[N] Output Value
 * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output or open-drain mode.
 * |        |          |0 = GPIO port [A/B/C/D] Pin[n] will drive Low if the corresponding output mode bit is set.
 * |        |          |1 = GPIO port [A/B/C/D] Pin[n] will drive High if the corresponding output mode bit is set.
 * |        |          |Note: PB_DOUT [15:2] are reserved to 0.
 * @var GPIO_T::PB_PIN
 * Offset: 0x50  GPIO PB Pin Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PIN       |Port [A/B/C/D] Pin[N] Pin Values
 * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin
 * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
 * |        |          |Note: PB_PIN [15:2] are reserved to 0.
 * @var GPIO_T::PB_INTTYPE
 * Offset: 0x58  GPIO PB Interrupt Trigger Type
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |TYPE      |Port [A/B/C/D] Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
 * |        |          |TYPE[n] is used to control whether the interrupt mode is level triggered or edge triggered
 * |        |          |If the interrupt mode is level triggered, the input source is sampled by one HCLK clock to generate the interrupt
 * |        |          |0 = Edge triggered interrupt.
 * |        |          |1 = Level triggered interrupt.
 * |        |          |Note 1: If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register
 * |        |          |If both levels are set, the setting is ignored and no interrupt will occur
 * |        |          |Note 2: PB_INTTYPE [15:2] are reserved to 0.
 * @var GPIO_T::PB_INTEN
 * Offset: 0x5C  GPIO PB Interrupt Enable
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FLIEN0    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[1]     |FLIEN1    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[2]     |FLIEN2    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[3]     |FLIEN3    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[4]     |FLIEN4    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[5]     |FLIEN5    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[6]     |FLIEN6    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[7]     |FLIEN7    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[8]     |FLIEN8    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[9]     |FLIEN9    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[10]    |FLIEN10   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[11]    |FLIEN11   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[12]    |FLIEN12   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[13]    |FLIEN13   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[14]    |FLIEN14   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[15]    |FLIEN15   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[16]    |RHIEN0    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[17]    |RHIEN1    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[18]    |RHIEN2    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[19]    |RHIEN3    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[20]    |RHIEN4    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[21]    |RHIEN5    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[22]    |RHIEN6    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[23]    |RHIEN7    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[24]    |RHIEN8    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[25]    |RHIEN9    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[26]    |RHIEN10   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[27]    |RHIEN11   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[28]    |RHIEN12   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[29]    |RHIEN13   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[30]    |RHIEN14   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[31]    |RHIEN15   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * @var GPIO_T::PB_INTSRC
 * Offset: 0x60  GPIO PB Interrupt Source Flag
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |INTSRC    |Port [A/B/C/D] Interrupt Source Flag
 * |        |          |Read operation:
 * |        |          |0 = No interrupt from Px.n.
 * |        |          |1 = Px.n generated an interrupt.
 * |        |          |Write operation:
 * |        |          |0 = No action.
 * |        |          |1 = Clear the corresponding pending interrupt.
 * |        |          |Note: PB_INTSRC [15:2] are reserved to 0.
 * @var GPIO_T::PC_MODE
 * Offset: 0x80  GPIO PC Pin I/O Mode Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |MODE0     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[3:2]   |MODE1     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[5:4]   |MODE2     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[7:6]   |MODE3     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[9:8]   |MODE4     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[11:10] |MODE5     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[13:12] |MODE6     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[15:14] |MODE7     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[17:16] |MODE8     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[19:18] |MODE9     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[21:20] |MODE10    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[23:22] |MODE11    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[25:24] |MODE12    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[27:26] |MODE13    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[29:28] |MODE14    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[31:30] |MODE15    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * @var GPIO_T::PC_DOUT
 * Offset: 0x88  GPIO PC Data Output Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |DOUT      |Port [A/B/C/D] Pin[N] Output Value
 * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output or open-drain mode.
 * |        |          |0 = GPIO port [A/B/C/D] Pin[n] will drive Low if the corresponding output mode bit is set.
 * |        |          |1 = GPIO port [A/B/C/D] Pin[n] will drive High if the corresponding output mode bit is set.
 * |        |          |Note: PB_DOUT [15:2] are reserved to 0.
 * @var GPIO_T::PC_PIN
 * Offset: 0x90  GPIO PC Pin Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PIN       |Port [A/B/C/D] Pin[N] Pin Values
 * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin
 * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
 * |        |          |Note: PB_PIN [15:2] are reserved to 0.
 * @var GPIO_T::PC_INTTYPE
 * Offset: 0x98  GPIO PC Interrupt Trigger Type
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |TYPE      |Port [A/B/C/D] Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
 * |        |          |TYPE[n] is used to control whether the interrupt mode is level triggered or edge triggered
 * |        |          |If the interrupt mode is level triggered, the input source is sampled by one HCLK clock to generate the interrupt
 * |        |          |0 = Edge triggered interrupt.
 * |        |          |1 = Level triggered interrupt.
 * |        |          |Note 1: If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register
 * |        |          |If both levels are set, the setting is ignored and no interrupt will occur
 * |        |          |Note 2: PB_INTTYPE [15:2] are reserved to 0.
 * @var GPIO_T::PC_INTEN
 * Offset: 0x9C  GPIO PC Interrupt Enable
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FLIEN0    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[1]     |FLIEN1    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[2]     |FLIEN2    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[3]     |FLIEN3    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[4]     |FLIEN4    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[5]     |FLIEN5    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[6]     |FLIEN6    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[7]     |FLIEN7    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[8]     |FLIEN8    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[9]     |FLIEN9    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[10]    |FLIEN10   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[11]    |FLIEN11   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[12]    |FLIEN12   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[13]    |FLIEN13   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[14]    |FLIEN14   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[15]    |FLIEN15   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[16]    |RHIEN0    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[17]    |RHIEN1    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[18]    |RHIEN2    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[19]    |RHIEN3    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[20]    |RHIEN4    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[21]    |RHIEN5    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[22]    |RHIEN6    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[23]    |RHIEN7    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[24]    |RHIEN8    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[25]    |RHIEN9    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[26]    |RHIEN10   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[27]    |RHIEN11   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[28]    |RHIEN12   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[29]    |RHIEN13   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[30]    |RHIEN14   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[31]    |RHIEN15   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * @var GPIO_T::PC_INTSRC
 * Offset: 0xA0  GPIO PC Interrupt Source Flag
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |INTSRC    |Port [A/B/C/D] Interrupt Source Flag
 * |        |          |Read operation:
 * |        |          |0 = No interrupt from Px.n.
 * |        |          |1 = Px.n generated an interrupt.
 * |        |          |Write operation:
 * |        |          |0 = No action.
 * |        |          |1 = Clear the corresponding pending interrupt.
 * |        |          |Note: PB_INTSRC [15:2] are reserved to 0.
 * @var GPIO_T::PD_MODE
 * Offset: 0xC0  GPIO PD Pin I/O Mode Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |MODE0     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[3:2]   |MODE1     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[5:4]   |MODE2     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[7:6]   |MODE3     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[9:8]   |MODE4     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[11:10] |MODE5     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[13:12] |MODE6     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[15:14] |MODE7     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[17:16] |MODE8     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[19:18] |MODE9     |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[21:20] |MODE10    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[23:22] |MODE11    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[25:24] |MODE12    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[27:26] |MODE13    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[29:28] |MODE14    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * |[31:30] |MODE15    |Port [A/B/C/D] Pin[N] I/O Mode Control
 * |        |          |Each GPIO Px pin has four modes:
 * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
 * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO Px[n] pin is in INPUT with internal PULLUP resister mode.
 * |        |          |Note: PB_MODE [31:4] are reserved to 0.
 * @var GPIO_T::PD_DOUT
 * Offset: 0xC8  GPIO PD Data Output Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |DOUT      |Port [A/B/C/D] Pin[N] Output Value
 * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output or open-drain mode.
 * |        |          |0 = GPIO port [A/B/C/D] Pin[n] will drive Low if the corresponding output mode bit is set.
 * |        |          |1 = GPIO port [A/B/C/D] Pin[n] will drive High if the corresponding output mode bit is set.
 * |        |          |Note: PB_DOUT [15:2] are reserved to 0.
 * @var GPIO_T::PD_PIN
 * Offset: 0xD0  GPIO PD Pin Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PIN       |Port [A/B/C/D] Pin[N] Pin Values
 * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin
 * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
 * |        |          |Note: PB_PIN [15:2] are reserved to 0.
 * @var GPIO_T::PD_INTTYPE
 * Offset: 0xD8  GPIO PD Interrupt Trigger Type
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |TYPE      |Port [A/B/C/D] Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
 * |        |          |TYPE[n] is used to control whether the interrupt mode is level triggered or edge triggered
 * |        |          |If the interrupt mode is level triggered, the input source is sampled by one HCLK clock to generate the interrupt
 * |        |          |0 = Edge triggered interrupt.
 * |        |          |1 = Level triggered interrupt.
 * |        |          |Note 1: If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register
 * |        |          |If both levels are set, the setting is ignored and no interrupt will occur
 * |        |          |Note 2: PB_INTTYPE [15:2] are reserved to 0.
 * @var GPIO_T::PD_INTEN
 * Offset: 0xDC  GPIO PD Interrupt Enable
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FLIEN0    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[1]     |FLIEN1    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[2]     |FLIEN2    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[3]     |FLIEN3    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[4]     |FLIEN4    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[5]     |FLIEN5    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[6]     |FLIEN6    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[7]     |FLIEN7    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[8]     |FLIEN8    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[9]     |FLIEN9    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[10]    |FLIEN10   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[11]    |FLIEN11   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[12]    |FLIEN12   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[13]    |FLIEN13   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[14]    |FLIEN14   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[15]    |FLIEN15   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
 * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
 * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
 * |        |          |Note: PB_FLIEN [15:2] are reserved to 0.
 * |[16]    |RHIEN0    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[17]    |RHIEN1    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[18]    |RHIEN2    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[19]    |RHIEN3    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[20]    |RHIEN4    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[21]    |RHIEN5    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[22]    |RHIEN6    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[23]    |RHIEN7    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[24]    |RHIEN8    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[25]    |RHIEN9    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[26]    |RHIEN10   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[27]    |RHIEN11   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[28]    |RHIEN12   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[29]    |RHIEN13   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[30]    |RHIEN14   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * |[31]    |RHIEN15   |Port [A/B/C/D] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin
 * |        |          |To set u201C1u201D also enables the pin wake-up function.
 * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
 * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
 * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
 * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
 * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 * |        |          |Note: PB_RHIEN [15:2] are reserved to 0.
 * @var GPIO_T::PD_INTSRC
 * Offset: 0xE0  GPIO PD Interrupt Source Flag
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |INTSRC    |Port [A/B/C/D] Interrupt Source Flag
 * |        |          |Read operation:
 * |        |          |0 = No interrupt from Px.n.
 * |        |          |1 = Px.n generated an interrupt.
 * |        |          |Write operation:
 * |        |          |0 = No action.
 * |        |          |1 = Clear the corresponding pending interrupt.
 * |        |          |Note: PB_INTSRC [15:2] are reserved to 0.
 * @var GPIO_T::PA0_PDIO
 * Offset: 0x800  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA1_PDIO
 * Offset: 0x804  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA2_PDIO
 * Offset: 0x808  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA3_PDIO
 * Offset: 0x80C  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA4_PDIO
 * Offset: 0x810  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA5_PDIO
 * Offset: 0x814  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA6_PDIO
 * Offset: 0x818  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA7_PDIO
 * Offset: 0x81C  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA8_PDIO
 * Offset: 0x820  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA9_PDIO
 * Offset: 0x824  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA10_PDIO
 * Offset: 0x828  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA11_PDIO
 * Offset: 0x82C  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA12_PDIO
 * Offset: 0x830  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA13_PDIO
 * Offset: 0x834  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA14_PDIO
 * Offset: 0x838  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PA15_PDIO
 * Offset: 0x83C  GPIO PA.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PB0_PDIO
 * Offset: 0x840  GPIO PB.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PB1_PDIO
 * Offset: 0x844  GPIO PB.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PB2_PDIO
 * Offset: 0x848  GPIO PB.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PB3_PDIO
 * Offset: 0x84C  GPIO PB.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PB4_PDIO
 * Offset: 0x850  GPIO PB.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PB5_PDIO
 * Offset: 0x854  GPIO PB.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PB6_PDIO
 * Offset: 0x858  GPIO PB.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PB7_PDIO
 * Offset: 0x85C  GPIO PB.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PB8_PDIO
 * Offset: 0x860  GPIO PB.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PB9_PDIO
 * Offset: 0x864  GPIO PB.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PB10_PDIO
 * Offset: 0x868  GPIO PB.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PB11_PDIO
 * Offset: 0x86C  GPIO PB.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC0_PDIO
 * Offset: 0x880  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC1_PDIO
 * Offset: 0x884  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC2_PDIO
 * Offset: 0x888  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC3_PDIO
 * Offset: 0x88C  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC4_PDIO
 * Offset: 0x890  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC5_PDIO
 * Offset: 0x894  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC6_PDIO
 * Offset: 0x898  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC7_PDIO
 * Offset: 0x89C  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC8_PDIO
 * Offset: 0x8A0  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC9_PDIO
 * Offset: 0x8A4  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC10_PDIO
 * Offset: 0x8A8  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC11_PDIO
 * Offset: 0x8AC  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC12_PDIO
 * Offset: 0x8B0  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC13_PDIO
 * Offset: 0x8B4  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC14_PDIO
 * Offset: 0x8B8  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PC15_PDIO
 * Offset: 0x8BC  GPIO PC.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD0_PDIO
 * Offset: 0x8C0  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD1_PDIO
 * Offset: 0x8C4  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD2_PDIO
 * Offset: 0x8C8  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD3_PDIO
 * Offset: 0x8CC  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD4_PDIO
 * Offset: 0x8D0  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD5_PDIO
 * Offset: 0x8D4  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD6_PDIO
 * Offset: 0x8D8  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD7_PDIO
 * Offset: 0x8DC  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD8_PDIO
 * Offset: 0x8E0  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD9_PDIO
 * Offset: 0x8E4  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD10_PDIO
 * Offset: 0x8E8  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD11_PDIO
 * Offset: 0x8EC  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD12_PDIO
 * Offset: 0x8F0  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD13_PDIO
 * Offset: 0x8F4  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD14_PDIO
 * Offset: 0x8F8  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 * @var GPIO_T::PD15_PDIO
 * Offset: 0x8FC  GPIO PD.n Pin Data Input/Output Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDIO      |GPIO Px.n Pin Data Input/Output
 * |        |          |Writing this bit can control one GPIO pin output value.
 * |        |          |0 = Corresponding GPIO pin set to low.
 * |        |          |1 = Corresponding GPIO pin set to high.
 * |        |          |Read this register to get GPIO pin status.
 * |        |          |For example, writing PA0_PDIO will reflect the written value to bit DOUT (Px_DOUT[0]), reading PA0_PDIO will return the value of PIN (PA_PIN[0]).
 * |        |          |Note1: The writing operation will not be affected by register DATMSK (Px_DATMSK[n]).
 * |        |          |Note2:
 * |        |          |Max. n=15 for port A. C. D
 * |        |          |Max. n=1 for port B.
 */
    __IO uint32_t MODE;                  /*!< [0x0000] GPIO Pin I/O Mode Control                                        */
    __I  uint32_t RESERVE0[1];                                                                                          
    __IO uint32_t DOUT;                  /*!< [0x0008] GPIO Data Output Value                                           */
    __I  uint32_t RESERVE1[1];                                                                                          
    __I  uint32_t PIN;                   /*!< [0x0010] GPIO Pin Value                                                   */
    __I  uint32_t RESERVE2[1];                                                                                          
    __IO uint32_t INTTYPE;               /*!< [0x0018] GPIO Interrupt Trigger Type                                      */
    __IO uint32_t INTEN;                 /*!< [0x001c] GPIO Interrupt Enable                                            */
    __IO uint32_t INTSRC;                /*!< [0x0020] GPIO Interrupt Source Flag                                       */

} GPIO_T;

/**
    @addtogroup GPIO_CONST GPIO Bit Field Definition
    Constant Definitions for GPIO Controller
@{ */

#define GPIO_MODE_MODE0_Pos              (0)                                               /*!< GPIO_T::MODE: MODE0 Position           */
#define GPIO_MODE_MODE0_Msk              (0x3ul << GPIO_MODE_MODE0_Pos)                    /*!< GPIO_T::MODE: MODE0 Mask               */

#define GPIO_MODE_MODE1_Pos              (2)                                               /*!< GPIO_T::MODE: MODE1 Position           */
#define GPIO_MODE_MODE1_Msk              (0x3ul << GPIO_MODE_MODE1_Pos)                    /*!< GPIO_T::MODE: MODE1 Mask               */

#define GPIO_MODE_MODE2_Pos              (4)                                               /*!< GPIO_T::MODE: MODE2 Position           */
#define GPIO_MODE_MODE2_Msk              (0x3ul << GPIO_MODE_MODE2_Pos)                    /*!< GPIO_T::MODE: MODE2 Mask               */

#define GPIO_MODE_MODE3_Pos              (6)                                               /*!< GPIO_T::MODE: MODE3 Position           */
#define GPIO_MODE_MODE3_Msk              (0x3ul << GPIO_MODE_MODE3_Pos)                    /*!< GPIO_T::MODE: MODE3 Mask               */

#define GPIO_MODE_MODE4_Pos              (8)                                               /*!< GPIO_T::MODE: MODE4 Position           */
#define GPIO_MODE_MODE4_Msk              (0x3ul << GPIO_MODE_MODE4_Pos)                    /*!< GPIO_T::MODE: MODE4 Mask               */

#define GPIO_MODE_MODE5_Pos              (10)                                              /*!< GPIO_T::MODE: MODE5 Position           */
#define GPIO_MODE_MODE5_Msk              (0x3ul << GPIO_MODE_MODE5_Pos)                    /*!< GPIO_T::MODE: MODE5 Mask               */

#define GPIO_MODE_MODE6_Pos              (12)                                              /*!< GPIO_T::MODE: MODE6 Position           */
#define GPIO_MODE_MODE6_Msk              (0x3ul << GPIO_MODE_MODE6_Pos)                    /*!< GPIO_T::MODE: MODE6 Mask               */

#define GPIO_MODE_MODE7_Pos              (14)                                              /*!< GPIO_T::MODE: MODE7 Position           */
#define GPIO_MODE_MODE7_Msk              (0x3ul << GPIO_MODE_MODE7_Pos)                    /*!< GPIO_T::MODE: MODE7 Mask               */

#define GPIO_MODE_MODE8_Pos              (16)                                              /*!< GPIO_T::MODE: MODE8 Position           */
#define GPIO_MODE_MODE8_Msk              (0x3ul << GPIO_MODE_MODE8_Pos)                    /*!< GPIO_T::MODE: MODE8 Mask               */

#define GPIO_MODE_MODE9_Pos              (18)                                              /*!< GPIO_T::MODE: MODE9 Position           */
#define GPIO_MODE_MODE9_Msk              (0x3ul << GPIO_MODE_MODE9_Pos)                    /*!< GPIO_T::MODE: MODE9 Mask               */

#define GPIO_MODE_MODE10_Pos             (20)                                              /*!< GPIO_T::MODE: MODE10 Position          */
#define GPIO_MODE_MODE10_Msk             (0x3ul << GPIO_MODE_MODE10_Pos)                   /*!< GPIO_T::MODE: MODE10 Mask              */

#define GPIO_MODE_MODE11_Pos             (22)                                              /*!< GPIO_T::MODE: MODE11 Position          */
#define GPIO_MODE_MODE11_Msk             (0x3ul << GPIO_MODE_MODE11_Pos)                   /*!< GPIO_T::MODE: MODE11 Mask              */

#define GPIO_MODE_MODE12_Pos             (24)                                              /*!< GPIO_T::MODE: MODE12 Position          */
#define GPIO_MODE_MODE12_Msk             (0x3ul << GPIO_MODE_MODE12_Pos)                   /*!< GPIO_T::MODE: MODE12 Mask              */

#define GPIO_MODE_MODE13_Pos             (26)                                              /*!< GPIO_T::MODE: MODE13 Position          */
#define GPIO_MODE_MODE13_Msk             (0x3ul << GPIO_MODE_MODE13_Pos)                   /*!< GPIO_T::MODE: MODE13 Mask              */

#define GPIO_MODE_MODE14_Pos             (28)                                              /*!< GPIO_T::MODE: MODE14 Position          */
#define GPIO_MODE_MODE14_Msk             (0x3ul << GPIO_MODE_MODE14_Pos)                   /*!< GPIO_T::MODE: MODE14 Mask              */

#define GPIO_MODE_MODE15_Pos             (30)                                              /*!< GPIO_T::MODE: MODE15 Position          */
#define GPIO_MODE_MODE15_Msk             (0x3ul << GPIO_MODE_MODE15_Pos)                   /*!< GPIO_T::MODE: MODE15 Mask              */

#define GPIO_DOUT_DOUT_Pos               (0)                                               /*!< GPIO_T::DOUT: DOUT Position            */
#define GPIO_DOUT_DOUT_Msk               (0xfffful << GPIO_DOUT_DOUT_Pos)                  /*!< GPIO_T::DOUT: DOUT Mask                */

#define GPIO_PIN_PIN_Pos                 (0)                                               /*!< GPIO_T::PIN: PIN Position              */
#define GPIO_PIN_PIN_Msk                 (0xfffful << GPIO_PIN_PIN_Pos)                    /*!< GPIO_T::PIN: PIN Mask                  */

#define GPIO_INTTYPE_TYPE_Pos            (0)                                               /*!< GPIO_T::INTTYPE: TYPE Position         */
#define GPIO_INTTYPE_TYPE_Msk            (0xfffful << GPIO_INTTYPE_TYPE_Pos)               /*!< GPIO_T::INTTYPE: TYPE Mask             */

#define GPIO_INTEN_FLIEN0_Pos            (0)                                               /*!< GPIO_T::INTEN: FLIEN0 Position         */
#define GPIO_INTEN_FLIEN0_Msk            (0x1ul << GPIO_INTEN_FLIEN0_Pos)                  /*!< GPIO_T::INTEN: FLIEN0 Mask             */

#define GPIO_INTEN_FLIEN1_Pos            (1)                                               /*!< GPIO_T::INTEN: FLIEN1 Position         */
#define GPIO_INTEN_FLIEN1_Msk            (0x1ul << GPIO_INTEN_FLIEN1_Pos)                  /*!< GPIO_T::INTEN: FLIEN1 Mask             */

#define GPIO_INTEN_FLIEN2_Pos            (2)                                               /*!< GPIO_T::INTEN: FLIEN2 Position         */
#define GPIO_INTEN_FLIEN2_Msk            (0x1ul << GPIO_INTEN_FLIEN2_Pos)                  /*!< GPIO_T::INTEN: FLIEN2 Mask             */

#define GPIO_INTEN_FLIEN3_Pos            (3)                                               /*!< GPIO_T::INTEN: FLIEN3 Position         */
#define GPIO_INTEN_FLIEN3_Msk            (0x1ul << GPIO_INTEN_FLIEN3_Pos)                  /*!< GPIO_T::INTEN: FLIEN3 Mask             */

#define GPIO_INTEN_FLIEN4_Pos            (4)                                               /*!< GPIO_T::INTEN: FLIEN4 Position         */
#define GPIO_INTEN_FLIEN4_Msk            (0x1ul << GPIO_INTEN_FLIEN4_Pos)                  /*!< GPIO_T::INTEN: FLIEN4 Mask             */

#define GPIO_INTEN_FLIEN5_Pos            (5)                                               /*!< GPIO_T::INTEN: FLIEN5 Position         */
#define GPIO_INTEN_FLIEN5_Msk            (0x1ul << GPIO_INTEN_FLIEN5_Pos)                  /*!< GPIO_T::INTEN: FLIEN5 Mask             */

#define GPIO_INTEN_FLIEN6_Pos            (6)                                               /*!< GPIO_T::INTEN: FLIEN6 Position         */
#define GPIO_INTEN_FLIEN6_Msk            (0x1ul << GPIO_INTEN_FLIEN6_Pos)                  /*!< GPIO_T::INTEN: FLIEN6 Mask             */

#define GPIO_INTEN_FLIEN7_Pos            (7)                                               /*!< GPIO_T::INTEN: FLIEN7 Position         */
#define GPIO_INTEN_FLIEN7_Msk            (0x1ul << GPIO_INTEN_FLIEN7_Pos)                  /*!< GPIO_T::INTEN: FLIEN7 Mask             */

#define GPIO_INTEN_FLIEN8_Pos            (8)                                               /*!< GPIO_T::INTEN: FLIEN8 Position         */
#define GPIO_INTEN_FLIEN8_Msk            (0x1ul << GPIO_INTEN_FLIEN8_Pos)                  /*!< GPIO_T::INTEN: FLIEN8 Mask             */

#define GPIO_INTEN_FLIEN9_Pos            (9)                                               /*!< GPIO_T::INTEN: FLIEN9 Position         */
#define GPIO_INTEN_FLIEN9_Msk            (0x1ul << GPIO_INTEN_FLIEN9_Pos)                  /*!< GPIO_T::INTEN: FLIEN9 Mask             */

#define GPIO_INTEN_FLIEN10_Pos           (10)                                              /*!< GPIO_T::INTEN: FLIEN10 Position        */
#define GPIO_INTEN_FLIEN10_Msk           (0x1ul << GPIO_INTEN_FLIEN10_Pos)                 /*!< GPIO_T::INTEN: FLIEN10 Mask            */

#define GPIO_INTEN_FLIEN11_Pos           (11)                                              /*!< GPIO_T::INTEN: FLIEN11 Position        */
#define GPIO_INTEN_FLIEN11_Msk           (0x1ul << GPIO_INTEN_FLIEN11_Pos)                 /*!< GPIO_T::INTEN: FLIEN11 Mask            */

#define GPIO_INTEN_FLIEN12_Pos           (12)                                              /*!< GPIO_T::INTEN: FLIEN12 Position        */
#define GPIO_INTEN_FLIEN12_Msk           (0x1ul << GPIO_INTEN_FLIEN12_Pos)                 /*!< GPIO_T::INTEN: FLIEN12 Mask            */

#define GPIO_INTEN_FLIEN13_Pos           (13)                                              /*!< GPIO_T::INTEN: FLIEN13 Position        */
#define GPIO_INTEN_FLIEN13_Msk           (0x1ul << GPIO_INTEN_FLIEN13_Pos)                 /*!< GPIO_T::INTEN: FLIEN13 Mask            */

#define GPIO_INTEN_FLIEN14_Pos           (14)                                              /*!< GPIO_T::INTEN: FLIEN14 Position        */
#define GPIO_INTEN_FLIEN14_Msk           (0x1ul << GPIO_INTEN_FLIEN14_Pos)                 /*!< GPIO_T::INTEN: FLIEN14 Mask            */

#define GPIO_INTEN_FLIEN15_Pos           (15)                                              /*!< GPIO_T::INTEN: FLIEN15 Position        */
#define GPIO_INTEN_FLIEN15_Msk           (0x1ul << GPIO_INTEN_FLIEN15_Pos)                 /*!< GPIO_T::INTEN: FLIEN15 Mask            */

#define GPIO_INTEN_RHIEN0_Pos            (16)                                              /*!< GPIO_T::INTEN: RHIEN0 Position         */
#define GPIO_INTEN_RHIEN0_Msk            (0x1ul << GPIO_INTEN_RHIEN0_Pos)                  /*!< GPIO_T::INTEN: RHIEN0 Mask             */

#define GPIO_INTEN_RHIEN1_Pos            (17)                                              /*!< GPIO_T::INTEN: RHIEN1 Position         */
#define GPIO_INTEN_RHIEN1_Msk            (0x1ul << GPIO_INTEN_RHIEN1_Pos)                  /*!< GPIO_T::INTEN: RHIEN1 Mask             */

#define GPIO_INTEN_RHIEN2_Pos            (18)                                              /*!< GPIO_T::INTEN: RHIEN2 Position         */
#define GPIO_INTEN_RHIEN2_Msk            (0x1ul << GPIO_INTEN_RHIEN2_Pos)                  /*!< GPIO_T::INTEN: RHIEN2 Mask             */

#define GPIO_INTEN_RHIEN3_Pos            (19)                                              /*!< GPIO_T::INTEN: RHIEN3 Position         */
#define GPIO_INTEN_RHIEN3_Msk            (0x1ul << GPIO_INTEN_RHIEN3_Pos)                  /*!< GPIO_T::INTEN: RHIEN3 Mask             */

#define GPIO_INTEN_RHIEN4_Pos            (20)                                              /*!< GPIO_T::INTEN: RHIEN4 Position         */
#define GPIO_INTEN_RHIEN4_Msk            (0x1ul << GPIO_INTEN_RHIEN4_Pos)                  /*!< GPIO_T::INTEN: RHIEN4 Mask             */

#define GPIO_INTEN_RHIEN5_Pos            (21)                                              /*!< GPIO_T::INTEN: RHIEN5 Position         */
#define GPIO_INTEN_RHIEN5_Msk            (0x1ul << GPIO_INTEN_RHIEN5_Pos)                  /*!< GPIO_T::INTEN: RHIEN5 Mask             */

#define GPIO_INTEN_RHIEN6_Pos            (22)                                              /*!< GPIO_T::INTEN: RHIEN6 Position         */
#define GPIO_INTEN_RHIEN6_Msk            (0x1ul << GPIO_INTEN_RHIEN6_Pos)                  /*!< GPIO_T::INTEN: RHIEN6 Mask             */

#define GPIO_INTEN_RHIEN7_Pos            (23)                                              /*!< GPIO_T::INTEN: RHIEN7 Position         */
#define GPIO_INTEN_RHIEN7_Msk            (0x1ul << GPIO_INTEN_RHIEN7_Pos)                  /*!< GPIO_T::INTEN: RHIEN7 Mask             */

#define GPIO_INTEN_RHIEN8_Pos            (24)                                              /*!< GPIO_T::INTEN: RHIEN8 Position         */
#define GPIO_INTEN_RHIEN8_Msk            (0x1ul << GPIO_INTEN_RHIEN8_Pos)                  /*!< GPIO_T::INTEN: RHIEN8 Mask             */

#define GPIO_INTEN_RHIEN9_Pos            (25)                                              /*!< GPIO_T::INTEN: RHIEN9 Position         */
#define GPIO_INTEN_RHIEN9_Msk            (0x1ul << GPIO_INTEN_RHIEN9_Pos)                  /*!< GPIO_T::INTEN: RHIEN9 Mask             */

#define GPIO_INTEN_RHIEN10_Pos           (26)                                              /*!< GPIO_T::INTEN: RHIEN10 Position        */
#define GPIO_INTEN_RHIEN10_Msk           (0x1ul << GPIO_INTEN_RHIEN10_Pos)                 /*!< GPIO_T::INTEN: RHIEN10 Mask            */

#define GPIO_INTEN_RHIEN11_Pos           (27)                                              /*!< GPIO_T::INTEN: RHIEN11 Position        */
#define GPIO_INTEN_RHIEN11_Msk           (0x1ul << GPIO_INTEN_RHIEN11_Pos)                 /*!< GPIO_T::INTEN: RHIEN11 Mask            */

#define GPIO_INTEN_RHIEN12_Pos           (28)                                              /*!< GPIO_T::INTEN: RHIEN12 Position        */
#define GPIO_INTEN_RHIEN12_Msk           (0x1ul << GPIO_INTEN_RHIEN12_Pos)                 /*!< GPIO_T::INTEN: RHIEN12 Mask            */

#define GPIO_INTEN_RHIEN13_Pos           (29)                                              /*!< GPIO_T::INTEN: RHIEN13 Position        */
#define GPIO_INTEN_RHIEN13_Msk           (0x1ul << GPIO_INTEN_RHIEN13_Pos)                 /*!< GPIO_T::INTEN: RHIEN13 Mask            */

#define GPIO_INTEN_RHIEN14_Pos           (30)                                              /*!< GPIO_T::INTEN: RHIEN14 Position        */
#define GPIO_INTEN_RHIEN14_Msk           (0x1ul << GPIO_INTEN_RHIEN14_Pos)                 /*!< GPIO_T::INTEN: RHIEN14 Mask            */

#define GPIO_INTEN_RHIEN15_Pos           (31)                                              /*!< GPIO_T::INTEN: RHIEN15 Position        */
#define GPIO_INTEN_RHIEN15_Msk           (0x1ul << GPIO_INTEN_RHIEN15_Pos)                 /*!< GPIO_T::INTEN: RHIEN15 Mask            */

#define GPIO_INTSRC_INTSRC_Pos           (0)                                               /*!< GPIO_T::INTSRC: INTSRC Position        */
#define GPIO_INTSRC_INTSRC_Msk           (0xfffful << GPIO_INTSRC_INTSRC_Pos)              /*!< GPIO_T::INTSRC: INTSRC Mask            */

/**@}*/ /* GPIO_CONST */
/**@}*/ /* end of GPIO register group */


/*---------------------- Inter-IC Bus Controller -------------------------*/
/**
    @addtogroup I2C Inter-IC Bus Controller(I2C)
    Memory Mapped Structure for I2C Controller
@{ */
 
typedef struct
{


/**
 * @var I2C_T::CTL
 * Offset: 0x00  I2C Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2]     |AA        |Assert Acknowledge Control Bit
 * |        |          |When AA=1 prior to address or data received, an acknowledge (ACK - low level to SDA) will be returned during the acknowledge clock pulse on the SCL line when:.
 * |        |          |1. A slave is acknowledging the address sent from master,
 * |        |          |2. The receiver devices are acknowledging the data sent by transmitter.
 * |        |          |When AA = 0 prior to address or data received, a Not acknowledged (high level to SDA) will be returned during the acknowledge clock pulse on the SCL line
 * |[3]     |SI        |I2C Interrupt Flag
 * |        |          |When a new SIO state is present in the I2C_STATUS register, the SI flag is set by hardware, and if bit I2CEN (I2C_CTL[7]) is set, the I2C interrupt is requested
 * |        |          |SI must be cleared by software
 * |        |          |Clear SI is by writing one to this bit.
 * |[4]     |STO       |I2C STOP Control Bit
 * |        |          |In master mode, set STO to transmit a STOP condition to bus
 * |        |          |I2C hardware will check the bus condition, when a STOP condition is detected this bit will be cleared by hardware automatically
 * |        |          |In slave mode, setting STO resets I2C hardware to the defined u201Cnot addressedu201D slave mode
 * |        |          |This means it is NO LONGER in the slave receiver mode able receive data from the master transmit device.
 * |[5]     |STA       |I2C START Control Bit
 * |        |          |Setting STA to logic 1 will enter master mode, the I2C hardware sends a START or repeat START condition to bus when the bus is free.
 * |[6]     |I2CEN     |I2C Controller Enable Bit
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |        |          |Set to enable I2C serial function block. 
 * |[7]     |INTEN     |Enable Interrupt
 * |        |          |0 = Disable interrupt.
 * |        |          |1 = Enable interrupt CPU.
 * @var I2C_T::ADDR0
 * Offset: 0x04  I2C Slave Address Register0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = Disable General Call Function.
 * |        |          |1 = Enable General Call Function.
 * |[7:1]   |ADDR      |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in master mode
 * |        |          |In the slave mode, the seven most significant bits must be loaded with the MCUu2019s own address
 * |        |          |The I2C hardware will react if any of the addresses are matched.
 * @var I2C_T::DAT
 * Offset: 0x08  I2C DATA Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DAT       |I2C Data Register
 * |        |          |During master or slave transmit mode, data to be transmitted is written to this register
 * |        |          |During master or slave receive mode, data that has been received may be read from this register.
 * @var I2C_T::STATUS
 * Offset: 0x0C  I2C Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |STATUS    |I2C Status Register
 * |        |          |The status register of I2C:
 * |        |          |The three least significant bits are always 0
 * |        |          |The five most significant bits contain the status code
 * |        |          |There are 26 possible status codes
 * |        |          |When I2C_STATUS contains F8H, no serial interrupt is requested
 * |        |          |All other I2C_STATUS values correspond to defined I2C states
 * |        |          |When each of these states is entered, a status interrupt is requested (SI = 1)
 * |        |          |A valid status code is present in I2C_STATUS one PCLK cycle after SI is set by hardware and is still present one PCLK cycle after SI has been reset by software
 * |        |          |In addition, states 00H stands for a Bus Error
 * |        |          |A Bus Error occurs when a START or STOP condition is present at an illegal position in the frame
 * |        |          |Example of illegal position are during the serial transfer of an address byte, a data byte or an acknowledge bit.
 * @var I2C_T::CLKDIV
 * Offset: 0x10  I2C Clock Divided Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DIVIDER   |I2C Clock Divided Register
 * |        |          |The I2C clock rate bits: Data Baud Rate of I2C = PCLK /(4x(I2C_CLKDIV+1)).
 * @var I2C_T::TOCTL
 * Offset: 0x14  I2C Time Out Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TOIF      |Time-out Flag
 * |        |          |0 = No time-out.
 * |        |          |1 = Time-out flag is set by H/W. It can interrupt CPU. Write 1 to clear..
 * |[1]     |TOCDIV4   |Time-out Counter Input Clock Divide by 4
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |        |          |When enabled, the time-out clock is PCLK/4.
 * |[2]     |TOCEN     |Time-out Counter Control Bit
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |        |          |When enabled, the 14 bit time-out counter will start counting when SI is clear
 * |        |          |Setting flag SI to high will reset counter and re-start up counting after SI is cleared.
 * @var I2C_T::ADDR1
 * Offset: 0x18  I2C Slave Address Register1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = Disable General Call Function.
 * |        |          |1 = Enable General Call Function.
 * |[7:1]   |ADDR      |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in master mode
 * |        |          |In the slave mode, the seven most significant bits must be loaded with the MCUu2019s own address
 * |        |          |The I2C hardware will react if any of the addresses are matched.
 * @var I2C_T::ADDR2
 * Offset: 0x1C  I2C Slave Address Register2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = Disable General Call Function.
 * |        |          |1 = Enable General Call Function.
 * |[7:1]   |ADDR      |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in master mode
 * |        |          |In the slave mode, the seven most significant bits must be loaded with the MCUu2019s own address
 * |        |          |The I2C hardware will react if any of the addresses are matched.
 * @var I2C_T::ADDR3
 * Offset: 0x20  I2C Slave Address Register3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = Disable General Call Function.
 * |        |          |1 = Enable General Call Function.
 * |[7:1]   |ADDR      |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in master mode
 * |        |          |In the slave mode, the seven most significant bits must be loaded with the MCUu2019s own address
 * |        |          |The I2C hardware will react if any of the addresses are matched.
 * @var I2C_T::ADDRMSK0
 * Offset: 0x24  I2C Slave Address Mask Register0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |ADDRMSK   |I2C Address Mask Register
 * |        |          |0 = Mask disable.
 * |        |          |1 = Mask enable (the received corresponding address bit is donu2019t care.).
 * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers
 * |        |          |Bits in this field mask the ADDRx registers masking bits from the address comparison.
 * @var I2C_T::ADDRMSK1
 * Offset: 0x28  I2C Slave Address Mask Register1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |ADDRMSK   |I2C Address Mask Register
 * |        |          |0 = Mask disable.
 * |        |          |1 = Mask enable (the received corresponding address bit is donu2019t care.).
 * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers
 * |        |          |Bits in this field mask the ADDRx registers masking bits from the address comparison.
 * @var I2C_T::ADDRMSK2
 * Offset: 0x2C  I2C Slave Address Mask Register2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |ADDRMSK   |I2C Address Mask Register
 * |        |          |0 = Mask disable.
 * |        |          |1 = Mask enable (the received corresponding address bit is donu2019t care.).
 * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers
 * |        |          |Bits in this field mask the ADDRx registers masking bits from the address comparison.
 * @var I2C_T::ADDRMSK3
 * Offset: 0x30  I2C Slave Address Mask Register3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |ADDRMSK   |I2C Address Mask Register
 * |        |          |0 = Mask disable.
 * |        |          |1 = Mask enable (the received corresponding address bit is donu2019t care.).
 * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers
 * |        |          |Bits in this field mask the ADDRx registers masking bits from the address comparison.
 */
    __IO uint32_t CTL;                   /*!< [0x0000] I2C Control Register                                             */
    __IO uint32_t ADDR0;                 /*!< [0x0004] I2C Slave Address Register0                                      */
    __IO uint32_t DAT;                   /*!< [0x0008] I2C DATA Register                                                */
    __I  uint32_t STATUS;                /*!< [0x000c] I2C Status Register                                              */
    __IO uint32_t CLKDIV;                /*!< [0x0010] I2C Clock Divided Register                                       */
    __IO uint32_t TOCTL;                 /*!< [0x0014] I2C Time Out Control Register                                    */
    __IO uint32_t ADDR1;                 /*!< [0x0018] I2C Slave Address Register1                                      */
    __IO uint32_t ADDR2;                 /*!< [0x001c] I2C Slave Address Register2                                      */
    __IO uint32_t ADDR3;                 /*!< [0x0020] I2C Slave Address Register3                                      */
    __IO uint32_t ADDRMSK0;              /*!< [0x0024] I2C Slave Address Mask Register0                                 */
    __IO uint32_t ADDRMSK1;              /*!< [0x0028] I2C Slave Address Mask Register1                                 */
    __IO uint32_t ADDRMSK2;              /*!< [0x002c] I2C Slave Address Mask Register2                                 */
    __IO uint32_t ADDRMSK3;              /*!< [0x0030] I2C Slave Address Mask Register3                                 */

} I2C_T;

/**
    @addtogroup I2C_CONST I2C Bit Field Definition
    Constant Definitions for I2C Controller
@{ */

#define I2C_CTL_AA_Pos                   (2)                                               /*!< I2C_T::CTL: AA Position                */
#define I2C_CTL_AA_Msk                   (0x1ul << I2C_CTL_AA_Pos)                         /*!< I2C_T::CTL: AA Mask                    */

#define I2C_CTL_SI_Pos                   (3)                                               /*!< I2C_T::CTL: SI Position                */
#define I2C_CTL_SI_Msk                   (0x1ul << I2C_CTL_SI_Pos)                         /*!< I2C_T::CTL: SI Mask                    */

#define I2C_CTL_STO_Pos                  (4)                                               /*!< I2C_T::CTL: STO Position               */
#define I2C_CTL_STO_Msk                  (0x1ul << I2C_CTL_STO_Pos)                        /*!< I2C_T::CTL: STO Mask                   */

#define I2C_CTL_STA_Pos                  (5)                                               /*!< I2C_T::CTL: STA Position               */
#define I2C_CTL_STA_Msk                  (0x1ul << I2C_CTL_STA_Pos)                        /*!< I2C_T::CTL: STA Mask                   */

#define I2C_CTL_I2CEN_Pos                (6)                                               /*!< I2C_T::CTL: I2CEN Position             */
#define I2C_CTL_I2CEN_Msk                (0x1ul << I2C_CTL_I2CEN_Pos)                      /*!< I2C_T::CTL: I2CEN Mask                 */

#define I2C_CTL_INTEN_Pos                (7)                                               /*!< I2C_T::CTL: INTEN Position             */
#define I2C_CTL_INTEN_Msk                (0x1ul << I2C_CTL_INTEN_Pos)                      /*!< I2C_T::CTL: INTEN Mask                 */

#define I2C_ADDR0_GC_Pos                 (0)                                               /*!< I2C_T::ADDR0: GC Position              */
#define I2C_ADDR0_GC_Msk                 (0x1ul << I2C_ADDR0_GC_Pos)                       /*!< I2C_T::ADDR0: GC Mask                  */

#define I2C_ADDR0_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR0: ADDR Position            */
#define I2C_ADDR0_ADDR_Msk               (0x7ful << I2C_ADDR0_ADDR_Pos)                    /*!< I2C_T::ADDR0: ADDR Mask                */

#define I2C_DAT_DAT_Pos                  (0)                                               /*!< I2C_T::DAT: DAT Position               */
#define I2C_DAT_DAT_Msk                  (0xfful << I2C_DAT_DAT_Pos)                       /*!< I2C_T::DAT: DAT Mask                   */

#define I2C_STATUS_STATUS_Pos            (0)                                               /*!< I2C_T::STATUS: STATUS Position         */
#define I2C_STATUS_STATUS_Msk            (0xfful << I2C_STATUS_STATUS_Pos)                 /*!< I2C_T::STATUS: STATUS Mask             */

#define I2C_CLKDIV_DIVIDER_Pos           (0)                                               /*!< I2C_T::CLKDIV: DIVIDER Position        */
#define I2C_CLKDIV_DIVIDER_Msk           (0xfful << I2C_CLKDIV_DIVIDER_Pos)                /*!< I2C_T::CLKDIV: DIVIDER Mask            */

#define I2C_TOCTL_TOIF_Pos               (0)                                               /*!< I2C_T::TOCTL: TOIF Position            */
#define I2C_TOCTL_TOIF_Msk               (0x1ul << I2C_TOCTL_TOIF_Pos)                     /*!< I2C_T::TOCTL: TOIF Mask                */

#define I2C_TOCTL_TOCDIV4_Pos            (1)                                               /*!< I2C_T::TOCTL: TOCDIV4 Position         */
#define I2C_TOCTL_TOCDIV4_Msk            (0x1ul << I2C_TOCTL_TOCDIV4_Pos)                  /*!< I2C_T::TOCTL: TOCDIV4 Mask             */

#define I2C_TOCTL_TOCEN_Pos              (2)                                               /*!< I2C_T::TOCTL: TOCEN Position           */
#define I2C_TOCTL_TOCEN_Msk              (0x1ul << I2C_TOCTL_TOCEN_Pos)                    /*!< I2C_T::TOCTL: TOCEN Mask               */

#define I2C_ADDR1_GC_Pos                 (0)                                               /*!< I2C_T::ADDR1: GC Position              */
#define I2C_ADDR1_GC_Msk                 (0x1ul << I2C_ADDR1_GC_Pos)                       /*!< I2C_T::ADDR1: GC Mask                  */

#define I2C_ADDR1_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR1: ADDR Position            */
#define I2C_ADDR1_ADDR_Msk               (0x7ful << I2C_ADDR1_ADDR_Pos)                    /*!< I2C_T::ADDR1: ADDR Mask                */

#define I2C_ADDR2_GC_Pos                 (0)                                               /*!< I2C_T::ADDR2: GC Position              */
#define I2C_ADDR2_GC_Msk                 (0x1ul << I2C_ADDR2_GC_Pos)                       /*!< I2C_T::ADDR2: GC Mask                  */

#define I2C_ADDR2_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR2: ADDR Position            */
#define I2C_ADDR2_ADDR_Msk               (0x7ful << I2C_ADDR2_ADDR_Pos)                    /*!< I2C_T::ADDR2: ADDR Mask                */

#define I2C_ADDR3_GC_Pos                 (0)                                               /*!< I2C_T::ADDR3: GC Position              */
#define I2C_ADDR3_GC_Msk                 (0x1ul << I2C_ADDR3_GC_Pos)                       /*!< I2C_T::ADDR3: GC Mask                  */

#define I2C_ADDR3_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR3: ADDR Position            */
#define I2C_ADDR3_ADDR_Msk               (0x7ful << I2C_ADDR3_ADDR_Pos)                    /*!< I2C_T::ADDR3: ADDR Mask                */

#define I2C_ADDRMSK0_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK0: ADDRMSK Position      */
#define I2C_ADDRMSK0_ADDRMSK_Msk         (0x7ful << I2C_ADDRMSK0_ADDRMSK_Pos)              /*!< I2C_T::ADDRMSK0: ADDRMSK Mask          */

#define I2C_ADDRMSK1_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK1: ADDRMSK Position      */
#define I2C_ADDRMSK1_ADDRMSK_Msk         (0x7ful << I2C_ADDRMSK1_ADDRMSK_Pos)              /*!< I2C_T::ADDRMSK1: ADDRMSK Mask          */

#define I2C_ADDRMSK2_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK2: ADDRMSK Position      */
#define I2C_ADDRMSK2_ADDRMSK_Msk         (0x7ful << I2C_ADDRMSK2_ADDRMSK_Pos)              /*!< I2C_T::ADDRMSK2: ADDRMSK Mask          */

#define I2C_ADDRMSK3_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK3: ADDRMSK Position      */
#define I2C_ADDRMSK3_ADDRMSK_Msk         (0x7ful << I2C_ADDRMSK3_ADDRMSK_Pos)              /*!< I2C_T::ADDRMSK3: ADDRMSK Mask          */

/**@}*/ /* I2C_CONST */
/**@}*/ /* end of I2C register group */


/*---------------------- I2S Interface Controller -------------------------*/
/**
    @addtogroup I2S I2S Interface Controller(I2S)
    Memory Mapped Structure for I2S Controller
@{ */
 
typedef struct
{


/**
 * @var I2S_T::CTL0
 * Offset: 0x00  I2S Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |I2SEN     |I2S Controller Enable Control
 * |        |          |0 = I2S controller Disabled.
 * |        |          |1 = I2S controller Enabled.
 * |[1]     |TXEN      |Transmit Enable Control
 * |        |          |0 = Data transmission Disabled.
 * |        |          |1 = Data transmission Enabled.
 * |[2]     |RXEN      |Receive Enable Control
 * |        |          |0 = Data receiving Disabled.
 * |        |          |1 = Data receiving Enabled.
 * |[3]     |MUTE      |Transmit Mute Enable Control
 * |        |          |0 = Transmit data is shifted from buffer.
 * |        |          |1 = Send zero on transmit channel.
 * |[5:4]   |DATWIDTH  |Data Width
 * |        |          |This bit field is used to define the bit-width of data word in each audio channel
 * |        |          |00 = The bit-width of data word is 8-bit.
 * |        |          |01 = The bit-width of data word is 16-bit.
 * |        |          |10 = The bit-width of data word is 24-bit.
 * |        |          |11 = The bit-width of data word is 32-bit.
 * |[6]     |MONO      |Monaural Data Control
 * |        |          |0 = Data is stereo format.
 * |        |          |1 = Data is monaural format.
 * |        |          |Note: when chip records data, RXLCH (I2S_CTL0[23]) indicates which channel data will be saved if monaural format is selected.
 * |[7]     |ORDER     |Stereo Data Order in FIFO
 * |        |          |In 8-bit/16-bit data width, this bit is used to select whether the even or odd channel data is stored in higher byte
 * |        |          |In 24-bit data width, this is used to select the left/right alignment method of audio data which is stored in data memory consisted of 32-bit FIFO entries.
 * |        |          |0 = Even channel data at high byte in 8-bit/16-bit data width.
 * |        |          |LSB of 24-bit audio data in each channel is aligned to right side in 32-bit FIFO entries.
 * |        |          |1 = Even channel data at low byte.
 * |        |          |MSB of 24-bit audio data in each channel is aligned to left side in 32-bit FIFO entries.
 * |[8]     |SLAVE     |Slave Mode Enable Control
 * |        |          |0 = Master mode.
 * |        |          |1 = Slave mode.
 * |        |          |Note: I2S can operate as master or slave
 * |        |          |For Master mode, I2S_BCLK and I2S_LRCLK pins are output mode and send out bit clock to Audio CODEC chip
 * |        |          |In Slave mode, I2S_BCLK and I2S_LRCLK pins are input mode and I2S_BCLK and I2S_LRCLK signals are received from outer Audio CODEC chip.
 * |[15]    |MCLKEN    |Master Clock Enable Control
 * |        |          |If MCLKEN is set to 1, I2S controller will generate master clock on I2S_MCLK pin for external audio devices.
 * |        |          |0 = Master clock Disabled.
 * |        |          |1 = Master clock Enabled.
 * |[16]    |FRZCDEN   |Force Right Channel Zero Cross Data Option Bit
 * |        |          |If this bit set to 1, when channel data sign bit changes or next shift data bits are all 0 then the channel ZCIF flag in I2S_STATUS1 register is set to 1 and channel data will force zero
 * |        |          |This function is only available in transmit operation.
 * |        |          |0 = Keep channel data, when zero crossing flag on.
 * |        |          |1 = Force channel data to zero, when zero crossing flag on.
 * |[17]    |FLZCDEN   |Force Left Channel Zero Cross Data Option Bit
 * |        |          |If this bit set to 1, when channel data sign bit changes or next shift data bits are all 0 then the channel ZCIF flag in I2S_STATUS1 register is set to 1 and channel data will force zero
 * |        |          |This function is only available in transmit operation.
 * |        |          |0 = Keep channel data, when zero crossing flag on.
 * |        |          |1 = Force channel data to zero, when zero crossing flag on.
 * |[18]    |TXFBCLR   |Transmit FIFO Buffer Clear
 * |        |          |0 = No Effect.
 * |        |          |1 = Clear TX FIFO.
 * |        |          |Note1: Write 1 to clear transmit FIFO, internal pointer is reset to FIFO start point, and TXCNT (I2S_STATUS1[12:8]) returns 0 and transmit FIFO becomes empty but data in transmit FIFO is not changed.
 * |        |          |Note2: This bit is clear by hardware automatically, read it return zero.
 * |[19]    |RXFBCLR   |Receive FIFO Buffer Clear
 * |        |          |0 = No Effect.
 * |        |          |1 = Clear RX FIFO.
 * |        |          |Note1: Write 1 to clear receive FIFO, internal pointer is reset to FIFO start point, and RXCNT (I2S_STATUS1[20:16]) returns 0 and receive FIFO becomes empty.
 * |        |          |Note2: This bit is cleared by hardware automatically, read it return zero.
 * |[20]    |TXPDMAEN  |Transmit PDMA Enable Control
 * |        |          |0 = Transmit PDMA function Disabled.
 * |        |          |1 = Transmit PDMA function Enabled.
 * |[21]    |RXPDMAEN  |Receive PDMA Enable Control
 * |        |          |0 = Receiver PDMA function Disabled.
 * |        |          |1 = Receiver PDMA function Enabled.
 * |[23]    |RXLCH     |Receive Left Channel Enable Control
 * |        |          |When monaural format is selected (MONO = 1), I2S will receive channel1 data if RXLCH is set to 0, and receive channel0 data if RXLCH is set to 1.
 * |        |          |0 = Receives channel1 data in MONO mode.
 * |        |          |1 = Receives channel0 data in MONO mode.
 * |[26:24] |FORMAT    |Data Format Selection
 * |        |          |000 = I2S standard data format.
 * |        |          |001 = I2S with MSB justified.
 * |        |          |010 = I2S with LSB justified.
 * |        |          |011 = Reserved. Do not use.
 * |        |          |100 = PCM standard data format.
 * |        |          |101 = PCM with MSB justified.
 * |        |          |110 = PCM with LSB justified.
 * |        |          |111 = Reserved. Do not use.
 * |[27]    |PCMSYNC   |PCM Synchronization Pulse Length Selection
 * |        |          |This bit field is used to select the high pulse length of frame synchronization signal in PCM protocol
 * |        |          |0 = One BCLK period.
 * |        |          |1 = One channel period.
 * |        |          |Note: This bit is only available in master mode
 * |[29:28] |CHWIDTH   |Channel Width
 * |        |          |This bit fields are used to define the length of audio channel
 * |        |          |If CHWIDTH < DATWIDTH, the hardware will set the real channel length as the bit-width of audio data which is defined by DATWIDTH.
 * |        |          |00 = The bit-width of each audio channel is 8-bit.
 * |        |          |01 = The bit-width of each audio channel is 16-bit.
 * |        |          |10 = The bit-width of each audio channel is 24-bit.
 * |        |          |11 = The bit-width of each audio channel is 32-bit.
 * @var I2S_T::CLKDIV
 * Offset: 0x04  I2S Clock Divider Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[6:0]   |MCLKDIV   |Master Clock Divider
 * |        |          |If chip external crystal frequency is (2xMCLKDIV)*256fs then software can program these bits to generate 256fs clock frequency to audio codec chip
 * |        |          |If MCLKDIV is set to 0, MCLK is the same as external clock input.
 * |        |          |For example, sampling rate is 24 kHz and chip external crystal clock is 12.288 MHz, set MCLKDIV = 1.
 * |        |          |F_MCLK = F_I2SCLK/(2x(MCLKDIV)) (When MCLKDIV is >= 1 ).
 * |        |          |F_MCLK = F_I2SCLK (When MCLKDIV is set to 0 ).
 * |        |          |Note: F_MCLK is the frequency of MCLK, and F_I2SCLK is the frequency of the I2S_CLK
 * |[17:8]  |BCLKDIV   |Bit Clock Divider
 * |        |          |The I2S controller will generate bit clock in Master mode
 * |        |          |Software can program these bit fields to generate sampling rate clock frequency.
 * |        |          |F_BCLK= F_I2SCLK / (2*(BCLKDIV + 1)).
 * |        |          |Note: F_BCLK is the frequency of BCLK and F_I2SCLK is the frequency of I2S_CLK
 * @var I2S_T::IEN
 * Offset: 0x08  I2S Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RXUDFIEN  |Receive FIFO Underflow Interrupt E Enable Control
 * |        |          |0 = Interrupt Disabled.
 * |        |          |1 = Interrupt Enabled.
 * |        |          |Note: If software reads receive FIFO when it is empty then RXUDIF (I2S_STATUS0[8]) flag is set to 1.
 * |[1]     |RXOVFIEN  |Receive FIFO Overflow Interrupt Enable Control
 * |        |          |0 = Interrupt Disabled.
 * |        |          |1 = Interrupt Enabled.
 * |        |          |Note: Interrupt occurs if this bit is set to 1 and RXOVIF (I2S_STATUS0[9]) flag is set to 1
 * |[2]     |RXTHIEN   |Receive FIFO Threshold Level Interrupt Enable Control
 * |        |          |0 = Interrupt Disabled.
 * |        |          |1 = Interrupt Enabled.
 * |        |          |Note: When data word in receive FIFO is equal or higher than RXTH (I2S_CTL1[19:16]) and the RXTHIF (I2S_STATUS0[10]) bit is set to 1
 * |        |          |If RXTHIEN bit is enabled, interrupt occur.
 * |[8]     |TXUDFIEN  |Transmit FIFO Underflow Interrupt Enable Control
 * |        |          |0 = Interrupt Disabled.
 * |        |          |1 = Interrupt Enabled.
 * |        |          |Note: Interrupt occur if this bit is set to 1 and TXUDIF (I2S_STATUS0[16]) flag is set to 1.
 * |[9]     |TXOVFIEN  |Transmit FIFO Overflow Interrupt Enable Control
 * |        |          |0 = Interrupt Disabled.
 * |        |          |1 = Interrupt Enabled.
 * |        |          |Note: Interrupt occurs if this bit is set to 1 and TXOVIF (I2S_STATUS0[17]) flag is set to 1
 * |[10]    |TXTHIEN   |Transmit FIFO Threshold Level Interrupt Enable Control
 * |        |          |0 = Interrupt Disabled.
 * |        |          |1 = Interrupt Enabled.
 * |        |          |Note: Interrupt occurs if this bit is set to 1 and data words in transmit FIFO is less than TXTH (I2S_CTL1[11:8]).
 * |[16]    |CH0ZCIEN  |Channel0 Zero-cross Interrupt Enable Control
 * |        |          |0 = Interrupt Disabled.
 * |        |          |1 = Interrupt Enabled.
 * |        |          |Note1: Interrupt occurs if this bit is set to 1 and channel0 zero-cross
 * |        |          |Note2: Channel0 also means left audio channel while I2S (FORMAT[2]=0) or 2-channel PCM mode.
 * |[17]    |CH1ZCIEN  |Channel1 Zero-cross Interrupt Enable Control
 * |        |          |0 = Interrupt Disabled.
 * |        |          |1 = Interrupt Enabled.
 * |        |          |Note1: Interrupt occurs if this bit is set to 1 and channel1 zero-cross
 * |        |          |Note2: Channel1 also means right audio channel while I2S (FORMAT[2]=0) or 2-channel PCM mode.
 * @var I2S_T::STATUS0
 * Offset: 0x0C  I2S Status Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |I2SINT    |I2S Interrupt Flag (Read Only)
 * |        |          |0 = No I2S interrupt.
 * |        |          |1 = I2S interrupt.
 * |        |          |Note: It is wire-OR of I2STXINT and I2SRXINT bits.
 * |[1]     |I2SRXINT  |I2S Receive Interrupt (Read Only)
 * |        |          |0 = No receive interrupt.
 * |        |          |1 = Receive interrupt.
 * |[2]     |I2STXINT  |I2S Transmit Interrupt (Read Only)
 * |        |          |0 = No transmit interrupt.
 * |        |          |1 = Transmit interrupt.
 * |[3]     |DATACH    |Transmission Data Channel (Read Only)
 * |        |          |This bit fields are used to indicate which audio channel is current transmit data belong.
 * |        |          |0 = channel0 (means left channel while 2-channel I2S/PCM mode).
 * |        |          |1 = channel1 (means right channel while 2-channel I2S/PCM mode).
 * |[8]     |RXUDIF    |Receive FIFO Underflow Interrupt Flag
 * |        |          |0 = No underflow occur.
 * |        |          |1 = Underflow occur.
 * |        |          |Note1: When receive FIFO is empty, and software reads the receive FIFO again
 * |        |          |This bit will be set to 1, and it indicates underflow situation occurs.
 * |        |          |Note2: Write 1 to clear this bit to zero
 * |[9]     |RXOVIF    |Receive FIFO Overflow Interrupt Flag
 * |        |          |0 = No overflow occur.
 * |        |          |1 = Overflow occur.
 * |        |          |Note1: When receive FIFO is full and receive hardware attempt to write data into receive FIFO then this bit is set to 1, data in 1st buffer is overwrote.
 * |        |          |Note2: Write 1 to clear this bit to 0.
 * |[10]    |RXTHIF    |Receive FIFO Threshold Interrupt Flag (Read Only)
 * |        |          |0 = Data word(s) in FIFO is not higher than threshold level.
 * |        |          |1 = Data word(s) in FIFO is higher than threshold level.
 * |        |          |Note: When data word(s) in receive FIFO is higher than threshold value set in RXTH (I2S_CTL1[19:16]) the RXTHIF bit becomes to 1
 * |        |          |It keeps at 1 till RXCNT (I2S_STATUS1[20:16]) is not higher than RXTH (I2S_CTL1[19:16]) after software read RXFIFO register.
 * |[11]    |RXFULL    |Receive FIFO Full (Read Only)
 * |        |          |0 = Not full.
 * |        |          |1 = Full.
 * |        |          |Note: This bit reflects data words number in receive FIFO is 12.
 * |[12]    |RXEMPTY   |Receive FIFO Empty (Read Only)
 * |        |          |0 = Not empty.
 * |        |          |1 = Empty.
 * |        |          |Note: This bit reflects data words number in receive FIFO is zero
 * |[16]    |TXUDIF    |Transmit FIFO Underflow Interrupt Flag
 * |        |          |0 = No underflow.
 * |        |          |1 = Underflow.
 * |        |          |Note1: This bit will be set to 1 when shift logic hardware read data from transmitting FIFO and the filling data level in transmitting FIFO is not enough for one audio frame.
 * |        |          |Note2: Write 1 to clear this bit to 0.
 * |[17]    |TXOVIF    |Transmit FIFO Overflow Interrupt Flag
 * |        |          |0 = No overflow.
 * |        |          |1 = Overflow.
 * |        |          |Note1: Write data to transmit FIFO when it is full and this bit set to 1
 * |        |          |Note2: Write 1 to clear this bit to 0.
 * |[18]    |TXTHIF    |Transmit FIFO Threshold Interrupt Flag (Read Only)
 * |        |          |0 = Data word(s) in FIFO is higher than threshold level.
 * |        |          |1 = Data word(s) in FIFO is equal or lower than threshold level.
 * |        |          |Note: When data word(s) in transmit FIFO is equal or lower than threshold value set in TXTH (I2S_CTL1[11:8]) the TXTHIF bit becomes to 1
 * |        |          |It keeps at 1 till TXCNT (I2S_STATUS1[12:8]) is higher than TXTH (I2S_CTL1[11:8]) after software write TXFIFO register.
 * |[19]    |TXFULL    |Transmit FIFO Full (Read Only)
 * |        |          |This bit reflect data word number in transmit FIFO is 12
 * |        |          |0 = Not full.
 * |        |          |1 = Full.
 * |[20]    |TXEMPTY   |Transmit FIFO Empty (Read Only)
 * |        |          |This bit reflect data word number in transmit FIFO is zero
 * |        |          |0 = Not empty.
 * |        |          |1 = Empty.
 * |[21]    |TXBUSY    |Transmit Busy (Read Only)
 * |        |          |0 = Transmit shift buffer is empty.
 * |        |          |1 = Transmit shift buffer is busy.
 * |        |          |Note: This bit is cleared to 0 when all data in transmit FIFO and shift buffer is shifted out
 * |        |          |And set to 1 when 1st data is load to shift buffer
 * @var I2S_T::TXFIFO
 * Offset: 0x10  I2S Transmit FIFO Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |TXFIFO    |Transmit FIFO Bits
 * |        |          |I2S contains 16 words (16x32 bit) data buffer for data transmit
 * |        |          |Write data to this register to prepare data for transmit
 * |        |          |The remaining word number is indicated by TXCNT (I2S_STATUS1[12:8]).
 * @var I2S_T::RXFIFO
 * Offset: 0x14  I2S Receive FIFO Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |RXFIFO    |Receive FIFO Bits
 * |        |          |I2S contains 16 words (16x32 bit) data buffer for data receive
 * |        |          |Read this register to get data in FIFO
 * |        |          |The remaining data word number is indicated by RXCNT (I2S_STATUS1[20:16]).
 * @var I2S_T::CTL1
 * Offset: 0x20  I2S Control Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CH0ZCEN   |Channel0 Zero-cross Detection Enable Control
 * |        |          |0 = channel0 zero-cross detect Disabled.
 * |        |          |1 = channel0 zero-cross detect Enabled.
 * |        |          |Note1: Channel0 also means left audio channel while I2S (FORMAT[2]=0) or 2-channel PCM mode.
 * |        |          |Note2: If this bit is set to 1, when channel0 data sign bit change or next shift data bits are all zero then CH0ZCIF(I2S_STATUS1[0]) flag is set to 1.
 * |        |          |Note3: If CH0ZCIF Flag is set to 1, the channel0 will be mute.
 * |[1]     |CH1ZCEN   |Channel1 Zero-cross Detect Enable Control
 * |        |          |0 = channel1 zero-cross detect Disabled.
 * |        |          |1 = channel1 zero-cross detect Enabled.
 * |        |          |Note1: Channel1 also means right audio channel while I2S (FORMAT[2]=0) or 2-channel PCM mode.
 * |        |          |Note2: If this bit is set to 1, when channel1 data sign bit change or next shift data bits are all zero then CH1ZCIF(I2S_STATUS1[1]) flag is set to 1.
 * |        |          |Note3: If CH1ZCIF Flag is set to 1, the channel1 will be mute.
 * |[11:8]  |TXTH      |Transmit FIFO Threshold Level
 * |        |          |0000 = 0 data word in transmit FIFO.
 * |        |          |0001 = 1 data word in transmit FIFO.
 * |        |          |0010 = 2 data words in transmit FIFO.
 * |        |          |....
 * |        |          |1110 = 14 data words in transmit FIFO.
 * |        |          |1111 = 15 data words in transmit FIFO.
 * |        |          |Note: If remain data word number in transmit FIFO is the same or less than threshold level then TXTHIF (I2S_STATUS0[18]) flag is set.
 * |[19:16] |RXTH      |Receive FIFO Threshold Level
 * |        |          |0000 = 1 data word in receive FIFO.
 * |        |          |0001 = 2 data words in receive FIFO.
 * |        |          |0010 = 3 data words in receive FIFO.
 * |        |          |u2026.
 * |        |          |1110 = 15 data words in receive FIFO.
 * |        |          |1111 = 16 data words in receive FIFO.
 * |        |          |Note: When received data word number in receive buffer is greater than threshold level then RXTHIF (I2S_STATUS0[10]) flag is set.
 * |[24]    |PBWIDTH   |Peripheral Bus Data Width Selection
 * |        |          |This bit is used to choice the available data width of APB bus
 * |        |          |It must be set to 1 while PDMA function is enable and it is set to 16-bit transmission mode
 * |        |          |0 = 32 bits data width.
 * |        |          |1 = 16 bits data width.
 * |        |          |Note1: If PBWIDTH=1, the low 16 bits of 32-bit data bus are available.
 * |        |          |Note2: If PBWIDTH=1, the transmitting FIFO level will be increased after two FIFO write operations.
 * |        |          |Note3: If PBWIDTH=1, the receiving FIFO level will be decreased after two FIFO read operations.
 * |[25]    |PB16ORD   |FIFO Read/Write Order in 16-bit Width of Peripheral Bus
 * |        |          |When PBWIDTH = 1, the data FIFO will be increased or decreased by two peripheral bus access
 * |        |          |This bit is used to select the order of FIFO access operations to meet the 32-bit transmitting/receiving FIFO entries.
 * |        |          |0 = Low 16-bit read/write access first.
 * |        |          |1 = High 16-bit read/write access first.
 * |        |          |Note: This bit is available while PBWIDTH = 1.
 * @var I2S_T::STATUS1
 * Offset: 0x24  I2S Status Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CH0ZCIF   |Channel0 Zero-cross Interrupt Flag
 * |        |          |It indicates channel0 next sample data sign bit is changed or all data bits are zero.
 * |        |          |0 = No zero-cross in channel0.
 * |        |          |1 = Channel0 zero-cross is detected.
 * |        |          |Note1: Write 1 to clear this bit to 0.
 * |        |          |Note2: Channel0 also means left audio channel while I2S (FORMAT[2]=0) or 2-channel PCM mode.
 * |[1]     |CH1ZCIF   |Channel1 Zero-cross Interrupt Flag
 * |        |          |It indi
 zero-cross in channel1.
 * |        |          |1 = Channel1 zero-cross is detected.
 * |        |          |Note1: Write 1 to clear this bit to 0.
 * |        |          |Note2: Channel1 also means right audio channel while I2S (FORMAT[2]=0) or 2-channel PCM mode.
 * |[12:8]  |TXCNT     |Transmit FIFO Level (Read Only)
 * |        |          |These bits indicate the number of available entries in transmit FIFO
 * |        |          |00000 = No data.
 * |        |          |00001 = 1 word in transmit FIFO.
 * |        |          |00010 = 2 words in transmit FIFO.
 * |        |          |u2026.
 * |        |          |01110 = 14 words in transmit FIFO.
 * |        |          |01111 = 15 words in transmit FIFO.
 * |        |          |10000 = 16 words in transmit FIFO.
 * |        |          |Others are reserved. Do not use.
 * |[20:16] |RXCNT     |Receive FIFO Level (Read Only)
 * |        |          |These bits indicate the number of available entries in receive FIFO
 * |        |          |00000 = No data.
 * |        |          |00001 = 1 word in receive FIFO.
 * |        |          |00010 = 2 words in receive FIFO.
 * |        |          |u2026.
 * |        |          |01110 = 14 words in receive FIFO.
 * |        |          |01111 = 15 words in receive FIFO.
 * |        |          |10000 = 16 words in receive FIFO.
 * |        |          |Others are reserved. Do not use.
 */
    __IO uint32_t CTL0;                  /*!< [0x0000] I2S Control Register 0                                           */
    __IO uint32_t CLKDIV;                /*!< [0x0004] I2S Clock Divider Register                                       */
    __IO uint32_t IEN;                   /*!< [0x0008] I2S Interrupt Enable Register                                    */
    __IO uint32_t STATUS0;               /*!< [0x000c] I2S Status Register 0                                            */
    __O  uint32_t TXFIFO;                /*!< [0x0010] I2S Transmit FIFO Register                                       */
    __I  uint32_t RXFIFO;                /*!< [0x0014] I2S Receive FIFO Register                                        */
    __I  uint32_t RESERVE0[2];
    __IO uint32_t CTL1;                  /*!< [0x0020] I2S Control Register 1                                           */
    __IO uint32_t STATUS1;               /*!< [0x0024] I2S Status Register 1                                            */

} I2S_T;

/**
    @addtogroup I2S_CONST I2S Bit Field Definition
    Constant Definitions for I2S Controller
@{ */

#define I2S_CTL0_I2SEN_Pos               (0)                                               /*!< I2S_T::CTL0: I2SEN Position            */
#define I2S_CTL0_I2SEN_Msk               (0x1ul << I2S_CTL0_I2SEN_Pos)                     /*!< I2S_T::CTL0: I2SEN Mask                */

#define I2S_CTL0_TXEN_Pos                (1)                                               /*!< I2S_T::CTL0: TXEN Position             */
#define I2S_CTL0_TXEN_Msk                (0x1ul << I2S_CTL0_TXEN_Pos)                      /*!< I2S_T::CTL0: TXEN Mask                 */

#define I2S_CTL0_RXEN_Pos                (2)                                               /*!< I2S_T::CTL0: RXEN Position             */
#define I2S_CTL0_RXEN_Msk                (0x1ul << I2S_CTL0_RXEN_Pos)                      /*!< I2S_T::CTL0: RXEN Mask                 */

#define I2S_CTL0_MUTE_Pos                (3)                                               /*!< I2S_T::CTL0: MUTE Position             */
#define I2S_CTL0_MUTE_Msk                (0x1ul << I2S_CTL0_MUTE_Pos)                      /*!< I2S_T::CTL0: MUTE Mask                 */

#define I2S_CTL0_DATWIDTH_Pos            (4)                                               /*!< I2S_T::CTL0: DATWIDTH Position         */
#define I2S_CTL0_DATWIDTH_Msk            (0x3ul << I2S_CTL0_DATWIDTH_Pos)                  /*!< I2S_T::CTL0: DATWIDTH Mask             */

#define I2S_CTL0_MONO_Pos                (6)                                               /*!< I2S_T::CTL0: MONO Position             */
#define I2S_CTL0_MONO_Msk                (0x1ul << I2S_CTL0_MONO_Pos)                      /*!< I2S_T::CTL0: MONO Mask                 */

#define I2S_CTL0_ORDER_Pos               (7)                                               /*!< I2S_T::CTL0: ORDER Position            */
#define I2S_CTL0_ORDER_Msk               (0x1ul << I2S_CTL0_ORDER_Pos)                     /*!< I2S_T::CTL0: ORDER Mask                */

#define I2S_CTL0_SLAVE_Pos               (8)                                               /*!< I2S_T::CTL0: SLAVE Position            */
#define I2S_CTL0_SLAVE_Msk               (0x1ul << I2S_CTL0_SLAVE_Pos)                     /*!< I2S_T::CTL0: SLAVE Mask                */

#define I2S_CTL0_MCLKEN_Pos              (15)                                              /*!< I2S_T::CTL0: MCLKEN Position           */
#define I2S_CTL0_MCLKEN_Msk              (0x1ul << I2S_CTL0_MCLKEN_Pos)                    /*!< I2S_T::CTL0: MCLKEN Mask               */

#define I2S_CTL0_FRZCDEN_Pos             (16)                                              /*!< I2S_T::CTL0: FRZCDEN Position          */
#define I2S_CTL0_FRZCDEN_Msk             (0x1ul << I2S_CTL0_FRZCDEN_Pos)                   /*!< I2S_T::CTL0: FRZCDEN Mask              */

#define I2S_CTL0_FLZCDEN_Pos             (17)                                              /*!< I2S_T::CTL0: FLZCDEN Position          */
#define I2S_CTL0_FLZCDEN_Msk             (0x1ul << I2S_CTL0_FLZCDEN_Pos)                   /*!< I2S_T::CTL0: FLZCDEN Mask              */

#define I2S_CTL0_TXFBCLR_Pos             (18)                                              /*!< I2S_T::CTL0: TXFBCLR Position          */
#define I2S_CTL0_TXFBCLR_Msk             (0x1ul << I2S_CTL0_TXFBCLR_Pos)                   /*!< I2S_T::CTL0: TXFBCLR Mask              */

#define I2S_CTL0_RXFBCLR_Pos             (19)                                              /*!< I2S_T::CTL0: RXFBCLR Position          */
#define I2S_CTL0_RXFBCLR_Msk             (0x1ul << I2S_CTL0_RXFBCLR_Pos)                   /*!< I2S_T::CTL0: RXFBCLR Mask              */

#define I2S_CTL0_TXPDMAEN_Pos            (20)                                              /*!< I2S_T::CTL0: TXPDMAEN Position         */
#define I2S_CTL0_TXPDMAEN_Msk            (0x1ul << I2S_CTL0_TXPDMAEN_Pos)                  /*!< I2S_T::CTL0: TXPDMAEN Mask             */

#define I2S_CTL0_RXPDMAEN_Pos            (21)                                              /*!< I2S_T::CTL0: RXPDMAEN Position         */
#define I2S_CTL0_RXPDMAEN_Msk            (0x1ul << I2S_CTL0_RXPDMAEN_Pos)                  /*!< I2S_T::CTL0: RXPDMAEN Mask             */

#define I2S_CTL0_RXLCH_Pos               (23)                                              /*!< I2S_T::CTL0: RXLCH Position            */
#define I2S_CTL0_RXLCH_Msk               (0x1ul << I2S_CTL0_RXLCH_Pos)                     /*!< I2S_T::CTL0: RXLCH Mask                */

#define I2S_CTL0_FORMAT_Pos              (24)                                              /*!< I2S_T::CTL0: FORMAT Position           */
#define I2S_CTL0_FORMAT_Msk              (0x7ul << I2S_CTL0_FORMAT_Pos)                    /*!< I2S_T::CTL0: FORMAT Mask               */

#define I2S_CTL0_PCMSYNC_Pos             (27)                                              /*!< I2S_T::CTL0: PCMSYNC Position          */
#define I2S_CTL0_PCMSYNC_Msk             (0x1ul << I2S_CTL0_PCMSYNC_Pos)                   /*!< I2S_T::CTL0: PCMSYNC Mask              */

#define I2S_CTL0_CHWIDTH_Pos             (28)                                              /*!< I2S_T::CTL0: CHWIDTH Position          */
#define I2S_CTL0_CHWIDTH_Msk             (0x3ul << I2S_CTL0_CHWIDTH_Pos)                   /*!< I2S_T::CTL0: CHWIDTH Mask              */

#define I2S_CLKDIV_MCLKDIV_Pos           (0)                                               /*!< I2S_T::CLKDIV: MCLKDIV Position        */
#define I2S_CLKDIV_MCLKDIV_Msk           (0x7ful << I2S_CLKDIV_MCLKDIV_Pos)                /*!< I2S_T::CLKDIV: MCLKDIV Mask            */

#define I2S_CLKDIV_BCLKDIV_Pos           (8)                                               /*!< I2S_T::CLKDIV: BCLKDIV Position        */
#define I2S_CLKDIV_BCLKDIV_Msk           (0x3fful << I2S_CLKDIV_BCLKDIV_Pos)               /*!< I2S_T::CLKDIV: BCLKDIV Mask            */

#define I2S_IEN_RXUDFIEN_Pos             (0)                                               /*!< I2S_T::IEN: RXUDFIEN Position          */
#define I2S_IEN_RXUDFIEN_Msk             (0x1ul << I2S_IEN_RXUDFIEN_Pos)                   /*!< I2S_T::IEN: RXUDFIEN Mask              */

#define I2S_IEN_RXOVFIEN_Pos             (1)                                               /*!< I2S_T::IEN: RXOVFIEN Position          */
#define I2S_IEN_RXOVFIEN_Msk             (0x1ul << I2S_IEN_RXOVFIEN_Pos)                   /*!< I2S_T::IEN: RXOVFIEN Mask              */

#define I2S_IEN_RXTHIEN_Pos              (2)                                               /*!< I2S_T::IEN: RXTHIEN Position           */
#define I2S_IEN_RXTHIEN_Msk              (0x1ul << I2S_IEN_RXTHIEN_Pos)                    /*!< I2S_T::IEN: RXTHIEN Mask               */

#define I2S_IEN_TXUDFIEN_Pos             (8)                                               /*!< I2S_T::IEN: TXUDFIEN Position          */
#define I2S_IEN_TXUDFIEN_Msk             (0x1ul << I2S_IEN_TXUDFIEN_Pos)                   /*!< I2S_T::IEN: TXUDFIEN Mask              */

#define I2S_IEN_TXOVFIEN_Pos             (9)                                               /*!< I2S_T::IEN: TXOVFIEN Position          */
#define I2S_IEN_TXOVFIEN_Msk             (0x1ul << I2S_IEN_TXOVFIEN_Pos)                   /*!< I2S_T::IEN: TXOVFIEN Mask              */

#define I2S_IEN_TXTHIEN_Pos              (10)                                              /*!< I2S_T::IEN: TXTHIEN Position           */
#define I2S_IEN_TXTHIEN_Msk              (0x1ul << I2S_IEN_TXTHIEN_Pos)                    /*!< I2S_T::IEN: TXTHIEN Mask               */

#define I2S_IEN_CH0ZCIEN_Pos             (16)                                              /*!< I2S_T::IEN: CH0ZCIEN Position          */
#define I2S_IEN_CH0ZCIEN_Msk             (0x1ul << I2S_IEN_CH0ZCIEN_Pos)                   /*!< I2S_T::IEN: CH0ZCIEN Mask              */

#define I2S_IEN_CH1ZCIEN_Pos             (17)                                              /*!< I2S_T::IEN: CH1ZCIEN Position          */
#define I2S_IEN_CH1ZCIEN_Msk             (0x1ul << I2S_IEN_CH1ZCIEN_Pos)                   /*!< I2S_T::IEN: CH1ZCIEN Mask              */

#define I2S_STATUS0_I2SINT_Pos           (0)                                               /*!< I2S_T::STATUS0: I2SINT Position        */
#define I2S_STATUS0_I2SINT_Msk           (0x1ul << I2S_STATUS0_I2SINT_Pos)                 /*!< I2S_T::STATUS0: I2SINT Mask            */

#define I2S_STATUS0_I2SRXINT_Pos         (1)                                               /*!< I2S_T::STATUS0: I2SRXINT Position      */
#define I2S_STATUS0_I2SRXINT_Msk         (0x1ul << I2S_STATUS0_I2SRXINT_Pos)               /*!< I2S_T::STATUS0: I2SRXINT Mask          */

#define I2S_STATUS0_I2STXINT_Pos         (2)                                               /*!< I2S_T::STATUS0: I2STXINT Position      */
#define I2S_STATUS0_I2STXINT_Msk         (0x1ul << I2S_STATUS0_I2STXINT_Pos)               /*!< I2S_T::STATUS0: I2STXINT Mask          */

#define I2S_STATUS0_DATACH_Pos           (3)                                               /*!< I2S_T::STATUS0: DATACH Position        */
#define I2S_STATUS0_DATACH_Msk           (0x1ul << I2S_STATUS0_DATACH_Pos)                 /*!< I2S_T::STATUS0: DATACH Mask            */

#define I2S_STATUS0_RXUDIF_Pos           (8)                                               /*!< I2S_T::STATUS0: RXUDIF Position        */
#define I2S_STATUS0_RXUDIF_Msk           (0x1ul << I2S_STATUS0_RXUDIF_Pos)                 /*!< I2S_T::STATUS0: RXUDIF Mask            */

#define I2S_STATUS0_RXOVIF_Pos           (9)                                               /*!< I2S_T::STATUS0: RXOVIF Position        */
#define I2S_STATUS0_RXOVIF_Msk           (0x1ul << I2S_STATUS0_RXOVIF_Pos)                 /*!< I2S_T::STATUS0: RXOVIF Mask            */

#define I2S_STATUS0_RXTHIF_Pos           (10)                                              /*!< I2S_T::STATUS0: RXTHIF Position        */
#define I2S_STATUS0_RXTHIF_Msk           (0x1ul << I2S_STATUS0_RXTHIF_Pos)                 /*!< I2S_T::STATUS0: RXTHIF Mask            */

#define I2S_STATUS0_RXFULL_Pos           (11)                                              /*!< I2S_T::STATUS0: RXFULL Position        */
#define I2S_STATUS0_RXFULL_Msk           (0x1ul << I2S_STATUS0_RXFULL_Pos)                 /*!< I2S_T::STATUS0: RXFULL Mask            */

#define I2S_STATUS0_RXEMPTY_Pos          (12)                                              /*!< I2S_T::STATUS0: RXEMPTY Position       */
#define I2S_STATUS0_RXEMPTY_Msk          (0x1ul << I2S_STATUS0_RXEMPTY_Pos)                /*!< I2S_T::STATUS0: RXEMPTY Mask           */

#define I2S_STATUS0_TXUDIF_Pos           (16)                                              /*!< I2S_T::STATUS0: TXUDIF Position        */
#define I2S_STATUS0_TXUDIF_Msk           (0x1ul << I2S_STATUS0_TXUDIF_Pos)                 /*!< I2S_T::STATUS0: TXUDIF Mask            */

#define I2S_STATUS0_TXOVIF_Pos           (17)                                              /*!< I2S_T::STATUS0: TXOVIF Position        */
#define I2S_STATUS0_TXOVIF_Msk           (0x1ul << I2S_STATUS0_TXOVIF_Pos)                 /*!< I2S_T::STATUS0: TXOVIF Mask            */

#define I2S_STATUS0_TXTHIF_Pos           (18)                                              /*!< I2S_T::STATUS0: TXTHIF Position        */
#define I2S_STATUS0_TXTHIF_Msk           (0x1ul << I2S_STATUS0_TXTHIF_Pos)                 /*!< I2S_T::STATUS0: TXTHIF Mask            */

#define I2S_STATUS0_TXFULL_Pos           (19)                                              /*!< I2S_T::STATUS0: TXFULL Position        */
#define I2S_STATUS0_TXFULL_Msk           (0x1ul << I2S_STATUS0_TXFULL_Pos)                 /*!< I2S_T::STATUS0: TXFULL Mask            */

#define I2S_STATUS0_TXEMPTY_Pos          (20)                                              /*!< I2S_T::STATUS0: TXEMPTY Position       */
#define I2S_STATUS0_TXEMPTY_Msk          (0x1ul << I2S_STATUS0_TXEMPTY_Pos)                /*!< I2S_T::STATUS0: TXEMPTY Mask           */

#define I2S_STATUS0_TXBUSY_Pos           (21)                                              /*!< I2S_T::STATUS0: TXBUSY Position        */
#define I2S_STATUS0_TXBUSY_Msk           (0x1ul << I2S_STATUS0_TXBUSY_Pos)                 /*!< I2S_T::STATUS0: TXBUSY Mask            */

#define I2S_TXFIFO_TXFIFO_Pos            (0)                                               /*!< I2S_T::TXFIFO: TXFIFO Position         */
#define I2S_TXFIFO_TXFIFO_Msk            (0xfffffffful << I2S_TXFIFO_TXFIFO_Pos)           /*!< I2S_T::TXFIFO: TXFIFO Mask             */

#define I2S_RXFIFO_RXFIFO_Pos            (0)                                               /*!< I2S_T::RXFIFO: RXFIFO Position         */
#define I2S_RXFIFO_RXFIFO_Msk            (0xfffffffful << I2S_RXFIFO_RXFIFO_Pos)           /*!< I2S_T::RXFIFO: RXFIFO Mask             */

#define I2S_CTL1_CH0ZCEN_Pos             (0)                                               /*!< I2S_T::CTL1: CH0ZCEN Position          */
#define I2S_CTL1_CH0ZCEN_Msk             (0x1ul << I2S_CTL1_CH0ZCEN_Pos)                   /*!< I2S_T::CTL1: CH0ZCEN Mask              */

#define I2S_CTL1_CH1ZCEN_Pos             (1)                                               /*!< I2S_T::CTL1: CH1ZCEN Position          */
#define I2S_CTL1_CH1ZCEN_Msk             (0x1ul << I2S_CTL1_CH1ZCEN_Pos)                   /*!< I2S_T::CTL1: CH1ZCEN Mask              */

#define I2S_CTL1_TXTH_Pos                (8)                                               /*!< I2S_T::CTL1: TXTH Position             */
#define I2S_CTL1_TXTH_Msk                (0xful << I2S_CTL1_TXTH_Pos)                      /*!< I2S_T::CTL1: TXTH Mask                 */

#define I2S_CTL1_RXTH_Pos                (16)                                              /*!< I2S_T::CTL1: RXTH Position             */
#define I2S_CTL1_RXTH_Msk                (0xful << I2S_CTL1_RXTH_Pos)                      /*!< I2S_T::CTL1: RXTH Mask                 */

#define I2S_CTL1_PBWIDTH_Pos             (24)                                              /*!< I2S_T::CTL1: PBWIDTH Position          */
#define I2S_CTL1_PBWIDTH_Msk             (0x1ul << I2S_CTL1_PBWIDTH_Pos)                   /*!< I2S_T::CTL1: PBWIDTH Mask              */

#define I2S_CTL1_PB16ORD_Pos             (25)                                              /*!< I2S_T::CTL1: PB16ORD Position          */
#define I2S_CTL1_PB16ORD_Msk             (0x1ul << I2S_CTL1_PB16ORD_Pos)                   /*!< I2S_T::CTL1: PB16ORD Mask              */

#define I2S_STATUS1_CH0ZCIF_Pos          (0)                                               /*!< I2S_T::STATUS1: CH0ZCIF Position       */
#define I2S_STATUS1_CH0ZCIF_Msk          (0x1ul << I2S_STATUS1_CH0ZCIF_Pos)                /*!< I2S_T::STATUS1: CH0ZCIF Mask           */

#define I2S_STATUS1_CH1ZCIF_Pos          (1)                                               /*!< I2S_T::STATUS1: CH1ZCIF Position       */
#define I2S_STATUS1_CH1ZCIF_Msk          (0x1ul << I2S_STATUS1_CH1ZCIF_Pos)                /*!< I2S_T::STATUS1: CH1ZCIF Mask           */

#define I2S_STATUS1_TXCNT_Pos            (8)                                               /*!< I2S_T::STATUS1: TXCNT Position         */
#define I2S_STATUS1_TXCNT_Msk            (0x1ful << I2S_STATUS1_TXCNT_Pos)                 /*!< I2S_T::STATUS1: TXCNT Mask             */

#define I2S_STATUS1_RXCNT_Pos            (16)                                              /*!< I2S_T::STATUS1: RXCNT Position         */
#define I2S_STATUS1_RXCNT_Msk            (0x1ful << I2S_STATUS1_RXCNT_Pos)                 /*!< I2S_T::STATUS1: RXCNT Mask             */

/**@}*/ /* I2S_CONST */
/**@}*/ /* end of I2S register group */


/*---------------------- Interrupt Source Register -------------------------*/
/**
    @addtogroup INT Interrupt Source Register(INT)
    Memory Mapped Structure for INT Controller
@{ */
 
typedef struct
{


/**
 * @var INT_T::IRQ0_SRC
 * Offset: 0x00  IRQ0 (WDT) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: WDT_INT
 * @var INT_T::IRQ1_SRC
 * Offset: 0x04  IRQ1 (DAC) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: DAC_INT
 * @var INT_T::IRQ2_SRC
 * Offset: 0x08  IRQ2 (SARADC) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: SARADC_INT
 * @var INT_T::IRQ3_SRC
 * Offset: 0x0C  IRQ3 (SDADC) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: SDADC_INT
 * @var INT_T::IRQ4_SRC
 * Offset: 0x10  IRQ4 (I2S) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: I2S_INT
 * @var INT_T::IRQ5_SRC
 * Offset: 0x14  IRQ5 (Timer0) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: Timer0_INT
 * @var INT_T::IRQ6_SRC
 * Offset: 0x18  IRQ6 (Timer1) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: Timer1_INT
 * @var INT_T::IRQ7_SRC
 * Offset: 0x1C  IRQ7 (Timer2) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: Timer2_INT
 * @var INT_T::IRQ8_SRC
 * Offset: 0x20  IRQ8 (GPA) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: GPA_INT
 * @var INT_T::IRQ9_SRC
 * Offset: 0x24  IRQ9 (GPB) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: GPB_INT
 * @var INT_T::IRQ10_SRC
 * Offset: 0x28  IRQ10 (GPC) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: GPC_INT
 * @var INT_T::IRQ11_SRC
 * Offset: 0x2C  IRQ11 (GPD) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: GPD_INT
 * @var INT_T::IRQ12_SRC
 * Offset: 0x30  IRQ12 (SPI0) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: SPI0_INT
 * @var INT_T::IRQ13_SRC
 * Offset: 0x34  IRQ13 (PWM0) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: PWM0_INT
 * @var INT_T::IRQ14_SRC
 * Offset: 0x38  IRQ14 (PWM1) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: PWM1_INT
 * @var INT_T::IRQ15_SRC
 * Offset: 0x3C  IRQ15 (PDMA) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: PDMA_INT
 * @var INT_T::IRQ16_SRC
 * Offset: 0x40  IRQ16 (I2C0) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: I2C0_INT
 * @var INT_T::IRQ17_SRC
 * Offset: 0x44  IRQ17 (I2C1) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: I2C1_INT
 * @var INT_T::IRQ18_SRC
 * Offset: 0x48  IRQ18 (BOD) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: BOD_INT
 * @var INT_T::IRQ19_SRC
 * Offset: 0x4C  Reserved IRQ19 (MAC) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: MAC_INT
 * @var INT_T::IRQ20_SRC
 * Offset: 0x50  IRQ20 (UART0) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: UART0_INT
 * @var INT_T::IRQ21_SRC
 * Offset: 0x54  IRQ21 (UART1) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: UART1_INT
 * @var INT_T::IRQ22_SRC
 * Offset: 0x58  IRQ22 (IRCTRIM) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: IRCTRIM_INT
 * @var INT_T::IRQ23_SRC
 * Offset: 0x5C  IRQ23 (USB) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: USB_INT
 * @var INT_T::IRQ24_SRC
 * Offset: 0x60  IRQ24 (CPD) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: CPD_INT
 * @var INT_T::IRQ25_SRC
 * Offset: 0x64  IRQ25 (XCLKF) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: XCLKF_INT
 * @var INT_T::IRQ26_SRC
 * Offset: 0x68  IRQ26 (SPI1) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: SPI1_INT
 * @var INT_T::NMI_SEL
 * Offset: 0x80  NMI Source Interrupt Select Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[4:0]   |NMI_SEL   |NMI Source Interrupt Select
 * |        |          |The NMI interrupt to Cortex-M0 can be selected from one of the interrupt [0:25].
 * |        |          |The NMI_SEL bit is used to select the NMI interrupt source.
 * |        |          |Note: IRQ19 are reserved in ISD91500.
 */
    __I  uint32_t IRQ0_SRC;              /*!< [0x0000] IRQ0 (WDT) Interrupt Source Identity Register                    */
    __I  uint32_t IRQ1_SRC;              /*!< [0x0004] IRQ1 (DAC) Interrupt Source Identity Register                    */
    __I  uint32_t IRQ2_SRC;              /*!< [0x0008] IRQ2 (SARADC) Interrupt Source Identity Register                 */
    __I  uint32_t IRQ3_SRC;              /*!< [0x000c] IRQ3 (SDADC) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ4_SRC;              /*!< [0x0010] IRQ4 (I2S) Interrupt Source Identity Register                    */
    __I  uint32_t IRQ5_SRC;              /*!< [0x0014] IRQ5 (Timer0) Interrupt Source Identity Register                 */
    __I  uint32_t IRQ6_SRC;              /*!< [0x0018] IRQ6 (Timer1) Interrupt Source Identity Register                 */
    __I  uint32_t IRQ7_SRC;              /*!< [0x001c] IRQ7 (Timer2) Interrupt Source Identity Register                 */
    __I  uint32_t IRQ8_SRC;              /*!< [0x0020] IRQ8 (GPA) Interrupt Source Identity Register                    */
    __I  uint32_t IRQ9_SRC;              /*!< [0x0024] IRQ9 (GPB) Interrupt Source Identity Register                    */
    __I  uint32_t IRQ10_SRC;             /*!< [0x0028] IRQ10 (GPC) Interrupt Source Identity Register                   */
    __I  uint32_t IRQ11_SRC;             /*!< [0x002c] IRQ11 (GPD) Interrupt Source Identity Register                   */
    __I  uint32_t IRQ12_SRC;             /*!< [0x0030] IRQ12 (SPI0) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ13_SRC;             /*!< [0x0034] IRQ13 (PWM0) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ14_SRC;             /*!< [0x0038] IRQ14 (PWM1) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ15_SRC;             /*!< [0x003c] IRQ15 (PDMA) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ16_SRC;             /*!< [0x0040] IRQ16 (I2C0) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ17_SRC;             /*!< [0x0044] IRQ17 (I2C1) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ18_SRC;             /*!< [0x0048] IRQ18 (BOD) Interrupt Source Identity Register                   */
    __I  uint32_t IRQ19_SRC;             /*!< [0x004c] Reserved IRQ19 (MAC) Interrupt Source Identity Register          */
    __I  uint32_t IRQ20_SRC;             /*!< [0x0050] IRQ20 (UART0) Interrupt Source Identity Register                 */
    __I  uint32_t IRQ21_SRC;             /*!< [0x0054] IRQ21 (UART1) Interrupt Source Identity Register                 */
    __I  uint32_t IRQ22_SRC;             /*!< [0x0058] IRQ22 (IRCTRIM) Interrupt Source Identity Register               */
    __I  uint32_t IRQ23_SRC;             /*!< [0x005c] IRQ23 (USB) Interrupt Source Identity Register                   */
    __I  uint32_t IRQ24_SRC;             /*!< [0x0060] IRQ24 (CPD) Interrupt Source Identity Register                   */
    __I  uint32_t IRQ25_SRC;             /*!< [0x0064] IRQ25 (XCLKF) Interrupt Source Identity Register                 */
    __I  uint32_t IRQ26_SRC;             /*!< [0x0068] IRQ26 (SPI1) Interrupt Source Identity Register                  */
    __I  uint32_t RESERVE0[5];
    __IO uint32_t NMI_SEL;               /*!< [0x0080] NMI Source Interrupt Select Control Register                     */

} INT_T;

/**
    @addtogroup INT_CONST INT Bit Field Definition
    Constant Definitions for INT Controller
@{ */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC: INT_SRC Position           */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC: INT_SRC Mask               */

#define INT_SEL_NMI_SEL_Pos              (0)                                               /*!< INT_T::SEL: NMI_SEL Position           */
#define INT_SEL_NMI_SEL_Msk              (0x1ful << INT_SEL_NMI_SEL_Pos)                   /*!< INT_T::SEL: NMI_SEL Mask               */

/**@}*/ /* INT_CONST */
/**@}*/ /* end of INT register group */


/*---------------------- Peripheral Direct Memory Access Controller -------------------------*/
/**
    @addtogroup PDMA Peripheral Direct Memory Access Controller(PDMA)
    Memory Mapped Structure for PDMA Controller
@{ */
 
typedef struct
{


/**
 * @var PDMA_T::CSR
 * Offset: 0x00  PDMA Channel x Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDMACEN   |PDMA Channel Enable
 * |        |          |Setting this bit to 1 enables PDMAu2019s operation
 * |        |          |If this bit is cleared, PDMA will ignore all PDMA request and force Bus Master into IDLE state.
 * |        |          |Note: SWRST will clear this bit.
 * |[1]     |SWRST     |Software Engine Reset
 * |        |          |0 = Writing 0 to this bit has no effect.
 * |        |          |1 = Writing 1 to this bit will reset the internal state machine and pointers
 * |        |          |The contents of the control register will not be cleared
 * |        |          |This bit will auto clear after a few clock cycles.Software Engine Reset
 * |        |          |0 = Writing 0 to this bit has no effect.
 * |        |          |1 = Writing 1 to this bit will reset the internal state machine and pointers
 * |        |          |The contents of the control register will not be cleared
 * |        |          |This bit will auto clear after a few clock cycles.
 * |[3:2]   |MODESEL   |PDMA Mode Select
 * |        |          |This parameter selects to transfer direction of the PDMA channel. Possible values are:
 * |        |          |00 = Memory to Memory mode (SRAM-to-SRAM).
 * |        |          |01 = IP to Memory mode (APB-to-SRAM).
 * |        |          |10 = Memory to IP mode (SRAM-to-APB).
 * |[5:4]   |SASEL     |Source Address Select
 * |        |          |This parameter determines the behavior of the current source address register with each PDMA transfer
 * |        |          |It can either be fixed, incremented or wrapped.
 * |        |          |00 = Transfer Source address is incremented.
 * |        |          |01 = Reserved.
 * |        |          |10 = Transfer Source address is fixed
 * |        |          |11 = Transfer Source address is wrapped
 * |        |          |When CBCR (Current Byte Count) equals zero, the CSAR (Current Source Address) and CBCR registers will be reloaded from the SAR (Source Address) and BCR (Byte Count) registers automatically and PDMA will start another transfer
 * |        |          |Cycle continues until software sets PDMACEN=0
 * |        |          |When PDMACEN is disabled, the PDMA will complete the active transfer but the remaining data in the SBUF will not be transferred to the destination address.
 * |[7:6]   |DASEL     |Destination Address Select
 * |        |          |This parameter determines the behavior of the current destination address register with each PDMA transfer
 * |        |          |It can either be fixed, incremented or wrapped.
 * |        |          |00 = Transfer Destination Address is incremented.
 * |        |          |01 = Reserved.
 * |        |          |10 = Transfer Destination Address is fixed (Used when data transferred from multiple addresses to a single destination such as peripheral FIFO input).
 * |        |          |11 = Transfer Destination Address is wrapped
 * |        |          |When CBCR (Current Byte Count) equals zero, the CDAR (Current Destination Address) and CBCR registers will be reloaded from the DAR (Destination Address) and BCR (Byte Count) registers automatically and PDMA will start another transfer
 * |        |          |Cycle continues until software sets PDMA_EN=0
 * |        |          |When PDMA_EN is disabled, the PDMA will complete the active transfer but the remaining data in the SBUF will not be transferred to the destination address.
 * |[15:12] |WAINTSEL  |Wrap Interrupt Select
 * |        |          |x1xx: If this bit is set, and wraparound mode is in operation a Wrap Interrupt can be generated when half each PDMA transfer is complete
 * |        |          |For example if BCR=32 then an interrupt could be generated when 16 bytes were sent.
 * |        |          |xxx1: If this bit is set, and wraparound mode is in operation a Wrap Interrupt can be generated when each PDMA transfer is wrapped
 * |        |          |For example if BCR=32 then an interrupt could be generated when 32 bytes were sent and PDMA wraps around.
 * |        |          |x1x1: Both half and w interrupts generated.
 * |[20:19] |APBTWS    |Peripheral Transfer Width Select.
 * |        |          |This parameter determines the data width to be transferred each PDMA transfer operation.
 * |        |          |00 = One word (32 bits) is transferred for every PDMA operation.
 * |        |          |01 = One byte (8 bits) is transferred for every PDMA operation.
 * |        |          |10 = One half-word (16 bits) is transferred for every PDMA operation.
 * |        |          |11 = Reserved.
 * |        |          |Note: This field is meaningful only when MODESEL is IP to Memory mode (APB-to-Memory) or Memory to IP mode (Memory-to-APB).
 * |[23]    |TRGEN     |Trigger Enable u2013 Start a PDMA operation.
 * |        |          |0 = Write: no effect. Read: Idle/Finished.
 * |        |          |1 = Enable PDMA data read or write transfer.
 * |        |          |Note: When PDMA transfer completed, this bit will be cleared automatically.
 * |        |          |If a bus error occurs, all PDMA transfer will be stopped
 * |        |          |Software must reset PDMA channel, and then trigger again.
 * @var PDMA_T::SAR
 * Offset: 0x04  PDMA Channel x Source Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |SAR       |PDMA Transfer Source Address Register
 * |        |          |This register holds the initial Source Address of PDMA transfer.
 * |        |          |Note: The source address must be word aligned.
 * @var PDMA_T::DAR
 * Offset: 0x08  PDMA Channel x Destination Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |DAR       |PDMA Transfer Destination Address Register
 * |        |          |This register holds the initial Destination Address of PDMA transfer.
 * |        |          |Note: The destination address must be word aligned.
 * @var PDMA_T::BCR
 * Offset: 0x0C  PDMA Channel x Transfer Byte Count Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |BCR       |PDMA Transfer Byte Count Register
 * |        |          |This register controls the transfer byte count of PDMA. Maximum value is 0xFFFF.
 * |        |          |Note: When in memory-to-memory (CSR.MODESEL = 00b) mode, tThe transfer byte count must be word aligned, that is multiples of 4bytes.
 * @var PDMA_T::POINT
 * Offset: 0x10  PDMA Channel x Internal Buffer Pointer Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |POINT     |PDMA Internal Buffer Pointer Register (Read Only)
 * |        |          |A PDMA transaction consists of two stages, a read from the source address and a write to the destination address
 * |        |          |Internally this data is buffered in a 32bit register
 * |        |          |If transaction width between the read and write transactions are different, this register tracks which byte/half-word of the internal buffer is being processed by the current transaction.
 * @var PDMA_T::CSAR
 * Offset: 0x14  PDMA Channel x Current Source Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |CSAR      |PDMA Current Source Address Register (Read Only)
 * |        |          |This register returns the source address from which the PDMA transfer is occurring
 * |        |          |This register is loaded from SAR when PDMA is triggered or when a wraparound occurs.
 * @var PDMA_T::CDAR
 * Offset: 0x18  PDMA Channel x Current Destination Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |CDAR      |PDMA Current Destination Address Register (Read Only)
 * |        |          |This register returns the destination address to which the PDMA transfer is occurring
 * |        |          |This register is loaded from DAR when PDMA is triggered or when a wraparound occurs.
 * @var PDMA_T::CBCR
 * Offset: 0x1C  PDMA Channel x Current Transfer Byte Count Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CBCR      |PDMA Current Byte Count Register (Read Only)
 * |        |          |This field indicates the current remaining byte count of PDMA transfer
 * |        |          |This register is initialized with BCR register when PDMA is triggered or when a wraparound occurs
 * @var PDMA_T::IER
 * Offset: 0x20  PDMA Channel x Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ABTIEN    |PDMA Read/Write Target Abort Interrupt Enable
 * |        |          |If enabled, the PDMA controller will generate and interrupt to the CPU whenever a PDMA transaction is aborted due to an error
 * |        |          |If a transfer is aborted, PDMA channel must be reset to resume DMA operation.
 * |        |          |0 = Disable PDMA transfer target abort interrupt generation.
 * |        |          |1 = Enable PDMA transfer target abort interrupt generation.
 * |[1]     |TXIEN     |PDMA Transfer Done Interrupt Enable
 * |        |          |If enabled, the PDMA controller will generate and interrupt to the CPU when the requested PDMA transfer is complete.
 * |        |          |0 = Disable PDMA transfer done interrupt generation.
 * |        |          |1 = Enable PDMA transfer done interrupt generation.
 * |[2]     |WRAPIEN   |Wraparound Interrupt Enable
 * |        |          |If enabled, and channel source or destination address is in wraparound mode, the PDMA controller will generate a WRAP interrupt to the CPU according to the setting of CSR.WAINTSEL
 * |        |          |This can be interrupts when the transaction has finished and has wrapped around and/or when the transaction is half way in progress
 * |        |          |This allows the efficient implementation of circular buffers for DMA.
 * |        |          |0 = Disable Wraparound PDMA interrupt generation.
 * |        |          |1 = Enable Wraparound interrupt generation.
 * @var PDMA_T::ISR
 * Offset: 0x24  PDMA Channel x Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ABTIF     |PDMA Read/Write Target Abort Interrupt Flag
 * |        |          |This flag indicates a Target Abort interrupt condition has occurred
 * |        |          |This condition can happen if attempt is made to read/write from invalid or non-existent memory space
 * |        |          |It occurs when PDMA controller receives a bus error from AHB master
 * |        |          |Upon occurrence PDMA will stop transfer and go to idle state
 * |        |          |To resume, software must reset PDMA channel and initiate transfer again.
 * |        |          |0 = No bus ERROR response received.
 * |        |          |1 = Bus ERROR response received.
 * |        |          |Note: This bit is cleared by writing 1 to itself.
 * |[1]     |TXIF      |Block Transfer Done Interrupt Flag
 * |        |          |This bit indicates that PDMA block transfer complete interrupt has been generated
 * |        |          |It is cleared by writing 1 to the bit.
 * |        |          |0 = Transfer ongoing or Idle.
 * |        |          |1 = Transfer Complete.
 * |[11:8]  |WRAPIF    |Wrap around transfer byte count interrupt flag.
 * |        |          |These flags are set whenever the conditions for a wraparound interrupt (complete or half complete) are met
 * |        |          |They are cleared by writing one to the bits.
 * |        |          |0001 = Current transfer finished flag (CBCR==0).
 * |        |          |0100 = Current transfer half complete flag (CBCR==BCR/2).
 * |[31]    |INTR      |Interrupt Pin Status (Read Only)
 * |        |          |This bit is the Interrupt pin status of PDMA channel.
 */
    __IO uint32_t CSR;                   /*!< [0x0000] PDMA Channel x Control Register                                  */
    __IO uint32_t SAR;                   /*!< [0x0004] PDMA Channel x Source Address Register                           */
    __IO uint32_t DAR;                   /*!< [0x0008] PDMA Channel x Destination Address Register                      */
    __IO uint32_t BCR;                   /*!< [0x000c] PDMA Channel x Transfer Byte Count Register                      */
    __I  uint32_t POINT;                 /*!< [0x0010] PDMA Channel x Internal Buffer Pointer Register                  */
    __I  uint32_t CSAR;                  /*!< [0x0014] PDMA Channel x Current Source Address Register                   */
    __I  uint32_t CDAR;                  /*!< [0x0018] PDMA Channel x Current Destination Address Register              */
    __I  uint32_t CBCR;                  /*!< [0x001c] PDMA Channel x Current Transfer Byte Count Register              */
    __IO uint32_t IER;                   /*!< [0x0020] PDMA Channel x Interrupt Enable Register                         */
    __IO uint32_t ISR;                   /*!< [0x0024] PDMA Channel x Interrupt Status Register                         */

} PDMA_T;

/**
    @addtogroup PDMA_CONST PDMA Bit Field Definition
    Constant Definitions for PDMA0 Controller
@{ */

#define PDMA_CSR_PDMACEN_Pos            (0)                                               /*!< PDMA_T::CSR: PDMACEN Position         */
#define PDMA_CSR_PDMACEN_Msk            (0x1ul << PDMA_CSR_PDMACEN_Pos)                   /*!< PDMA_T::CSR: PDMACEN Mask             */

#define PDMA_CSR_SW_RST_Pos             (1)                                               /*!< PDMA_T::CSR: SW_RST Position           */
#define PDMA_CSR_SW_RST_Msk             (0x1ul << PDMA_CSR_SW_RST_Pos)                    /*!< PDMA_T::CSR: SW_RST Mask               */
  
#define PDMA_CSR_MODE_SEL_Pos           (2)                                               /*!< PDMA_T::CSR: MODE_SEL Position         */
#define PDMA_CSR_MODE_SEL_Msk           (0x3ul << PDMA_CSR_MODE_SEL_Pos)                  /*!< PDMA_T::CSR: MODE_SEL Mask             */
                                                                                                                                       
#define PDMA_CSR_SAD_SEL_Pos            (4)                                               /*!< PDMA_T::CSR: SAD_SEL Position          */
#define PDMA_CSR_SAD_SEL_Msk            (0x3ul << PDMA_CSR_SAD_SEL_Pos)                   /*!< PDMA_T::CSR: SAD_SEL Mask              */
                                                                                                                                       
#define PDMA_CSR_DAD_SEL_Pos            (6)                                               /*!< PDMA_T::CSR: DAD_SEL Position          */
#define PDMA_CSR_DAD_SEL_Msk            (0x3ul << PDMA_CSR_DAD_SEL_Pos)                   /*!< PDMA_T::CSR: DAD_SEL Mask              */
                                                                                                                                       
#define PDMA_CSR_WRA_INT_SEL_Pos        (12)                                              /*!< PDMA_T::CSR: WRA_INT_SEL Position      */
#define PDMA_CSR_WRA_INT_SEL_Msk        (0xful << PDMA_CSR_WRA_INT_SEL_Pos)               /*!< PDMA_T::CSR: WRA_INT_SEL Mask          */
     
#define PDMA_CSR_APB_TWS_Pos            (19)                                              /*!< PDMA_T::CSR: APB_TWS Position          */
#define PDMA_CSR_APB_TWS_Msk            (0x3ul << PDMA_CSR_APB_TWS_Pos)                   /*!< PDMA_T::CSR: APB_TWS Mask              */
    
#define PDMA_CSR_TRIG_EN_Pos            (23)                                              /*!< PDMA_T::CSR: TRIG_EN Position          */
#define PDMA_CSR_TRIG_EN_Msk            (0x1ul << PDMA_CSR_TRIG_EN_Pos)                   /*!< PDMA_T::CSR: TRIG_EN Mask              */
 
#define PDMA_SAR_SAR_Pos                (0)                                               /*!< PDMA_T::SAR: SAR Position             */
#define PDMA_SAR_SAR_Msk                (0xfffffffful << PDMA_SAR_SAR_Pos)                /*!< PDMA_T::SAR: SAR Mask                 */

#define PDMA_DAR_DAR_Pos                (0)                                               /*!< PDMA_T::DAR: DAR Position             */
#define PDMA_DAR_DAR_Msk                (0xfffffffful << PDMA_DAR_DAR_Pos)                /*!< PDMA_T::DAR: DAR Mask                 */

#define PDMA_BCR_BCR_Pos                (0)                                               /*!< PDMA_T::BCR: BCR Position             */
#define PDMA_BCR_BCR_Msk                (0xfffful << PDMA_BCR_BCR_Pos)                    /*!< PDMA_T::BCR: BCR Mask                 */

#define PDMA_POINT_POINT_Pos            (0)                                               /*!< PDMA_T::POINT: POINT Position         */
#define PDMA_POINT_POINT_Msk            (0xful << PDMA_POINT_POINT_Pos)                   /*!< PDMA_T::POINT: POINT Mask             */

#define PDMA_CSAR_CSAR_Pos              (0)                                               /*!< PDMA_T::CSAR: CSAR Position           */
#define PDMA_CSAR_CSAR_Msk              (0xfffffffful << PDMA_CSAR_CSAR_Pos)              /*!< PDMA_T::CSAR: CSAR Mask               */

#define PDMA_CDAR_CDAR_Pos              (0)                                               /*!< PDMA_T::CDAR: CDAR Position           */
#define PDMA_CDAR_CDAR_Msk              (0xfffffffful << PDMA_CDAR_CDAR_Pos)              /*!< PDMA_T::CDAR: CDAR Mask               */

#define PDMA_CBCR_CBCR_Pos              (0)                                               /*!< PDMA_T::CBCR: CBCR Position           */
#define PDMA_CBCR_CBCR_Msk              (0xfffful << PDMA_CBCR_CBCR_Pos)                  /*!< PDMA_T::CBCR: CBCR Mask               */

#define PDMA_IER_TABORT_IE_Pos          (0)                                               /*!< PDMA_T::IER: TABORT_IE Position        */
#define PDMA_IER_TABORT_IE_Msk          (0x1ul << PDMA_IER_TABORT_IE_Pos)                 /*!< PDMA_T::IER: TABORT_IE Mask            */
                                                                                                                                        
#define PDMA_IER_BLKD_IE_Pos            (1)                                               /*!< PDMA_T::IER: BLKD_IE Position          */
#define PDMA_IER_BLKD_IE_Msk            (0x1ul << PDMA_IER_BLKD_IE_Pos)                   /*!< PDMA_T::IER: BLKD_IE Mask              */
                                                                                                                                        
#define PDMA_IER_WAR_IE_Pos             (2)                                               /*!< PDMA_T::IER: WAR_IE Position           */
#define PDMA_IER_WAR_IE_Msk             (0x1ul << PDMA_IER_WAR_IE_Pos)                    /*!< PDMA_T::IER: WAR_IE Mask               */
                                                                                                                                        
#define PDMA_ISR_TABORT_IF_Pos          (0)                                               /*!< PDMA_T::ISR: TABORT_IF Position        */
#define PDMA_ISR_TABORT_IF_Msk          (0x1ul << PDMA_ISR_TABORT_IF_Pos)                 /*!< PDMA_T::ISR: TABORT_IF Mask            */
                                                                                                                                        
#define PDMA_ISR_BLKD_IF_Pos            (1)                                               /*!< PDMA_T::ISR: BLKD_IF Position          */
#define PDMA_ISR_BLKD_IF_Msk            (0x1ul << PDMA_ISR_BLKD_IF_Pos)                   /*!< PDMA_T::ISR: BLKD_IF Mask              */
                                                                                                                                       
#define PDMA_ISR_WAR_IF_Pos             (8)                                               /*!< PDMA_T::ISR: WAR_IF Position           */
#define PDMA_ISR_WAR_IF_Msk             (0xful << PDMA_ISR_WAR_IF_Pos)                    /*!< PDMA_T::ISR: WAR_IF Mask               */
                                                                                                                                       
#define PDMA_ISR_INTR_Pos               (31)                                              /*!< PDMA_T::ISR: INTR Position             */
#define PDMA_ISR_INTR_Msk               (0x1ul << PDMA_ISR_INTR_Pos)                      /*!< PDMA_T::ISR: INTR Mask                 */

/**@}*/ /* PDMA_CONST */
/**@}*/ /* end of PDMA register group */


/*---------------------- PDMA Global Control -------------------------*/
/**
    @addtogroup PDMA_GCR PDMA Global Control(PDMA_GCR)
    Memory Mapped Structure for PDMA_GCR Controller
@{ */
 
typedef struct
{


/**
 * @var PDMA_GCR_T::GCRCSR
 * Offset: 0x00  PDMA Global Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RST       |PDMA Software Reset
 * |        |          |0 = Writing 0 to this bit has no effect.
 * |        |          |1 = Writing 1 to this bit will reset the internal state machine and pointers
 * |        |          |The contents of control register will not be cleared
 * |        |          |This bit will auto clear after several clock cycles.
 * |        |          |Note: This bit can reset all channels register(global reset) , but not reset each channel internal state machine..
 * |[15:8]  |HCLKEN    |PDMA Controller Channel Clock Enable Control
 * |        |          |To enable clock for channel n HCLKEN[n] must be set.
 * |        |          |HCLKEN[n]=1: Enable Channel n clock
 * |        |          |HCLKEN[n]=0: Disable Channel n clock
 * @var PDMA_GCR_T::PDSSR0
 * Offset: 0x04  PDMA Service Selection Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |SPI0RXSEL |PDMA SPI0 Receive Selection
 * |        |          |This field defines which PDMA channel is connected to SPI0 peripheral receive (PDMA source) request.
 * |        |          |n = Select channel n.(n= 0~7)
 * |        |          |Others = Reserved.n = Select channel n-1.(n= 1~8)
 * |        |          |0 = No channel select
 * |        |          |Others = Reserved.
 * |[7:4]   |SPI0TXSEL |PDMA SPI0 Transmit Selection
 * |        |          |This field defines which PDMA channel is connected to SPI0 peripheral transmit (PDMA destination) request.
 * |        |          |n = Select channel n.(n= 0~7)
 * |        |          |Others = Reserved.n = Select channel n-1.(n= 1~8)
 * |        |          |0 = No channel select
 * |        |          |Others = Reserved.
 * |[11:8]  |I2SRXSEL  |PDMA I2S Receive Selection
 * |        |          |This field defines which PDMA channel is connected to I2S peripheral receive (PDMA source) request.
 * |        |          |n = Select channel n.(n= 0~7)
 * |        |          |Others = Reserved.n = Select channel n-1.(n= 1~8)
 * |        |          |0 = No channel select
 * |        |          |Others = Reserved.
 * |[15:12] |I2STXSEL  |PDMA I2S Transmit Selection
 * |        |          |This field defines which PDMA channel is connected to I2S peripheral transmit (PDMA destination) request.
 * |        |          |n = Select channel n.(n= 0~7)
 * |        |          |Others = Reserved.n = Select channel n-1.(n= 1~8)
 * |        |          |0 = No channel select
 * |        |          |Others = Reserved.
 * |[19:16] |UART1RXSEL|PDMA UART1 Receive Selection
 * |        |          |This field defines which PDMA channel is connected to UART1 peripheral receive (PDMA source) request.
 * |        |          |n = Select channel n.(n= 0~7)
 * |        |          |Others = Reserved.n = Select channel n-1.(n= 1~8)
 * |        |          |0 = No channel select
 * |        |          |Others = Reserved.
 * |[23:20] |UART1TXSEL|PDMA UART1 Transmit Selection
 * |        |          |This field defines which PDMA channel is connected to UART1 peripheral transmit (PDMA destination) request.
 * |        |          |n = Select channel n.(n= 0~7)
 * |        |          |Others = Reserved.n = Select channel n-1.(n= 1~8)
 * |        |          |0 = No channel select
 * |        |          |Others = Reserved.
 * |[27:24] |UART0RXSEL|PDMA UART0 Receive Selection
 * |        |          |This field defines which PDMA channel is connected to UART0 peripheral receive (PDMA source) request.
 * |        |          |n = Select channel n.(n= 0~7)
 * |        |          |Others = Reserved.n = Select channel n-1.(n= 1~8)
 * |        |          |0 = No channel select
 * |        |          |Others = Reserved.
 * |[31:28] |UART0TXSEL|PDMA UART0 Transmit Selection
 * |        |          |This field defines which PDMA channel is connected to UART0 peripheral transmit (PDMA destination) request.
 * |        |          |n = Select channel n-1.(n= 10~87)
 * |        |          |0 = No channel select
 * |        |          |Others = Reserved.
 * @var PDMA_GCR_T::PDSSR1
 * Offset: 0x08  PDMA Service Selection Control Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |SDADCSEL  |PDMA SDADC Receive Selection
 * |        |          |This field defines which PDMA channel is connected to SDADC peripheral receive (PDMA source) request.
 * |        |          |n = Select channel n.(n= 0~7)
 * |        |          |Others = Reserved.n = Select channel n-1.(n= 1~8)
 * |        |          |0 = No channel select
 * |        |          |Others = Reserved.
 * |[7:4]   |DACTXSEL  |PDMA DAC Transmit Selection
 * |        |          |This field defines which PDMA channel is connected to DAC peripheral transmit (PDMA destination) request.
 * |        |          |n = Select channel n.(n= 0~7)
 * |        |          |Others = Reserved.n = Select channel n-1.(n= 1~8)
 * |        |          |0 = No channel select
 * |        |          |Others = Reserved.
 * |[11:8]  |SARADCSEL |PDMA SARADC Receive Selection
 * |        |          |This field defines which PDMA channel is connected to SARADC peripheral receive (PDMA source) request.
 * |        |          |n = Select channel n.(n= 0~7)
 * |        |          |Others = Reserved.n = Select channel n-1.(n= 1~8)
 * |        |          |0 = No channel select
 * |        |          |Others = Reserved.
 * |[19:16] |SPI1RXSEL |PDMA SPI1 Receive Selection
 * |        |          |This field defines which PDMA channel is connected to SPI1 peripheral receive (PDMA source) request.
 * |        |          |n = Select channel n.(n= 0~7)
 * |        |          |Others = Reserved.
 * |[23:20] |SPI1TXSEL |PDMA SPI1 Transmit Selection
 * |        |          |This field defines which PDMA channel is connected to SPI1 peripheral transmit (PDMA destination) request.
 * |        |          |n = Select channel n.(n= 0~7)
 * |        |          |Others = Reserved.
 * @var PDMA_GCR_T::GCRISR
 * Offset: 0x0C  PDMA Global Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |GCRISR    |Interrupt Pin Status (Read Only)
 * |        |          |GCRISR[n] is the interrupt status of PDMA channel n.
 */
    __IO uint32_t GCRCSR;                /*!< [0x0000] PDMA Global Control Register                                     */
    __IO uint32_t PDSSR0;                /*!< [0x0004] PDMA Service Selection Control Register 0                        */
    __IO uint32_t PDSSR1;                /*!< [0x0008] PDMA Service Selection Control Register 1                        */
    __I  uint32_t GCRISR;                /*!< [0x000c] PDMA Global Interrupt Status Register                            */

} PDMA_GCR_T;

/**
    @addtogroup PDMA_GCR_CONST PDMA_GCR Bit Field Definition
    Constant Definitions for PDMA_GCR Controller
@{ */

#define PDMA_GCRCSR_PDMA_RST_Pos         (0)                                               /*!< PDMA_GCR_T::GCRCSR: PDMA_RST Position  */
#define PDMA_GCRCSR_PDMA_RST_Msk         (0x1ul << PDMA_GCRCSR_PDMA_RST_Pos)               /*!< PDMA_GCR_T::GCRCSR: PDMA_RST Mask      */
                                                                                           
#define PDMA_GCRCSR_HCLK_EN_Pos          (8)                                               /*!< PDMA_GCR_T::GCRCSR: HCLK_EN Position   */
#define PDMA_GCRCSR_HCLK_EN_Msk          (0x3ul << PDMA_GCRCSR_HCLK_EN_Pos)                /*!< PDMA_GCR_T::GCRCSR: HCLK_EN Mask       */
    
#define PDMA_PDSSR0_SPI0RXSEL_Pos        (0)                                               /*!< PDMA_GCR_T::PDSSR0: SPI0RXSEL Position */
#define PDMA_PDSSR0_SPI0RXSEL_Msk        (0xful << PDMA_PDSSR0_SPI0RXSEL_Pos)              /*!< PDMA_GCR_T::PDSSR0: SPI0RXSEL Mask     */

#define PDMA_PDSSR0_SPI0TXSEL_Pos        (4)                                               /*!< PDMA_GCR_T::PDSSR0: SPI0TXSEL Position */
#define PDMA_PDSSR0_SPI0TXSEL_Msk        (0xful << PDMA_PDSSR0_SPI0TXSEL_Pos)              /*!< PDMA_GCR_T::PDSSR0: SPI0TXSEL Mask     */

#define PDMA_PDSSR0_I2SRXSEL_Pos         (8)                                               /*!< PDMA_GCR_T::PDSSR0: I2SRXSEL Position  */
#define PDMA_PDSSR0_I2SRXSEL_Msk         (0xful << PDMA_PDSSR0_I2SRXSEL_Pos)               /*!< PDMA_GCR_T::PDSSR0: I2SRXSEL Mask      */

#define PDMA_PDSSR0_I2STXSEL_Pos         (12)                                              /*!< PDMA_GCR_T::PDSSR0: I2STXSEL Position  */
#define PDMA_PDSSR0_I2STXSEL_Msk         (0xful << PDMA_PDSSR0_I2STXSEL_Pos)               /*!< PDMA_GCR_T::PDSSR0: I2STXSEL Mask      */

#define PDMA_PDSSR0_UART1RXSEL_Pos       (16)                                              /*!< PDMA_GCR_T::PDSSR0: UART1RXSEL Position*/
#define PDMA_PDSSR0_UART1RXSEL_Msk       (0xful << PDMA_PDSSR0_UART1RXSEL_Pos)             /*!< PDMA_GCR_T::PDSSR0: UART1RXSEL Mask    */

#define PDMA_PDSSR0_UART1TXSEL_Pos       (20)                                              /*!< PDMA_GCR_T::PDSSR0: UART1TXSEL Position*/
#define PDMA_PDSSR0_UART1TXSEL_Msk       (0xful << PDMA_PDSSR0_UART1TXSEL_Pos)             /*!< PDMA_GCR_T::PDSSR0: UART1TXSEL Mask    */

#define PDMA_PDSSR0_UART0RXSEL_Pos       (24)                                              /*!< PDMA_GCR_T::PDSSR0: UART0RXSEL Position*/
#define PDMA_PDSSR0_UART0RXSEL_Msk       (0xful << PDMA_PDSSR0_UART0RXSEL_Pos)             /*!< PDMA_GCR_T::PDSSR0: UART0RXSEL Mask    */

#define PDMA_PDSSR0_UART0TXSEL_Pos       (28)                                              /*!< PDMA_GCR_T::PDSSR0: UART0TXSEL Position*/
#define PDMA_PDSSR0_UART0TXSEL_Msk       (0xful << PDMA_PDSSR0_UART0TXSEL_Pos)             /*!< PDMA_GCR_T::PDSSR0: UART0TXSEL Mask    */

#define PDMA_PDSSR1_SDADCSEL_Pos         (0)                                               /*!< PDMA_GCR_T::PDSSR1: SDADCSEL Position  */
#define PDMA_PDSSR1_SDADCSEL_Msk         (0xful << PDMA_PDSSR1_SDADCSEL_Pos)               /*!< PDMA_GCR_T::PDSSR1: SDADCSEL Mask      */

#define PDMA_PDSSR1_DACTXSEL_Pos         (4)                                               /*!< PDMA_GCR_T::PDSSR1: DACTXSEL Position  */
#define PDMA_PDSSR1_DACTXSEL_Msk         (0xful << PDMA_PDSSR1_DACTXSEL_Pos)               /*!< PDMA_GCR_T::PDSSR1: DACTXSEL Mask      */

#define PDMA_PDSSR1_SARADCSEL_Pos        (8)                                               /*!< PDMA_GCR_T::PDSSR1: SARADCSEL Position */
#define PDMA_PDSSR1_SARADCSEL_Msk        (0xful << PDMA_PDSSR1_SARADCSEL_Pos)              /*!< PDMA_GCR_T::PDSSR1: SARADCSEL Mask     */

#define PDMA_PDSSR1_SPI1RXSEL_Pos        (16)                                              /*!< PDMA_GCR_T::PDSSR1: SPI1RXSEL Position */
#define PDMA_PDSSR1_SPI1RXSEL_Msk        (0xful << PDMA_PDSSR1_SPI1RXSEL_Pos)              /*!< PDMA_GCR_T::PDSSR1: SPI1RXSEL Mask     */

#define PDMA_PDSSR1_SPI1TXSEL_Pos        (20)                                              /*!< PDMA_GCR_T::PDSSR1: SPI1TXSEL Position */
#define PDMA_PDSSR1_SPI1TXSEL_Msk        (0xful << PDMA_PDSSR1_SPI1TXSEL_Pos)              /*!< PDMA_GCR_T::PDSSR1: SPI1TXSEL Mask     */

#define PDMA_GCRISR_GCRISR_Pos           (0)                                               /*!< PDMA_GCR_T::GCRISR: GCRISR Position    */
#define PDMA_GCRISR_GCRISR_Msk           (0xfful << PDMA_GCRISR_GCRISR_Pos)                /*!< PDMA_GCR_T::GCRISR: GCRISR Mask        */

/**@}*/ /* PDMA_GCR_CONST */
/**@}*/ /* end of PDMA_GCR register group */


/*---------------------- Pulse Width Modulation Controller -------------------------*/
/**
    @addtogroup PWM Pulse Width Modulation Controller(PWM)
    Memory Mapped Structure for PWM Controller
@{ */
 
typedef struct
{


/**
 * @var PWM_T::CLKPSC
 * Offset: 0x00  PWM Prescaler Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |CLKPSC    |Clock Prescaler For PWM Timer
 * |        |          |Clock input is divided by (CLKPSC + 1)
 * |        |          |If CLKPSC = 0, then the prescaler output clock will be stopped
 * |        |          |This implies PWM counter will also be stopped.
 * |[23:16] |DZI0      |Dead Zone Interval Register 0
 * |        |          |These 8 bits determine dead zone length.
 * |        |          |The unit time of dead zone length is that from clock selector.
 * |[31:24] |DZI1      |Dead Zone Interval Register 1
 * |        |          |These 8 bits determine dead zone length.
 * |        |          |The unit time of dead zone length is that from clock selector.
 * @var PWM_T::CLKDIV
 * Offset: 0x04  PWM Clock Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |CLKDIV    |PWM Timer Clock Source Selection
 * |        |          |Value : Input clock divided by
 * |        |          |000 : 2
 * |        |          |001 : 4
 * |        |          |010 : 8
 * |        |          |011 : 16
 * |        |          |1xx : 1
 * @var PWM_T::CTL
 * Offset: 0x08  PWM Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CNTEN     |PWM-Timer Enable
 * |        |          |0 = Stop PWM-Timer Running.
 * |        |          |1 = Enable PWM-Timer.
 * |[2]     |PINV      |PWM-Timer Output Inverter ON/OFF
 * |        |          |0 = Inverter OFF.
 * |        |          |1 = Inverter ON.
 * |[3]     |CNTMODE   |PWM-Timer Auto-Reload/One-Shot Mode
 * |        |          |0 = One-Shot Mode.
 * |        |          |1 = Auto-reload Mode.
 * |[4]     |DTEN0     |Dead-Zone 0 Generator Enable/Disable
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |[5]     |DTEN1     |Dead-Zone 1 Generator Enable/Disable
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * @var PWM_T::PERIOD
 * Offset: 0x0C  PWM Period Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PERIOD    |PWM Counter/Timer Reload Value
 * |        |          |PERIOD determines the PWM period.
 * |        |          |Note: One PWM cycle width = (PERIOD + 1).
 * @var PWM_T::CMPDAT0
 * Offset: 0x10  PWM Comparator Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMP       |PWM Comparator Register
 * |        |          |CMP determines the PWM duty ratio.
 * |        |          |Assumption: PWM output initial is high
 * |        |          |CMP > = PERIOD: PWM output is always high.
 * |        |          |CMP < PERIOD: PWM low width = (PERIOD - CMP) unit; PWM high width = (CMP+1) unit.
 * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
 * |        |          |Note1: Unit = one PWM clock cycle.
 * |        |          |Note2: Any write to CMP will take effect in next PWM cycle.
 * @var PWM_T::CNT
 * Offset: 0x14  PWM Counter Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNT       |PWM Counter Register
 * |        |          |Reports the current value of the 16-bit down counter.
 * @var PWM_T::CMPDAT1
 * Offset: 0x1C  PWM Comparator Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMP       |PWM Comparator Register
 * |        |          |CMP determines the PWM duty ratio.
 * |        |          |Assumption: PWM output initial is high
 * |        |          |CMP > = PERIOD: PWM output is always high.
 * |        |          |CMP < PERIOD: PWM low width = (PERIOD - CMP) unit; PWM high width = (CMP+1) unit.
 * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
 * |        |          |Note1: Unit = one PWM clock cycle.
 * |        |          |Note2: Any write to CMP will take effect in next PWM cycle.
 * @var PWM_T::CMPDAT2
 * Offset: 0x28  PWM Comparator Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMP       |PWM Comparator Register
 * |        |          |CMP determines the PWM duty ratio.
 * |        |          |Assumption: PWM output initial is high
 * |        |          |CMP > = PERIOD: PWM output is always high.
 * |        |          |CMP < PERIOD: PWM low width = (PERIOD - CMP) unit; PWM high width = (CMP+1) unit.
 * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
 * |        |          |Note1: Unit = one PWM clock cycle.
 * |        |          |Note2: Any write to CMP will take effect in next PWM cycle.
 * @var PWM_T::CMPDAT3
 * Offset: 0x34  PWM Comparator Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMP       |PWM Comparator Register
 * |        |          |CMP determines the PWM duty ratio.
 * |        |          |Assumption: PWM output initial is high
 * |        |          |CMP > = PERIOD: PWM output is always high.
 * |        |          |CMP < PERIOD: PWM low width = (PERIOD - CMP) unit; PWM high width = (CMP+1) unit.
 * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
 * |        |          |Note1: Unit = one PWM clock cycle.
 * |        |          |Note2: Any write to CMP will take effect in next PWM cycle.
 * @var PWM_T::INTEN
 * Offset: 0x40  PWM Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PIEN      |PWM Timer Interrupt Enable
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * @var PWM_T::INTSTS
 * Offset: 0x44  PWM Interrupt Flag Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PIF       |PWM Timer Interrupt Flag
 * |        |          |Flag is set by hardware when PWM down counter reaches zero, software can clear this bit by writing u20181u2019 to it.
 * @var PWM_T::CAPCTL
 * Offset: 0x50  Capture Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CAPINV    |Inverter ON/OFF
 * |        |          |0 = Inverter OFF.
 * |        |          |1 = Inverter ON. Reverse the input signal from GPIO before fed to Capture timer
 * |[1]     |CRLIEN    |Rising Latch Interrupt Enable ON/OFF
 * |        |          |0 = Disable rising latch interrupt.
 * |        |          |1 = Enable rising latch interrupt.
 * |        |          |When enabled, capture block generates an interrupt on rising edge of input.
 * |[2]     |CFLIEN    |Falling Latch Interrupt Enable ON/OFF
 * |        |          |0 = Disable falling latch interrupt.
 * |        |          |1 = Enable falling latch interrupt.
 * |        |          |When enabled, capture block generates an interrupt on falling edge of input.
 * |[3]     |CAPEN     |Capture Channel Input Transition Enable/Disable
 * |        |          |0 = Disable capture function.
 * |        |          |1 = Enable capture function.
 * |        |          |When enabled, Capture function latches the PMW-counter to RCAPDAT (Rising latch) and FCAPDAT (Falling latch) registers on input edge transition.
 * |        |          |When disabled, Capture function is inactive as is interrupt.
 * |[4]     |CAPIF     |Capture Indication Flag
 * |        |          |When capture input has a falling/rising transition with CFLIF = 1 or CRLIF = 1, CAPIF0 is set 1 by hardware
 * |        |          |Software can clear this bit by writing a one to it.
 * |        |          |Note: If this bit is u201C1u201D(not clear by SW), PWM counter will not be reloaded when next capture event occurs.
 * |[6]     |CRLIF     |PWM_RCAPDAT Latched Indicator Bit
 * |        |          |When input channel has a rising transition, PWM_RCAPDAT was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it.
 * |[7]     |CFLIF     |PWM_FCAPDAT Latched Indicator Bit
 * |        |          |When input channel has a falling transition, PWM_FCAPDAT was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it
 * @var PWM_T::RCAPDAT
 * Offset: 0x58  Capture Rising Latch Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |RCAPDAT   |Capture Rising Latch Register
 * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a rising edge of the input signal.
 * @var PWM_T::FCAPDAT
 * Offset: 0x5C  Capture Falling Latch Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |FCAPDAT   |Capture Falling Latch Register
 * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a falling edge of the input signal.
 * @var PWM_T::PCEN
 * Offset: 0x7C  PWM Output and Capture Input Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |POEN0     |PWM Channel0 Output Enable Register
 * |        |          |0 = Disable PWM Channel0 output to pin.
 * |        |          |1 = Enable PWM Channel0 output to pin.
 * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP) 
 * |[1]     |POEN1     |PWM Channel 1 Output Enable Register
 * |        |          |0 = Disable PWM Channel 1 output to pin.
 * |        |          |1 = Enable PWM Channel 1 output to pin.
 * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP) 
 * |[2]     |POEN2     |PWM Channel 2 Output Enable Register
 * |        |          |0 = Disable PWM Channel 2output to pin.
 * |        |          |1 = Enable PWM Channel 2 output to pin.
 * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP) 
 * |[3]     |POEN3     |PWM Channel 3 Output Enable Register
 * |        |          |0 = Disable PWM Channel 3 output to pin.
 * |        |          |1 = Enable PWM Channel 3 output to pin.
 * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP)
 * |[8]     |CAPINEN   |Capture Input Enable Register
 * |        |          |0 = OFF (PA.7/PB.4 pin input disconnected from Capture block).
 * |        |          |1 = ON (PA.7/PB.4 pin, if in PWM alternative function, will be configured as an input and fed to capture function).
 */
    __IO uint32_t CLKPSC;                /*!< [0x0000] PWM Prescaler Register                                           */
    __IO uint32_t CLKDIV;                /*!< [0x0004] PWM Clock Select Register                                        */
    __IO uint32_t CTL;                   /*!< [0x0008] PWM Control Register                                             */
    __IO uint32_t PERIOD;                /*!< [0x000c] PWM Period Register                                              */
    __IO uint32_t CMPDAT0;               /*!< [0x0010] PWM Comparator Register 0                                        */
    __I  uint32_t CNT;                   /*!< [0x0014] PWM Counter Register                                             */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t CMPDAT1;               /*!< [0x001c] PWM Comparator Register 1                                        */
    __I  uint32_t RESERVE1[2];
    __IO uint32_t CMPDAT2;               /*!< [0x0028] PWM Comparator Register 2                                        */
    __I  uint32_t RESERVE2[2];
    __IO uint32_t CMPDAT3;               /*!< [0x0034] PWM Comparator Register 3                                        */
    __I  uint32_t RESERVE3[2];
    __IO uint32_t INTEN;                 /*!< [0x0040] PWM Interrupt Enable Register                                    */
    __IO uint32_t INTSTS;                /*!< [0x0044] PWM Interrupt Flag Register                                      */
    __I  uint32_t RESERVE4[2];
    __IO uint32_t CAPCTL;                /*!< [0x0050] Capture Control Register                                         */
    __I  uint32_t RESERVE5[1];
    __I  uint32_t RCAPDAT;               /*!< [0x0058] Capture Rising Latch Register                                    */
    __I  uint32_t FCAPDAT;               /*!< [0x005c] Capture Falling Latch Register                                   */
    __I  uint32_t RESERVE6[7];
    __IO uint32_t PCEN;                  /*!< [0x007c] PWM Output and Capture Input Enable Register                     */

} PWM_T;

/**
    @addtogroup PWM_CONST PWM Bit Field Definition
    Constant Definitions for PWM Controller
@{ */

#define PWM_CLKPSC_CLKPSC_Pos            (0)                                               /*!< PWM_T::CLKPSC: CLKPSC Position         */
#define PWM_CLKPSC_CLKPSC_Msk            (0xfful << PWM_CLKPSC_CLKPSC_Pos)                 /*!< PWM_T::CLKPSC: CLKPSC Mask             */

#define PWM_CLKPSC_DZI0_Pos              (16)                                              /*!< PWM_T::CLKPSC: DZI0 Position           */
#define PWM_CLKPSC_DZI0_Msk              (0xfful << PWM_CLKPSC_DZI0_Pos)                   /*!< PWM_T::CLKPSC: DZI0 Mask               */

#define PWM_CLKPSC_DZI1_Pos              (24)                                              /*!< PWM_T::CLKPSC: DZI1 Position           */
#define PWM_CLKPSC_DZI1_Msk              (0xfful << PWM_CLKPSC_DZI1_Pos)                   /*!< PWM_T::CLKPSC: DZI1 Mask               */

#define PWM_CLKDIV_CLKDIV_Pos            (0)                                               /*!< PWM_T::CLKDIV: CLKDIV Position         */
#define PWM_CLKDIV_CLKDIV_Msk            (0x7ul << PWM_CLKDIV_CLKDIV_Pos)                  /*!< PWM_T::CLKDIV: CLKDIV Mask             */

#define PWM_CTL_CNTEN_Pos                (0)                                               /*!< PWM_T::CTL: CNTEN Position             */
#define PWM_CTL_CNTEN_Msk                (0x1ul << PWM_CTL_CNTEN_Pos)                      /*!< PWM_T::CTL: CNTEN Mask                 */

#define PWM_CTL_PINV_Pos                 (2)                                               /*!< PWM_T::CTL: PINV Position              */
#define PWM_CTL_PINV_Msk                 (0x1ul << PWM_CTL_PINV_Pos)                       /*!< PWM_T::CTL: PINV Mask                  */

#define PWM_CTL_CNTMODE_Pos              (3)                                               /*!< PWM_T::CTL: CNTMODE Position           */
#define PWM_CTL_CNTMODE_Msk              (0x1ul << PWM_CTL_CNTMODE_Pos)                    /*!< PWM_T::CTL: CNTMODE Mask               */

#define PWM_CTL_DTEN0_Pos                (4)                                               /*!< PWM_T::CTL: DTEN0 Position             */
#define PWM_CTL_DTEN0_Msk                (0x1ul << PWM_CTL_DTEN0_Pos)                      /*!< PWM_T::CTL: DTEN0 Mask                 */

#define PWM_CTL_DTEN1_Pos                (5)                                               /*!< PWM_T::CTL: DTEN1 Position             */
#define PWM_CTL_DTEN1_Msk                (0x1ul << PWM_CTL_DTEN1_Pos)                      /*!< PWM_T::CTL: DTEN1 Mask                 */

#define PWM_PERIOD_PERIOD_Pos            (0)                                               /*!< PWM_T::PERIOD: PERIOD Position         */
#define PWM_PERIOD_PERIOD_Msk            (0xfffful << PWM_PERIOD_PERIOD_Pos)               /*!< PWM_T::PERIOD: PERIOD Mask             */

#define PWM_CMPDAT0_CMP_Pos              (0)                                               /*!< PWM_T::CMPDAT0: CMP Position           */
#define PWM_CMPDAT0_CMP_Msk              (0xfffful << PWM_CMPDAT0_CMP_Pos)                 /*!< PWM_T::CMPDAT0: CMP Mask               */

#define PWM_CNT_CNT_Pos                  (0)                                               /*!< PWM_T::CNT: CNT Position               */
#define PWM_CNT_CNT_Msk                  (0xfffful << PWM_CNT_CNT_Pos)                     /*!< PWM_T::CNT: CNT Mask                   */

#define PWM_CMPDAT1_CMP_Pos              (0)                                               /*!< PWM_T::CMPDAT1: CMP Position           */
#define PWM_CMPDAT1_CMP_Msk              (0xfffful << PWM_CMPDAT1_CMP_Pos)                 /*!< PWM_T::CMPDAT1: CMP Mask               */

#define PWM_CMPDAT2_CMP_Pos              (0)                                               /*!< PWM_T::CMPDAT2: CMP Position           */
#define PWM_CMPDAT2_CMP_Msk              (0xfffful << PWM_CMPDAT2_CMP_Pos)                 /*!< PWM_T::CMPDAT2: CMP Mask               */

#define PWM_CMPDAT3_CMP_Pos              (0)                                               /*!< PWM_T::CMPDAT3: CMP Position           */
#define PWM_CMPDAT3_CMP_Msk              (0xfffful << PWM_CMPDAT3_CMP_Pos)                 /*!< PWM_T::CMPDAT3: CMP Mask               */

#define PWM_INTEN_PIEN_Pos               (0)                                               /*!< PWM_T::INTEN: PIEN Position            */
#define PWM_INTEN_PIEN_Msk               (0x1ul << PWM_INTEN_PIEN_Pos)                     /*!< PWM_T::INTEN: PIEN Mask                */

#define PWM_INTSTS_PIF_Pos               (0)                                               /*!< PWM_T::INTSTS: PIF Position            */
#define PWM_INTSTS_PIF_Msk               (0x1ul << PWM_INTSTS_PIF_Pos)                     /*!< PWM_T::INTSTS: PIF Mask                */

#define PWM_CAPCTL_CAPINV_Pos            (0)                                               /*!< PWM_T::CAPCTL: CAPINV Position         */
#define PWM_CAPCTL_CAPINV_Msk            (0x1ul << PWM_CAPCTL_CAPINV_Pos)                  /*!< PWM_T::CAPCTL: CAPINV Mask             */

#define PWM_CAPCTL_CRLIEN_Pos            (1)                                               /*!< PWM_T::CAPCTL: CRLIEN Position         */
#define PWM_CAPCTL_CRLIEN_Msk            (0x1ul << PWM_CAPCTL_CRLIEN_Pos)                  /*!< PWM_T::CAPCTL: CRLIEN Mask             */

#define PWM_CAPCTL_CFLIEN_Pos            (2)                                               /*!< PWM_T::CAPCTL: CFLIEN Position         */
#define PWM_CAPCTL_CFLIEN_Msk            (0x1ul << PWM_CAPCTL_CFLIEN_Pos)                  /*!< PWM_T::CAPCTL: CFLIEN Mask             */

#define PWM_CAPCTL_CAPEN_Pos             (3)                                               /*!< PWM_T::CAPCTL: CAPEN Position          */
#define PWM_CAPCTL_CAPEN_Msk             (0x1ul << PWM_CAPCTL_CAPEN_Pos)                   /*!< PWM_T::CAPCTL: CAPEN Mask              */

#define PWM_CAPCTL_CAPIF_Pos             (4)                                               /*!< PWM_T::CAPCTL: CAPIF Position          */
#define PWM_CAPCTL_CAPIF_Msk             (0x1ul << PWM_CAPCTL_CAPIF_Pos)                   /*!< PWM_T::CAPCTL: CAPIF Mask              */

#define PWM_CAPCTL_CRLIF_Pos             (6)                                               /*!< PWM_T::CAPCTL: CRLIF Position          */
#define PWM_CAPCTL_CRLIF_Msk             (0x1ul << PWM_CAPCTL_CRLIF_Pos)                   /*!< PWM_T::CAPCTL: CRLIF Mask              */

#define PWM_CAPCTL_CFLIF_Pos             (7)                                               /*!< PWM_T::CAPCTL: CFLIF Position          */
#define PWM_CAPCTL_CFLIF_Msk             (0x1ul << PWM_CAPCTL_CFLIF_Pos)                   /*!< PWM_T::CAPCTL: CFLIF Mask              */

#define PWM_RCAPDAT_RCAPDAT_Pos          (0)                                               /*!< PWM_T::RCAPDAT: RCAPDAT Position       */
#define PWM_RCAPDAT_RCAPDAT_Msk          (0xfffful << PWM_RCAPDAT_RCAPDAT_Pos)             /*!< PWM_T::RCAPDAT: RCAPDAT Mask           */

#define PWM_FCAPDAT_FCAPDAT_Pos          (0)                                               /*!< PWM_T::FCAPDAT: FCAPDAT Position       */
#define PWM_FCAPDAT_FCAPDAT_Msk          (0xfffful << PWM_FCAPDAT_FCAPDAT_Pos)             /*!< PWM_T::FCAPDAT: FCAPDAT Mask           */

#define PWM_PCEN_POEN0_Pos               (0)                                               /*!< PWM_T::PCEN: POEN0 Position            */
#define PWM_PCEN_POEN0_Msk               (0x1ul << PWM_PCEN_POEN0_Pos)                     /*!< PWM_T::PCEN: POEN0 Mask                */

#define PWM_PCEN_POEN1_Pos               (1)                                               /*!< PWM_T::PCEN: POEN1 Position            */
#define PWM_PCEN_POEN1_Msk               (0x1ul << PWM_PCEN_POEN1_Pos)                     /*!< PWM_T::PCEN: POEN1 Mask                */

#define PWM_PCEN_POEN2_Pos               (2)                                               /*!< PWM_T::PCEN: POEN2 Position            */
#define PWM_PCEN_POEN2_Msk               (0x1ul << PWM_PCEN_POEN2_Pos)                     /*!< PWM_T::PCEN: POEN2 Mask                */

#define PWM_PCEN_POEN3_Pos               (3)                                               /*!< PWM_T::PCEN: POEN3 Position            */
#define PWM_PCEN_POEN3_Msk               (0x1ul << PWM_PCEN_POEN3_Pos)                     /*!< PWM_T::PCEN: POEN3 Mask                */

#define PWM_PCEN_CAPINEN_Pos             (8)                                               /*!< PWM_T::PCEN: CAPINEN Position          */
#define PWM_PCEN_CAPINEN_Msk             (0x1ul << PWM_PCEN_CAPINEN_Pos)                   /*!< PWM_T::PCEN: CAPINEN Mask              */

/**@}*/ /* PWM_CONST */
/**@}*/ /* end of PWM register group */


/*---------------------- Successive Approximation Analog-to-Digital Convertor -------------------------*/
/**
    @addtogroup SARADC Successive Approximation Analog-to-Digital Convertor(SARADC)
    Memory Mapped Structure for SARADC Controller
@{ */
 
typedef struct
{


/**
 * @var SARADC_T::DAT0
 * Offset: 0x00  SARADC Data Register for the Channel Defined in CHSEQ0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT1
 * Offset: 0x04  SARADC Data Register for the Channel Defined in CHSEQ1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT2
 * Offset: 0x08  SARADC Data Register for the Channel Defined in CHSEQ2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT3
 * Offset: 0x0C  SARADC Data Register for the Channel Defined in CHSEQ3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT4
 * Offset: 0x10  SARADC Data Register for the Channel Defined in CHSEQ4
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT5
 * Offset: 0x14  SARADC Data Register for the Channel Defined in CHSEQ5
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT6
 * Offset: 0x18  SARADC Data Register for the Channel Defined in CHSEQ6
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT7
 * Offset: 0x1C  SARADC Data Register for the Channel Defined in CHSEQ7
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT8
 * Offset: 0x20  SARADC Data Register for the Channel Defined in CHSEQ8
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT9
 * Offset: 0x24  SARADC Data Register for the Channel Defined in CHSEQ9
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT10
 * Offset: 0x28  SARADC Data Register for the Channel Defined in CHSEQ10
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT11
 * Offset: 0x2C  SARADC Data Register for the Channel Defined in CHSEQ11
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT12
 * Offset: 0x30  SARADC Data Register for the Channel Defined in CHSEQ12
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT13
 * Offset: 0x34  SARADC Data Register for the Channel Defined in CHSEQ13
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::DAT14
 * Offset: 0x38  SARADC Data Register for the Channel Defined in CHSEQ14
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
 * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
 * |        |          |If ADCFM is u201C0u201D, EXTS all are read as u201C0u201D.
 * |        |          |If ADCFM is u201C1u201D, EXTS all are read as bit RESULT [11].
 * |[16]    |OV        |Over Run Flag
 * |        |          |0 = Data in RESULT are recent conversion result.
 * |        |          |1 = Data in RESULT are overwritten.
 * |        |          |If converted data in RESULT [11:0] have not been read before new conversion result is loaded to this register, OV is set to 1
 * |        |          |It is cleared by hardware after ADC_DAT register is read.
 * |[17]    |VALID     |Valid Flag
 * |        |          |0 = Data in RESULT are not valid.
 * |        |          |1 = Data in RESULT are valid.
 * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
 * @var SARADC_T::CTL
 * Offset: 0x3C  SARADC Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADCEN     |A/D Converter Enable
 * |        |          |0 = Disable
 * |        |          |1 = Enable
 * |        |          |Before starting A/D conversion function, this bit should be set to 1
 * |        |          |Clear it to 0 to disable A/D converter analog circuit power consumption.
 * |[1]     |ADCIE     |A/D Interrupt Enable
 * |        |          |0 = Disable A/D interrupt function
 * |        |          |1 = Enable A/D interrupt function
 * |        |          |A/D conversion end interrupt request is generated if ADCIE bit is set to 1.
 * |[3:2]   |OPMODE    |A/D Converter Operation Mode
 * |        |          |00 = Single conversion
 * |        |          |01 = Reserved
 * |        |          |10 = Single-cycle scan
 * |        |          |11 = Continuous scan
 * |        |          |Note 1: When changing the operation mode, software should disable SWTRG bit firstly.
 * |[4]     |PDMAEN    |PDMA Transfer Enable Bit
 * |        |          |When A/D conversion is completed, the converted data is loaded into ADC_DATn (n: 0 ~ 1314) register, user can enable this bit to generate a PDMA data transfer request.
 * |        |          |0 = PDMA data transfer Disabled.
 * |        |          |1 = PDMA data transfer Enabled. 
 * |[5]     |MUXSW     |MUXEN software control register
 * |        |          |MUXSW is used for MUXEN control. When MUXSW = 0u2019b, MUX is always enable.
 * |        |          |0 : MUX always enable turn on
 * |        |          |1 : MUX control by MUXEN
 * |[7:6]   |DLYTRIM   |Trim bit for SARADC speed
 * |        |          |00 = 636.5284ns
 * |        |          |01 = 720.5ns
 * |        |          |10 = 807ns
 * |        |          |11 = 976.5ns
 * |[8]     |MUXEN     |Input channel MUX enable control
 * |        |          |0 = Disable(MUX output floating)
 * |        |          |1 = Enable
 * |        |          |Note: Only works when MUXSW = 1.
 * |[9]     |MODESEL   |SARADC conversion speed mode selection
 * |        |          |0 = High speed(500KSPS)
 * |        |          |1 = Low speed(200KSPS)
 * |[10]    |OVRIE     |Sample rate over interrupt enable
 * |        |          |0 = Disable
 * |        |          |1 = Enable
 * |[11]    |SWTRG     |A/D Conversion Start
 * |        |          |0 = Conversion is stopped and A/D converter enters idle state.
 * |        |          |1 = Start conversion.
 * |        |          |Note1: SWTRG bit can be reset to 0 by software, or can be cleared to 0 by hardware automatically at the end of single mode and single-cycle scan mode on specified channel
 * |        |          |In continuous scan mode, A/D conversion is continuously performed sequentially until software writes 0 to this bit or chip resets.
 * |        |          |Note2: Before trigger SWTRG to start ADC convert , the ADC relative setting should be completed. 
 * |[12]    |ADCFM     |Data Format Of ADC Conversion Result
 * |        |          |0 = Unsigned
 * |        |          |1 = 2u2019s Complement
 * @var SARADC_T::CHSEQ0
 * Offset: 0x40  SARADC Channel Sequence Register0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |CHSEQ0    |Select Channel N As The 1st Conversion In Scan Sequence
 * |        |          |One of the following channel is selected according to CHSEQ0 [3:0].
 * |        |          |CHSEQ0
 * |        |          |Selected channel to   ADC input
 * |        |          |0000
 * |        |          |Channel 0
 * |        |          |0001
 * |        |          |Channel 1
 * |        |          |0010
 * |        |          |Channel 2
 * |        |          |0011
 * |        |          |Channel 3
 * |        |          |0100
 * |        |          |Channel 4
 * |        |          |0101
 * |        |          |Channel 5
 * |        |          |0110
 * |        |          |Channel 6
 * |        |          |0111
 * |        |          |Channel 7
 * |        |          |1000
 * |        |          |Channel 8
 * |        |          |1000
 * |        |          |Channel 8
 * |        |          |1001
 * |        |          |Channel 9
 * |        |          |1010
 * |        |          |Channel 10
 * |        |          |1011
 * |        |          |Channel 11
 * |        |          |1100
 * |        |          |mLDO
 * |        |          |1101
 * |        |          |uLDO
 * |        |          | CHSEQ0 = 1111: No channel is selected, scan sequence is stopped.
 * |[7:4]   |CHSEQ1    |Select Channel N As The 2nd Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * |[11:8]  |CHSEQ2    |Select Channel N As The 3rd Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * |[15:12] |CHSEQ3    |Select Channel N As The 4th Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * |[19:16] |CHSEQ4    |Select Channel N As The 5th Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * |[23:20] |CHSEQ5    |Select Channel N As The 6th Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * |[27:24] |CHSEQ6    |Select Channel N As The 7th Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * |[31:28] |CHSEQ7    |Select Channel N As The 8th Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * @var SARADC_T::CHSEQ1
 * Offset: 0x44  SARADC Channel Sequence Register1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |CHSEQ8    |Select Channel N As The 9th Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * |[7:4]   |CHSEQ9    |Select Channel N As The 10th Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * |[11:8]  |CHSEQ10   |Select Channel N As The 11th Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * |[15:12] |CHSEQ11   |Select Channel N As The 12th Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * |[19:16] |CHSEQ12   |Select Channel N As The 13th Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * |[23:20] |CHSEQ13   |Select Channel N As The 14th Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * |[27:24] |CHSEQ14   |Select Channel N As The 15th Conversion In Scan Sequence
 * |        |          |The definition of channel selection is the same as CHSEQ0.
 * @var SARADC_T::CMP0
 * Offset: 0x48  SARADC Compare Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADCMPEN   |Compare Enable
 * |        |          |0 = Disable compare.
 * |        |          |1 = Enable compare.
 * |        |          |Set this bit to 1 to enable the comparison CMPDAT with specified channel conversion result when converted data is loaded into SARADC_DAT register.
 * |[1]     |ADCMPIE   |Compare Interrupt Enable
 * |        |          |0 = Disable
 * |        |          |1 = Enable
 * |        |          |When converted data in RESULT is less (or greater) than the compare data CMPDAT, ADCMPF bit is asserted
 * |        |          |If ADCMPIE is set to 1, a compare interrupt request is generated.
 * |[2]     |CMPCOND   |Compare Condition
 * |        |          |0 = ADCMPFx bit is set if conversion result is less than CMPDAT.
 * |        |          |1 = ADCMPFx bit is set if conversion result is greater or equal to CMPDAT,
 * |[6:3]   |CMPCH     |Compare Channel Selection
 * |        |          |0000 = Channel 0 conversion result is selected to be compared.
 * |        |          |0001 = Channel 1 conversion result is selected to be compared.
 * |        |          |0010 = Channel 2 conversion result is selected to be compared.
 * |        |          |0011 = Channel 3 conversion result is selected to be compared.
 * |        |          |0100 = Channel 4 conversion result is selected to be compared.
 * |        |          |0101 = Channel 5 conversion result is selected to be compared.
 * |        |          |0110 = Channel 6 conversion result is selected to be compared.
 * |        |          |0111 = Channel 7 conversion result is selected to be compared.
 * |        |          |1000 = Channel 8 conversion result is selected to be compared.
 * |        |          |1001 = Channel 9 conversion result is selected to be compared.
 * |        |          |1010 = Channel 10 conversion result is selected to be compared.
 * |        |          |1011 = Channel 11 conversion result is selected to be compared.
 * |        |          |1100 = Channel 12 conversion result is selected to be compared.
 * |        |          |1101 = Channel 13 conversion result is selected to be compared.
 * |        |          |1110 = Channel 14 conversion result is selected to be compared.
 * |        |          |Others = Reserved
 * |[11:8]  |CMPMCNT   |Compare Match Count
 * |        |          |When the specified A/D channel analog conversion result matches the comparing condition, the internal match counter will increase 1
 * |        |          |When the internal counter achieves the setting, (CMPMCNT+1) hardware will set the ADCMPF bit.
 * |[27:16] |CMPDAT    |Compare Data
 * |        |          |This field possessing 12-bit compare data, is used to compare with conversion result of specified channel
 * |        |          |Software can use it to monitor the external analog input pin voltage transition in scan mode without imposing a load on software.
 * |        |          |The data format should be consistent with the setting of ADCFM bit.
 * @var SARADC_T::CMP1
 * Offset: 0x4C  SARADC Compare Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADCMPEN   |Compare Enable
 * |        |          |0 = Disable compare.
 * |        |          |1 = Enable compare.
 * |        |          |Set this bit to 1 to enable the comparison CMPDAT with specified channel conversion result when converted data is loaded into SARADC_DAT register.
 * |[1]     |ADCMPIE   |Compare Interrupt Enable
 * |        |          |0 = Disable
 * |        |          |1 = Enable
 * |        |          |When converted data in RESULT is less (or greater) than the compare data CMPDAT, ADCMPF bit is asserted
 * |        |          |If ADCMPIE is set to 1, a compare interrupt request is generated.
 * |[2]     |CMPCOND   |Compare Condition
 * |        |          |0 = ADCMPFx bit is set if conversion result is less than CMPDAT.
 * |        |          |1 = ADCMPFx bit is set if conversion result is greater or equal to CMPDAT,
 * |[6:3]   |CMPCH     |Compare Channel Selection
 * |        |          |0000 = Channel 0 conversion result is selected to be compared.
 * |        |          |0001 = Channel 1 conversion result is selected to be compared.
 * |        |          |0010 = Channel 2 conversion result is selected to be compared.
 * |        |          |0011 = Channel 3 conversion result is selected to be compared.
 * |        |          |0100 = Channel 4 conversion result is selected to be compared.
 * |        |          |0101 = Channel 5 conversion result is selected to be compared.
 * |        |          |0110 = Channel 6 conversion result is selected to be compared.
 * |        |          |0111 = Channel 7 conversion result is selected to be compared.
 * |        |          |1000 = Channel 8 conversion result is selected to be compared.
 * |        |          |1001 = Channel 9 conversion result is selected to be compared.
 * |        |          |1010 = Channel 10 conversion result is selected to be compared.
 * |        |          |1011 = Channel 11 conversion result is selected to be compared.
 * |        |          |1100 = Channel 12 conversion result is selected to be compared.
 * |        |          |1101 = Channel 13 conversion result is selected to be compared.
 * |        |          |1110 = Channel 14 conversion result is selected to be compared.
 * |        |          |Others = Reserved
 * |[11:8]  |CMPMCNT   |Compare Match Count
 * |        |          |When the specified A/D channel analog conversion result matches the comparing condition, the internal match counter will increase 1
 * |        |          |When the internal counter achieves the setting, (CMPMCNT+1) hardware will set the ADCMPF bit.
 * |[27:16] |CMPDAT    |Compare Data
 * |        |          |This field possessing 12-bit compare data, is used to compare with conversion result of specified channel
 * |        |          |Software can use it to monitor the external analog input pin voltage transition in scan mode without imposing a load on software.
 * |        |          |The data format should be consistent with the setting of ADCFM bit.
 * @var SARADC_T::STATUS0
 * Offset: 0x50  SARADC Status Register0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADIF      |A/D Conversion End Flag
 * |        |          |A status flag that indicates the end of A/D conversion.
 * |        |          |ADIF is set to 1 under the following two conditions:
 * |        |          |1. When A/D conversion ends in single mode,
 * |        |          |2. When A/D conversion ends on all channels specified by channel sequence register in scan mode.
 * |        |          |And it is cleared when 1 is written.
 * |[1]     |ADCMPF0   |Compare Flag0
 * |        |          |When the selected channel A/D conversion result meets setting conditions in SARADC_CMP0, then this bit is set to 1
 * |        |          |And it is cleared by write 1.
 * |        |          |0 = Converted result RESULT in SARADC_DAT does not meet SARADC_CMP0 setting.
 * |        |          |1 = Converted result RESULT in SARADC_DAT meets SARADC_CMP0 setting,
 * |[2]     |ADCMPF1   |Compare Flag1
 * |        |          |When the selected channel A/D conversion result meets setting conditions in SARADC_CMP1, then this bit is set to 1
 * |        |          |And it is cleared by write 1.
 * |        |          |0 = Converted result RESULT in SARADC_DAT does not meet SARADC_CMP1 setting.
 * |        |          |1 = Converted result RESULT in SARADC_DAT meets SARADC_CMP1 setting,
 * |[3]     |BUSY      |BUSY/IDLE
 * |        |          |0 = A/D converter is in idle state.
 * |        |          |1 = A/D converter is busy at conversion.
 * |        |          |This bit is mirror of SWTRG bit in SARADC_CTL.
 * |        |          |It is read only.
 * |[7:4]   |CHANNEL   |Current Conversion Channel
 * |        |          |This filed reflects current conversion channel when BUSY=1
 * |        |          |When BUSY=0, it shows the next channel will be converted.
 * |        |          |It is read only.
 * |[8]     |OVRF      |Sampling rate over flag
 * |        |          |0= user setting sample rate not exceed real conversion rate
 * |        |          |1= user setting sample rate exceed real conversion rate
 * |        |          |It is cleared when 1 is written.
 * @var SARADC_T::STATUS1
 * Offset: 0x54  SARADC Status Register1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[14:0]  |VALID     |Data Valid Flag
 * |        |          |It is a mirror of VALID bit in SARADC_DATn.
 * |[30:16] |OV        |Over Run Flag
 * |        |          |It is a mirror to OV bit in SARADC_DATn.
 * @var SARADC_T::PDMADAT
 * Offset: 0x58  SARADC PDMA Result Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |DATA      |SARADC PDMA transfer data
 * |        |          |This register is a shadow register of SARADC_DATn (n=0~14) for PDMA support.
 * |        |          |This is a read only register.
 * @var SARADC_T::HWPARA
 * Offset: 0x5C  SARADC H/W Parameter Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:0]   |SHCLKN    |Specify the high level of SARADC start signal.
 * |        |          |SARADC start signal high level duration time = SARADC_CLK x (SHCLKN + 1).
 * |        |          |Note: SHCLKN must larger than 400ns.
 * |[14:8]  |CONVN     |Specify SARADC conversion clock number
 * |        |          |SARADC Conversion clock number = (CONVN + 1).
 * |        |          |To update this field, programmer can only revise bit [14:8] and keep other bits the same as before.
 * |        |          |Note: CONVN value must bigger than SHCLKN value and should bigger than 2us(500KSPS).
 */
    __I  uint32_t DAT[15];               /*!< [0x0000] SAR ADC Data Register 0~14                                       */
    __IO uint32_t CTL;                   /*!< [0x003c] SARADC Control Register                                          */
    __IO uint32_t CHSEQ0;                /*!< [0x0040] SARADC Channel Sequence Register0                                */
    __IO uint32_t CHSEQ1;                /*!< [0x0044] SARADC Channel Sequence Register1                                */
    __IO uint32_t CMP[2];                /*!< [0x0048] SARADC Compare Register 0                                        */
    __IO uint32_t STATUS0;               /*!< [0x0050] SARADC Status Register0                                          */
    __IO uint32_t STATUS1;               /*!< [0x0054] SARADC Status Register1                                          */
    __IO uint32_t PDMADAT;               /*!< [0x0058] SARADC PDMA Result Register                                      */
    __IO uint32_t HWPARA;                /*!< [0x005c] SARADC H/W Parameter Control Register                            */

} SARADC_T;

/**
    @addtogroup SARADC_CONST SARADC Bit Field Definition
    Constant Definitions for SARADC Controller
@{ */

#define SARADC_DAT_RESULT_Pos            (0)                                               /*!< SARADC_T::DAT0: RESULT Position        */
#define SARADC_DAT_RESULT_Msk            (0xffful << SARADC_DAT_RESULT_Pos)                /*!< SARADC_T::DAT0: RESULT Mask            */
  
#define SARADC_DAT_EXTS_Pos              (12)                                              /*!< SARADC_T::DAT0: EXTS Position          */
#define SARADC_DAT_EXTS_Msk              (0xful << SARADC_DAT_EXTS_Pos)                    /*!< SARADC_T::DAT0: EXTS Mask              */
 
#define SARADC_DAT_OV_Pos                (16)                                              /*!< SARADC_T::DAT0: OV Position            */
#define SARADC_DAT_OV_Msk                (0x1ul << SARADC_DAT_OV_Pos)                      /*!< SARADC_T::DAT0: OV Mask                */

#define SARADC_DAT_VALID_Pos             (17)                                              /*!< SARADC_T::DAT0: VALID Position         */
#define SARADC_DAT_VALID_Msk             (0x1ul << SARADC_DAT_VALID_Pos)                   /*!< SARADC_T::DAT0: VALID Mask             */

#define SARADC_CTL_ADCEN_Pos             (0)                                               /*!< SARADC_T::CTL: ADCEN Position          */
#define SARADC_CTL_ADCEN_Msk             (0x1ul << SARADC_CTL_ADCEN_Pos)                   /*!< SARADC_T::CTL: ADCEN Mask              */

#define SARADC_CTL_ADCIE_Pos             (1)                                               /*!< SARADC_T::CTL: ADCIE Position          */
#define SARADC_CTL_ADCIE_Msk             (0x1ul << SARADC_CTL_ADCIE_Pos)                   /*!< SARADC_T::CTL: ADCIE Mask              */

#define SARADC_CTL_OPMODE_Pos            (2)                                               /*!< SARADC_T::CTL: OPMODE Position         */
#define SARADC_CTL_OPMODE_Msk            (0x3ul << SARADC_CTL_OPMODE_Pos)                  /*!< SARADC_T::CTL: OPMODE Mask             */

#define SARADC_CTL_PDMAEN_Pos            (4)                                               /*!< SARADC_T::CTL: PDMAEN Position         */
#define SARADC_CTL_PDMAEN_Msk            (0x1ul << SARADC_CTL_PDMAEN_Pos)                  /*!< SARADC_T::CTL: PDMAEN Mask             */

#define SARADC_CTL_MUXSW_Pos             (5)                                               /*!< SARADC_T::CTL: MUXSW Position          */
#define SARADC_CTL_MUXSW_Msk             (0x1ul << SARADC_CTL_MUXSW_Pos)                   /*!< SARADC_T::CTL: MUXSW Mask              */

#define SARADC_CTL_DLYTRIM_Pos           (6)                                               /*!< SARADC_T::CTL: DLYTRIM Position        */
#define SARADC_CTL_DLYTRIM_Msk           (0x3ul << SARADC_CTL_DLYTRIM_Pos)                 /*!< SARADC_T::CTL: DLYTRIM Mask            */

#define SARADC_CTL_MUXEN_Pos             (8)                                               /*!< SARADC_T::CTL: MUXEN Position          */
#define SARADC_CTL_MUXEN_Msk             (0x1ul << SARADC_CTL_MUXEN_Pos)                   /*!< SARADC_T::CTL: MUXEN Mask              */

#define SARADC_CTL_MODESEL_Pos           (9)                                               /*!< SARADC_T::CTL: MODESEL Position        */
#define SARADC_CTL_MODESEL_Msk           (0x1ul << SARADC_CTL_MODESEL_Pos)                 /*!< SARADC_T::CTL: MODESEL Mask            */

#define SARADC_CTL_OVRIE_Pos             (10)                                              /*!< SARADC_T::CTL: OVRIE Position          */
#define SARADC_CTL_OVRIE_Msk             (0x1ul << SARADC_CTL_OVRIE_Pos)                   /*!< SARADC_T::CTL: OVRIE Mask              */

#define SARADC_CTL_SWTRG_Pos             (11)                                              /*!< SARADC_T::CTL: SWTRG Position          */
#define SARADC_CTL_SWTRG_Msk             (0x1ul << SARADC_CTL_SWTRG_Pos)                   /*!< SARADC_T::CTL: SWTRG Mask              */

#define SARADC_CTL_ADCFM_Pos             (12)                                              /*!< SARADC_T::CTL: ADCFM Position          */
#define SARADC_CTL_ADCFM_Msk             (0x1ul << SARADC_CTL_ADCFM_Pos)                   /*!< SARADC_T::CTL: ADCFM Mask              */

#define SARADC_CHSEQ0_CHSEQ0_Pos         (0)                                               /*!< SARADC_T::CHSEQ0: CHSEQ0 Position      */
#define SARADC_CHSEQ0_CHSEQ0_Msk         (0xful << SARADC_CHSEQ0_CHSEQ0_Pos)               /*!< SARADC_T::CHSEQ0: CHSEQ0 Mask          */

#define SARADC_CHSEQ0_CHSEQ1_Pos         (4)                                               /*!< SARADC_T::CHSEQ0: CHSEQ1 Position      */
#define SARADC_CHSEQ0_CHSEQ1_Msk         (0xful << SARADC_CHSEQ0_CHSEQ1_Pos)               /*!< SARADC_T::CHSEQ0: CHSEQ1 Mask          */

#define SARADC_CHSEQ0_CHSEQ2_Pos         (8)                                               /*!< SARADC_T::CHSEQ0: CHSEQ2 Position      */
#define SARADC_CHSEQ0_CHSEQ2_Msk         (0xful << SARADC_CHSEQ0_CHSEQ2_Pos)               /*!< SARADC_T::CHSEQ0: CHSEQ2 Mask          */

#define SARADC_CHSEQ0_CHSEQ3_Pos         (12)                                              /*!< SARADC_T::CHSEQ0: CHSEQ3 Position      */
#define SARADC_CHSEQ0_CHSEQ3_Msk         (0xful << SARADC_CHSEQ0_CHSEQ3_Pos)               /*!< SARADC_T::CHSEQ0: CHSEQ3 Mask          */

#define SARADC_CHSEQ0_CHSEQ4_Pos         (16)                                              /*!< SARADC_T::CHSEQ0: CHSEQ4 Position      */
#define SARADC_CHSEQ0_CHSEQ4_Msk         (0xful << SARADC_CHSEQ0_CHSEQ4_Pos)               /*!< SARADC_T::CHSEQ0: CHSEQ4 Mask          */

#define SARADC_CHSEQ0_CHSEQ5_Pos         (20)                                              /*!< SARADC_T::CHSEQ0: CHSEQ5 Position      */
#define SARADC_CHSEQ0_CHSEQ5_Msk         (0xful << SARADC_CHSEQ0_CHSEQ5_Pos)               /*!< SARADC_T::CHSEQ0: CHSEQ5 Mask          */

#define SARADC_CHSEQ0_CHSEQ6_Pos         (24)                                              /*!< SARADC_T::CHSEQ0: CHSEQ6 Position      */
#define SARADC_CHSEQ0_CHSEQ6_Msk         (0xful << SARADC_CHSEQ0_CHSEQ6_Pos)               /*!< SARADC_T::CHSEQ0: CHSEQ6 Mask          */

#define SARADC_CHSEQ0_CHSEQ7_Pos         (28)                                              /*!< SARADC_T::CHSEQ0: CHSEQ7 Position      */
#define SARADC_CHSEQ0_CHSEQ7_Msk         (0xful << SARADC_CHSEQ0_CHSEQ7_Pos)               /*!< SARADC_T::CHSEQ0: CHSEQ7 Mask          */

#define SARADC_CHSEQ1_CHSEQ8_Pos         (0)                                               /*!< SARADC_T::CHSEQ1: CHSEQ8 Position      */
#define SARADC_CHSEQ1_CHSEQ8_Msk         (0xful << SARADC_CHSEQ1_CHSEQ8_Pos)               /*!< SARADC_T::CHSEQ1: CHSEQ8 Mask          */

#define SARADC_CHSEQ1_CHSEQ9_Pos         (4)                                               /*!< SARADC_T::CHSEQ1: CHSEQ9 Position      */
#define SARADC_CHSEQ1_CHSEQ9_Msk         (0xful << SARADC_CHSEQ1_CHSEQ9_Pos)               /*!< SARADC_T::CHSEQ1: CHSEQ9 Mask          */

#define SARADC_CHSEQ1_CHSEQ10_Pos        (8)                                               /*!< SARADC_T::CHSEQ1: CHSEQ10 Position     */
#define SARADC_CHSEQ1_CHSEQ10_Msk        (0xful << SARADC_CHSEQ1_CHSEQ10_Pos)              /*!< SARADC_T::CHSEQ1: CHSEQ10 Mask         */

#define SARADC_CHSEQ1_CHSEQ11_Pos        (12)                                              /*!< SARADC_T::CHSEQ1: CHSEQ11 Position     */
#define SARADC_CHSEQ1_CHSEQ11_Msk        (0xful << SARADC_CHSEQ1_CHSEQ11_Pos)              /*!< SARADC_T::CHSEQ1: CHSEQ11 Mask         */

#define SARADC_CHSEQ1_CHSEQ12_Pos        (16)                                              /*!< SARADC_T::CHSEQ1: CHSEQ12 Position     */
#define SARADC_CHSEQ1_CHSEQ12_Msk        (0xful << SARADC_CHSEQ1_CHSEQ12_Pos)              /*!< SARADC_T::CHSEQ1: CHSEQ12 Mask         */

#define SARADC_CHSEQ1_CHSEQ13_Pos        (20)                                              /*!< SARADC_T::CHSEQ1: CHSEQ13 Position     */
#define SARADC_CHSEQ1_CHSEQ13_Msk        (0xful << SARADC_CHSEQ1_CHSEQ13_Pos)              /*!< SARADC_T::CHSEQ1: CHSEQ13 Mask         */

#define SARADC_CHSEQ1_CHSEQ14_Pos        (24)                                              /*!< SARADC_T::CHSEQ1: CHSEQ14 Position     */
#define SARADC_CHSEQ1_CHSEQ14_Msk        (0xful << SARADC_CHSEQ1_CHSEQ14_Pos)              /*!< SARADC_T::CHSEQ1: CHSEQ14 Mask         */

#define SARADC_CMP_ADCMPEN_Pos          (0)                                                /*!< SARADC_T::CMP0: ADCMPEN Position       */
#define SARADC_CMP_ADCMPEN_Msk          (0x1ul << SARADC_CMP_ADCMPEN_Pos)                  /*!< SARADC_T::CMP0: ADCMPEN Mask           */

#define SARADC_CMP_ADCMPIE_Pos          (1)                                                /*!< SARADC_T::CMP0: ADCMPIE Position       */
#define SARADC_CMP_ADCMPIE_Msk          (0x1ul << SARADC_CMP_ADCMPIE_Pos)                  /*!< SARADC_T::CMP0: ADCMPIE Mask           */

#define SARADC_CMP_CMPCOND_Pos          (2)                                                /*!< SARADC_T::CMP0: CMPCOND Position       */
#define SARADC_CMP_CMPCOND_Msk          (0x1ul << SARADC_CMP_CMPCOND_Pos)                  /*!< SARADC_T::CMP0: CMPCOND Mask           */

#define SARADC_CMP_CMPCH_Pos            (3)                                                /*!< SARADC_T::CMP0: CMPCH Position         */
#define SARADC_CMP_CMPCH_Msk            (0xful << SARADC_CMP_CMPCH_Pos)                    /*!< SARADC_T::CMP0: CMPCH Mask             */

#define SARADC_CMP_CMPMCNT_Pos          (8)                                                /*!< SARADC_T::CMP0: CMPMCNT Position       */
#define SARADC_CMP_CMPMCNT_Msk          (0xful << SARADC_CMP_CMPMCNT_Pos)                  /*!< SARADC_T::CMP0: CMPMCNT Mask           */

#define SARADC_CMP_CMPDAT_Pos           (16)                                               /*!< SARADC_T::CMP0: CMPDAT Position        */
#define SARADC_CMP_CMPDAT_Msk           (0xffful << SARADC_CMP_CMPDAT_Pos)                 /*!< SARADC_T::CMP0: CMPDAT Mask            */

#define SARADC_STATUS0_ADIF_Pos          (0)                                               /*!< SARADC_T::STATUS0: ADIF Position       */
#define SARADC_STATUS0_ADIF_Msk          (0x1ul << SARADC_STATUS0_ADIF_Pos)                /*!< SARADC_T::STATUS0: ADIF Mask           */

#define SARADC_STATUS0_ADCMPF0_Pos       (1)                                               /*!< SARADC_T::STATUS0: ADCMPF0 Position    */
#define SARADC_STATUS0_ADCMPF0_Msk       (0x1ul << SARADC_STATUS0_ADCMPF0_Pos)             /*!< SARADC_T::STATUS0: ADCMPF0 Mask        */

#define SARADC_STATUS0_ADCMPF1_Pos       (2)                                               /*!< SARADC_T::STATUS0: ADCMPF1 Position    */
#define SARADC_STATUS0_ADCMPF1_Msk       (0x1ul << SARADC_STATUS0_ADCMPF1_Pos)             /*!< SARADC_T::STATUS0: ADCMPF1 Mask        */

#define SARADC_STATUS0_BUSY_Pos          (3)                                               /*!< SARADC_T::STATUS0: BUSY Position       */
#define SARADC_STATUS0_BUSY_Msk          (0x1ul << SARADC_STATUS0_BUSY_Pos)                /*!< SARADC_T::STATUS0: BUSY Mask           */

#define SARADC_STATUS0_CHANNEL_Pos       (4)                                               /*!< SARADC_T::STATUS0: CHANNEL Position    */
#define SARADC_STATUS0_CHANNEL_Msk       (0xful << SARADC_STATUS0_CHANNEL_Pos)             /*!< SARADC_T::STATUS0: CHANNEL Mask        */

#define SARADC_STATUS0_OVRF_Pos          (8)                                               /*!< SARADC_T::STATUS0: OVRF Position       */
#define SARADC_STATUS0_OVRF_Msk          (0x1ul << SARADC_STATUS0_OVRF_Pos)                /*!< SARADC_T::STATUS0: OVRF Mask           */

#define SARADC_STATUS1_VALID_Pos         (0)                                               /*!< SARADC_T::STATUS1: VALID Position      */
#define SARADC_STATUS1_VALID_Msk         (0x7ffful << SARADC_STATUS1_VALID_Pos)            /*!< SARADC_T::STATUS1: VALID Mask          */

#define SARADC_STATUS1_OV_Pos            (16)                                              /*!< SARADC_T::STATUS1: OV Position         */
#define SARADC_STATUS1_OV_Msk            (0x7ffful << SARADC_STATUS1_OV_Pos)               /*!< SARADC_T::STATUS1: OV Mask             */

#define SARADC_PDMADAT_DATA_Pos          (0)                                               /*!< SARADC_T::PDMADAT: DATA Position       */
#define SARADC_PDMADAT_DATA_Msk          (0xfffful << SARADC_PDMADAT_DATA_Pos)             /*!< SARADC_T::PDMADAT: DATA Mask           */

#define SARADC_HWPARA_SHCLKN_Pos         (0)                                               /*!< SARADC_T::HWPARA: SHCLKN Position      */
#define SARADC_HWPARA_SHCLKN_Msk         (0x3ful << SARADC_HWPARA_SHCLKN_Pos)              /*!< SARADC_T::HWPARA: SHCLKN Mask          */

#define SARADC_HWPARA_CONVN_Pos          (8)                                               /*!< SARADC_T::HWPARA: CONVN Position       */
#define SARADC_HWPARA_CONVN_Msk          (0x7ful << SARADC_HWPARA_CONVN_Pos)               /*!< SARADC_T::HWPARA: CONVN Mask           */

/**@}*/ /* SARADC_CONST */
/**@}*/ /* end of SARADC register group */


/*---------------------- Sigma- Delta Analog-to-Digital Converter -------------------------*/
/**
    @addtogroup SDADC Sigma- Delta Analog-to-Digital Converter(SDADC)
    Memory Mapped Structure for SDADC Controller
@{ */
 
typedef struct
{


/**
 * @var SDADC_T::DAT
 * Offset: 0x00  SD ADC FIFO Data Read Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |RESULT    |Delta-Sigma ADC DATA FIFO Read
 * |        |          |A read of this register will read data from the audio FIFO and increment the read pointer
 * |        |          |A read past empty will repeat the last data
 * |        |          |Can be used with SDADC_FIFOSTS.THIF interrupt to determine if valid data is present in FIFO.
 * |        |          |Data width can be selected by SDADC_CTL.FIFO_BITS
 * @var SDADC_T::EN
 * Offset: 0x04  SD ADC Enable  Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SDADCEN   |ADC   Enable
 * |        |          |1   = ADC Conversion enabled.
 * |        |          |0   = Conversion stopped and ADC is reset including FIFO pointers.
 * |[1]     |DINEDGE   | ADC data input clock edge selection
 * |        |          | 1 = ADC clock positive edge latch
 * |        |          | 0 = ADC clock negetive edge latch
 * |[2]     |DINBYPS   |ADC data input bypass (internal debug)
 * |        |          |1 = analog 5bits to FIFO for testing
 * |        |          |0 = normal mode
 * @var SDADC_T::CLKDIV
 * Offset: 0x08  SD ADC Clock Divider Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |CLKDIV    |ADC Clock Divider
 * |        |          |This register determines the clock division ration between the incoming ADC_CLK and the Sigma-Delta sampling clock of the ADC
 * |        |          |This together with the over-sampling ratio (OSR) determines the audio sample rate of the converter
 * |        |          |CLK_DIV should be set to give a SD_CLK frequency in the range of 1.024-6.144MHz.
 * |        |          |CLKDIV must be greater than and equal 2.
 * |        |          |CLKDIV = ADC Clock/Sample Rate/Down Sample Rate
 * @var SDADC_T::CTL
 * Offset: 0x0C  SD ADC Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |DSRATE    |Down Sampling Ratio
 * |        |          |0 = reserved
 * |        |          |1 = down sample X 16
 * |        |          |2 = down sample X 32
 * |        |          |3 = down sample X 64
 * |[3:2]   |FIFOBITS  |FIFO Data Bits Selection
 * |        |          |0 = 32 bits
 * |        |          |1 = 16 bits
 * |        |          |2 = 8 bits
 * |        |          |3 = 24 bits
 * |[6:4]   |FIFOTH    |FIFO Threshold:
 * |        |          |Determines at what level the ADC FIFO will generate a interrupt.
 * |        |          |Interrupt will be generated when number of words present in ADC FIFO is >
 * |        |          |FIFOTH.
 * |[7]     |FIFOTHIE  |FIFO Threshold Interrupt Enable
 * |        |          |0 = disable interrupt whenever FIFO level exceeds that set in FIFOIELEVFIFOTH.
 * |        |          |1 = enable interrupt whenever FIFO level exceeds that set in FIFOIELEVFIFOTH. 
 * @var SDADC_T::FIFOSTS
 * Offset: 0x10  SD ADC FIFO Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FULL      |FIFO Full
 * |        |          |1 = FIFO is full.
 * |        |          |0 = FIFO is not full.
 * |[1]     |EMPTY     |FIFO Empty
 * |        |          |1= FIFO is empty.
 * |        |          |0= FIFO is not empty.
 * |[2]     |THIF      |ADC FIFO Threshold Interrupt Status (Read Only)
 * |        |          |1 = The valid data count within the ADC FIFO buffer is larger than or equal the setting value of FIFO_IE_LEVTH.
 * |        |          |0 = The valid data count within the transmit FIFO buffer is less than to the setting value of FIFO_IE_LEVTH
 * |[7:4]   |POINTER   |ADC FIFO Pointer (Read Only)
 * |        |          |The FULL bit and FIFOPOINTER[3:0] indicates the field that the valid data count within the SDADC FIFO buffer.
 * |        |          |The Maximum value shown in FIFOPOINTER is 15
 * |        |          |When the using level of SDADC FIFO Buffer equal to 16, The FULL bit is set to 1.
 * |[31]    |BISTEN    |BIST Enable
 * |        |          |1 = Enable ADC FIFO BIST testing ADC FIFO can be testing by Cortex-M0
 * |        |          |0 = Disable ADC FIFO BIST testing
 * |        |          |Internal use
 * @var SDADC_T::PDMACTL
 * Offset: 0x14  SD ADC PDMA Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDMAEN    |Enable SDADC PDMA Receive Channel
 * |        |          |1 = Enable SDADC PDMA.
 * |        |          |0 = Disable SDADC PDMA.
 * @var SDADC_T::CMPR0
 * Offset: 0x18  SD ADC Comparator 0 Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADCMPEN   |Compare Enable
 * |        |          |0 = Disable compare.
 * |        |          |1 = Enable compare.
 * |        |          |Set this bit to 1 to enable compare CMPDAT with FIFO data output.
 * |[1]     |CMPIE     |Compare Interrupt Enable
 * |        |          |1 = Enable compare function interrupt.
 * |        |          |0 = Disable compare function interrupt.
 * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMATCNT, CMPF bit will be asserted, if CMPIE is set to 1, a compare interrupt request is generated.
 * |[2]     |CMPCOND   |Compare Condition
 * |        |          |1= Set the compare condition that result is greater or equal to CMPD
 * |        |          |0= Set the compare condition that result is less than CMPD
 * |        |          |Note: When the internal counter reaches the value (CMPMATCNT +1), the CMPF bit will be set.
 * |[3]     |CMPF      |Compare Flag
 * |        |          |When the conversion result meets condition in ADCMPR0 this bit is set to 1
 * |        |          |It is cleared by writing 1 to self.
 * |[7:4]   |CMPMATCNT |Compare Match Count
 * |        |          |When the A/D FIFO result matches the compare condition defined by CMPCOND, the internal match counter will increase by 1
 * |        |          |When the internal counter reaches the value to (CMPMATCNT +1), the CMPF bit will be set.
 * |[30:8]  |CMPD      |Comparison Data
 * |        |          |23 bit value to compare to FIFO output word.
 * |[31]    |CMPOEN    |Compare Match output FIFO zero
 * |        |          |1 = compare match then FIFO out zero
 * |        |          |0 = FIFO data keep original one.
 * @var SDADC_T::CMPR1
 * Offset: 0x1C  SD ADC Comparator 1 Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADCMPEN   |Compare Enable
 * |        |          |0 = Disable compare.
 * |        |          |1 = Enable compare.
 * |        |          |Set this bit to 1 to enable compare CMPDAT with FIFO data output.
 * |[1]     |CMPIE     |Compare Interrupt Enable
 * |        |          |1 = Enable compare function interrupt.
 * |        |          |0 = Disable compare function interrupt.
 * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMATCNT, CMPF bit will be asserted, if CMPIE is set to 1, a compare interrupt request is generated.
 * |[2]     |CMPCOND   |Compare Condition
 * |        |          |1= Set the compare condition that result is greater or equal to CMPD
 * |        |          |0= Set the compare condition that result is less than CMPD
 * |        |          |Note: When the internal counter reaches the value (CMPMATCNT +1), the CMPF bit will be set.
 * |[3]     |CMPF      |Compare Flag
 * |        |          |When the conversion result meets condition in ADCMPR0 this bit is set to 1
 * |        |          |It is cleared by writing 1 to self.
 * |[7:4]   |CMPMATCNT |Compare Match Count
 * |        |          |When the A/D FIFO result matches the compare condition defined by CMPCOND, the internal match counter will increase by 1
 * |        |          |When the internal counter reaches the value to (CMPMATCNT +1), the CMPF bit will be set.
 * |[30:8]  |CMPD      |Comparison Data
 * |        |          |23 bit value to compare to FIFO output word.
 * |[31]    |CMPOEN    |Compare Match output FIFO zero
 * |        |          |1 = compare match then FIFO out zero
 * |        |          |0 = FIFO data keep original one.
 * @var SDADC_T::ANA0
 * Offset: 0x20  Sigma Delta Analog Block Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PD        |SDADC Power Down
 * |        |          |0 = SDADC power on
 * |        |          |1 = SDADC power off
 * |[2:1]   |BIAS      |SDADC Bias Current Selection
 * |        |          |00 = 1
 * |        |          |01 = 0.75
 * |        |          |10 = 0.5
 * |        |          |11 = 1.25
 * |[3]     |VREF      |SDADC Chopper in Reference Buffer
 * |        |          |0 = chopper off
 * |        |          |1 = chopper on
 * |[4]     |PGA_PU    |Power up PGA
 * |        |          |0u2014disable
 * |        |          |1u2014enable
 * |[5]     |PGA_MUTE  |Mute control signal
 * |        |          |0u2014disable
 * |        |          |1u2014enable
 * |[8:6]   |PGA_MODE  |PGA mode selection
 * |        |          |PGA_MODE[0] = Disable anti-aliasing filter adjust
 * |        |          |PGA_MODE [1] = Short the inputs
 * |        |          |PGA_MODE[2] = Noise reduction enable.
 * |        |          |1 = Enable
 * |        |          |0 = Disable
 * |[11:9]  |PGA_IBCTR |Trim PGA Current
 * |        |          |0=default
 * |[12]    |PGA_IBLOOP|Trim PGA current
 * |        |          |1=default
 * |[13]    |PGA_GAIN  |PGA Gain (default=0)
 * |        |          |PGA_GAIN
 * |        |          |PGA_HZMODE=0
 * |        |          |PGA_HZMODE=1
 * |        |          |0
 * |        |          |0dB
 * |        |          |6dB
 * |        |          |1
 * |        |          |6dB
 * |        |          |12dB
 * |[14]    |PGA_DISCH |Charge inputs selected by PGA_ACDC[1:0] to VREF
 * |        |          |1 = Enable
 * |        |          |0 = Disable
 * |[15]    |PGA_CMLCK |Common mode Threshold lock adjust enable
 * |        |          |0 = Enable
 * |        |          |1 = Disable
 * |[17:16] |PGA_CMLCKADJ|Common mode Threshold lock adjust. Action takes effect when PGA_CMLCK=0
 * |        |          |00---0.98 (default)
 * |        |          |01---0.96
 * |        |          |10---1.01
 * |        |          |11---1.04
 * |        |          |default=00
 * |[18]    |PGA_CLASSA|Enable Class A mode of operation
 * |        |          |0 = Class AB
 * |        |          |1 = Class A (default)
 * |[19]    |PGA_TRIMOBC|Trim current in output driver
 * |        |          |0=disable
 * |        |          |1=enable (default)
 * |[20]    |PGA_HZMODE|Select input impedance
 * |        |          |0 = 12k Ohm input impedance
 * |        |          |1 = 500k Ohm input impedance (default)
 * |[22:21] |PGA_ADCDC |Action takes effect when PGA_DISCH=1
 * |        |          |Bit[21]->ACDC_CTRL[0] charges INP to VREF
 * |        |          |Bit[22]->ACDC_CTRL[1] charges INN to VREF
 * |        |          |00=Default
 * |[24:23] |CHOPF     |SDADC Chopper Frequency in fixed chop mode
 * |        |          |00 = Fs/2 (default)
 * |        |          |01 = Fs/4
 * |        |          |10 = Fs/8
 * |        |          |11 = Fs/16
 * |[25]    |CHOPCLKPH |SDADC Chopper Clock phase selection
 * |        |          |1 = chopper transition after rising edge of ADC_CLK
 * |        |          |0 = chopper transition after falling edge of ADC_CLK (default)
 * |[26]    |CHOPFIX   |SDADC Chopper Fixed Frequency
 * |        |          |1 = choose fixed frequency
 * |        |          |0 = dither chopper frequency (default)
 * |[27]    |CHOPORD   |SDADC Chopper Order
 * |        |          |1 = 2nd order dithering of chopper frequency
 * |        |          |0 = 1st order dithering of chopper frequency (default)
 * |[28]    |CHOPPH    |SDADC chopper phase
 * |        |          |When chopper is off:
 * |        |          |0 = chopper switches in default state
 * |        |          |1 = invert chopper switches
 * |[29]    |CHOPEN    |SDADC chopper enable
 * |        |          |1 = enable
 * |        |          |0 = disable (default)
 */
    __I  uint32_t DAT;                   /*!< [0x0000] SD ADC FIFO Data Read Register                                   */
    __IO uint32_t EN;                    /*!< [0x0004] SD ADC Enable  Register                                          */
    __IO uint32_t CLKDIV;                /*!< [0x0008] SD ADC Clock Divider Register                                    */
    __IO uint32_t CTL;                   /*!< [0x000c] SD ADC Control Register                                          */
    __IO uint32_t FIFOSTS;               /*!< [0x0010] SD ADC FIFO Status Register                                      */
    __IO uint32_t PDMACTL;               /*!< [0x0014] SD ADC PDMA Control Register                                     */
    __IO uint32_t CMPR0;                 /*!< [0x0018] SD ADC Comparator 0 Control Register                             */
    __IO uint32_t CMPR1;                 /*!< [0x001c] SD ADC Comparator 1 Control Register                             */
    __IO uint32_t ANA0;                /*!< [0x0020] Sigma Delta Analog Block Control Register 0                       */
	__I  uint32_t RESERVE0[1];
	__IO uint32_t ANA1;                /*!< [0x0028] Sigma Delta Analog Block Control Register 1                       */
	__IO uint32_t ANA2;                /*!< [0x002C] Sigma Delta Analog Block Control Register 2                      */

} SDADC_T;

/**
    @addtogroup SDADC_CONST SDADC Bit Field Definition
    Constant Definitions for SDADC Controller
@{ */

#define SDADC_DAT_RESULT_Pos             (0)                                               /*!< SDADC_T::DAT: RESULT Position          */
#define SDADC_DAT_RESULT_Msk             (0xfffffffful << SDADC_DAT_RESULT_Pos)            /*!< SDADC_T::DAT: RESULT Mask              */

#define SDADC_EN_SDADCEN_Pos             (0)                                               /*!< SDADC_T::EN: SDADCEN Position          */
#define SDADC_EN_SDADCEN_Msk             (0x1ul << SDADC_EN_SDADCEN_Pos)                   /*!< SDADC_T::EN: SDADCEN Mask              */

#define SDADC_EN_DINEDGE_Pos             (1)                                               /*!< SDADC_T::EN: DINEDGE Position          */
#define SDADC_EN_DINEDGE_Msk             (0x1ul << SDADC_EN_DINEDGE_Pos)                   /*!< SDADC_T::EN: DINEDGE Mask              */

#define SDADC_EN_DINBYPS_Pos             (2)                                               /*!< SDADC_T::EN: DINBYPS Position          */
#define SDADC_EN_DINBYPS_Msk             (0x1ul << SDADC_EN_DINBYPS_Pos)                   /*!< SDADC_T::EN: DINBYPS Mask              */

#define SDADC_CLKDIV_CLKDIV_Pos          (0)                                               /*!< SDADC_T::CLKDIV: CLKDIV Position       */
#define SDADC_CLKDIV_CLKDIV_Msk          (0xfful << SDADC_CLKDIV_CLKDIV_Pos)               /*!< SDADC_T::CLKDIV: CLKDIV Mask           */

#define SDADC_CTL_DSRATE_Pos             (0)                                               /*!< SDADC_T::CTL: DSRATE Position          */
#define SDADC_CTL_DSRATE_Msk             (0x3ul << SDADC_CTL_DSRATE_Pos)                   /*!< SDADC_T::CTL: DSRATE Mask              */

#define SDADC_CTL_FIFOBITS_Pos           (2)                                               /*!< SDADC_T::CTL: FIFOBITS Position        */
#define SDADC_CTL_FIFOBITS_Msk           (0x3ul << SDADC_CTL_FIFOBITS_Pos)                 /*!< SDADC_T::CTL: FIFOBITS Mask            */

#define SDADC_CTL_FIFOTH_Pos             (4)                                               /*!< SDADC_T::CTL: FIFOTH Position          */
#define SDADC_CTL_FIFOTH_Msk             (0xful << SDADC_CTL_FIFOTH_Pos)                   /*!< SDADC_T::CTL: FIFOTH Mask              */
         
#define SDADC_CTL_FIFOTHIE_Pos           (8)                                               /*!< SDADC_T::CTL: FIFOTHIE Position        */
#define SDADC_CTL_FIFOTHIE_Msk           (0x1ul << SDADC_CTL_FIFOTHIE_Pos)                 /*!< SDADC_T::CTL: FIFOTHIE Mask            */

#define SDADC_CTL_SPDS_Pos               (12)   
#define SDADC_CTL_SPDS_Msk               (0x1ul << SDADC_CTL_SPDS_Pos)                     /*!< SDADC_T::CTL: SPDS Mask            */

#define SDADC_FIFOSTS_FULL_Pos           (0)                                               /*!< SDADC_T::FIFOSTS: FULL Position        */
#define SDADC_FIFOSTS_FULL_Msk           (0x1ul << SDADC_FIFOSTS_FULL_Pos)                 /*!< SDADC_T::FIFOSTS: FULL Mask            */

#define SDADC_FIFOSTS_EMPTY_Pos          (1)                                               /*!< SDADC_T::FIFOSTS: EMPTY Position       */
#define SDADC_FIFOSTS_EMPTY_Msk          (0x1ul << SDADC_FIFOSTS_EMPTY_Pos)                /*!< SDADC_T::FIFOSTS: EMPTY Mask           */

#define SDADC_FIFOSTS_THIF_Pos           (2)                                               /*!< SDADC_T::FIFOSTS: THIF Position        */
#define SDADC_FIFOSTS_THIF_Msk           (0x1ul << SDADC_FIFOSTS_THIF_Pos)                 /*!< SDADC_T::FIFOSTS: THIF Mask            */

#define SDADC_FIFOSTS_POINTER_Pos        (4)                                               /*!< SDADC_T::FIFOSTS: POINTER Position     */
#define SDADC_FIFOSTS_POINTER_Msk        (0xful << SDADC_FIFOSTS_POINTER_Pos)              /*!< SDADC_T::FIFOSTS: POINTER Mask         */

#define SDADC_FIFOSTS_BISTEN_Pos         (31)                                              /*!< SDADC_T::FIFOSTS: BISTEN Position      */
#define SDADC_FIFOSTS_BISTEN_Msk         (0x1ul << SDADC_FIFOSTS_BISTEN_Pos)               /*!< SDADC_T::FIFOSTS: BISTEN Mask          */

#define SDADC_PDMACTL_PDMAEN_Pos         (0)                                               /*!< SDADC_T::PDMACTL: PDMAEN Position      */
#define SDADC_PDMACTL_PDMAEN_Msk         (0x1ul << SDADC_PDMACTL_PDMAEN_Pos)               /*!< SDADC_T::PDMACTL: PDMAEN Mask          */

#define SDADC_CMPR0_ADCMPEN_Pos          (0)                                               /*!< SDADC_T::CMPR0: ADCMPEN Position       */
#define SDADC_CMPR0_ADCMPEN_Msk          (0x1ul << SDADC_CMPR0_ADCMPEN_Pos)                /*!< SDADC_T::CMPR0: ADCMPEN Mask           */

#define SDADC_CMPR0_CMPIE_Pos            (1)                                               /*!< SDADC_T::CMPR0: CMPIE Position         */
#define SDADC_CMPR0_CMPIE_Msk            (0x1ul << SDADC_CMPR0_CMPIE_Pos)                  /*!< SDADC_T::CMPR0: CMPIE Mask             */

#define SDADC_CMPR0_CMPCOND_Pos          (2)                                               /*!< SDADC_T::CMPR0: CMPCOND Position       */
#define SDADC_CMPR0_CMPCOND_Msk          (0x1ul << SDADC_CMPR0_CMPCOND_Pos)                /*!< SDADC_T::CMPR0: CMPCOND Mask           */

#define SDADC_CMPR0_CMPF_Pos             (3)                                               /*!< SDADC_T::CMPR0: CMPF Position          */
#define SDADC_CMPR0_CMPF_Msk             (0x1ul << SDADC_CMPR0_CMPF_Pos)                   /*!< SDADC_T::CMPR0: CMPF Mask              */

#define SDADC_CMPR0_CMPMATCNT_Pos        (4)                                               /*!< SDADC_T::CMPR0: CMPMATCNT Position     */
#define SDADC_CMPR0_CMPMATCNT_Msk        (0xful << SDADC_CMPR0_CMPMATCNT_Pos)              /*!< SDADC_T::CMPR0: CMPMATCNT Mask         */

#define SDADC_CMPR0_CMPD_Pos             (8)                                               /*!< SDADC_T::CMPR0: CMPD Position          */
#define SDADC_CMPR0_CMPD_Msk             (0x7ffffful << SDADC_CMPR0_CMPD_Pos)              /*!< SDADC_T::CMPR0: CMPD Mask              */

#define SDADC_CMPR0_CMPOEN_Pos           (31)                                              /*!< SDADC_T::CMPR0: CMPOEN Position        */
#define SDADC_CMPR0_CMPOEN_Msk           (0x1ul << SDADC_CMPR0_CMPOEN_Pos)                 /*!< SDADC_T::CMPR0: CMPOEN Mask            */

#define SDADC_CMPR1_ADCMPEN_Pos          (0)                                               /*!< SDADC_T::CMPR1: ADCMPEN Position       */
#define SDADC_CMPR1_ADCMPEN_Msk          (0x1ul << SDADC_CMPR1_ADCMPEN_Pos)                /*!< SDADC_T::CMPR1: ADCMPEN Mask           */

#define SDADC_CMPR1_CMPIE_Pos            (1)                                               /*!< SDADC_T::CMPR1: CMPIE Position         */
#define SDADC_CMPR1_CMPIE_Msk            (0x1ul << SDADC_CMPR1_CMPIE_Pos)                  /*!< SDADC_T::CMPR1: CMPIE Mask             */

#define SDADC_CMPR1_CMPCOND_Pos          (2)                                               /*!< SDADC_T::CMPR1: CMPCOND Position       */
#define SDADC_CMPR1_CMPCOND_Msk          (0x1ul << SDADC_CMPR1_CMPCOND_Pos)                /*!< SDADC_T::CMPR1: CMPCOND Mask           */

#define SDADC_CMPR1_CMPF_Pos             (3)                                               /*!< SDADC_T::CMPR1: CMPF Position          */
#define SDADC_CMPR1_CMPF_Msk             (0x1ul << SDADC_CMPR1_CMPF_Pos)                   /*!< SDADC_T::CMPR1: CMPF Mask              */

#define SDADC_CMPR1_CMPMATCNT_Pos        (4)                                               /*!< SDADC_T::CMPR1: CMPMATCNT Position     */
#define SDADC_CMPR1_CMPMATCNT_Msk        (0xful << SDADC_CMPR1_CMPMATCNT_Pos)              /*!< SDADC_T::CMPR1: CMPMATCNT Mask         */

#define SDADC_CMPR1_CMPD_Pos             (8)                                               /*!< SDADC_T::CMPR1: CMPD Position          */
#define SDADC_CMPR1_CMPD_Msk             (0x7ffffful << SDADC_CMPR1_CMPD_Pos)              /*!< SDADC_T::CMPR1: CMPD Mask              */

#define SDADC_CMPR1_CMPOEN_Pos           (31)                                              /*!< SDADC_T::CMPR1: CMPOEN Position        */
#define SDADC_CMPR1_CMPOEN_Msk           (0x1ul << SDADC_CMPR1_CMPOEN_Pos)                 /*!< SDADC_T::CMPR1: CMPOEN Mask            */

#define SDADC_ANA0_PD_Pos                (0)                                               /*!< SDADC_T::ANA0: PD Position           */
#define SDADC_ANA0_PD_Msk                (0x1ul << SDADC_ANA0_PD_Pos)                      /*!< SDADC_T::ANA0: PD Mask               */

#define SDADC_ANA0_BIAS_Pos              (1)                                               /*!< SDADC_T::ANA0: BIAS Position         */
#define SDADC_ANA0_BIAS_Msk              (0x3ul << SDADC_ANA0_BIAS_Pos)                    /*!< SDADC_T::ANA0: BIAS Mask             */

#define SDADC_ANA0_VREF_Pos              (3)                                               /*!< SDADC_T::ANA0: VREF Position         */
#define SDADC_ANA0_VREF_Msk              (0x1ul << SDADC_ANA0_VREF_Pos)                    /*!< SDADC_T::ANA0: VREF Mask             */

#define SDADC_ANA0_PGA_PU_Pos            (4)                                               /*!< SDADC_T::ANA0: PGA_PU Position       */
#define SDADC_ANA0_PGA_PU_Msk            (0x1ul << SDADC_ANA0_PGA_PU_Pos)                  /*!< SDADC_T::ANA0: PGA_PU Mask           */

#define SDADC_ANA0_PGA_MUTE_Pos          (5)                                               /*!< SDADC_T::ANA0: PGA_MUTE Position     */
#define SDADC_ANA0_PGA_MUTE_Msk          (0x1ul << SDADC_ANA0_PGA_MUTE_Pos)                /*!< SDADC_T::ANA0: PGA_MUTE Mask         */

#define SDADC_ANA0_PGA_MODE_Pos          (6)                                               /*!< SDADC_T::ANA0: PGA_MODE Position     */
#define SDADC_ANA0_PGA_MODE_Msk          (0x7ul << SDADC_ANA0_PGA_MODE_Pos)                /*!< SDADC_T::ANA0: PGA_MODE Mask         */

#define SDADC_ANA0_PGA_IBCTR_Pos         (9)                                               /*!< SDADC_T::ANA0: PGA_IBCTR Position    */
#define SDADC_ANA0_PGA_IBCTR_Msk         (0x7ul << SDADC_ANA0_PGA_IBCTR_Pos)               /*!< SDADC_T::ANA0: PGA_IBCTR Mask        */

#define SDADC_ANA0_PGA_IBLOOP_Pos        (12)                                              /*!< SDADC_T::ANA0: PGA_IBLOOP Position   */
#define SDADC_ANA0_PGA_IBLOOP_Msk        (0x1ul << SDADC_ANA0_PGA_IBLOOP_Pos)              /*!< SDADC_T::ANA0: PGA_IBLOOP Mask       */

#define SDADC_ANA0_PGA_DISCH_Pos         (14)                                              /*!< SDADC_T::ANA0: PGA_DISCH Position    */
#define SDADC_ANA0_PGA_DISCH_Msk         (0x1ul << SDADC_ANA0_PGA_DISCH_Pos)               /*!< SDADC_T::ANA0: PGA_DISCH Mask        */

#define SDADC_ANA0_PGA_CMLCK_Pos         (15)                                              /*!< SDADC_T::ANA0: PGA_CMLCK Position    */
#define SDADC_ANA0_PGA_CMLCK_Msk         (0x1ul << SDADC_ANA0_PGA_CMLCK_Pos)               /*!< SDADC_T::ANA0: PGA_CMLCK Mask        */

#define SDADC_ANA0_PGA_CMLCKADJ_Pos      (16)                                              /*!< SDADC_T::ANA0: PGA_CMLCKADJ Position */
#define SDADC_ANA0_PGA_CMLCKADJ_Msk      (0x3ul << SDADC_ANA0_PGA_CMLCKADJ_Pos)            /*!< SDADC_T::ANA0: PGA_CMLCKADJ Mask     */

#define SDADC_ANA0_PGA_CLASSA_Pos        (18)                                              /*!< SDADC_T::ANA0: PGA_CLASSA Position   */
#define SDADC_ANA0_PGA_CLASSA_Msk        (0x1ul << SDADC_ANA0_PGA_CLASSA_Pos)              /*!< SDADC_T::ANA0: PGA_CLASSA Mask       */

#define SDADC_ANA0_PGA_TRIMOBC_Pos       (19)                                              /*!< SDADC_T::ANA0: PGA_TRIMOBC Position  */
#define SDADC_ANA0_PGA_TRIMOBC_Msk       (0x1ul << SDADC_ANA0_PGA_TRIMOBC_Pos)             /*!< SDADC_T::ANA0: PGA_TRIMOBC Mask      */

#define SDADC_ANA0_PGA_HZMODE_Pos        (20)                                              /*!< SDADC_T::ANA0: PGA_HZMODE Position   */
#define SDADC_ANA0_PGA_HZMODE_Msk        (0x1ul << SDADC_ANA0_PGA_HZMODE_Pos)              /*!< SDADC_T::ANA0: PGA_HZMODE Mask       */

#define SDADC_ANA0_PGA_ADCDC_Pos         (21)                                              /*!< SDADC_T::ANA0: PGA_ADCDC Position    */
#define SDADC_ANA0_PGA_ADCDC_Msk         (0x3ul << SDADC_ANA0_PGA_ADCDC_Pos)               /*!< SDADC_T::ANA0: PGA_ADCDC Mask        */
 
#define SDADC_ANA0_CHOPF_Pos             (23)                                              /*!< SDADC_T::ANA0: CHOPF Position        */
#define SDADC_ANA0_CHOPF_Msk             (0x3ul << SDADC_ANA0_CHOPF_Pos)                   /*!< SDADC_T::ANA0: CHOPF Mask            */

#define SDADC_ANA0_CHOPCLKPH_Pos         (25)                                              /*!< SDADC_T::ANA0: CHOPCLKPH Position    */
#define SDADC_ANA0_CHOPCLKPH_Msk         (0x1ul << SDADC_ANA0_CHOPCLKPH_Pos)               /*!< SDADC_T::ANA0: CHOPCLKPH Mask        */

#define SDADC_ANA0_CHOPFIX_Pos           (26)                                              /*!< SDADC_T::ANA0: CHOPFIX Position      */
#define SDADC_ANA0_CHOPFIX_Msk           (0x1ul << SDADC_ANA0_CHOPFIX_Pos)                 /*!< SDADC_T::ANA0: CHOPFIX Mask          */

#define SDADC_ANA0_CHOPORD_Pos           (27)                                              /*!< SDADC_T::ANA0: CHOPORD Position      */
#define SDADC_ANA0_CHOPORD_Msk           (0x1ul << SDADC_ANA0_CHOPORD_Pos)                 /*!< SDADC_T::ANA0: CHOPORD Mask          */

#define SDADC_ANA0_CHOPPH_Pos            (28)                                              /*!< SDADC_T::ANA0: CHOPPH Position       */
#define SDADC_ANA0_CHOPPH_Msk            (0x1ul << SDADC_ANA0_CHOPPH_Pos)                  /*!< SDADC_T::ANA0: CHOPPH Mask           */

#define SDADC_ANA0_CHOPEN_Pos            (29)                                              /*!< SDADC_T::ANA0: CHOPEN Position       */
#define SDADC_ANA0_CHOPEN_Msk            (0x1ul << SDADC_ANA0_CHOPEN_Pos)                  /*!< SDADC_T::ANA0: CHOPEN Mask           */

#define SDADC_ANA1_CMLCKADJ_Pos          (0)                                               /*!< SDADC_T::ANA1: CMLCKADJ Position       */
#define SDADC_ANA1_CMLCKADJ_Msk          (0x3ul << SDADC_ANA1_CMLCKADJ_Pos)                /*!< SDADC_T::ANA1: CMLCKADJ Mask           */

#define SDADC_ANA1_CMLCKEN_Pos           (2)                                               /*!< SDADC_T::ANA1: CMLCKEN Position        */
#define SDADC_ANA1_CMLCKEN_Msk           (0x1ul << SDADC_ANA1_CMLCKEN_Pos)                 /*!< SDADC_T::ANA1: CMLCKEN Mask            */

#define SDADC_ANA1_CLASSAEN_Pos          (3)                                               /*!< SDADC_T::ANA1: CLASSAEN Position       */
#define SDADC_ANA1_CLASSAEN_Msk          (0x1ul << SDADC_ANA1_CLASSAEN_Pos)                /*!< SDADC_T::ANA1: CLASSAEN Mask           */

#define SDADC_ANA1_DISCHRG_Pos           (4)                                               /*!< SDADC_T::ANA1: DISCHRG Position        */
#define SDADC_ANA1_DISCHRG_Msk           (0x1ul << SDADC_ANA1_DISCHRG_Pos)                 /*!< SDADC_T::ANA1: DISCHRG Mask            */

#define SDADC_ANA1_IBCTRCODE_Pos         (5)                                               /*!< SDADC_T::ANA1: IBCTRCODE Position      */
#define SDADC_ANA1_IBCTRCODE_Msk         (0x7ul << SDADC_ANA1_IBCTRCODE_Pos)               /*!< SDADC_T::ANA1: IBCTRCODE Mask          */

#define SDADC_ANA1_IBLOOPCTR_Pos         (8)                                               /*!< SDADC_T::ANA1: IBLOOPCTR Position      */
#define SDADC_ANA1_IBLOOPCTR_Msk         (0x1ul << SDADC_ANA1_IBLOOPCTR_Pos)               /*!< SDADC_T::ANA1: IBLOOPCTR Mask          */

#define SDADC_ANA1_BSTMODE_Pos           (9)                                               /*!< SDADC_T::ANA1: BSTMODE Position        */
#define SDADC_ANA1_BSTMODE_Msk           (0xful << SDADC_ANA1_BSTMODE_Pos)                 /*!< SDADC_T::ANA1: BSTMODE Mask            */

#define SDADC_ANA1_BSTMUTE_Pos           (13)                                              /*!< SDADC_T::ANA1: BSTMUTE Position        */
#define SDADC_ANA1_BSTMUTE_Msk           (0x1ul << SDADC_ANA1_BSTMUTE_Pos)                 /*!< SDADC_T::ANA1: BSTMUTE Mask            */

#define SDADC_ANA1_BSTPUP_Pos            (14)                                              /*!< SDADC_T::ANA1: BSTPUP Position         */
#define SDADC_ANA1_BSTPUP_Msk            (0x1ul << SDADC_ANA1_BSTPUP_Pos)                  /*!< SDADC_T::ANA1: BSTPUP Mask             */

#define SDADC_ANA1_BSTTRIMOBC_Pos        (15)                                              /*!< SDADC_T::ANA1: BSTTRIMOBC Position     */
#define SDADC_ANA1_BSTTRIMOBC_Msk        (0x1ul << SDADC_ANA1_BSTTRIMOBC_Pos)              /*!< SDADC_T::ANA1: BSTTRIMOBC Mask         */

#define SDADC_ANA1_ACDC_Pos              (16)                                              /*!< SDADC_T::ANA1: ACDC Position           */
#define SDADC_ANA1_ACDC_Msk              (0x3ul << SDADC_ANA1_ACDC_Pos)                    /*!< SDADC_T::ANA1: ACDC Mask               */

#define SDADC_ANA2_GAINSET_Pos           (0)                                               /*!< SDADC_T::ANA2: PGA_GAIN Position     */
#define SDADC_ANA2_GAINSET_Msk           (0x1Ful << SDADC_ANA2_GAINSET_Pos)                /*!< SDADC_T::ANA2: PGA_GAIN Mask         */

/**@}*/ /* SDADC_CONST */
/**@}*/ /* end of SDADC register group */


/*---------------------- Serial Peripheral Interface Controller -------------------------*/
/**
    @addtogroup SPI Serial Peripheral Interface Controller(SPI)
    Memory Mapped Structure for SPI Controller
@{ */
 
typedef struct
{


/**
 * @var SPI_T::CTL
 * Offset: 0x00  Control and Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SPIEN     |SPI Transfer Enable
 * |        |          |0 = Disable SPI Transfer.
 * |        |          |1 = Enable SPI Transfer.
 * |        |          |In Master mode, the transfer will start when there is data in the FIFO buffer after this is set to 1
 * |        |          |In Slave mode, the device is ready to receive data when this bit is set to 1.
 * |        |          |Note:
 * |        |          |All configuration should be set before writing 1 to this SPIEN bit
 * |        |          |(e.g.: TXNEG, RXNEG, DWIDTH, LSB, CLKP, and so on).
 * |[1]     |RXNEG     |Receive at Negative Edge
 * |        |          |0 = The received data input signal is latched at the rising edge of SCLK.
 * |        |          |1 = The received data input signal is latched at the falling edge of SCLK.
 * |[2]     |TXNEG     |Transmit at Negative Edge
 * |        |          |0 = The transmitted data output signal is changed at the rising edge of SCLK.
 * |        |          |1 = The transmitted data output signal is changed at the falling edge of SCLK.
 * |[3]     |CLKPOL    |Clock Polarity
 * |        |          |0 = SCLK idle low.
 * |        |          |1 = SCLK idle high.
 * |[7:4]   |SUSPITV   |Suspend Interval (Master Only)
 * |        |          |The four bits provide configurable suspend interval between two successive transmit/receive transactions in a transfer
 * |        |          |The definition of the suspend interval is the interval between the last clock edge of the preceding transaction word and the first clock edge of the following transaction word
 * |        |          |The default value is 0x3
 * |        |          |The period of the suspend interval is obtained according to the following equation
 * |        |          |SUSPITV is available for standard SPI transactions, it must be set to 0 for DUAL and QUAD mode transactions.
 * |        |          |(SUSPITV[3:0] + 0.5) * period of SPICLK clock cycle
 * |        |          |Example:
 * |        |          |SUSPITV = 0x0 ... 0.5 SPICLK clock cycle.
 * |        |          |SUSPITV = 0x1 ... 1.5 SPICLK clock cycle.
 * |        |          |....
 * |        |          |SUSPITV = 0xE ... 14.5 SPICLK clock cycle.
 * |        |          |SUSPITV = 0xF ... 15.5 SPICLK clock cycle.
 * |        |          |Note:
 * |        |          |For DUAL and QUAD transactions with SUSPITV must be set to 0.
 * |[12:8]  |DWIDTH    |DWIDTH u2013 Data Word Bit Length
 * |        |          |This field specifies how many bits are transmitted in one transmit/receive
 * |        |          |Up to 32 bits can be transmitted.
 * |        |          |DWIDTH = 0x01 ... 1 bit.
 * |        |          |DWIDTH = 0x02 ... 2 bits.
 * |        |          |......
 * |        |          |DWIDTH = 0x1f ... 31 bits.
 * |        |          |DWIDTH = 0x00 ... 32 bits.
 * |[13]    |LSB       |LSB First
 * |        |          |0 = The MSB is transmitted/received first (which bit in TX and RX FIFO depends on the DWIDTH field).
 * |        |          |1 = The LSB is sent first on the line (bit 0 of TX FIFO]), and the first bit received from the line will be put in the LSB position in the SPIn_RX FIFO (bit 0 SPIn_RX).
 * |        |          |Note:
 * |        |          |For DUAL and QUAD transactions with LSB must be set to 0.
 * |[16]    |TWOBIT    |Two Bits Transfer Mode
 * |        |          |0 = Disable two-bit transfer mode.
 * |        |          |1 = Enable two-bit transfer mode.
 * |        |          |When 2-bit mode is enabled, the first serial transmitted bit data is from the first FIFO buffer data, and the 2nd serial transmitted bit data is from the second FIFO buffer data
 * |        |          |As the same as transmitted function, the first received bit data is stored into the first FIFO buffer and the 2nd received bit data is stored into the second FIFO buffer at the same time.
 * |[17]    |UNITIEN   |Unit Transfer Interrupt Enable
 * |        |          |0 = Disable SPI Unit Transfer Interrupt.
 * |        |          |1 = Enable SPI Unit Transfer Interrupt to CPU.
 * |[18]    |SLAVE     |Master Slave Mode Control
 * |        |          |0 = Master mode.
 * |        |          |1 = Slave mode.
 * |[19]    |REORDER   |Byte Reorder Function Enable
 * |        |          |0 = Byte reorder function Disabled.
 * |        |          |1 = Byte reorder function Enabled
 * |        |          |A byte suspend interval will be inserted between each byte
 * |        |          |The period of the byte suspend interval depends on the setting of SUSPITV.
 * |        |          |Note:
 * |        |          |Byte reorder function is only available if DWIDTH is defined as 16, 24, and 32 bits.
 * |        |          |REORDER is only available for Receive mode in DUAL and QUAD transactions.
 * |        |          |For DUAL and QUAD transactions with REORDER, SUSPITV must be set to 0.
 * |[20]    |QDIODIR   |Quad or Dual I/O Mode Direction Control
 * |        |          |0 = Quad or Dual Input mode.
 * |        |          |1 = Quad or Dual Output mode.
 * |[21]    |DUALIOEN  |Dual I/O Mode Enable
 * |        |          |0 = Dual I/O mode Disabled.
 * |        |          |1 = Dual I/O mode Enabled.
 * |[22]    |QUADIOEN  |Quad I/O Mode Enable
 * |        |          |0 = Quad I/O mode Disabled.
 * |        |          |1 = Quad I/O mode Enabled.
 * |[23]    |RXTCNTEN  |DMA Receive Transaction Count Enable
 * |        |          |0 = Disable function.
 * |        |          |1 = Enable transaction counter for DMA receive only mode
 * |        |          |SPI will perform the number of transfers specified in the SPI_RXTSNCNT register, allowing the SPI interface to read ahead of DMA controller.
 * |[24]    |RXMODEEN  |FIFO Receive Mode Enable
 * |        |          |0 = Disable function.
 * |        |          |1 = Enable FIFO receive mode
 * |        |          |In this mode SPI transactions will be continuously performed while RXFULL is not active
 * |        |          |To stop transactions, set RXMODEEN to 0.
 * @var SPI_T::CLKDIV
 * Offset: 0x04  Clock Divider Register (Master Only)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DIVIDER   |Clock Divider Register
 * |        |          |The value in this field is the frequency divider for generating the SPI engine clock,Fspi_sclk, and the SPI serial clock of SPI master
 * |        |          |The frequency is obtained according to the following equation.
 * |        |          |Fspi_sclk = Fspi_clockSRC / (DIVIDER+1).
 * |        |          |where
 * |        |          |Fspi_clockSRC is the SPI engine clock source, which is defined in the clock control, CLKSEL1 register.
 * @var SPI_T::SSCTL
 * Offset: 0x08  Slave Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |SS        |Slave Select Control Bits (Master Only)
 * |        |          |If AUTOSS bit is cleared, writing 1 to any bit of this field sets the proper SPISSx0/1 line to an active state and writing 0 sets the line back to inactive state.
 * |        |          |If the AUTOSS bit is set, writing 0 to any bit location of this field will keep the corresponding SPI_SS0/1 line at inactive state; writing 1 to any bit location of this field will select appropriate SPI_SS0/1 line to be automatically driven to active state for the duration of the transmit/receive, and will be driven to inactive state for the rest of the time
 * |        |          |The active state of SPI_SS0/1 is specified in SSACTPOL.
 * |        |          |Note: SPI_SS0 is defined as the slave select input in Slave mode.
 * |[2]     |SSACTPOL  |Slave Select Active Level
 * |        |          |This bit defines the active status of slave select signal (SPI_SS0/1).
 * |        |          |0 = The slave select signal SPI_SS0/1 is active on low-level/falling-edge.
 * |        |          |1 = The slave select signal SPI_SS0/1 is active on high-level/rising-edge.
 * |[3]     |AUTOSS    |Automatic Slave Select Function Enable (Master Only)
 * |        |          |0 = If this bit is cleared, slave select signals will be asserted/de-asserted by setting/clearing the corresponding bits of SPI_SSCTL[1:0].
 * |        |          |1 = If this bit is set, SPI_SS0/1 signals will be generated automatically
 * |        |          |It means that device/slave select signal, which is set in SPI_SSCTL[1:0], will be asserted by the SPI controller when transmit/receive is started, and will be de-asserted after each transmit/receive is finished.
 * |[4]     |SLV3WIRE  |Slave 3-wire Mode Enable
 * |        |          |This is used to ignore the slave select signal in Slave mode
 * |        |          |The SPI controller can work with 3-wire interface consisting of SPI_CLK, SPI_MISO, and SPI_MOSI.
 * |        |          |0 = 4-wire bi-directional interface.
 * |        |          |1 = 3-wire bi-directional interface.
 * |[5]     |SLVTOIEN  |Slave Mode Time-out Interrupt Enable
 * |        |          |0 = Slave mode time-out interrupt Disabled.
 * |        |          |1 = Slave mode time-out interrupt Enabled.
 * |[6]     |SLVTORST  |Slave Mode Time-out FIFO Clear
 * |        |          |0 = Function disabled.
 * |        |          |1 = Both the FIFO clear function, TXRST and RXRST, are activated automatically when there is a slave mode time-out event.
 * |[8]     |SLVBCEIEN |Slave Mode Error 0 Interrupt Enable
 * |        |          |0 = Slave mode error 0 interrupt Disable.
 * |        |          |1 = Slave mode error 0 interrupt Enable.
 * |[9]     |SLVUDRIEN |Slave Mode Error 1 Interrupt Enable
 * |        |          |0 = Slave mode error 1 interrupt Disable.
 * |        |          |1 = Slave mode error 1 interrupt Enable.
 * |[12]    |SSACTIEN  |Slave Select Active Interrupt Enable
 * |        |          |0 = Slave select active interrupt Disable.
 * |        |          |1 = Slave select active interrupt Enable.
 * |[13]    |SSINAIEN  |Slave Select Inactive Interrupt Enable
 * |        |          |0 = Slave select inactive interrupt Disable.
 * |        |          |1 = Slave select inactive interrupt Enable.
 * |[31:16] |SLVTOCNT  |Slave Mode Time-out Period
 * |        |          |In Slave mode, these bits indicate the time out period when there is serial clock input during slave select active
 * |        |          |The clock source of the time out counter is Slave engine clock
 * |        |          |If the value is 0, it indicates the slave mode time-out function is disabled.
 * @var SPI_T::PDMACTL
 * Offset: 0x0C  SPI PDMA Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TXPDMAEN  |Transmit DMA Enable
 * |        |          |Setting this bit to 1 will start the transmit PDMA process
 * |        |          |SPI controller will issue request to PDMA controller automatically
 * |        |          |Hardware will clear this bit to 0 automatically after PDMA transfer done.
 * |[1]     |RXPDMAEN  |Receive PDMA Enable
 * |        |          |Setting this bit to 1 will start the receive PDMA process
 * |        |          |The SPI controller will issue request to PDMA controller automatically when the SPI receive buffer is not empty
 * |        |          |This bit will be cleared to 0 by hardware automatically after PDMA transfer is done.
 * |[2]     |PDMARST   |PDMA Reset
 * |        |          |0 = No effect.
 * |        |          |1 = Reset the PDMA control logic of the SPI controller. This bit will be cleared to 0 automatically.
 * @var SPI_T::FIFOCTL
 * Offset: 0x10  FIFO Control/Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RXRST     |Clear Receive FIFO Buffer
 * |        |          |0 = No effect.
 * |        |          |1 = Clear receive FIFO buffer
 * |        |          |The RXFULL bit will be cleared to 0 and the RXEMPTY bit will be set to 1
 * |        |          |This bit will be cleared to 0 by hardware about 3 system clocks + 3 SPI engine clock after it is set to 1.
 * |        |          |Note: If there is slave receive time out event, the RXRST will be set 1 when the SPI_SSCTL.SLVTORST, is enabled.
 * |[1]     |TXRST     |Clear Transmit FIFO Buffer
 * |        |          |0 = No effect.
 * |        |          |1 = Clear transmit FIFO buffer
 * |        |          |The TXFULL bit will be cleared to 0 and the TXEMPTY bit will be set to 1
 * |        |          |This bit will be cleared to 0 by hardware about 3 system clocks + 3 SPI engine clock after it is set to 1.
 * |        |          |Note: If there is slave receive time out event, the TXRST will be set 1 when the SPI_SSCTL.SLVTORST, is enabled.
 * |[2]     |RXTHIEN   |Receive FIFO Threshold Interrupt Enable
 * |        |          |0 = RX FIFO threshold interrupt Disabled.
 * |        |          |1 = RX FIFO threshold interrupt Enabled.
 * |[3]     |TXTHIEN   |Transmit FIFO Threshold Interrupt Enable
 * |        |          |0 = TX FIFO threshold interrupt Disabled.
 * |        |          |1 = TX FIFO threshold interrupt Enabled.
 * |[4]     |RXTOIEN   |Slave Receive Time-out Interrupt Enable
 * |        |          |0 = Receive time-out interrupt Disabled.
 * |        |          |1 = Receive time-out interrupt Enabled.
 * |[5]     |RXOVIEN   |Receive FIFO Overrun Interrupt Enable
 * |        |          |0 = Receive FIFO overrun interrupt Disabled.
 * |        |          |1 = Receive FIFO overrun interrupt Enabled.
 * |[6]     |TXUDFPOL  |Transmit Under-run Data Out
 * |        |          |0 = The SPI data out is 0 if there is transmit under-run event in Slave mode.
 * |        |          |1 = The SPI data out is 1 if there is transmit under-run event in Slave mode.
 * |        |          |Note: The under run event is active after the serial clock input and the hardware synchronous, so that the first 1~3 bit (depending on the relation between system clock and the engine clock) data out will be the last transaction data.
 * |        |          |Note: If the frequency of system clock approach the engine clock, they may be a 3-bit time to report the transmit under-run data out.
 * |[7]     |TXUDFIEN  |Slave Transmit Under Run Interrupt Enable
 * |        |          |0 = Slave Transmit FIFO under-run interrupt Disabled.
 * |        |          |1 = Slave Transmit FIFO under-run interrupt Enabled.
 * |[25:24] |RXTH      |Receive FIFO Threshold
 * |        |          |If the valid data count of the receive FIFO buffer is larger than the RXTH setting, the RXTHIF bit will be set to 1, else the RXTHIF bit will be cleared to 0.
 * |        |          |00: 1 word will transmit
 * |        |          |01: 2 word will transmit
 * |        |          |10: 3 word will transmit
 * |        |          |11: 4 word will transmit
 * |[29:28] |TXTH      |Transmit FIFO Threshold
 * |        |          |If the valid data count of the transmit FIFO buffer is less than or equal to the TXTH setting, the TXTHIF bit will be set to 1, else the TXTHIF bit will be cleared to 0.
 * |        |          |00: 1 word will transmit
 * |        |          |01: 2 word will transmit
 * |        |          |10: 3 word will transmit
 * |        |          |11: 4 word will transmit
 * @var SPI_T::STATUS
 * Offset: 0x14  Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BUSY      |SPI Unit Bus Status (Read Only)
 * |        |          |0 = No transaction in the SPI bus.
 * |        |          |1 = SPI controller unit is in busy state.
 * |        |          |The following listing are the bus busy conditions:
 * |        |          |SPIEN = 1 and the TXEMPTY = 0.
 * |        |          |For SPI Master, the TXEMPTY = 1 but the current transaction is not finished yet.
 * |        |          |For SPI Slave receive mode, the SPIEN = 1 and there is serial clock input into the SPI core logic when slave select is active.
 * |        |          |For SPI Slave transmit mode, the SPIEN = 1 and the transmit buffer is not empty in SPI core logic event if the slave select is inactive.
 * |[1]     |UNITIF    |Unit Transfer Interrupt Status
 * |        |          |0 = No transaction has been finished since this bit was cleared to 0.
 * |        |          |1 = SPI controller has finished one unit transfer.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[2]     |SSACTIF   |Slave Select Active Interrupt Status
 * |        |          |0 = Slave select active interrupt is clear or not occur.
 * |        |          |1 = Slave select active interrupt event has occur.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[3]     |SSINAIF   |Slave Select Inactive Interrupt Status
 * |        |          |0 = Slave select inactive interrupt is clear or not occur.
 * |        |          |1 = Slave select inactive interrupt event has occur.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[4]     |SSLINE    |Slave Select Line Bus Status (Read Only)
 * |        |          |0 = Indicates the slave select line bus status is 0.
 * |        |          |1 = Indicates the slave select line bus status is 1.
 * |        |          |Note: If SPI_SSCTL.SSACTPOL is set 0, and the SSLINE is 1, the SPI slave select is in inactive status.
 * |[5]     |SLVTOIF   |Slave Time-out Interrupt Status (Read Only)
 * |        |          |When the Slave Select is active and the value of SLVTOCNT is not 0 and the serial clock input, the slave time-out counter in SPI controller logic will be start
 * |        |          |When the value of time-out counter greater or equal than the value of SPI_SSCTL.SLVTOCNT, during before one transaction done, the slave time-out interrupt event will active.
 * |        |          |0 = Slave time-out is not active.
 * |        |          |1 = Slave time-out is active.
 * |        |          |Note: If the DWIDTH is set 16, one transaction is equal 16 bits serial clock period.
 * |[6]     |SLVBEIF   |Slave Mode Error 0 Interrupt Status (Read Only)
 * |        |          |In Slave mode, there is bit counter mismatch with DWIDTH when the slave select line goes to inactive state.
 * |        |          |0 = No Slave mode error 0 event.
 * |        |          |1 = Slave mode error 0 occurs.
 * |        |          |Note: If the slave select active but there is no any serial clock input, the SLVBEIF also active when the slave select goes to inactive state.
 * |[7]     |SLVURIF   |Slave Mode Error 1 Interrupt Status (Read Only)
 * |        |          |In Slave mode, transmit under-run occurs when the slave select line goes to inactive state.
 * |        |          |0 = No Slave mode error 1 event.
 * |        |          |1 = Slave mode error 1 occurs.
 * |[8]     |RXEMPTY   |Receive FIFO Buffer Empty Indicator (Read Only)
 * |        |          |0 = Receive FIFO buffer is not empty.
 * |        |          |1 = Receive FIFO buffer is empty.
 * |[9]     |RXFULL    |Receive FIFO Buffer Full Indicator (Read Only)
 * |        |          |0 = Receive FIFO buffer is not full.
 * |        |          |1 = Receive FIFO buffer is full.
 * |[10]    |RXTHIF    |Receive FIFO Threshold Interrupt Status (Read Only)
 * |        |          |0 = The valid data count within the Rx FIFO buffer is smaller than or equal to the setting value of RXTH.
 * |        |          |1 = The valid data count within the receive FIFO buffer is larger than the setting value of RXTH.
 * |        |          |Note: If RXTHIEN = 1 and RXTHIF = 1, the SPI controller will generate a SPI interrupt request.
 * |[11]    |RXOVIF    |Receive FIFO Overrun Status
 * |        |          |When the receive FIFO buffer is full, the follow-up data will be dropped and this bit will be set to 1.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[12]    |RXTOIF    |Receive Time-out Interrupt Status
 * |        |          |0 = No receive FIFO time-out event.
 * |        |          |1 = Receive FIFO buffer is not empty and no read operation on receive FIFO buffer over 64 SPI clock period in Master mode or over 576 SPI engine clock period in Slave mode
 * |        |          |When the received FIFO buffer is read by software, the time-out status will be cleared automatically.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[15]    |SPIENSTS  |SPI Enable Bit Status (Read Only)
 * |        |          |0 = Indicate the transmit control bit is disabled.
 * |        |          |1 = Indicate the transfer control bit is active.
 * |        |          |Note: The clock source of SPI controller logic is engine clock, it is asynchronous with the system clock
 * |        |          |In order to make sure the function is disabled in SPI controller logic, this bit indicates the real status of SPIEN in SPI controller logic for user.
 * |[16]    |TXEMPTY   |Transmit FIFO Buffer Empty Indicator (Read Only)
 * |        |          |0 = Transmit FIFO buffer is not empty.
 * |        |          |1 = Transmit FIFO buffer is empty.
 * |[17]    |TXFULL    |Transmit FIFO Buffer Full Indicator (Read Only)
 * |        |          |0 = Transmit FIFO buffer is not full.
 * |        |          |1 = Transmit FIFO buffer is full.
 * |[18]    |TXTHIF    |Transmit FIFO Threshold Interrupt Status (Read Only)
 * |        |          |0 = The valid data count of the transmit FIFO buffer is larger than the setting value of TXTH.
 * |        |          |1 = The valid data count of the transmit FIFO buffer is less than or equal to the setting value of TXTH.
 * |        |          |Note: If TXTHIEN = 1 and TXTHIF = 1, the SPI controller will generate a SPI interrupt request.
 * |[19]    |TXUFIF    |Slave Transmit FIFO Under-run Interrupt Status (Read Only)
 * |        |          |When the transmit FIFO buffer is empty and further serial clock pulses occur, data transmitted will be the value of the last transmitted bit and this under-run bit will be set.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[23]    |TXRXRST   |FIFO CLR Status (Read Only)
 * |        |          |0 = Done the FIFO buffer clear function of TXRST and RXRST.
 * |        |          |1 = Doing the FIFO buffer clear function of TXRST or RXRST.
 * |        |          |Note: Both the TXRST, RXRST, need 3 system clock + 3 engine clocks, the status of this bit allows the user to monitor whether the clear function is busy or done.
 * |[27:24] |RXCNT     |Receive FIFO Data Count (Read Only)
 * |        |          |This bit field indicates the valid data count of receive FIFO buffer.
 * |[31:28] |TXCNT     |Transmit FIFO Data Count (Read Only)
 * |        |          |This bit field indicates the valid data count of transmit FIFO buffer.
 * @var SPI_T::RXTSNCNT
 * Offset: 0x18  Receive Transaction Count Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[16:0]  |RXTSNCNT  |DMA Receive Transaction Count
 * |        |          |When using DMA to receive SPI data without transmitting data, this register can be used in conjunction with the control bit SPI_CTL.RXTCNTEN to set number of transactions to perform
 * |        |          |Without this, the SPI interface will only initiate a transaction when it receives a request from the DMA system, resulting in a lower achievable data rate.
 * @var SPI_T::TX
 * Offset: 0x20  FIFO Data Transmit Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |TX        |Data Transmit Register
 * |        |          |A write to the data transmit register pushes data onto into the 8-level transmit FIFO buffer
 * |        |          |The number of valid bits depends on the setting of transmit bit width field of the SPI_CTL register.
 * |        |          |For example, if DWIDTH is set to 0x08, the bits TX[7:0] will be transmitted
 * |        |          |If DWIDTH is set to 0, the SPI controller will perform a 32-bit transfer.
 * @var SPI_T::RX
 * Offset: 0x30  FIFO Data Receive Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |RX        |Data Receive Register
 * |        |          |A read from this register pops data from the 8-level receive FIFO
 * |        |          |Valid data is present if the SPI_STATUS
 * |        |          |RXEMPTY bit is not set to 1
 * |        |          |This is a read-only register
 */
    __IO uint32_t CTL;                   /*!< [0x0000] Control and Status Register                                      */
    __IO uint32_t CLKDIV;                /*!< [0x0004] Clock Divider Register (Master Only)                             */
    __IO uint32_t SSCTL;                 /*!< [0x0008] Slave Select Register                                            */
    __IO uint32_t PDMACTL;               /*!< [0x000c] SPI PDMA Control Register                                        */
    __IO uint32_t FIFOCTL;               /*!< [0x0010] FIFO Control/Status Register                                     */
    __IO uint32_t STATUS;                /*!< [0x0014] Status Register                                                  */
    __IO uint32_t RXTSNCNT;              /*!< [0x0018] Receive Transaction Count Register                               */
    __I  uint32_t RESERVE0[1];
    __O  uint32_t TX;                    /*!< [0x0020] FIFO Data Transmit Register                                      */
    __I  uint32_t RESERVE1[3];
    __I  uint32_t RX;                    /*!< [0x0030] FIFO Data Receive Register                                       */

} SPI_T;

/**
    @addtogroup SPI_CONST SPI Bit Field Definition
    Constant Definitions for SPI Controller
@{ */

#define SPI_CTL_SPIEN_Pos                (0)                                               /*!< SPI_T::CTL: SPIEN Position             */
#define SPI_CTL_SPIEN_Msk                (0x1ul << SPI_CTL_SPIEN_Pos)                      /*!< SPI_T::CTL: SPIEN Mask                 */

#define SPI_CTL_RXNEG_Pos                (1)                                               /*!< SPI_T::CTL: RXNEG Position             */
#define SPI_CTL_RXNEG_Msk                (0x1ul << SPI_CTL_RXNEG_Pos)                      /*!< SPI_T::CTL: RXNEG Mask                 */

#define SPI_CTL_TXNEG_Pos                (2)                                               /*!< SPI_T::CTL: TXNEG Position             */
#define SPI_CTL_TXNEG_Msk                (0x1ul << SPI_CTL_TXNEG_Pos)                      /*!< SPI_T::CTL: TXNEG Mask                 */

#define SPI_CTL_CLKPOL_Pos               (3)                                               /*!< SPI_T::CTL: CLKPOL Position            */
#define SPI_CTL_CLKPOL_Msk               (0x1ul << SPI_CTL_CLKPOL_Pos)                     /*!< SPI_T::CTL: CLKPOL Mask                */

#define SPI_CTL_SUSPITV_Pos              (4)                                               /*!< SPI_T::CTL: SUSPITV Position           */
#define SPI_CTL_SUSPITV_Msk              (0xful << SPI_CTL_SUSPITV_Pos)                    /*!< SPI_T::CTL: SUSPITV Mask               */

#define SPI_CTL_DWIDTH_Pos               (8)                                               /*!< SPI_T::CTL: DWIDTH Position            */
#define SPI_CTL_DWIDTH_Msk               (0x1ful << SPI_CTL_DWIDTH_Pos)                    /*!< SPI_T::CTL: DWIDTH Mask                */

#define SPI_CTL_LSB_Pos                  (13)                                              /*!< SPI_T::CTL: LSB Position               */
#define SPI_CTL_LSB_Msk                  (0x1ul << SPI_CTL_LSB_Pos)                        /*!< SPI_T::CTL: LSB Mask                   */

#define SPI_CTL_TWOBIT_Pos               (16)                                              /*!< SPI_T::CTL: TWOBIT Position            */
#define SPI_CTL_TWOBIT_Msk               (0x1ul << SPI_CTL_TWOBIT_Pos)                     /*!< SPI_T::CTL: TWOBIT Mask                */

#define SPI_CTL_UNITIEN_Pos              (17)                                              /*!< SPI_T::CTL: UNITIEN Position           */
#define SPI_CTL_UNITIEN_Msk              (0x1ul << SPI_CTL_UNITIEN_Pos)                    /*!< SPI_T::CTL: UNITIEN Mask               */

#define SPI_CTL_SLAVE_Pos                (18)                                              /*!< SPI_T::CTL: SLAVE Position             */
#define SPI_CTL_SLAVE_Msk                (0x1ul << SPI_CTL_SLAVE_Pos)                      /*!< SPI_T::CTL: SLAVE Mask                 */

#define SPI_CTL_REORDER_Pos              (19)                                              /*!< SPI_T::CTL: REORDER Position           */
#define SPI_CTL_REORDER_Msk              (0x1ul << SPI_CTL_REORDER_Pos)                    /*!< SPI_T::CTL: REORDER Mask               */

#define SPI_CTL_QDIODIR_Pos              (20)                                              /*!< SPI_T::CTL: QDIODIR Position           */
#define SPI_CTL_QDIODIR_Msk              (0x1ul << SPI_CTL_QDIODIR_Pos)                    /*!< SPI_T::CTL: QDIODIR Mask               */

#define SPI_CTL_DUALIOEN_Pos             (21)                                              /*!< SPI_T::CTL: DUALIOEN Position          */
#define SPI_CTL_DUALIOEN_Msk             (0x1ul << SPI_CTL_DUALIOEN_Pos)                   /*!< SPI_T::CTL: DUALIOEN Mask              */

#define SPI_CTL_QUADIOEN_Pos             (22)                                              /*!< SPI_T::CTL: QUADIOEN Position          */
#define SPI_CTL_QUADIOEN_Msk             (0x1ul << SPI_CTL_QUADIOEN_Pos)                   /*!< SPI_T::CTL: QUADIOEN Mask              */

#define SPI_CTL_RXTCNTEN_Pos             (23)                                              /*!< SPI_T::CTL: RXTCNTEN Position          */
#define SPI_CTL_RXTCNTEN_Msk             (0x1ul << SPI_CTL_RXTCNTEN_Pos)                   /*!< SPI_T::CTL: RXTCNTEN Mask              */

#define SPI_CTL_RXMODEEN_Pos             (24)                                              /*!< SPI_T::CTL: RXMODEEN Position          */
#define SPI_CTL_RXMODEEN_Msk             (0x1ul << SPI_CTL_RXMODEEN_Pos)                   /*!< SPI_T::CTL: RXMODEEN Mask              */

#define SPI_CLKDIV_DIVIDER_Pos           (0)                                               /*!< SPI_T::CLKDIV: DIVIDER Position        */
#define SPI_CLKDIV_DIVIDER_Msk           (0xfful << SPI_CLKDIV_DIVIDER_Pos)                /*!< SPI_T::CLKDIV: DIVIDER Mask            */

#define SPI_SSCTL_SS_Pos                 (0)                                               /*!< SPI_T::SSCTL: SS Position              */
#define SPI_SSCTL_SS_Msk                 (0x3ul << SPI_SSCTL_SS_Pos)                       /*!< SPI_T::SSCTL: SS Mask                  */

#define SPI_SSCTL_SSACTPOL_Pos           (2)                                               /*!< SPI_T::SSCTL: SSACTPOL Position        */
#define SPI_SSCTL_SSACTPOL_Msk           (0x1ul << SPI_SSCTL_SSACTPOL_Pos)                 /*!< SPI_T::SSCTL: SSACTPOL Mask            */

#define SPI_SSCTL_AUTOSS_Pos             (3)                                               /*!< SPI_T::SSCTL: AUTOSS Position          */
#define SPI_SSCTL_AUTOSS_Msk             (0x1ul << SPI_SSCTL_AUTOSS_Pos)                   /*!< SPI_T::SSCTL: AUTOSS Mask              */

#define SPI_SSCTL_SLV3WIRE_Pos           (4)                                               /*!< SPI_T::SSCTL: SLV3WIRE Position        */
#define SPI_SSCTL_SLV3WIRE_Msk           (0x1ul << SPI_SSCTL_SLV3WIRE_Pos)                 /*!< SPI_T::SSCTL: SLV3WIRE Mask            */

#define SPI_SSCTL_SLVTOIEN_Pos           (5)                                               /*!< SPI_T::SSCTL: SLVTOIEN Position        */
#define SPI_SSCTL_SLVTOIEN_Msk           (0x1ul << SPI_SSCTL_SLVTOIEN_Pos)                 /*!< SPI_T::SSCTL: SLVTOIEN Mask            */

#define SPI_SSCTL_SLVTORST_Pos           (6)                                               /*!< SPI_T::SSCTL: SLVTORST Position        */
#define SPI_SSCTL_SLVTORST_Msk           (0x1ul << SPI_SSCTL_SLVTORST_Pos)                 /*!< SPI_T::SSCTL: SLVTORST Mask            */

#define SPI_SSCTL_SLVBCEIEN_Pos          (8)                                               /*!< SPI_T::SSCTL: SLVBCEIEN Position       */
#define SPI_SSCTL_SLVBCEIEN_Msk          (0x1ul << SPI_SSCTL_SLVBCEIEN_Pos)                /*!< SPI_T::SSCTL: SLVBCEIEN Mask           */

#define SPI_SSCTL_SLVUDRIEN_Pos          (9)                                               /*!< SPI_T::SSCTL: SLVUDRIEN Position       */
#define SPI_SSCTL_SLVUDRIEN_Msk          (0x1ul << SPI_SSCTL_SLVUDRIEN_Pos)                /*!< SPI_T::SSCTL: SLVUDRIEN Mask           */

#define SPI_SSCTL_SSACTIEN_Pos           (12)                                              /*!< SPI_T::SSCTL: SSACTIEN Position        */
#define SPI_SSCTL_SSACTIEN_Msk           (0x1ul << SPI_SSCTL_SSACTIEN_Pos)                 /*!< SPI_T::SSCTL: SSACTIEN Mask            */

#define SPI_SSCTL_SSINAIEN_Pos           (13)                                              /*!< SPI_T::SSCTL: SSINAIEN Position        */
#define SPI_SSCTL_SSINAIEN_Msk           (0x1ul << SPI_SSCTL_SSINAIEN_Pos)                 /*!< SPI_T::SSCTL: SSINAIEN Mask            */

#define SPI_SSCTL_SLVTOCNT_Pos           (16)                                              /*!< SPI_T::SSCTL: SLVTOCNT Position        */
#define SPI_SSCTL_SLVTOCNT_Msk           (0xfffful << SPI_SSCTL_SLVTOCNT_Pos)              /*!< SPI_T::SSCTL: SLVTOCNT Mask            */

#define SPI_PDMACTL_TXPDMAEN_Pos         (0)                                               /*!< SPI_T::PDMACTL: TXPDMAEN Position      */
#define SPI_PDMACTL_TXPDMAEN_Msk         (0x1ul << SPI_PDMACTL_TXPDMAEN_Pos)               /*!< SPI_T::PDMACTL: TXPDMAEN Mask          */

#define SPI_PDMACTL_RXPDMAEN_Pos         (1)                                               /*!< SPI_T::PDMACTL: RXPDMAEN Position      */
#define SPI_PDMACTL_RXPDMAEN_Msk         (0x1ul << SPI_PDMACTL_RXPDMAEN_Pos)               /*!< SPI_T::PDMACTL: RXPDMAEN Mask          */

#define SPI_PDMACTL_PDMARST_Pos          (2)                                               /*!< SPI_T::PDMACTL: PDMARST Position       */
#define SPI_PDMACTL_PDMARST_Msk          (0x1ul << SPI_PDMACTL_PDMARST_Pos)                /*!< SPI_T::PDMACTL: PDMARST Mask           */

#define SPI_FIFOCTL_RXRST_Pos            (0)                                               /*!< SPI_T::FIFOCTL: RXRST Position         */
#define SPI_FIFOCTL_RXRST_Msk            (0x1ul << SPI_FIFOCTL_RXRST_Pos)                  /*!< SPI_T::FIFOCTL: RXRST Mask             */

#define SPI_FIFOCTL_TXRST_Pos            (1)                                               /*!< SPI_T::FIFOCTL: TXRST Position         */
#define SPI_FIFOCTL_TXRST_Msk            (0x1ul << SPI_FIFOCTL_TXRST_Pos)                  /*!< SPI_T::FIFOCTL: TXRST Mask             */

#define SPI_FIFOCTL_RXTHIEN_Pos          (2)                                               /*!< SPI_T::FIFOCTL: RXTHIEN Position       */
#define SPI_FIFOCTL_RXTHIEN_Msk          (0x1ul << SPI_FIFOCTL_RXTHIEN_Pos)                /*!< SPI_T::FIFOCTL: RXTHIEN Mask           */

#define SPI_FIFOCTL_TXTHIEN_Pos          (3)                                               /*!< SPI_T::FIFOCTL: TXTHIEN Position       */
#define SPI_FIFOCTL_TXTHIEN_Msk          (0x1ul << SPI_FIFOCTL_TXTHIEN_Pos)                /*!< SPI_T::FIFOCTL: TXTHIEN Mask           */

#define SPI_FIFOCTL_RXTOIEN_Pos          (4)                                               /*!< SPI_T::FIFOCTL: RXTOIEN Position       */
#define SPI_FIFOCTL_RXTOIEN_Msk          (0x1ul << SPI_FIFOCTL_RXTOIEN_Pos)                /*!< SPI_T::FIFOCTL: RXTOIEN Mask           */

#define SPI_FIFOCTL_RXOVIEN_Pos          (5)                                               /*!< SPI_T::FIFOCTL: RXOVIEN Position       */
#define SPI_FIFOCTL_RXOVIEN_Msk          (0x1ul << SPI_FIFOCTL_RXOVIEN_Pos)                /*!< SPI_T::FIFOCTL: RXOVIEN Mask           */

#define SPI_FIFOCTL_TXUDFPOL_Pos         (6)                                               /*!< SPI_T::FIFOCTL: TXUDFPOL Position      */
#define SPI_FIFOCTL_TXUDFPOL_Msk         (0x1ul << SPI_FIFOCTL_TXUDFPOL_Pos)               /*!< SPI_T::FIFOCTL: TXUDFPOL Mask          */

#define SPI_FIFOCTL_TXUDFIEN_Pos         (7)                                               /*!< SPI_T::FIFOCTL: TXUDFIEN Position      */
#define SPI_FIFOCTL_TXUDFIEN_Msk         (0x1ul << SPI_FIFOCTL_TXUDFIEN_Pos)               /*!< SPI_T::FIFOCTL: TXUDFIEN Mask          */

#define SPI_FIFOCTL_RXTH_Pos             (24)                                              /*!< SPI_T::FIFOCTL: RXTH Position          */
#define SPI_FIFOCTL_RXTH_Msk             (0x3ul << SPI_FIFOCTL_RXTH_Pos)                   /*!< SPI_T::FIFOCTL: RXTH Mask              */

#define SPI_FIFOCTL_TXTH_Pos             (28)                                              /*!< SPI_T::FIFOCTL: TXTH Position          */
#define SPI_FIFOCTL_TXTH_Msk             (0x3ul << SPI_FIFOCTL_TXTH_Pos)                   /*!< SPI_T::FIFOCTL: TXTH Mask              */

#define SPI_STATUS_BUSY_Pos              (0)                                               /*!< SPI_T::STATUS: BUSY Position           */
#define SPI_STATUS_BUSY_Msk              (0x1ul << SPI_STATUS_BUSY_Pos)                    /*!< SPI_T::STATUS: BUSY Mask               */

#define SPI_STATUS_UNITIF_Pos            (1)                                               /*!< SPI_T::STATUS: UNITIF Position         */
#define SPI_STATUS_UNITIF_Msk            (0x1ul << SPI_STATUS_UNITIF_Pos)                  /*!< SPI_T::STATUS: UNITIF Mask             */

#define SPI_STATUS_SSACTIF_Pos           (2)                                               /*!< SPI_T::STATUS: SSACTIF Position        */
#define SPI_STATUS_SSACTIF_Msk           (0x1ul << SPI_STATUS_SSACTIF_Pos)                 /*!< SPI_T::STATUS: SSACTIF Mask            */

#define SPI_STATUS_SSINAIF_Pos           (3)                                               /*!< SPI_T::STATUS: SSINAIF Position        */
#define SPI_STATUS_SSINAIF_Msk           (0x1ul << SPI_STATUS_SSINAIF_Pos)                 /*!< SPI_T::STATUS: SSINAIF Mask            */

#define SPI_STATUS_SSLINE_Pos            (4)                                               /*!< SPI_T::STATUS: SSLINE Position         */
#define SPI_STATUS_SSLINE_Msk            (0x1ul << SPI_STATUS_SSLINE_Pos)                  /*!< SPI_T::STATUS: SSLINE Mask             */

#define SPI_STATUS_SLVTOIF_Pos           (5)                                               /*!< SPI_T::STATUS: SLVTOIF Position        */
#define SPI_STATUS_SLVTOIF_Msk           (0x1ul << SPI_STATUS_SLVTOIF_Pos)                 /*!< SPI_T::STATUS: SLVTOIF Mask            */

#define SPI_STATUS_SLVBEIF_Pos           (6)                                               /*!< SPI_T::STATUS: SLVBEIF Position        */
#define SPI_STATUS_SLVBEIF_Msk           (0x1ul << SPI_STATUS_SLVBEIF_Pos)                 /*!< SPI_T::STATUS: SLVBEIF Mask            */

#define SPI_STATUS_SLVURIF_Pos           (7)                                               /*!< SPI_T::STATUS: SLVURIF Position        */
#define SPI_STATUS_SLVURIF_Msk           (0x1ul << SPI_STATUS_SLVURIF_Pos)                 /*!< SPI_T::STATUS: SLVURIF Mask            */

#define SPI_STATUS_RXEMPTY_Pos           (8)                                               /*!< SPI_T::STATUS: RXEMPTY Position        */
#define SPI_STATUS_RXEMPTY_Msk           (0x1ul << SPI_STATUS_RXEMPTY_Pos)                 /*!< SPI_T::STATUS: RXEMPTY Mask            */

#define SPI_STATUS_RXFULL_Pos            (9)                                               /*!< SPI_T::STATUS: RXFULL Position         */
#define SPI_STATUS_RXFULL_Msk            (0x1ul << SPI_STATUS_RXFULL_Pos)                  /*!< SPI_T::STATUS: RXFULL Mask             */

#define SPI_STATUS_RXTHIF_Pos            (10)                                              /*!< SPI_T::STATUS: RXTHIF Position         */
#define SPI_STATUS_RXTHIF_Msk            (0x1ul << SPI_STATUS_RXTHIF_Pos)                  /*!< SPI_T::STATUS: RXTHIF Mask             */

#define SPI_STATUS_RXOVIF_Pos            (11)                                              /*!< SPI_T::STATUS: RXOVIF Position         */
#define SPI_STATUS_RXOVIF_Msk            (0x1ul << SPI_STATUS_RXOVIF_Pos)                  /*!< SPI_T::STATUS: RXOVIF Mask             */

#define SPI_STATUS_RXTOIF_Pos            (12)                                              /*!< SPI_T::STATUS: RXTOIF Position         */
#define SPI_STATUS_RXTOIF_Msk            (0x1ul << SPI_STATUS_RXTOIF_Pos)                  /*!< SPI_T::STATUS: RXTOIF Mask             */

#define SPI_STATUS_SPIENSTS_Pos          (15)                                              /*!< SPI_T::STATUS: SPIENSTS Position       */
#define SPI_STATUS_SPIENSTS_Msk          (0x1ul << SPI_STATUS_SPIENSTS_Pos)                /*!< SPI_T::STATUS: SPIENSTS Mask           */

#define SPI_STATUS_TXEMPTY_Pos           (16)                                              /*!< SPI_T::STATUS: TXEMPTY Position        */
#define SPI_STATUS_TXEMPTY_Msk           (0x1ul << SPI_STATUS_TXEMPTY_Pos)                 /*!< SPI_T::STATUS: TXEMPTY Mask            */

#define SPI_STATUS_TXFULL_Pos            (17)                                              /*!< SPI_T::STATUS: TXFULL Position         */
#define SPI_STATUS_TXFULL_Msk            (0x1ul << SPI_STATUS_TXFULL_Pos)                  /*!< SPI_T::STATUS: TXFULL Mask             */

#define SPI_STATUS_TXTHIF_Pos            (18)                                              /*!< SPI_T::STATUS: TXTHIF Position         */
#define SPI_STATUS_TXTHIF_Msk            (0x1ul << SPI_STATUS_TXTHIF_Pos)                  /*!< SPI_T::STATUS: TXTHIF Mask             */

#define SPI_STATUS_TXUFIF_Pos            (19)                                              /*!< SPI_T::STATUS: TXUFIF Position         */
#define SPI_STATUS_TXUFIF_Msk            (0x1ul << SPI_STATUS_TXUFIF_Pos)                  /*!< SPI_T::STATUS: TXUFIF Mask             */

#define SPI_STATUS_TXRXRST_Pos           (23)                                              /*!< SPI_T::STATUS: TXRXRST Position        */
#define SPI_STATUS_TXRXRST_Msk           (0x1ul << SPI_STATUS_TXRXRST_Pos)                 /*!< SPI_T::STATUS: TXRXRST Mask            */

#define SPI_STATUS_RXCNT_Pos             (24)                                              /*!< SPI_T::STATUS: RXCNT Position          */
#define SPI_STATUS_RXCNT_Msk             (0xful << SPI_STATUS_RXCNT_Pos)                   /*!< SPI_T::STATUS: RXCNT Mask              */

#define SPI_STATUS_TXCNT_Pos             (28)                                              /*!< SPI_T::STATUS: TXCNT Position          */
#define SPI_STATUS_TXCNT_Msk             (0xful << SPI_STATUS_TXCNT_Pos)                   /*!< SPI_T::STATUS: TXCNT Mask              */

#define SPI_RXTSNCNT_RXTSNCNT_Pos        (0)                                               /*!< SPI_T::RXTSNCNT: RXTSNCNT Position     */
#define SPI_RXTSNCNT_RXTSNCNT_Msk        (0x1fffful << SPI_RXTSNCNT_RXTSNCNT_Pos)          /*!< SPI_T::RXTSNCNT: RXTSNCNT Mask         */

#define SPI_TX_TX_Pos                    (0)                                               /*!< SPI_T::TX: TX Position                 */
#define SPI_TX_TX_Msk                    (0xfffffffful << SPI_TX_TX_Pos)                   /*!< SPI_T::TX: TX Mask                     */

#define SPI_RX_RX_Pos                    (0)                                               /*!< SPI_T::RX: RX Position                 */
#define SPI_RX_RX_Msk                    (0xfffffffful << SPI_RX_RX_Pos)                   /*!< SPI_T::RX: RX Mask                     */

/**@}*/ /* SPI_CONST */
/**@}*/ /* end of SPI register group */


/*---------------------- System Manger Controller -------------------------*/
/**
    @addtogroup SYS System Manger Controller(SYS)
    Memory Mapped Structure for SYS Controller
@{ */
 
typedef struct
{


/**
 * @var SYS_T::PDID
 * Offset: 0x00  Product Identifier Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |IMG2      |Product Identifier
 * |        |          |Data in MAP2 of information block are copied to this register after power on
 * |        |          |MAP2 is used to store part number defined by Nuvoton.
 * @var SYS_T::RSTSTS
 * Offset: 0x04  System Reset Source Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PORF      |POR Reset Flag
 * |        |          |The POR reset flag is set by the u201CReset Signalu201D from the Power-on Reset (POR) Controller to indicate the previous reset source.
 * |        |          |0 = No reset from POR.
 * |        |          |1 = Power-on Reset (POR) Controller had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0. 
 * |[1]     |PINRF     |nRESET Pin Reset Flag
 * |        |          |The nRESET pin reset flag is set by the u201CReset Signalu201D from the nRESET Pin to indicate the previous reset source.
 * |        |          |0 = No reset from nRESET pin.
 * |        |          |1 = Pin nRESET had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0. 
 * |[2]     |WDTRF     |Reset Source From WDG
 * |        |          |The WDTRF flag is set if pervious reset source originates from the Watch-Dog module.
 * |        |          |0= No reset from Watch-Dog.
 * |        |          |1= The Watch-Dog module issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[3]     |LVRF      |LVR Reset Flag
 * |        |          |The LVR reset flag is set by the u201CReset Signalu201D from the Low Voltage Reset Controller to indicate the previous reset source.
 * |        |          |0 = No reset from LVR.
 * |        |          |1 = LVR controller had issued the reset signal to reset the system.
 * |        |          |Note1: Write 1 to clear this bit to 0.
 * |        |          |Note2: If power rising reach 1.6V under 20us when fast power on, the LVRF will not happen.
 * |[4]     |BODRF     |BOD Reset Flag
 * |        |          |The BOD reset flag is set by the u201CReset Signalu201D from the Brown Out Reset Controller to indicate the previous reset source.
 * |        |          |0 = No reset from BOD.
 * |        |          |1 = BOD controller had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[6]     |PMURSTF   |Reset Source From PMU
 * |        |          |The PMURSTF flag is set by the reset signal from the PMU module to indicate the previous reset source.
 * |        |          |0= No reset from PMU.
 * |        |          |1= The PMU has issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[8]     |PINWK     |Wakeup from DPD From PIN
 * |        |          |The device was woken from Deep Power Down by a low transition on the RESETn pin.
 * |        |          |0= No wakeup from RESETn pin.
 * |        |          |1= The device was issued a wakeup from DPD by a RESETn pin trasition.
 * |        |          |Note: Write 1 to this register to clear all wakeup flags.
 * |[9]     |TIMWK     |Wakeup from DPD From TIMER
 * |        |          |The device was woken from Deep Power Down by count of 10 KHz timer.
 * |        |          |0= No wakeup from TIMER.
 * |        |          |1= The device was issued a wakeup from DPD by a TIMER event.
 * |        |          |Note: Clear by write SYS_RSTSTS[8] =1u2019b
 * |[10]    |PORWK     |Wakeup from DPD From POR
 * |        |          |The device was woken from Deep Power Down by a Power On Reset.
 * |        |          |0= No wakeup from POR.
 * |        |          |1= The device was issued a wakeup from DPD by a POR.
 * |        |          |Note: Clear by write SYS_RSTSTS[8] =1u2019b
 * @var SYS_T::IPRST0
 * Offset: 0x08  IP Reset Control Resister0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CHIPRST   |CHIP One Shot Reset
 * |        |          |Set this bit will reset the whole chip, this bit will automatically return to u201C0u201D after 2 clock cycles.
 * |        |          |CHIPRST is same as POR reset, all the chip modules are reset and the chip configuration settings from flash are reloaded.
 * |        |          |0 = Normal.
 * |        |          |1 = Reset CHIP.
 * |[1]     |CPURST    |CPU Kernel One Shot Reset
 * |        |          |Setting this bit will reset the CPU kernel and Flash Memory Controller (FMC), this bit will automatically return to u201C0u201D after the 2 clock cycles
 * |        |          |0 = Normal.
 * |        |          |1 = Reset CPU.
 * @var SYS_T::IPRST1
 * Offset: 0x0C  IP Reset Control Resister1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |GPIORST   |GPIO Controller Reset
 * |        |          |0 = Normal operation.
 * |        |          |1 = Reset.
 * |[2]     |TMR0RST   |Timer0 Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[3]     |TMR1RST   |Timer1 Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[4]     |TMR2RST   |Timer2 Controller Reset
 * |        |          |0 = Normal operation.
 * |        |          |1 = Reset.
 * |[6]     |CPDRST    |Companding Controller Reset
 * |        |          |0 = Normal operation.
 * |        |          |1 = Reset.
 * |[7]     |PDMARST   |PDMA Controller Reset
 * |        |          |0 = Normal operation.
 * |        |          |1 = Reset.
 * |[8]     |I2C0RST   |I2C0 Controller Reset
 * |        |          |0 = Normal operation.
 * |        |          |1 = Reset.
 * |[9]     |I2C1RST   |I2C1 Controller Reset
 * |        |          |0 = Normal operation.
 * |        |          |1 = Reset.
 * |[11]    |SPI1RST   |SPI1 Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.Reserved
 * |[12]    |SPI0RST   |SPI0 Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[13]    |I2SRST    |I2S Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[16]    |UART0RST  |UART0 Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[17]    |UART1RST  |UART1 Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[18]    |BIQRST    |BIQ Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[20]    |PWM0RST   |PWM0 Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[21]    |PWM1RST   |PWM1 Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[24]    |USBRST    |USB Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[28]    |SARADCRST |SARADC Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[29]    |DACRST    |DAC Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[30]    |SDADCRST  |SDADC Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * |[31]    |ANARST    |Analog Block Controller Reset
 * |        |          |0 = Normal Operation.
 * |        |          |1 = Reset.
 * @var SYS_T::BODCTL
 * Offset: 0x18  Brown-out Detector Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BODEN     |Brown-Out Detector Threshold Voltage Selection Extension (Initialized & Protected Bit)
 * |        |          |The default value is set by flash controller as inverse of user configuration CBODEN bit (config0 [20]).
 * |        |          |0 = Brown-Out Detector function is disabled
 * |        |          |1 = Brown-Out Detector function enabled
 * |[1]     |BODRSTEN  |Brown-Out Detector Reset or Interrupt Bit (Initialized & Protected Bit)
 * |        |          |The default value is set by flash controller as inverse of user configuration CBORST bit (config0 [21]).
 * |        |          |0 = Brown-Out Detector generate an interrupt
 * |        |          |1 = Brown-Out Detector will reset chip
 * |        |          |When the BOD is enabled and the interrupt is asserted, the interrupt will be kept till the BOD is disabled
 * |        |          |The interrupt for CPU can be blocked either by disabling the interrupt in the NVIC or by disabling the interrupt source by disabling the BOD
 * |        |          |BOD can then be re-enabled as required.
 * |[5:2]   |BODLVL    |Brown-Out Detector Threshold Voltage Selection (Initialized & Protected Bit)
 * |        |          |The default value is set by flash controller user configuration CBOV bit (config0 [25:22]).
 * |        |          |BOD_LVL[3:0]
 * |        |          |Brown-out voltage
 * |        |          |BOD_LVL[3:0]
 * |        |          |Brown-out voltage
 * |        |          |0111
 * |        |          |2.8V
 * |        |          |1111
 * |        |          |3.4V
 * |        |          |0110
 * |        |          |2.6V
 * |        |          |1110
 * |        |          |3.4V
 * |        |          |0101
 * |        |          |2.4V
 * |        |          |1101
 * |        |          |3.4V
 * |        |          |0100
 * |        |          |2.2V
 * |        |          |1100
 * |        |          |3.4V
 * |        |          |0011
 * |        |          |2.1V
 * |        |          |1011
 * |        |          |3.4V
 * |        |          |0010
 * |        |          |2.0V
 * |        |          |1010
 * |        |          |3.4V
 * |        |          |0001
 * |        |          |1.9V
 * |        |          |1001
 * |        |          |3.1V
 * |        |          |0000
 * |        |          |1.8V
 * |        |          |1000
 * |        |          |3.0V
 * |[6]     |BODHYS    |Brown-Out Detector Hysteresis (Initialized & Protected Bit)
 * |        |          |The default value is set by flash controller user configuration CBOV [4] bit (config0 [26]).
 * |        |          |0 = No hysteresis on BOD detection.
 * |        |          |1 = BOD hysteresis enabled.
 * |[7]     |BODOUT    |Brown-Out Detector Output State
 * |        |          |0 = Brown-out Detector status output is 0, the detected voltage is higher than BOD_VL setting.
 * |        |          |1 = Brown-out Detector status output is 1, the detected voltage is lower than BOD_VL setting.
 * |[8]     |BODINT    |Brown-Out Dectector Interrupt
 * |        |          |1 = indicates BOD_INT is active. Write 1 to clear.
 * |[16]    |LVREN     |Low Voltage Reset (LVR) Enable (Initialized & Protected Bit)
 * |        |          |The LVR function resets the chip when the input power voltage is lower than LVR trip point
 * |        |          |Default value is set by flash controller as inverse of CLVR config 0[27].
 * |        |          |0 = Disable LVR function.
 * |        |          |1 = Enable LVR function.
 * |[18:17] |LVRFILTER |00 = LVR output will be filtered by 1 HCLK.
 * |        |          |01 = LVR output will be filtered by 2 HCLK
 * |        |          |10 = LVR output will be filtered by 8 HCLK
 * |        |          |11 = LVR output will be filtered by 15 HCLK
 * |        |          |Default value is 00.
 * @var SYS_T::PORCTL
 * Offset: 0x1C  Power-On-reset Controller Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |POROFF    |Power-on Reset Enable Bit (Write Protected)
 * |        |          |When power is applied to device, the POR circuit generates a reset signal to reset the entire chip function
 * |        |          |Noise on the power may cause the POR to become active again
 * |        |          |User can disable internal POR circuit to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
 * |        |          |The POR function will be active again when this field is set to another value or chip is reset by other reset source, including:
 * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
 * |        |          |Note1: This bit does write protected. Refer to the SYS_REGLCTL register.
 * |        |          |Note2: This function will not work under DPD mode.
 * @var SYS_T::GPA_MFP
 * Offset: 0x20  GPIO PA Multiple Alternate Functions and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |PA0MFP    |PA.0 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA0MFP
 * |        |          |PA.0
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.0
 * |        |          |I2S0_MCLK
 * |        |          |IROUT
 * |        |          |SPI1_MISO1
 * |[3:2]   |PA1MFP    |PA.1 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA1MFP
 * |        |          |PA.1
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.1
 * |        |          |I2S0_LRCK
 * |        |          |I2C0_SCL
 * |        |          |SPI1_MOSI1 UART1_TX
 * |[5:4]   |PA2MFP    |PA.2 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA2MFP
 * |        |          |PA.2
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.2
 * |        |          |I2S0_BCLK
 * |        |          |I2C0_SDA
 * |        |          |SPI1_MOSI0 UART1_RX
 * |[7:6]   |PA3MFP    |PA.3 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA3MFP
 * |        |          |PA.3
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.3
 * |        |          |I2S0_DO
 * |        |          |UART1_TX
 * |        |          |SPI1_CLK
 * |[9:8]   |PA4MFP    |PA.4 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA4MFP
 * |        |          |PA.4
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.4
 * |        |          |I2S0_DI
 * |        |          |UART1_RX
 * |        |          |SPI1_MISO0
 * |[11:10] |PA5MFP    |PA.5 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA5MFP
 * |        |          |PA.5
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.5
 * |        |          |MCLKI
 * |        |          |SPI1_SS0
 * |[13:12] |PA6MFP    |PA.6 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA6MFP
 * |        |          |PA.6
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.6
 * |        |          |TM0
 * |        |          |CAP1
 * |        |          |PWM10
 * |[15:14] |PA7MFP    |PA.7 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA7MFP
 * |        |          |PA.7
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.7
 * |        |          |TM1
 * |        |          |SPI1_SS1
 * |        |          |PWM11
 * |[17:16] |PA8MFP    |PA.8 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA8MFP
 * |        |          |PA.8
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.8
 * |        |          |TM2
 * |        |          |UART0_TX
 * |        |          |PWM12
 * |[19:18] |PA9MFP    |PA.9 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA9MFP
 * |        |          |PA.9
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.9
 * |        |          |SPI0_SS1
 * |        |          |UART0_RX
 * |        |          |PWM13
 * |[21:20] |PA10MFP   |PA.10 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA10MFP
 * |        |          |PA.10
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.10
 * |        |          |SPI0_MISO1
 * |        |          |MCLKI
 * |        |          |UART0_TX
 * |[23:22] |PA11MFP   |PA.11 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA11MFP
 * |        |          |PA.11
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.11
 * |        |          |SPI0_MOSI1
 * |        |          |I2C1_SCL
 * |        |          |UART0_RX
 * |[25:24] |PA12MFP   |PA.12 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA12MFP
 * |        |          |PA.12
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.12
 * |        |          |SPI0_MOSI0
 * |        |          |I2C1_SDA
 * |        |          |UART0_nCTS
 * |[27:26] |PA13MFP   |PA.13 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA13MFP
 * |        |          |PA.13
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.13
 * |        |          |SPI0_CLK
 * |        |          |UART0_nRTS
 * |[29:28] |PA14MFP   |PA.14 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA14MFP
 * |        |          |PA.14
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.14
 * |        |          |SPI0_MISO0
 * |        |          |UART1_TX
 * |[31:30] |PA15MFP   |PA.15 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PA15MFP
 * |        |          |PA.15
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PA.15
 * |        |          |SPI0_SS0
 * |        |          |UART1_RX
 * @var SYS_T::GPB_MFP
 * Offset: 0x24  GPIO PB Multiple Alternate Functions and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |PB0MFP    |PB.0 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PB0MFP
 * |        |          |PB.0
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PB.0
 * |        |          |I2C1_SCL
 * |[3:2]   |PB1MFP    |PB.1 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PB1MFP
 * |        |          |PB.1
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PB.1
 * |        |          |I2C1_SDA
 * @var SYS_T::GPC_MFP
 * Offset: 0x28  GPIO PC Multiple Alternate Functions and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |PC0MFP    |PC.0 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC0MFP
 * |        |          |PC.0
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.0
 * |        |          |XT1_OUT
 * |        |          |I2C0_SCL
 * |[3:2]   |PC1MFP    |PC.1 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC1MFP
 * |        |          |PC.1
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.1
 * |        |          |XT1_IN
 * |        |          |I2C0_SDA
 * |[5:4]   |PC2MFP    |PC.2 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC2MFP
 * |        |          |PC.2
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.2
 * |        |          |UART0_nCTS
 * |        |          |I2S0_LRCK
 * |        |          |PWM00
 * |[7:6]   |PC3MFP    |PC.3 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC3MFP
 * |        |          |PC.3
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.3
 * |        |          |UART0_nRTS
 * |        |          |I2S0_BCLK
 * |        |          |PWM01
 * |[9:8]   |PC4MFP    |PC.4 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC4MFP
 * |        |          |PC.4
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.4
 * |        |          |UART0_TX
 * |        |          |I2S0_DO
 * |        |          |PWM02
 * |[11:10] |PC5MFP    |PC.5 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC5MFP
 * |        |          |PC.5
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.5
 * |        |          |UART0_RX
 * |        |          |I2S0_DI
 * |        |          |PWM03
 * |[13:12] |PC6MFP    |PC.6 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC6MFP
 * |        |          |PC.6
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.6
 * |        |          |I2C0_SCL
 * |        |          |SPI1_SS0
 * |        |          |PWM10
 * |[15:14] |PC7MFP    |PC.7 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC7MFP
 * |        |          |PC.7
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.7
 * |        |          |I2C0_SDA
 * |        |          |SPI1_SS1
 * |        |          |PWM11
 * |[17:16] |PC8MFP    |PC.8 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC8MFP
 * |        |          |PC.8
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.8
 * |        |          |SPI0_SS1
 * |        |          |I2S0_MCLK
 * |        |          |PWM12
 * |[19:18] |PC9MFP    |PC.9 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC9MFP
 * |        |          |PC.9
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.9
 * |        |          |SPI0_MISO1
 * |        |          |PWM13
 * |[21:20] |PC10MFP   |PC.10 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC10MFP
 * |        |          |PC.10
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.10
 * |        |          |SPI0_MOSI1
 * |        |          |I2S0_MCLK
 * |        |          |MCLKI
 * |[23:22] |PC11MFP   |PC.11 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC11MFP
 * |        |          |PC.11
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.11
 * |        |          |SPI0_MOSI0
 * |        |          |I2S0_LRCK
 * |        |          |UART1_nCTS
 * |[25:24] |PC12MFP   |PC.12 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC12MFP
 * |        |          |PC.12
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.12
 * |        |          |SPI0_CLK
 * |        |          |I2S0_BCLK
 * |        |          |UART1_nRTS
 * |[27:26] |PC13MFP   |PC.13 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC13MFP
 * |        |          |PC.13
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.13
 * |        |          |SPI0_MISO0
 * |        |          |I2S0_DO
 * |        |          |UART1_TX
 * |[29:28] |PC14MFP   |PC.14 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC14MFP
 * |        |          |PC.14
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.14
 * |        |          |SPI0_SS0
 * |        |          |I2S0_DI
 * |        |          |UART1_RX
 * |[31:30] |PC15MFP   |PC.15 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC15MFP
 * |        |          |PC.15
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.15
 * |        |          |MCLKI
 * @var SYS_T::GPD_MFP
 * Offset: 0x2C  GPIO PD Multiple Alternate Functions and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |PD0MFP    |PD.0 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD0MFP
 * |        |          |PD.0
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.0
 * |        |          |UART1_nCTS
 * |        |          |PWM00
 * |        |          |I2S0_MCLK
 * |[3:2]   |PD1MFP    |PD.1 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD1MFP
 * |        |          |PD.1
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.1
 * |        |          |UART1_nRTS
 * |        |          |PWM01
 * |        |          |I2S0_LRCK
 * |[5:4]   |PD2MFP    |PD.2 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD2MFP
 * |        |          |PD.2
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PC.2
 * |        |          |UART1_TX
 * |        |          |PWM02
 * |        |          |I2S0_BCLK
 * |[7:6]   |PD3MFP    |PC.3 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD3MFP
 * |        |          |PD.3
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.3
 * |        |          |UART1_RX
 * |        |          |PWM03
 * |        |          |I2S0_DO
 * |[9:8]   |PD4MFP    |PC.4 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD4MFP
 * |        |          |PD.4
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.4
 * |        |          |PWM00
 * |        |          |CAP0
 * |        |          |I2S0_DI
 * |[11:10] |PD5MFP    |PD.5 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD5MFP
 * |        |          |PD.5
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.5
 * |        |          |PWM01
 * |        |          |SPI1_MOSI0
 * |[13:12] |PD6MFP    |PD.6 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD6MFP
 * |        |          |PD.6
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.6
 * |        |          |PWM02
 * |        |          |SPI1_CLK
 * |[15:14] |PD7MFP    |PD.7 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD7MFP
 * |        |          |PD.7
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.7
 * |        |          |PWM03
 * |        |          |SPI1_MISO0
 * |[17:16] |PD8MFP    |PD.8 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD8MFP
 * |        |          |PD.8
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.8
 * |        |          |PWM10
 * |        |          |SPI0_SS0
 * |        |          |UART0_TX
 * |[19:18] |PD9MFP    |PD.9 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD9MFP
 * |        |          |PD.9
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.9
 * |        |          |PWM11
 * |        |          |SPI0_MISO0
 * |        |          |UART0_RX
 * |[21:20] |PD10MFP   |PD.10 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD10MFP
 * |        |          |PD.10
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.10
 * |        |          |PWM12
 * |        |          |SPI0_CLK
 * |        |          |I2C1_SCL
 * |[23:22] |PD11MFP   |PD.11 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD11MFP
 * |        |          |PD.11
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.11
 * |        |          |PWM13
 * |        |          |SPI0_MOSI0
 * |        |          |I2C1_SDA
 * |[25:24] |PD12MFP   |PD.12 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD12MFP
 * |        |          |PD.12
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.12
 * |        |          |I2C1_SCL
 * |        |          |SPI0_MOSI1
 * |        |          |ICE_CLK
 * |[27:26] |PD13MFP   |PD.13 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD13MFP
 * |        |          |PD.13
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.13
 * |        |          |I2C1_SDA
 * |        |          |SPI0_MISO1
 * |        |          |ICE_DAT
 * |[29:28] |PD14MFP   |PD.14 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PD14MFP
 * |        |          |PD.14
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.14
 * |        |          |I2C0_SCL
 * |        |          |SPI0_SS1
 * |        |          |SPI1_MISO1
 * |[31:30] |PD15MFP   |PD.15 Multi-function Pin Selection
 * |        |          |Pin Name
 * |        |          |PC15MFP
 * |        |          |PD.15
 * |        |          |00
 * |        |          |01
 * |        |          |10
 * |        |          |11
 * |        |          |PD.15
 * |        |          |I2C0_SDA
 * |        |          |SPI1_MOSI1
 * @var SYS_T::GPIO_INTP
 * Offset: 0x40  GPIO Input Type and Slew Rate Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[13:0]  |GPxSSGPxHS|This   register controls whether the GPIO input buffer Schmitt trigger is enabled   and whether high or low slew rate is selected for output driver.
 * |        |          |Each   bit controls a group of four GPIO pins
 * |        |          |GPx[m:n]SS   = 1 :   input Schmitt Trigger enabled.
 * |        |          |GPx[m:n]SS   = 0 :   input CMOS enabled.
 * |        |          |GPx[m:n]HS   = 1:   Output high slew rate.
 * |        |          |GPx[m:n]HS   = 0 :   Output low slew rate.
 * @var SYS_T::GPA_PULL
 * Offset: 0x44  PA.15 ~ PA.0 Pull Resistance Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PUEN0     |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[1]     |PUEN1     |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[2]     |PUEN2     |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[3]     |PUEN3     |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[4]     |PUEN4     |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[5]     |PUEN5     |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[6]     |PUEN6     |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[7]     |PUEN7     |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[8]     |PUEN8     |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[9]     |PUEN9     |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[10]    |PUEN10    |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[11]    |PUEN11    |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[12]    |PUEN12    |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[13]    |PUEN13    |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[14]    |PUEN14    |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[15]    |PUEN15    |PA.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * @var SYS_T::GPA_HR
 * Offset: 0x48  PA.15 ~ PA.0 Pull Resistance Select Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PUHR0     |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[1]     |PUHR1     |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[2]     |PUHR2     |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[3]     |PUHR3     |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[4]     |PUHR4     |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[5]     |PUHR5     |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[6]     |PUHR6     |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[7]     |PUHR7     |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[8]     |PUHR8     |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[9]     |PUHR9     |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[10]    |PUHR10    |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[11]    |PUHR11    |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[12]    |PUHR12    |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[13]    |PUHR13    |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[14]    |PUHR14    |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[15]    |PUHR15    |PA.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * @var SYS_T::GPA_IEN
 * Offset: 0x4C  PA.15 ~ PA.0 Digital and Analog Input Buffer Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IEN       |PA.n Digital Input Buffer Control Register. n = 15~0
 * |        |          |0 = Input buffer Enabled.
 * |        |          |1 = Input buffer disabled, and input signal always equals to 0.
 * @var SYS_T::GPB_PULL
 * Offset: 0x54  PB.1 ~ PB.0 Pull Resistance Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PUEN0     |PB.n Pull Control Register. n = 1~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[1]     |PUEN1     |PB.n Pull Control Register. n = 1~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * @var SYS_T::GPB_HR
 * Offset: 0x58  PB.1 ~ PB.0 Pull Resistance Select Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PUHR0     |PB.n Pull Resistance Select Control Register. n = 1~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[1]     |PUHR1     |PB.n Pull Resistance Select Control Register. n = 1~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * @var SYS_T::GPB_IEN
 * Offset: 0x5C  PB.1 ~ PB.0 Digital Input Buffer Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IEN0      |PB.n Digital Input Buffer Control Register. n = 1~0
 * |        |          |0 = Input buffer Enabled.
 * |        |          |1 = Input buffer disabled, and input signal always equals to 0.
 * |[1]     |IEN1      |PB.n Digital Input Buffer Control Register. n = 1~0
 * |        |          |0 = Input buffer Enabled.
 * |        |          |1 = Input buffer disabled, and input signal always equals to 0.
 * @var SYS_T::GPC_PULL
 * Offset: 0x64  PC.15 ~ PC.0 Pull Resistance Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PUEN0     |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[1]     |PUEN1     |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[2]     |PUEN2     |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[3]     |PUEN3     |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[4]     |PUEN4     |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[5]     |PUEN5     |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[6]     |PUEN6     |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[7]     |PUEN7     |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[8]     |PUEN8     |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[9]     |PUEN9     |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[10]    |PUEN10    |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[11]    |PUEN11    |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[12]    |PUEN12    |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[13]    |PUEN13    |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[14]    |PUEN14    |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[15]    |PUEN15    |PC.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * @var SYS_T::GPC_HR
 * Offset: 0x68  PC.15 ~ PC.0 Pull Resistance Select Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PUHR0     |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[1]     |PUHR1     |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[2]     |PUHR2     |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[3]     |PUHR3     |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[4]     |PUHR4     |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[5]     |PUHR5     |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[6]     |PUHR6     |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[7]     |PUHR7     |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[8]     |PUHR8     |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[9]     |PUHR9     |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[10]    |PUHR10    |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[11]    |PUHR11    |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[12]    |PUHR12    |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[13]    |PUHR13    |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[14]    |PUHR14    |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[15]    |PUHR15    |PC.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * @var SYS_T::GPC_IEN
 * Offset: 0x6C  PC.15 ~ PC.0 Digital Input Buffer Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IEN       |PC.n Digital Input Buffer Control Register. n = 15~0
 * |        |          |0 = Input buffer Enabled.
 * |        |          |1 = Input buffer disabled, and input signal always equals to 0.
 * @var SYS_T::GPD_PULL
 * Offset: 0x74  PD.15 ~ PD.0 Pull Resistance Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PUEN0     |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[1]     |PUEN1     |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[2]     |PUEN2     |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[3]     |PUEN3     |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[4]     |PUEN4     |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[5]     |PUEN5     |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[6]     |PUEN6     |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[7]     |PUEN7     |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[8]     |PUEN8     |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[9]     |PUEN9     |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[10]    |PUEN10    |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[11]    |PUEN11    |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[12]    |PUEN12    |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[13]    |PUEN13    |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[14]    |PUEN14    |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[15]    |PUEN15    |PD.n Pull Control Register. n = 15~0
 * |        |          |1 = Pull-Up function Enable
 * |        |          |0 = Pull-Up function Disable.
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * @var SYS_T::GPD_HR
 * Offset: 0x78  PD.15 ~ PD.0 Pull Resistance Select Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PUHR0     |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[1]     |PUHR1     |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[2]     |PUHR2     |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[3]     |PUHR3     |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[4]     |PUHR4     |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[5]     |PUHR5     |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[6]     |PUHR6     |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[7]     |PUHR7     |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[8]     |PUHR8     |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[9]     |PUHR9     |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[10]    |PUHR10    |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[11]    |PUHR11    |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[12]    |PUHR12    |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[13]    |PUHR13    |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[14]    |PUHR14    |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * |[15]    |PUHR15    |PD.n Pull Resistance Select Control Register. n = 15~0
 * |        |          |1 = Pull-Up 1M resistance
 * |        |          |0 = Pull-Up 100K resistance
 * |        |          |This function only for the GPIO Px[n] pin as an INPUT mode.
 * @var SYS_T::GPD_IEN
 * Offset: 0x7C  PD.15 ~ PD.0 Digital Input Buffer Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |IEN       |PD.n Digital Input Buffer Control Register. n = 15~0
 * |        |          |0 = Input buffer Enabled.
 * |        |          |1 = Input buffer disabled, and input signal always equals to 0.
 * @var SYS_T::SRAM_BISTCTL
 * Offset: 0xD0  System SRAM BIST Test Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SRAMBIST  |SRAM BIST Enable Bit (Write Protect)
 * |        |          |This bit enables BIST test for SRAM.
 * |        |          |0 = system SRAM BIST Disabled.
 * |        |          |1 = system SRAM BIST Enabled.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[2]     |CACHEBIST |CACHE SRAM BIST Enable Bit (Write Protect)
 * |        |          |This bit enables BIST test for CACHE SRAM.
 * |        |          |0 = CACHE SRAM BIST Disabled.
 * |        |          |1 = CACHE SRAM BIST Enabled.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * @var SYS_T::SRAM_BISTSTS
 * Offset: 0xD4  System SRAM BIST Test Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SRAMBISTEF|System SRAM BIST Fail Flag
 * |        |          |0 = system SRAM BIST test pass.
 * |        |          |1 = system SRAM BIST test fail.
 * |[2]     |CACHEBISTEF|CACHE SRAM BIST Fail Flag
 * |        |          |0 = CACHE SRAM BIST test pass.
 * |        |          |1 = CACHE SRAM BIST test fail.
 * |[16]    |SRAMBEND  |System SRAM BIST Test Finish
 * |        |          |0 = system SRAM BIST active.
 * |        |          |1 = system SRAM BIST finish. 
 * |[18]    |CACHEBEND |CACHE SRAM BIST Test Finish
 * |        |          |0 = CACHE SRAM BIST is active.
 * |        |          |1 = CACHE SRAM BIST finish.
 * @var SYS_T::IMGMAP3
 * Offset: 0xF0  MAP3 Data Image Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |IMG3      |Data Image of MAP3
 * |        |          |Data in MAP3 of information block are copied to this register after power on.
 * @var SYS_T::DEVICEID
 * Offset: 0xF4  Device ID Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |DEVICEID  |Device ID Data
 * |        |          |This register provides specific read-only information for the Device ID
 * @var SYS_T::IMGMAP0
 * Offset: 0xF8  MAP0 Data Image Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |IMG0      |Data Image of MAP0
 * |        |          |Data in MAP0 of information block are copied to this register after power on.
 * @var SYS_T::IMGMAP1
 * Offset: 0xFC  MAP1 Data Image Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |IMG1      |Data Image of MAP1
 * |        |          |Data in MAP1 of information block are copied to this register after power on.
 * @var SYS_T::REGLCTL
 * Offset: 0x100  Register Lock Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |SYS_REGLCTL_REGLCTL|Register Lock Control Code (Write Only)
 * |        |          |Some registers have write-protection function
 * |        |          |Writing these registers have to disable the protected function by writing the sequence value u201C59hu201D, u201C16hu201D, u201C88hu201D to this field
 * |        |          |After this sequence is completed, the REGLCTL bit will be set to 1 and write-protection registers can be normal write.
 * |        |          |Protected Register Lock/Unlock Index (Read Only)
 * |        |          |0 = Protected registers are locked. Any write to the target register is ignored.
 * |        |          |1 = Protected registers are unlocked.
 * @var SYS_T::OSCTRIM
 * Offset: 0x110  Internal Oscillator Trim Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[9:0]   |TRIM      |10 bit trim for oscillator,
 * |[15]    |EN2MHZ    |01: High frequency mode (20-50 MHz)
 * |        |          |10: Low Frequency mode of oscillator active (2 MHz).
 * @var SYS_T::OSC10K
 * Offset: 0x114  10KHz Oscillator and Bias Trim Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[22:0]  |OSC10K_TRIM|23bit trim for 10 KHz oscillator.
 * |        |          |[7:0] = OSC10K TC Trim value
 * |        |          |[12:8] = OSC10K SLDO Trim value
 * |        |          |[22:13] = OSC10K RC Frequency Trim value
 * |[27:24] |TM_REG    |Analog test modes
 * |        |          |Bit25 for analog PGA output to PB6 enable
 * |[31]    |TRM_CLK   |Must be toggled to load a new OSC10K_TRIM
 * @var SYS_T::OSC_TRIM0
 * Offset: 0x118  Oscillator Frequency Adjustment Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |TRIM      |16bit sign extended representation of 10bit trim.
 * |        |          |SYS_OSC_TRIM[0n] load from factory trim value after reset.
 * |        |          |One of SYS_OSC_TRIM[n] will map to SYS_OSCTRIM base on OSCFSEL maps to above-mentiond OSCTRIM.
 * |        |          |SYS_OSC_TRIM[1] & SYS_OSC_TRIM[2] are reserved.
 * |[20:16] |TC        |Temperature compensation setting. Set by factory
 * |[31]    |EN2MHZ    |01: High frequency mode (20-50 MHz)
 * |        |          |10: Low Frequency mode of oscillator active (2 MHz).
 * @var SYS_T::OSC_TRIM1
 * Offset: 0x11C  Oscillator Frequency Adjustment Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |TRIM      |16bit sign extended representation of 10bit trim.
 * |        |          |SYS_OSC_TRIM[0n] load from factory trim value after reset.
 * |        |          |One of SYS_OSC_TRIM[n] will map to SYS_OSCTRIM base on OSCFSEL maps to above-mentiond OSCTRIM.
 * |        |          |SYS_OSC_TRIM[1] & SYS_OSC_TRIM[2] are reserved.
 * |[20:16] |TC        |Temperature compensation setting. Set by factory
 * |[31]    |EN2MHZ    |01: High frequency mode (20-50 MHz)
 * |        |          |10: Low Frequency mode of oscillator active (2 MHz).
 * @var SYS_T::OSC_TRIM2
 * Offset: 0x120  Oscillator Frequency Adjustment Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |TRIM      |16bit sign extended representation of 10bit trim.
 * |        |          |SYS_OSC_TRIM[0n] load from factory trim value after reset.
 * |        |          |One of SYS_OSC_TRIM[n] will map to SYS_OSCTRIM base on OSCFSEL maps to above-mentiond OSCTRIM.
 * |        |          |SYS_OSC_TRIM[1] & SYS_OSC_TRIM[2] are reserved.
 * |[20:16] |TC        |Temperature compensation setting. Set by factory
 * |[31]    |EN2MHZ    |01: High frequency mode (20-50 MHz)
 * |        |          |10: Low Frequency mode of oscillator active (2 MHz).
 * @var SYS_T::IRCTCTL
 * Offset: 0x130  HIRC Trim Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |FREQSEL   |Trim Frequency Selection
 * |        |          |This field indicates the target frequency of 49.152 MHz internal high speed RC oscillator (HIRC) auto trim.
 * |        |          |During auto trim operation, if clock error detected with CESTOPEN is set to 1 or trim retry limitation count reached, this field will be cleared to 00 automatically.
 * |        |          |00 = Disable HIRC auto trim function.
 * |        |          |01 = Enable HIRC auto trim function and trim HIRC to 48 MHz.
 * |        |          |10 = Disable HIRC auto trim function.
 * |        |          |11 = Enable HIRC auto trim function and trim HIRC to 49.152 MHz.
 * |[5:4]   |LOOPSEL   |Trim Calculation Loop Selection
 * |        |          |This field defines that trim value calculation is based on how many internal reference clocks.
 * |        |          |00 = Trim value calculation is based on average difference in 4 clocks of reference clock.
 * |        |          |01 = Trim value calculation is based on average difference in 8 clocks of reference clock.
 * |        |          |10 = Trim value calculation is based on average difference in 16 clocks of reference clock.
 * |        |          |11 = Trim value calculation is based on average difference in 32 clocks of reference clock.
 * |        |          |Note1: For example, if LOOPSEL is set as 00, auto trim circuit will calculate trim value based on the average frequency difference in 4 clocks of reference clock.
 * |        |          |Note2: If source clock from HXT , the internal reference clock is 32 KHz
 * |        |          | If source clock from SOF , the internal reference clock is 1 KHz.
 * |[7:6]   |RETRYCNT  |Trim Value Update Limitation Count
 * |        |          |This field defines that how many times the auto trim circuit will try to update the HIRC trim value before the frequency of HIRC locked.
 * |        |          |Once the HIRC locked, the internal trim value update counter will be reset.
 * |        |          |If the trim value update counter reached this limitation value and frequency of HIRC still doesnu2019t lock, the auto trim operation will be disabled and FREQSEL will be cleared to 00.
 * |        |          |00 = Trim retry count limitation is 64 loops.
 * |        |          |01 = Trim retry count limitation is 128 loops.
 * |        |          |10 = Trim retry count limitation is 256 loops.
 * |        |          |11 = Trim retry count limitation is 512 loops.
 * |[8]     |CESTOPEN  |Clock Error Stop Enable Bit
 * |        |          |0 = The trim operation is keep going if clock is inaccuracy.
 * |        |          |1 = The trim operation is stopped if clock is inaccuracy.
 * |[10]    |REFCKSEL  |Reference Clock Selection
 * |        |          |0 = HIRC trim reference clock is from HXT (4~24.576 MHz) .
 * |        |          |1 = HIRC trim reference clock USB SOF (Start-Of-Frame) packet.
 * |        |          |Note: HIRC trim reference clock is 20K Hz in test mode
 * |[11]    |IGNORE    |Ignore HIRC Unstable Period Selection
 * |        |          |0 = Enable function of ignoring the counting cycles in HIRC unstable period.
 * |        |          |1 = Disable function of ignoring the counting cycles in HIRC unstable period.
 * |        |          |Note: For the current version of HIRC, its clock frequency will shift when trim bits change from 0 to 1 or 1 to 0
 * |        |          |To solve this problem, RC_TRIM ignore the counting clock of unstable HIRC clock period to prevent trim bit Inaccuracies.
 * @var SYS_T::IRCTIEN
 * Offset: 0x134  HIRC Trim Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |TFAILIEN  |Trim Failure Interrupt Enable Bit
 * |        |          |This bit controls if an interrupt will be triggered while HIRC trim value update limitation count reached and HIRC frequency still not locked on target frequency set by FREQSEL (SYS_IRCTCTL [1:0]).
 * |        |          |If this bit is high and TFAILIF (SYS_IRCTSTS [1]) is set during auto trim operation, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached.
 * |        |          |0 = Disable TFAILIF (SYS_IRCTSTS [1]) status to trigger an interrupt to CPU.
 * |        |          |1 = Enable TFAILIF (SYS_IRCTSTS [1]) status to trigger an interrupt to CPU.
 * |[2]     |CLKEIEN   |Clock Error Interrupt Enable Bit
 * |        |          |This bit controls if CPU would get an interrupt while clock is inaccuracy during auto trim operation.
 * |        |          |If this bit is set to1, and CLKERRIF (SYS_IRCTSTS [2]) is set during auto trim operation, an interrupt will be triggered to notify the clock frequency is inaccuracy.
 * |        |          |0 = Disable CLKERRIF (SYS_IRCTSTS [2]) status to trigger an interrupt to CPU.
 * |        |          |1 = Enable CLKERRIF (SYS_IRCTSTS [2]) status to trigger an interrupt to CPU.
 * @var SYS_T::IRCTISTS
 * Offset: 0x138  HIRC Trim Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FREQLOCK  |HIRC Frequency Lock Status
 * |        |          |This bit indicates the HIRC frequency is locked.
 * |        |          |This is a status bit and doesnu2019t trigger any interrupt
 * |        |          |Write 1 to clear this to 0
 * |        |          |This bit will be set automatically, if the frequecy is lock and the RC_TRIM is enabled.
 * |        |          |0 = The internal high-speed oscillator frequency doesnu2019t lock at 49.152 MHztarget frequency yet.
 * |        |          |1 = The internal high-speed oscillator frequency locked at target frequency49.152 MHz.
 * |[1]     |TFAILIF   |Trim Failure Interrupt Status
 * |        |          |This bit indicates that HIRC trim value update limitation count reached and the HIRC clock frequency still doesnu2019t be locked
 * |        |          |Once this bit is set, the auto trim operation stopped and FREQSEL (SYS_IRCTCTL [1:0]) will be cleared to 00 by hardware automatically.
 * |        |          |If this bit is set and TFAILIEN (SYS_IRCTIEN [1]) is high, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached
 * |        |          |Write 1 to clear this to 0.
 * |        |          |0 = Trim value update limitation count does not reach.
 * |        |          |1 = Trim value update limitation count reached and HIRC frequency still not locked.
 * |[2]     |CLKERRIF  |Clock Error Interrupt Status
 * |        |          |When the frequency of external high speed crystal oscillator (HXT) or internal high speed RC oscillator (HIRC) is shift larger to unreasonable value, this bit will be set and to be an indicate that clock frequency is inaccuracy
 * |        |          |Once this bit is set to 1, the auto trim operation stopped and FREQSEL (SYS_IRCTCL [1:0]) will be cleared to 00 by hardware automatically if CESTOPEN (SYS_IRCTCTL [8]) is set to 1.
 * |        |          |If this bit is set and CLKEIEN (SYS_IRCTIEN [2]) is high, an interrupt will be triggered to notify the clock frequency is inaccuracy
 * |        |          |Write 1 to clear this to 0.
 * |        |          |0 = Clock frequency is accuracy.
 * |        |          |1 = Clock frequency is inaccuracy.
 * @var SYS_T::IRCTCKRF
 * Offset: 0x13C  HIRC Trim Clock Reference Frequency Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[14:0]  |HXTFREQ   |HIRC Trim reference clock frequency value when reference clock from HXT
 * |        |          |User can insert the HXT frequency on PCB to this register for internal trim
 * |        |          |The insert frequency value is unit KHz.
 * |        |          |For example:
 * |        |          |If HXT = 4 MHz(4000KHz), register value = 0xFA0.
 * |        |          |If HXT = 12 MHz(12000KHz), register value = 0x2EE0.
 * |        |          |If HXT = 24.576 MHz(24576KHz), register value = 0x6000.
 * |        |          |Note1: The HXT frequency register should set correct value before HIRC auto trim enable.
 * |        |          |Note2: It recommends the HXT should be multiple of 4MHz or 4.096MHz. 
 * @var SYS_T::BGAPTRIM
 * Offset: 0x140  Bandgap Trim Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |TRIM      |4 bit trim for Bandgap.
 * |[7]     |TM        |Bandgap test modes
 * |        |          |Bandgap output to IO(TBD) enable
 * |        |          |0 = Disable
 * |        |          |1 = Enable
 * @var SYS_T::UCIDn
 * Offset: 0x150  Specified ID Register for Library and Customized Feature Checking
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |UCID      |Uniq Customer UCID Data Value
 * |        |          |This register provides specific read-only information for the Uniq Customer IDUCID
 * @var SYS_T::FPGADAT
 * Offset: 0x1F0  FPGA Date Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |DATE      |FPGA Date register
 * |        |          |This register provides the FPGA date
 * @var SYS_T::FPGAVER
 * Offset: 0x1F4  FPGA Version Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |VERSION   |FPGA Version register
 * |        |          |This register provides the FPGA version
 */
    __I  uint32_t PDID;                  /*!< [0x0000] Product Identifier Register                                      */
    __IO uint32_t RSTSTS;                /*!< [0x0004] System Reset Source Register                                     */
    __IO uint32_t IPRST0;                /*!< [0x0008] IP Reset Control Resister0                                       */
    __IO uint32_t IPRST1;                /*!< [0x000c] IP Reset Control Resister1                                       */
    __I  uint32_t RESERVE0[2];
    __IO uint32_t BODCTL;                /*!< [0x0018] Brown-out Detector Control Register                              */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t GPA_MFP;               /*!< [0x0020] GPIO PA Multiple Alternate Functions and Input Type Control Register */
    __IO uint32_t GPB_MFP;               /*!< [0x0024] GPIO PB Multiple Alternate Functions and Input Type Control Register */
    __IO uint32_t GPC_MFP;               /*!< [0x0028] GPIO PC Multiple Alternate Functions and Input Type Control Register */
    __IO uint32_t GPD_MFP;               /*!< [0x002c] GPIO PD Multiple Alternate Functions and Input Type Control Register */
    __I  uint32_t RESERVE2[4];
    __IO uint32_t GPIO_INTP;             /*!< [0x0040] GPIO Input Type and Slew Rate Control                            */
    __IO uint32_t GPA_PULL;              /*!< [0x0044] PA.15 ~ PA.0 Pull Resistance Control Register                    */
    __IO uint32_t GPA_HR;                /*!< [0x0048] PA.15 ~ PA.0 Pull Resistance Select Control Register             */
    __IO uint32_t GPA_IEN;               /*!< [0x004c] PA.15 ~ PA.0 Digital and Analog Input Buffer Control Register    */
    __I  uint32_t RESERVE3[1];
    __IO uint32_t GPB_PULL;              /*!< [0x0054] PB.1 ~ PB.0 Pull Resistance Control Register                     */
    __IO uint32_t GPB_HR;                /*!< [0x0058] PB.1 ~ PB.0 Pull Resistance Select Control Register              */
    __IO uint32_t GPB_IEN;               /*!< [0x005c] PB.1 ~ PB.0 Digital Input Buffer Control Register                */
    __I  uint32_t RESERVE4[1];
    __IO uint32_t GPC_PULL;              /*!< [0x0064] PC.15 ~ PC.0 Pull Resistance Control Register                    */
    __IO uint32_t GPC_HR;                /*!< [0x0068] PC.15 ~ PC.0 Pull Resistance Select Control Register             */
    __IO uint32_t GPC_IEN;               /*!< [0x006c] PC.15 ~ PC.0 Digital Input Buffer Control Register               */
    __I  uint32_t RESERVE5[1];
    __IO uint32_t GPD_PULL;              /*!< [0x0074] PD.15 ~ PD.0 Pull Resistance Control Register                    */
    __IO uint32_t GPD_HR;                /*!< [0x0078] PD.15 ~ PD.0 Pull Resistance Select Control Register             */
    __IO uint32_t GPD_IEN;               /*!< [0x007c] PD.15 ~ PD.0 Digital Input Buffer Control Register               */
    __I  uint32_t RESERVE6[28];
    __I  uint32_t IMGMAP3;               /*!< [0x00f0] MAP3 Data Image Register                                         */
    __I  uint32_t RESERVE7[3];
    __O  uint32_t REGLCTL;               /*!< [0x0100] Register Lock Control Register                                   */
    __I  uint32_t RESERVE8[3];
    __IO uint32_t OSCTRIM;               /*!< [0x0110] Internal Oscillator Trim Register                                */
    __IO uint32_t OSC10K;                /*!< [0x0114] 10KHz Oscillator and Bias Trim Register                          */
    __IO uint32_t OSC_TRIM[3];           /*!< [0x0118] Oscillator Frequency Adjustment Control Register                 */
    __I  uint32_t RESERVE9[3];
    __IO uint32_t IRCTCTL;               /*!< [0x0130] HIRC Trim Control Register                                       */
    __IO uint32_t IRCTIEN;               /*!< [0x0134] HIRC Trim Interrupt Enable Register                              */
    __IO uint32_t IRCTISTS;              /*!< [0x0138] HIRC Trim Interrupt Status Register                              */
    __IO uint32_t IRCTCKRF;              /*!< [0x013c] HIRC Trim Clock Reference Frequency Register                     */
    __IO uint32_t BGAPTRIM;              /*!< [0x0140] Bandgap Trim Control Register                                    */
    __I  uint32_t RESERVE10[3];
    __I  uint32_t UCIDn;                 /*!< [0x0150] Specified ID Register for Library and Customized Feature Checking */        
	
} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */

#define SYS_PDID_IMG2_Pos                (0)                                               /*!< SYS_T::PDID: IMG2 Position             */
#define SYS_PDID_IMG2_Msk                (0xfffffffful << SYS_PDID_IMG2_Pos)               /*!< SYS_T::PDID: IMG2 Mask                 */

#define SYS_RSTSTS_PORF_Pos              (0)                                               /*!< SYS_T::RSTSTS: PORF Position           */
#define SYS_RSTSTS_PORF_Msk              (0x1ul << SYS_RSTSTS_PORF_Pos)                    /*!< SYS_T::RSTSTS: PORF Mask               */

#define SYS_RSTSTS_PINRF_Pos             (1)                                               /*!< SYS_T::RSTSTS: PINRF Position          */
#define SYS_RSTSTS_PINRF_Msk             (0x1ul << SYS_RSTSTS_PINRF_Pos)                   /*!< SYS_T::RSTSTS: PINRF Mask              */

#define SYS_RSTSTS_WDTRF_Pos             (2)                                               /*!< SYS_T::RSTSTS: WDTRF Position          */
#define SYS_RSTSTS_WDTRF_Msk             (0x1ul << SYS_RSTSTS_WDTRF_Pos)                   /*!< SYS_T::RSTSTS: WDTRF Mask              */

#define SYS_RSTSTS_LVRF_Pos              (3)                                               /*!< SYS_T::RSTSTS: LVRF Position           */
#define SYS_RSTSTS_LVRF_Msk              (0x1ul << SYS_RSTSTS_LVRF_Pos)                    /*!< SYS_T::RSTSTS: LVRF Mask               */

#define SYS_RSTSTS_BODRF_Pos             (4)                                               /*!< SYS_T::RSTSTS: BODRF Position          */
#define SYS_RSTSTS_BODRF_Msk             (0x1ul << SYS_RSTSTS_BODRF_Pos)                   /*!< SYS_T::RSTSTS: BODRF Mask              */

#define SYS_RSTSTS_PMURSTF_Pos           (6)                                               /*!< SYS_T::RSTSTS: PMURSTF Position        */
#define SYS_RSTSTS_PMURSTF_Msk           (0x1ul << SYS_RSTSTS_PMURSTF_Pos)                 /*!< SYS_T::RSTSTS: PMURSTF Mask            */

#define SYS_RSTSTS_PINWK_Pos             (8)                                               /*!< SYS_T::RSTSTS: PINWK Position          */
#define SYS_RSTSTS_PINWK_Msk             (0x1ul << SYS_RSTSTS_PINWK_Pos)                   /*!< SYS_T::RSTSTS: PINWK Mask              */

#define SYS_RSTSTS_TIMWK_Pos             (9)                                               /*!< SYS_T::RSTSTS: TIMWK Position          */
#define SYS_RSTSTS_TIMWK_Msk             (0x1ul << SYS_RSTSTS_TIMWK_Pos)                   /*!< SYS_T::RSTSTS: TIMWK Mask              */

#define SYS_RSTSTS_PORWK_Pos             (10)                                              /*!< SYS_T::RSTSTS: PORWK Position          */
#define SYS_RSTSTS_PORWK_Msk             (0x1ul << SYS_RSTSTS_PORWK_Pos)                   /*!< SYS_T::RSTSTS: PORWK Mask              */

#define SYS_IPRST0_CHIPRST_Pos           (0)                                               /*!< SYS_T::IPRST0: CHIPRST Position        */
#define SYS_IPRST0_CHIPRST_Msk           (0x1ul << SYS_IPRST0_CHIPRST_Pos)                 /*!< SYS_T::IPRST0: CHIPRST Mask            */

#define SYS_IPRST0_CPURST_Pos            (1)                                               /*!< SYS_T::IPRST0: CPURST Position         */
#define SYS_IPRST0_CPURST_Msk            (0x1ul << SYS_IPRST0_CPURST_Pos)                  /*!< SYS_T::IPRST0: CPURST Mask             */

#define SYS_IPRST1_GPIORST_Pos           (1)                                               /*!< SYS_T::IPRST1: GPIORST Position        */
#define SYS_IPRST1_GPIORST_Msk           (0x1ul << SYS_IPRST1_GPIORST_Pos)                 /*!< SYS_T::IPRST1: GPIORST Mask            */

#define SYS_IPRST1_TMR0RST_Pos           (2)                                               /*!< SYS_T::IPRST1: TMR0RST Position        */
#define SYS_IPRST1_TMR0RST_Msk           (0x1ul << SYS_IPRST1_TMR0RST_Pos)                 /*!< SYS_T::IPRST1: TMR0RST Mask            */

#define SYS_IPRST1_TMR1RST_Pos           (3)                                               /*!< SYS_T::IPRST1: TMR1RST Position        */
#define SYS_IPRST1_TMR1RST_Msk           (0x1ul << SYS_IPRST1_TMR1RST_Pos)                 /*!< SYS_T::IPRST1: TMR1RST Mask            */

#define SYS_IPRST1_TMR2RST_Pos           (4)                                               /*!< SYS_T::IPRST1: TMR2RST Position        */
#define SYS_IPRST1_TMR2RST_Msk           (0x1ul << SYS_IPRST1_TMR2RST_Pos)                 /*!< SYS_T::IPRST1: TMR2RST Mask            */

#define SYS_IPRST1_CPDRST_Pos            (6)                                               /*!< SYS_T::IPRST1: CPDRST Position         */
#define SYS_IPRST1_CPDRST_Msk            (0x1ul << SYS_IPRST1_CPDRST_Pos)                  /*!< SYS_T::IPRST1: CPDRST Mask             */

#define SYS_IPRST1_PDMARST_Pos           (7)                                               /*!< SYS_T::IPRST1: PDMARST Position        */
#define SYS_IPRST1_PDMARST_Msk           (0x1ul << SYS_IPRST1_PDMARST_Pos)                 /*!< SYS_T::IPRST1: PDMARST Mask            */

#define SYS_IPRST1_I2C0RST_Pos           (8)                                               /*!< SYS_T::IPRST1: I2C0RST Position        */
#define SYS_IPRST1_I2C0RST_Msk           (0x1ul << SYS_IPRST1_I2C0RST_Pos)                 /*!< SYS_T::IPRST1: I2C0RST Mask            */

#define SYS_IPRST1_I2C1RST_Pos           (9)                                               /*!< SYS_T::IPRST1: I2C1RST Position        */
#define SYS_IPRST1_I2C1RST_Msk           (0x1ul << SYS_IPRST1_I2C1RST_Pos)                 /*!< SYS_T::IPRST1: I2C1RST Mask            */

#define SYS_IPRST1_SPI1RST_Pos           (11)                                              /*!< SYS_T::IPRST1: SPI1RST Position        */
#define SYS_IPRST1_SPI1RST_Msk           (0x1ul << SYS_IPRST1_SPI1RST_Pos)                 /*!< SYS_T::IPRST1: SPI1RST Mask            */

#define SYS_IPRST1_SPI0RST_Pos           (12)                                              /*!< SYS_T::IPRST1: SPI0RST Position        */
#define SYS_IPRST1_SPI0RST_Msk           (0x1ul << SYS_IPRST1_SPI0RST_Pos)                 /*!< SYS_T::IPRST1: SPI0RST Mask            */

#define SYS_IPRST1_I2S0RST_Pos           (13)                                              /*!< SYS_T::IPRST1: I2SRST Position         */
#define SYS_IPRST1_I2S0RST_Msk           (0x1ul << SYS_IPRST1_I2S0RST_Pos)                  /*!< SYS_T::IPRST1: I2SRST Mask             */

#define SYS_IPRST1_UART0RST_Pos          (16)                                              /*!< SYS_T::IPRST1: UART0RST Position       */
#define SYS_IPRST1_UART0RST_Msk          (0x1ul << SYS_IPRST1_UART0RST_Pos)                /*!< SYS_T::IPRST1: UART0RST Mask           */

#define SYS_IPRST1_UART1RST_Pos          (17)                                              /*!< SYS_T::IPRST1: UART1RST Position       */
#define SYS_IPRST1_UART1RST_Msk          (0x1ul << SYS_IPRST1_UART1RST_Pos)                /*!< SYS_T::IPRST1: UART1RST Mask           */

#define SYS_IPRST1_BIQRST_Pos            (18)                                              /*!< SYS_T::IPRST1: BIQRST Position         */
#define SYS_IPRST1_BIQRST_Msk            (0x1ul << SYS_IPRST1_BIQRST_Pos)                  /*!< SYS_T::IPRST1: BIQRST Mask             */

#define SYS_IPRST1_PWM0RST_Pos           (20)                                              /*!< SYS_T::IPRST1: PWM0RST Position        */
#define SYS_IPRST1_PWM0RST_Msk           (0x1ul << SYS_IPRST1_PWM0RST_Pos)                 /*!< SYS_T::IPRST1: PWM0RST Mask            */

#define SYS_IPRST1_PWM1RST_Pos           (21)                                              /*!< SYS_T::IPRST1: PWM1RST Position        */
#define SYS_IPRST1_PWM1RST_Msk           (0x1ul << SYS_IPRST1_PWM1RST_Pos)                 /*!< SYS_T::IPRST1: PWM1RST Mask            */

#define SYS_IPRST1_USBRST_Pos            (24)                                              /*!< SYS_T::IPRST1: USBRST Position         */
#define SYS_IPRST1_USBRST_Msk            (0x1ul << SYS_IPRST1_USBRST_Pos)                  /*!< SYS_T::IPRST1: USBRST Mask             */

#define SYS_IPRST1_SARADCRST_Pos         (28)                                              /*!< SYS_T::IPRST1: SARADCRST Position      */
#define SYS_IPRST1_SARADCRST_Msk         (0x1ul << SYS_IPRST1_SARADCRST_Pos)               /*!< SYS_T::IPRST1: SARADCRST Mask          */

#define SYS_IPRST1_DACRST_Pos            (29)                                              /*!< SYS_T::IPRST1: DACRST Position         */
#define SYS_IPRST1_DACRST_Msk            (0x1ul << SYS_IPRST1_DACRST_Pos)                  /*!< SYS_T::IPRST1: DACRST Mask             */

#define SYS_IPRST1_SDADCRST_Pos          (30)                                              /*!< SYS_T::IPRST1: SDADCRST Position       */
#define SYS_IPRST1_SDADCRST_Msk          (0x1ul << SYS_IPRST1_SDADCRST_Pos)                /*!< SYS_T::IPRST1: SDADCRST Mask           */

#define SYS_IPRST1_ANARST_Pos            (31)                                              /*!< SYS_T::IPRST1: ANARST Position         */
#define SYS_IPRST1_ANARST_Msk            (0x1ul << SYS_IPRST1_ANARST_Pos)                  /*!< SYS_T::IPRST1: ANARST Mask             */

#define SYS_BODCTL_BODEN_Pos             (0)                                               /*!< SYS_T::BODCTL: BODEN Position          */
#define SYS_BODCTL_BODEN_Msk             (0x1ul << SYS_BODCTL_BODEN_Pos)                   /*!< SYS_T::BODCTL: BODEN Mask              */

#define SYS_BODCTL_BODRSTEN_Pos          (1)                                               /*!< SYS_T::BODCTL: BODRSTEN Position       */
#define SYS_BODCTL_BODRSTEN_Msk          (0x1ul << SYS_BODCTL_BODRSTEN_Pos)                /*!< SYS_T::BODCTL: BODRSTEN Mask           */

#define SYS_BODCTL_BODLVL_Pos            (2)                                               /*!< SYS_T::BODCTL: BODLVL Position         */
#define SYS_BODCTL_BODLVL_Msk            (0xful << SYS_BODCTL_BODLVL_Pos)                  /*!< SYS_T::BODCTL: BODLVL Mask             */

#define SYS_BODCTL_BODHYS_Pos            (6)                                               /*!< SYS_T::BODCTL: BODHYS Position         */
#define SYS_BODCTL_BODHYS_Msk            (0x1ul << SYS_BODCTL_BODHYS_Pos)                  /*!< SYS_T::BODCTL: BODHYS Mask             */

#define SYS_BODCTL_BODOUT_Pos            (7)                                               /*!< SYS_T::BODCTL: BODOUT Position         */
#define SYS_BODCTL_BODOUT_Msk            (0x1ul << SYS_BODCTL_BODOUT_Pos)                  /*!< SYS_T::BODCTL: BODOUT Mask             */

#define SYS_BODCTL_BODINT_Pos            (8)                                               /*!< SYS_T::BODCTL: BODINT Position         */
#define SYS_BODCTL_BODINT_Msk            (0x1ul << SYS_BODCTL_BODINT_Pos)                  /*!< SYS_T::BODCTL: BODINT Mask             */

#define SYS_BODCTL_LVREN_Pos             (16)                                              /*!< SYS_T::BODCTL: LVREN Position          */
#define SYS_BODCTL_LVREN_Msk             (0x1ul << SYS_BODCTL_LVREN_Pos)                   /*!< SYS_T::BODCTL: LVREN Mask              */

#define SYS_BODCTL_LVRFILTER_Pos         (17)                                              /*!< SYS_T::BODCTL: LVRFILTER Position      */
#define SYS_BODCTL_LVRFILTER_Msk         (0x3ul << SYS_BODCTL_LVRFILTER_Pos)               /*!< SYS_T::BODCTL: LVRFILTER Mask          */

#define SYS_GPA_MFP_PA0MFP_Pos           (0)                                               /*!< SYS_T::GPA_MFP: PA0MFP Position        */
#define SYS_GPA_MFP_PA0MFP_Msk           (0x3ul << SYS_GPA_MFP_PA0MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA0MFP Mask            */

#define SYS_GPA_MFP_PA1MFP_Pos           (2)                                               /*!< SYS_T::GPA_MFP: PA1MFP Position        */
#define SYS_GPA_MFP_PA1MFP_Msk           (0x3ul << SYS_GPA_MFP_PA1MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA1MFP Mask            */

#define SYS_GPA_MFP_PA2MFP_Pos           (4)                                               /*!< SYS_T::GPA_MFP: PA2MFP Position        */
#define SYS_GPA_MFP_PA2MFP_Msk           (0x3ul << SYS_GPA_MFP_PA2MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA2MFP Mask            */

#define SYS_GPA_MFP_PA3MFP_Pos           (6)                                               /*!< SYS_T::GPA_MFP: PA3MFP Position        */
#define SYS_GPA_MFP_PA3MFP_Msk           (0x3ul << SYS_GPA_MFP_PA3MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA3MFP Mask            */

#define SYS_GPA_MFP_PA4MFP_Pos           (8)                                               /*!< SYS_T::GPA_MFP: PA4MFP Position        */
#define SYS_GPA_MFP_PA4MFP_Msk           (0x3ul << SYS_GPA_MFP_PA4MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA4MFP Mask            */

#define SYS_GPA_MFP_PA5MFP_Pos           (10)                                              /*!< SYS_T::GPA_MFP: PA5MFP Position        */
#define SYS_GPA_MFP_PA5MFP_Msk           (0x3ul << SYS_GPA_MFP_PA5MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA5MFP Mask            */

#define SYS_GPA_MFP_PA6MFP_Pos           (12)                                              /*!< SYS_T::GPA_MFP: PA6MFP Position        */
#define SYS_GPA_MFP_PA6MFP_Msk           (0x3ul << SYS_GPA_MFP_PA6MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA6MFP Mask            */

#define SYS_GPA_MFP_PA7MFP_Pos           (14)                                              /*!< SYS_T::GPA_MFP: PA7MFP Position        */
#define SYS_GPA_MFP_PA7MFP_Msk           (0x3ul << SYS_GPA_MFP_PA7MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA7MFP Mask            */

#define SYS_GPA_MFP_PA8MFP_Pos           (16)                                              /*!< SYS_T::GPA_MFP: PA8MFP Position        */
#define SYS_GPA_MFP_PA8MFP_Msk           (0x3ul << SYS_GPA_MFP_PA8MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA8MFP Mask            */

#define SYS_GPA_MFP_PA9MFP_Pos           (18)                                              /*!< SYS_T::GPA_MFP: PA9MFP Position        */
#define SYS_GPA_MFP_PA9MFP_Msk           (0x3ul << SYS_GPA_MFP_PA9MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA9MFP Mask            */

#define SYS_GPA_MFP_PA10MFP_Pos          (20)                                              /*!< SYS_T::GPA_MFP: PA10MFP Position       */
#define SYS_GPA_MFP_PA10MFP_Msk          (0x3ul << SYS_GPA_MFP_PA10MFP_Pos)                /*!< SYS_T::GPA_MFP: PA10MFP Mask           */

#define SYS_GPA_MFP_PA11MFP_Pos          (22)                                              /*!< SYS_T::GPA_MFP: PA11MFP Position       */
#define SYS_GPA_MFP_PA11MFP_Msk          (0x3ul << SYS_GPA_MFP_PA11MFP_Pos)                /*!< SYS_T::GPA_MFP: PA11MFP Mask           */

#define SYS_GPA_MFP_PA12MFP_Pos          (24)                                              /*!< SYS_T::GPA_MFP: PA12MFP Position       */
#define SYS_GPA_MFP_PA12MFP_Msk          (0x3ul << SYS_GPA_MFP_PA12MFP_Pos)                /*!< SYS_T::GPA_MFP: PA12MFP Mask           */

#define SYS_GPA_MFP_PA13MFP_Pos          (26)                                              /*!< SYS_T::GPA_MFP: PA13MFP Position       */
#define SYS_GPA_MFP_PA13MFP_Msk          (0x3ul << SYS_GPA_MFP_PA13MFP_Pos)                /*!< SYS_T::GPA_MFP: PA13MFP Mask           */

#define SYS_GPA_MFP_PA14MFP_Pos          (28)                                              /*!< SYS_T::GPA_MFP: PA14MFP Position       */
#define SYS_GPA_MFP_PA14MFP_Msk          (0x3ul << SYS_GPA_MFP_PA14MFP_Pos)                /*!< SYS_T::GPA_MFP: PA14MFP Mask           */

#define SYS_GPA_MFP_PA15MFP_Pos          (30)                                              /*!< SYS_T::GPA_MFP: PA15MFP Position       */
#define SYS_GPA_MFP_PA15MFP_Msk          (0x3ul << SYS_GPA_MFP_PA15MFP_Pos)                /*!< SYS_T::GPA_MFP: PA15MFP Mask           */

#define SYS_GPB_MFP_PB0MFP_Pos           (0)                                               /*!< SYS_T::GPB_MFP: PB0MFP Position        */
#define SYS_GPB_MFP_PB0MFP_Msk           (0x3ul << SYS_GPB_MFP_PB0MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB0MFP Mask            */

#define SYS_GPB_MFP_PB1MFP_Pos           (2)                                               /*!< SYS_T::GPB_MFP: PB1MFP Position        */
#define SYS_GPB_MFP_PB1MFP_Msk           (0x3ul << SYS_GPB_MFP_PB1MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB1MFP Mask            */

#define SYS_GPC_MFP_PC0MFP_Pos           (0)                                               /*!< SYS_T::GPC_MFP: PC0MFP Position        */
#define SYS_GPC_MFP_PC0MFP_Msk           (0x3ul << SYS_GPC_MFP_PC0MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC0MFP Mask            */

#define SYS_GPC_MFP_PC1MFP_Pos           (2)                                               /*!< SYS_T::GPC_MFP: PC1MFP Position        */
#define SYS_GPC_MFP_PC1MFP_Msk           (0x3ul << SYS_GPC_MFP_PC1MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC1MFP Mask            */

#define SYS_GPC_MFP_PC2MFP_Pos           (4)                                               /*!< SYS_T::GPC_MFP: PC2MFP Position        */
#define SYS_GPC_MFP_PC2MFP_Msk           (0x3ul << SYS_GPC_MFP_PC2MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC2MFP Mask            */

#define SYS_GPC_MFP_PC3MFP_Pos           (6)                                               /*!< SYS_T::GPC_MFP: PC3MFP Position        */
#define SYS_GPC_MFP_PC3MFP_Msk           (0x3ul << SYS_GPC_MFP_PC3MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC3MFP Mask            */

#define SYS_GPC_MFP_PC4MFP_Pos           (8)                                               /*!< SYS_T::GPC_MFP: PC4MFP Position        */
#define SYS_GPC_MFP_PC4MFP_Msk           (0x3ul << SYS_GPC_MFP_PC4MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC4MFP Mask            */

#define SYS_GPC_MFP_PC5MFP_Pos           (10)                                              /*!< SYS_T::GPC_MFP: PC5MFP Position        */
#define SYS_GPC_MFP_PC5MFP_Msk           (0x3ul << SYS_GPC_MFP_PC5MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC5MFP Mask            */

#define SYS_GPC_MFP_PC6MFP_Pos           (12)                                              /*!< SYS_T::GPC_MFP: PC6MFP Position        */
#define SYS_GPC_MFP_PC6MFP_Msk           (0x3ul << SYS_GPC_MFP_PC6MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC6MFP Mask            */

#define SYS_GPC_MFP_PC7MFP_Pos           (14)                                              /*!< SYS_T::GPC_MFP: PC7MFP Position        */
#define SYS_GPC_MFP_PC7MFP_Msk           (0x3ul << SYS_GPC_MFP_PC7MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC7MFP Mask            */

#define SYS_GPC_MFP_PC8MFP_Pos           (16)                                              /*!< SYS_T::GPC_MFP: PC8MFP Position        */
#define SYS_GPC_MFP_PC8MFP_Msk           (0x3ul << SYS_GPC_MFP_PC8MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC8MFP Mask            */

#define SYS_GPC_MFP_PC9MFP_Pos           (18)                                              /*!< SYS_T::GPC_MFP: PC9MFP Position        */
#define SYS_GPC_MFP_PC9MFP_Msk           (0x3ul << SYS_GPC_MFP_PC9MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC9MFP Mask            */

#define SYS_GPC_MFP_PC10MFP_Pos          (20)                                              /*!< SYS_T::GPC_MFP: PC10MFP Position       */
#define SYS_GPC_MFP_PC10MFP_Msk          (0x3ul << SYS_GPC_MFP_PC10MFP_Pos)                /*!< SYS_T::GPC_MFP: PC10MFP Mask           */

#define SYS_GPC_MFP_PC11MFP_Pos          (22)                                              /*!< SYS_T::GPC_MFP: PC11MFP Position       */
#define SYS_GPC_MFP_PC11MFP_Msk          (0x3ul << SYS_GPC_MFP_PC11MFP_Pos)                /*!< SYS_T::GPC_MFP: PC11MFP Mask           */

#define SYS_GPC_MFP_PC12MFP_Pos          (24)                                              /*!< SYS_T::GPC_MFP: PC12MFP Position       */
#define SYS_GPC_MFP_PC12MFP_Msk          (0x3ul << SYS_GPC_MFP_PC12MFP_Pos)                /*!< SYS_T::GPC_MFP: PC12MFP Mask           */

#define SYS_GPC_MFP_PC13MFP_Pos          (26)                                              /*!< SYS_T::GPC_MFP: PC13MFP Position       */
#define SYS_GPC_MFP_PC13MFP_Msk          (0x3ul << SYS_GPC_MFP_PC13MFP_Pos)                /*!< SYS_T::GPC_MFP: PC13MFP Mask           */

#define SYS_GPC_MFP_PC14MFP_Pos          (28)                                              /*!< SYS_T::GPC_MFP: PC14MFP Position       */
#define SYS_GPC_MFP_PC14MFP_Msk          (0x3ul << SYS_GPC_MFP_PC14MFP_Pos)                /*!< SYS_T::GPC_MFP: PC14MFP Mask           */

#define SYS_GPC_MFP_PC15MFP_Pos          (30)                                              /*!< SYS_T::GPC_MFP: PC15MFP Position       */
#define SYS_GPC_MFP_PC15MFP_Msk          (0x3ul << SYS_GPC_MFP_PC15MFP_Pos)                /*!< SYS_T::GPC_MFP: PC15MFP Mask           */

#define SYS_GPD_MFP_PD0MFP_Pos           (0)                                               /*!< SYS_T::GPD_MFP: PD0MFP Position        */
#define SYS_GPD_MFP_PD0MFP_Msk           (0x3ul << SYS_GPD_MFP_PD0MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD0MFP Mask            */

#define SYS_GPD_MFP_PD1MFP_Pos           (2)                                               /*!< SYS_T::GPD_MFP: PD1MFP Position        */
#define SYS_GPD_MFP_PD1MFP_Msk           (0x3ul << SYS_GPD_MFP_PD1MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD1MFP Mask            */

#define SYS_GPD_MFP_PD2MFP_Pos           (4)                                               /*!< SYS_T::GPD_MFP: PD2MFP Position        */
#define SYS_GPD_MFP_PD2MFP_Msk           (0x3ul << SYS_GPD_MFP_PD2MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD2MFP Mask            */

#define SYS_GPD_MFP_PD3MFP_Pos           (6)                                               /*!< SYS_T::GPD_MFP: PD3MFP Position        */
#define SYS_GPD_MFP_PD3MFP_Msk           (0x3ul << SYS_GPD_MFP_PD3MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD3MFP Mask            */

#define SYS_GPD_MFP_PD4MFP_Pos           (8)                                               /*!< SYS_T::GPD_MFP: PD4MFP Position        */
#define SYS_GPD_MFP_PD4MFP_Msk           (0x3ul << SYS_GPD_MFP_PD4MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD4MFP Mask            */

#define SYS_GPD_MFP_PD5MFP_Pos           (10)                                              /*!< SYS_T::GPD_MFP: PD5MFP Position        */
#define SYS_GPD_MFP_PD5MFP_Msk           (0x3ul << SYS_GPD_MFP_PD5MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD5MFP Mask            */

#define SYS_GPD_MFP_PD6MFP_Pos           (12)                                              /*!< SYS_T::GPD_MFP: PD6MFP Position        */
#define SYS_GPD_MFP_PD6MFP_Msk           (0x3ul << SYS_GPD_MFP_PD6MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD6MFP Mask            */

#define SYS_GPD_MFP_PD7MFP_Pos           (14)                                              /*!< SYS_T::GPD_MFP: PD7MFP Position        */
#define SYS_GPD_MFP_PD7MFP_Msk           (0x3ul << SYS_GPD_MFP_PD7MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD7MFP Mask            */

#define SYS_GPD_MFP_PD8MFP_Pos           (16)                                              /*!< SYS_T::GPD_MFP: PD8MFP Position        */
#define SYS_GPD_MFP_PD8MFP_Msk           (0x3ul << SYS_GPD_MFP_PD8MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD8MFP Mask            */

#define SYS_GPD_MFP_PD9MFP_Pos           (18)                                              /*!< SYS_T::GPD_MFP: PD9MFP Position        */
#define SYS_GPD_MFP_PD9MFP_Msk           (0x3ul << SYS_GPD_MFP_PD9MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD9MFP Mask            */

#define SYS_GPD_MFP_PD10MFP_Pos          (20)                                              /*!< SYS_T::GPD_MFP: PD10MFP Position       */
#define SYS_GPD_MFP_PD10MFP_Msk          (0x3ul << SYS_GPD_MFP_PD10MFP_Pos)                /*!< SYS_T::GPD_MFP: PD10MFP Mask           */

#define SYS_GPD_MFP_PD11MFP_Pos          (22)                                              /*!< SYS_T::GPD_MFP: PD11MFP Position       */
#define SYS_GPD_MFP_PD11MFP_Msk          (0x3ul << SYS_GPD_MFP_PD11MFP_Pos)                /*!< SYS_T::GPD_MFP: PD11MFP Mask           */

#define SYS_GPD_MFP_PD12MFP_Pos          (24)                                              /*!< SYS_T::GPD_MFP: PD12MFP Position       */
#define SYS_GPD_MFP_PD12MFP_Msk          (0x3ul << SYS_GPD_MFP_PD12MFP_Pos)                /*!< SYS_T::GPD_MFP: PD12MFP Mask           */

#define SYS_GPD_MFP_PD13MFP_Pos          (26)                                              /*!< SYS_T::GPD_MFP: PD13MFP Position       */
#define SYS_GPD_MFP_PD13MFP_Msk          (0x3ul << SYS_GPD_MFP_PD13MFP_Pos)                /*!< SYS_T::GPD_MFP: PD13MFP Mask           */

#define SYS_GPD_MFP_PD14MFP_Pos          (28)                                              /*!< SYS_T::GPD_MFP: PD14MFP Position       */
#define SYS_GPD_MFP_PD14MFP_Msk          (0x3ul << SYS_GPD_MFP_PD14MFP_Pos)                /*!< SYS_T::GPD_MFP: PD14MFP Mask           */

#define SYS_GPD_MFP_PD15MFP_Pos          (30)                                              /*!< SYS_T::GPD_MFP: PD15MFP Position       */
#define SYS_GPD_MFP_PD15MFP_Msk          (0x3ul << SYS_GPD_MFP_PD15MFP_Pos)                /*!< SYS_T::GPD_MFP: PD15MFP Mask           */

#define SYS_GPIO_INTP_GPxSSGPxHS_Pos     (0)                                               /*!< SYS_T::GPIO_INTP: GPxSSGPxHS Position  */
#define SYS_GPIO_INTP_GPxSSGPxHS_Msk     (0x3ffful << SYS_GPIO_INTP_GPxSSGPxHS_Pos)        /*!< SYS_T::GPIO_INTP: GPxSSGPxHS Mask      */

#define SYS_GPA_PULL_PUEN0_Pos           (0)                                               /*!< SYS_T::GPA_PULL: PUEN0 Position        */
#define SYS_GPA_PULL_PUEN0_Msk           (0x1ul << SYS_GPA_PULL_PUEN0_Pos)                 /*!< SYS_T::GPA_PULL: PUEN0 Mask            */

#define SYS_GPA_PULL_PUEN1_Pos           (1)                                               /*!< SYS_T::GPA_PULL: PUEN1 Position        */
#define SYS_GPA_PULL_PUEN1_Msk           (0x1ul << SYS_GPA_PULL_PUEN1_Pos)                 /*!< SYS_T::GPA_PULL: PUEN1 Mask            */

#define SYS_GPA_PULL_PUEN2_Pos           (2)                                               /*!< SYS_T::GPA_PULL: PUEN2 Position        */
#define SYS_GPA_PULL_PUEN2_Msk           (0x1ul << SYS_GPA_PULL_PUEN2_Pos)                 /*!< SYS_T::GPA_PULL: PUEN2 Mask            */

#define SYS_GPA_PULL_PUEN3_Pos           (3)                                               /*!< SYS_T::GPA_PULL: PUEN3 Position        */
#define SYS_GPA_PULL_PUEN3_Msk           (0x1ul << SYS_GPA_PULL_PUEN3_Pos)                 /*!< SYS_T::GPA_PULL: PUEN3 Mask            */

#define SYS_GPA_PULL_PUEN4_Pos           (4)                                               /*!< SYS_T::GPA_PULL: PUEN4 Position        */
#define SYS_GPA_PULL_PUEN4_Msk           (0x1ul << SYS_GPA_PULL_PUEN4_Pos)                 /*!< SYS_T::GPA_PULL: PUEN4 Mask            */

#define SYS_GPA_PULL_PUEN5_Pos           (5)                                               /*!< SYS_T::GPA_PULL: PUEN5 Position        */
#define SYS_GPA_PULL_PUEN5_Msk           (0x1ul << SYS_GPA_PULL_PUEN5_Pos)                 /*!< SYS_T::GPA_PULL: PUEN5 Mask            */

#define SYS_GPA_PULL_PUEN6_Pos           (6)                                               /*!< SYS_T::GPA_PULL: PUEN6 Position        */
#define SYS_GPA_PULL_PUEN6_Msk           (0x1ul << SYS_GPA_PULL_PUEN6_Pos)                 /*!< SYS_T::GPA_PULL: PUEN6 Mask            */

#define SYS_GPA_PULL_PUEN7_Pos           (7)                                               /*!< SYS_T::GPA_PULL: PUEN7 Position        */
#define SYS_GPA_PULL_PUEN7_Msk           (0x1ul << SYS_GPA_PULL_PUEN7_Pos)                 /*!< SYS_T::GPA_PULL: PUEN7 Mask            */

#define SYS_GPA_PULL_PUEN8_Pos           (8)                                               /*!< SYS_T::GPA_PULL: PUEN8 Position        */
#define SYS_GPA_PULL_PUEN8_Msk           (0x1ul << SYS_GPA_PULL_PUEN8_Pos)                 /*!< SYS_T::GPA_PULL: PUEN8 Mask            */

#define SYS_GPA_PULL_PUEN9_Pos           (9)                                               /*!< SYS_T::GPA_PULL: PUEN9 Position        */
#define SYS_GPA_PULL_PUEN9_Msk           (0x1ul << SYS_GPA_PULL_PUEN9_Pos)                 /*!< SYS_T::GPA_PULL: PUEN9 Mask            */

#define SYS_GPA_PULL_PUEN10_Pos          (10)                                              /*!< SYS_T::GPA_PULL: PUEN10 Position       */
#define SYS_GPA_PULL_PUEN10_Msk          (0x1ul << SYS_GPA_PULL_PUEN10_Pos)                /*!< SYS_T::GPA_PULL: PUEN10 Mask           */

#define SYS_GPA_PULL_PUEN11_Pos          (11)                                              /*!< SYS_T::GPA_PULL: PUEN11 Position       */
#define SYS_GPA_PULL_PUEN11_Msk          (0x1ul << SYS_GPA_PULL_PUEN11_Pos)                /*!< SYS_T::GPA_PULL: PUEN11 Mask           */

#define SYS_GPA_PULL_PUEN12_Pos          (12)                                              /*!< SYS_T::GPA_PULL: PUEN12 Position       */
#define SYS_GPA_PULL_PUEN12_Msk          (0x1ul << SYS_GPA_PULL_PUEN12_Pos)                /*!< SYS_T::GPA_PULL: PUEN12 Mask           */

#define SYS_GPA_PULL_PUEN13_Pos          (13)                                              /*!< SYS_T::GPA_PULL: PUEN13 Position       */
#define SYS_GPA_PULL_PUEN13_Msk          (0x1ul << SYS_GPA_PULL_PUEN13_Pos)                /*!< SYS_T::GPA_PULL: PUEN13 Mask           */

#define SYS_GPA_PULL_PUEN14_Pos          (14)                                              /*!< SYS_T::GPA_PULL: PUEN14 Position       */
#define SYS_GPA_PULL_PUEN14_Msk          (0x1ul << SYS_GPA_PULL_PUEN14_Pos)                /*!< SYS_T::GPA_PULL: PUEN14 Mask           */

#define SYS_GPA_PULL_PUEN15_Pos          (15)                                              /*!< SYS_T::GPA_PULL: PUEN15 Position       */
#define SYS_GPA_PULL_PUEN15_Msk          (0x1ul << SYS_GPA_PULL_PUEN15_Pos)                /*!< SYS_T::GPA_PULL: PUEN15 Mask           */

#define SYS_GPA_HR_PUHR0_Pos             (0)                                               /*!< SYS_T::GPA_HR: PUHR0 Position          */
#define SYS_GPA_HR_PUHR0_Msk             (0x1ul << SYS_GPA_HR_PUHR0_Pos)                   /*!< SYS_T::GPA_HR: PUHR0 Mask              */

#define SYS_GPA_HR_PUHR1_Pos             (1)                                               /*!< SYS_T::GPA_HR: PUHR1 Position          */
#define SYS_GPA_HR_PUHR1_Msk             (0x1ul << SYS_GPA_HR_PUHR1_Pos)                   /*!< SYS_T::GPA_HR: PUHR1 Mask              */

#define SYS_GPA_HR_PUHR2_Pos             (2)                                               /*!< SYS_T::GPA_HR: PUHR2 Position          */
#define SYS_GPA_HR_PUHR2_Msk             (0x1ul << SYS_GPA_HR_PUHR2_Pos)                   /*!< SYS_T::GPA_HR: PUHR2 Mask              */

#define SYS_GPA_HR_PUHR3_Pos             (3)                                               /*!< SYS_T::GPA_HR: PUHR3 Position          */
#define SYS_GPA_HR_PUHR3_Msk             (0x1ul << SYS_GPA_HR_PUHR3_Pos)                   /*!< SYS_T::GPA_HR: PUHR3 Mask              */

#define SYS_GPA_HR_PUHR4_Pos             (4)                                               /*!< SYS_T::GPA_HR: PUHR4 Position          */
#define SYS_GPA_HR_PUHR4_Msk             (0x1ul << SYS_GPA_HR_PUHR4_Pos)                   /*!< SYS_T::GPA_HR: PUHR4 Mask              */

#define SYS_GPA_HR_PUHR5_Pos             (5)                                               /*!< SYS_T::GPA_HR: PUHR5 Position          */
#define SYS_GPA_HR_PUHR5_Msk             (0x1ul << SYS_GPA_HR_PUHR5_Pos)                   /*!< SYS_T::GPA_HR: PUHR5 Mask              */

#define SYS_GPA_HR_PUHR6_Pos             (6)                                               /*!< SYS_T::GPA_HR: PUHR6 Position          */
#define SYS_GPA_HR_PUHR6_Msk             (0x1ul << SYS_GPA_HR_PUHR6_Pos)                   /*!< SYS_T::GPA_HR: PUHR6 Mask              */

#define SYS_GPA_HR_PUHR7_Pos             (7)                                               /*!< SYS_T::GPA_HR: PUHR7 Position          */
#define SYS_GPA_HR_PUHR7_Msk             (0x1ul << SYS_GPA_HR_PUHR7_Pos)                   /*!< SYS_T::GPA_HR: PUHR7 Mask              */

#define SYS_GPA_HR_PUHR8_Pos             (8)                                               /*!< SYS_T::GPA_HR: PUHR8 Position          */
#define SYS_GPA_HR_PUHR8_Msk             (0x1ul << SYS_GPA_HR_PUHR8_Pos)                   /*!< SYS_T::GPA_HR: PUHR8 Mask              */

#define SYS_GPA_HR_PUHR9_Pos             (9)                                               /*!< SYS_T::GPA_HR: PUHR9 Position          */
#define SYS_GPA_HR_PUHR9_Msk             (0x1ul << SYS_GPA_HR_PUHR9_Pos)                   /*!< SYS_T::GPA_HR: PUHR9 Mask              */

#define SYS_GPA_HR_PUHR10_Pos            (10)                                              /*!< SYS_T::GPA_HR: PUHR10 Position         */
#define SYS_GPA_HR_PUHR10_Msk            (0x1ul << SYS_GPA_HR_PUHR10_Pos)                  /*!< SYS_T::GPA_HR: PUHR10 Mask             */

#define SYS_GPA_HR_PUHR11_Pos            (11)                                              /*!< SYS_T::GPA_HR: PUHR11 Position         */
#define SYS_GPA_HR_PUHR11_Msk            (0x1ul << SYS_GPA_HR_PUHR11_Pos)                  /*!< SYS_T::GPA_HR: PUHR11 Mask             */

#define SYS_GPA_HR_PUHR12_Pos            (12)                                              /*!< SYS_T::GPA_HR: PUHR12 Position         */
#define SYS_GPA_HR_PUHR12_Msk            (0x1ul << SYS_GPA_HR_PUHR12_Pos)                  /*!< SYS_T::GPA_HR: PUHR12 Mask             */

#define SYS_GPA_HR_PUHR13_Pos            (13)                                              /*!< SYS_T::GPA_HR: PUHR13 Position         */
#define SYS_GPA_HR_PUHR13_Msk            (0x1ul << SYS_GPA_HR_PUHR13_Pos)                  /*!< SYS_T::GPA_HR: PUHR13 Mask             */

#define SYS_GPA_HR_PUHR14_Pos            (14)                                              /*!< SYS_T::GPA_HR: PUHR14 Position         */
#define SYS_GPA_HR_PUHR14_Msk            (0x1ul << SYS_GPA_HR_PUHR14_Pos)                  /*!< SYS_T::GPA_HR: PUHR14 Mask             */

#define SYS_GPA_HR_PUHR15_Pos            (15)                                              /*!< SYS_T::GPA_HR: PUHR15 Position         */
#define SYS_GPA_HR_PUHR15_Msk            (0x1ul << SYS_GPA_HR_PUHR15_Pos)                  /*!< SYS_T::GPA_HR: PUHR15 Mask             */

#define SYS_GPA_IEN_IEN_Pos              (0)                                               /*!< SYS_T::GPA_IEN: IEN Position           */
#define SYS_GPA_IEN_IEN_Msk              (0x1ul << SYS_GPA_IEN_IEN_Pos)                    /*!< SYS_T::GPA_IEN: IEN Mask               */

#define SYS_GPB_PULL_PUEN0_Pos           (0)                                               /*!< SYS_T::GPB_PULL: PUEN0 Position        */
#define SYS_GPB_PULL_PUEN0_Msk           (0x1ul << SYS_GPB_PULL_PUEN0_Pos)                 /*!< SYS_T::GPB_PULL: PUEN0 Mask            */

#define SYS_GPB_PULL_PUEN1_Pos           (1)                                               /*!< SYS_T::GPB_PULL: PUEN1 Position        */
#define SYS_GPB_PULL_PUEN1_Msk           (0x1ul << SYS_GPB_PULL_PUEN1_Pos)                 /*!< SYS_T::GPB_PULL: PUEN1 Mask            */

#define SYS_GPB_HR_PUHR0_Pos             (0)                                               /*!< SYS_T::GPB_HR: PUHR0 Position          */
#define SYS_GPB_HR_PUHR0_Msk             (0x1ul << SYS_GPB_HR_PUHR0_Pos)                   /*!< SYS_T::GPB_HR: PUHR0 Mask              */

#define SYS_GPB_HR_PUHR1_Pos             (1)                                               /*!< SYS_T::GPB_HR: PUHR1 Position          */
#define SYS_GPB_HR_PUHR1_Msk             (0x1ul << SYS_GPB_HR_PUHR1_Pos)                   /*!< SYS_T::GPB_HR: PUHR1 Mask              */

#define SYS_GPB_IEN_IEN0_Pos             (0)                                               /*!< SYS_T::GPB_IEN: IEN0 Position          */
#define SYS_GPB_IEN_IEN0_Msk             (0x1ul << SYS_GPB_IEN_IEN0_Pos)                   /*!< SYS_T::GPB_IEN: IEN0 Mask              */

#define SYS_GPB_IEN_IEN1_Pos             (1)                                               /*!< SYS_T::GPB_IEN: IEN1 Position          */
#define SYS_GPB_IEN_IEN1_Msk             (0x1ul << SYS_GPB_IEN_IEN1_Pos)                   /*!< SYS_T::GPB_IEN: IEN1 Mask              */

#define SYS_GPC_PULL_PUEN0_Pos           (0)                                               /*!< SYS_T::GPC_PULL: PUEN0 Position        */
#define SYS_GPC_PULL_PUEN0_Msk           (0x1ul << SYS_GPC_PULL_PUEN0_Pos)                 /*!< SYS_T::GPC_PULL: PUEN0 Mask            */

#define SYS_GPC_PULL_PUEN1_Pos           (1)                                               /*!< SYS_T::GPC_PULL: PUEN1 Position        */
#define SYS_GPC_PULL_PUEN1_Msk           (0x1ul << SYS_GPC_PULL_PUEN1_Pos)                 /*!< SYS_T::GPC_PULL: PUEN1 Mask            */

#define SYS_GPC_PULL_PUEN2_Pos           (2)                                               /*!< SYS_T::GPC_PULL: PUEN2 Position        */
#define SYS_GPC_PULL_PUEN2_Msk           (0x1ul << SYS_GPC_PULL_PUEN2_Pos)                 /*!< SYS_T::GPC_PULL: PUEN2 Mask            */

#define SYS_GPC_PULL_PUEN3_Pos           (3)                                               /*!< SYS_T::GPC_PULL: PUEN3 Position        */
#define SYS_GPC_PULL_PUEN3_Msk           (0x1ul << SYS_GPC_PULL_PUEN3_Pos)                 /*!< SYS_T::GPC_PULL: PUEN3 Mask            */

#define SYS_GPC_PULL_PUEN4_Pos           (4)                                               /*!< SYS_T::GPC_PULL: PUEN4 Position        */
#define SYS_GPC_PULL_PUEN4_Msk           (0x1ul << SYS_GPC_PULL_PUEN4_Pos)                 /*!< SYS_T::GPC_PULL: PUEN4 Mask            */

#define SYS_GPC_PULL_PUEN5_Pos           (5)                                               /*!< SYS_T::GPC_PULL: PUEN5 Position        */
#define SYS_GPC_PULL_PUEN5_Msk           (0x1ul << SYS_GPC_PULL_PUEN5_Pos)                 /*!< SYS_T::GPC_PULL: PUEN5 Mask            */

#define SYS_GPC_PULL_PUEN6_Pos           (6)                                               /*!< SYS_T::GPC_PULL: PUEN6 Position        */
#define SYS_GPC_PULL_PUEN6_Msk           (0x1ul << SYS_GPC_PULL_PUEN6_Pos)                 /*!< SYS_T::GPC_PULL: PUEN6 Mask            */

#define SYS_GPC_PULL_PUEN7_Pos           (7)                                               /*!< SYS_T::GPC_PULL: PUEN7 Position        */
#define SYS_GPC_PULL_PUEN7_Msk           (0x1ul << SYS_GPC_PULL_PUEN7_Pos)                 /*!< SYS_T::GPC_PULL: PUEN7 Mask            */

#define SYS_GPC_PULL_PUEN8_Pos           (8)                                               /*!< SYS_T::GPC_PULL: PUEN8 Position        */
#define SYS_GPC_PULL_PUEN8_Msk           (0x1ul << SYS_GPC_PULL_PUEN8_Pos)                 /*!< SYS_T::GPC_PULL: PUEN8 Mask            */

#define SYS_GPC_PULL_PUEN9_Pos           (9)                                               /*!< SYS_T::GPC_PULL: PUEN9 Position        */
#define SYS_GPC_PULL_PUEN9_Msk           (0x1ul << SYS_GPC_PULL_PUEN9_Pos)                 /*!< SYS_T::GPC_PULL: PUEN9 Mask            */

#define SYS_GPC_PULL_PUEN10_Pos          (10)                                              /*!< SYS_T::GPC_PULL: PUEN10 Position       */
#define SYS_GPC_PULL_PUEN10_Msk          (0x1ul << SYS_GPC_PULL_PUEN10_Pos)                /*!< SYS_T::GPC_PULL: PUEN10 Mask           */

#define SYS_GPC_PULL_PUEN11_Pos          (11)                                              /*!< SYS_T::GPC_PULL: PUEN11 Position       */
#define SYS_GPC_PULL_PUEN11_Msk          (0x1ul << SYS_GPC_PULL_PUEN11_Pos)                /*!< SYS_T::GPC_PULL: PUEN11 Mask           */

#define SYS_GPC_PULL_PUEN12_Pos          (12)                                              /*!< SYS_T::GPC_PULL: PUEN12 Position       */
#define SYS_GPC_PULL_PUEN12_Msk          (0x1ul << SYS_GPC_PULL_PUEN12_Pos)                /*!< SYS_T::GPC_PULL: PUEN12 Mask           */

#define SYS_GPC_PULL_PUEN13_Pos          (13)                                              /*!< SYS_T::GPC_PULL: PUEN13 Position       */
#define SYS_GPC_PULL_PUEN13_Msk          (0x1ul << SYS_GPC_PULL_PUEN13_Pos)                /*!< SYS_T::GPC_PULL: PUEN13 Mask           */

#define SYS_GPC_PULL_PUEN14_Pos          (14)                                              /*!< SYS_T::GPC_PULL: PUEN14 Position       */
#define SYS_GPC_PULL_PUEN14_Msk          (0x1ul << SYS_GPC_PULL_PUEN14_Pos)                /*!< SYS_T::GPC_PULL: PUEN14 Mask           */

#define SYS_GPC_PULL_PUEN15_Pos          (15)                                              /*!< SYS_T::GPC_PULL: PUEN15 Position       */
#define SYS_GPC_PULL_PUEN15_Msk          (0x1ul << SYS_GPC_PULL_PUEN15_Pos)                /*!< SYS_T::GPC_PULL: PUEN15 Mask           */

#define SYS_GPC_HR_PUHR0_Pos             (0)                                               /*!< SYS_T::GPC_HR: PUHR0 Position          */
#define SYS_GPC_HR_PUHR0_Msk             (0x1ul << SYS_GPC_HR_PUHR0_Pos)                   /*!< SYS_T::GPC_HR: PUHR0 Mask              */

#define SYS_GPC_HR_PUHR1_Pos             (1)                                               /*!< SYS_T::GPC_HR: PUHR1 Position          */
#define SYS_GPC_HR_PUHR1_Msk             (0x1ul << SYS_GPC_HR_PUHR1_Pos)                   /*!< SYS_T::GPC_HR: PUHR1 Mask              */

#define SYS_GPC_HR_PUHR2_Pos             (2)                                               /*!< SYS_T::GPC_HR: PUHR2 Position          */
#define SYS_GPC_HR_PUHR2_Msk             (0x1ul << SYS_GPC_HR_PUHR2_Pos)                   /*!< SYS_T::GPC_HR: PUHR2 Mask              */

#define SYS_GPC_HR_PUHR3_Pos             (3)                                               /*!< SYS_T::GPC_HR: PUHR3 Position          */
#define SYS_GPC_HR_PUHR3_Msk             (0x1ul << SYS_GPC_HR_PUHR3_Pos)                   /*!< SYS_T::GPC_HR: PUHR3 Mask              */

#define SYS_GPC_HR_PUHR4_Pos             (4)                                               /*!< SYS_T::GPC_HR: PUHR4 Position          */
#define SYS_GPC_HR_PUHR4_Msk             (0x1ul << SYS_GPC_HR_PUHR4_Pos)                   /*!< SYS_T::GPC_HR: PUHR4 Mask              */

#define SYS_GPC_HR_PUHR5_Pos             (5)                                               /*!< SYS_T::GPC_HR: PUHR5 Position          */
#define SYS_GPC_HR_PUHR5_Msk             (0x1ul << SYS_GPC_HR_PUHR5_Pos)                   /*!< SYS_T::GPC_HR: PUHR5 Mask              */

#define SYS_GPC_HR_PUHR6_Pos             (6)                                               /*!< SYS_T::GPC_HR: PUHR6 Position          */
#define SYS_GPC_HR_PUHR6_Msk             (0x1ul << SYS_GPC_HR_PUHR6_Pos)                   /*!< SYS_T::GPC_HR: PUHR6 Mask              */

#define SYS_GPC_HR_PUHR7_Pos             (7)                                               /*!< SYS_T::GPC_HR: PUHR7 Position          */
#define SYS_GPC_HR_PUHR7_Msk             (0x1ul << SYS_GPC_HR_PUHR7_Pos)                   /*!< SYS_T::GPC_HR: PUHR7 Mask              */

#define SYS_GPC_HR_PUHR8_Pos             (8)                                               /*!< SYS_T::GPC_HR: PUHR8 Position          */
#define SYS_GPC_HR_PUHR8_Msk             (0x1ul << SYS_GPC_HR_PUHR8_Pos)                   /*!< SYS_T::GPC_HR: PUHR8 Mask              */

#define SYS_GPC_HR_PUHR9_Pos             (9)                                               /*!< SYS_T::GPC_HR: PUHR9 Position          */
#define SYS_GPC_HR_PUHR9_Msk             (0x1ul << SYS_GPC_HR_PUHR9_Pos)                   /*!< SYS_T::GPC_HR: PUHR9 Mask              */

#define SYS_GPC_HR_PUHR10_Pos            (10)                                              /*!< SYS_T::GPC_HR: PUHR10 Position         */
#define SYS_GPC_HR_PUHR10_Msk            (0x1ul << SYS_GPC_HR_PUHR10_Pos)                  /*!< SYS_T::GPC_HR: PUHR10 Mask             */

#define SYS_GPC_HR_PUHR11_Pos            (11)                                              /*!< SYS_T::GPC_HR: PUHR11 Position         */
#define SYS_GPC_HR_PUHR11_Msk            (0x1ul << SYS_GPC_HR_PUHR11_Pos)                  /*!< SYS_T::GPC_HR: PUHR11 Mask             */

#define SYS_GPC_HR_PUHR12_Pos            (12)                                              /*!< SYS_T::GPC_HR: PUHR12 Position         */
#define SYS_GPC_HR_PUHR12_Msk            (0x1ul << SYS_GPC_HR_PUHR12_Pos)                  /*!< SYS_T::GPC_HR: PUHR12 Mask             */

#define SYS_GPC_HR_PUHR13_Pos            (13)                                              /*!< SYS_T::GPC_HR: PUHR13 Position         */
#define SYS_GPC_HR_PUHR13_Msk            (0x1ul << SYS_GPC_HR_PUHR13_Pos)                  /*!< SYS_T::GPC_HR: PUHR13 Mask             */

#define SYS_GPC_HR_PUHR14_Pos            (14)                                              /*!< SYS_T::GPC_HR: PUHR14 Position         */
#define SYS_GPC_HR_PUHR14_Msk            (0x1ul << SYS_GPC_HR_PUHR14_Pos)                  /*!< SYS_T::GPC_HR: PUHR14 Mask             */

#define SYS_GPC_HR_PUHR15_Pos            (15)                                              /*!< SYS_T::GPC_HR: PUHR15 Position         */
#define SYS_GPC_HR_PUHR15_Msk            (0x1ul << SYS_GPC_HR_PUHR15_Pos)                  /*!< SYS_T::GPC_HR: PUHR15 Mask             */

#define SYS_GPC_IEN_IEN_Pos              (0)                                               /*!< SYS_T::GPC_IEN: IEN Position           */
#define SYS_GPC_IEN_IEN_Msk              (0x1ul << SYS_GPC_IEN_IEN_Pos)                    /*!< SYS_T::GPC_IEN: IEN Mask               */

#define SYS_GPD_PULL_PUEN0_Pos           (0)                                               /*!< SYS_T::GPD_PULL: PUEN0 Position        */
#define SYS_GPD_PULL_PUEN0_Msk           (0x1ul << SYS_GPD_PULL_PUEN0_Pos)                 /*!< SYS_T::GPD_PULL: PUEN0 Mask            */

#define SYS_GPD_PULL_PUEN1_Pos           (1)                                               /*!< SYS_T::GPD_PULL: PUEN1 Position        */
#define SYS_GPD_PULL_PUEN1_Msk           (0x1ul << SYS_GPD_PULL_PUEN1_Pos)                 /*!< SYS_T::GPD_PULL: PUEN1 Mask            */

#define SYS_GPD_PULL_PUEN2_Pos           (2)                                               /*!< SYS_T::GPD_PULL: PUEN2 Position        */
#define SYS_GPD_PULL_PUEN2_Msk           (0x1ul << SYS_GPD_PULL_PUEN2_Pos)                 /*!< SYS_T::GPD_PULL: PUEN2 Mask            */

#define SYS_GPD_PULL_PUEN3_Pos           (3)                                               /*!< SYS_T::GPD_PULL: PUEN3 Position        */
#define SYS_GPD_PULL_PUEN3_Msk           (0x1ul << SYS_GPD_PULL_PUEN3_Pos)                 /*!< SYS_T::GPD_PULL: PUEN3 Mask            */

#define SYS_GPD_PULL_PUEN4_Pos           (4)                                               /*!< SYS_T::GPD_PULL: PUEN4 Position        */
#define SYS_GPD_PULL_PUEN4_Msk           (0x1ul << SYS_GPD_PULL_PUEN4_Pos)                 /*!< SYS_T::GPD_PULL: PUEN4 Mask            */

#define SYS_GPD_PULL_PUEN5_Pos           (5)                                               /*!< SYS_T::GPD_PULL: PUEN5 Position        */
#define SYS_GPD_PULL_PUEN5_Msk           (0x1ul << SYS_GPD_PULL_PUEN5_Pos)                 /*!< SYS_T::GPD_PULL: PUEN5 Mask            */

#define SYS_GPD_PULL_PUEN6_Pos           (6)                                               /*!< SYS_T::GPD_PULL: PUEN6 Position        */
#define SYS_GPD_PULL_PUEN6_Msk           (0x1ul << SYS_GPD_PULL_PUEN6_Pos)                 /*!< SYS_T::GPD_PULL: PUEN6 Mask            */

#define SYS_GPD_PULL_PUEN7_Pos           (7)                                               /*!< SYS_T::GPD_PULL: PUEN7 Position        */
#define SYS_GPD_PULL_PUEN7_Msk           (0x1ul << SYS_GPD_PULL_PUEN7_Pos)                 /*!< SYS_T::GPD_PULL: PUEN7 Mask            */

#define SYS_GPD_PULL_PUEN8_Pos           (8)                                               /*!< SYS_T::GPD_PULL: PUEN8 Position        */
#define SYS_GPD_PULL_PUEN8_Msk           (0x1ul << SYS_GPD_PULL_PUEN8_Pos)                 /*!< SYS_T::GPD_PULL: PUEN8 Mask            */

#define SYS_GPD_PULL_PUEN9_Pos           (9)                                               /*!< SYS_T::GPD_PULL: PUEN9 Position        */
#define SYS_GPD_PULL_PUEN9_Msk           (0x1ul << SYS_GPD_PULL_PUEN9_Pos)                 /*!< SYS_T::GPD_PULL: PUEN9 Mask            */

#define SYS_GPD_PULL_PUEN10_Pos          (10)                                              /*!< SYS_T::GPD_PULL: PUEN10 Position       */
#define SYS_GPD_PULL_PUEN10_Msk          (0x1ul << SYS_GPD_PULL_PUEN10_Pos)                /*!< SYS_T::GPD_PULL: PUEN10 Mask           */

#define SYS_GPD_PULL_PUEN11_Pos          (11)                                              /*!< SYS_T::GPD_PULL: PUEN11 Position       */
#define SYS_GPD_PULL_PUEN11_Msk          (0x1ul << SYS_GPD_PULL_PUEN11_Pos)                /*!< SYS_T::GPD_PULL: PUEN11 Mask           */

#define SYS_GPD_PULL_PUEN12_Pos          (12)                                              /*!< SYS_T::GPD_PULL: PUEN12 Position       */
#define SYS_GPD_PULL_PUEN12_Msk          (0x1ul << SYS_GPD_PULL_PUEN12_Pos)                /*!< SYS_T::GPD_PULL: PUEN12 Mask           */

#define SYS_GPD_PULL_PUEN13_Pos          (13)                                              /*!< SYS_T::GPD_PULL: PUEN13 Position       */
#define SYS_GPD_PULL_PUEN13_Msk          (0x1ul << SYS_GPD_PULL_PUEN13_Pos)                /*!< SYS_T::GPD_PULL: PUEN13 Mask           */

#define SYS_GPD_PULL_PUEN14_Pos          (14)                                              /*!< SYS_T::GPD_PULL: PUEN14 Position       */
#define SYS_GPD_PULL_PUEN14_Msk          (0x1ul << SYS_GPD_PULL_PUEN14_Pos)                /*!< SYS_T::GPD_PULL: PUEN14 Mask           */

#define SYS_GPD_PULL_PUEN15_Pos          (15)                                              /*!< SYS_T::GPD_PULL: PUEN15 Position       */
#define SYS_GPD_PULL_PUEN15_Msk          (0x1ul << SYS_GPD_PULL_PUEN15_Pos)                /*!< SYS_T::GPD_PULL: PUEN15 Mask           */

#define SYS_GPD_HR_PUHR0_Pos             (0)                                               /*!< SYS_T::GPD_HR: PUHR0 Position          */
#define SYS_GPD_HR_PUHR0_Msk             (0x1ul << SYS_GPD_HR_PUHR0_Pos)                   /*!< SYS_T::GPD_HR: PUHR0 Mask              */

#define SYS_GPD_HR_PUHR1_Pos             (1)                                               /*!< SYS_T::GPD_HR: PUHR1 Position          */
#define SYS_GPD_HR_PUHR1_Msk             (0x1ul << SYS_GPD_HR_PUHR1_Pos)                   /*!< SYS_T::GPD_HR: PUHR1 Mask              */

#define SYS_GPD_HR_PUHR2_Pos             (2)                                               /*!< SYS_T::GPD_HR: PUHR2 Position          */
#define SYS_GPD_HR_PUHR2_Msk             (0x1ul << SYS_GPD_HR_PUHR2_Pos)                   /*!< SYS_T::GPD_HR: PUHR2 Mask              */

#define SYS_GPD_HR_PUHR3_Pos             (3)                                               /*!< SYS_T::GPD_HR: PUHR3 Position          */
#define SYS_GPD_HR_PUHR3_Msk             (0x1ul << SYS_GPD_HR_PUHR3_Pos)                   /*!< SYS_T::GPD_HR: PUHR3 Mask              */

#define SYS_GPD_HR_PUHR4_Pos             (4)                                               /*!< SYS_T::GPD_HR: PUHR4 Position          */
#define SYS_GPD_HR_PUHR4_Msk             (0x1ul << SYS_GPD_HR_PUHR4_Pos)                   /*!< SYS_T::GPD_HR: PUHR4 Mask              */

#define SYS_GPD_HR_PUHR5_Pos             (5)                                               /*!< SYS_T::GPD_HR: PUHR5 Position          */
#define SYS_GPD_HR_PUHR5_Msk             (0x1ul << SYS_GPD_HR_PUHR5_Pos)                   /*!< SYS_T::GPD_HR: PUHR5 Mask              */

#define SYS_GPD_HR_PUHR6_Pos             (6)                                               /*!< SYS_T::GPD_HR: PUHR6 Position          */
#define SYS_GPD_HR_PUHR6_Msk             (0x1ul << SYS_GPD_HR_PUHR6_Pos)                   /*!< SYS_T::GPD_HR: PUHR6 Mask              */

#define SYS_GPD_HR_PUHR7_Pos             (7)                                               /*!< SYS_T::GPD_HR: PUHR7 Position          */
#define SYS_GPD_HR_PUHR7_Msk             (0x1ul << SYS_GPD_HR_PUHR7_Pos)                   /*!< SYS_T::GPD_HR: PUHR7 Mask              */

#define SYS_GPD_HR_PUHR8_Pos             (8)                                               /*!< SYS_T::GPD_HR: PUHR8 Position          */
#define SYS_GPD_HR_PUHR8_Msk             (0x1ul << SYS_GPD_HR_PUHR8_Pos)                   /*!< SYS_T::GPD_HR: PUHR8 Mask              */

#define SYS_GPD_HR_PUHR9_Pos             (9)                                               /*!< SYS_T::GPD_HR: PUHR9 Position          */
#define SYS_GPD_HR_PUHR9_Msk             (0x1ul << SYS_GPD_HR_PUHR9_Pos)                   /*!< SYS_T::GPD_HR: PUHR9 Mask              */

#define SYS_GPD_HR_PUHR10_Pos            (10)                                              /*!< SYS_T::GPD_HR: PUHR10 Position         */
#define SYS_GPD_HR_PUHR10_Msk            (0x1ul << SYS_GPD_HR_PUHR10_Pos)                  /*!< SYS_T::GPD_HR: PUHR10 Mask             */

#define SYS_GPD_HR_PUHR11_Pos            (11)                                              /*!< SYS_T::GPD_HR: PUHR11 Position         */
#define SYS_GPD_HR_PUHR11_Msk            (0x1ul << SYS_GPD_HR_PUHR11_Pos)                  /*!< SYS_T::GPD_HR: PUHR11 Mask             */

#define SYS_GPD_HR_PUHR12_Pos            (12)                                              /*!< SYS_T::GPD_HR: PUHR12 Position         */
#define SYS_GPD_HR_PUHR12_Msk            (0x1ul << SYS_GPD_HR_PUHR12_Pos)                  /*!< SYS_T::GPD_HR: PUHR12 Mask             */

#define SYS_GPD_HR_PUHR13_Pos            (13)                                              /*!< SYS_T::GPD_HR: PUHR13 Position         */
#define SYS_GPD_HR_PUHR13_Msk            (0x1ul << SYS_GPD_HR_PUHR13_Pos)                  /*!< SYS_T::GPD_HR: PUHR13 Mask             */

#define SYS_GPD_HR_PUHR14_Pos            (14)                                              /*!< SYS_T::GPD_HR: PUHR14 Position         */
#define SYS_GPD_HR_PUHR14_Msk            (0x1ul << SYS_GPD_HR_PUHR14_Pos)                  /*!< SYS_T::GPD_HR: PUHR14 Mask             */

#define SYS_GPD_HR_PUHR15_Pos            (15)                                              /*!< SYS_T::GPD_HR: PUHR15 Position         */
#define SYS_GPD_HR_PUHR15_Msk            (0x1ul << SYS_GPD_HR_PUHR15_Pos)                  /*!< SYS_T::GPD_HR: PUHR15 Mask             */

#define SYS_GPD_IEN_IEN_Pos              (0)                                               /*!< SYS_T::GPD_IEN: IEN Position           */
#define SYS_GPD_IEN_IEN_Msk              (0x1ul << SYS_GPD_IEN_IEN_Pos)                    /*!< SYS_T::GPD_IEN: IEN Mask               */

#define SYS_IMGMAP3_IMG3_Pos             (0)                                               /*!< SYS_T::IMGMAP3: IMG3 Position          */
#define SYS_IMGMAP3_IMG3_Msk             (0xfffffffful << SYS_IMGMAP3_IMG3_Pos)            /*!< SYS_T::IMGMAP3: IMG3 Mask              */

#define SYS_REGLCTL_REGLCTL_Pos          (0)                                               /*!< SYS_T::REGLCTL: REGLCTL Position       */
#define SYS_REGLCTL_REGLCTL_Msk          (0x1ul << SYS_REGLCTL_REGLCTL_Pos)                /*!< SYS_T::REGLCTL: REGLCTL Mask           */

#define SYS_OSCTRIM_TRIM_Pos             (0)                                               /*!< SYS_T::OSCTRIM: TRIM Position          */
#define SYS_OSCTRIM_TRIM_Msk             (0x3fful << SYS_OSCTRIM_TRIM_Pos)                 /*!< SYS_T::OSCTRIM: TRIM Mask              */

#define SYS_OSCTRIM_EN2MHZ_Pos           (15)                                              /*!< SYS_T::OSCTRIM: EN2MHZ Position        */
#define SYS_OSCTRIM_EN2MHZ_Msk           (0x1ul << SYS_OSCTRIM_EN2MHZ_Pos)                 /*!< SYS_T::OSCTRIM: EN2MHZ Mask            */

#define SYS_OSC10K_OSC10K_TRIM_Pos       (0)                                               /*!< SYS_T::OSC10K: OSC10K_TRIM Position    */
#define SYS_OSC10K_OSC10K_TRIM_Msk       (0x7ffffful << SYS_OSC10K_OSC10K_TRIM_Pos)        /*!< SYS_T::OSC10K: OSC10K_TRIM Mask        */

#define SYS_OSC10K_TRM_CLK_Pos           (31)                                              /*!< SYS_T::OSC10K: TRM_CLK Position        */
#define SYS_OSC10K_TRM_CLK_Msk           (0x1ul << SYS_OSC10K_TRM_CLK_Pos)                 /*!< SYS_T::OSC10K: TRM_CLK Mask            */

#define SYS_OSC_TRIM0_TRIM_Pos           (0)                                               /*!< SYS_T::OSC_TRIM0: TRIM Position        */
#define SYS_OSC_TRIM0_TRIM_Msk           (0xfffful << SYS_OSC_TRIM0_TRIM_Pos)              /*!< SYS_T::OSC_TRIM0: TRIM Mask            */

#define SYS_OSC_TRIM0_EN2MHZ_Pos         (31)                                              /*!< SYS_T::OSC_TRIM0: EN2MHZ Position      */
#define SYS_OSC_TRIM0_EN2MHZ_Msk         (0x1ul << SYS_OSC_TRIM0_EN2MHZ_Pos)               /*!< SYS_T::OSC_TRIM0: EN2MHZ Mask          */

#define SYS_OSC_TRIM1_TRIM_Pos           (0)                                               /*!< SYS_T::OSC_TRIM1: TRIM Position        */
#define SYS_OSC_TRIM1_TRIM_Msk           (0xfffful << SYS_OSC_TRIM1_TRIM_Pos)              /*!< SYS_T::OSC_TRIM1: TRIM Mask            */

#define SYS_OSC_TRIM1_EN2MHZ_Pos         (31)                                              /*!< SYS_T::OSC_TRIM1: EN2MHZ Position      */
#define SYS_OSC_TRIM1_EN2MHZ_Msk         (0x1ul << SYS_OSC_TRIM1_EN2MHZ_Pos)               /*!< SYS_T::OSC_TRIM1: EN2MHZ Mask          */

#define SYS_OSC_TRIM2_TRIM_Pos           (0)                                               /*!< SYS_T::OSC_TRIM2: TRIM Position        */
#define SYS_OSC_TRIM2_TRIM_Msk           (0xfffful << SYS_OSC_TRIM2_TRIM_Pos)              /*!< SYS_T::OSC_TRIM2: TRIM Mask            */

#define SYS_OSC_TRIM2_EN2MHZ_Pos         (31)                                              /*!< SYS_T::OSC_TRIM2: EN2MHZ Position      */
#define SYS_OSC_TRIM2_EN2MHZ_Msk         (0x1ul << SYS_OSC_TRIM2_EN2MHZ_Pos)               /*!< SYS_T::OSC_TRIM2: EN2MHZ Mask          */

#define SYS_IRCTCTL_FREQSEL_Pos          (0)                                               /*!< SYS_T::IRCTCTL: FREQSEL Position       */
#define SYS_IRCTCTL_FREQSEL_Msk          (0x3ul << SYS_IRCTCTL_FREQSEL_Pos)                /*!< SYS_T::IRCTCTL: FREQSEL Mask           */

#define SYS_IRCTCTL_LOOPSEL_Pos          (4)                                               /*!< SYS_T::IRCTCTL: LOOPSEL Position       */
#define SYS_IRCTCTL_LOOPSEL_Msk          (0x3ul << SYS_IRCTCTL_LOOPSEL_Pos)                /*!< SYS_T::IRCTCTL: LOOPSEL Mask           */

#define SYS_IRCTCTL_RETRYCNT_Pos         (6)                                               /*!< SYS_T::IRCTCTL: RETRYCNT Position      */
#define SYS_IRCTCTL_RETRYCNT_Msk         (0x3ul << SYS_IRCTCTL_RETRYCNT_Pos)               /*!< SYS_T::IRCTCTL: RETRYCNT Mask          */

#define SYS_IRCTCTL_CESTOPEN_Pos         (8)                                               /*!< SYS_T::IRCTCTL: CESTOPEN Position      */
#define SYS_IRCTCTL_CESTOPEN_Msk         (0x1ul << SYS_IRCTCTL_CESTOPEN_Pos)               /*!< SYS_T::IRCTCTL: CESTOPEN Mask          */

#define SYS_IRCTCTL_REFCKSEL_Pos         (10)                                              /*!< SYS_T::IRCTCTL: REFCKSEL Position      */
#define SYS_IRCTCTL_REFCKSEL_Msk         (0x1ul << SYS_IRCTCTL_REFCKSEL_Pos)               /*!< SYS_T::IRCTCTL: REFCKSEL Mask          */

#define SYS_IRCTIEN_TFAILIEN_Pos         (1)                                               /*!< SYS_T::IRCTIEN: TFAILIEN Position      */
#define SYS_IRCTIEN_TFAILIEN_Msk         (0x1ul << SYS_IRCTIEN_TFAILIEN_Pos)               /*!< SYS_T::IRCTIEN: TFAILIEN Mask          */

#define SYS_IRCTIEN_CLKEIEN_Pos          (2)                                               /*!< SYS_T::IRCTIEN: CLKEIEN Position       */
#define SYS_IRCTIEN_CLKEIEN_Msk          (0x1ul << SYS_IRCTIEN_CLKEIEN_Pos)                /*!< SYS_T::IRCTIEN: CLKEIEN Mask           */

#define SYS_IRCTISTS_FREQLOCK_Pos        (0)                                               /*!< SYS_T::IRCTISTS: FREQLOCK Position     */
#define SYS_IRCTISTS_FREQLOCK_Msk        (0x1ul << SYS_IRCTISTS_FREQLOCK_Pos)              /*!< SYS_T::IRCTISTS: FREQLOCK Mask         */

#define SYS_IRCTISTS_TFAILIF_Pos         (1)                                               /*!< SYS_T::IRCTISTS: TFAILIF Position      */
#define SYS_IRCTISTS_TFAILIF_Msk         (0x1ul << SYS_IRCTISTS_TFAILIF_Pos)               /*!< SYS_T::IRCTISTS: TFAILIF Mask          */

#define SYS_IRCTISTS_CLKERRIF_Pos        (2)                                               /*!< SYS_T::IRCTISTS: CLKERRIF Position     */
#define SYS_IRCTISTS_CLKERRIF_Msk        (0x1ul << SYS_IRCTISTS_CLKERRIF_Pos)              /*!< SYS_T::IRCTISTS: CLKERRIF Mask         */

#define SYS_IRCTCKRF_HXTFREQ_Pos         (0)                                               /*!< SYS_T::IRCTCKRF: HXTFREQ Position      */
#define SYS_IRCTCKRF_HXTFREQ_Msk         (0x7ffful << SYS_IRCTCKRF_HXTFREQ_Pos)            /*!< SYS_T::IRCTCKRF: HXTFREQ Mask          */

#define SYS_BGAPTRIM_TRIM_Pos            (0)                                               /*!< SYS_T::BGAPTRIM: TRIM Position         */
#define SYS_BGAPTRIM_TRIM_Msk            (0xful << SYS_BGAPTRIM_TRIM_Pos)                  /*!< SYS_T::BGAPTRIM: TRIM Mask             */

#define SYS_UCIDn_UCID_Pos               (0)                                               /*!< SYS_T::UCIDn: UCID Position            */
#define SYS_UCIDn_UCID_Msk               (0xfffffffful << SYS_UCIDn_UCID_Pos)              /*!< SYS_T::UCIDn: UCID Mask                */


/**@}*/ /* SYS_CONST */
/**@}*/ /* end of SYS register group */


/*---------------------- System Control -------------------------*/
/**
    @addtogroup SYSINFO System Control(SYSINFO)
    Memory Mapped Structure for SYSINFO Controller
@{ */
 
typedef struct
{


/**
 * @var SYSINFO_T::CPUID
 * Offset: 0x00  CPUID Base Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |REVISION  |Revision
 * |        |          |Reads as 0x0
 * |[15:4]  |PARTNO    |Part Number
 * |        |          |Reads as 0xC20.
 * |[19:16] |PART      |ARMv6-m Parts
 * |        |          |Reads as 0xC for ARMv6-M parts
 * |[31:24] |IMPCODE   |Implementer Code Assigned by ARM
 * |        |          |ARM = 0x41.
 * @var SYSINFO_T::ICSR
 * Offset: 0x04  Interrupt Control State Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[8:0]   |VTACT     |Vector Active
 * |        |          |0: Thread mode
 * |        |          |Value > 1: the exception number for the current executing exception.
 * |[20:12] |VTPEND    |Vector Pending
 * |        |          |Indicates the exception number for the highest priority pending exception
 * |        |          |The pending state includes the effect of memory-mapped enable and mask registers
 * |        |          |It does not include the PRIMASK special-purpose register qualifier
 * |        |          |A value of zero indicates no pending exceptions.
 * |[22]    |ISRPEND   |ISR Pending
 * |        |          |Indicates if an external configurable (NVIC generated) interrupt is pending.
 * |[23]    |ISRPREEM  |ISR Preemptive
 * |        |          |If set, a pending exception will be serviced on exit from the debug halt state.
 * |[25]    |PSTKICLR  |Clear a Pending SYST
 * |        |          |Write 1 to clear a pending SYST.
 * |[26]    |PSTKISET  |Set a Pending SYST
 * |        |          |Reads back with current state (1 if Pending, 0 if not).
 * |[27]    |PPSVICLR  |Clear a Pending PendSV Interrupt
 * |        |          |Write 1 to clear a pending PendSV interrupt.
 * |[28]    |PPSVISET  |Set a Pending PendSV Interrupt
 * |        |          |This is normally used to request a context switch
 * |        |          |Reads back with current state (1 if Pending, 0 if not).
 * |[31]    |NMIPNSET  |NMI Pending Set Control
 * |        |          |Setting this bit will activate an NMI
 * |        |          |Since NMI is the highest priority exception, it will activate as soon as it is registered
 * |        |          |Reads back with current state (1 if Pending, 0 if not).
 * @var SYSINFO_T::AIRCTL
 * Offset: 0x0C  Application Interrupt and Reset Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |CLRACTVT  |Clear All Active Vector
 * |        |          |Clears all active state information for fixed and configurable exceptions.
 * |        |          |0= do not clear state information.
 * |        |          |1= clear state information.
 * |        |          |The effect of writing a 1 to this bit if the processor is not halted in Debug, is UNPREDICTABLE.
 * |[2]     |SRSTREQ   |System Reset Request
 * |        |          |0 =do not request a reset.
 * |        |          |1 =request reset.
 * |        |          |Writing 1 to this bit asserts a signal to request a reset by the external system.
 * |[15]    |ENDIANES  |Endianness
 * |        |          |Read Only. Reads 0 indicating little endian machine.
 * |[31:16] |VTKEY     |Vector Key
 * |        |          |The value 0x05FA must be written to this register, otherwise
 * |        |          |a write to register is UNPREDICTABLE.
 * @var SYSINFO_T::SCR
 * Offset: 0x10  System Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |SLPONEXC  |Sleep on Exception
 * |        |          |When set to 1, the core can enter a sleep state on an exception return to Thread mode
 * |        |          |This is the mode and exception level entered at reset, the base level of execution
 * |        |          |Setting this bit to 1 enables an interrupt driven application to avoid returning to an empty main application.
 * |[2]     |SLPDEEP   |Controls Whether the Processor Uses Sleep or Deep Sleep As Its Low Power Mode
 * |        |          |0 = sleep.
 * |        |          |1 = deep sleep.
 * |        |          |The SLPDEEP flag is also used in conjunction with CLK_PWRCTL register to enter deeper power-down states than purely core sleep states.
 * |[4]     |SEVNONPN  |Send Event on Pending Bit
 * |        |          |0 = only enabled interrupts or events can wake-up the processor, disabled interrupts are excluded.
 * |        |          |1 = enabled events and all interrupts, including disabled interrupts, can wake-up the processor.
 * |        |          |When enabled, interrupt transitions from Inactive to Pending are included in the list of wakeup events for the WFE instruction.
 * |        |          |When an event or interrupt enters pending state, the event signal wakes up the processor from WFE
 * |        |          |If the processor is not waiting for an event, the event is registered and affects the next WFE.
 * |        |          |The processor also wakes up on execution of an SEV instruction.
 * @var SYSINFO_T::SHPR2
 * Offset: 0x1C  System Handler Priority Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:30] |PRI11     |Priority of System Handler 11 u2013 SVCall
 * |        |          |u201C0u201D denotes the highest priority and u201C3u201D denotes lowest priority
 * @var SYSINFO_T::SHPR3
 * Offset: 0x20  System Handler Priority Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:22] |PRI14     |Priority of System Handler 14 u2013 PendSV
 * |        |          |u201C0u201D denotes the highest priority and u201C3u201D denotes lowest priority
 * |[31:30] |PRI15     |Priority of System Handler 15 u2013 SYST
 * |        |          |u201C0u201D denotes the highest priority and u201C3u201D denotes lowest priority
 */
    __I  uint32_t CPUID;                 /*!< [0x0000] CPUID Base Register                                              */
    __IO uint32_t ICSR;                  /*!< [0x0004] Interrupt Control State Register                                 */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t AIRCTL;                /*!< [0x000c] Application Interrupt and Reset Control Register                 */
    __IO uint32_t SCR;                   /*!< [0x0010] System Control Register                                          */
    __I  uint32_t RESERVE1[2];
    __IO uint32_t SHPR2;                 /*!< [0x001c] System Handler Priority Register 2                               */
    __IO uint32_t SHPR3;                 /*!< [0x0020] System Handler Priority Register 3                               */

} SYSINFO_T;

/**
    @addtogroup SYSINFO_CONST SYSINFO Bit Field Definition
    Constant Definitions for SYSINFO Controller
@{ */

#define SYSINFO_CPUID_REVISION_Pos       (0)                                               /*!< SYSINFO_T::CPUID: REVISION Position    */
#define SYSINFO_CPUID_REVISION_Msk       (0xful << SYSINFO_CPUID_REVISION_Pos)             /*!< SYSINFO_T::CPUID: REVISION Mask        */

#define SYSINFO_CPUID_PARTNO_Pos         (4)                                               /*!< SYSINFO_T::CPUID: PARTNO Position      */
#define SYSINFO_CPUID_PARTNO_Msk         (0xffful << SYSINFO_CPUID_PARTNO_Pos)             /*!< SYSINFO_T::CPUID: PARTNO Mask          */

#define SYSINFO_CPUID_PART_Pos           (16)                                              /*!< SYSINFO_T::CPUID: PART Position        */
#define SYSINFO_CPUID_PART_Msk           (0xful << SYSINFO_CPUID_PART_Pos)                 /*!< SYSINFO_T::CPUID: PART Mask            */

#define SYSINFO_CPUID_IMPCODE_Pos        (24)                                              /*!< SYSINFO_T::CPUID: IMPCODE Position     */
#define SYSINFO_CPUID_IMPCODE_Msk        (0xfful << SYSINFO_CPUID_IMPCODE_Pos)             /*!< SYSINFO_T::CPUID: IMPCODE Mask         */

#define SYSINFO_ICSR_VTACT_Pos           (0)                                               /*!< SYSINFO_T::ICSR: VTACT Position        */
#define SYSINFO_ICSR_VTACT_Msk           (0x1fful << SYSINFO_ICSR_VTACT_Pos)               /*!< SYSINFO_T::ICSR: VTACT Mask            */

#define SYSINFO_ICSR_VTPEND_Pos          (12)                                              /*!< SYSINFO_T::ICSR: VTPEND Position       */
#define SYSINFO_ICSR_VTPEND_Msk          (0x1fful << SYSINFO_ICSR_VTPEND_Pos)              /*!< SYSINFO_T::ICSR: VTPEND Mask           */

#define SYSINFO_ICSR_ISRPEND_Pos         (22)                                              /*!< SYSINFO_T::ICSR: ISRPEND Position      */
#define SYSINFO_ICSR_ISRPEND_Msk         (0x1ul << SYSINFO_ICSR_ISRPEND_Pos)               /*!< SYSINFO_T::ICSR: ISRPEND Mask          */

#define SYSINFO_ICSR_ISRPREEM_Pos        (23)                                              /*!< SYSINFO_T::ICSR: ISRPREEM Position     */
#define SYSINFO_ICSR_ISRPREEM_Msk        (0x1ul << SYSINFO_ICSR_ISRPREEM_Pos)              /*!< SYSINFO_T::ICSR: ISRPREEM Mask         */

#define SYSINFO_ICSR_PSTKICLR_Pos        (25)                                              /*!< SYSINFO_T::ICSR: PSTKICLR Position     */
#define SYSINFO_ICSR_PSTKICLR_Msk        (0x1ul << SYSINFO_ICSR_PSTKICLR_Pos)              /*!< SYSINFO_T::ICSR: PSTKICLR Mask         */

#define SYSINFO_ICSR_PSTKISET_Pos        (26)                                              /*!< SYSINFO_T::ICSR: PSTKISET Position     */
#define SYSINFO_ICSR_PSTKISET_Msk        (0x1ul << SYSINFO_ICSR_PSTKISET_Pos)              /*!< SYSINFO_T::ICSR: PSTKISET Mask         */

#define SYSINFO_ICSR_PPSVICLR_Pos        (27)                                              /*!< SYSINFO_T::ICSR: PPSVICLR Position     */
#define SYSINFO_ICSR_PPSVICLR_Msk        (0x1ul << SYSINFO_ICSR_PPSVICLR_Pos)              /*!< SYSINFO_T::ICSR: PPSVICLR Mask         */

#define SYSINFO_ICSR_PPSVISET_Pos        (28)                                              /*!< SYSINFO_T::ICSR: PPSVISET Position     */
#define SYSINFO_ICSR_PPSVISET_Msk        (0x1ul << SYSINFO_ICSR_PPSVISET_Pos)              /*!< SYSINFO_T::ICSR: PPSVISET Mask         */

#define SYSINFO_ICSR_NMIPNSET_Pos        (31)                                              /*!< SYSINFO_T::ICSR: NMIPNSET Position     */
#define SYSINFO_ICSR_NMIPNSET_Msk        (0x1ul << SYSINFO_ICSR_NMIPNSET_Pos)              /*!< SYSINFO_T::ICSR: NMIPNSET Mask         */

#define SYSINFO_AIRCTL_CLRACTVT_Pos      (1)                                               /*!< SYSINFO_T::AIRCTL: CLRACTVT Position   */
#define SYSINFO_AIRCTL_CLRACTVT_Msk      (0x1ul << SYSINFO_AIRCTL_CLRACTVT_Pos)            /*!< SYSINFO_T::AIRCTL: CLRACTVT Mask       */

#define SYSINFO_AIRCTL_SRSTREQ_Pos       (2)                                               /*!< SYSINFO_T::AIRCTL: SRSTREQ Position    */
#define SYSINFO_AIRCTL_SRSTREQ_Msk       (0x1ul << SYSINFO_AIRCTL_SRSTREQ_Pos)             /*!< SYSINFO_T::AIRCTL: SRSTREQ Mask        */

#define SYSINFO_AIRCTL_ENDIANES_Pos      (15)                                              /*!< SYSINFO_T::AIRCTL: ENDIANES Position   */
#define SYSINFO_AIRCTL_ENDIANES_Msk      (0x1ul << SYSINFO_AIRCTL_ENDIANES_Pos)            /*!< SYSINFO_T::AIRCTL: ENDIANES Mask       */

#define SYSINFO_AIRCTL_VTKEY_Pos         (16)                                              /*!< SYSINFO_T::AIRCTL: VTKEY Position      */
#define SYSINFO_AIRCTL_VTKEY_Msk         (0xfffful << SYSINFO_AIRCTL_VTKEY_Pos)            /*!< SYSINFO_T::AIRCTL: VTKEY Mask          */

#define SYSINFO_SCR_SLPONEXC_Pos         (1)                                               /*!< SYSINFO_T::SCR: SLPONEXC Position      */
#define SYSINFO_SCR_SLPONEXC_Msk         (0x1ul << SYSINFO_SCR_SLPONEXC_Pos)               /*!< SYSINFO_T::SCR: SLPONEXC Mask          */

#define SYSINFO_SCR_SLPDEEP_Pos          (2)                                               /*!< SYSINFO_T::SCR: SLPDEEP Position       */
#define SYSINFO_SCR_SLPDEEP_Msk          (0x1ul << SYSINFO_SCR_SLPDEEP_Pos)                /*!< SYSINFO_T::SCR: SLPDEEP Mask           */

#define SYSINFO_SCR_SEVNONPN_Pos         (4)                                               /*!< SYSINFO_T::SCR: SEVNONPN Position      */
#define SYSINFO_SCR_SEVNONPN_Msk         (0x1ul << SYSINFO_SCR_SEVNONPN_Pos)               /*!< SYSINFO_T::SCR: SEVNONPN Mask          */

#define SYSINFO_SHPR2_PRI11_Pos          (30)                                              /*!< SYSINFO_T::SHPR2: PRI11 Position       */
#define SYSINFO_SHPR2_PRI11_Msk          (0x3ul << SYSINFO_SHPR2_PRI11_Pos)                /*!< SYSINFO_T::SHPR2: PRI11 Mask           */

#define SYSINFO_SHPR3_PRI14_Pos          (22)                                              /*!< SYSINFO_T::SHPR3: PRI14 Position       */
#define SYSINFO_SHPR3_PRI14_Msk          (0x3ul << SYSINFO_SHPR3_PRI14_Pos)                /*!< SYSINFO_T::SHPR3: PRI14 Mask           */

#define SYSINFO_SHPR3_PRI15_Pos          (30)                                              /*!< SYSINFO_T::SHPR3: PRI15 Position       */
#define SYSINFO_SHPR3_PRI15_Msk          (0x3ul << SYSINFO_SHPR3_PRI15_Pos)                /*!< SYSINFO_T::SHPR3: PRI15 Mask           */

/**@}*/ /* SYSINFO_CONST */
/**@}*/ /* end of SYSINFO register group */


/*---------------------- System Timer Control -------------------------*/
/**
    @addtogroup SYSTICK System Timer Control(SYSTICK)
    Memory Mapped Structure for SYSTICK Controller
@{ */
 
typedef struct
{


/**
 * @var SYSTICK_T::CSR
 * Offset: 0x10  SYST Control and Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ENABLE    |ENABLE
 * |        |          |0 = The counter is disabled.
 * |        |          |1 = The counter will operate in a multi-shot manner.
 * |[1]     |TICKINT   |Enables SYST Exception Request
 * |        |          |0 = Counting down to 0 does not cause the SYST exception to be pended
 * |        |          |Software can use COUNTFLAG to determine if a count to zero has occurred.
 * |        |          |1 = Counting down to 0 will cause SYST exception to be pended
 * |        |          |Clearing the SYST Current Value register by a register write in software will not cause SYST to be pended.
 * |[2]     |CLKSRC    |Clock Source
 * |        |          |0 = Clock selected from CLK_CLKSEL0.STCLKSEL is used as clock source.
 * |        |          |1 = Core clock used for SYST.
 * |[16]    |COUNTFLAG |Count Flag
 * |        |          |Returns 1 if timer counted to 0 since last time this register was read.
 * |        |          |0= Cleared on read or by a write to the Current Value register.
 * |        |          |1= Set by a count transition from 1 to 0.
 * @var SYSTICK_T::RVR
 * Offset: 0x14  SYST Reload Value Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |RELOAD    |SYST Reload
 * |        |          |Value to load into the Current Value register when the counter reaches 0.
 * |        |          |To generate a multi-shot timer with a period of N processor clock cycles, use a RELOAD value of N-1
 * |        |          |For example, if the SYST interrupt is required every 200 clock pulses, set RELOAD to 199.
 * @var SYSTICK_T::CVR
 * Offset: 0x18  SYST Current Value Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |CURRENT   |Current Counter Value
 * |        |          |This is the value of the counter at the time it is sampled
 * |        |          |The counter does not provide read-modify-write protection
 * |        |          |The register is write-clear
 * |        |          |A software write of any value will clear the register to 0 and also clear the COUNTFLAG bit.
 */
    __I  uint32_t RESERVE0[4];
    __IO uint32_t CSR;                   /*!< [0x0010] SYST Control and Status Register                                 */
    __IO uint32_t RVR;                   /*!< [0x0014] SYST Reload Value Register                                       */
    __IO uint32_t CVR;                   /*!< [0x0018] SYST Current Value Register                                      */

} SYSTICK_T;

/**
    @addtogroup SYSTICK_CONST SYSTICK Bit Field Definition
    Constant Definitions for SYSTICK Controller
@{ */

#define SYSTICK_CSR_ENABLE_Pos           (0)                                               /*!< SYSTICK_T::CSR: ENABLE Position        */
#define SYSTICK_CSR_ENABLE_Msk           (0x1ul << SYSTICK_CSR_ENABLE_Pos)                 /*!< SYSTICK_T::CSR: ENABLE Mask            */

#define SYSTICK_CSR_TICKINT_Pos          (1)                                               /*!< SYSTICK_T::CSR: TICKINT Position       */
#define SYSTICK_CSR_TICKINT_Msk          (0x1ul << SYSTICK_CSR_TICKINT_Pos)                /*!< SYSTICK_T::CSR: TICKINT Mask           */

#define SYSTICK_CSR_CLKSRC_Pos           (2)                                               /*!< SYSTICK_T::CSR: CLKSRC Position        */
#define SYSTICK_CSR_CLKSRC_Msk           (0x1ul << SYSTICK_CSR_CLKSRC_Pos)                 /*!< SYSTICK_T::CSR: CLKSRC Mask            */

#define SYSTICK_CSR_COUNTFLAG_Pos        (16)                                              /*!< SYSTICK_T::CSR: COUNTFLAG Position     */
#define SYSTICK_CSR_COUNTFLAG_Msk        (0x1ul << SYSTICK_CSR_COUNTFLAG_Pos)              /*!< SYSTICK_T::CSR: COUNTFLAG Mask         */

#define SYSTICK_RVR_RELOAD_Pos           (0)                                               /*!< SYSTICK_T::RVR: RELOAD Position        */
#define SYSTICK_RVR_RELOAD_Msk           (0xfffffful << SYSTICK_RVR_RELOAD_Pos)            /*!< SYSTICK_T::RVR: RELOAD Mask            */

#define SYSTICK_CVR_CURRENT_Pos          (0)                                               /*!< SYSTICK_T::CVR: CURRENT Position       */
#define SYSTICK_CVR_CURRENT_Msk          (0xfffffful << SYSTICK_CVR_CURRENT_Pos)           /*!< SYSTICK_T::CVR: CURRENT Mask           */

/**@}*/ /* SYSTICK_CONST */
/**@}*/ /* end of SYSTICK register group */


/*---------------------- Timer Controller -------------------------*/
/**
    @addtogroup TMR Timer Controller(TMR)
    Memory Mapped Structure for TMR Controller
@{ */
 
typedef struct
{
/**
 * @var TMR_T::CTL
 * Offset: 0x00  Timer0 Control and Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |PSC       |Timer Clock Prescaler
 * |        |          |Clock input is divided by (PSC+1) before it is fed to the counter
 * |        |          |If PSC = 0, then there is no scaling.
 * |        |          |Note: No matter CNTEN is 0 or 1, whenever software writes a new value into this register, TIMER will restart counting by using this new value and abort previous count.
 * |[25]    |ACTSTS    |Timer Active Status Bit (Read Only)
 * |        |          |This bit indicates the counter status of Timer.
 * |        |          |0 = Timer is not active.
 * |        |          |1 = Timer is active.
 * |[26]    |RSTCNT    |Counter Reset Bit
 * |        |          |Set this bit will reset the Timer counter, pre-scale and also force CNTEN to 0.
 * |        |          |0 = No effect.
 * |        |          |1 = Reset Timeru2019s pre-scale counter, internal 16-bit up-counter and CNTEN bit.
 * |[28:27] |OPMODE    |Timer Operating Mode
 * |        |          |00 = The Timer is operating in the one-shot mode
 * |        |          |The associated interrupt signal is generated once (if INTEN is 1) and CNTEN is automatically cleared by hardware.
 * |        |          |01 = The Timer is operating in the periodic mode
 * |        |          |The associated interrupt signal is generated periodically (if INTEN is 1).
 * |        |          |10 = Reserved.
 * |        |          |11 = The Timer is operating in continuous counting mode
 * |        |          |The associated interrupt signal is generated when TIMERx_CNT = TIMERx_CMP (if INTEN is 1); however, the 16-bit up-counter counts continuously without reset.
 * |        |          |Note: When changing the Timer Operating Mode, the CNTEN bit should be set to 0 disable first. 
 * |[29]    |INTEN     |Interrupt Enable Bit
 * |        |          |0 = Disable TIMER Interrupt.
 * |        |          |1 = Enable TIMER Interrupt.
 * |        |          |If timer interrupt is enabled, and time-out flag (TOF) is 1u2019b .tThe timer asserts its interrupt signal when the associated count is equal to TIMERx_CMPto CPU.
 * |[30]    |CNTEN     |Counter Enable Bit
 * |        |          |0 = Stop/Suspend counting.
 * |        |          |1 = Start counting.
 * |        |          |Note: This bit is auto-cleared by hardware in one-shot mode (OPMODE = 00b) when the associated timer interrupt is generated (INTEN = 1).
 * @var TMR_T::CMP
 * Offset: 0x04  Timer0 Compare Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMPDAT    |Timer Comparison Value
 * |        |          |CMPDAT is a 16-bit comparison register
 * |        |          |When the 16-bit up-counter is enabled and its value is equal to CMPDAT value, a Timer Interrupt out flag (TOF) is requested if the timer interrupt is enabled with TIMERx_CTL.INTEN = 1.
 * |        |          |Note 1: Never set CMPDAT to 0x000 or 0x001. Timer will not function correctly.
 * |        |          |Note 2: No matter CNTEN is 0 or 1, whenever software writes a new value into this register, TIMER will restart counting by using this new value and abort previous count.
 * @var TMR_T::INTSTS
 * Offset: 0x08  Timer0 Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TOF       |Timer Interrupt Out Flag (Read Only)
 * |        |          |This bit indicates the interrupt status of Timer.
 * |        |          |TIF TOF bit is set by hardware when the 16-bit counter matches the timer comparison value (CMPDAT)
 * |        |          |It is cleared by writing 1 to itself
 * |        |          |0 = No effect.
 * |        |          |1 = CNT (TIMERx_CNT [15:0]) value matches the CMPDAT (TIMERx_CMP[15:0]) value. 
 * @var TMR_T::CNT
 * Offset: 0x0C  Timer0 Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNT       |Timer Data Register
 * |        |          |User can read this register for the current up-counter value while TIMERx_CTL.CNTEN is set to 1, 
 * @var TMR_T::CTL
 * Offset: 0x20  Timer1 Control and Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |PSC       |Timer Clock Prescaler
 * |        |          |Clock input is divided by (PSC+1) before it is fed to the counter
 * |        |          |If PSC = 0, then there is no scaling.
 * |        |          |Note: No matter CNTEN is 0 or 1, whenever software writes a new value into this register, TIMER will restart counting by using this new value and abort previous count.
 * |[25]    |ACTSTS    |Timer Active Status Bit (Read Only)
 * |        |          |This bit indicates the counter status of Timer.
 * |        |          |0 = Timer is not active.
 * |        |          |1 = Timer is active.
 * |[26]    |RSTCNT    |Counter Reset Bit
 * |        |          |Set this bit will reset the Timer counter, pre-scale and also force CNTEN to 0.
 * |        |          |0 = No effect.
 * |        |          |1 = Reset Timeru2019s pre-scale counter, internal 16-bit up-counter and CNTEN bit.
 * |[28:27] |OPMODE    |Timer Operating Mode
 * |        |          |00 = The Timer is operating in the one-shot mode
 * |        |          |The associated interrupt signal is generated once (if INTEN is 1) and CNTEN is automatically cleared by hardware.
 * |        |          |01 = The Timer is operating in the periodic mode
 * |        |          |The associated interrupt signal is generated periodically (if INTEN is 1).
 * |        |          |10 = Reserved.
 * |        |          |11 = The Timer is operating in continuous counting mode
 * |        |          |The associated interrupt signal is generated when TIMERx_CNT = TIMERx_CMP (if INTEN is 1); however, the 16-bit up-counter counts continuously without reset.
 * |        |          |Note: When changing the Timer Operating Mode, the CNTEN bit should be set to 0 disable first. 
 * |[29]    |INTEN     |Interrupt Enable Bit
 * |        |          |0 = Disable TIMER Interrupt.
 * |        |          |1 = Enable TIMER Interrupt.
 * |        |          |If timer interrupt is enabled, and time-out flag (TOF) is 1u2019b .tThe timer asserts its interrupt signal when the associated count is equal to TIMERx_CMPto CPU.
 * |[30]    |CNTEN     |Counter Enable Bit
 * |        |          |0 = Stop/Suspend counting.
 * |        |          |1 = Start counting.
 * |        |          |Note: This bit is auto-cleared by hardware in one-shot mode (OPMODE = 00b) when the associated timer interrupt is generated (INTEN = 1).
 * @var TMR_T::CMP
 * Offset: 0x24  Timer1 Compare Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMPDAT    |Timer Comparison Value
 * |        |          |CMPDAT is a 16-bit comparison register
 * |        |          |When the 16-bit up-counter is enabled and its value is equal to CMPDAT value, a Timer Interrupt out flag (TOF) is requested if the timer interrupt is enabled with TIMERx_CTL.INTEN = 1.
 * |        |          |Note 1: Never set CMPDAT to 0x000 or 0x001. Timer will not function correctly.
 * |        |          |Note 2: No matter CNTEN is 0 or 1, whenever software writes a new value into this register, TIMER will restart counting by using this new value and abort previous count.
 * @var TMR_T::INTSTS
 * Offset: 0x28  Timer1 Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TOF       |Timer Interrupt Out Flag (Read Only)
 * |        |          |This bit indicates the interrupt status of Timer.
 * |        |          |TIF TOF bit is set by hardware when the 16-bit counter matches the timer comparison value (CMPDAT)
 * |        |          |It is cleared by writing 1 to itself
 * |        |          |0 = No effect.
 * |        |          |1 = CNT (TIMERx_CNT [15:0]) value matches the CMPDAT (TIMERx_CMP[15:0]) value. 
 * @var TMR_T::CNT
 * Offset: 0x2C  Timer1 Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNT       |Timer Data Register
 * |        |          |User can read this register for the current up-counter value while TIMERx_CTL.CNTEN is set to 1, 
 */
    __IO uint32_t CTL;                   /*!< [0x0000] Timer0 Control and Status Register                               */
    __IO uint32_t CMP;                   /*!< [0x0004] Timer0 Compare Register                                          */
    __IO uint32_t INTSTS;                /*!< [0x0008] Timer0 Status Register                                           */
    __I  uint32_t CNT;                   /*!< [0x000c] Timer0 Data Register                                             */
} TMR_T;

typedef struct
{
/**
 * @var TMR_T::CTL
 * Offset: 0x34  IR Carrier Output Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |NONCS     |Non-carrier state
 * |        |          |0 = IROUT keeps low when IRCEN is 0,
 * |        |          |1 = IROUT keeps high when IRCEN is 0.
 * |[1]     |IRCEN     |IR carrier output enable
 * |        |          |0 = Disable IR carrier output,
 * |        |          |1 = Enable IR carrier output. Timer1 time out will toggle the output state on IROUT pin.
 */
    uint32_t CTL;
} TMRIR_T;

/**
    @addtogroup TMR_CONST TMR Bit Field Definition
    Constant Definitions for TMR Controller
@{ */

#define TMR_CTL_PSC_Pos                  (0)                                               /*!< TMR_T::CTL: PSC Position               */
#define TMR_CTL_PSC_Msk                  (0xfful << TMR_CTL_PSC_Pos)                       /*!< TMR_T::CTL: PSC Mask                   */

#define TMR_CTL_ACTSTS_Pos               (25)                                              /*!< TMR_T::CTL: ACTSTS Position            */
#define TMR_CTL_ACTSTS_Msk               (0x1ul << TMR_CTL_ACTSTS_Pos)                     /*!< TMR_T::CTL: ACTSTS Mask                */

#define TMR_CTL_RSTCNT_Pos               (26)                                              /*!< TMR_T::CTL: RSTCNT Position            */
#define TMR_CTL_RSTCNT_Msk               (0x1ul << TMR_CTL_RSTCNT_Pos)                     /*!< TMR_T::CTL: RSTCNT Mask                */

#define TMR_CTL_OPMODE_Pos               (27)                                              /*!< TMR_T::CTL: OPMODE Position            */
#define TMR_CTL_OPMODE_Msk               (0x3ul << TMR_CTL_OPMODE_Pos)                     /*!< TMR_T::CTL: OPMODE Mask                */

#define TMR_CTL_INTEN_Pos                (29)                                              /*!< TMR_T::CTL: INTEN Position             */
#define TMR_CTL_INTEN_Msk                (0x1ul << TMR_CTL_INTEN_Pos)                      /*!< TMR_T::CTL: INTEN Mask                 */

#define TMR_CTL_CNTEN_Pos                (30)                                              /*!< TMR_T::CTL: CNTEN Position             */
#define TMR_CTL_CNTEN_Msk                (0x1ul << TMR_CTL_CNTEN_Pos)                      /*!< TMR_T::CTL: CNTEN Mask                 */

#define TMR_CMP_CMPDAT_Pos               (0)                                               /*!< TMR_T::CMP: CMPDAT Position            */
#define TMR_CMP_CMPDAT_Msk               (0xfffful << TMR_CMP_CMPDAT_Pos)                  /*!< TMR_T::CMP: CMPDAT Mask                */

#define TMR_INTSTS_TIF_Pos               (0)                                               /*!< TMR_T::INTSTS: TIF Position            */
#define TMR_INTSTS_TIF_Msk               (0x1ul << TMR_INTSTS_TIF_Pos)                     /*!< TMR_T::INTSTS: TIF Mask                */

#define TMR_CNT_CNT_Pos                  (0)                                               /*!< TMR_T::CNT: CNT Position               */
#define TMR_CNT_CNT_Msk                  (0xfffful << TMR_CNT_CNT_Pos)                     /*!< TMR_T::CNT: CNT Mask                   */

#define TMRIR_CTL_NONCS_Pos              (0)                                               /*!< TMRIR_T::CTL: NONCS Position           */
#define TMRIR_CTL_NONCS_Msk              (0x1ul << TMRIR_CTL_NONCS_Pos)                    /*!< TMRIR_T::CTL: NONCS Mask               */

#define TMRIR_CTL_IRCEN_Pos              (1)                                               /*!< TMRIR_T::CTL: IRCEN Position           */
#define TMRIR_CTL_IRCEN_Msk              (0x1ul << TMRIR_CTL_IRCEN_Pos)                    /*!< TMRIR_T::CTL: IRCEN Mask               */

/**@}*/ /* TMR_CONST */
/**@}*/ /* end of TMR register group */


/*---------------------- Universal Asynchronous Receiver/Transmitter Controller -------------------------*/
/**
    @addtogroup UART Universal Asynchronous Receiver/Transmitter Controller(UART)
    Memory Mapped Structure for UART Controller
@{ */
 
typedef struct
{


/**
 * @var UART_T::DAT
 * Offset: 0x00  UART Receive/Transmit FIFO Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DAT       |Receive/Transmit FIFO Register
 * |        |          |Reading this register will return data from the receive data FIFO
 * |        |          |By reading this register, the UART will return the 8-bit data received from Rx pin (LSB first).
 * |        |          |By writing to this register, transmit data will be pushed onto the transmit FIFO
 * |        |          |The UART will send out an 8-bit data through the Tx pin (LSB first).
 * @var UART_T::INTEN
 * Offset: 0x04  UART Interrupt Enable Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RDAIEN    |Receive Data Available Interrupt Enable.
 * |        |          |0 = Mask off RDA_INT
 * |        |          |1 = Enable RDA_INT
 * |[1]     |THREIEN   |Transmit FIFO Register Empty Interrupt Enable
 * |        |          |0 = Mask off THRE_INT
 * |        |          |1 = Enable THRE_INT
 * |[2]     |RLSIEN    |Receive Line Status Interrupt Enable
 * |        |          |0 = Mask off RLS_INT
 * |        |          |1 = EnableRLS_INT
 * |[3]     |MODEMIEN  |Modem Status Interrupt Enable
 * |        |          |0 = Mask off MODEM_INT
 * |        |          |1 = Enable MODEM_INT
 * |[4]     |RXTOIEN   |Receive Time out Interrupt Enable
 * |        |          |0 = Mask off TOUT_INT
 * |        |          |1 = Enable TOUT_INT
 * |[5]     |BUFERRIEN |Buffer Error Interrupt Enable
 * |        |          |0 = Mask off BUF_ERR_INT
 * |        |          |1 = Enable IBUF_ERR_INT
 * |[11]    |TOCNTEN   |Time-Out Counter Enable
 * |        |          |0 = Disable Time-out counter.
 * |        |          |1 = Enable Time-out counter.
 * |[12]    |ATORTSEN  |RTS Auto Flow Control Enable
 * |        |          |0 = Disable RTS auto flow control.
 * |        |          |1 = Enable RTS auto flow control.
 * |        |          |When RTS auto-flow is enabled, if the number of bytes in the Rx FIFO equals FCR.RTS_TRIG_LEVEL, the UART will de-assert the RTS signal.
 * |[13]    |ATOCTSEN  |CTS Auto Flow Control Enable
 * |        |          |0 = Disable CTS auto flow control.
 * |        |          |1 = Enable CTS auto flow control.
 * |        |          |When CTS auto-flow is enabled, the UART will send data to external device when CTS input is asserted (UART will not send data to device until CTS is de-asserted).
 * |[14]    |DMATXEN   |Transmit DMA Enable
 * |        |          |If enabled, the UART will request DMA service when space is available in transmit FIFO.
 * |[15]    |DMARXEN   |Receive DMA Enable
 * |        |          |If enabled, the UART will request DMA service when data is available in receive FIFO.
 * @var UART_T::FIFO
 * Offset: 0x08  UART FIFO Control Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |RXRST     |Receive FIFO Reset
 * |        |          |When RFR is set, all the bytes in the receive FIFO are cleared and receive internal state machine is reset.
 * |        |          |0 = Writing 0 to this bit has no effect.
 * |        |          |1 = Writing 1 to this bit will reset the receive internal state machine and pointers.
 * |        |          |Note: This bit will auto-clear after 3 UART engine clock cycles.
 * |[2]     |TXRST     |Transmit FIFO Reset
 * |        |          |When TFR is set, all the bytes in the transmit FIFO are cleared and transmit internal state machine is reset.
 * |        |          |0 = Writing 0 to this bit has no effect.
 * |        |          |1 = Writing 1 to this bit will reset the transmit internal state machine and pointers.
 * |        |          |Note: This bit will auto-clear after 3 UART engine clock cycles.
 * |[7:4]   |RFITL     |Receive FIFO Interrupt (RDA_INT) Trigger Level
 * |        |          |When the number of bytes in the receive FIFO equals the RFITL then the RDA_IF will be set and, if enabled, an RDA_INT interrupt will generated.
 * |        |          |RFITL
 * |        |          |INTR_RDA Trigger Level   (Bytes)
 * |        |          |0000
 * |        |          |1 Byte
 * |        |          |0001
 * |        |          |4 Bytes
 * |        |          |0010
 * |        |          |8 Bytes
 * |        |          |0011
 * |        |          |14 Bytes
 * |[19:16] |RTSTRGLV  |RTS Trigger Level for Auto-flow Control
 * |        |          |Sets the FIFO trigger level when auto-flow control will de-assert RTS (request-to-send).
 * |        |          |RTS_Tri_Lev
 * |        |          |Trigger Level (Bytes)
 * |        |          |0000
 * |        |          |1 Byte
 * |        |          |0001
 * |        |          |4 Bytes
 * |        |          |0010
 * |        |          |8 Bytes
 * |        |          |0011
 * |        |          |14 Bytes
 * @var UART_T::LINE
 * Offset: 0x0C  UART Line Control Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |WLS       |Word Length Select
 * |        |          |WLS[1:0]
 * |        |          |Character length
 * |        |          |00
 * |        |          |5 bits
 * |        |          |01
 * |        |          |6 bits
 * |        |          |10
 * |        |          |7 bits
 * |        |          |11
 * |        |          |8 bits
 * |[2]     |NSB       |Number of STOP bits
 * |        |          |0= One u201CSTOP bitu201D is generated after the transmitted data
 * |        |          |1= Two u201CSTOP bitsu201D are generated when 6-, 7- and 8-bit word length is selected; One and a half u201CSTOP bitsu201D are generated in the transmitted data when 5-bit word length is selected
 * |[3]     |PBE       |Parity Bit Enable
 * |        |          |0 = Parity bit is not generated (transmit data) or checked (receive data) during transfer.
 * |        |          |1 = Parity bit is generated or checked between the "last data word bit" and "stop bit" of the serial data.
 * |[4]     |EPE       |Even Parity Enable
 * |        |          |0 = Odd number of logic 1u2019s are transmitted or checked in the data word and parity bits.
 * |        |          |1 = Even number of logic 1u2019s are transmitted or checked in the data word and parity bits.
 * |        |          |This bit has effect only when PBE (parity bit enable) is set.
 * |[5]     |SPE       |Stick Parity Enable
 * |        |          |0 = Disable stick parity
 * |        |          |1 = When bits PBE and SPE are set u2018Stick Parityu2019 is enabled
 * |        |          |If EPE=0 the parity bit is transmitted and checked as always set, if EPE=1, the parity bit is transmitted and checked as always cleared
 * |[6]     |BCB       |Break Control Bit
 * |        |          |When this bit is set to logic 1, the serial data output (Tx) is forced to the u2018Spaceu2019 state (logic 0)
 * |        |          |Normal condition is serial data output is u2018Marku2019 state
 * |        |          |This bit acts only on Tx and has no effect on the transmitter logic.
 * @var UART_T::MODEM
 * Offset: 0x10  UART Modem Control Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |RTS       |RTS (Request-To-Send) Signal
 * |        |          |If IER.ATORTSEN=0, this bit controls whether RTS pin is active or not.
 * |        |          |1: Drive RTS inactive (=~RTSACTLV).
 * |        |          |0: Drive RTS active (=RTSACTLV).
 * |[4]     |LBMEN     |Loopback Mode Enable. 
 * |[9]     |RTSACTLV  |Request-to-Send (RTS)Active Trigger Level
 * |        |          |This bit can change the RTS trigger level.
 * |        |          |1=RTS is active low level.
 * |        |          |0=RTS is active high level
 * |[13]    |RTSSTS    |RTS Pin State(read only)
 * |        |          |This bit is the pin status of RTS.
 * @var UART_T::MODEMSTS
 * Offset: 0x14  UART Modem Status Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CTSDETF   |Detect CTS State Change Flag
 * |        |          |This bit is set whenever CTS input has state change
 * |        |          |It will generate Modem interrupt to CPU when IER.MS_IEN=1
 * |        |          |Note: This bit is cleared by writing 1 to itself.
 * |[4]     |CTSSTS    |CTS Pin Status (read only)
 * |        |          |This bit is the pin status of CTS.
 * |[8]     |CTSACTLV  |Clear-to-Send (CTS)Active Trigger Level
 * |        |          |This bit can change the CTS trigger level.
 * |        |          |1= CTS is active low level.
 * |        |          |0= CTS is active high level
 * @var UART_T::FIFOSTS
 * Offset: 0x18  UART FIFO Status Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RXOVIF    |Rx Overflow Error Interrupt Flag
 * |        |          |If the Rx FIFO (UART->DATA) is full, and an additional byte is received by the UART, an overflow condition will occur and set this bit to logic 1
 * |        |          |It will also generate a BUFERRIF event and interrupt if enabled.
 * |        |          |Note: This bit is cleared by writing 1 to itself.
 * |[4]     |PEF       |Parity Error Flag
 * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "parity bit", and is reset whenever the CPU writes 1 to this bit.
 * |[5]     |FEF       |Framing Error Flag
 * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "stop bit" (that is, the stop bit following the last data bit or parity bit is detected as a logic 0),and is reset whenever the CPU writes 1 to this bit.
 * |[6]     |BIF       |Break Interrupt Flag
 * |        |          |This bit is set to a logic 1 whenever the receive data input(Rx) is held in the "spaceu201D state (logic 0) for longer than a full word transmission time (that is, the total time of start bit + data bits + parity + stop bits)
 * |        |          |It is reset whenever the CPU writes 1 to this bit.
 * |[13:8]  |RXPTR     |Rx FIFO pointer (Read Only)
 * |        |          |This field returns the Rx FIFO buffer pointer
 * |        |          |It is the number of bytes available for read in the Rx FIFO
 * |        |          |When UART receives one byte from external device, RXPTR is incremented
 * |        |          |When one byte of Rx FIFO is read by CPU, RXPTR is decremented
 * |[14]    |RXEMPTY   |Receive FIFO Empty(Read Only)
 * |        |          |This bit indicates whether the Rx FIFO is empty or not.
 * |        |          |When the last byte of Rx FIFO has been read by CPU, hardware sets this bit high
 * |        |          |It will be cleared when UART receives any new data.
 * |[15]    |RXFULL    |Receive FIFO Full(Read Only)
 * |        |          |This bit indicates whether the Rx FIFO is full or not.
 * |        |          |This bit is set when RxFIFO is full; otherwise it is cleared by hardware.
 * |[21:16] |TXPTR     |Tx FIFO Pointer (Read Only)
 * |        |          |This field returns the Tx FIFO buffer pointer
 * |        |          |When CPU writes a byte into the TxFIFO, TXPTR is incremented
 * |        |          |When a byte from Tx FIFO is transferred to the Transmit Shift Register, TXPTR is decremented.
 * |[22]    |TXEMPTY   |Transmit FIFO Empty(Read Only)
 * |        |          |This bit indicates whether the Tx FIFO is empty or not.
 * |        |          |When the last byte of Tx FIFO has been transferred to Transmitter Shift Register, hardware sets this bit high
 * |        |          |It will be cleared after writing data to FIFO (Tx FIFO not empty).
 * |[23]    |TXFULL    |Transmit FIFO Full(Read Only)
 * |        |          |This bit indicates whether the Tx FIFO is full or not.
 * |        |          |This bit is set when TxFIFO is full; otherwise it is cleared by hardware
 * |        |          |TXFULL=0 indicates there is room to write more data to Tx FIFO.
 * |[24]    |TXOVIF    |Tx Overflow Error Interrupt Flag
 * |        |          |If the Tx FIFO (UART->DATA) is full, an additional write to UART->DATA will cause an overflow condition and set this bit to logic 1
 * |        |          |It will also generate a BUFERRIF event and interrupt if enabled.
 * |        |          |Note: This bit is cleared by writing 1 to itself.
 * |[28]    |TXEMPTYF  |Transmitter Empty (Read Only)
 * |        |          |Bit is set by hardware when Tx FIFO is empty and the STOP bit of the last byte has been transmitted.
 * |        |          |Bit is cleared automatically when Tx FIFO is not empty or the last byte transmission has not completed.
 * |        |          |Note: This bit is read only.
 * @var UART_T::INTSTS
 * Offset: 0x1C  UART Interrupt Status Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RDAIF     |Receive Data Available Interrupt Flag (Read Only).
 * |        |          |When the number of bytes in the Rx FIFO equals FCR.RFITL then the RDA_IF will be set
 * |        |          |If IER.RDA_IEN is enabled, the RDA interrupt will be generated.
 * |        |          |Note: This bit is read only and it will be cleared when the number of unread bytes of Rx FIFO drops below the threshold level (RFITL).
 * |[1]     |THREIF    |Transmit Holding Register Empty Interrupt Flag (Read Only).
 * |        |          |This bit is set when the last data of Tx FIFO is transferred to Transmitter Shift Register
 * |        |          |If IER.THRE_IEN is enabled, the THRE interrupt will be generated.
 * |        |          |Note: This bit is read only and it will be cleared when writing data into the Tx FIFO.
 * |[2]     |RLSIF     |Receive Line Status Interrupt Flag (Read Only).
 * |        |          |This bit is set when the Rx receive data has a parity, framing or break error (at least one of, FSR.BIF, FSR.FEF and FSR.PEF, is set)
 * |        |          |If IER.RLS_IEN is enabled, the RLS interrupt will be generated.
 * |        |          |Note: This bit is read only and reset to 0 when all bits of BIF, FEF and PEF are cleared.
 * |[3]     |MODEMIF   |MODEM Interrupt Flag (Read Only)
 * |        |          |This bit is set when the CTS pin has changed state (MSR.DCTSF=1)
 * |        |          |If IER.MS_IEN is enabled, a CPU interrupt request will be generated.
 * |        |          |Note: This bit is read only and reset when bit MSR.DCTSF is cleared by a write 1.
 * |[4]     |RXTOIF    |Time Out Interrupt Flag (Read Only)
 * |        |          |This bit is set when the Rx FIFO is not empty and no activity occurs in the Rx FIFO and the time out counter equal to TOIC
 * |        |          |If IER.RXTOIEN is enabled a CPU interrupt request will be generated.
 * |        |          |Note: This bit is read only and user can read FIFO to clear it.
 * |[5]     |BUFERRIF  |Buffer Error Interrupt Flag (Read Only)
 * |        |          |This bit is set when either the Tx or Rx FIFO overflows (FSR.TXOVIF or FSR.RXOVIF is set)
 * |        |          |When BUFERRIF is set, the serial transfer may be corrupted
 * |        |          |If IER.BUFERRIEN is enabled a CPU interrupt request will be generated.
 * |        |          |Note: This bit is cleared when both FSR.TXOVIF and FSR.RXOVIF are cleared. 
 * |[8]     |RDAINT    |Receive Data Available Interrupt Indicator to Interrupt Controller
 * |        |          |Logical AND of IER.RDAIEN and RDAIF
 * |[9]     |THREINT   |Transmit Holding Register Empty Interrupt Indicator to Interrupt Controller
 * |        |          |Logical AND of IER.THREIEN and THREIF
 * |[10]    |RLSINT    |Receive Line Status Interrupt Indicator to Interrupt Controller
 * |        |          |Logical AND of IER.RLSIEN and RLSIF
 * |[11]    |MODEMINT  |MODEM Status Interrupt Indicator to Interrupt
 * |        |          |Logical AND of IER.MSIEN and MODEMIF
 * |[12]    |RXTOINT   |Time Out Interrupt Indicator to Interrupt Controller
 * |        |          |Logical AND of IER.RXTOIEN and RXTOIF
 * |[13]    |BUFERRINT |Buffer Error Interrupt Indicator to Interrupt Controller
 * |        |          |Logical AND of IER.BUFERRIEN and BUFERRIF
 * |[18]    |DRLSIF    |DMA MODE Receive Line Status Interrupt Flag (Read Only)
 * |        |          |This bit is set when the Rx receive data has a parity, framing or break error (at least one of, UART_FIFOSTS.BIF, UART_FIFOSTS.FEF and UART_FIFOSTS.PEF, is set)
 * |        |          |If UART_INTEN.RLSIEN is enabled, the RLS interrupt will be generated.
 * |        |          |NOTE: This bit is read only and reset to 0 when all bits of BIF, FEF and PEF are cleared.
 * |[19]    |DMODIF    |DMA MODE MODEM Interrupt Flag (Read Only)
 * |        |          |This bit is set when the CTS pin has changed state (UART_MODEMSTS.DCTSF =1)
 * |        |          |If UART_INTEN.MODEMIEN is enabled, a CPU interrupt request will be generated.
 * |        |          |NOTE: This bit is read only and reset when bit UART_MODEMSTS.DCTSF is cleared by a write 1.
 * |[20]    |DRXTOIF   |DMA MODE Time Out Interrupt Flag (Read Only)
 * |        |          |This bit is set when the Rx FIFO is not empty and no activity occurs in the Rx FIFO and the time out counter equal to TOIC
 * |        |          |If UART_INTEN.RXTOIEN is enabled a CPU interrupt request will be generated.
 * |        |          |NOTE: This bit is read only and user can read FIFO to clear it.
 * |[21]    |DBERRIF   |DMA MODE Buffer Error Interrupt Flag (Read Only)
 * |        |          |This bit is set when either the Tx or Rx FIFO overflows (UART_FIFOSTS.TXOVIF or UART_FIFOSTS.RXOVIF is set)
 * |        |          |When BUFERRIF is set, the serial transfer may be corrupted
 * |        |          |If UART_INTEN.BUFERRIEN is enabled a CPU interrupt request will be generated.
 * |        |          |NOTE: This bit is cleared when both UART_FIFOSTS.TXOVIF and UART_FIFOSTS.RXOVIF are cleared. 
 * |[26]    |DRLSINT   |DMA MODE Receive Line Status Interrupt Indicator to Interrupt Controller
 * |        |          |Logical AND of UART_INTEN.DMARXEN or UART_INTEN.DMATXEN and DRLSIF.
 * |[27]    |DMODINT   |DMA MODE MODEM Status Interrupt Indicator to Interrupt
 * |        |          |Logical AND of UART_INTEN.DMARXEN or UART_INTEN.DMATXEN and DMODENIF.
 * |[28]    |DRXTOINT  |DMA MODE Time Out Interrupt Indicator to Interrupt Controller
 * |        |          |Logical AND of UART_INTEN.DMARXEN or UART_INTEN.DMATXEN and DRXTOIF.
 * |[29]    |DBERRINT  |DMA MODE Buffer Error Interrupt Indicator to Interrupt Controller
 * |        |          |Logical AND of UART_INTEN.DMARXEN or UART_INTEN.DMATXEN and DBERRIF.
 * @var UART_T::TOUT
 * Offset: 0x20  UART Time Out Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[6:0]   |TOIC      |Time Out Interrupt Comparator
 * |        |          |The time out counter resets and starts counting whenever the Rx FIFO receives a new data word
 * |        |          |Once the content of time out counter (TOIC) is equal to that of time out interrupt comparator (TOIC), a receiver time out interrupt (RXTOINT) is generated if IER.RXTOIEN is set
 * |        |          |A new incoming data word or RX FIFO empty clears RXTOIF
 * |        |          |The period of the time out counter is the baud rate.
 * @var UART_T::BAUD
 * Offset: 0x24  UART Baud Rate Divisor Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |BRD       |Baud Rate Divider. Refer to Table 5.15-5 for more information.
 * |[27:24] |EDIVM1    |Extra Divider for BAUD Rate Mode 1
 * |        |          |The baud rate divider M = EDIVM1+1.
 * |[28]    |BAUDM0    |BAUD Rate Mode Selection Bit 0
 * |        |          |0:M = EDIVM1+1, with restriction EDIVM1 >=8.
 * |        |          |1: M = 1, with restriction BRD[15:0]>= 3.
 * |        |          |Refer to Table 5.15-5 for more information.
 * |[29]    |BAUDM1    |BAUD Rate Mode Selection Bit 1
 * |        |          |The baud rate equation is: Baud Rate = UART_CLK / [ M * (BRD + 2) ] ; The default value of M is 16.
 * |        |          |0 = Disable divider X ( M = 16)
 * |        |          |1 = Enable divider X (M = EDIVM1+1, with EDIVM1 >=8).
 * |        |          |Refer to Table 5.15-5 for more information.
 * |        |          |NOTE: When in IrDA mode, this bit must disabled.
 */
    __IO uint32_t DAT;                   /*!< [0x0000] UART Receive/Transmit FIFO Register.                             */
    __IO uint32_t INTEN;                 /*!< [0x0004] UART Interrupt Enable Register.                                  */
    __IO uint32_t FIFO;                  /*!< [0x0008] UART FIFO Control Register.                                      */
    __IO uint32_t LINE;                  /*!< [0x000c] UART Line Control Register.                                      */
    __IO uint32_t MODEM;                 /*!< [0x0010] UART Modem Control Register.                                     */
    __IO uint32_t MODEMSTS;              /*!< [0x0014] UART Modem Status Register.                                      */
    __IO uint32_t FIFOSTS;               /*!< [0x0018] UART FIFO Status Register.                                       */
    __IO uint32_t INTSTS;                /*!< [0x001c] UART Interrupt Status Register.                                  */
    __IO uint32_t TOUT;                  /*!< [0x0020] UART Time Out Register                                           */
    __IO uint32_t BAUD;                  /*!< [0x0024] UART Baud Rate Divisor Register                                  */

} UART_T;

/**
    @addtogroup UART_CONST UART Bit Field Definition
    Constant Definitions for UART Controller
@{ */

#define UART_DAT_DAT_Pos                 (0)                                               /*!< UART_T::DAT: DAT Position              */
#define UART_DAT_DAT_Msk                 (0xfful << UART_DAT_DAT_Pos)                      /*!< UART_T::DAT: DAT Mask                  */

#define UART_INTEN_RDAIEN_Pos            (0)                                               /*!< UART_T::INTEN: RDAIEN Position         */
#define UART_INTEN_RDAIEN_Msk            (0x1ul << UART_INTEN_RDAIEN_Pos)                  /*!< UART_T::INTEN: RDAIEN Mask             */

#define UART_INTEN_THREIEN_Pos           (1)                                               /*!< UART_T::INTEN: THREIEN Position        */
#define UART_INTEN_THREIEN_Msk           (0x1ul << UART_INTEN_THREIEN_Pos)                 /*!< UART_T::INTEN: THREIEN Mask            */

#define UART_INTEN_RLSIEN_Pos            (2)                                               /*!< UART_T::INTEN: RLSIEN Position         */
#define UART_INTEN_RLSIEN_Msk            (0x1ul << UART_INTEN_RLSIEN_Pos)                  /*!< UART_T::INTEN: RLSIEN Mask             */

#define UART_INTEN_MODEMIEN_Pos          (3)                                               /*!< UART_T::INTEN: MODEMIEN Position       */
#define UART_INTEN_MODEMIEN_Msk          (0x1ul << UART_INTEN_MODEMIEN_Pos)                /*!< UART_T::INTEN: MODEMIEN Mask           */

#define UART_INTEN_RXTOIEN_Pos           (4)                                               /*!< UART_T::INTEN: RXTOIEN Position        */
#define UART_INTEN_RXTOIEN_Msk           (0x1ul << UART_INTEN_RXTOIEN_Pos)                 /*!< UART_T::INTEN: RXTOIEN Mask            */

#define UART_INTEN_BUFERRIEN_Pos         (5)                                               /*!< UART_T::INTEN: BUFERRIEN Position      */
#define UART_INTEN_BUFERRIEN_Msk         (0x1ul << UART_INTEN_BUFERRIEN_Pos)               /*!< UART_T::INTEN: BUFERRIEN Mask          */

#define UART_INTEN_TOCNTEN_Pos           (11)                                              /*!< UART_T::INTEN: TOCNTEN Position        */
#define UART_INTEN_TOCNTEN_Msk           (0x1ul << UART_INTEN_TOCNTEN_Pos)                 /*!< UART_T::INTEN: TOCNTEN Mask            */

#define UART_INTEN_ATORTSEN_Pos          (12)                                              /*!< UART_T::INTEN: ATORTSEN Position       */
#define UART_INTEN_ATORTSEN_Msk          (0x1ul << UART_INTEN_ATORTSEN_Pos)                /*!< UART_T::INTEN: ATORTSEN Mask           */

#define UART_INTEN_ATOCTSEN_Pos          (13)                                              /*!< UART_T::INTEN: ATOCTSEN Position       */
#define UART_INTEN_ATOCTSEN_Msk          (0x1ul << UART_INTEN_ATOCTSEN_Pos)                /*!< UART_T::INTEN: ATOCTSEN Mask           */

#define UART_INTEN_DMATXEN_Pos           (14)                                              /*!< UART_T::INTEN: DMATXEN Position        */
#define UART_INTEN_DMATXEN_Msk           (0x1ul << UART_INTEN_DMATXEN_Pos)                 /*!< UART_T::INTEN: DMATXEN Mask            */

#define UART_INTEN_DMARXEN_Pos           (15)                                              /*!< UART_T::INTEN: DMARXEN Position        */
#define UART_INTEN_DMARXEN_Msk           (0x1ul << UART_INTEN_DMARXEN_Pos)                 /*!< UART_T::INTEN: DMARXEN Mask            */

#define UART_FIFO_RXRST_Pos              (1)                                               /*!< UART_T::FIFO: RXRST Position           */
#define UART_FIFO_RXRST_Msk              (0x1ul << UART_FIFO_RXRST_Pos)                    /*!< UART_T::FIFO: RXRST Mask               */

#define UART_FIFO_TXRST_Pos              (2)                                               /*!< UART_T::FIFO: TXRST Position           */
#define UART_FIFO_TXRST_Msk              (0x1ul << UART_FIFO_TXRST_Pos)                    /*!< UART_T::FIFO: TXRST Mask               */

#define UART_FIFO_RFITL_Pos              (4)                                               /*!< UART_T::FIFO: RFITL Position           */
#define UART_FIFO_RFITL_Msk              (0xful << UART_FIFO_RFITL_Pos)                    /*!< UART_T::FIFO: RFITL Mask               */

#define UART_FIFO_RTSTRGLV_Pos           (16)                                              /*!< UART_T::FIFO: RTSTRGLV Position        */
#define UART_FIFO_RTSTRGLV_Msk           (0xful << UART_FIFO_RTSTRGLV_Pos)                 /*!< UART_T::FIFO: RTSTRGLV Mask            */

#define UART_LINE_WLS_Pos                (0)                                               /*!< UART_T::LINE: WLS Position             */
#define UART_LINE_WLS_Msk                (0x3ul << UART_LINE_WLS_Pos)                      /*!< UART_T::LINE: WLS Mask                 */

#define UART_LINE_NSB_Pos                (2)                                               /*!< UART_T::LINE: NSB Position             */
#define UART_LINE_NSB_Msk                (0x1ul << UART_LINE_NSB_Pos)                      /*!< UART_T::LINE: NSB Mask                 */

#define UART_LINE_PBE_Pos                (3)                                               /*!< UART_T::LINE: PBE Position             */
#define UART_LINE_PBE_Msk                (0x1ul << UART_LINE_PBE_Pos)                      /*!< UART_T::LINE: PBE Mask                 */

#define UART_LINE_EPE_Pos                (4)                                               /*!< UART_T::LINE: EPE Position             */
#define UART_LINE_EPE_Msk                (0x1ul << UART_LINE_EPE_Pos)                      /*!< UART_T::LINE: EPE Mask                 */

#define UART_LINE_SPE_Pos                (5)                                               /*!< UART_T::LINE: SPE Position             */
#define UART_LINE_SPE_Msk                (0x1ul << UART_LINE_SPE_Pos)                      /*!< UART_T::LINE: SPE Mask                 */

#define UART_LINE_BCB_Pos                (6)                                               /*!< UART_T::LINE: BCB Position             */
#define UART_LINE_BCB_Msk                (0x1ul << UART_LINE_BCB_Pos)                      /*!< UART_T::LINE: BCB Mask                 */

#define UART_MODEM_RTS_Pos               (1)                                               /*!< UART_T::MODEM: RTS Position            */
#define UART_MODEM_RTS_Msk               (0x1ul << UART_MODEM_RTS_Pos)                     /*!< UART_T::MODEM: RTS Mask                */

#define UART_MODEM_LBMEN_Pos             (4)                                               /*!< UART_T::MODEM: LBMEN Position          */
#define UART_MODEM_LBMEN_Msk             (0x1ul << UART_MODEM_LBMEN_Pos)                   /*!< UART_T::MODEM: LBMEN Mask              */

#define UART_MODEM_RTSACTLV_Pos          (9)                                               /*!< UART_T::MODEM: RTSACTLV Position       */
#define UART_MODEM_RTSACTLV_Msk          (0x1ul << UART_MODEM_RTSACTLV_Pos)                /*!< UART_T::MODEM: RTSACTLV Mask           */

#define UART_MODEM_RTSSTS_Pos            (13)                                              /*!< UART_T::MODEM: RTSSTS Position         */
#define UART_MODEM_RTSSTS_Msk            (0x1ul << UART_MODEM_RTSSTS_Pos)                  /*!< UART_T::MODEM: RTSSTS Mask             */

#define UART_MODEMSTS_CTSDETF_Pos        (0)                                               /*!< UART_T::MODEMSTS: CTSDETF Position     */
#define UART_MODEMSTS_CTSDETF_Msk        (0x1ul << UART_MODEMSTS_CTSDETF_Pos)              /*!< UART_T::MODEMSTS: CTSDETF Mask         */

#define UART_MODEMSTS_CTSSTS_Pos         (4)                                               /*!< UART_T::MODEMSTS: CTSSTS Position      */
#define UART_MODEMSTS_CTSSTS_Msk         (0x1ul << UART_MODEMSTS_CTSSTS_Pos)               /*!< UART_T::MODEMSTS: CTSSTS Mask          */

#define UART_MODEMSTS_CTSACTLV_Pos       (8)                                               /*!< UART_T::MODEMSTS: CTSACTLV Position    */
#define UART_MODEMSTS_CTSACTLV_Msk       (0x1ul << UART_MODEMSTS_CTSACTLV_Pos)             /*!< UART_T::MODEMSTS: CTSACTLV Mask        */

#define UART_FIFOSTS_RXOVIF_Pos          (0)                                               /*!< UART_T::FIFOSTS: RXOVIF Position       */
#define UART_FIFOSTS_RXOVIF_Msk          (0x1ul << UART_FIFOSTS_RXOVIF_Pos)                /*!< UART_T::FIFOSTS: RXOVIF Mask           */

#define UART_FIFOSTS_PEF_Pos             (4)                                               /*!< UART_T::FIFOSTS: PEF Position          */
#define UART_FIFOSTS_PEF_Msk             (0x1ul << UART_FIFOSTS_PEF_Pos)                   /*!< UART_T::FIFOSTS: PEF Mask              */

#define UART_FIFOSTS_FEF_Pos             (5)                                               /*!< UART_T::FIFOSTS: FEF Position          */
#define UART_FIFOSTS_FEF_Msk             (0x1ul << UART_FIFOSTS_FEF_Pos)                   /*!< UART_T::FIFOSTS: FEF Mask              */

#define UART_FIFOSTS_BIF_Pos             (6)                                               /*!< UART_T::FIFOSTS: BIF Position          */
#define UART_FIFOSTS_BIF_Msk             (0x1ul << UART_FIFOSTS_BIF_Pos)                   /*!< UART_T::FIFOSTS: BIF Mask              */

#define UART_FIFOSTS_RXPTR_Pos           (8)                                               /*!< UART_T::FIFOSTS: RXPTR Position        */
#define UART_FIFOSTS_RXPTR_Msk           (0x3ful << UART_FIFOSTS_RXPTR_Pos)                /*!< UART_T::FIFOSTS: RXPTR Mask            */

#define UART_FIFOSTS_RXEMPTY_Pos         (14)                                              /*!< UART_T::FIFOSTS: RXEMPTY Position      */
#define UART_FIFOSTS_RXEMPTY_Msk         (0x1ul << UART_FIFOSTS_RXEMPTY_Pos)               /*!< UART_T::FIFOSTS: RXEMPTY Mask          */

#define UART_FIFOSTS_RXFULL_Pos          (15)                                              /*!< UART_T::FIFOSTS: RXFULL Position       */
#define UART_FIFOSTS_RXFULL_Msk          (0x1ul << UART_FIFOSTS_RXFULL_Pos)                /*!< UART_T::FIFOSTS: RXFULL Mask           */

#define UART_FIFOSTS_TXPTR_Pos           (16)                                              /*!< UART_T::FIFOSTS: TXPTR Position        */
#define UART_FIFOSTS_TXPTR_Msk           (0x3ful << UART_FIFOSTS_TXPTR_Pos)                /*!< UART_T::FIFOSTS: TXPTR Mask            */

#define UART_FIFOSTS_TXEMPTY_Pos         (22)                                              /*!< UART_T::FIFOSTS: TXEMPTY Position      */
#define UART_FIFOSTS_TXEMPTY_Msk         (0x1ul << UART_FIFOSTS_TXEMPTY_Pos)               /*!< UART_T::FIFOSTS: TXEMPTY Mask          */

#define UART_FIFOSTS_TXFULL_Pos          (23)                                              /*!< UART_T::FIFOSTS: TXFULL Position       */
#define UART_FIFOSTS_TXFULL_Msk          (0x1ul << UART_FIFOSTS_TXFULL_Pos)                /*!< UART_T::FIFOSTS: TXFULL Mask           */

#define UART_FIFOSTS_TXOVIF_Pos          (24)                                              /*!< UART_T::FIFOSTS: TXOVIF Position       */
#define UART_FIFOSTS_TXOVIF_Msk          (0x1ul << UART_FIFOSTS_TXOVIF_Pos)                /*!< UART_T::FIFOSTS: TXOVIF Mask           */

#define UART_FIFOSTS_TXEMPTYF_Pos        (28)                                              /*!< UART_T::FIFOSTS: TXEMPTYF Position     */
#define UART_FIFOSTS_TXEMPTYF_Msk        (0x1ul << UART_FIFOSTS_TXEMPTYF_Pos)              /*!< UART_T::FIFOSTS: TXEMPTYF Mask         */

#define UART_INTSTS_RDAIF_Pos            (0)                                               /*!< UART_T::INTSTS: RDAIF Position         */
#define UART_INTSTS_RDAIF_Msk            (0x1ul << UART_INTSTS_RDAIF_Pos)                  /*!< UART_T::INTSTS: RDAIF Mask             */

#define UART_INTSTS_THREIF_Pos           (1)                                               /*!< UART_T::INTSTS: THREIF Position        */
#define UART_INTSTS_THREIF_Msk           (0x1ul << UART_INTSTS_THREIF_Pos)                 /*!< UART_T::INTSTS: THREIF Mask            */

#define UART_INTSTS_RLSIF_Pos            (2)                                               /*!< UART_T::INTSTS: RLSIF Position         */
#define UART_INTSTS_RLSIF_Msk            (0x1ul << UART_INTSTS_RLSIF_Pos)                  /*!< UART_T::INTSTS: RLSIF Mask             */

#define UART_INTSTS_MODEMIF_Pos          (3)                                               /*!< UART_T::INTSTS: MODEMIF Position       */
#define UART_INTSTS_MODEMIF_Msk          (0x1ul << UART_INTSTS_MODEMIF_Pos)                /*!< UART_T::INTSTS: MODEMIF Mask           */

#define UART_INTSTS_RXTOIF_Pos           (4)                                               /*!< UART_T::INTSTS: RXTOIF Position        */
#define UART_INTSTS_RXTOIF_Msk           (0x1ul << UART_INTSTS_RXTOIF_Pos)                 /*!< UART_T::INTSTS: RXTOIF Mask            */

#define UART_INTSTS_BUFERRIF_Pos         (5)                                               /*!< UART_T::INTSTS: BUFERRIF Position      */
#define UART_INTSTS_BUFERRIF_Msk         (0x1ul << UART_INTSTS_BUFERRIF_Pos)               /*!< UART_T::INTSTS: BUFERRIF Mask          */

#define UART_INTSTS_RDAINT_Pos           (8)                                               /*!< UART_T::INTSTS: RDAINT Position        */
#define UART_INTSTS_RDAINT_Msk           (0x1ul << UART_INTSTS_RDAINT_Pos)                 /*!< UART_T::INTSTS: RDAINT Mask            */

#define UART_INTSTS_THREINT_Pos          (9)                                               /*!< UART_T::INTSTS: THREINT Position       */
#define UART_INTSTS_THREINT_Msk          (0x1ul << UART_INTSTS_THREINT_Pos)                /*!< UART_T::INTSTS: THREINT Mask           */

#define UART_INTSTS_RLSINT_Pos           (10)                                              /*!< UART_T::INTSTS: RLSINT Position        */
#define UART_INTSTS_RLSINT_Msk           (0x1ul << UART_INTSTS_RLSINT_Pos)                 /*!< UART_T::INTSTS: RLSINT Mask            */

#define UART_INTSTS_MODEMINT_Pos         (11)                                              /*!< UART_T::INTSTS: MODEMINT Position      */
#define UART_INTSTS_MODEMINT_Msk         (0x1ul << UART_INTSTS_MODEMINT_Pos)               /*!< UART_T::INTSTS: MODEMINT Mask          */

#define UART_INTSTS_RXTOINT_Pos          (12)                                              /*!< UART_T::INTSTS: RXTOINT Position       */
#define UART_INTSTS_RXTOINT_Msk          (0x1ul << UART_INTSTS_RXTOINT_Pos)                /*!< UART_T::INTSTS: RXTOINT Mask           */

#define UART_INTSTS_BUFERRINT_Pos        (13)                                              /*!< UART_T::INTSTS: BUFERRINT Position     */
#define UART_INTSTS_BUFERRINT_Msk        (0x1ul << UART_INTSTS_BUFERRINT_Pos)              /*!< UART_T::INTSTS: BUFERRINT Mask         */

#define UART_INTSTS_DRLSIF_Pos           (18)                                              /*!< UART_T::INTSTS: DRLSIF Position        */
#define UART_INTSTS_DRLSIF_Msk           (0x1ul << UART_INTSTS_DRLSIF_Pos)                 /*!< UART_T::INTSTS: DRLSIF Mask            */

#define UART_INTSTS_DMODIF_Pos           (19)                                              /*!< UART_T::INTSTS: DMODIF Position        */
#define UART_INTSTS_DMODIF_Msk           (0x1ul << UART_INTSTS_DMODIF_Pos)                 /*!< UART_T::INTSTS: DMODIF Mask            */

#define UART_INTSTS_DRXTOIF_Pos          (20)                                              /*!< UART_T::INTSTS: DRXTOIF Position       */
#define UART_INTSTS_DRXTOIF_Msk          (0x1ul << UART_INTSTS_DRXTOIF_Pos)                /*!< UART_T::INTSTS: DRXTOIF Mask           */

#define UART_INTSTS_DBERRIF_Pos          (21)                                              /*!< UART_T::INTSTS: DBERRIF Position       */
#define UART_INTSTS_DBERRIF_Msk          (0x1ul << UART_INTSTS_DBERRIF_Pos)                /*!< UART_T::INTSTS: DBERRIF Mask           */

#define UART_INTSTS_DRLSINT_Pos          (26)                                              /*!< UART_T::INTSTS: DRLSINT Position       */
#define UART_INTSTS_DRLSINT_Msk          (0x1ul << UART_INTSTS_DRLSINT_Pos)                /*!< UART_T::INTSTS: DRLSINT Mask           */

#define UART_INTSTS_DMODINT_Pos          (27)                                              /*!< UART_T::INTSTS: DMODINT Position       */
#define UART_INTSTS_DMODINT_Msk          (0x1ul << UART_INTSTS_DMODINT_Pos)                /*!< UART_T::INTSTS: DMODINT Mask           */

#define UART_INTSTS_DRXTOINT_Pos         (28)                                              /*!< UART_T::INTSTS: DRXTOINT Position      */
#define UART_INTSTS_DRXTOINT_Msk         (0x1ul << UART_INTSTS_DRXTOINT_Pos)               /*!< UART_T::INTSTS: DRXTOINT Mask          */

#define UART_INTSTS_DBERRINT_Pos         (29)                                              /*!< UART_T::INTSTS: DBERRINT Position      */
#define UART_INTSTS_DBERRINT_Msk         (0x1ul << UART_INTSTS_DBERRINT_Pos)               /*!< UART_T::INTSTS: DBERRINT Mask          */

#define UART_TOUT_TOIC_Pos               (0)                                               /*!< UART_T::TOUT: TOIC Position            */
#define UART_TOUT_TOIC_Msk               (0x7ful << UART_TOUT_TOIC_Pos)                    /*!< UART_T::TOUT: TOIC Mask                */

#define UART_BAUD_BRD_Pos                (0)                                               /*!< UART_T::BAUD: BRD Position             */
#define UART_BAUD_BRD_Msk                (0xfffful << UART_BAUD_BRD_Pos)                   /*!< UART_T::BAUD: BRD Mask                 */

#define UART_BAUD_EDIVM1_Pos             (24)                                              /*!< UART_T::BAUD: EDIVM1 Position          */
#define UART_BAUD_EDIVM1_Msk             (0xful << UART_BAUD_EDIVM1_Pos)                   /*!< UART_T::BAUD: EDIVM1 Mask              */

#define UART_BAUD_BAUDM0_Pos             (28)                                              /*!< UART_T::BAUD: BAUDM0 Position          */
#define UART_BAUD_BAUDM0_Msk             (0x1ul << UART_BAUD_BAUDM0_Pos)                   /*!< UART_T::BAUD: BAUDM0 Mask              */

#define UART_BAUD_BAUDM1_Pos             (29)                                              /*!< UART_T::BAUD: BAUDM1 Position          */
#define UART_BAUD_BAUDM1_Msk             (0x1ul << UART_BAUD_BAUDM1_Pos)                   /*!< UART_T::BAUD: BAUDM1 Mask              */

/**@}*/ /* UART_CONST */
/**@}*/ /* end of UART register group */


/*---------------------- USB Device Controller -------------------------*/
/**
    @addtogroup USBD USB Device Controller(USBD)
    Memory Mapped Structure for USBD Controller
@{ */
 
typedef struct
{
	
/**
 * @var USBD_T::BUFSEG
 * Offset: 0x500  Endpoint Buffer Segmentation Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[8:3]   |BUFSEG    |Endpoint Buffer Segmentation
 * |        |          |It is used to indicate the offset address for each endpoint with the USB SRAM starting address The effective starting address of the endpoint is
 * |        |          |USBD_SRAM address + { BUFSEG, 3u2019b000}
 * |        |          |Where the USBD_SRAM address = USBD_BA+0x100h.
 * |        |          |Refer to the section 6.18.5.7 for the endpoint SRAM structure and its description.
 * @var USBD_T::MXPLD
 * Offset: 0x504  Endpoint Maximal Payload Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[8:0]   |MXPLD     |Maximal Payload
 * |        |          |Define the data length which is transmitted to host (IN token) or the actual data length which is received from the host (OUT token)
 * |        |          |It also used to indicate that the endpoint is ready to be transmitted in IN token or received in OUT token.
 * |        |          |(1) When the register is written by CPU,
 * |        |          |For IN token, the value of MXPLD is used to define the data length to be transmitted and indicate the data buffer is ready.
 * |        |          |For OUT token, it means that the controller is ready to receive data from the host and the value of MXPLD is the maximal data length comes from host.
 * |        |          |(2) When the register is read by CPU,
 * |        |          |For IN token, the value of MXPLD is indicated by the data length be transmitted to host
 * |        |          |For OUT token, the value of MXPLD is indicated the actual data length receiving from host.
 * |        |          |Note: Once MXPLD is written, the data packets will be transmitted/received immediately after IN/OUT token arrived.
 * @var USBD_T::CFG
 * Offset: 0x508  Endpoint Configuration Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |EPNUM     |Endpoint Number
 * |        |          |These bits are used to define the endpoint number of the current endpoint
 * |[4]     |ISOCH     |Isochronous Endpoint
 * |        |          |This bit is used to set the endpoint as Isochronous endpoint, no handshake.
 * |        |          |0 = No Isochronous endpoint.
 * |        |          |1 = Isochronous endpoint.
 * |[6:5]   |STATE     |Endpoint STATE
 * |        |          |00 = Endpoint is Disabled.
 * |        |          |01 = Out endpoint.
 * |        |          |10 = IN endpoint.
 * |        |          |11 = Undefined.
 * |[7]     |DSQSYNC   |Data Sequence Synchronization
 * |        |          |0 = DATA0 PID.
 * |        |          |1 = DATA1 PID.
 * |        |          |Note: It is used to specify the DATA0 or DATA1 PID in the following IN token transaction
 * |        |          |hardware will toggle automatically in IN token base on the bit.
 * |[9]     |CSTALL    |Clear STALL Response
 * |        |          |0 = Disable the device to clear the STALL handshake in setup stage.
 * |        |          |1 = Clear the device to response STALL handshake in setup stage.
 * @var USBD_T::CFGP
 * Offset: 0x50C  Endpoint Set Stall and Clear In/Out Ready Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CLRRDY    |Clear Ready
 * |        |          |When the USBD_MXPLDx register is set by user, it means that the endpoint is ready to transmit or receive data
 * |        |          |If the user wants to disable this transaction before the transaction start, users can set this bit to 1 to disable it and it is auto clear to 0.
 * |        |          |For IN token, write u20181u2019 to clear the IN token had ready to transmit the data to USB.
 * |        |          |For OUT token, write u20181u2019 to clear the OUT token had ready to receive the data from USB.
 * |        |          |This bit is write 1 only and is always 0 when it is read back.
 * |[1]     |SSTALL    |Set STALL
 * |        |          |0 = Disable the device to response STALL.
 * |        |          |1 = Set the device to respond STALL automatically.
 */
 
    __IO uint32_t BUFSEG;        /* Offset: 0x500/0x510/0x520/0x530/0x540/0x550/0x560/0x570  Endpoint 0~7 Buffer Segmentation Register */
    __IO uint32_t MXPLD;         /* Offset: 0x504/0x514/0x524/0x534/0x544/0x554/0x564/0x574  Endpoint 0~7 Maximal Payload Register */
    __IO uint32_t CFG;           /* Offset: 0x508/0x518/0x528/0x538/0x548/0x558/0x568/0x578  Endpoint 0~7 Configuration Register */
    __IO uint32_t CFGP;          /* Offset: 0x50C/0x51C/0x52C/0x53C/0x54C/0x55C/0x56C/0x57C  Endpoint 0~7 Set Stall and Clear In/Out Ready Control Register */
} USBD_EP_T;

typedef struct
{
	
/**
 * @var USBD_T::INTEN
 * Offset: 0x00  USB Device Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BUSIEN    |Bus Event Interrupt Enable Bit
 * |        |          |0 = BUS event interrupt Disabled.
 * |        |          |1 = BUS event interrupt Enabled.
 * |[1]     |USBIEN    |USB Event Interrupt Enable Bit
 * |        |          |0 = USB event interrupt Disabled.
 * |        |          |1 = USB event interrupt Enabled.
 * |[2]     |VBDETIEN  |VBUS Detection Interrupt Enable Bit
 * |        |          |0 = VBUS detection Interrupt Disabled.
 * |        |          |1 = VBUS detection Interrupt Enabled.
 * |[3]     |NEVWKIEN  |USB No-event-wake-up Interrupt Enable Bit
 * |        |          |0 = No-event-wake-up Interrupt Disabled.
 * |        |          |1 = No-event-wake-up Interrupt Enabled.
 * |[4]     |SOFIEN    |Start of Frame Interrupt Enable Bit
 * |        |          |0 = SOF Interrupt Disabled.
 * |        |          |1 = SOF Interrupt Enabled.
 * |[8]     |WKEN      |Wake-up Function Enable Bit
 * |        |          |0 = USB wake-up function Disabled.
 * |        |          |1 = USB wake-up function Enabled.
 * |[15]    |INNAKEN   |Active NAK Function and Its Status in IN Token
 * |        |          |0 = When device responds NAK after receiving IN token, IN NAK status will not be updated to USBD_EPSTS0 and USBD_EPSTS1, so that the USB interrupt event will not be asserted.
 * |        |          |1 = IN NAK status will be updated to USBD_EPSTS0 and USBD_EPSTS1 and the USB interrupt event will be asserted, when the device responds NAK after receiving IN token.
 * @var USBD_T::INTSTS
 * Offset: 0x04  USB Device Interrupt Event Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BUSIF     |BUS Interrupt Status
 * |        |          |The BUS event means that there is one of the suspend or the resume function in the bus.
 * |        |          |0 = No BUS event occurred.
 * |        |          |1 = Bus event occurred; check USBD_ATTR[3:0] to know which kind of bus event was occurred, cleared by writing 1 to USBD_INTSTS[0].
 * |[1]     |USBIF     |USB Event Interrupt Status
 * |        |          |The USB event includes the SETUP Token, IN Token, OUT ACK, ISO IN, or ISO OUT events in the bus.
 * |        |          |0 = No USB event occurred.
 * |        |          |1 = USB event occurred, check EPSTS (USBD_EPSTS0 and USBD_EPSTS1) to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[1] or EPEVT11~0 (USBD_INTSTS[27:16) and SETUP (USBD_INTSTS[31]).
 * |[2]     |VBDETIF   |VBUS Detection Interrupt Status
 * |        |          |0 = There is not attached/detached event in the USB.
 * |        |          |1 = There is attached/detached event in the USB bus and it is cleared by writing 1 to USBD_INTSTS[2].
 * |[3]     |NEVWKIF   |No-event-wake-up Interrupt Status
 * |        |          |0 = NEVWK event does not occur.
 * |        |          |1 = No-event-wake-up event occurred, cleared by writing 1 to USBD_INTSTS[3].
 * |[4]     |SOFIF     |Start of Frame Interrupt Status
 * |        |          |0 = SOF event does not occur.
 * |        |          |1 = SOF event occurred, cleared by write 1 to USBD_INTSTS[4].
 * |[16]    |EPEVT0    |Endpoint 0u2019s USB Event Status
 * |        |          |0 = No event occurred in endpoint 0.
 * |        |          |1 = USB event occurred on Endpoint 0, check USBD_EPSTS0[3:0] to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[16] or USBD_INTSTS[1].
 * |[17]    |EPEVT1    |Endpoint 1u2019s USB Event Status
 * |        |          |0 = No event occurred in endpoint 1.
 * |        |          |1 = USB event occurred on Endpoint 1, check USBD_EPSTS0[7:4] to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[17] or USBD_INTSTS[1].
 * |[18]    |EPEVT2    |Endpoint 2u2019s USB Event Status
 * |        |          |0 = No event occurred in endpoint 2.
 * |        |          |1 = USB event occurred on Endpoint 2, check USBD_EPSTS0[11:8] to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[18] or USBD_INTSTS[1].
 * |[19]    |EPEVT3    |Endpoint 3u2019s USB Event Status
 * |        |          |0 = No event occurred in endpoint 3.
 * |        |          |1 = USB event occurred on Endpoint 3, check USBD_EPSTS0[15:12] to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[19] or USBD_INTSTS[1].
 * |[20]    |EPEVT4    |Endpoint 4u2019s USB Event Status
 * |        |          |0 = No event occurred in endpoint 4.
 * |        |          |1 = USB event occurred on Endpoint 4, check USBD_EPSTS0[19:16] to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[20] or USBD_INTSTS[1].
 * |[21]    |EPEVT5    |Endpoint 5u2019s USB Event Status
 * |        |          |0 = No event occurred in endpoint 5.
 * |        |          |1 = USB event occurred on Endpoint 5, check USBD_EPSTS0[23:20] to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[21] or USBD_INTSTS[1].
 * |[22]    |EPEVT6    |Endpoint 6u2019s USB Event Status
 * |        |          |0 = No event occurred in endpoint 6.
 * |        |          |1 = USB event occurred on Endpoint 6, check USBD_EPSTS0[27:24] to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[22] or USBD_INTSTS[1].
 * |[23]    |EPEVT7    |Endpoint 7u2019s USB Event Status
 * |        |          |0 = No event occurred in endpoint 7.
 * |        |          |1 = USB event occurred on Endpoint 7, check USBD_EPSTS0[31:28] to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[23] or USBD_INTSTS[1].
 * |[24]    |EPEVT8    |Endpoint 8u2019s USB Event Status
 * |        |          |0 = No event occurred in endpoint 8.
 * |        |          |1 = USB event occurred on Endpoint 8, check USBD_EPSTS1[3 :0] to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[24] or USBD_INTSTS[1].
 * |[25]    |EPEVT9    |Endpoint 9u2019s USB Event Status
 * |        |          |0 = No event occurred in endpoint 9.
 * |        |          |1 = USB event occurred on Endpoint 9, check USBD_EPSTS1[7 :4] to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[25] or USBD_INTSTS[1].
 * |[26]    |EPEVT10   |Endpoint 10u2019s USB Event Status
 * |        |          |0 = No event occurred in endpoint 10.
 * |        |          |1 = USB event occurred on Endpoint 10, check USBD_EPSTS1[11 :8] to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[26] or USBD_INTSTS[1].
 * |[27]    |EPEVT11   |Endpoint 11u2019s USB Event Status
 * |        |          |0 = No event occurred in endpoint 11.
 * |        |          |1 = USB event occurred on Endpoint 11, check USBD_EPSTS1[15:12] to know which kind of USB event was occurred, cleared by writing 1 to USBD_INTSTS[27] or USBD_INTSTS[1].
 * |[31]    |SETUP     |Setup Event Status
 * |        |          |0 = No Setup event.
 * |        |          |1 = Setup event occurred, cleared by writing 1 to USBD_INTSTS[31].
 * @var USBD_T::FADDR
 * Offset: 0x08  USB Device Function Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[6:0]   |FADDR     |USB Device Function Address
 * @var USBD_T::EPSTS
 * Offset: 0x0C  USB Device Endpoint Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7]     |OV        |Overrun
 * |        |          |It indicates that the received data is over the maximum payload number or not.
 * |        |          |0 = No overrun.
 * |        |          |1 = Out Data is more than the Max Payload in MXPLD register or the Setup Data is more than 8 Bytes.
 * @var USBD_T::ATTR
 * Offset: 0x10  USB Device Bus Status and Attribution Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |USBRST    |USB Reset Status
 * |        |          |0 = Bus no reset.
 * |        |          |1 = Bus reset when SE0 (single-ended 0) more than 2.5us.
 * |        |          |Note: This bit is read only.
 * |[1]     |SUSPEND   |Suspend Status
 * |        |          |0 = Bus no suspend.
 * |        |          |1 = Bus idle more than 3ms, either cable is plugged off or host is sleeping.
 * |        |          |Note: This bit is read only.
 * |[2]     |RESUME    |Resume Status
 * |        |          |0 = No bus resume.
 * |        |          |1 = Resume from suspend.
 * |        |          |Note: This bit is read only.
 * |[3]     |TOUT      |Time-out Status
 * |        |          |0 = No time-out.
 * |        |          |1 = No Bus response more than 18 bits time.
 * |        |          |Note: This bit is read only.
 * |[4]     |PHYEN     |PHY Transceiver Function Enable Bit
 * |        |          |0 = PHY transceiver function Disabled.
 * |        |          |1 = PHY transceiver function Enabled.
 * |[5]     |RWAKEUP   |Remote Wake-up
 * |        |          |0 = Release the USB bus from K state.
 * |        |          |1 = Force USB bus to K (USB_D+ low, USB_D-: high) state, used for remote wake-up.
 * |[7]     |USBEN     |USB Controller Enable Bit
 * |        |          |0 = USB Controller Disabled.
 * |        |          |1 = USB Controller Enabled.
 * |[8]     |DPPUEN    |Pull-up Resistor on USB_DP Enable Bit
 * |        |          |0 = Pull-up resistor in USB_D+ bus Disabled.
 * |        |          |1 = Pull-up resistor in USB_D+ bus Active.
 * |[10]    |BYTEM     |CPU Access USB SRAM Size Mode Selection
 * |        |          |0 = Word mode: The size of the transfer from CPU to USB SRAM can be Word only.
 * |        |          |1 = Byte mode: The size of the transfer from CPU to USB SRAM can be Byte only.
 * @var USBD_T::VBUSDET
 * Offset: 0x14  USB Device VBUS Detection Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |VBUSDET   |Device VBUS Detection
 * |        |          |0 = Controller is not attached to the USB host.
 * |        |          |1 = Controller is attached to the USB host.
 * @var USBD_T::STBUFSEG
 * Offset: 0x18  SETUP Token Buffer Segmentation Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[8:3]   |STBUFSEG  |SETUP Token Buffer Segmentation
 * |        |          |It is used to indicate the offset address for the SETUP token with the USB Device SRAM starting address The effective starting address is
 * |        |          |USBD_SRAM address + {STBUFSEG, 3u2019b000}
 * |        |          |Where the USBD_SRAM address = USBD_BA+0x100h.
 * |        |          |Note: It is used for SETUP token only.
 * @var USBD_T::EPSTS0
 * Offset: 0x20  USB Device Endpoint Status Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |EPSTS0    |Endpoint 0 Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |0000 = In ACK.
 * |        |          |0001 = In NAK.
 * |        |          |0010 = Out Packet Data0 ACK.
 * |        |          |0011 = Setup ACK.
 * |        |          |0110 = Out Packet Data1 ACK.
 * |        |          |0111 = Isochronous transfer end.
 * |[7:4]   |EPSTS1    |Endpoint 1 Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |0000 = In ACK.
 * |        |          |0001 = In NAK.
 * |        |          |0010 = Out Packet Data0 ACK.
 * |        |          |0011 = Setup ACK.
 * |        |          |0110 = Out Packet Data1 ACK.
 * |        |          |0111 = Isochronous transfer end.
 * |[11:8]  |EPSTS2    |Endpoint 2 Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |0000 = In ACK.
 * |        |          |0001 = In NAK.
 * |        |          |0010 = Out Packet Data0 ACK.
 * |        |          |0011 = Setup ACK.
 * |        |          |0110 = Out Packet Data1 ACK.
 * |        |          |0111 = Isochronous transfer end.
 * |[15:12] |EPSTS3    |Endpoint 3 Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |0000 = In ACK.
 * |        |          |0001 = In NAK.
 * |        |          |0010 = Out Packet Data0 ACK.
 * |        |          |0011 = Setup ACK.
 * |        |          |0110 = Out Packet Data1 ACK.
 * |        |          |0111 = Isochronous transfer end.
 * |[19:16] |EPSTS4    |Endpoint 4 Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |0000 = In ACK.
 * |        |          |0001 = In NAK.
 * |        |          |0010 = Out Packet Data0 ACK.
 * |        |          |0011 = Setup ACK.
 * |        |          |0110 = Out Packet Data1 ACK.
 * |        |          |0111 = Isochronous transfer end.
 * |[23:20] |EPSTS5    |Endpoint 5 Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |0000 = In ACK.
 * |        |          |0001 = In NAK.
 * |        |          |0010 = Out Packet Data0 ACK.
 * |        |          |0011 = Setup ACK.
 * |        |          |0110 = Out Packet Data1 ACK.
 * |        |          |0111 = Isochronous transfer end.
 * |[27:24] |EPSTS6    |Endpoint 6 Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |0000 = In ACK.
 * |        |          |0001 = In NAK.
 * |        |          |0010 = Out Packet Data0 ACK.
 * |        |          |0011 = Setup ACK.
 * |        |          |0110 = Out Packet Data1 ACK.
 * |        |          |0111 = Isochronous transfer end.
 * |[31:28] |EPSTS7    |Endpoint 7 Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |0000 = In ACK.
 * |        |          |0001 = In NAK.
 * |        |          |0010 = Out Packet Data0 ACK.
 * |        |          |0011 = Setup ACK.
 * |        |          |0110 = Out Packet Data1 ACK.
 * |        |          |0111 = Isochronous transfer end.
 * @var USBD_T::EPSTS1
 * Offset: 0x24  USB Device Endpoint Status Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |EPSTS8    |Endpoint 8 Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |0000 = In ACK.
 * |        |          |0001 = In NAK.
 * |        |          |0010 = Out Packet Data0 ACK.
 * |        |          |0011 = Setup ACK.
 * |        |          |0110 = Out Packet Data1 ACK.
 * |        |          |0111 = Isochronous transfer end.
 * |[7:4]   |EPSTS9    |Endpoint 9 Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |0000 = In ACK.
 * |        |          |0001 = In NAK.
 * |        |          |0010 = Out Packet Data0 ACK.
 * |        |          |0011 = Setup ACK.
 * |        |          |0110 = Out Packet Data1 ACK.
 * |        |          |0111 = Isochronous transfer end.
 * |[11:8]  |EPSTS10   |Endpoint 10 Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |0000 = In ACK.
 * |        |          |0001 = In NAK.
 * |        |          |0010 = Out Packet Data0 ACK.
 * |        |          |0011 = Setup ACK.
 * |        |          |0110 = Out Packet Data1 ACK.
 * |        |          |0111 = Isochronous transfer end.
 * |[15:12] |EPSTS11   |Endpoint 11 Status
 * |        |          |These bits are used to indicate the current status of this endpoint
 * |        |          |0000 = In ACK.
 * |        |          |0001 = In NAK.
 * |        |          |0010 = Out Packet Data0 ACK.
 * |        |          |0011 = Setup ACK.
 * |        |          |0110 = Out Packet Data1 ACK.
 * |        |          |0111 = Isochronous transfer end.
 * @var USBD_T::FN
 * Offset: 0x8C  USB Frame Number Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[10:0]  |FN        |Frame Number
 * |        |          |These bits contain the 11-bits frame number in the last received SOF packet.
 * @var USBD_T::SE0
 * Offset: 0x90  USB Device Drive SE0 Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SE0       |Drive Single Ended Zero in USB Bus
 * |        |          |The Single Ended Zero (SE0) is when both lines (USB_D+ and USB_D-) are being pulled low.
 * |        |          |0 = Normal operation.
 * |        |          |1 = Force USB PHY transceiver to drive SE0.
 */
	__IO uint32_t INTEN;                 /*!< [0x0000] USB Device Interrupt Enable Register                             */
    __IO uint32_t INTSTS;                /*!< [0x0004] USB Device Interrupt Event Status Register                       */
    __IO uint32_t FADDR;                 /*!< [0x0008] USB Device Function Address Register                             */
    __I  uint32_t EPSTS;                 /*!< [0x000c] USB Device Endpoint Status Register                              */
    __IO uint32_t ATTR;                  /*!< [0x0010] USB Device Bus Status and Attribution Register                   */
    __I  uint32_t VBUSDET;               /*!< [0x0014] USB Device VBUS Detection Register                               */
    __IO uint32_t STBUFSEG;              /*!< [0x0018] SETUP Token Buffer Segmentation Register                         */
    __I  uint32_t RESERVE0[1];
    __I  uint32_t EPSTS0;                /*!< [0x0020] USB Device Endpoint Status Register 0                            */
    __I  uint32_t EPSTS1;                /*!< [0x0024] USB Device Endpoint Status Register 1                            */
    __I  uint32_t RESERVE1[25];
    __I  uint32_t FN;                    /*!< [0x008c] USB Frame Number Register                                        */
    __IO uint32_t SE0;                   /*!< [0x0090] USB Device Drive SE0 Control Register                            */
    __I  uint32_t RESERVE2[283];
	USBD_EP_T     EP[12];                /* Offset: 0x500 ~ 0x57C  USB End Point 0 ~ 7 Configuration Register           */

} USBD_T;

/**
    @addtogroup USBD_CONST USBD Bit Field Definition
    Constant Definitions for USBD Controller
@{ */

#define USBD_INTEN_BUSIEN_Pos            (0)                                               /*!< USBD_T::INTEN: BUSIEN Position         */
#define USBD_INTEN_BUSIEN_Msk            (0x1ul << USBD_INTEN_BUSIEN_Pos)                  /*!< USBD_T::INTEN: BUSIEN Mask             */

#define USBD_INTEN_USBIEN_Pos            (1)                                               /*!< USBD_T::INTEN: USBIEN Position         */
#define USBD_INTEN_USBIEN_Msk            (0x1ul << USBD_INTEN_USBIEN_Pos)                  /*!< USBD_T::INTEN: USBIEN Mask             */

#define USBD_INTEN_VBDETIEN_Pos          (2)                                               /*!< USBD_T::INTEN: VBDETIEN Position       */
#define USBD_INTEN_VBDETIEN_Msk          (0x1ul << USBD_INTEN_VBDETIEN_Pos)                /*!< USBD_T::INTEN: VBDETIEN Mask           */

#define USBD_INTEN_NEVWKIEN_Pos          (3)                                               /*!< USBD_T::INTEN: NEVWKIEN Position       */
#define USBD_INTEN_NEVWKIEN_Msk          (0x1ul << USBD_INTEN_NEVWKIEN_Pos)                /*!< USBD_T::INTEN: NEVWKIEN Mask           */

#define USBD_INTEN_SOFIEN_Pos            (4)                                               /*!< USBD_T::INTEN: SOFIEN Position         */
#define USBD_INTEN_SOFIEN_Msk            (0x1ul << USBD_INTEN_SOFIEN_Pos)                  /*!< USBD_T::INTEN: SOFIEN Mask             */

#define USBD_INTEN_WKEN_Pos              (8)                                               /*!< USBD_T::INTEN: WKEN Position           */
#define USBD_INTEN_WKEN_Msk              (0x1ul << USBD_INTEN_WKEN_Pos)                    /*!< USBD_T::INTEN: WKEN Mask               */

#define USBD_INTEN_INNAKEN_Pos           (15)                                              /*!< USBD_T::INTEN: INNAKEN Position        */
#define USBD_INTEN_INNAKEN_Msk           (0x1ul << USBD_INTEN_INNAKEN_Pos)                 /*!< USBD_T::INTEN: INNAKEN Mask            */

#define USBD_INTSTS_BUSIF_Pos            (0)                                               /*!< USBD_T::INTSTS: BUSIF Position         */
#define USBD_INTSTS_BUSIF_Msk            (0x1ul << USBD_INTSTS_BUSIF_Pos)                  /*!< USBD_T::INTSTS: BUSIF Mask             */

#define USBD_INTSTS_USBIF_Pos            (1)                                               /*!< USBD_T::INTSTS: USBIF Position         */
#define USBD_INTSTS_USBIF_Msk            (0x1ul << USBD_INTSTS_USBIF_Pos)                  /*!< USBD_T::INTSTS: USBIF Mask             */

#define USBD_INTSTS_VBDETIF_Pos          (2)                                               /*!< USBD_T::INTSTS: VBDETIF Position       */
#define USBD_INTSTS_VBDETIF_Msk          (0x1ul << USBD_INTSTS_VBDETIF_Pos)                /*!< USBD_T::INTSTS: VBDETIF Mask           */

#define USBD_INTSTS_NEVWKIF_Pos          (3)                                               /*!< USBD_T::INTSTS: NEVWKIF Position       */
#define USBD_INTSTS_NEVWKIF_Msk          (0x1ul << USBD_INTSTS_NEVWKIF_Pos)                /*!< USBD_T::INTSTS: NEVWKIF Mask           */

#define USBD_INTSTS_SOFIF_Pos            (4)                                               /*!< USBD_T::INTSTS: SOFIF Position         */
#define USBD_INTSTS_SOFIF_Msk            (0x1ul << USBD_INTSTS_SOFIF_Pos)                  /*!< USBD_T::INTSTS: SOFIF Mask             */

#define USBD_INTSTS_EPEVT0_Pos           (16)                                              /*!< USBD_T::INTSTS: EPEVT0 Position        */
#define USBD_INTSTS_EPEVT0_Msk           (0x1ul << USBD_INTSTS_EPEVT0_Pos)                 /*!< USBD_T::INTSTS: EPEVT0 Mask            */

#define USBD_INTSTS_EPEVT1_Pos           (17)                                              /*!< USBD_T::INTSTS: EPEVT1 Position        */
#define USBD_INTSTS_EPEVT1_Msk           (0x1ul << USBD_INTSTS_EPEVT1_Pos)                 /*!< USBD_T::INTSTS: EPEVT1 Mask            */

#define USBD_INTSTS_EPEVT2_Pos           (18)                                              /*!< USBD_T::INTSTS: EPEVT2 Position        */
#define USBD_INTSTS_EPEVT2_Msk           (0x1ul << USBD_INTSTS_EPEVT2_Pos)                 /*!< USBD_T::INTSTS: EPEVT2 Mask            */

#define USBD_INTSTS_EPEVT3_Pos           (19)                                              /*!< USBD_T::INTSTS: EPEVT3 Position        */
#define USBD_INTSTS_EPEVT3_Msk           (0x1ul << USBD_INTSTS_EPEVT3_Pos)                 /*!< USBD_T::INTSTS: EPEVT3 Mask            */

#define USBD_INTSTS_EPEVT4_Pos           (20)                                              /*!< USBD_T::INTSTS: EPEVT4 Position        */
#define USBD_INTSTS_EPEVT4_Msk           (0x1ul << USBD_INTSTS_EPEVT4_Pos)                 /*!< USBD_T::INTSTS: EPEVT4 Mask            */

#define USBD_INTSTS_EPEVT5_Pos           (21)                                              /*!< USBD_T::INTSTS: EPEVT5 Position        */
#define USBD_INTSTS_EPEVT5_Msk           (0x1ul << USBD_INTSTS_EPEVT5_Pos)                 /*!< USBD_T::INTSTS: EPEVT5 Mask            */

#define USBD_INTSTS_EPEVT6_Pos           (22)                                              /*!< USBD_T::INTSTS: EPEVT6 Position        */
#define USBD_INTSTS_EPEVT6_Msk           (0x1ul << USBD_INTSTS_EPEVT6_Pos)                 /*!< USBD_T::INTSTS: EPEVT6 Mask            */

#define USBD_INTSTS_EPEVT7_Pos           (23)                                              /*!< USBD_T::INTSTS: EPEVT7 Position        */
#define USBD_INTSTS_EPEVT7_Msk           (0x1ul << USBD_INTSTS_EPEVT7_Pos)                 /*!< USBD_T::INTSTS: EPEVT7 Mask            */

#define USBD_INTSTS_EPEVT8_Pos           (24)                                              /*!< USBD_T::INTSTS: EPEVT8 Position        */
#define USBD_INTSTS_EPEVT8_Msk           (0x1ul << USBD_INTSTS_EPEVT8_Pos)                 /*!< USBD_T::INTSTS: EPEVT8 Mask            */

#define USBD_INTSTS_EPEVT9_Pos           (25)                                              /*!< USBD_T::INTSTS: EPEVT9 Position        */
#define USBD_INTSTS_EPEVT9_Msk           (0x1ul << USBD_INTSTS_EPEVT9_Pos)                 /*!< USBD_T::INTSTS: EPEVT9 Mask            */

#define USBD_INTSTS_EPEVT10_Pos          (26)                                              /*!< USBD_T::INTSTS: EPEVT10 Position       */
#define USBD_INTSTS_EPEVT10_Msk          (0x1ul << USBD_INTSTS_EPEVT10_Pos)                /*!< USBD_T::INTSTS: EPEVT10 Mask           */

#define USBD_INTSTS_EPEVT11_Pos          (27)                                              /*!< USBD_T::INTSTS: EPEVT11 Position       */
#define USBD_INTSTS_EPEVT11_Msk          (0x1ul << USBD_INTSTS_EPEVT11_Pos)                /*!< USBD_T::INTSTS: EPEVT11 Mask           */

#define USBD_INTSTS_SETUP_Pos            (31)                                              /*!< USBD_T::INTSTS: SETUP Position         */
#define USBD_INTSTS_SETUP_Msk            (0x1ul << USBD_INTSTS_SETUP_Pos)                  /*!< USBD_T::INTSTS: SETUP Mask             */

#define USBD_FADDR_FADDR_Pos             (0)                                               /*!< USBD_T::FADDR: FADDR Position          */
#define USBD_FADDR_FADDR_Msk             (0x7ful << USBD_FADDR_FADDR_Pos)                  /*!< USBD_T::FADDR: FADDR Mask              */

#define USBD_EPSTS_OV_Pos                (7)                                               /*!< USBD_T::EPSTS: OV Position             */
#define USBD_EPSTS_OV_Msk                (0x1ul << USBD_EPSTS_OV_Pos)                      /*!< USBD_T::EPSTS: OV Mask                 */

#define USBD_ATTR_USBRST_Pos             (0)                                               /*!< USBD_T::ATTR: USBRST Position          */
#define USBD_ATTR_USBRST_Msk             (0x1ul << USBD_ATTR_USBRST_Pos)                   /*!< USBD_T::ATTR: USBRST Mask              */

#define USBD_ATTR_SUSPEND_Pos            (1)                                               /*!< USBD_T::ATTR: SUSPEND Position         */
#define USBD_ATTR_SUSPEND_Msk            (0x1ul << USBD_ATTR_SUSPEND_Pos)                  /*!< USBD_T::ATTR: SUSPEND Mask             */

#define USBD_ATTR_RESUME_Pos             (2)                                               /*!< USBD_T::ATTR: RESUME Position          */
#define USBD_ATTR_RESUME_Msk             (0x1ul << USBD_ATTR_RESUME_Pos)                   /*!< USBD_T::ATTR: RESUME Mask              */

#define USBD_ATTR_TOUT_Pos               (3)                                               /*!< USBD_T::ATTR: TOUT Position            */
#define USBD_ATTR_TOUT_Msk               (0x1ul << USBD_ATTR_TOUT_Pos)                     /*!< USBD_T::ATTR: TOUT Mask                */

#define USBD_ATTR_PHYEN_Pos              (4)                                               /*!< USBD_T::ATTR: PHYEN Position           */
#define USBD_ATTR_PHYEN_Msk              (0x1ul << USBD_ATTR_PHYEN_Pos)                    /*!< USBD_T::ATTR: PHYEN Mask               */

#define USBD_ATTR_RWAKEUP_Pos            (5)                                               /*!< USBD_T::ATTR: RWAKEUP Position         */
#define USBD_ATTR_RWAKEUP_Msk            (0x1ul << USBD_ATTR_RWAKEUP_Pos)                  /*!< USBD_T::ATTR: RWAKEUP Mask             */

#define USBD_ATTR_USBEN_Pos              (7)                                               /*!< USBD_T::ATTR: USBEN Position           */
#define USBD_ATTR_USBEN_Msk              (0x1ul << USBD_ATTR_USBEN_Pos)                    /*!< USBD_T::ATTR: USBEN Mask               */

#define USBD_ATTR_DPPUEN_Pos             (8)                                               /*!< USBD_T::ATTR: DPPUEN Position          */
#define USBD_ATTR_DPPUEN_Msk             (0x1ul << USBD_ATTR_DPPUEN_Pos)                   /*!< USBD_T::ATTR: DPPUEN Mask              */

#define USBD_ATTR_PWRDN_Pos              (9)                                               /*!< USBD_T::ATTR: PWRDN Position           */
#define USBD_ATTR_PWRDN_Msk              (0x1ul << USBD_ATTR_PWRDN_Pos)                    /*!< USBD_T::ATTR: PWRDN Mask               */

#define USBD_ATTR_BYTEM_Pos              (10)                                              /*!< USBD_T::ATTR: BYTEM Position           */
#define USBD_ATTR_BYTEM_Msk              (0x1ul << USBD_ATTR_BYTEM_Pos)                    /*!< USBD_T::ATTR: BYTEM Mask               */

#define USBD_VBUSDET_VBUSDET_Pos         (0)                                               /*!< USBD_T::VBUSDET: VBUSDET Position      */
#define USBD_VBUSDET_VBUSDET_Msk         (0x1ul << USBD_VBUSDET_VBUSDET_Pos)               /*!< USBD_T::VBUSDET: VBUSDET Mask          */

#define USBD_STBUFSEG_STBUFSEG_Pos       (3)                                               /*!< USBD_T::STBUFSEG: STBUFSEG Position    */
#define USBD_STBUFSEG_STBUFSEG_Msk       (0x3ful << USBD_STBUFSEG_STBUFSEG_Pos)            /*!< USBD_T::STBUFSEG: STBUFSEG Mask        */

#define USBD_EPSTS0_EPSTS0_Pos           (0)                                               /*!< USBD_T::EPSTS0: EPSTS0 Position        */
#define USBD_EPSTS0_EPSTS0_Msk           (0xful << USBD_EPSTS0_EPSTS0_Pos)                 /*!< USBD_T::EPSTS0: EPSTS0 Mask            */

#define USBD_EPSTS0_EPSTS1_Pos           (4)                                               /*!< USBD_T::EPSTS0: EPSTS1 Position        */
#define USBD_EPSTS0_EPSTS1_Msk           (0xful << USBD_EPSTS0_EPSTS1_Pos)                 /*!< USBD_T::EPSTS0: EPSTS1 Mask            */

#define USBD_EPSTS0_EPSTS2_Pos           (8)                                               /*!< USBD_T::EPSTS0: EPSTS2 Position        */
#define USBD_EPSTS0_EPSTS2_Msk           (0xful << USBD_EPSTS0_EPSTS2_Pos)                 /*!< USBD_T::EPSTS0: EPSTS2 Mask            */

#define USBD_EPSTS0_EPSTS3_Pos           (12)                                              /*!< USBD_T::EPSTS0: EPSTS3 Position        */
#define USBD_EPSTS0_EPSTS3_Msk           (0xful << USBD_EPSTS0_EPSTS3_Pos)                 /*!< USBD_T::EPSTS0: EPSTS3 Mask            */

#define USBD_EPSTS0_EPSTS4_Pos           (16)                                              /*!< USBD_T::EPSTS0: EPSTS4 Position        */
#define USBD_EPSTS0_EPSTS4_Msk           (0xful << USBD_EPSTS0_EPSTS4_Pos)                 /*!< USBD_T::EPSTS0: EPSTS4 Mask            */

#define USBD_EPSTS0_EPSTS5_Pos           (20)                                              /*!< USBD_T::EPSTS0: EPSTS5 Position        */
#define USBD_EPSTS0_EPSTS5_Msk           (0xful << USBD_EPSTS0_EPSTS5_Pos)                 /*!< USBD_T::EPSTS0: EPSTS5 Mask            */

#define USBD_EPSTS0_EPSTS6_Pos           (24)                                              /*!< USBD_T::EPSTS0: EPSTS6 Position        */
#define USBD_EPSTS0_EPSTS6_Msk           (0xful << USBD_EPSTS0_EPSTS6_Pos)                 /*!< USBD_T::EPSTS0: EPSTS6 Mask            */

#define USBD_EPSTS0_EPSTS7_Pos           (28)                                              /*!< USBD_T::EPSTS0: EPSTS7 Position        */
#define USBD_EPSTS0_EPSTS7_Msk           (0xful << USBD_EPSTS0_EPSTS7_Pos)                 /*!< USBD_T::EPSTS0: EPSTS7 Mask            */

#define USBD_EPSTS1_EPSTS8_Pos           (0)                                               /*!< USBD_T::EPSTS1: EPSTS8 Position        */
#define USBD_EPSTS1_EPSTS8_Msk           (0xful << USBD_EPSTS1_EPSTS8_Pos)                 /*!< USBD_T::EPSTS1: EPSTS8 Mask            */

#define USBD_EPSTS1_EPSTS9_Pos           (4)                                               /*!< USBD_T::EPSTS1: EPSTS9 Position        */
#define USBD_EPSTS1_EPSTS9_Msk           (0xful << USBD_EPSTS1_EPSTS9_Pos)                 /*!< USBD_T::EPSTS1: EPSTS9 Mask            */

#define USBD_EPSTS1_EPSTS10_Pos          (8)                                               /*!< USBD_T::EPSTS1: EPSTS10 Position       */
#define USBD_EPSTS1_EPSTS10_Msk          (0xful << USBD_EPSTS1_EPSTS10_Pos)                /*!< USBD_T::EPSTS1: EPSTS10 Mask           */

#define USBD_EPSTS1_EPSTS11_Pos          (12)                                              /*!< USBD_T::EPSTS1: EPSTS11 Position       */
#define USBD_EPSTS1_EPSTS11_Msk          (0xful << USBD_EPSTS1_EPSTS11_Pos)                /*!< USBD_T::EPSTS1: EPSTS11 Mask           */

#define USBD_FN_FN_Pos                   (0)                                               /*!< USBD_T::FN: FN Position                */
#define USBD_FN_FN_Msk                   (0x7fful << USBD_FN_FN_Pos)                       /*!< USBD_T::FN: FN Mask                    */

#define USBD_SE0_SE0_Pos                 (0)                                               /*!< USBD_T::SE0: SE0 Position              */
#define USBD_SE0_SE0_Msk                 (0x1ul << USBD_SE0_SE0_Pos)                       /*!< USBD_T::SE0: SE0 Mask                  */

#define USBD_BUFSEG_BUFSEG_Pos           (3)                                               /*!< USBD_T::BUFSEG: BUFSEG Position        */
#define USBD_BUFSEG_BUFSEG_Msk           (0x3ful << USBD_BUFSEG_BUFSEG_Pos)                /*!< USBD_T::BUFSEG: BUFSEG Mask            */

#define USBD_MXPLD_MXPLD_Pos             (0)                                               /*!< USBD_T::MXPLD: MXPLD Position          */
#define USBD_MXPLD_MXPLD_Msk             (0x1fful << USBD_MXPLD_MXPLD_Pos)                 /*!< USBD_T::MXPLD: MXPLD Mask              */

#define USBD_CFG_EPNUM_Pos               (0)                                               /*!< USBD_T::CFG: EPNUM Position            */
#define USBD_CFG_EPNUM_Msk               (0xful << USBD_CFG_EPNUM_Pos)                     /*!< USBD_T::CFG: EPNUM Mask                */

#define USBD_CFG_ISOCH_Pos               (4)                                               /*!< USBD_T::CFG: ISOCH Position            */
#define USBD_CFG_ISOCH_Msk               (0x1ul << USBD_CFG_ISOCH_Pos)                     /*!< USBD_T::CFG: ISOCH Mask                */

#define USBD_CFG_STATE_Pos               (5)                                               /*!< USBD_T::CFG: STATE Position            */
#define USBD_CFG_STATE_Msk               (0x3ul << USBD_CFG_STATE_Pos)                     /*!< USBD_T::CFG: STATE Mask                */

#define USBD_CFG_DSQSYNC_Pos             (7)                                               /*!< USBD_T::CFG: DSQSYNC Position          */
#define USBD_CFG_DSQSYNC_Msk             (0x1ul << USBD_CFG_DSQSYNC_Pos)                   /*!< USBD_T::CFG: DSQSYNC Mask              */

#define USBD_CFG_CSTALL_Pos              (9)                                               /*!< USBD_T::CFG: CSTALL Position           */
#define USBD_CFG_CSTALL_Msk              (0x1ul << USBD_CFG_CSTALL_Pos)                    /*!< USBD_T::CFG: CSTALL Mask               */

#define USBD_CFGP_CLRRDY_Pos             (0)                                               /*!< USBD_T::CFGP: CLRRDY Position          */
#define USBD_CFGP_CLRRDY_Msk             (0x1ul << USBD_CFGP_CLRRDY_Pos)                   /*!< USBD_T::CFGP: CLRRDY Mask              */

#define USBD_CFGP_SSTALL_Pos             (1)                                               /*!< USBD_T::CFGP: SSTALL Position          */
#define USBD_CFGP_SSTALL_Msk             (0x1ul << USBD_CFGP_SSTALL_Pos)                   /*!< USBD_T::CFGP: SSTALL Mask              */

/**@}*/ /* USBD_CONST */
/**@}*/ /* end of USBD register group */


/*---------------------- Watch Dog Timer Controller -------------------------*/
/**
    @addtogroup WDT Watch Dog Timer Controller(WDT)
    Memory Mapped Structure for WDT Controller
@{ */
 
typedef struct
{


/**
 * @var WDT_T::CTL
 * Offset: 0x00  Watchdog Timer Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RSTCNT    |Clear Watchdog Timer (Write Protected)
 * |        |          |Set this bit will clear the Watchdog timer.
 * |        |          |0 = Writing 0 to this bit has no effect.
 * |        |          |1 = Reset the contents of the Watchdog timer.
 * |        |          |Note1: This bit will be automatically cleared by hardware.
 * |        |          |Note2: This bit is writing protected. Refer to the SYS_REGLCTL.
 * |[1]     |RSTEN     |Watchdog Timer Reset Enable(Write Protected)
 * |        |          |Setting this bit will enable the Watchdog timer reset function.
 * |        |          |0 = Disable Watchdog timer reset function.
 * |        |          |1= Enable Watchdog timer reset function.
 * |        |          |Note: This bit is writing protected. Refer to the SYS_REGLCTL.
 * |[2]     |RSTF      |Watchdog Timer Reset Flag
 * |        |          |When the Watchdog timer initiates a reset, the hardware will set this bit
 * |        |          |This flag can be read by software to determine the source of reset
 * |        |          |Software is responsible to clear it manually by writing 1 to it
 * |        |          |If RSTEN is disabled, then the Watchdog timer has no effect on this bit.
 * |        |          |0 = Watchdog timer reset has not occurred.
 * |        |          |1= Watchdog timer reset has occurred.
 * |        |          |Note: This bit is cleared by writing 1 to this bit.
 * |[3]     |IF        |Watchdog Timer Interrupt Flag
 * |        |          |If the Watchdog timer interrupt is enabled, then the hardware will set this bit to indicate that the Watchdog timer interrupt has occurred
 * |        |          |If the Watchdog timer interrupt is not enabled, then this bit indicates that a timeout period has elapsed.
 * |        |          |0 = Watchdog timer interrupt has not occurred.
 * |        |          |1 = Watchdog timer interrupt has occurred.
 * |        |          |Note: This bit is cleared by writing 1 to this bit.
 * |[4]     |WKEN      |WDT Time-Out Wake-Up Function Control
 * |        |          |If this bit is set to 1, while WDT time-out interrupt flag IF (WDT_CTL[3]) is generated to 1 and interrupt enable bit INTEN (WDT_CTL[6]) is enabled, the WDT time-out interrupt signal will generate a wake-up trigger event to chip.
 * |        |          |0 = Enable the Wakeup function that WDT timeout can wake up CPU from power-down mode.
 * |        |          |1 = Disable WDT Wakeup CPU function.
 * |[5]     |WKF       |WDT Time-Out Wake-Up Flag
 * |        |          |If WDT causes CPU wake up from sleep or power-down mode, this bit will be set to high.
 * |        |          |0 = WDT does not cause CPU wake-up.
 * |        |          |1 = CPU wakes up from sleep or power-down mode by WDT time-out interrupt.
 * |        |          |Note: This bit is cleared by writing 1 to it. 
 * |[6]     |INTEN     |Watchdog Time-Out Interrupt Enable
 * |        |          |0 = Disable the WDT time-out interrupt.
 * |        |          |1 = Enable the WDT time-out interrupt.
 * |[7]     |WDTEN     |Watchdog Timer Enable
 * |        |          |0 = Disable the WDT(Watchdog timer) (This action will reset the internal counter).
 * |        |          |1 = Enable the WDT(Watchdog timer).
 * |[10:8]  |TOUTSEL   |Watchdog Timer Interval Select
 * |        |          |These three bits select the timeout interval for the Watchdog timer, a watchdog reset will occur 1024 clock cycles later if Watchdog timer is not reset.
 * |        |          |The WDT interrupt timeout is given by:
 * |        |          |000 = 24 * WDT_CLK.
 * |        |          |001 = 26 * WDT_CLK.
 * |        |          |010 = 28 * WDT_CLK.
 * |        |          |011 = 210 * WDT_CLK.
 * |        |          |100 = 212 * WDT_CLK.
 * |        |          |101 = 214 * WDT_CLK.
 * |        |          |110 = 216 * WDT_CLK.
 * |        |          |111 = 218 * WDT_CLK.
 * |        |          |WDT reset timeout = (WDT interrupt timeout +1024) * WDT_CLK.
 * |        |          |Where WDT_CLK is the period of the Watchdog Timer clock source.
 */
    __IO uint32_t CTL;                   /*!< [0x0000] Watchdog Timer Control Register                                  */

} WDT_T;

/**
    @addtogroup WDT_CONST WDT Bit Field Definition
    Constant Definitions for WDT Controller
@{ */

#define WDT_CTL_RSTCNT_Pos               (0)                                               /*!< WDT_T::CTL: RSTCNT Position            */
#define WDT_CTL_RSTCNT_Msk               (0x1ul << WDT_CTL_RSTCNT_Pos)                     /*!< WDT_T::CTL: RSTCNT Mask                */

#define WDT_CTL_RSTEN_Pos                (1)                                               /*!< WDT_T::CTL: RSTEN Position             */
#define WDT_CTL_RSTEN_Msk                (0x1ul << WDT_CTL_RSTEN_Pos)                      /*!< WDT_T::CTL: RSTEN Mask                 */

#define WDT_CTL_RSTF_Pos                 (2)                                               /*!< WDT_T::CTL: RSTF Position              */
#define WDT_CTL_RSTF_Msk                 (0x1ul << WDT_CTL_RSTF_Pos)                       /*!< WDT_T::CTL: RSTF Mask                  */

#define WDT_CTL_IF_Pos                   (3)                                               /*!< WDT_T::CTL: IF Position                */
#define WDT_CTL_IF_Msk                   (0x1ul << WDT_CTL_IF_Pos)                         /*!< WDT_T::CTL: IF Mask                    */

#define WDT_CTL_WKEN_Pos                 (4)                                               /*!< WDT_T::CTL: WKEN Position              */
#define WDT_CTL_WKEN_Msk                 (0x1ul << WDT_CTL_WKEN_Pos)                       /*!< WDT_T::CTL: WKEN Mask                  */

#define WDT_CTL_WKF_Pos                  (5)                                               /*!< WDT_T::CTL: WKF Position               */
#define WDT_CTL_WKF_Msk                  (0x1ul << WDT_CTL_WKF_Pos)                        /*!< WDT_T::CTL: WKF Mask                   */

#define WDT_CTL_INTEN_Pos                (6)                                               /*!< WDT_T::CTL: INTEN Position             */
#define WDT_CTL_INTEN_Msk                (0x1ul << WDT_CTL_INTEN_Pos)                      /*!< WDT_T::CTL: INTEN Mask                 */

#define WDT_CTL_WDTEN_Pos                (7)                                               /*!< WDT_T::CTL: WDTEN Position             */
#define WDT_CTL_WDTEN_Msk                (0x1ul << WDT_CTL_WDTEN_Pos)                      /*!< WDT_T::CTL: WDTEN Mask                 */

#define WDT_CTL_TOUTSEL_Pos              (8)                                               /*!< WDT_T::CTL: TOUTSEL Position           */
#define WDT_CTL_TOUTSEL_Msk              (0x7ul << WDT_CTL_TOUTSEL_Pos)                    /*!< WDT_T::CTL: TOUTSEL Mask               */

/**@}*/ /* WDT_CONST */
/**@}*/ /* end of WDT register group */


/**@}*/ /* end of REGISTER group */


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup I91500_PERIPHERAL_MEM_MAP I91500 Peripheral Memory Map
  Memory Mapped Structure for I91500 Series Peripheral
  @{
 */
/* Peripheral and SRAM base address */
#define FLASH_BASE          ((     uint32_t)0x00000000)
#define SRAM_BASE           ((     uint32_t)0x20000000)
#define AHB_BASE            ((     uint32_t)0x50000000)
#define APB1_BASE           ((     uint32_t)0x40000000)

/* Peripheral memory map */
/******************************************************************************/
/*						   Peripheral memory map							  */
/******************************************************************************/
#define WDT_BASE       	     (APB1_BASE      + 0x04000)
#define TIMER0_BASE          (APB1_BASE      + 0x10000)
#define TIMER1_BASE          (APB1_BASE      + 0x10020)
#define	TIMERIR_BASE         (APB1_BASE      + 0x10034)
#define	TIMER2_BASE          (APB1_BASE      + 0x10040)
#define	I2C0_BASE            (APB1_BASE      + 0x20000)
#define	I2C1_BASE            (APB1_BASE      + 0x21000)
#define SPI0_BASE            (APB1_BASE      + 0x30000)
#define SPI1_BASE            (APB1_BASE      + 0x31000)
#define PWM0_BASE            (APB1_BASE      + 0x40000)
#define PWM1_BASE            (APB1_BASE      + 0x50000)
#define UART0_BASE           (APB1_BASE      + 0x60000)
#define UART1_BASE           (APB1_BASE      + 0x61000)
#define DAC_BASE             (APB1_BASE      + 0x70000)
#define ANA_BASE             (APB1_BASE      + 0x80000)
#define	I2S0_BASE            (APB1_BASE      + 0x90000)
#define	BIQ_BASE             (APB1_BASE      + 0xB0000)
#define	USBD_BASE            (APB1_BASE      + 0xC0000)
#define SDADC_BASE           (APB1_BASE      + 0xD0000)
#define	SARADC_BASE          (APB1_BASE      + 0xE0000)

#define SYS_BASE             (AHB_BASE       + 0x00000)
#define CLK_BASE             (AHB_BASE       + 0x00200)
#define INT_BASE             (AHB_BASE       + 0x00300)
#define GPIO_BASE            (AHB_BASE       + 0x04000)
#define GPIOA_BASE           (GPIO_BASE               )
#define GPIOB_BASE           (GPIO_BASE      + 0x00040)
#define GPIOC_BASE           (GPIO_BASE      + 0x00080)
#define GPIOD_BASE           (GPIO_BASE      + 0x000C0)
#define GPIO_PIN_DATA_BASE   (GPIO_BASE      + 0x00800)                 /*!< GPIO Pin Data Input/Output Control Base Address  */
#define PDMA0_BASE           (AHB_BASE       + 0x09000)                 /*!< PDMA Base Address                                */
#define PDMA_GCR_BASE        (AHB_BASE       + 0x09F00)                 /*!< PDMA Grobal Base Address                         */
#define FMC_BASE             (AHB_BASE       + 0x0C000)                 /*!< Flash Memory Control Registers                   */
#define CPD_BASE             (AHB_BASE       + 0x0E000)                 /*!< Companding Control Registers                     */

/*@}*/ /* end of group I91500_PERIPHERAL_MEM_MAP */
/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
/** @addtogroup I91500_PeripheralDecl I91500 Peripheral Declaration
    @{
*/
#define WDT                 ((WDT_T *) WDT_BASE)
#define TIMER0              ((TMR_T *) TIMER0_BASE)
#define TIMER1              ((TMR_T *) TIMER1_BASE)
#define TIMER_IR            ((TMRIR_T *) TIMERIR_BASE)
#define TIMER2              ((TMR_T *) TIMER2_BASE)
#define I2C0                ((I2C_T *) I2C0_BASE)
#define I2C1                ((I2C_T *) I2C1_BASE)
#define SPI0                ((SPI_T *) SPI0_BASE)
#define SPI1                ((SPI_T *) SPI1_BASE)
#define PWM0                ((PWM_T *) PWM0_BASE)
#define PWM1                ((PWM_T *) PWM1_BASE)
#define UART0               ((UART_T *) UART0_BASE)
#define UART1               ((UART_T *) UART1_BASE)
#define DAC                 ((DAC_T *) DAC_BASE)
#define ANA                 ((ANA_T *) ANA_BASE)
#define I2S0                ((I2S_T *) I2S0_BASE)
#define USBD                ((USBD_T *) USBD_BASE)
#define SDADC               ((SDADC_T *) SDADC_BASE)
#define SARADC              ((SARADC_T *) SARADC_BASE)
#define	BIQ                 ((BIQ_T *) BIQ_BASE)

#define SYS                 ((SYS_T *) SYS_BASE)
#define CLK                 ((CLK_T *) CLK_BASE)
#define SYSINT              ((SYSINT_T *) INT_BASE)
#define PA                  ((GPIO_T *) GPIOA_BASE)
#define PB                  ((GPIO_T *) GPIOB_BASE)
#define PC                  ((GPIO_T *) GPIOC_BASE)
#define PD                  ((GPIO_T *) GPIOD_BASE)
#define PDMA0               ((PDMA_T *) PDMA0_BASE)                     /*!< PDMA0 Configuration Struct                       */
#define PDMA_GCR            ((PDMA_GCR_T *) PDMA_GCR_BASE)              /*!< PDMA Global Configuration Struct                 */
#define FMC                 ((FMC_T *) FMC_BASE)
#define CPD                 ((CPD_T *) CPD_BASE)

#define PDMA1               ((PDMA_T *) (PDMA0_BASE + 0x100))           /*!< PDMA1 Configuration Struct                       */
#define PDMA2               ((PDMA_T *) (PDMA0_BASE + 0x200))           /*!< PDMA2 Configuration Struct                       */
#define PDMA3               ((PDMA_T *) (PDMA0_BASE + 0x300))           /*!< PDMA3 Configuration Struct                       */
#define PDMA4               ((PDMA_T *) (PDMA0_BASE + 0x400))           /*!< PDMA4 Configuration Struct                       */
#define PDMA5               ((PDMA_T *) (PDMA0_BASE + 0x500))           /*!< PDMA5 Configuration Struct                       */
#define PDMA6               ((PDMA_T *) (PDMA0_BASE + 0x600))           /*!< PDMA6 Configuration Struct                       */
#define PDMA7               ((PDMA_T *) (PDMA0_BASE + 0x700))           /*!< PDMA7 Configuration Struct                       */


/*@}*/ /* end of group I91500_PeripheralDecl */

#define UNLOCKREG(x)        do{*((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x59;*((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x16;*((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x88;}while(*((__IO uint32_t *)(SYS_BASE + 0x100))==0)
#define LOCKREG(x)          *((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x00

#define REGCOPY(dest, src)  *((uint32_t *)&(dest)) = *((uint32_t *)&(src))
#define CLEAR(dest)         *((uint32_t *)&(dest)) = 0


//=============================================================================
/** @addtogroup I91500_IO_ROUTINE I91500 I/O routines
  The Declaration of I91500 I/O routines
  @{
 */

typedef volatile unsigned char  vu8;        ///< Define 8-bit unsigned volatile data type
typedef volatile unsigned short vu16;       ///< Define 16-bit unsigned volatile data type
typedef volatile unsigned long  vu32;       ///< Define 32-bit unsigned volatile data type

/**
  * @brief Get a 8-bit unsigned value from specified address
  * @param[in] addr Address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified address
  */
#define M8(addr)  (*((vu8  *) (addr)))

/**
  * @brief Get a 16-bit unsigned value from specified address
  * @param[in] addr Address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified address
  * @note The input address must be 16-bit aligned
  */
#define M16(addr) (*((vu16 *) (addr)))

/**
  * @brief Get a 32-bit unsigned value from specified address
  * @param[in] addr Address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified address
  * @note The input address must be 32-bit aligned
  */
#define M32(addr) (*((vu32 *) (addr)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outpw(port,value)     *((volatile unsigned int *)(port)) = value

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inpw(port)            (*((volatile unsigned int *)(port)))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outps(port,value)     *((volatile unsigned short *)(port)) = value

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inps(port)            (*((volatile unsigned short *)(port)))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outpb(port,value)     *((volatile unsigned char *)(port)) = value

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inpb(port)            (*((volatile unsigned char *)(port)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outp32(port,value)    *((volatile unsigned int *)(port)) = value

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inp32(port)           (*((volatile unsigned int *)(port)))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outp16(port,value)    *((volatile unsigned short *)(port)) = value

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inp16(port)           (*((volatile unsigned short *)(port)))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outp8(port,value)     *((volatile unsigned char *)(port)) = value

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inp8(port)            (*((volatile unsigned char *)(port)))

/*@}*/ /* end of group I91500_IO_ROUTINE */


/** @addtogroup I91500_legacy_Constants I91500 Legacy Constants
  I91500 Legacy Constants
  @{
*/

#define E_SUCCESS   0
#ifndef NULL
#define NULL        0
#endif

#define TRUE        1
#define FALSE       0

#define ENABLE      1
#define DISABLE     0

/* Define one bit mask */
#define BIT0    0x00000001
#define BIT1    0x00000002
#define BIT2    0x00000004
#define BIT3    0x00000008
#define BIT4    0x00000010
#define BIT5    0x00000020
#define BIT6    0x00000040
#define BIT7    0x00000080
#define BIT8    0x00000100
#define BIT9    0x00000200
#define BIT10   0x00000400
#define BIT11   0x00000800
#define BIT12   0x00001000
#define BIT13   0x00002000
#define BIT14   0x00004000
#define BIT15   0x00008000
#define BIT16   0x00010000
#define BIT17   0x00020000
#define BIT18   0x00040000
#define BIT19   0x00080000
#define BIT20   0x00100000
#define BIT21   0x00200000
#define BIT22   0x00400000
#define BIT23   0x00800000
#define BIT24   0x01000000
#define BIT25   0x02000000
#define BIT26   0x04000000
#define BIT27   0x08000000
#define BIT28   0x10000000
#define BIT29   0x20000000
#define BIT30   0x40000000
#define BIT31   0x80000000

/* Byte Mask Definitions */
#define BYTE0_Msk               (0x000000FF)
#define BYTE1_Msk               (0x0000FF00)
#define BYTE2_Msk               (0x00FF0000)
#define BYTE3_Msk               (0xFF000000)

#define _GET_BYTE0(u32Param)    ((u32Param & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define _GET_BYTE1(u32Param)    ((u32Param & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define _GET_BYTE2(u32Param)    ((u32Param & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define _GET_BYTE3(u32Param)    ((u32Param & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */

/*@}*/ /* end of group I91500_legacy_Constants */

/*@}*/ /* end of group I91500_Definitions */

#define __I91500_SERIES__ (0x91500000)
#define __CHIP_SERIES__ __I91500_SERIES__

/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "sys.h"
#include "clk.h"
#include "spi.h"
#include "dac.h"
#include "gpio.h"
#include "bod.h"
#include "timer.h"
#include "pwm.h"
#include "fmc.h"
#include "wdt.h"
#include "pdma.h"
#include "uart.h"
#include "i2c.h"
#include "saradc.h"
#include "sdadc.h"
#include "usbd.h"
#include "i2s.h"
#include "biq.h"
#include "ana.h"

#endif	// __I91500_H__
