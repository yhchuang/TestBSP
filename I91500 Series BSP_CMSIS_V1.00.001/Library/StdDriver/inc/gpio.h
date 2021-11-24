/**************************************************************************//**
 * @file     gpio.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/07/15 11:50a $
 * @brief    I91500 GPIO driver header file
 *
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_GPIO_Driver GPIO Driver
  @{
*/

/** @addtogroup I91500_GPIO_EXPORTED_CONSTANTS GPIO Exported Constants
  @{
*/
	
#define GPIO_PIN_MAX    			16   /*!< Specify Maximum Pins of Each GPIO Port */

/*---------------------------------------------------------------------------------------------------------*/
/*  MODE Constant Definitions                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_MODE_INPUT              0x0UL           /*!< Input Mode */
#define GPIO_MODE_OUTPUT             0x1UL           /*!< Output Mode */
#define GPIO_MODE_OPEN_DRAIN         0x2UL           /*!< Open-Drain Mode */
#define GPIO_MODE_PULLUP             0x3UL           /*!< Input and PULL-up mode */

/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO Interrupt Type Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_INT_RISING             0x00010000UL    /*!< Interrupt enable by Input Rising Edge */
#define GPIO_INT_FALLING            0x00000001UL    /*!< Interrupt enable by Input Falling Edge */
#define GPIO_INT_BOTH_EDGE          0x00010001UL    /*!< Interrupt enable by both Rising Edge and Falling Edge */
#define GPIO_INT_HIGH               0x01010000UL    /*!< Interrupt enable by Level-High */
#define GPIO_INT_LOW                0x01000001UL    /*!< Interrupt enable by Level-Level */

/*---------------------------------------------------------------------------------------------------------*/
/*  INTTYPE Constant Definitions                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define GPIO_INTTYPE_EDGE               0UL             /*!< INTTYPE Setting for Edge Trigger Mode */
#define GPIO_INTTYPE_LEVEL              1UL             /*!< INTTYPE Setting for Edge Level Mode */

/* Define GPIO Pin Data Input/Output. It could be used to control each I/O pin by pin address mapping.
   Example 1:

       PA0 = 1;

   It is used to set GPIO PA.0 to high;

   Example 2:

       if (PA0)
           PA0 = 0;

   If GPIO PA.0 pin status is high, then set GPIO PA.0 data output to low.
 */
#define GPIO_PIN_DATA(port, pin)    (*((volatile uint32_t *)((GPIO_PIN_DATA_BASE+(0x40*(port))) + ((pin)<<2))))
#define PA0             GPIO_PIN_DATA(0, 0 ) /*!< Specify PA.0 Pin Data Input/Output */
#define PA1             GPIO_PIN_DATA(0, 1 ) /*!< Specify PA.1 Pin Data Input/Output */
#define PA2             GPIO_PIN_DATA(0, 2 ) /*!< Specify PA.2 Pin Data Input/Output */
#define PA3             GPIO_PIN_DATA(0, 3 ) /*!< Specify PA.3 Pin Data Input/Output */
#define PA4             GPIO_PIN_DATA(0, 4 ) /*!< Specify PA.4 Pin Data Input/Output */
#define PA5             GPIO_PIN_DATA(0, 5 ) /*!< Specify PA.5 Pin Data Input/Output */
#define PA6             GPIO_PIN_DATA(0, 6 ) /*!< Specify PA.6 Pin Data Input/Output */
#define PA7             GPIO_PIN_DATA(0, 7 ) /*!< Specify PA.7 Pin Data Input/Output */
#define PA8             GPIO_PIN_DATA(0, 8 ) /*!< Specify PA.8 Pin Data Input/Output */
#define PA9             GPIO_PIN_DATA(0, 9 ) /*!< Specify PA.9 Pin Data Input/Output */
#define PA10            GPIO_PIN_DATA(0, 10) /*!< Specify PA.10 Pin Data Input/Output */
#define PA11            GPIO_PIN_DATA(0, 11) /*!< Specify PA.11 Pin Data Input/Output */
#define PA12            GPIO_PIN_DATA(0, 12) /*!< Specify PA.12 Pin Data Input/Output */
#define PA13            GPIO_PIN_DATA(0, 13) /*!< Specify PA.13 Pin Data Input/Output */
#define PA14            GPIO_PIN_DATA(0, 14) /*!< Specify PA.14 Pin Data Input/Output */
#define PA15            GPIO_PIN_DATA(0, 15) /*!< Specify PA.15 Pin Data Input/Output */
#define PB0             GPIO_PIN_DATA(1, 0 ) /*!< Specify PB.0 Pin Data Input/Output */
#define PB1             GPIO_PIN_DATA(1, 1 ) /*!< Specify PB.1 Pin Data Input/Output */
#define PB2             GPIO_PIN_DATA(1, 2 ) /*!< Specify PB.2 Pin Data Input/Output */
#define PB3             GPIO_PIN_DATA(1, 3 ) /*!< Specify PB.3 Pin Data Input/Output */
#define PB4             GPIO_PIN_DATA(1, 4 ) /*!< Specify PB.4 Pin Data Input/Output */
#define PC0             GPIO_PIN_DATA(2, 0 ) /*!< Specify PC.0 Pin Data Input/Output */
#define PC1             GPIO_PIN_DATA(2, 1 ) /*!< Specify PC.1 Pin Data Input/Output */
#define PC2             GPIO_PIN_DATA(2, 2 ) /*!< Specify PC.2 Pin Data Input/Output */
#define PC3             GPIO_PIN_DATA(2, 3 ) /*!< Specify PC.3 Pin Data Input/Output */
#define PC4             GPIO_PIN_DATA(2, 4 ) /*!< Specify PC.4 Pin Data Input/Output */
#define PC5             GPIO_PIN_DATA(2, 5 ) /*!< Specify PC.5 Pin Data Input/Output */
#define PC6             GPIO_PIN_DATA(2, 6 ) /*!< Specify PC.6 Pin Data Input/Output */
#define PC7             GPIO_PIN_DATA(2, 7 ) /*!< Specify PC.7 Pin Data Input/Output */
#define PC8             GPIO_PIN_DATA(2, 8 ) /*!< Specify PC.8 Pin Data Input/Output */
#define PC9             GPIO_PIN_DATA(2, 9 ) /*!< Specify PC.9 Pin Data Input/Output */
#define PC10            GPIO_PIN_DATA(2, 10) /*!< Specify PC.10 Pin Data Input/Output */
#define PC11            GPIO_PIN_DATA(2, 11) /*!< Specify PC.11 Pin Data Input/Output */
#define PC12            GPIO_PIN_DATA(2, 12) /*!< Specify PC.12 Pin Data Input/Output */
#define PC13            GPIO_PIN_DATA(2, 13) /*!< Specify PC.13 Pin Data Input/Output */
#define PC14            GPIO_PIN_DATA(2, 14) /*!< Specify PC.14 Pin Data Input/Output */
#define PC15            GPIO_PIN_DATA(2, 15) /*!< Specify PC.15 Pin Data Input/Output */
#define PD0             GPIO_PIN_DATA(3, 0 ) /*!< Specify PD.0 Pin Data Input/Output */
#define PD1             GPIO_PIN_DATA(3, 1 ) /*!< Specify PD.1 Pin Data Input/Output */
#define PD2             GPIO_PIN_DATA(3, 2 ) /*!< Specify PD.2 Pin Data Input/Output */
#define PD3             GPIO_PIN_DATA(3, 3 ) /*!< Specify PD.3 Pin Data Input/Output */
#define PD4             GPIO_PIN_DATA(3, 4 ) /*!< Specify PD.4 Pin Data Input/Output */
#define PD5             GPIO_PIN_DATA(3, 5 ) /*!< Specify PD.5 Pin Data Input/Output */
#define PD6             GPIO_PIN_DATA(3, 6 ) /*!< Specify PD.6 Pin Data Input/Output */
#define PD7             GPIO_PIN_DATA(3, 7 ) /*!< Specify PD.7 Pin Data Input/Output */
#define PD8             GPIO_PIN_DATA(3, 8 ) /*!< Specify PD.8 Pin Data Input/Output */
#define PD9             GPIO_PIN_DATA(3, 9 ) /*!< Specify PD.9 Pin Data Input/Output */
#define PD10            GPIO_PIN_DATA(3, 10) /*!< Specify PD.10 Pin Data Input/Output */
#define PD11            GPIO_PIN_DATA(3, 11) /*!< Specify PD.11 Pin Data Input/Output */
#define PD12            GPIO_PIN_DATA(3, 12) /*!< Specify PD.12 Pin Data Input/Output */
#define PD13            GPIO_PIN_DATA(3, 13) /*!< Specify PD.13 Pin Data Input/Output */
#define PD14            GPIO_PIN_DATA(3, 14) /*!< Specify PD.14 Pin Data Input/Output */
#define PD15            GPIO_PIN_DATA(3, 15) /*!< Specify PD.15 Pin Data Input/Output */

/*@}*/ /* end of group I91500_GPIO_EXPORTED_CONSTANTS */

/** @addtogroup I91500_GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
 * @brief       Clear GPIO Pin Interrupt Flag
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @return      None
 *
 * @details     Clear the interrupt status of specified GPIO pin.
 */
#define GPIO_CLR_INT_FLAG(gpio, u32PinMask)   ((gpio)->INTSRC = u32PinMask)

/**
 * @brief       Disable I/O Digital Input Path
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @return      None
 *
 * @details     Disable I/O digital input path of specified GPIO pin.
 */
#define GPIO_DISABLE_DIGITAL_PATH(gpio, u32PinMask)   ((gpio)->DINOFF |= (u32PinMask << 16))

/**
 * @brief       Enable I/O Digital Input Path
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @return      None
 *
 * @details     Enable I/O digital input path of specified GPIO pin.
 */
#define GPIO_ENABLE_DIGITAL_PATH(gpio, u32PinMask)    ((gpio)->DINOFF &= ~(u32PinMask << 16))

/**
 * @brief       Disable I/O DOUT mask
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @return      None
 *
 * @details     Disable I/O DOUT mask of specified GPIO pin.
 */
#define GPIO_DISABLE_DOUT_MASK(gpio, u32PinMask)   ((gpio)->DATMSK |= u32PinMask)

/**
 * @brief       Enable I/O DOUT mask
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @return      None
 *
 * @details     Enable I/O DOUT mask of specified GPIO pin.
 */
#define GPIO_ENABLE_DOUT_MASK(gpio, u32PinMask)   ((gpio)->DATMSK &= ~u32PinMask)

/**
 * @brief       Get GPIO Pin Interrupt Flag
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *
 * @retval      0           No interrupt at specified GPIO pin
 * @retval      1           The specified GPIO pin generate an interrupt
 *
 * @details     Get the interrupt status of specified GPIO pin.
 */
#define GPIO_GET_INT_FLAG(gpio, u32PinMask)   ((gpio)->INTSRC & u32PinMask)

/**
 * @brief       Get GPIO Port IN Data
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB, PC, PD.
 *
 * @retval      The specified port data
 *
 * @details     Get the PIN register of specified GPIO port.
 */
#define GPIO_GET_IN_DATA(gpio)   ((gpio)->PIN)

/**
 * @brief       Set GPIO Port OUT Data
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB, PC, PD.
 * @param[in]   u32Data     GPIO port data.
 *
 * @retval      None
 *
 * @details     Set the Data into specified GPIO port.
 */
#define GPIO_SET_OUT_DATA(gpio, u32Data)   ((gpio)->DOUT = (u32Data))

/**
 * @brief       Get GPIO Port OUT Data
 *
 * @param[in]   gpio        GPIO port. It could be PA, PB, PC, PD.
 *
 * @retval      None
 *
 * @details     Get the Data into specified GPIO port.
 */
#define GPIO_GET_OUT_DATA(gpio)   ((gpio)->DOUT)

/**
 * @brief       Toggle Specified GPIO pin
 *
 * @param[in]   u32Pin       Pxy
 *
 * @retval      None
 *
 * @details     Toggle the specified GPIO pint.
 */
#define GPIO_TOGGLE(u32Pin)   ((u32Pin) ^= 1)

void GPIO_SetMode(GPIO_T *gpio, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *gpio, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *gpio, uint32_t u32Pin);

/*@}*/ /* end of group I91500_GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_GPIO_Driver */

/*@}*/ /* end of group I91500_Device_Driver */


#ifdef __cplusplus
}
#endif

#endif //__GPIO_H__

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
