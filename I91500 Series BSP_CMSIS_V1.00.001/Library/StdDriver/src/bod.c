/*************************************************************************//**
 * @file     BOD.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 20/11/18 7:06p $
 * @brief    I91500 BOD driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include  "Platform.h"

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_BOD_Driver BOD Driver
  @{
*/


/** @addtogroup I91500_BOD_EXPORTED_FUNCTIONS BOD Exported Functions
  @{
*/

/**
  * @brief      This function will enable BOD
  * @param[in]  u8Mode is BOD_RESET_MODE or BOD_INTERRUPT_MODE.
  * @param[in]  u8BODLevel is BOD Voltage Level.
  * @return     None
  */
void BOD_Open(uint8_t u8Mode,uint8_t u8BODLevel)
{
	uint8_t u8Lock = SYS_Unlock();

	//BOD operation require that the OSC16K low power oscillator is enabled
	CLK->PWRCTL &= (~CLK_PWRCTL_WK10KEN_Msk);

	SYS->BODCTL |= (SYS_BODCTL_BODEN_Msk);

	SYS->BODCTL &= (~SYS_BODCTL_BODLVL_Msk);
	SYS->BODCTL |= (u8BODLevel<< SYS_BODCTL_BODLVL_Pos);

	SYS->BODCTL &= (~SYS_BODCTL_BODRSTEN_Msk);
	SYS->BODCTL |= (u8Mode<<SYS_BODCTL_BODRSTEN_Pos);

	SYS_Lock(u8Lock);
}

/**
  * @brief  This function will disable BOD
  * @return None
  */
void BOD_Close(void)
{
	uint8_t u8Lock = SYS_Unlock();

	SYS->BODCTL &= (~SYS_BODCTL_BODEN_Msk);
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function will enable LVR
  * @return None
  */
void BOD_LVR_Enable(void)
{
	uint8_t u8Lock = SYS_Unlock();

	SYS->BODCTL |= SYS_BODCTL_LVREN_Msk;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function will disable LVR
  * @return None
  */
void BOD_LVR_Disable(void)
{
	uint8_t u8Lock = SYS_Unlock();

	SYS->BODCTL &= (~SYS_BODCTL_LVREN_Msk);
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function will enable BOD Hysteresis
  * @return None
  */
void BOD_HYS_Enable(void)
{
	uint8_t u8Lock = SYS_Unlock();

	SYS->BODCTL |= SYS_BODCTL_BODHYS_Msk;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function will disable BOD Hysteresis
  * @return None
  */
void BOD_HYS_Disable(void)
{
	uint8_t u8Lock = SYS_Unlock();

	SYS->BODCTL &= (~SYS_BODCTL_BODHYS_Msk);
	SYS_Lock(u8Lock);
}

/**
  * @brief     This function clears the BOD interrupt flag.
  * @return    None
  */
void BOD_ClearIntFlag(void)
{
	uint8_t u8Lock = SYS_Unlock();

	SYS->BODCTL |= SYS_BODCTL_BODINT_Msk;
	SYS_Lock(u8Lock);
}


/*@}*/ /* end of group I91500_BOD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_BOD_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/

