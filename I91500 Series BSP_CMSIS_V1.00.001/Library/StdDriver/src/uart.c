/**************************************************************************//**
 * @file     uart.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/09/24 2:45p $
 * @brief    I91500 UART driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Platform.h"

/** @addtogroup I91500_Device_Driver I91500 Device Driver
  @{
*/

/** @addtogroup I91500_UART_Driver UART Driver
  @{
*/

/** @addtogroup I91500_UART_EXPORTED_FUNCTIONS UART Exported Functions
  @{
*/

/**
 *    @brief  The function is used to clear UART specified interrupt flag.
 *
 *    @param  uart                The base address of UART module.
 *    @param  u32InterruptFlag    The specified interrupt of UART module..
 *
 *    @return None
 */
void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag)
{
    if(u32InterruptFlag & UART_INTSTS_RLSINT_Msk)      /* clear Receive Line Status Interrupt */
        uart->FIFOSTS |= UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk;

    if(u32InterruptFlag & UART_INTSTS_MODEMINT_Msk)    /* clear Modem Interrupt */
        uart->MODEMSTS |= UART_MODEMSTS_CTSDETF_Msk;

    if(u32InterruptFlag & UART_INTSTS_BUFERRINT_Msk)   /* clear Buffer Error Interrupt */
        uart->FIFOSTS |= ( UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_TXOVIF_Msk );

    if(u32InterruptFlag & UART_INTSTS_RXTOINT_Msk)     /* clear Modem Interrupt */
        uart->INTSTS |= UART_INTSTS_RXTOIF_Msk;
}

/**
 *  @brief  The function is used to disable UART.
 *
 *  @param  uart        The base address of UART module.
 *
 *  @return None
 */
void UART_Close(UART_T* uart)
{
    uart->INTEN = 0;
}

/**
 *  @brief The function is used to disable UART auto flow control.
 *
 *  @param uart        The base address of UART module.
 *
 *  @return None
 */
void UART_DisableFlowCtrl(UART_T* uart)
{
    uart->INTEN &= ~(UART_INTEN_ATORTSEN_Msk | UART_INTEN_ATOCTSEN_Msk);
}

/**
 *    @brief    The function is used to Enable UART auto flow control.
 *
 *    @param    uart    The base address of UART module.
 *
 *    @return   None
 */
void UART_EnableFlowCtrl(UART_T* uart )
{
    uart->MODEM    |= UART_MODEM_RTSACTLV_Msk;
    uart->MODEM    &= UART_MODEM_RTS_Msk;
    uart->MODEMSTS |= UART_MODEMSTS_CTSACTLV_Msk;
    uart->INTEN    |= UART_INTEN_ATORTSEN_Msk | UART_INTEN_ATOCTSEN_Msk;
}

/**
 *    @brief        Open and set UART function
 *
 *    @param[in]    uart            The pointer of the specified UART module.
 *    @param[in]    u32baudrate     The baudrate of UART module.
 *
 *    @return       None
 *
 *    @details      This function use to enable UART function and set baud-rate.
 */
void UART_Open(UART_T* uart, uint32_t u32baudrate)
{
    uint8_t u8UartClkSrcSel, u8UartClkDivNum;
    uint32_t u32ClkTbl[4] = {__HXT, 0, 0, 0};
    uint32_t u32Baud_Div = 0;

    if(uart == UART0)
    {
        /* Get UART clock source selection */
        u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART0SEL_Msk) >> CLK_CLKSEL1_UART0SEL_Pos;
        
        /* Get UART clock divider number */
        u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos;    
    }    
    else
    {
        /* Get UART clock source selection */
        u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART1SEL_Msk) >> CLK_CLKSEL1_UART1SEL_Pos;
        
        /* Get UART clock divider number */
        u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART1DIV_Msk) >> CLK_CLKDIV0_UART1DIV_Pos;       
    }   

    /* Set UART line configuration */
    uart->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* Set UART Rx and RTS trigger level */
    uart->FIFO &= ~(UART_FIFO_RFITL_Msk | UART_FIFO_RTSTRGLV_Msk);

    /* Get PLL clock frequency if UART clock source selection is PLL */
    if(u8UartClkSrcSel == 1)
        u32ClkTbl[u8UartClkSrcSel] = CLK_GetPLLFreq();
	 
	/* Get HIRC clock frequency if UART clock source selection is HIRC */
    if((u8UartClkSrcSel == 2) || (u8UartClkSrcSel == 3))
    {
        if(((CLK->CLKSEL1 & CLK_CLKSEL0_OSCFSEL_Msk) == CLK_CLKSEL0_HIRCSEL_49M_VCC33) || ((CLK->CLKSEL1 & CLK_CLKSEL0_OSCFSEL_Msk) == CLK_CLKSEL0_HIRCSEL_49M_VCC18))
        {
            u32ClkTbl[2] = 49152000UL;    
            u32ClkTbl[3] = 49152000UL;         
        }
        else
        {
            u32ClkTbl[2] = 48000000UL;    
            u32ClkTbl[3] = 48000000UL;           
        }
    }    
    /* Set UART baud rate */
    if(u32baudrate != 0)
    {
        u32Baud_Div = UART_BAUD_MODE2_DIVIDER((u32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1), u32baudrate);

        if(u32Baud_Div > 0xFFFF)
            uart->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER((u32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1), u32baudrate));
        else
            uart->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);
    }
}


/**
 *    @brief    The function is used to read Rx data from RX FIFO and the data will be stored in pu8RxBuf.
 *
 *    @param    uart            The base address of UART module.
 *    @param    pu8RxBuf        The buffer to receive the data of receive FIFO.
 *    @param    u32ReadBytes    The the read bytes number of data.
 *
 *    @return   u32Count: Receive byte count
 *
 */
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes)
{
    uint32_t  u32Count;

    for(u32Count=0; u32Count < u32ReadBytes; u32Count++) 
	{
        if(uart->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk)  /* Check RX empty => failed */
            return u32Count;
        
        pu8RxBuf[u32Count] = uart->DAT;               /* Get Data from UART RX  */
    }

    return u32Count;
}

/**
 *    @brief    This function use to config UART line setting.
 *
 *    @param    uart            The base address of UART module.
 *    @param    u32baudrate     The register value of baudrate of UART module.
 *                              if u32baudrate = 0, UART baudrate will not change.
 *    @param    u32data_width   The data length of UART module. [UART_WORD_LEN_5 / UART_WORD_LEN_6 / UART_WORD_LEN_7 / UART_WORD_LEN_8]
 *    @param    u32parity       The parity setting (odd/even/none) of UART module. [UART_PARITY_NONE / UART_PARITY_ODD / UART_PARITY_EVEN / UART_PARITY_MARK / UART_PARITY_SPACE]
 *    @param    u32stop_bits    The stop bit length (1/1.5/2 bit) of UART module. [UART_STOP_BIT_1 / UART_STOP_BIT_1_5 / UART_STOP_BIT_2]
 *
 *    @return   None
 */
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits)
{
    uint8_t u8UartClkSrcSel, u8UartClkDivNum;
    uint32_t u32ClkTbl[4] = {__HXT, 0, 0, 0};
    uint32_t u32Baud_Div = 0;
	
    if(u32baudrate != 0) 
	{
		if(uart == UART0)
		{
			/* Get UART clock source selection */
			u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART0SEL_Msk) >> CLK_CLKSEL1_UART0SEL_Pos;
			
			/* Get UART clock divider number */
			u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos;    
		}    
		else
		{
			/* Get UART clock source selection */
			u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART1SEL_Msk) >> CLK_CLKSEL1_UART1SEL_Pos;
			
			/* Get UART clock divider number */
			u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART1DIV_Msk) >> CLK_CLKDIV0_UART1DIV_Pos;       
		} 

		/* Get PLL clock frequency if UART clock source selection is PLL */
		if(u8UartClkSrcSel == 1)
			u32ClkTbl[u8UartClkSrcSel] = CLK_GetPLLFreq();
		 
		/* Get HIRC clock frequency if UART clock source selection is HIRC */
		if((u8UartClkSrcSel == 2) || (u8UartClkSrcSel == 3))
		{
			if(((CLK->CLKSEL1 & CLK_CLKSEL0_OSCFSEL_Msk) == CLK_CLKSEL0_HIRCSEL_49M_VCC33) || ((CLK->CLKSEL1 & CLK_CLKSEL0_OSCFSEL_Msk) == CLK_CLKSEL0_HIRCSEL_49M_VCC18))
			{
				u32ClkTbl[2] = 49152000UL;    
				u32ClkTbl[3] = 49152000UL;         
			}
			else
			{
				u32ClkTbl[2] = 48000000UL;    
				u32ClkTbl[3] = 48000000UL;           
			}
		} 
		
        u32Baud_Div = UART_BAUD_MODE2_DIVIDER((u32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1), u32baudrate);
 
        if(u32Baud_Div > 0xFFFF)
            uart->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(CLK_GetHCLKFreq(), u32baudrate));
        else
            uart->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);
    }

    uart->LINE = u32data_width | u32parity | u32stop_bits;
}


/**
 *    @brief    This function use to set Rx timeout count.
 *
 *    @param    uart    The base address of UART module.
 *    @param    u32TOC  Rx timeout counter.
 *
 *    @return   None
 */
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC)
{
    uart->TOUT = (uart->TOUT & ~UART_TOUT_TOIC_Msk)| (u32TOC);
    uart->INTEN |= UART_INTEN_TOCNTEN_Msk;
}

/**
 *    @brief    The function is to write data into TX buffer to transmit data by UART.
 *
 *    @param    uart            The base address of UART module.
 *    @param    pu8TxBuf        The buffer to send the data to UART transmission FIFO.
 *    @param    u32WriteBytes   The byte number of data.
 *
 *    @return   u32Count: transfer byte count
 */
uint32_t UART_Write(UART_T* uart,uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t  u32Count, u32delayno;

    for(u32Count = 0; u32Count != u32WriteBytes; u32Count++) {
        u32delayno = 0;
        while((uart->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0) { /* Wait Tx empty and Time-out manner */
            u32delayno++;
            if(u32delayno >= 0x40000000)
                return FALSE;
        }
        uart->DAT = pu8TxBuf[u32Count];    /* Send UART Data from buffer */
    }

    return u32Count;
}


/*@}*/ /* end of group I91500_UART_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91500_UART_Driver */

/*@}*/ /* end of group I91500_Device_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
