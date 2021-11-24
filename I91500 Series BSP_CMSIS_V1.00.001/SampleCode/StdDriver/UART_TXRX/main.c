/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief
 *       Transmit and receive data to/from PC terminal through RS232 interface.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Platform.h"

#define RXBUFSIZE      1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RecData[RXBUFSIZE]   = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;

void SYS_Init(void)
{
    /* Unlock Protected Registers */
    SYS_UnlockReg();

    /* Enable Internal OSC49M */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Switch HCLK Clock Source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));

    /* Update System Core Clock */
    /* User can Use SystemCoreClockUpdate() to Calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock Protected Registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Enable UART Module Clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Configure UART0 and Set UART0 Baud Rate */
    UART_Open(UART0, 115200);

    /* Set GPD Multi-function Pins for UART0 TXD(PD.8) and RXD(PD.9) */
    SYS->GPD_MFP  = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD8MFP_Msk|SYS_GPD_MFP_PD9MFP_Msk) ) | (SYS_GPD_MFP_PD8MFP_UART0_TX|SYS_GPD_MFP_PD9MFP_UART0_RX);
}

void UART0_IRQHandler(void)
{
    uint8_t u8InChar = 0xFF;

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk))
    {
        printf("\nReceive Character From Terminal Console(RX):");

        /* Get All The Input Characters */
        while(UART_IS_RX_READY(UART0))
        {
            /* Get The Character from UART Buffer */
            u8InChar = UART_READ(UART0);

            printf("%c ", u8InChar);

            if(u8InChar == '0')
            {
                g_bWait = FALSE;
            }

            /* Check if Buffer Full */
            if(g_u32comRbytes < RXBUFSIZE)
            {
                /* Enqueue The Character */
                g_u8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
                g_u32comRbytes++;
            }
        }
        UART_ENABLE_INT(UART0, UART_INTEN_THREIEN_Msk);
        printf("\nTransmit Character to Terminal Console(TX):");
    }

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_THREINT_Msk))
    {
        uint16_t tmp;
        tmp = g_u32comRtail;
        if(g_u32comRhead != tmp)
        {
            u8InChar = g_u8RecData[g_u32comRhead];
            /* Wait Tx is not Full to Transmit Data */
            while(UART_IS_TX_FULL(UART0));
            UART_WRITE(UART0, u8InChar);
            g_u32comRhead = (g_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
            g_u32comRbytes--;
        }
        printf("\n");
        UART_DISABLE_INT(UART0, UART_INTEN_THREIEN_Msk);
    }
}

/* Main */
int main(void)
{
    SYS_Init();

    UART0_Init();

    printf("\n+------------------------------------------------------------------------+\n");
    printf("|                        Uart Demo Sample                                |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("Press any characters to test TX and RX.\n");
    printf("Press '0' to Exit.\n");

    /* Enable UART RDA/THRE/Time-out Interrupt */
    NVIC_EnableIRQ(UART0_IRQn);
    UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk /*| UART_INTEN_THREIEN_Msk*/ | UART_INTEN_RXTOIEN_Msk));
    while(g_bWait);

    /* Disable UART RDA/THRE/Time-out Interrupt */
    UART_DISABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk /*| UART_INTEN_THREIEN_Msk*/ | UART_INTEN_RXTOIEN_Msk));
    g_bWait = TRUE;
    printf("\nUART Demo Sample End.\n");
}

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
