/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 20/11/19 2:52p $
 * @brief    I2C Driver Sample Code
 *           This is a I2C_master mode demo and need to be tested with I2C_Slave demo.
 * @note
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Platform.h"
#include "VI2C/VI2C.h"

// Pre-declare function.
void System_Initiate(void);

// I2C Master API's function.
BOOL I2C_Master_Setup(void);
void I2C_Master_Send(UINT8 u8SlaveNo,PUINT8 pu8Buf,UINT32 u32Count);
void I2C_Master_Receive(UINT8 u8SlaveNo,PUINT8 pu8Buf,UINT32 u32Count);
BOOL I2C_Master_IsBusy(void);
void I2C_Master_IRQProcess(void);

// Show message.
void Show_DemoLabel(void);
void Show_MasterSetup(BOOL bSuccess);
void Show_DemoHwPrepare(void);
void Show_MasterTx(void);
void Show_MasterTxDone(void);
void Show_MasterRx(void);
void Show_MasterRxDone(void);
void Show_DataVerity(BOOL bSuccess);
void Show_DemoEnd(void);

int main(void)
{
	UINT8 au8SendBuf[64],au8ReceiveBuf[64],u8i;

	// Initiate system clock
	System_Initiate();	
	// Initiate buffer data.
	for( u8i=0; u8i<64; u8i++ )
	{
		au8SendBuf[u8i] = u8i;
		au8ReceiveBuf[u8i] = 0;
	}
	// Message : Demo label.
	Show_DemoLabel();
	
	// Setup VI2C0 to be a I2C master
	// Message : I2C master setup success/fail.
	if( I2C_Master_Setup() == TRUE ) 
		Show_MasterSetup(TRUE);
	else
	{
		Show_MasterSetup(FALSE);
		while(1);
	}
	
	// Message : When hardware demo prepare(include GPIO pin connect.),user press 'ENTER'
	Show_DemoHwPrepare();

	// Message : Send data to slave.
	Show_MasterTx();
	// Send data(0~63) from auSendBuf to '0x15'(device address).
	I2C_Master_Send(0x15,au8SendBuf,64);
	// Wait send process finish.
	while(I2C_Master_IsBusy()==TRUE);
	
	// Message : Receive data from slave
	Show_MasterRx();
	// Receive data from '0x15'(device address) to auReceiveBuf.
	I2C_Master_Receive(0x15,au8ReceiveBuf,64);	
	// Wait receive process finish.
	while(I2C_Master_IsBusy()==TRUE);
	
	// Compare au8SendBuf & au8ReceiveBuf.
	if( memcmp(au8SendBuf,au8ReceiveBuf,64) == 0 )
		Show_DataVerity(TRUE);
	else
		Show_DataVerity(FALSE);
	
	// Message : End demo(while in).
	Show_DemoEnd();
}

// I2C0's IRQ handler.
void I2C0_IRQHandler(void)
{
	I2C_Master_IRQProcess();
}

// VI2C provide callback API when data send complete.
// User could add some action here.
// Note. If VI2C_CFG_INTEN is set, this callback function was in I2C's IRQ.
void VI2C_SendComplete(UINT8 u8VI2CNo)
{
	// Message : Send data done.
	// Suggest "not" to use 'printf' in the real case.
	Show_MasterTxDone();
}

// VI2C provide callback API when data receive complete.
// User could add some action here.
// Note. If VI2C_CFG_INTEN is set, this callback function was in I2C's IRQ.
void VI2C_ReceiveComplete(UINT8 u8VI2CNo)
{
	// Message : Receive data done.
	// Suggest "not" to use 'printf' in the real case.
	Show_MasterRxDone();
}

// I2C Master =========================================================================
#define I2C0_MASTER_PINS_MSK    (SYS_GPA_MFP_PA1MFP_Msk|SYS_GPA_MFP_PA2MFP_Msk)
#define I2C0_MASTER_PINS        (SYS_GPA_MFP_PA1MFP_I2C0_SCL|SYS_GPA_MFP_PA2MFP_I2C0_SDA)	

BOOL I2C_Master_Setup(void)
{	
    // Open I2C0 and set clock to 100k 
    if( VI2C_Open(0, 100000, E_VI2C_CLK_DEFAULT, TRUE) )
	{
		// Master, Enable interrupt, Disable time-out 
		VI2C_Config(0,VI2C_CFG_MASTER|VI2C_CFG_INTEN|VI2C_CFG_TODIS);
		// Init I/O multi-function ; I2C0: GPA2=SDA, GPA1= SCL
		SYS->GPA_MFP  = (SYS->GPA_MFP & (~I2C0_MASTER_PINS_MSK) ) | I2C0_MASTER_PINS;
		return TRUE;
	}
	return FALSE;
}
void I2C_Master_Send(UINT8 u8DeviceAddr,PUINT8 pu8Buf,UINT32 u32Count)
{
	// Config device address in master mode.
	VI2C_ConfigDeviceAddress(0,u8DeviceAddr);
	// Set Tx data buffer & data count.
	VI2C_SetTxData(0,pu8Buf,u32Count);
	// Start send data from VI2C0 to device. 
	VI2C_Start(0);
}
void I2C_Master_Receive(UINT8 u8DeviceAddr,PUINT8 pu8Buf,UINT32 u32Count)
{
	// Config device address in master mode.
	VI2C_ConfigDeviceAddress(0,u8DeviceAddr);
	// Set Tx data buffer & data count.
	VI2C_SetRxData(0,pu8Buf,u32Count);
	// Start send data from VI2C0 to device. 
	VI2C_Start(0);
}
void I2C_Master_IRQProcess(void)
{
	VI2C_Process(0);
}
BOOL I2C_Master_IsBusy(void)
{
	return VI2C_IsBusy(0);
}

// =====================================================================================
void System_Initiate(void)
{
    // Unlock protected registers
    SYS_UnlockReg();
    // Enable clock source 
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    // Switch HCLK clock source to HIRC
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
	// Update System Core Clock
    SystemCoreClockUpdate();
    // Lock protected registers
	
	// Config gpio pin multi-functon.
	SYS->GPA_MFP = ( SYS->GPA_MFP & ~(SYS_GPA_MFP_PA0MFP_Msk|SYS_GPA_MFP_PA3MFP_Msk|SYS_GPA_MFP_PA4MFP_Msk) ) | (SYS_GPA_MFP_PA0MFP_GPIO|SYS_GPA_MFP_PA3MFP_GPIO|SYS_GPA_MFP_PA4MFP_GPIO);
	// Config gpio mode.
	GPIO_SetMode( PA, BIT0, GPIO_MODE_PULLUP);
		GPIO_SetMode( PA, BIT3, GPIO_MODE_PULLUP);
			GPIO_SetMode( PA, BIT4, GPIO_MODE_PULLUP);
	// Light off LED GPA3 and GPA4.
	GPIO_SET_OUT_DATA( PA, GPIO_GET_OUT_DATA(PA)|(BIT3));
    GPIO_SET_OUT_DATA( PA, GPIO_GET_OUT_DATA(PA)|(BIT4));
	
    SYS_LockReg();	
}
void Show_DemoLabel(void)
{
	printf("\r\n+------------------------------------------------------------------------+\r\n");
	printf("|                       I2C Driver Sample Code                           |\r\n");
	printf("+------------------------------------------------------------------------+\r\n");			
}
void Show_MasterSetup(BOOL bSuccess)
{
	printf("(1) Setup VI2C0 to be a I2C master.\r\n");
	printf("    1. Open VI2C0 interface(frequency = 100000).\r\n");
	printf("    2. Config I2C0's gpio multi-function.\r\n");
    (bSuccess)?printf("    VI2C setup success.\r\n"):printf("    VI2C setup fail.\r\n");			
}
void Show_DemoHwPrepare(void)
{
	printf("(2) Please make sure GPA1 & GPA2 connect I2C_Slave.\r\n");	
	printf("    Before master start to send, make sure slave started listening\r\n");


	// Light on GPA4
	GPIO_SET_OUT_DATA( PA, GPIO_GET_OUT_DATA(PA)&(~BIT4));
}
void Show_MasterTx(void)
{
	printf("(3) Send data(0~63) from auSendBuf to '0x15'(device address).\r\n");
	printf("    1. Config device address into VI2C0 in master mode.\r\n");
	printf("    2. Set Tx data buffer & data count into VI2C0 .\r\n");
	printf("    3. Start VI2C0.\r\n");
	printf("Press 'Enter' for master start to data transter .\r\n");
  getchar();
	printf("Master start to data transfer .\r\n"); 	
}
void Show_MasterTxDone(void)
{
	printf("    VI2C0 send done.\r\n");
}
void Show_MasterRx(void)
{
	printf("(4) Receive data from '0x15'(device address) to auReceiveBuf.\r\n");
	printf("    1. Config device address into VI2C0 in master mode.\r\n");
	printf("    2. Set Rx data buffer & data count into VI2C0 .\r\n");
	printf("    3. Start VI2C0.\r\n");
}	
void Show_MasterRxDone(void)
{
	printf("    VI2C0 receive done.\r\n");	
}
void Show_DataVerity(BOOL bSuccess)
{
	printf("(5) Send & receive data verity ..."); 
    (bSuccess)?printf("success.\r\n"):printf("fail.\r\n");
	if(bSuccess)
		GPIO_SET_OUT_DATA( PA, GPIO_GET_OUT_DATA(PA)&(~BIT3));
}
void Show_DemoEnd(void)
{
	printf("(6) VI2C_Master demo end.\r\n"); 
	while(1);
}
