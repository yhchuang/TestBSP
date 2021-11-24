/*************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate how to implement a USB audio class device.
 *           Audio codec is used in this sample code to play the audio data from Host.
 *           It also supports to record data from audio codec to Host.
 *           Please connect EVB-I91500ADI board to EVB-AUDIO board.
 * @version  1.0.0
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <string.h> 
#include "Platform.h"
#include "peripheral.h"
#include "nau88l21_drv.h"

int main(void)
{
    SYS_Init(); // Initiate System.
    USBD_Init();    // Initiate USBD and AUDIO configuration.
    PDMA_Init();    // Initiate PDMA configuration.
    I2C0_Init();    // Initiate I2C configuration.
    nau88l21_init();    // Initiate audio codec NAU88L21 configuration.     
    I2S0_Init();    // Initiate I2S configuration.
    while(1)
    {
        AudioDeviceCtrl();  // I2S TX and RX control.   
        RecordVolumeControl();  // Audio codec PGA control.
    }; 
}
