/*************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate how to implement a USB audio class device.
 *           DAC is used in this sample code to play the audio data from Host.
 *           It also supports to record data from SDADC to Host.
 * @version  1.0.0
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <string.h> 
#include "Platform.h"
#include "peripheral.h"

int main(void)
{
    SYS_Init(); // Initiate System.
    USBD_Init();    // Initiate USBD and AUDIO configuration.
    PDMA_Init();    // Initiate PDMA configuration.
    ANA_Init(); // Initiate VMID and MICBIAS configuration.
	SDADC_Init();   // Initiate SDADC configuration.
	DAC_Init(); // Initiate DAC configuration.     
    while(1)
    {
        AudioDeviceCtrl();  // DAC and SDADC enable control.   
        RecordVolumeControl();  // SDADC gain control.   
    }; 
}
