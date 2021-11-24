/*****************************************************************************************************************
 *  @file           nau88l21_drv.c
 *
 *  @brief          
 *
 *  @Author         
 *
 *  @history:
 *                  Date          Name            Comments
 *                  2021/03/28                    0.1
 *
 *  Copyright (C) 2021 Nuvoton Technology. All rights reserved.
 *  
 *****************************************************************************************************************/
#include <stdio.h>
#include "Platform.h"
#include "peripheral.h"
#include "nau88l21_drv.h"


static const unsigned char NAU88L21_reg_defaults[] = {
	REG_R00_SOFTWARE_RST 					    ,						
	REG_R01_ENA_CTRL 		 					,						
	REG_R03_CLK_DIVIDER						    ,						
	REG_R04_FLL1								,						
	REG_R05_FLL2								,						
	REG_R06_FLL3								,						
	REG_R07_FLL4								,						
	REG_R08_FLL5								,						
	REG_R09_FLL6								,						
	REG_R0A_FLL7								,						
	REG_R0B_FLL8								,						
	REG_R0D_JACK_DET_CTRL					    ,						
	REG_R0F_INTERRUPT_MASK				        ,						
	REG_R10_IRQ_STATUS						    ,						
	REG_R11_INT_CLR_KEY_STATUS  	            ,						
	REG_R12_INTERRUPT_DIS_CTRL  	            ,						
	REG_R13_DMIC_CTRL           	            ,						
	REG_R1A_GPIO12_CTRL         	            ,						
	REG_R1B_TDM_CTRL            	            ,						
	REG_R1C_I2S_PCM_CTRL1       	            ,						
	REG_R1D_I2S_PCM_CTRL2       	            ,						
	REG_R1E_LEFT_TIME_SLOT      	            ,						
	REG_R1F_RIGHT_TIME_SLOT     	            ,						
	REG_R21_BIQ0_COF1           	            ,						
	REG_R22_BIQ0_COF2           	            ,						
	REG_R23_BIQ0_COF3           	            ,						
	REG_R24_BIQ0_COF4           	            ,						
	REG_R25_BIQ0_COF5           	            ,						
	REG_R26_BIQ0_COF6           	            ,						
	REG_R27_BIQ0_COF7           	            ,						
	REG_R28_BIQ0_COF8           	            ,						
	REG_R29_BIQ0_COF9           	            ,						
	REG_R2A_BIQ0_COF10          	            ,						
	REG_R2B_ADC_RATE            	            ,						
	REG_R2C_DAC_CTRL1           	            ,						
	REG_R2D_DAC_CTRL2           	            ,						
	REG_R2F_DAC_DGAIN_CTRL      	            ,						
	REG_R30_ADC_DGAIN_CTRL      	            ,						
	REG_R31_MUTE_CTRL           	            ,						
	REG_R32_HSVOL_CTRL          	            ,						
	REG_R34_DACR_CTRL							,						
	REG_R35_ADC_DGAIN_CTRL1     	            ,						
	REG_R36_ADC_DRC_KNEE_IP12   	            ,						
	REG_R37_ADC_DRC_KNEE_IP34   	            ,						
	REG_R38_ADC_DRC_SLOPES      	            ,						
	REG_R39_ADC_DRC_ATKDCY      	            ,						
	REG_R3A_DAC_DRC_KNEE_IP12   	            ,						
	REG_R3B_DAC_DRC_KNEE_IP34   	            ,						
	REG_R3C_DAC_DRC_SLOPES      	            ,						
	REG_R3D_DAC_DRC_ATKDCY      	            ,						
	REG_R41_BIQ1_COF1           	            ,						
	REG_R42_BIQ1_COF2           	            ,						
	REG_R43_BIQ1_COF3           	            ,						
	REG_R44_BIQ1_COF4           	            ,						
	REG_R45_BIQ1_COF5           	            ,						
	REG_R46_BIQ1_COF6           	            ,						
	REG_R47_BIQ1_COF7           	            ,						
	REG_R48_BIQ1_COF8           	            ,						
	REG_R49_BIQ1_COF9           	            ,						
	REG_R4A_BIQ1_COF10          	            ,						
	REG_R4B_CLASSG_CTRL						    ,						
	REG_R4C_IMM_MODE_CTRL       	            ,						
	REG_R4D_IMM_RMS_L           	            ,						
	REG_R53_OTPDOUT_1           	            ,						
	REG_R54_OTPDOUT_2           	            ,						
    REG_R55_MISC_CTRL           	            ,						
	REG_R58_I2C_DEVICE_ID       	            ,						
    REG_R59_SARDOUT_RAM_STATUS  	            ,						
	REG_R66_BIAS_ADJ            	            ,						
	REG_R69_ANALOG_CONTROL_1    	            ,						
	REG_R6A_ANALOG_CONTROL_2    	            ,						
  //#define no name define        0x6B          ,						
	REG_R71_ANALOG_ADC_1        	            ,						
	REG_R72_ANALOG_ADC_2        	            ,						
	REG_R73_RDAC                	            ,						
	REG_R74_MIC_BIAS            	            ,						
	REG_R76_BOOST               	            ,						
	REG_R77_FEPGA               	            ,						
	REG_R7E_PGA_GAIN            	            ,						
	REG_R7F_POWER_UP_CONTROL    	            ,						
	REG_R80_CHARGE_PUMP_AND_POWER_DOWN_CONTROL  ,
	REG_R81_CHARGE_PUMP_INPUT_READ              ,            
	REG_R82_GENERAL_STATUS                    
	
};


static const struct S_NAU88L21_I2CCMD asMIC_Cmd_88L21_MicI[] = {
{	REG_R00_SOFTWARE_RST												,	0xFFFF	},
{	REG_R00_SOFTWARE_RST												,	0xFFFF	},
{	REG_R31_MUTE_CTRL	    											,	0x0300	},
{	REG_R66_BIAS_ADJ	        									    ,	0x0060	},
{	REG_R76_BOOST	            									    ,	0x1140	},
{	REG_R01_ENA_CTRL        										    ,	0x0FD6	},
{	REG_R03_CLK_DIVIDER     										    ,	0x0410	},
{	REG_R1D_I2S_PCM_CTRL2	    									    ,	0x0000	},
{	REG_R72_ANALOG_ADC_2	    									    ,	0x0170	},
{	REG_R2B_ADC_RATE	        									    ,	0x0003	},
{	REG_R73_RDAC														,	0x3308	},
{	REG_R76_BOOST														,	0x7140	},
{	REG_R32_HSVOL_CTRL													,	0x8000	},
{	REG_R35_ADC_DGAIN_CTRL1											    ,	0xF3F3	},
{	REG_R4B_CLASSG_CTRL													,	0x2007	},
{	REG_R6A_ANALOG_CONTROL_2										    ,	0x1003	},
{	REG_R80_CHARGE_PUMP_AND_POWER_DOWN_CONTROL	                        ,	0x0020	},
{	REG_R74_MIC_BIAS													,	0x1506	},
{	REG_R77_FEPGA														,	0x0000	},
{	REG_R7E_PGA_GAIN												    ,	0x1111	},
{	REG_R7F_POWER_UP_CONTROL										    ,	0x0030	},
{	REG_R7F_POWER_UP_CONTROL										    ,	0x003C	},
{	REG_R7F_POWER_UP_CONTROL										    ,	0xC03F	},
{	REG_R32_HSVOL_CTRL													,	0x0000	},
{	REG_R80_CHARGE_PUMP_AND_POWER_DOWN_CONTROL	                        ,	0x0720	},
{	REG_R0D_JACK_DET_CTRL												,	0xC000	},
{	REG_R31_MUTE_CTRL													,	0x1004	},
{	REG_R21_BIQ0_COF1													,	0x23CE	},
{	REG_R22_BIQ0_COF2													,	0x0006	},
{	REG_R23_BIQ0_COF3													,	0xDE80	},
{	REG_R24_BIQ0_COF4													,	0x0000	},
{	REG_R25_BIQ0_COF5													,	0xEEB0	},
{	REG_R26_BIQ0_COF6													,	0x0000	},
{	REG_R27_BIQ0_COF7													,	0x22A0	},
{	REG_R28_BIQ0_COF8													,	0x0006	},
{	REG_R29_BIQ0_COF9													,	0xEEB0	},
{	REG_R2A_BIQ0_COF10													,	0x0008	},
};

int32_t nau88l21_reg_write(uint16_t u16reg, uint16_t u16data)
{
    uint8_t u8i;
    int32_t i32ret;
    
    for(u8i=0; u8i<RETRY; u8i++)
    {
        i32ret = i2c_reg_write_16b(I2C0, NAU88L21_DEVID, u16reg, u16data);
        if(i32ret == 0)
            return 0;
    }   
    return i32ret;
}



int32_t nau88l21_reg_read(uint16_t u16reg, uint16_t *pu16val)
{
    uint8_t u8i;
    int32_t i32ret;
    
    for(u8i=0; u8i<RETRY; u8i++)
    {
        i32ret = i2c_reg_read_16b(I2C0, NAU88L21_DEVID, u16reg, pu16val);
        if(i32ret == 0)
        {
            return 0;
        }
    } 
    return i32ret;
}



int32_t nau88l21_init(void)
{
    uint16_t u16i;
		uint16_t cmd_init_array_size;
    int32_t i32ret;
		const struct S_NAU88L21_I2CCMD *cmd_init_array;
    
    cmd_init_array = asMIC_Cmd_88L21_MicI;
    cmd_init_array_size = sizeof(asMIC_Cmd_88L21_MicI);
    
    for(u16i=0; u16i<(cmd_init_array_size / sizeof(struct S_NAU88L21_I2CCMD)); u16i++)
    {
        i32ret = nau88l21_reg_write(cmd_init_array[u16i].u16Reg, cmd_init_array[u16i].u16Value);
        if(i32ret)
        {
            return i32ret;  
        }
    }
    return i32ret;
}

void nau88l21_pga_control(uint8_t u8LeftVolume, uint8_t u8RightVolume)
{
    nau88l21_reg_write(REG_R7E_PGA_GAIN, (uint16_t)(((u8LeftVolume + 1) << 8) | (u8RightVolume + 1)));
}


void nau88l21_channel_reverse(uint8_t u8sw)
{

	if(u8sw == _ENABLE)
	{
		nau88l21_reg_write(REG_R31_MUTE_CTRL,0x3200);
		nau88l21_reg_write(REG_R1B_TDM_CTRL, 0x1800);
		nau88l21_reg_write(REG_R31_MUTE_CTRL,0x3000);
	}
	else
	{
		nau88l21_reg_write(REG_R31_MUTE_CTRL,0x3200);
		nau88l21_reg_write(REG_R1B_TDM_CTRL, 0x0000);
		nau88l21_reg_write(REG_R31_MUTE_CTRL,0x3000);
	}
}

void nau88l21_sidetone_enable(uint8_t u8sw)
{
	uint16_t u16RegValue = (uint16_t)((SIDETONE_GAIN_L << 12) | (SIDETONE_GAIN_R << 8) | 0x0040);

	static uint8_t u8PreStatus = _DISABLE;

	if(u8PreStatus == u8sw)   
	return;
	u8PreStatus = u8sw;

	if(u8sw == _ENABLE)
	nau88l21_reg_write(REG_R30_ADC_DGAIN_CTRL,u16RegValue);
	else
	nau88l21_reg_write(REG_R30_ADC_DGAIN_CTRL,0x0000);
}


void nau88l21_reset(void)
{
	nau88l21_reg_write(REG_R00_SOFTWARE_RST,0xFFFF);
    nau88l21_reg_write(REG_R00_SOFTWARE_RST,0xFFFF);
}


int32_t nau88l21_reg_dump(void)
{
    uint16_t u16i;
    int32_t i32ret;
    uint16_t u16data;

    uint16_t u16RegSize = sizeof(NAU88L21_reg_defaults);

    for(u16i=0; u16i<  u16RegSize; u16i++)
    {
        i32ret = nau88l21_reg_read((uint16_t)NAU88L21_reg_defaults[u16i], &u16data);
        printf("0x%04X <= 0x%04X\r\n", NAU88L21_reg_defaults[u16i], u16data);
        if(i32ret)
        {
            return i32ret;
        }
    }
           
    return i32ret;
}




















