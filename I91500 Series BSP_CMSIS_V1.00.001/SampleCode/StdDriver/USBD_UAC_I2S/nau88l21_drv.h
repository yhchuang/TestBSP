#ifndef _NAU88L21_DRV_H_
#define _NAU88L21_DRV_H_


#define NAU88L21_DEVID				        0x1B
#define RETRY 								3
//0x1B(CSB=0) or 0x54(CSB=1)

/*Gain level 0 ~ 15*/
#define SIDETONE_GAIN_L 7    
#define SIDETONE_GAIN_R 7         


struct S_NAU88L21_I2CCMD {
	uint16_t u16Reg;
	uint16_t u16Value;
};

enum{

	_DISABLE = 0,
	_ENABLE  = 1
};



//----------------------------------------------------------------------------
//  Functions Definition
//----------------------------------------------------------------------------

void nau88l21_pga_control(uint8_t u8LeftVolume, uint8_t u8RightVolume);
int32_t nau88l21_reg_write(uint16_t u16reg, uint16_t u16data);
int32_t nau88l21_reg_read(uint16_t u16reg, uint16_t *pu16val);
int32_t nau88l21_init(void);
void nau88l21_reset(void);
void nau88l21_channel_reverse(uint8_t u8sw);
void nau88l21_sidetone_enable(uint8_t u8sw);

//----------------------------------------------------------------------------
//  NAU88L21 Registers Definition
//----------------------------------------------------------------------------
#define REG_R00_SOFTWARE_RST                                                0x00 
#define REG_R01_ENA_CTRL                                                    0x01
#define	REG_R03_CLK_DIVIDER	                                                0x03
#define	REG_R04_FLL1			                                            0x04
#define REG_R05_FLL2                                                        0x05
#define REG_R06_FLL3                                                        0x06
#define REG_R07_FLL4                                                        0x07
#define REG_R08_FLL5                                                        0x08
#define REG_R09_FLL6                                                        0x09
#define REG_R0A_FLL7                                                        0x0A
#define REG_R0B_FLL8                                                        0x0B									
#define REG_R0D_JACK_DET_CTRL                                               0x0D
#define REG_R0F_INTERRUPT_MASK                                              0x0F
#define REG_R10_IRQ_STATUS                                                  0x10
#define REG_R11_INT_CLR_KEY_STATUS  										0x11
#define REG_R12_INTERRUPT_DIS_CTRL  										0x12
#define REG_R13_DMIC_CTRL           										0x13
#define REG_R1A_GPIO12_CTRL         										0x1A
#define REG_R1B_TDM_CTRL            										0x1B
#define REG_R1C_I2S_PCM_CTRL1       										0x1C
#define REG_R1D_I2S_PCM_CTRL2       										0x1D
#define REG_R1E_LEFT_TIME_SLOT      										0x1E
#define REG_R1F_RIGHT_TIME_SLOT     										0x1F
#define REG_R21_BIQ0_COF1           										0x21
#define REG_R22_BIQ0_COF2           										0x22
#define REG_R23_BIQ0_COF3           										0x23
#define REG_R24_BIQ0_COF4           										0x24
#define REG_R25_BIQ0_COF5           										0x25
#define REG_R26_BIQ0_COF6           										0x26
#define REG_R27_BIQ0_COF7           										0x27
#define REG_R28_BIQ0_COF8           										0x28
#define REG_R29_BIQ0_COF9           										0x29
#define REG_R2A_BIQ0_COF10          										0x2A
#define REG_R2B_ADC_RATE            										0x2B
#define REG_R2C_DAC_CTRL1           										0x2C
#define REG_R2D_DAC_CTRL2           										0x2D
#define REG_R2F_DAC_DGAIN_CTRL      										0x2F
#define REG_R30_ADC_DGAIN_CTRL      										0x30
#define REG_R31_MUTE_CTRL           										0x31
#define REG_R32_HSVOL_CTRL          										0x32
#define REG_R34_DACR_CTRL                                                   0x34
#define REG_R35_ADC_DGAIN_CTRL1     										0x35
#define REG_R36_ADC_DRC_KNEE_IP12   										0x36
#define REG_R37_ADC_DRC_KNEE_IP34   										0x37
#define REG_R38_ADC_DRC_SLOPES      										0x38
#define REG_R39_ADC_DRC_ATKDCY      										0x39
#define REG_R3A_DAC_DRC_KNEE_IP12   										0x3A
#define REG_R3B_DAC_DRC_KNEE_IP34   										0x3B
#define REG_R3C_DAC_DRC_SLOPES      										0x3C
#define REG_R3D_DAC_DRC_ATKDCY      										0x3D
#define REG_R41_BIQ1_COF1           										0x41
#define REG_R42_BIQ1_COF2           										0x42
#define REG_R43_BIQ1_COF3           										0x43
#define REG_R44_BIQ1_COF4           										0x44
#define REG_R45_BIQ1_COF5           										0x45
#define REG_R46_BIQ1_COF6           										0x46
#define REG_R47_BIQ1_COF7           										0x47
#define REG_R48_BIQ1_COF8           										0x48
#define REG_R49_BIQ1_COF9           										0x49
#define REG_R4A_BIQ1_COF10          										0x4A
#define REG_R4B_CLASSG_CTRL                                                 0x4B
#define REG_R4C_IMM_MODE_CTRL       										0x4C
#define REG_R4D_IMM_RMS_L           										0x4D
#define REG_R53_OTPDOUT_1           										0x53
#define REG_R54_OTPDOUT_2           										0x54
#define REG_R55_MISC_CTRL           										0x55
#define REG_R58_I2C_DEVICE_ID       										0x58
#define REG_R59_SARDOUT_RAM_STATUS  										0x59
#define REG_R66_BIAS_ADJ            										0x66
#define REG_R69_ANALOG_CONTROL_1    										0x69
#define REG_R6A_ANALOG_CONTROL_2    										0x6A
//#define no name define                                                0x6B
#define REG_R71_ANALOG_ADC_1        										0x71
#define REG_R72_ANALOG_ADC_2        										0x72
#define REG_R73_RDAC                										0x73
#define REG_R74_MIC_BIAS            										0x74
#define REG_R76_BOOST               										0x76
#define REG_R77_FEPGA               										0x77
#define REG_R7E_PGA_GAIN            										0x7E
#define REG_R7F_POWER_UP_CONTROL    										0x7F
#define REG_R80_CHARGE_PUMP_AND_POWER_DOWN_CONTROL                          0x80
#define REG_R81_CHARGE_PUMP_INPUT_READ                                      0x81
#define REG_R82_GENERAL_STATUS                                              0x82
                                                     











#endif



