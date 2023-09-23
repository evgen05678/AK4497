/*
 * AK4497.h
 *
 *  Created on: Sep 21, 2023
 *      Author: User
 */

#ifndef AK4497_AK4497_H_
#define AK4497_AK4497_H_

#include "main.h"
#ifdef USE_FREERTOS
#include "cmsis_os.h"
#elif USE_THREADX
#include "app_threadx.h"
#endif

#define CONTROL1_REG_ADDR		0x00
#define CONTROL2_REG_ADDR		0x01
#define CONTROL3_REG_ADDR		0x02
#define LCH_ATT_REG_ADDR		0x03
#define RCH_ATT_REG_ADDR		0x04
#define CONTROL4_REG_ADDR		0x05
#define DSD1_REG_ADDR			0x06
#define CONTROL5_REG_ADDR		0x07
#define SOUNDCONTROL_REG_ADDR	0x08
#define DSD2_REG_ADDR			0x09
#define CONTROL7_REG_ADDR		0x0A
#define CONTROL8_REG_ADDR		0x0B
#define DFS_READ_REG_ADDR		0x15

typedef union{
	struct{
	uint8_t RSTN:1;
	uint8_t DIF:3;
	uint8_t AFSD:1;
	uint8_t ECS:1;
	uint8_t EXDF:1;
	uint8_t ACKS:1;
	}bits;
	uint8_t reg;
}Control1_reg_u;

typedef union{
	struct{
	uint8_t SMUTE:1;
	uint8_t DEM:2;
	uint8_t DFS:2;
	uint8_t SD:1;
	uint8_t DZFM:1;
	uint8_t DZFE:1;
	}bits;
	uint8_t reg;
}Control2_reg_u;

typedef union{
	struct{
	uint8_t SLOW:1;
	uint8_t SELLR:1;
	uint8_t DZFB:1;
	uint8_t MONO:1;
	uint8_t DCKB:1;
	uint8_t DCKS:1;
	uint8_t empty_bit:1;
	uint8_t DP:1;
	}bits;
	uint8_t reg;
}Control3_reg_u;

typedef union{
	struct{
	uint8_t ATT:8;
	}bits;
	uint8_t reg;
}LCH_ATT_reg_u;

typedef union{
	struct{
	uint8_t ATT:8;
	}bits;
	uint8_t reg;
}RCH_ATT_reg_u;

typedef union{
	struct{
	uint8_t SSLOW:1;
	uint8_t DFS2:1;
	uint8_t empty_bit:4;
	uint8_t INVR:1;
	uint8_t INVL:1;
	}bits;
	uint8_t reg;
}Control4_reg_u;

typedef union{
	struct{
	uint8_t DSDSEL0:1;
	uint8_t DSDD:1;
	uint8_t empty_bit:1;
	uint8_t DMRE:1;
	uint8_t DMC:1;
	uint8_t DMR:1;
	uint8_t DML:1;
	uint8_t DDM:1;
	}bits;
	uint8_t reg;
}DSD1_reg_u;

typedef union{
	struct{
		uint8_t SYNCE:1;
		uint8_t GC:3;
		uint8_t empty_bits:4;
	}bits;
	uint8_t reg;
}Control5_reg_u;


typedef union{
	struct{
		uint8_t SC:3;
		uint8_t HLOAD:1;
		uint8_t empty_bits:4;
	}bits;
	uint8_t reg;
}SoundControl_reg_u;

typedef union{
	struct{
		uint8_t DSDSEL1:1;
		uint8_t DSDF:1;
		uint8_t DSDPATH:1;
		uint8_t empty_bits:5;
	}bits;
	uint8_t reg;
}DSD2_reg_u;


typedef union{
	struct{
	uint8_t empty_bit_2:2;
	uint8_t PW:1;
	uint8_t empty_bit_1:1;
	uint8_t SDS:2;
	uint8_t TDM:2;
	}bits;
	uint8_t reg;
}Control7_reg_u;

typedef union{
	struct{
	uint8_t TEST:1;
	uint8_t DCHAIN:1;
	uint8_t empty_bit_2:2;
	uint8_t SDS0:1;
	uint8_t empty_bit_1:1;
	uint8_t ATS:2;
	}bits;
	uint8_t reg;
}Control8_reg_u;

typedef union{
	struct{
		uint8_t ADFS:3;
		uint8_t empty_bits:5;
	}bits;
	uint8_t reg;
}DFS_Read_reg_u;

typedef struct{
	Control1_reg_u 		Control1;
	Control2_reg_u 		Control2;
	Control3_reg_u 		Control3;
	LCH_ATT_reg_u  		LCH_ATT;
	RCH_ATT_reg_u  		RCH_ATT;
	Control4_reg_u 		Control4;
	DSD1_reg_u	   		DSD1;
	Control5_reg_u 		Control5;
	SoundControl_reg_u 	SoundControl;
	DSD2_reg_u			DSD2;
	Control7_reg_u		Control7;
	Control8_reg_u		Control8;
	DFS_Read_reg_u		DFS_Read;
}register_map_s;

typedef struct{
	I2C_HandleTypeDef *i2c_port;
	uint8_t DevAddr_read;
	uint8_t DevAddr_write;
	register_map_s regMap;
#ifdef USE_FREERTOS
	SemaphoreHandle_t *mutex;
#elif USE_THREADX
	TX_MUTEX *mutex;
#endif
}dev_s;

typedef enum{
	PCM_MODE,
	DSD_MODE
}Mode;

typedef enum {
	SHARP_ROLL_OFF,
	SLOW_ROLL_OFF,
	SHORT_DELAY_SHARP,
	SHORT_DELAY_SLOW,
	SUPER_SLOW,
	SUPER_SLOW_,
	LOW_DISPERSION_SHORT_DELAY,
	RESERVED
}PCM_Filters;

typedef enum {
	LSB_16bit,
	LSB_20bit,
	MSB_24bit,
	I2S_16_24bit,
	LSB_24bit,
	LSB_32bit,
	MSB_32bit,
	I2S_32bit
}Audio_Interface_Format_e;

typedef enum {
	Normal_Mode,	//default
	TDM128_Mode,
	TDM256_Mode,
	TDM512_Mode
}Audio_Interface_Mode_e;

typedef enum {
	PCM_2_8vpp_DSD_2_8vpp = 0, //default
	PCM_2_8vpp_DSD_2_5vpp,
	PCM_2_5vpp_DSD_2_5vpp,
	PCM_3_75vpp_DSD_3_75vpp = 4,
	PCM_3_75vpp_DSD_2_5vpp,
}Gain_Adjustment_e;

typedef enum {
	Only_LEFT_CH,
	Only_RIGHT_CH
}Mono_mode_e;

typedef enum {
	PINS_16_17_19,
	PINS_3_4_5
}Input_Data_Pins_e;

typedef enum {
	UNMUTED,
	MUTED
}DAC_mute_e;

typedef enum {
  Manual_Setting_Mode,
  Auto_Setting_Mode,
  FS_AutoDetect_Mode
}Clock_Setting_Mode;

HAL_StatusTypeDef AK4497_Init(dev_s *dev);
HAL_StatusTypeDef AK4497_Configure(dev_s *dev);
HAL_StatusTypeDef AK4497_Write_Register(dev_s *dev, uint8_t regAddr, uint8_t *value);
HAL_StatusTypeDef AK4497_Read_Register(dev_s *dev, uint8_t regAddr, uint8_t *pRegister);
void AK4497_DSD_Stream_Select(dev_s *dev, uint8_t stream);
void AK4497_PCM_DSD_Mode(dev_s *dev, Mode mode);
Mode AK4497_is_DSD_Mode(dev_s *dev);
void AK4497_PCM_Filter_Select(dev_s *dev, PCM_Filters filter);
PCM_Filters AK4497_Get_PCM_Filter(dev_s *dev);
void AK4497_Set_Audio_Format(dev_s *dev, Audio_Interface_Format_e format, Audio_Interface_Mode_e mode );
void AK4497_Set_Output_Volume(dev_s *dev, uint8_t volume);
void AK4497_Set_Gain_Adjustment(dev_s *dev, Gain_Adjustment_e gain);
void AK4497_Set_MONO_Mode(dev_s *dev, Mono_mode_e mode);
void AK4497_DSD_InputData_Pins_Select(dev_s *dev, Input_Data_Pins_e pins);
void AK4497_SoftMute(dev_s *dev, DAC_mute_e mute);
void AK4497_Set_SystemClock_Mode(dev_s *dev, Clock_Setting_Mode mode);
#endif /* AK4497_AK4497_H_ */
