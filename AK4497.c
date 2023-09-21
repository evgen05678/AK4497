/*
 * AK4497.c
 *
 *  Created on: Sep 21, 2023
 *      Author: User
 */

#include "AK4497.h"

/**
 * @fn HAL_StatusTypeDef AK4497_Init(dev_s*)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @return
 */
HAL_StatusTypeDef AK4497_Init(dev_s *dev) {
	HAL_StatusTypeDef status;
	uint8_t rxBuffer[21];
#ifdef USE_FREERTOS

#elif USE_THREADX
	tx_mutex_get(dev->mutex, TX_WAIT_FOREVER);
#endif
	status = HAL_I2C_Mem_Read(dev->i2c_port, dev->DevAddr, CONTROL1_REG_ADDR,
			I2C_MEMADD_SIZE_8BIT, rxBuffer, 21, 100);
#ifdef USE_FREERTOS

#elif USE_THREADX
	tx_mutex_put(dev->mutex);
#endif
	dev->regMap.Control1.reg = rxBuffer[CONTROL1_REG_ADDR];
	dev->regMap.Control2.reg = rxBuffer[CONTROL2_REG_ADDR];
	dev->regMap.Control3.reg = rxBuffer[CONTROL3_REG_ADDR];
	dev->regMap.Control4.reg = rxBuffer[CONTROL4_REG_ADDR];
	dev->regMap.Control5.reg = rxBuffer[CONTROL5_REG_ADDR];
	dev->regMap.Control7.reg = rxBuffer[CONTROL7_REG_ADDR];
	dev->regMap.Control8.reg = rxBuffer[CONTROL8_REG_ADDR];
	dev->regMap.LCH_ATT.reg = rxBuffer[LCH_ATT_REG_ADDR];
	dev->regMap.RCH_ATT.reg = rxBuffer[RCH_ATT_REG_ADDR];
	dev->regMap.DSD1.reg = rxBuffer[DSD1_REG_ADDR];
	dev->regMap.DSD2.reg = rxBuffer[DSD2_REG_ADDR];
	dev->regMap.SoundControl.reg = rxBuffer[SOUNDCONTROL_REG_ADDR];
	dev->regMap.DFS_Read.reg = rxBuffer[DFS_READ_REG_ADDR];
	return status;
}

/**
 * @fn HAL_StatusTypeDef AK4497_Configure(dev_s*)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @return
 */
HAL_StatusTypeDef AK4497_Configure(dev_s *dev) {

	uint8_t txBuffer[12] = { 0 };
	HAL_StatusTypeDef status;
	txBuffer[CONTROL1_REG_ADDR] = dev->regMap.Control1.reg;
	txBuffer[CONTROL2_REG_ADDR] = dev->regMap.Control2.reg;
	txBuffer[CONTROL3_REG_ADDR] = dev->regMap.Control3.reg;
	txBuffer[CONTROL4_REG_ADDR] = dev->regMap.Control4.reg;
	txBuffer[CONTROL5_REG_ADDR] = dev->regMap.Control5.reg;
	txBuffer[CONTROL7_REG_ADDR] = dev->regMap.Control7.reg;
	txBuffer[CONTROL8_REG_ADDR] = dev->regMap.Control8.reg;
	txBuffer[LCH_ATT_REG_ADDR] = dev->regMap.LCH_ATT.reg;
	txBuffer[RCH_ATT_REG_ADDR] = dev->regMap.RCH_ATT.reg;
	txBuffer[DSD1_REG_ADDR] = dev->regMap.DSD1.reg;
	txBuffer[DSD2_REG_ADDR] = dev->regMap.DSD2.reg;
	txBuffer[SOUNDCONTROL_REG_ADDR] = dev->regMap.SoundControl.reg;

#ifdef USE_FREERTOS

#elif USE_THREADX
	tx_mutex_get(dev->mutex, TX_WAIT_FOREVER);
#endif
	status = HAL_I2C_Mem_Write(dev->i2c_port, dev->DevAddr, CONTROL1_REG_ADDR,
			I2C_MEMADD_SIZE_8BIT, txBuffer, sizeof(txBuffer), 100);
#ifdef USE_FREERTOS

#elif USE_THREADX
	tx_mutex_put(dev->mutex);
#endif
	return status;
}

/**
 * @fn HAL_StatusTypeDef AK4497_Write_Register(dev_s*, uint8_t, uint8_t*)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @param regAddr
 * @param value
 * @return
 */
HAL_StatusTypeDef AK4497_Write_Register(dev_s *dev, uint8_t regAddr,
		uint8_t *value) {

	HAL_StatusTypeDef status;
#ifdef USE_FREERTOS

#elif USE_THREADX
	tx_mutex_get(dev->mutex, TX_WAIT_FOREVER);
#endif
	status = HAL_I2C_Mem_Write(dev->i2c_port, dev->DevAddr, regAddr,
			I2C_MEMADD_SIZE_8BIT, value, 1, 100);
#ifdef USE_FREERTOS

#elif USE_THREADX
	tx_mutex_put(dev->mutex);
#endif
	return status;
}

/**
 * @fn HAL_StatusTypeDef AK4497_Read_Register(dev_s*, uint8_t, uint8_t*)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @param regAddr
 * @param pRegister
 * @return
 */
HAL_StatusTypeDef AK4497_Read_Register(dev_s *dev, uint8_t regAddr, uint8_t *pRegister){
	HAL_StatusTypeDef status;
#ifdef USE_FREERTOS

#elif USE_THREADX
	tx_mutex_get(dev->mutex, TX_WAIT_FOREVER);
#endif
	status = HAL_I2C_Mem_Read(dev->i2c_port, dev->DevAddr, regAddr,
			I2C_MEMADD_SIZE_8BIT, pRegister, 1, 100);
#ifdef USE_FREERTOS

#elif USE_THREADX
	tx_mutex_put(dev->mutex);
#endif
	return status;
}
/**
 * @fn HAL_StatusTypeDef AK4497_DSD_Stream_Select(dev_s*, uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @param stream
 * @return
 */
void AK4497_DSD_Stream_Select(dev_s *dev, uint8_t stream){
	switch(stream){
	case 0:
		dev->regMap.DSD1.bits.DSDSEL0 = 0;
		dev->regMap.DSD2.bits.DSDSEL1 = 0;
		break;
	case 1:
		dev->regMap.DSD1.bits.DSDSEL0 = 1;
		dev->regMap.DSD2.bits.DSDSEL1 = 0;
		break;
	case 2:
		dev->regMap.DSD1.bits.DSDSEL0 = 0;
		dev->regMap.DSD2.bits.DSDSEL1 = 1;
		break;
	case 3:
		dev->regMap.DSD1.bits.DSDSEL0 = 1;
		dev->regMap.DSD2.bits.DSDSEL1 = 1;
		break;
	}
	AK4497_Write_Register(dev,DSD1_REG_ADDR, &dev->regMap.DSD1.reg);
	AK4497_Write_Register(dev,DSD1_REG_ADDR, &dev->regMap.DSD2.reg);
}

/**
 * @fn void AK4497_PCM_DSD_Mode(dev_s*, uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @param mode
 */
void AK4497_PCM_DSD_Mode(dev_s *dev, Mode mode){
	dev->regMap.Control3.bits.DP = mode;
	AK4497_Write_Register(dev, CONTROL3_REG_ADDR, &dev->regMap.Control3.reg);
}

/**
 * @fn Mode AK4497_is_DSD_Mode(dev_s*)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @return 0 - PCM mode, 1 - DSD mode
 */
Mode AK4497_is_DSD_Mode(dev_s *dev){
	AK4497_Read_Register(dev, CONTROL3_REG_ADDR, &dev->regMap.Control3.reg);
	return dev->regMap.Control3.bits.DP;
}

/**
 * @fn void AK4497_PCM_Filter_Select(dev_s*, PCM_Filters)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @param filter
 */
void AK4497_PCM_Filter_Select(dev_s *dev, PCM_Filters filter){
	switch(filter){
	case SHARP_ROLL_OFF:
		dev->regMap.Control2.bits.SD = 0;
		dev->regMap.Control3.bits.SLOW = 0;
		dev->regMap.Control4.bits.SSLOW = 0;
		break;
	case SLOW_ROLL_OFF:
		dev->regMap.Control2.bits.SD = 0;
		dev->regMap.Control3.bits.SLOW = 1;
		dev->regMap.Control4.bits.SSLOW = 0;
		break;
	case SHORT_DELAY_SHARP:
		dev->regMap.Control2.bits.SD = 1;
		dev->regMap.Control3.bits.SLOW = 0;
		dev->regMap.Control4.bits.SSLOW = 0;
		break;
	case SHORT_DELAY_SLOW:
		dev->regMap.Control2.bits.SD = 1;
		dev->regMap.Control3.bits.SLOW = 1;
		dev->regMap.Control4.bits.SSLOW = 0;
		break;
	case SUPER_SLOW:
		dev->regMap.Control2.bits.SD = 0;
		dev->regMap.Control3.bits.SLOW = 0;
		dev->regMap.Control4.bits.SSLOW = 1;
		break;
	case LOW_DISPERSION_SHORT_DELAY:
		dev->regMap.Control2.bits.SD = 1;
		dev->regMap.Control3.bits.SLOW = 0;
		dev->regMap.Control4.bits.SSLOW = 1;
		break;
	}
	AK4497_Write_Register(dev, CONTROL2_REG_ADDR, &dev->regMap.Control2.reg);
	AK4497_Write_Register(dev, CONTROL3_REG_ADDR, &dev->regMap.Control3.reg);
	AK4497_Write_Register(dev, CONTROL4_REG_ADDR, &dev->regMap.Control4.reg);
}

/**
 * @fn PCM_Filters AK4497_Get_PCM_Filter(dev_s*)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @return
 */
PCM_Filters AK4497_Get_PCM_Filter(dev_s *dev){
	PCM_Filters filter;
	AK4497_Read_Register(dev, CONTROL2_REG_ADDR, &dev->regMap.Control2.reg);
	AK4497_Read_Register(dev, CONTROL3_REG_ADDR, &dev->regMap.Control3.reg);
	AK4497_Read_Register(dev, CONTROL4_REG_ADDR, &dev->regMap.Control4.reg);
	filter = (dev->regMap.Control4.bits.SSLOW << 2) |
			 (dev->regMap.Control2.bits.SD << 1) |
			  dev->regMap.Control3.bits.SLOW;
	return filter;
}

/**
 * @fn void AK4497_Set_Audio_Format(dev_s*, Audio_Interface_Format_e, Audio_Interface_Mode_e)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @param format
 * @param mode
 */
void AK4497_Set_Audio_Format(dev_s *dev, Audio_Interface_Format_e format,  Audio_Interface_Mode_e mode  ){

	dev->regMap.Control1.bits.DIF = format;
	dev->regMap.Control7.bits.TDM = mode;
	AK4497_Write_Register(dev, CONTROL1_REG_ADDR, &dev->regMap.Control1.reg);
	AK4497_Write_Register(dev, CONTROL7_REG_ADDR, &dev->regMap.Control7.reg);
}

/**
 * @fn void AK4497_Set_Output_Volume(dev_s*, uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @param volume
 */
void AK4497_Set_Output_Volume(dev_s *dev, uint8_t volume){
	dev->regMap.LCH_ATT.reg = volume;
	dev->regMap.RCH_ATT.reg = volume;
	AK4497_Write_Register(dev, RCH_ATT_REG_ADDR, &dev->regMap.RCH_ATT.reg);
	AK4497_Write_Register(dev, LCH_ATT_REG_ADDR, &dev->regMap.LCH_ATT.reg);
}

/**
 * @fn void AK4497_Set_Gain_Adjustment(dev_s*, Gain_Adjustment_e)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @param gain
 */
void AK4497_Set_Gain_Adjustment(dev_s *dev, Gain_Adjustment_e gain){
	if(gain == 3 || gain > 5) {
		gain = PCM_2_5vpp_DSD_2_5vpp;
	}
	dev->regMap.Control5.bits.GC = gain;
	AK4497_Write_Register(dev, CONTROL5_REG_ADDR, &dev->regMap.Control5.reg);
}

/**
 * @fn void AK4497_Set_MONO_Mode(dev_s*, Mono_mode_e)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @param mode
 */
void AK4497_Set_MONO_Mode(dev_s *dev, Mono_mode_e mode){
	dev->regMap.Control3.bits.MONO = 1;
	dev->regMap.Control3.bits.SELLR = mode;
	AK4497_Write_Register(dev, CONTROL3_REG_ADDR, &dev->regMap.Control3.reg);
}

/**
 * @fn void AK4497_DSD_InputData_Pins_Select(dev_s*, Input_Data_Pins_e)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @param pins
 */
void AK4497_DSD_InputData_Pins_Select(dev_s *dev, Input_Data_Pins_e pins){
	dev->regMap.DSD2.bits.DSDPATH = pins;
	AK4497_Write_Register(dev, DSD2_REG_ADDR, &dev->regMap.DSD2.reg);
}

/**
 * @fn void AK4497_SoftMute(dev_s*, DAC_mute_e)
 * @brief
 *
 * @pre
 * @post
 * @param dev
 * @param mute
 */
void AK4497_SoftMute(dev_s *dev, DAC_mute_e mute){
	dev->regMap.Control2.bits.SMUTE = mute;
	AK4497_Write_Register(dev, CONTROL2_REG_ADDR, &dev->regMap.Control2.reg);
}
