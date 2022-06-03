/*
 * BMP280_func.c
 *
 *  Created on: 28/05/2022
 *      Author: Usuario
 */
#include "main.h"
/*/
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_spi.h"
#include "BMP280_def.h"


//---------------START - CS Configurations----------------------//

void CS_Config_Low() {

		HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_RESET);
}


void CS_Config_High() {
	HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_SET);
}
//---------------END - CS Configurations----------------------//

//---------------START - READ/WRITE FUNCTIONS----------------------//
uint8_t SPI_Read_Write(uint8_t spi_tx_data)
{
	uint8_t spi_rx_data = 255;
	if (HAL_SPI_Transmit(&hspi1, &spi_tx_data, 1, 100) == HAL_OK){
		if (HAL_SPI_Receive(&hspi1, &spi_rx_data, 1, 100) == HAL_OK){
			return spi_rx_data;
		}
	}
	return 0;
}


uint8_t Read_Register (uint8_t Register_Address) {

	CS_Config_Low();
	SPI_Read_Write(Register_Address);
	uint8_t value = SPI_Read_Write(0);
	CS_Config_High();
	return value;
}


void Write_Register(uint8_t Register_Address, uint8_t value)
{
	CS_Config_Low();
	SPI_Read_Write(Register_Address & BMP280_SPI_WRITING_MASK);
	SPI_Read_Write(value);
	CS_Config_High();
}
//---------------END - READ/WRITE FUNCTIONS----------------------//

//---------------START - CONFIG SENSOR FUNCTIONS----------------------//
void Filter_IIR(BMP280_IIR_Filter data_f){

	uint8_t conf = Read_Register(BMP280_REG_ADDR_CONFIG);
	conf = (conf & 0b11100011) | (data_f << 2); // off (000)
	Write_Register(BMP280_REG_ADDR_CONFIG, conf);
}

void Power_Mode(BMP280_Power_Mode mode)
{
	uint8_t value = Read_Register(BMP280_REG_ADDR_CTRL_MEAS);
	value = (value & 0b11111100) | mode; //Sleep_mode (00)
	Write_Register(BMP280_REG_ADDR_CTRL_MEAS, value);
}

void Temperature_OS(BMP280_OverS osrs_t)
{
	uint8_t conf = Read_Register(BMP280_REG_ADDR_CTRL_MEAS);
	conf = (conf & 0b00011111) | (osrs_t << 5);
	Write_Register(BMP280_REG_ADDR_CTRL_MEAS, conf);
}

void StandbyTime(BMP280_StandbyTime t_sb)
{
	uint8_t conf = Read_Register(BMP280_REG_ADDR_CONFIG);
	conf = (conf & 0b00011111) | (t_sb << 5);
	Write_Register(BMP280_REG_ADDR_CONFIG, conf);
}
//---------------END - CONFIG SENSOR FUNCTIONS----------------------//

//---------------START - SENSOR COMMANDS----------------------//
uint8_t getID()
{
	return Read_Register(BMP280_REG_ADDR_ID);
}

void Reset()
{
	Write_Register(BMP280_REG_ADDR_CAL_START_ADDR, BMP280_REG_ADDR_RESET_V);
}
//---------------START - SENSOR COMMANDS----------------------//


//---------------START - TEMPERATURE CONFIG----------------------//
float BMP280_Convert_Formula (double mea_temp){

	uint16_t dig_T1 = 27504;
	int16_t dig_T2 = 26435;
	int16_t dig_T3 = -1000;


	double var1 = 0, var2 = 0;
	float T = 0;

	var1 = (((double)mea_temp) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
	var2 = ((((double)mea_temp) / 131072.0 - ((double)dig_T1) / 8192.0) * (((double)mea_temp)/131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
	T = (float)(var1 + var2) / 5120.0;
	return T;
}


void Measure()
{
	float temperature_in_degrees = 0;
	uint8_t tx_data = BMP280_REG_ADDR_TEMP_MSB;
	uint8_t rx_data[3] = {0x00};


	HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&hspi1, &tx_data, 1, 100)==HAL_OK){
		if (HAL_SPI_Receive(&hspi1, rx_data, 3, 100)== HAL_OK){

			int32_t adc_T = (int32_t)(rx_data[2] & 0b00001111) | (rx_data[1] << 4) | (rx_data[0] << 12);
			temperature_in_degrees = (float)(BMP280_Convert_Formula(adc_T));
		}
	}
	HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_SET);
}

//---------------END - TEMPERATURE CONFIG----------------------//

//---------------START - PRINCIPAL INIT----------------------//
uint8_t Init_Sensor()
{
	if (getID() != BMP280_ID)
	{
		return 1;
	}
	Reset();

	//------ Start Config sensor ----//
	Temperature_OS(Oversamplingx2);

	Power_Mode(Normal_Mode);
	Filter_IIR(Filter_coeff_16);
	StandbyTime(standby_time_1000ms);
	//----- End Config sensor-----//

	return 0;
}
//---------------END - PRINCIPAL INIT----------------------//
/*/
