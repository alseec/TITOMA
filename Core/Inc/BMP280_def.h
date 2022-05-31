/*
 * BMP280_def.h
 *
 *  Created on: 28/05/2022
 *      Author: Usuario
 */

#include "main.h"



#ifndef BMP280_DEF_H_
#define BMP280_DEF_H_

#define BMP280_ID						(0x58)
//--------------------------//
#define BMP280_SPI_WRITING_MASK			(0x7F) // = 0b01111111
#define BMP280_SPI_READING_MASK			(0xFF) // = 0b11111111
//--------------------------//
#define FILTER_MASK						0b11100011


extern SPI_HandleTypeDef hspi1;

/*/
SPI_HandleTypeDef *hspi1;

UART_HandleTypeDef *huart1;
/*/
//Register Address default
typedef enum BMP280_Register_Address{
	BMP280_REG_ADDR_ERROR			 = 0x00,
	BMP280_REG_ADDR_CAL_START_ADDR	 = 0x88,
	BMP280_REG_ADDR_ID				 = 0xD0,
	BMP280_REG_ADDR_RESET			 = 0xE0,
	BMP280_REG_ADDR_STATUS			 = 0xF3,
	BMP280_REG_ADDR_CTRL_MEAS		 = 0xF4,
	BMP280_REG_ADDR_CONFIG			 = 0xF5,
	BMP280_REG_ADDR_PRESS_MSB	 	 = 0xF7,
	BMP280_REG_ADDR_PRESS_LSB		 = 0xF8,
	BMP280_REG_ADDR_PRESS_XLSB		 = 0xF9,
	BMP280_REG_ADDR_TEMP_MSB		 = 0xFA,
	BMP280_REG_ADDR_TEMP_LSB		 = 0xFB,
	BMP280_REG_ADDR_TEMP_XLSB		 = 0xFC,
	BMP280_REG_ADDR_RESET_V			 = 0xB6,
	BMP280_REG_ADDR_RESET_MEAS		 = 0x80,
}BMP280_Register_Address;

//Registers used by sensor

typedef struct BMP280_Register_Info{
	uint16_t 	dig_T1;
	int16_t		dig_T2;
	int16_t 	dig_T3;
	uint16_t 	dig_P1;
	int16_t 	dig_P2;
	int16_t 	dig_P3;
	int16_t 	dig_P4;
	int16_t 	dig_P5;
	int16_t 	dig_P6;
	int16_t 	dig_P7;
	int16_t 	dig_P8;
	int16_t 	dig_P9;
}BMP280_Register_Info;


//Power modes
typedef enum BMP280_Power_Mode{
	Sleep_Mode 	= 0b00,
	Forced_Mode = 0b01,
	Normal_Mode = 0b11,
	//0xF4 [1:0]
}BMP280_Power_Mode;

//Filters
typedef enum BMP280_IIR_Filter {
	Filter_off 		= 0b000,
	Filter_coeff_2 	= 0b001,
	Filter_coeff_4 	= 0b010,
	Filter_coeff_8 	= 0b011,
	Filter_coeff_16 = 0b100,
	//0xF5 [2:0]
}BMP280_IIR_Filter;

//Oversampling meas
typedef enum BMP280_OverS{
	Oversampling_WO		= 0b000,
	Oversamplingx1 		= 0b001,
	Oversamplingx2 		= 0b010,
	Oversamplingx4 		= 0b011,
	Oversamplingx8 		= 0b100,
	Oversamplingx16 	= 0b101,
}BMP280_OverS;

//Time between measures
typedef enum BMP280_StandbyTime{
	standby_time_500us = 0b000,
	standby_time_62500us = 0b001,
	standby_time_125ms = 0b010,
	standby_time_250ms = 0b011,
	standby_time_500ms = 0b100,
	standby_time_1000ms = 0b101,
	standby_time_2000ms = 0b110,
	standby_time_4000ms = 0b111
}BMP280_StandbyTime;

//Read-Write
void CS_Config_Low();
void CS_Config_High();
uint8_t SPI_Read_Write(uint8_t spi_tx_data);
uint8_t Read_Register (uint8_t Register_Address);
void Write_Register(uint8_t Register_Address, uint8_t value);


//Config Sensor
void Power_Mode(BMP280_Power_Mode mode);
void Filter_IIR(BMP280_IIR_Filter data_f);
void Temperature_OS(BMP280_OverS osrs_t);
void StandbyTime(BMP280_StandbyTime t_sb);

//ID and Reset
uint8_t getID();
void Reset();

//Config measure temperature from sensor
float BMP280_Convert_Formula (double mea_temp);
void Measure();

//Initialize the sensor with all config below
uint8_t Init_Sensor();

#endif /* BMP280_DEF_H_ */
