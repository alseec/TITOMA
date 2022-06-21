#include "Command_Parser.h"

#include "Uart_Driver.h"
#include "main.h"
#include "BMP280_def.h"

#include <string.h>
#include <stdio.h>

#define FW_VERSION "V1.0.20220524"

extern uart_driver_t uart_driver;

extern float temperature_in_degrees, Temperature_to_show;

const uint8_t preamble = PROTOCOL_PREAMBLE;
const uint8_t posamble = PROTOCOL_POSAMBLE;

//----- START - REFERENCE -----//
/*/
const uint8_t turn_led_on_cmd[] = "TURN_LED_ON";
const uint8_t turn_led_off_cmd[] = "TURN_LED_OFF";
const uint8_t turn_led_period1_cmd[] = "LED_SET_PERIOD_1";
const uint8_t turn_led_period2_cmd[] = "LED_SET_PERIOD_2";
const uint8_t turn_led_period3_cmd[] = "LED_SET_PERIOD_3";
/*/
//----- END - REFERENCE -----//

const uint8_t read_temperature_cmd[] = 	"T";
const uint8_t read_pwm_fan_cmd[] =		"F";
const uint8_t set_pwm_fan_cmd[] =		"S";
const uint8_t set_door_close_cmd[] = 	"CD";
const uint8_t set_door_open_cmd[] = 	"OD";
const uint8_t get_door_state_cmd[] = 	"D";
const uint8_t get_FWversion_cmd[] = 	"V";
const uint8_t get_tick_cmd[] = 			"C";
const uint8_t get_heater_state_cmd[] = 	"H";

const uint8_t ack_control_cmd[] = 		"C";



const uint8_t ack_message[] = "*ACK-VALID#";
const uint8_t nack_message[] = "*NACK#";

//uint8_t prueba_message[] = "*T%f#";

const uint8_t Fw_version[] =  	"*V1.0.20220524#";
const uint8_t door_is_close[] = "*D0#";
const uint8_t door_is_open[] = 	"*D1#";

const uint8_t heater_is_off[] = "*H0#";
const uint8_t heater_is_on[] = 	"*H1#";

uint8_t fan_value = 0;
uint8_t state_door = -1;


void parse_command(uint8_t *rx_packet)
{
	if (memcmp(rx_packet, read_temperature_cmd, sizeof(read_temperature_cmd) - 1) == 0){
		//Send the last sample of temperature to the host

		//type -> *T35.29# | 2 data info + 2 data protocol + 1 Data Null -> (function sprintf)
		uint16_t last_value[5] = {0x00};

		//last_value[0] = preamble;
		//last_value[1] = "T";
		//last_value[8] = posamble;

		//printf("T : %f ", temperature_sensor);
		sprintf((uint8_t *)last_value, "%cT%.3f%c", preamble, Temperature_to_show, posamble);
		uart_driver_send(&uart_driver, (uint8_t *)last_value, sizeof(last_value) - 1);

	}else if (memcmp(rx_packet, read_pwm_fan_cmd, sizeof(read_pwm_fan_cmd) - 1) == 0){
		//Calculates the percentage of the fan based on the duty cycle and sends it to the host

		uint8_t show_state = 0;
		uint8_t fan[7] = {0x00};

		show_state = control_pwm(fan_value);

		sprintf((uint8_t *)fan, "%cF0%d%c", preamble, show_state, posamble);
		uart_driver_send(&uart_driver, (uint8_t *)fan, sizeof(fan) - 1);

	}else if (memcmp(rx_packet, set_pwm_fan_cmd, sizeof(set_pwm_fan_cmd) - 1) == 0){
		//Changes the duty cyle of the PWM to different values: 0%, 25%, 50%, 75%, 100%

		fan_value = fan_value + 25;
		control_pwm(fan_value);

		if (fan_value > 100){
			fan_value = 0;
		}
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, set_door_close_cmd, sizeof(set_door_close_cmd) - 1) == 0){
		//Set the state of the door to closed
		HAL_GPIO_WritePin(DOOR_GPIO_Port, DOOR_Pin, GPIO_PIN_RESET);
		state_door = 0;
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, set_door_open_cmd, sizeof(set_door_open_cmd) - 1) == 0){
		//Set the state of the door to opened
		HAL_GPIO_WritePin(DOOR_GPIO_Port, DOOR_Pin, GPIO_PIN_SET);
		state_door = 0;

		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, get_door_state_cmd, sizeof(get_door_state_cmd) - 1) == 0){
		//Sends the state of the doors (Opened/Closed) to the host


		//uint8_t door_pos[3] = {0x00};

		if (state_door == 0){
			//sprintf((uint8_t *)door_pos, "%cD0%c", preamble, posamble);
			uart_driver_send(&uart_driver, (uint8_t *)door_is_close, sizeof(door_is_close) - 1);
		}
		if (state_door == 1){
			//sprintf((uint8_t *)door_pos, "%cD1%c", preamble, posamble);
			uart_driver_send(&uart_driver, (uint8_t *)door_is_open, sizeof(door_is_open) - 1);
		}


	}else if (memcmp(rx_packet, get_FWversion_cmd, sizeof(get_FWversion_cmd) - 1) == 0){
		//Sends the FW version (Updated every time a release is made) to the host.
		//printf("*%#", FW_VERSION);
		//uint16_t fw_version[8] = {0x00};

		//sprintf((uint8_t *)fw_version, "%cV%f%c", preamble, FW_VERSION, posamble);
		uart_driver_send(&uart_driver, (uint8_t *)Fw_version, sizeof(Fw_version) - 1);

	}else if (memcmp(rx_packet, get_tick_cmd, sizeof(get_tick_cmd) - 1) == 0){
		//Sends the total time the unit has been running.
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);



	}else if (memcmp(rx_packet, get_heater_state_cmd, sizeof(get_heater_state_cmd) - 1) == 0){
		//Send the state of the heater, indicating if it's turned on or off.

		if (HAL_GPIO_ReadPin(HEATER_GPIO_Port, HEATER_Pin) == 0){
			uart_driver_send(&uart_driver, (uint8_t *)heater_is_off, sizeof(heater_is_off) - 1);
		}

		if (HAL_GPIO_ReadPin(HEATER_GPIO_Port, HEATER_Pin) == 1){
			uart_driver_send(&uart_driver, (uint8_t *)heater_is_on, sizeof(heater_is_on) - 1);
		}

	}else{
		uart_driver_send(&uart_driver, (uint8_t *)nack_message, sizeof(nack_message) - 1);
	}


	/*/--------START - REFERENCE------------//
	if (memcmp(rx_packet, turn_led_on_cmd, sizeof(turn_led_on_cmd) - 1) == 0){
		//Turn Green Led ON
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, turn_led_off_cmd, sizeof(turn_led_off_cmd) - 1) == 0){
		//Turn Green Led OFF
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, turn_led_period1_cmd, sizeof(turn_led_period1_cmd) - 1) == 0){
		//Blink with period 1
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, turn_led_period2_cmd, sizeof(turn_led_period2_cmd) - 1) == 0){
		//Blink with period 2
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, turn_led_period3_cmd, sizeof(turn_led_period3_cmd) - 1) == 0){
		//Blink with period 3
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else{
		uart_driver_send(&uart_driver, (uint8_t *)nack_message, sizeof(nack_message) - 1);
	}
	//--------END - REFERENCE------------/*/
}
