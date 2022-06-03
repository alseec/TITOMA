#include "Command_Parser.h"

#include "Uart_Driver.h"
#include "main.h"

#include <string.h>

extern uart_driver_t uart_driver;

const uint8_t preamble = PROTOCOL_PREAMBLE;
const uint8_t posamble = PROTOCOL_POSAMBLE;

const uint8_t turn_led_on_cmd[] = "TURN_LED_ON";
const uint8_t turn_led_off_cmd[] = "TURN_LED_OFF";
const uint8_t turn_led_period1_cmd[] = "LED_SET_PERIOD_1";
const uint8_t turn_led_period2_cmd[] = "LED_SET_PERIOD_2";
const uint8_t turn_led_period3_cmd[] = "LED_SET_PERIOD_3";

const uint8_t read_temperature_cmd[] = 	"T";
const uint8_t read_pwm_fan_cmd[] =		"F";
const uint8_t get_door_state_cmd[] = 	"D";
const uint8_t get_FWversion_cmd[] = 	"V";
const uint8_t get_tick_cmd[] = 			"C";
const uint8_t get_heater_state_cmd[] = 	"H";

const uint8_t ack_control_cmd[] = 		"C";



const uint8_t ack_message[] = "*ACK-VALID#";
const uint8_t nack_message[] = "*NACK#";


void parse_command(uint8_t *rx_packet)
{
	if (memcmp(rx_packet, read_temperature_cmd, sizeof(read_temperature_cmd) - 1) == 0){
		//Send the last sample of temperature to the host
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, read_pwm_fan_cmd, sizeof(read_pwm_fan_cmd) - 1) == 0){
		//Calculates the percentage of the fan based on the duty cycle and sends it to the host
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, get_door_state_cmd, sizeof(get_door_state_cmd) - 1) == 0){
		//Sends the state of the doors (Opened/Closed) to the host
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, get_FWversion_cmd, sizeof(get_FWversion_cmd) - 1) == 0){
		//Sends the FW version (Updated every time a release is made) to the host.
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, get_tick_cmd, sizeof(get_tick_cmd) - 1) == 0){
		//Sends the total time the unit has been running.
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, get_heater_state_cmd, sizeof(get_heater_state_cmd) - 1) == 0){
		//Send the state of the heater, indicating if it's turned on or off.
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

	}else if (memcmp(rx_packet, ack_control_cmd, sizeof(ack_control_cmd) - 1) == 0){
		//Changes the duty cyle of the PWM to different values: 0%, 25%, 50%, 75%, 100%
		//Set the state of the door to closed
		HAL_GPIO_WritePin(DOOR_GPIO_Port, DOOR_Pin, GPIO_PIN_RESET);
		//Set the state of the door to opened
		HAL_GPIO_WritePin(DOOR_GPIO_Port, DOOR_Pin, GPIO_PIN_SET);
		uart_driver_send(&uart_driver, (uint8_t *)ack_message, sizeof(ack_message) - 1);

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
