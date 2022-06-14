#include "uart_driver.h"

#include "Command_Parser.h"

#include <string.h>

#define RECEIVED_PACKET_MAX_LEN		(128)

#define RX_PACKET_MAX_TIMEOUT_MS	(50)

extern uart_driver_t uart_driver;

typedef enum rx_packet_state{
	RX_PACKET_PREAMBLE = 0x00,
	RX_PACKET_DATA,
}rx_packet_state_t;

struct {
	uint8_t  buffer[RECEIVED_PACKET_MAX_LEN];
	size_t size;

	rx_packet_state_t state;
}received_packet;

uint8_t uart_driver_get_rx_len(uart_driver_t *uart_driver)
{
	return (ring_buffer_size(&uart_driver->rx_buffer));
}

uint8_t uart_driver_get_rx_data(uart_driver_t *uart_driver, uint8_t *p_data)
{
	return (ring_buffer_get(&uart_driver->rx_buffer, p_data));
}

void uart_driver_store_rx_data(uart_driver_t *uart_driver, uint8_t data)
{
	ring_buffer_put(&uart_driver->rx_buffer, data);
}

void uart_driver_send(uart_driver_t *uart_driver, uint8_t *data, size_t len)
{
	//Append data to tx buffer
	for (size_t byte = 0; byte < len; byte++) {
		ring_buffer_put(&uart_driver->tx_buffer, data[byte]);
	}

	if (!ring_buffer_empty(&uart_driver->tx_buffer)) {
		ring_buffer_get(&uart_driver->tx_buffer, &uart_driver->tx_byte);
		HAL_UART_Transmit_IT(uart_driver->huart, &uart_driver->tx_byte, 1);
	}

}

void uart_driver_init(uart_driver_t *uart_driver,  UART_HandleTypeDef *huart)
{
	memset(uart_driver, 0, sizeof(uart_driver_t));

	uart_driver->huart = huart;

	ring_buffer_init(&uart_driver->rx_buffer, uart_driver->rx, UART_RX_BUFF_LEN);
	ring_buffer_init(&uart_driver->tx_buffer, uart_driver->tx, UART_TX_BUFF_LEN);

	HAL_UART_Receive_IT(uart_driver->huart, &uart_driver->rx_byte, 1);
}

void uart_driver_run(uart_driver_t *uart_driver)
{
	size_t rx_buff_len = ring_buffer_size(&uart_driver->rx_buffer);
	uint8_t data = 0;


	if (received_packet.state == RX_PACKET_DATA){
		if ((HAL_GetTick() - uart_driver->rx_tick) > RX_PACKET_MAX_TIMEOUT_MS){
			memset(received_packet.buffer, 0, RECEIVED_PACKET_MAX_LEN);
			received_packet.size = 0;
			received_packet.state = RX_PACKET_PREAMBLE;
		}
	}

	while (rx_buff_len > 0){
		switch (received_packet.state){
		case RX_PACKET_PREAMBLE:
			ring_buffer_get(&uart_driver->rx_buffer, &data);
			if(data == PROTOCOL_PREAMBLE){
				received_packet.state = RX_PACKET_DATA;
			}
			break;
		case RX_PACKET_DATA:
			ring_buffer_get(&uart_driver->rx_buffer, &data);
			if(data == PROTOCOL_POSAMBLE){
				received_packet.state = RX_PACKET_PREAMBLE;
				parse_command(received_packet.buffer);

				memset(received_packet.buffer, 0, RECEIVED_PACKET_MAX_LEN);
				received_packet.size = 0;
			}else{
				received_packet.buffer[received_packet.size++] = data;
			}
			break;
		default:
			break;
		}
		rx_buff_len--;
	}
}

//HAL_UART_TxCpltCallback(huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		if (ring_buffer_empty(&uart_driver.tx_buffer) == 0) {
			ring_buffer_get(&uart_driver.tx_buffer, &uart_driver.tx_byte);
			HAL_UART_Transmit_IT(uart_driver.huart, &uart_driver.tx_byte, 1);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1){
		uart_driver_store_rx_data(&uart_driver, uart_driver.rx_byte);
		HAL_UART_Receive_IT(uart_driver.huart, &uart_driver.rx_byte, 1);

		uart_driver.rx_tick = HAL_GetTick();
	}
}
