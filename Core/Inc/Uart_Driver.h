#ifndef __UART_DRIVER_INC_
#define __UART_DRIVER_INC_

#include "ring_buffer.h"
#include "main.h"

#define UART_RX_BUFF_LEN	(32)
#define UART_TX_BUFF_LEN	(64)

typedef struct uart_driver_ {

	UART_HandleTypeDef *huart;

	ring_buffer_t rx_buffer;
	ring_buffer_t tx_buffer;

	uint8_t rx[UART_RX_BUFF_LEN];
	uint8_t tx[UART_TX_BUFF_LEN];


	uint8_t rx_byte;
	uint8_t tx_byte;

	uint32_t rx_tick;

} uart_driver_t;

void uart_driver_init(uart_driver_t*, UART_HandleTypeDef *);
void uart_driver_run(uart_driver_t *);
void uart_driver_send(uart_driver_t *, uint8_t *, size_t);


#endif /* __UART_DRIVER_INC_*/
