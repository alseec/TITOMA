/*
 * host_comm_manager.h
 *
 *  Created on: Nov 10, 2020
 *      Author: JFG
 */

#ifndef __HOST_COMM_MANAGER__
#define __HOST_COMM_MANAGER__

#include <stdint.h>
#include "main.h"

#define RX_BUFFER_LEN 16

typedef struct host_comm_mcu_intf_ {
	UART_HandleTypeDef *huart;

} host_comm_mcu_intf_t;

typedef struct host_comm_manager_ {
	host_comm_mcu_intf_t *mcu_intf;

	uint8_t  rx_buffer[RX_BUFFER_LEN];
	uint8_t  rx_unread_flag;
	uint8_t  rx_data;
	uint8_t  rx_head;

	uint8_t  command;
	uint16_t value;

} host_comm_manager_t;

uint8_t host_comm_manager_store_rx_data(host_comm_manager_t *);
void host_comm_manager_send_buffer(host_comm_manager_t *, uint8_t *, uint16_t);
void host_comm_rx_callback(host_comm_manager_t *host_comm);
void host_comm_manager_init(host_comm_manager_t *, const host_comm_mcu_intf_t *);
void host_comm_manager_run(host_comm_manager_t *);

#endif
