/*
 * host_comm_manager.c
 *
 *  Created on: Nov 10, 2020
 *      Author: JFG
 */

#include <stdio.h>
#include <string.h>

#include "host_comm_manager.h"

#define debug_host_comm printf



/**
* @brief    This function is used to storage the received Rx buffer in host_comm structure.
* @param    host_comm_manager_t *host_comm: Pointer to the host_comm_manager_t struct. host_comm->rx_buffer will be used to storage
* 			byte by byte the buffer in host_comm->rx_buffer until find a "#". Otherwise, host_comm->rx_unread_flag won't be set with 1.
* @retval   uint8_t ret_val
*/
uint8_t host_comm_manager_store_rx_data(host_comm_manager_t *host_comm)
{
	uint8_t ret_val = 1;

	host_comm->rx_buffer[host_comm->rx_head++] = host_comm->rx_data;
	if (host_comm->rx_data == '#') {
		host_comm->rx_unread_flag = 1;
	}
	if (host_comm->rx_head > RX_BUFFER_LEN) {
		host_comm->rx_head = 0;
		ret_val = 0;
	}
	HAL_UART_Receive_IT(host_comm->mcu_intf->huart, &host_comm->rx_data, 1); // wait for the next byte
	return ret_val;
}


/**
* @brief    Function to send a buffer by UART
* @param    host_comm_manager_t *host_comm: pointer to the host_comm_manager_t struct.
*           uint8_t *tx_ptr: pointer to the buffer to be sent by UART.
*           uint16_t tx_len: buffer length.
* @retval   None
*/
void host_comm_manager_send_buffer(host_comm_manager_t *host_comm, uint8_t *tx_ptr, uint16_t tx_len)
{
	HAL_UART_Transmit(host_comm->mcu_intf->huart, tx_ptr, tx_len, 10); // send the packet in blocking mode
}


/**
* @brief    Function to handle whatever process when a message arrives.
* @param    host_comm_manager_t *host_comm: pointer to the host_comm_manager_t struct.
* @retval   None
*/
__weak void host_comm_rx_callback(host_comm_manager_t *host_comm)
{
	UNUSED(host_comm);
}


/**
* @brief    Function to set reset parameters in host_comm_manager_t struct
* @param    host_comm_manager_t *host_comm: pointer to the host_comm_manager_t struct.
* @retval   None
*/
static void reset_variables(host_comm_manager_t *host_comm)
{
	memset(host_comm->rx_buffer, 0, RX_BUFFER_LEN);
	host_comm->rx_unread_flag = 0;
	host_comm->rx_head = 0;
}


/**
* @brief    Function to initialize the host_comm_manager_t struct
* @param    host_comm_mcu_intf_t *mcu_intf: Pointer to the uart instance which sends the value.
*           host_comm_manager_t *host_comm: pointer to host_comm_manager_t struct to set parameters
* @retval   None
*/
void host_comm_manager_init(host_comm_manager_t *host_comm, const host_comm_mcu_intf_t *mcu_intf)
{
	host_comm->mcu_intf = (host_comm_mcu_intf_t *)mcu_intf;

	HAL_UART_Receive_IT(host_comm->mcu_intf->huart, &host_comm->rx_data, 1); // wait for the first byte
	reset_variables(host_comm);
}


/**
* @brief    Function to handle all the UART Rx processes.
* @param    host_comm_manager_t *host_comm: pointer to the host_comm_manager_t struct.
* @retval   None
*/
void host_comm_manager_run(host_comm_manager_t *host_comm)
{
	if (host_comm->rx_unread_flag != 0) {
		host_comm_manager_send_buffer(host_comm, host_comm->rx_buffer, host_comm->rx_head);
		debug_host_comm(" -> Received\r\n");
		host_comm_rx_callback(host_comm);
		reset_variables(host_comm);
	}
}

