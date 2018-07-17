#ifndef UART_HELPER_H_
#define UART_HELPER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "app_uart.h"

#define UART_TX_BUF_SIZE 1024
#define UART_RX_BUF_SIZE 1024
#define UART_MAX_DATA_LEN (32)


//void uart_event_handle(app_uart_evt_t* p_event);
void uart_init(void);

#endif
