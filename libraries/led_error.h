#ifndef NRF_LED_ERROR_H_
#define NRF_LED_ERROR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "bsp.h"
#include "boards.h"

#define ESB_TRANSMISSION_ERR (uint8_t) 3
#define RESET (uint8_t) 5

#define SPI_NULL_RX 2
#define TWI_NO_RESP 4


void critical_error(uint8_t led_index, uint8_t error_code);
void alert(uint8_t led_index, uint8_t error_code);
void led_on(uint8_t led_index);
void led_off(uint8_t led_index);

#endif
