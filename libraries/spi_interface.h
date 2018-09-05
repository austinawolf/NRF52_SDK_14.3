#ifndef SPI_INTERFACE_H_
#define SPI_INTERFACE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "sdk_common.h"
#include "nrf_drv_common.h"
#include "nrf_drv_spi.h"


void spi_init(void);
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void *p_context);
void spi_transfer(uint8_t* m_tx_buf, uint8_t tx_length, uint8_t* m_rx_buf, uint8_t rx_length);

#endif
