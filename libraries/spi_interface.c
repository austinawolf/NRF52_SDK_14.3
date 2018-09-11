
#include "spi_interface.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_gpio.h"


#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
   

void spi_init(void) {
		
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = NRF_GPIO_PIN_MAP(0,31);
    spi_config.miso_pin = NRF_GPIO_PIN_MAP(0,2);
    spi_config.mosi_pin = NRF_GPIO_PIN_MAP(0,28);
    spi_config.sck_pin  = NRF_GPIO_PIN_MAP(0,29);
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}

void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void *p_context)
{
	spi_xfer_done = true;
    //NRF_LOG_INFO("Transfer completed.");
}

void spi_transfer(uint8_t* m_tx_buf, uint8_t tx_length, uint8_t* m_rx_buf, uint8_t rx_length) {
	
	//setup temp buffer
	uint8_t m_rx_buf_temp[rx_length+1];
	memset(m_rx_buf_temp, 0, rx_length+1);
	
	//start transfer
	spi_xfer_done = false;
	
	if (m_rx_buf != 0x0) {
		nrf_drv_spi_transfer(&spi, m_tx_buf, tx_length, m_rx_buf_temp, rx_length+1);
	}
	else {
		nrf_drv_spi_transfer(&spi, m_tx_buf, tx_length, 0, 0);
	}
	
	//wait
	while (!spi_xfer_done)
	{
			__WFE();
	}
	
	//copy temp buffer to original buffer
	if (m_rx_buf != 0x0) {
		memcpy(m_rx_buf, &m_rx_buf_temp[1], rx_length);
	}	
	
	
}
