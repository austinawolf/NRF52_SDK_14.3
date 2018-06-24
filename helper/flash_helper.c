

#include "flash_helper.h"
#include "boards.h"
#include "led_error.h"
#include "nrf_delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "esb_logger.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "nrf_log_default_backends.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_drv_spi.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */


void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context);




/**
 * @brief SPI user event handler.
 * @param event
 */
void wait_for_transfer() {
		while (!spi_xfer_done)
		{
				__WFE();
		}
}
 
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
    spi_xfer_done = true;
		esb_log_write("SPI write");

		
	
    if (p_event->data.done.p_rx_buffer[0] != 0)
    {
				esb_log_write_value("Rx:%u",p_event->data.done.p_rx_buffer[0]);

    }
		else critical_error(BSP_BOARD_LED_0, SPI_NULL_RX);
}

void spi_write(uint8_t command, uint8_t* m_tx_buf, uint8_t* m_rx_buf) {
		bsp_board_led_on(BSP_BOARD_LED_0);
	
		memset(m_tx_buf,0,FRAME_LEN); 	
		m_tx_buf[0] = command;

		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, sizeof(m_tx_buf), m_rx_buf, sizeof(m_rx_buf)));
		wait_for_transfer();
	
		bsp_board_led_off(BSP_BOARD_LED_0);

}




void spi_helper_init() {

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler,NULL));
	

	
		spi_xfer_done = false;

}

SPI_STATUS Serialize_SPI(const CharStream* char_stream_send,
                         CharStream* char_stream_recv,
                         SpiConfigOptions optBefore, SpiConfigOptions optAfter) {

}
