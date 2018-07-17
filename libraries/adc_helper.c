#include "adc_helper.h"



void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    //setup sampling event trigger

    APP_ERROR_CHECK(err_code);
}

int32_t get_adc_time_av(void) {
		return adc_time_av;
}


/*
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
		
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;

				int32_t adc = 0;
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
						adc += ((p_event->data.done.p_buffer[i]) / (SAMPLES_IN_BUFFER));
        }

				adc_time_av = adc;
				
				NRF_LOG_INFO("ADC event number: %d", (int) adc_evt_counter);
				xQueueSendToBack(adc_queue,&adc_evt_counter,0);
				
        adc_evt_counter++;
    }
}
*/
