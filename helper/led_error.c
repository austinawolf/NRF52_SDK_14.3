#include "boards.h"
#include "led_error.h"
#include "nrf_delay.h"
#include "FreeRTOS.h"
#include "task.h"

#define FAST_BLINK_MS 200
#define SLOW_BLINK_MS 500


void fast_blink(uint8_t led_idx);
void slow_blink(uint8_t led_idx);


void fast_blink(uint8_t led_idx) {
	bsp_board_led_on(led_idx);
	nrf_delay_ms(FAST_BLINK_MS);
	bsp_board_led_off(led_idx);
	nrf_delay_ms(FAST_BLINK_MS);
}

void slow_blink(uint8_t led_idx) {
	bsp_board_led_on(led_idx);
	nrf_delay_ms(SLOW_BLINK_MS);
	bsp_board_led_off(led_idx);
	nrf_delay_ms(SLOW_BLINK_MS);
}

void critical_error(uint8_t led_index, uint8_t error_code) {
	taskENTER_CRITICAL();
	while(1) {
		alert(led_index, error_code);
	}
	//taskEXIT_CRITICAL();
}

void alert(uint8_t led_index, uint8_t error_code) {
		for (int i = 0; i < error_code; i++) fast_blink(led_index);
		nrf_delay_ms(SLOW_BLINK_MS);
}

void led_on(uint8_t led_index) {
	bsp_board_led_on(led_index);
}


void led_off(uint8_t led_index) {
	bsp_board_led_off(led_index);
}
