#ifndef TWI_INTERFACE_H_
#define TWI_INTERFACE_H_


#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "sdk_common.h"
#include "nrf_drv_common.h"
#include "app_util_platform.h"


/* The following functions must be defined for this platform:
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
 * reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin)
 * labs(long x)
 * fabsf(float x)
 * min(int a, int b)
 */

void twi_interface_init (void);
int twi_scan (void);


int nrf_twi_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int nrf_twi_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);


int twi_write_read(uint8_t address, uint8_t command);
int twi_write_write(uint8_t address, uint8_t command, uint8_t data);

#endif
