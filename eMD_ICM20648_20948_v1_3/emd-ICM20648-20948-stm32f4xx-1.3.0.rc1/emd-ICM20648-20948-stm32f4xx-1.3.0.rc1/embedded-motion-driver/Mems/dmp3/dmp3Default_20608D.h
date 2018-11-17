/*
 * Copyright (C) 2012 Invensense, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#if defined MEMS_20609

#include "../common/mltypes.h"
inv_error_t inv_load_firmware_20609(const unsigned char *dmp_image_sram);
void inv_get_dmp_start_address_20609(unsigned short *dmp_cnfg);
inv_error_t dmp_set_sensor_rate_20609(int invSensor, short divider);
int dmp_set_dataout_control1_20609(uint16_t cntl1);
int dmp_set_dataout_control2_20609(uint16_t cntl2);
int dmp_set_motion_interrupt_control_20609(uint16_t motion_event_cntl);
inv_error_t dmp_get_pedometer_num_of_steps_20609(unsigned long *steps);
int inv_add_step_indicator(int enable);
int inv_enable_batch(int on);
static int inv_enable_eis(int enable);
int inv_enable_gyro_cal(int en);
int inv_enable_pedometer(int en);
int inv_enable_pedometer_interrupt(int en);
int inv_enable_smd(int en);
static int inv_out_fsync(int enable);
int inv_send_accel_data(int enable);
int inv_send_gyro_data(int enable);
int inv_send_ped_q_data(int enable);
int inv_send_six_q_data(int enable);
int inv_send_stepdet_data(int enable);
inv_error_t dmp_reset_odr_counters_20609();
inv_error_t dmp_set_bias_20609(int *bias);
inv_error_t dmp_get_bias_20609(int *bias);

#endif