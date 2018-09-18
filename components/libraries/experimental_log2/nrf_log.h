/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */


#ifndef NRF_LOG_H_
#define NRF_LOG_H_

#include "sdk_common.h"
#include "nrf_section.h"
#if NRF_MODULE_ENABLED(NRF_LOG)

#if (NRF_LOG_BACKEND_UART_ENABLED)
#include "uart_backend"
#define NRF_LOG_INIT(NULL);                    LOG_INIT()
#define NRF_LOG_ERROR(...)                     printf(__VA_ARGS__)
#define NRF_LOG_WARNING(...)                   printf( __VA_ARGS__)
#define NRF_LOG_INFO(...)                      printf( __VA_ARGS__)
#define NRF_LOG_DEBUG(...)                     printf( __VA_ARGS__)
#elif
#define "rtt_backend";                         LOG_INIT()
#define NRF_LOG_INIT(NULL);                    LOG_INIT()
#define NRF_LOG_ERROR(...)                     printf(__VA_ARGS__)
#define NRF_LOG_WARNING(...)                   printf( __VA_ARGS__)
#define NRF_LOG_INFO(...)                      printf( __VA_ARGS__)
#define NRF_LOG_DEBUG(...)                     printf( __VA_ARGS__)
#else
#warning NO NRF LOG BACKEND DEFINED
#endif


#endif

#endif // NRF_LOG_H_

/** @} */
