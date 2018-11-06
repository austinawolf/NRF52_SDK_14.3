/**
 * Copyright (c) 2012 - 2017, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup ble_motion Motion Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Motion Service module.
 *
 * @details This module implements the Motion Service with the Motion Measurement,
 *          Body Sensor Location and Motion Control Point characteristics.
 *          During initialization it adds the Motion Service and Motion Measurement
 *          characteristic to the BLE stack database. Optionally it also adds the
 *          Body Sensor Location and Motion Control Point characteristics.
 *
 *          If enabled, notification of the Motion Measurement characteristic is performed
 *          when the application calls ble_motion_heart_rate_measurement_send().
 *
 *          The Motion Service also provides a set of functions for manipulating the
 *          various fields in the Motion Measurement characteristic, as well as setting
 *          the Body Sensor Location characteristic value.
 *
 *          If an event handler is supplied by the application, the Motion Service will
 *          generate Motion Service events to the application.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_motion_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_MOTION_BLE_OBSERVER_PRIO,
 *                                   ble_motion_on_ble_evt, &instance);
 *          @endcode
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_MOTION_H__
#define BLE_MOTION_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_motion instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_MOTION_DEF(_name)                                                                          \
ble_motion_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_MOTION_BLE_OBSERVER_PRIO,                                                     \
                     ble_motion_on_ble_evt, &_name)

// Body Sensor Location values
#define BLE_MOTION_BODY_SENSOR_LOCATION_OTHER      0
#define BLE_MOTION_BODY_SENSOR_LOCATION_CHEST      1
#define BLE_MOTION_BODY_SENSOR_LOCATION_WRIST      2
#define BLE_MOTION_BODY_SENSOR_LOCATION_FINGER     3
#define BLE_MOTION_BODY_SENSOR_LOCATION_HAND       4
#define BLE_MOTION_BODY_SENSOR_LOCATION_EAR_LOBE   5
#define BLE_MOTION_BODY_SENSOR_LOCATION_FOOT       6

#define MOTION_PREAMBLE 0xaa

// Motion Packet Flags
#define QUAT_PACKET_FLAG    	(1<<0)
#define IMU_PACKET_FLAG			(1<<1)
#define COMPASS_PACKET_FLAG		(1<<2)
#define TIMESTAMP_PACKET_FLAG   (1<<3)
//#define UNUSED				(1<<4)
//#define UNUSED				(1<<5)
//#define UNUSED				(1<<6)
//#define UNUSED				(1<<7)


/**@brief Motion Service event type. */
typedef enum
{
    BLE_MOTION_EVT_NOTIFICATION_ENABLED,   /**< Motion value notification enabled event. */
    BLE_MOTION_EVT_NOTIFICATION_DISABLED   /**< Motion value notification disabled event. */
} ble_motion_evt_type_t;

/**@brief Motion Service event. */
typedef struct
{
    ble_motion_evt_type_t evt_type;    /**< Type of event. */
} ble_motion_evt_t;

// Forward declaration of the ble_motion_t type.
typedef struct ble_motion_s ble_motion_t;

/**@brief Motion Service event handler type. */
typedef void (*ble_motion_evt_handler_t) (ble_motion_t * p_motion, ble_motion_evt_t * p_evt);

/**@brief Motion Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_motion_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the Motion Service. */
    bool                         is_sensor_contact_supported;                          /**< Determines if sensor contact detection is to be supported. */
    uint8_t *                    p_body_sensor_location;                               /**< If not NULL, initial value of the Body Sensor Location characteristic. */
    ble_srv_cccd_security_mode_t motion_motionm_attr_md;                                      /**< Initial security level for Motion service measurement attribute */
    ble_srv_security_mode_t      motion_bsl_attr_md;                                      /**< Initial security level for body sensor location attribute */
} ble_motion_init_t;

/**@brief Motion Service structure. This contains various status information for the service. */
struct ble_motion_s
{
    ble_motion_evt_handler_t        evt_handler;                                          /**< Event handler to be called for handling events in the Motion Service. */
    bool                         is_expended_energy_supported;                         /**< TRUE if Expended Energy measurement is supported. */
    bool                         is_sensor_contact_supported;                          /**< TRUE if sensor contact detection is supported. */
    uint16_t                     service_handle;                                       /**< Handle of Motion Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     motionm_handles;                                      /**< Handles related to the Motion Measurement characteristic. */
	ble_gatts_char_handles_t	 command_handles;
    ble_gatts_char_handles_t     bsl_handles;                                          /**< Handles related to the Body Sensor Location characteristic. */
    ble_gatts_char_handles_t     hrcp_handles;                                         /**< Handles related to the Motion Control Point characteristic. */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                         is_sensor_contact_detected;                           /**< TRUE if sensor contact has been detected. */
    uint8_t                      max_motionm_len;
	uint8_t						 uuid_type;
	/**< Current maximum HR measurement length, adjusted according to the current ATT MTU. */
};


/**@brief Function for initializing the Motion Service.
 *
 * @param[out]  p_motion       Motion Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_motion_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_motion_init(ble_motion_t * p_motion, ble_motion_init_t const * p_motion_init);


/**@brief Function for handling the GATT module's events.
 *
 * @details Handles all events from the GATT module of interest to the Motion Service.
 *
 * @param[in]   p_motion      Motion Service structure.
 * @param[in]   p_gatt_evt  Event received from the GATT module.
 */
void ble_motion_on_gatt_evt(ble_motion_t * p_motion, nrf_ble_gatt_evt_t const * p_gatt_evt);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Motion Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Motion Service structure.
 */
void ble_motion_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending Motion measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a Motion measurement.
 *          If notification has been enabled, the Motion measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_motion                 Motion Service structure.
 * @param[in]   motion               	 New Motion measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_motion_quaternion_send(ble_motion_t * p_motion, int32_t * quat);

/**@brief Function for sending Motion measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a Motion measurement.
 *          If notification has been enabled, the Motion measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_motion                 Motion Service structure.
 * @param[in]   motion               	 New Motion measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_motion_imu_send(ble_motion_t * p_motion, int16_t * gyro, int16_t * accel);

/**@brief Function for sending Motion measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a Motion measurement.
 *          If notification has been enabled, the Motion measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_motion                 Motion Service structure.
 * @param[in]   motion               	 New Motion measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_motion_compass_send(ble_motion_t * p_motion, int16_t * compass);

/**@brief Function for setting the state of the Sensor Contact Supported bit.
 *
 * @param[in]   p_motion                        Motion Service structure.
 * @param[in]   is_sensor_contact_supported  New state of the Sensor Contact Supported bit.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_motion_sensor_contact_supported_set(ble_motion_t * p_motion, bool is_sensor_contact_supported);


/**@brief Function for setting the state of the Sensor Contact Detected bit.
 *
 * @param[in]   p_motion                        Motion Service structure.
 * @param[in]   is_sensor_contact_detected   TRUE if sensor contact is detected, FALSE otherwise.
 */
void ble_motion_sensor_contact_detected_update(ble_motion_t * p_motion, bool is_sensor_contact_detected);


/**@brief Function for setting the Body Sensor Location.
 *
 * @details Sets a new value of the Body Sensor Location characteristic. The new value will be sent
 *          to the client the next time the client reads the Body Sensor Location characteristic.
 *
 * @param[in]   p_motion                 Motion Service structure.
 * @param[in]   body_sensor_location  New Body Sensor Location.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_motion_body_sensor_location_set(ble_motion_t * p_motion, uint8_t body_sensor_location);


#ifdef __cplusplus
}
#endif

#endif // BLE_MOTION_H__

/** @} */
