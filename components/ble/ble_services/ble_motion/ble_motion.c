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
/* Attention!
 * To maintain compliance with Nordic Semiconductor ASA's Bluetooth profile
 * qualification listings, this section of source code must not be modified.
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_MOTION)
#include "ble_motion.h"
#include <string.h>
#include "ble_srv_common.h"
#include "sdk_config.h"

#define NRF_LOG_MODULE_NAME motion

#if BLE_SRV_MOTION_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_SRV_MOTION_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_SRV_MOTION_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_SRV_MOTION_CONFIG_DEBUG_COLOR
#else
#define NRF_LOG_LEVEL       0
#endif
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();


#define OPCODE_LENGTH 1                                                              /**< Length of opcode inside Quaternion Orientation packet. */
#define HANDLE_LENGTH 2                                                              /**< Length of handle inside Quaternion Orientation packet. */
#define MAX_MOTIONM_LEN      (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maxmotionm size of a transmitted Quaternion Orientation. */

#define INITIAL_VALUE_MOTIONM                       0                                    /**< Initial Quaternion Orientation value. */

// Heart Rate Measurement flag bits
#define MOTIONM_FLAG_MASK_HR_VALUE_16BIT            (0x01 << 0)                           /**< Quaternion Orientation Value Format bit. */
#define MOTIONM_FLAG_MASK_SENSOR_CONTACT_DETECTED   (0x01 << 1)                           /**< Sensor Contact Detected bit. */
#define MOTIONM_FLAG_MASK_SENSOR_CONTACT_SUPPORTED  (0x01 << 2)                           /**< Sensor Contact Supported bit. */
#define MOTIONM_FLAG_MASK_EXPENDED_ENERGY_INCLUDED  (0x01 << 3)                           /**< Energy Expended Status bit. Feature Not Supported */
#define MOTIONM_FLAG_MASK_RR_INTERVAL_INCLUDED      (0x01 << 4)                           /**< RR-Interval bit. */

uint8_t packet_num = 0;

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_motion       Quaternion Orientation Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_motion_t * p_motion, ble_evt_t const * p_ble_evt)
{
    p_motion->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_motion       Quaternion Orientation Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_motion_t * p_motion, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_motion->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the Quaternion Orientation Measurement characteristic.
 *
 * @param[in]   p_motion         Quaternion Orientation Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_motionm_cccd_write(ble_motion_t * p_motion, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_motion->evt_handler != NULL)
        {
            ble_motion_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_MOTION_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_MOTION_EVT_NOTIFICATION_DISABLED;
            }

            p_motion->evt_handler(p_motion, &evt);
        }
    }
}


/**@brief Function for handling write events to the Quaternion Orientation Measurement characteristic.
 *
 * @param[in]   p_motion         Quaternion Orientation Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static uint32_t on_motion_command_char_write(ble_motion_t * p_motion, ble_gatts_evt_write_t const * p_evt_write)
{
    uint8_t read_value, write_value;
	ble_gatts_value_t gatts_value;

	read_value = p_evt_write->data[0];
	NRF_LOG_DEBUG("Command Characterstic Value Read: %x", read_value);
	
	write_value = ~read_value;
	NRF_LOG_DEBUG("Command Characterstic Value Write: %x", write_value);

	gatts_value.p_value = &write_value;
	gatts_value.offset = 0;
	gatts_value.len = 1;
	

	return sd_ble_gatts_value_set(p_motion->conn_handle, p_motion->command_handles.value_handle, &gatts_value);
	
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_motion       Quaternion Orientation Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_motion_t * p_motion, ble_evt_t const * p_ble_evt)
{
	uint32_t err_code;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	if (p_evt_write->handle == p_motion->motionm_handles.cccd_handle)
    {
        on_motionm_cccd_write(p_motion, p_evt_write);
    }
    else if (p_evt_write->handle == p_motion->command_handles.value_handle)
    {
		err_code = on_motion_command_char_write(p_motion, p_evt_write);
        if (err_code) {
			NRF_LOG_ERROR("on_motion_command_char_write: %d", err_code);
		}
    }	
	
}


void ble_motion_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_motion_t * p_motion = (ble_motion_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_motion, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_motion, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_motion, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding the Quaternion Measurement characteristic.
 *
 * @param[in]   p_motion        Heart Rate Service structure.
 * @param[in]   p_motion_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t quaternion_meas_char_add(ble_motion_t * p_motion, const ble_motion_init_t * p_motion_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             encoded_initial_motionm[MAX_MOTIONM_LEN];
	uint32_t			err_code;
	
    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_motion_init->motion_motionm_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;	
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    // Add Custom Char UUID
    ble_uuid128_t base_uuid = {BLE_UUID_ORIENTATION_CHAR_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_motion->uuid_type);
    VERIFY_SUCCESS(err_code);
	
    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_ORIENTATION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_motion_init->motion_motionm_attr_md.read_perm;
    attr_md.write_perm = p_motion_init->motion_motionm_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 20;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;
    attr_char_value.p_value   = encoded_initial_motionm;

    return sd_ble_gatts_characteristic_add(p_motion->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_motion->motionm_handles);
}


/**@brief Function for adding the Quaternion Measurement characteristic.
 *
 * @param[in]   p_motion        Heart Rate Service structure.
 * @param[in]   p_motion_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t command_char_add(ble_motion_t * p_motion, const ble_motion_init_t * p_motion_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             encoded_initial_motionm = 0xaa;
	uint32_t			err_code;
	
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;	
	char_md.char_props.notify = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_motion_init->motion_motionm_attr_md.read_perm;
    attr_md.write_perm = p_motion_init->motion_motionm_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {BLE_UUID_COMMAND_CHAR_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_motion->uuid_type);
    VERIFY_SUCCESS(err_code);
	
    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_COMMAND_CHAR;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 1;
    attr_char_value.p_value   = &encoded_initial_motionm;

    return sd_ble_gatts_characteristic_add(p_motion->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_motion->command_handles);
}


/**@brief Function for adding the Body Sensor Location characteristic.
 *
 * @param[in]   p_motion        Heart Rate Service structure.
 * @param[in]   p_motion_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t body_sensor_location_char_add(ble_motion_t * p_motion, const ble_motion_init_t * p_motion_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BODY_SENSOR_LOCATION_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_motion_init->motion_bsl_attr_md.read_perm;
    attr_md.write_perm = p_motion_init->motion_bsl_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof (uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof (uint8_t);
    attr_char_value.p_value   = p_motion_init->p_body_sensor_location;

    return sd_ble_gatts_characteristic_add(p_motion->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_motion->bsl_handles);
}


uint32_t ble_motion_init(ble_motion_t * p_motion, const ble_motion_init_t * p_motion_init)
{
    
    if (p_motion == NULL || p_motion_init == NULL)
    {
        return NRF_ERROR_NULL;
    }	
	
	uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_motion->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_motion->evt_handler                 = p_motion_init->evt_handler;
    p_motion->is_sensor_contact_supported = p_motion_init->is_sensor_contact_supported;
    p_motion->is_sensor_contact_detected  = false;
    p_motion->max_motionm_len             = MAX_MOTIONM_LEN;

	
    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {BLE_UUID_MOTION_SERVICE_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_motion->uuid_type);
    VERIFY_SUCCESS(err_code);
	
    ble_uuid.type = p_motion->uuid_type;
    ble_uuid.uuid = BLE_UUID_MOTION_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_motion->service_handle);	
	
	
	
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add quaterion characteristic
    err_code = quaternion_meas_char_add(p_motion, p_motion_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = command_char_add(p_motion, p_motion_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }	
	
	
    if (p_motion_init->p_body_sensor_location != NULL)
    {
        // Add body sensor location characteristic
        err_code = body_sensor_location_char_add(p_motion, p_motion_init);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    return NRF_SUCCESS;
}

static void motion_encode(uint8_t * data,  uint8_t preamble, uint8_t flags, uint8_t packet_number, Motion * motion) {
	//byte 0: 0xaa (preamble)
	//byte 1: flags
	//byte 2: packet number
	//byte 3-17: quaternion
	//byte 18-19: empty
	memcpy(&data[0], &preamble, 1);
	memcpy(&data[1], &flags,    1);
	memcpy(&data[2], &packet_number, 1);
	memcpy(&data[3], &motion->quat, 16);
}

uint32_t ble_motion_orientation_send(ble_motion_t * p_motion, Motion * motion)
{
    uint32_t err_code;

	
    // Send value if connected and notifying
    if (p_motion->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t				   preamble;
		uint8_t				   flags;
		uint8_t                encoded_data[20];   
        uint16_t               hvx_len;
		
		memset(encoded_data, 0, 20);
		
        ble_gatts_hvx_params_t hvx_params;	
		
		preamble = 0xaa;
		flags = 0;
		motion_encode(encoded_data, preamble, flags, packet_num++, motion);
		hvx_len = 20;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_motion->motionm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_data;

        err_code = sd_ble_gatts_hvx(p_motion->conn_handle, &hvx_params);

    }
    else
    {
		NRF_LOG_ERROR("NRF_ERROR_INVALID_STATE");
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


uint32_t ble_motion_sensor_contact_supported_set(ble_motion_t * p_motion, bool is_sensor_contact_supported)
{
    // Check if we are connected to peer
    if (p_motion->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        p_motion->is_sensor_contact_supported = is_sensor_contact_supported;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }
}


void ble_motion_sensor_contact_detected_update(ble_motion_t * p_motion, bool is_sensor_contact_detected)
{
    p_motion->is_sensor_contact_detected = is_sensor_contact_detected;
}


uint32_t ble_motion_body_sensor_location_set(ble_motion_t * p_motion, uint8_t body_sensor_location)
{
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &body_sensor_location;

    return sd_ble_gatts_value_set(p_motion->conn_handle, p_motion->bsl_handles.value_handle, &gatts_value);
}


void ble_motion_on_gatt_evt(ble_motion_t * p_motion, nrf_ble_gatt_evt_t const * p_gatt_evt)
{
    if (    (p_motion->conn_handle == p_gatt_evt->conn_handle)
        &&  (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        p_motion->max_motionm_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}
#endif // NRF_MODULE_ENABLED(BLE_MOTION)
