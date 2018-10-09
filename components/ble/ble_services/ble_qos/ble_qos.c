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
#if NRF_MODULE_ENABLED(BLE_QOS)
#include "ble_qos.h"
#include <string.h>
#include "ble_srv_common.h"
#include "sdk_config.h"

#define NRF_LOG_MODULE_NAME qos

#if BLE_SRV_QOS_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BLE_SRV_QOS_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BLE_SRV_QOS_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BLE_SRV_QOS_CONFIG_DEBUG_COLOR
#else
#define NRF_LOG_LEVEL       0
#endif
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();


#define OPCODE_LENGTH 1                                                              /**< Length of opcode inside Quaternion Orientation packet. */
#define HANDLE_LENGTH 2                                                              /**< Length of handle inside Quaternion Orientation packet. */
#define MAX_QOM_LEN      (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted Quaternion Orientation. */

#define INITIAL_VALUE_QOM                       0                                    /**< Initial Quaternion Orientation value. */

// Heart Rate Measurement flag bits
#define QOM_FLAG_MASK_HR_VALUE_16BIT            (0x01 << 0)                           /**< Quaternion Orientation Value Format bit. */
#define QOM_FLAG_MASK_SENSOR_CONTACT_DETECTED   (0x01 << 1)                           /**< Sensor Contact Detected bit. */
#define QOM_FLAG_MASK_SENSOR_CONTACT_SUPPORTED  (0x01 << 2)                           /**< Sensor Contact Supported bit. */
#define QOM_FLAG_MASK_EXPENDED_ENERGY_INCLUDED  (0x01 << 3)                           /**< Energy Expended Status bit. Feature Not Supported */
#define QOM_FLAG_MASK_RR_INTERVAL_INCLUDED      (0x01 << 4)                           /**< RR-Interval bit. */


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_qos       Quaternion Orientation Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_qos_t * p_qos, ble_evt_t const * p_ble_evt)
{
    p_qos->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_qos       Quaternion Orientation Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_qos_t * p_qos, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_qos->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the Quaternion Orientation Measurement characteristic.
 *
 * @param[in]   p_qos         Quaternion Orientation Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_qom_cccd_write(ble_qos_t * p_qos, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_qos->evt_handler != NULL)
        {
            ble_qos_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_QOS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_QOS_EVT_NOTIFICATION_DISABLED;
            }

            p_qos->evt_handler(p_qos, &evt);
        }
    }
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_qos       Quaternion Orientation Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_qos_t * p_qos, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_qos->qom_handles.cccd_handle)
    {
        on_qom_cccd_write(p_qos, p_evt_write);
    }
}


void ble_qos_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_qos_t * p_qos = (ble_qos_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_qos, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_qos, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_qos, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for encoding a Quaternion Orientation Measurement.
 *
 * @param[in]   p_qos              Heart Rate Service structure.
 * @param[in]   quaternion         Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t qom_encode(ble_qos_t * p_qos, uint16_t heart_rate, uint8_t * p_encoded_buffer)
{
    uint8_t flags = 0;
    uint8_t len   = 1;
    int     i;

    // Set sensor contact related flags
    if (p_qos->is_sensor_contact_supported)
    {
        flags |= QOM_FLAG_MASK_SENSOR_CONTACT_SUPPORTED;
    }
    if (p_qos->is_sensor_contact_detected)
    {
        flags |= QOM_FLAG_MASK_SENSOR_CONTACT_DETECTED;
    }

    // Encode quaternion measurement
    if (heart_rate > 0xff)
    {
        flags |= QOM_FLAG_MASK_HR_VALUE_16BIT;
        len   += uint16_encode(heart_rate, &p_encoded_buffer[len]);
    }
    else
    {
        p_encoded_buffer[len++] = (uint8_t)heart_rate;
    }

    // Encode rr_interval values
    if (p_qos->rr_interval_count > 0)
    {
        flags |= QOM_FLAG_MASK_RR_INTERVAL_INCLUDED;
    }
    for (i = 0; i < p_qos->rr_interval_count; i++)
    {
        if (len + sizeof(uint16_t) > p_qos->max_qom_len)
        {
            // Not all stored rr_interval values can fit into the encoded qom,
            // move the remaining values to the start of the buffer.
            memmove(&p_qos->rr_interval[0],
                    &p_qos->rr_interval[i],
                    (p_qos->rr_interval_count - i) * sizeof(uint16_t));
            break;
        }
        len += uint16_encode(p_qos->rr_interval[i], &p_encoded_buffer[len]);
    }
    p_qos->rr_interval_count -= i;

    // Add flags
    p_encoded_buffer[0] = flags;

    return len;
}


/**@brief Function for adding the Quaternion Measurement characteristic.
 *
 * @param[in]   p_qos        Heart Rate Service structure.
 * @param[in]   p_qos_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t heart_rate_measurement_char_add(ble_qos_t            * p_qos,
                                                const ble_qos_init_t * p_qos_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             encoded_initial_qom[MAX_QOM_LEN];

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_qos_init->qos_qom_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_QUATERNION_ORIENTATION_MEASUREMENT_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_qos_init->qos_qom_attr_md.read_perm;
    attr_md.write_perm = p_qos_init->qos_qom_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 2;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 2;
    attr_char_value.p_value   = encoded_initial_qom;

    return sd_ble_gatts_characteristic_add(p_qos->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_qos->qom_handles);
}


/**@brief Function for adding the Body Sensor Location characteristic.
 *
 * @param[in]   p_qos        Heart Rate Service structure.
 * @param[in]   p_qos_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t body_sensor_location_char_add(ble_qos_t * p_qos, const ble_qos_init_t * p_qos_init)
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

    attr_md.read_perm  = p_qos_init->qos_bsl_attr_md.read_perm;
    attr_md.write_perm = p_qos_init->qos_bsl_attr_md.write_perm;
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
    attr_char_value.p_value   = p_qos_init->p_body_sensor_location;

    return sd_ble_gatts_characteristic_add(p_qos->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_qos->bsl_handles);
}


uint32_t ble_qos_init(ble_qos_t * p_qos, const ble_qos_init_t * p_qos_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_qos->evt_handler                 = p_qos_init->evt_handler;
    p_qos->is_sensor_contact_supported = p_qos_init->is_sensor_contact_supported;
    p_qos->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_qos->is_sensor_contact_detected  = false;
    p_qos->rr_interval_count           = 0;
    p_qos->max_qom_len                 = MAX_QOM_LEN;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_QUATERNION_ORIENTATION_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_qos->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add heart rate measurement characteristic
    err_code = heart_rate_measurement_char_add(p_qos, p_qos_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (p_qos_init->p_body_sensor_location != NULL)
    {
        // Add body sensor location characteristic
        err_code = body_sensor_location_char_add(p_qos, p_qos_init);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    return NRF_SUCCESS;
}


uint32_t ble_qos_orientation_measurement_send(ble_qos_t * p_qos, uint16_t heart_rate)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_qos->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_qom[MAX_QOM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        //len     = qom_encode(p_qos, heart_rate, encoded_qom);
        //hvx_len = len;
		
		encoded_qom[0] = 0xBE;
		encoded_qom[1] = 0xEF;
		len = 2;
		hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_qos->qom_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_qom;

        err_code = sd_ble_gatts_hvx(p_qos->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


void ble_qos_rr_interval_add(ble_qos_t * p_qos, uint16_t rr_interval)
{
    if (p_qos->rr_interval_count == BLE_QOS_MAX_BUFFERED_RR_INTERVALS)
    {
        // The rr_interval buffer is full, delete the oldest value
        memmove(&p_qos->rr_interval[0],
                &p_qos->rr_interval[1],
                (BLE_QOS_MAX_BUFFERED_RR_INTERVALS - 1) * sizeof(uint16_t));
        p_qos->rr_interval_count--;
    }

    // Add new value
    p_qos->rr_interval[p_qos->rr_interval_count++] = rr_interval;
}


bool ble_qos_rr_interval_buffer_is_full(ble_qos_t * p_qos)
{
    return (p_qos->rr_interval_count == BLE_QOS_MAX_BUFFERED_RR_INTERVALS);
}


uint32_t ble_qos_sensor_contact_supported_set(ble_qos_t * p_qos, bool is_sensor_contact_supported)
{
    // Check if we are connected to peer
    if (p_qos->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        p_qos->is_sensor_contact_supported = is_sensor_contact_supported;
        return NRF_SUCCESS;
    }
    else
    {
        return NRF_ERROR_INVALID_STATE;
    }
}


void ble_qos_sensor_contact_detected_update(ble_qos_t * p_qos, bool is_sensor_contact_detected)
{
    p_qos->is_sensor_contact_detected = is_sensor_contact_detected;
}


uint32_t ble_qos_body_sensor_location_set(ble_qos_t * p_qos, uint8_t body_sensor_location)
{
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &body_sensor_location;

    return sd_ble_gatts_value_set(p_qos->conn_handle, p_qos->bsl_handles.value_handle, &gatts_value);
}


void ble_qos_on_gatt_evt(ble_qos_t * p_qos, nrf_ble_gatt_evt_t const * p_gatt_evt)
{
    if (    (p_qos->conn_handle == p_gatt_evt->conn_handle)
        &&  (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        p_qos->max_qom_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}
#endif // NRF_MODULE_ENABLED(BLE_QOS)
