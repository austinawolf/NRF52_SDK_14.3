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
/**@cond To Make Doxygen skip documentation generation for this file.
 * @{
 */
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_MOTION_C)
#include "ble_motion_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_srv_common.h"
#include "ble_gattc.h"
#include "nordic_common.h"

#define NRF_LOG_MODULE_NAME ble_motion_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define MOTIONM_FLAG_MASK_HR_16BIT  (0x01 << 0)           /**< Bit mask used to extract the type of heart rate value. This is used to find if the received heart rate is a 16 bit value or an 8 bit value. */
#define MOTIONM_FLAG_MASK_HR_RR_INT (0x01 << 4)           /**< Bit mask used to extract the presence of RR_INTERVALS. This is used to find if the received measurement includes RR_INTERVALS. */

#define TX_BUFFER_MASK         0x07                  /**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE         (TX_BUFFER_MASK + 1)  /**< Size of send buffer, which is 1 higher than the mask. */

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */
#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */

typedef enum
{
    READ_REQ,  /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ  /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
    uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                       /**< GATTC parameters for this message. */
} write_params_t;

/**@brief Structure for holding data to be transmitted to the connected central.
 */
typedef struct
{
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    tx_request_t type;         /**< Type of this message, i.e. read or write message. */
    union
    {
        uint16_t       read_handle;  /**< Read request message. */
        write_params_t write_req;    /**< Write request message. */
    } req;
} tx_message_t;


static tx_message_t  m_tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer for messages to be transmitted to the central. */
static uint32_t      m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t      m_tx_index = 0;               /**< Current index in the transmit buffer from where the next message to be transmitted resides. */


/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index)
    {
        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ)
        {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        }
        else
        {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }
        if (err_code == NRF_SUCCESS)
        {
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        }
        else
        {
            NRF_LOG_DEBUG("SD Read/Write API returns error. This message sending will be "
                          "attempted again..");
        }
    }
}


/**@brief     Function for handling write response events.
 *
 * @param[in] p_ble_motion_c Pointer to the Quaternion Orientation Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_write_rsp(ble_motion_c_t * p_ble_motion_c, const ble_evt_t * p_ble_evt)
{
    // Check if the event if on the link for this instance
    if (p_ble_motion_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    // Check if there is any message to be sent across to the peer and send it.
    tx_buffer_process();
}


static int motion_decode(const ble_gattc_evt_hvx_t * hvx,  uint8_t length, ble_motion_c_evt_t * ble_motion_c_evt) {
	
	if (hvx->data[0] != 0xaa) {
		return -1;
	}
	
	uint8_t data_flags = hvx->data[1];
	uint8_t packet_num = hvx->data[2];
	int32_t quat[4] = {0,0,0,0};
	int16_t gyro[3] = {0,0,0};
	int16_t accel[3] = {0,0,0};
	int16_t compass[3] = {0,0,0};
	
	
	if (data_flags == 0x02) {
		//imu
		memcpy(gyro, &hvx->data[3],6);
		memcpy(accel, &hvx->data[9],6);
		printf("Packet:%u,%u,%d,%d,%d,%d,%d,%d\n\r",packet_num,data_flags,gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2]);

	}
	if (data_flags == 0x01) {
		//quat
		memcpy(quat, &hvx->data[3],16);
		printf("Packet:%u,%u,%ld,%ld,%ld,%ld\n\r",packet_num,data_flags, (long) quat[0], (long) quat[1], (long) quat[2], (long) quat[3]);

	}
	if (data_flags == 0x04) {
		//compass
		memcpy(compass, &hvx->data[3],6);
		printf("Packet:%u,%u,%d,%d,%d\n\r",packet_num,data_flags,compass[0],compass[1],compass[2]);
	}
	
	//memcpy(&ble_motion_c_evt->params.motionm.motion_data.quat.q[0], &hvx->data[3], length - 3);
	return 0;
}




/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function will uses the Handle Value Notification received from the SoftDevice
 *            and checks if it is a notification of the quaternion orientation measurement from the peer. If
 *            it is, this function will decode the quaternion orientation measurement and send it to the
 *            application.
 *
 * @param[in] p_ble_motion_c Pointer to the quaternion orientation Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_motion_c_t * p_ble_motion_c, const ble_evt_t * p_ble_evt)
{
	int err_code;
	
	// Check if the event is on the link for this instance
    if (p_ble_motion_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        NRF_LOG_DEBUG("Received HVX on link 0x%x, not associated to instance 0x%x, ignore",
                      p_ble_evt->evt.gattc_evt.conn_handle, p_ble_motion_c->conn_handle);
        
		return;
    }
	
    NRF_LOG_DEBUG("Received HVX on link 0x%x, motionm_handle 0x%x",
    p_ble_evt->evt.gattc_evt.params.hvx.handle,
    p_ble_motion_c->peer_motion_db.motionm_handle);
	NRF_LOG_DEBUG("packet num: %d, flag: %d", p_ble_evt->evt.gattc_evt.params.hvx.data[2], p_ble_evt->evt.gattc_evt.params.hvx.data[1]);

    // Check if this is a quaternion orientation notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_motion_c->peer_motion_db.motionm_handle)
    {
        ble_motion_c_evt_t ble_motion_c_evt;

        ble_motion_c_evt.evt_type                    = BLE_MOTION_C_EVT_MOTIONM_NOTIFICATION;
        ble_motion_c_evt.conn_handle                 = p_ble_motion_c->conn_handle;
					
		err_code = motion_decode(&p_ble_evt->evt.gattc_evt.params.hvx, p_ble_evt->evt.gattc_evt.params.hvx.len, &ble_motion_c_evt);
					
		
		if (!err_code) {
			p_ble_motion_c->evt_handler(p_ble_motion_c, &ble_motion_c_evt);
		}
    }
}



/**@brief     Function for handling Disconnected event received from the SoftDevice.
 *
 * @details   This function check if the disconnect event is happening on the link
 *            associated with the current instance of the module, if so it will set its
 *            conn_handle to invalid.
 *
 * @param[in] p_ble_motion_c Pointer to the Quaternion Orientation Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_motion_c_t * p_ble_motion_c, const ble_evt_t * p_ble_evt)
{
    if (p_ble_motion_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ble_motion_c->conn_handle                 = BLE_CONN_HANDLE_INVALID;
        p_ble_motion_c->peer_motion_db.motionm_cccd_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_motion_c->peer_motion_db.motionm_handle      = BLE_GATT_HANDLE_INVALID;
    }
}


void ble_motion_on_db_disc_evt(ble_motion_c_t * p_ble_motion_c, const ble_db_discovery_evt_t * p_evt)
{
	NRF_LOG_DEBUG("evt_type: %d", p_evt->evt_type);
	NRF_LOG_DEBUG("uuid: 0x%x", p_evt->params.discovered_db.srv_uuid.uuid);
	NRF_LOG_DEBUG("type: %d", p_evt->params.discovered_db.srv_uuid.type);
	
    // Check if the Quaternion Orientation Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_MOTION_SERVICE)
    {
        // Find the CCCD Handle of the Quaternion Orientation Measurement characteristic.
        uint32_t i;

        ble_motion_c_evt_t evt;

        evt.evt_type    = BLE_MOTION_C_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;

		NRF_LOG_DEBUG("Chars found: %d", p_evt->params.discovered_db.char_count);
        for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
			NRF_LOG_DEBUG("Char discovery: 0x%x, %d", p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid, p_evt->params.discovered_db.charateristics[i].characteristic.uuid.type)
            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_ORIENTATION_CHAR)
            {
                // Found Quaternion Orientation characteristic. Store CCCD handle and break.
                evt.params.peer_db.motionm_cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                evt.params.peer_db.motionm_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                
				break;
				
            }
        }

        NRF_LOG_DEBUG("Quaternion Service discovered at peer.");
        //If the instance has been assigned prior to db_discovery, assign the db_handles
        if (p_ble_motion_c->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if ((p_ble_motion_c->peer_motion_db.motionm_cccd_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_ble_motion_c->peer_motion_db.motionm_handle == BLE_GATT_HANDLE_INVALID))
            {
                p_ble_motion_c->peer_motion_db = evt.params.peer_db;
            }
        }


        p_ble_motion_c->evt_handler(p_ble_motion_c, &evt);
    }
}


uint32_t ble_motion_c_init(ble_motion_c_t * p_ble_motion_c, ble_motion_c_init_t * p_ble_motion_c_init)
{
	uint32_t err_code;
	
    VERIFY_PARAM_NOT_NULL(p_ble_motion_c);
    VERIFY_PARAM_NOT_NULL(p_ble_motion_c_init);

	
	
    // Add Custom Service UUID
    ble_uuid_t motion_uuid;	
    ble_uuid128_t base_uuid = {BLE_UUID_MOTION_SERVICE_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &motion_uuid.type);
	motion_uuid.uuid = BLE_UUID_MOTION_SERVICE;
    VERIFY_SUCCESS(err_code);
	
    // Add Custom Service UUID
    ble_uuid_t orientation_uuid;	
    ble_uuid128_t orientation_base_uuid = {BLE_UUID_ORIENTATION_CHAR_BASE};
    err_code =  sd_ble_uuid_vs_add(&orientation_base_uuid, &orientation_uuid.type);

    // Add Custom Service UUID
    ble_uuid_t command_uuid;	
    ble_uuid128_t command_base_uuid = {BLE_UUID_COMMAND_CHAR_BASE};
    err_code =  sd_ble_uuid_vs_add(&command_base_uuid, &command_uuid.type);	
	
    p_ble_motion_c->evt_handler                 = p_ble_motion_c_init->evt_handler;
    p_ble_motion_c->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_ble_motion_c->peer_motion_db.motionm_cccd_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_motion_c->peer_motion_db.motionm_handle      = BLE_GATT_HANDLE_INVALID;

    return ble_db_discovery_evt_register(&motion_uuid);
}

void ble_motion_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
     ble_motion_c_t * p_ble_motion_c = (ble_motion_c_t *)p_context;

    if ((p_ble_motion_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_motion_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_ble_motion_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_motion_c, p_ble_evt);
            break;

        default:
            break;
    }
}


/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool enable)
{
    NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
        handle_cccd,conn_handle);

    tx_message_t * p_msg;
    uint16_t       cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    p_msg->req.write_req.gattc_params.len      = WRITE_MESSAGE_LENGTH;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}


uint32_t ble_motion_c_motionm_notif_enable(ble_motion_c_t * p_ble_motion_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_motion_c);

    return cccd_configure(p_ble_motion_c->conn_handle,
                          p_ble_motion_c->peer_motion_db.motionm_cccd_handle,
                          true);
}


uint32_t ble_motion_c_handles_assign(ble_motion_c_t * p_ble_motion_c,
                                  uint16_t conn_handle,
                                  const motion_db_t * p_peer_motion_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_motion_c);

    p_ble_motion_c->conn_handle = conn_handle;
    if (p_peer_motion_handles != NULL)
    {
        p_ble_motion_c->peer_motion_db = *p_peer_motion_handles;
    }
    return NRF_SUCCESS;
}
/** @}
 *  @endcond
 */
#endif // NRF_MODULE_ENABLED(BLE_MOTION_C)
