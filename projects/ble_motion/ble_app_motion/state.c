#include "state.h"

#define NRF_LOG_MODULE_NAME state
#define NRF_LOG_LEVEL       3
#define NRF_LOG_INFO_COLOR  0
#define NRF_LOG_DEBUG_COLOR 0
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

#include "main.h"
#include "motion.h"
#include "battery_level.h"


static SYSTEM_STATE event_from_init(SYSTEM_EVENT event);
static SYSTEM_STATE event_from_connected(SYSTEM_EVENT event);
static SYSTEM_STATE event_from_disconnected(SYSTEM_EVENT event);
static SYSTEM_STATE event_from_sleep(SYSTEM_EVENT event);

APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */
APP_TIMER_DEF(m_orientation_timer_id);                         /**< Quaternion Orientation timer. */
APP_TIMER_DEF(m_sensor_contact_timer_id);                           /**< Sensor contact detected timer. */

static void sampling_timers_init(void);
static void sampling_timers_start(void);
static void sampling_timers_stop(void);

State state = 
{
	.system_state = INIT_STATE,
	.event_call = event_from_init,
	.sample_rate = DEFAULT_SAMPLE_RATE,
	.sample_destination = DEFAULT_SAMPLE_DESTINATION,
	.sensor_config_reg = DEFAULT_IMU_CONFIG,
};

void system_init(void) {
	sampling_timers_init();
}

void system_event_call(SYSTEM_EVENT event) {
	state.event_call(event);
}

static SYSTEM_STATE event_from_init(SYSTEM_EVENT event) {
	
	switch (event) {
			
		case ON_CONNECT:
			NRF_LOG_INFO("State Transition: Init -> Connected");
		
			imu_start();
			sampling_timers_start();
		
			state.event_call = event_from_connected;
			return CONNECTED;
		
		case ON_DISCONNECT:
			NRF_LOG_INFO("State Transition: Init -> Disconnected");
		
			imu_stop();
			sampling_timers_stop();
		
			state.event_call = event_from_disconnected;
			return DISCONNECTED;
		
		case ON_SLEEP:
			NRF_LOG_INFO("State Transition: Init -> Sleep");

			imu_stop();
			sampling_timers_stop();		
		
			state.event_call = event_from_sleep;
			return SLEEP;
		
		default:
			NRF_LOG_INFO("State Transition: Init -> Error");
			return state.system_state;
		
	}
}

static SYSTEM_STATE event_from_connected(SYSTEM_EVENT event){
	
	switch (event) {
			

		case ON_DISCONNECT:
			NRF_LOG_INFO("State Transition: Connected -> Disconnected");
			
			imu_stop();
			sampling_timers_stop();
		
			state.event_call = event_from_disconnected;
			return DISCONNECTED;
		
		case ON_SLEEP:
			NRF_LOG_INFO("State Transition: Connected -> Sleep");

			imu_stop();
			sampling_timers_stop();		
		
			state.event_call = event_from_sleep;
			return SLEEP;
		
		default:
			NRF_LOG_INFO("State Transition: Connected -> Error");
	
			return state.system_state;
		
	}
}

static SYSTEM_STATE event_from_disconnected(SYSTEM_EVENT event){
	
	switch (event) {
			
		case ON_CONNECT:
			NRF_LOG_INFO("State Transition: Disconnected -> Connected");
		
			imu_start();
			sampling_timers_start();
		
			state.event_call = event_from_connected;
			return CONNECTED;
		
		case ON_DISCONNECT:
			NRF_LOG_INFO("State Transition: Disconnected -> Disconnected");

			imu_stop();
			sampling_timers_stop();		
		
			state.event_call = event_from_disconnected;
			return DISCONNECTED;
		
		case ON_SLEEP:
			NRF_LOG_INFO("State Transition: Disconnected -> Sleep");

			imu_stop();
			sampling_timers_stop();				
		
			state.event_call = event_from_sleep;
			return SLEEP;
		
		default:
			NRF_LOG_INFO("State Transition: Disconnected -> Error");
			return state.system_state;
		
	}
}

static SYSTEM_STATE event_from_sleep(SYSTEM_EVENT event) {
	
	switch (event) {
			
		case ON_CONNECT:
			NRF_LOG_INFO("State Transition: Sleep -> Connected");

			imu_start();
			sampling_timers_start();			
		
			state.event_call = event_from_connected;
			return CONNECTED;
		
		default:
			NRF_LOG_INFO("State Transition: Sleep -> Error");
			return state.system_state;
		
	}
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void sampling_timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_orientation_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                orientation_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sensor_contact_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_contact_detected_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting application timers.
 */
static void sampling_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_orientation_timer_id, ORIENTATION_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting application timers.
 */
static void sampling_timers_stop(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_stop(m_battery_timer_id);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_stop(m_orientation_timer_id);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_stop(m_sensor_contact_timer_id);
    APP_ERROR_CHECK(err_code);
}

