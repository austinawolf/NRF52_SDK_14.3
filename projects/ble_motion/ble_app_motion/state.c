#include "state.h"

#define NRF_LOG_MODULE_NAME state
#define NRF_LOG_LEVEL       3
#define NRF_LOG_INFO_COLOR  0
#define NRF_LOG_DEBUG_COLOR 0
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

#include "main.h"
#include "battery_level.h"


static SYSTEM_STATE event_from_init(SYSTEM_EVENT event);
static SYSTEM_STATE event_from_connected(SYSTEM_EVENT event);
static SYSTEM_STATE event_from_disconnected(SYSTEM_EVENT event);
static SYSTEM_STATE event_from_sleep(SYSTEM_EVENT event);

State state = 
{
	.system_state = INIT_STATE,
	.event_call = event_from_init,
	.sample_rate = DEFAULT_SAMPLE_RATE,
	.sample_destination = DEFAULT_SAMPLE_DESTINATION,
};

void system_init(void) {
	
}

void system_event_call(SYSTEM_EVENT event) {
	state.event_call(event);
}

static SYSTEM_STATE event_from_init(SYSTEM_EVENT event) {
	
	switch (event) {
			
		case ON_CONNECT:
			NRF_LOG_INFO("State Transition: Init -> Connected");
			motion_set_sample_state(SAMPLE_ON);			
			state.event_call = event_from_connected;
			return CONNECTED;
		
		case ON_DISCONNECT:
			NRF_LOG_INFO("State Transition: Init -> Disconnected");
		
			motion_set_sample_state(SAMPLE_OFF);
			state.event_call = event_from_disconnected;
			return DISCONNECTED;
		
		case ON_SLEEP:
			NRF_LOG_INFO("State Transition: Init -> Sleep");

			motion_set_sample_state(SAMPLE_OFF);		
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
			
			motion_set_sample_state(SAMPLE_OFF);		
			state.event_call = event_from_disconnected;
			return DISCONNECTED;
		
		case ON_SLEEP:
			NRF_LOG_INFO("State Transition: Connected -> Sleep");

			motion_set_sample_state(SAMPLE_OFF);		
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
				
			state.event_call = event_from_connected;
			return CONNECTED;
		
		case ON_SLEEP:
			NRF_LOG_INFO("State Transition: Disconnected -> Sleep");

			motion_set_sample_state(SAMPLE_OFF);		
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
		
			state.event_call = event_from_connected;
			return CONNECTED;
		
		default:
			NRF_LOG_INFO("State Transition: Sleep -> Error");
			return state.system_state;
		
	}
}

