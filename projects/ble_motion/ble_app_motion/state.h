
#ifndef SDS_COMMAND_H_
#define SDS_COMMAND_H_
#include "imu.h"
#include "app_timer.h"

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(5000)                   /**< Battery level measurement interval (ticks). */
#define ORIENTATION_MEAS_INTERVAL APP_TIMER_TICKS(1000/IMU_SAMPLE_RATE_HZ)          /**< Heart rate measurement interval (ticks). */
#define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(5000)                   /**< Sensor Contact Detected toggle interval (ticks). */


#define DEFAULT_SAMPLE_RATE 			_1_HZ
#define DEFAULT_SAMPLE_DESTINATION		SEND_TO_CENTRAL
#define DEFAULT_IMU_CONFIG				   (ON<<SAMPLE_QUATERNION)\
										|| (OFF<<SAMPLE_ACCEL)\
										|| (OFF<<SAMPLE_GYRO)\
										|| (OFF<<SAMPLE_COMPASS)


typedef enum {
	MIN_VAL = 0x01,
	_1_HZ = MIN_VAL,
	_5_HZ,
	_10_HZ,
	_20_HZ,
	_50_HZ,
	_100_HZ,
	_500_HZ,
	MAX_VAL = _500_HZ,
} SAMPLE_RATE;

typedef enum {
	ON_CONNECT,
	ON_DISCONNECT,
	ON_SLEEP,
} SYSTEM_EVENT;

typedef enum {
	INIT_STATE,
	CONNECTED,
	DISCONNECTED,
	SLEEP,
	ERROR_STATE,
} SYSTEM_STATE;

typedef enum {
	SEND_TO_CENTRAL,
} SAMPLE_DESTINATION;

typedef struct state {
	SYSTEM_STATE system_state;
	SYSTEM_STATE (*event_call)(SYSTEM_EVENT event); 
	SAMPLE_RATE sample_rate;
	SAMPLE_DESTINATION sample_destination;
	SensorConfigReg sensor_config_reg;	
} State;

extern State state;

void system_event_call(SYSTEM_EVENT);
void system_init(void);

#endif
