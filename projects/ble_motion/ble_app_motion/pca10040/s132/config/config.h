
#ifndef CONFIG_H
#define CONFIG_H

//ble config
#define DEV_MODE

//motion config
#define IMU_ENABLED
#define SENSOR_NUM 1

#define IMU_SAMPLE_RATE_HZ 2


#define COMPASS_SAMPLE_RATE_HZ 1
#define MPU_INTERAL_RATE 100
#define DEFAULT_SENSOR_CONFIG	   SENSOR_SAMPLE_QUATERNION \
								| SENSOR_SAMPLE_RAW_IMU \
								| SENSOR_COMPASS \
								| SENSOR_USE_GYRO_CAL


#endif

