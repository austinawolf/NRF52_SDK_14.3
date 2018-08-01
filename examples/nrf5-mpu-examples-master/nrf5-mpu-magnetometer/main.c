 /* 
  * This example is not extensively tested and only 
  * meant as a simple explanation and for inspiration. 
  * NO WARRANTY of ANY KIND is provided. 
  */

#include <stdio.h>
#include <string.h>
#include "boards.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "math.h"
#include "app_mpu.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


void mpu_init(void)
{
    ret_code_t ret_code;
    // Initiate MPU driver
    ret_code = app_mpu_init();
    APP_ERROR_CHECK(ret_code); // Check for errors in return value
    
    // Setup and configure the MPU with intial values
    app_mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG(); // Load default values
    p_mpu_config.smplrt_div = 19;   // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 19 gives a sample rate of 50Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G; // Set accelerometer full scale range to 2G
    ret_code = app_mpu_config(&p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(ret_code); // Check for errors in return value 
    

	// Enable magnetometer
	app_mpu_magn_config_t magnetometer_config;
	magnetometer_config.mode = CONTINUOUS_MEASUREMENT_100Hz_MODE;
    ret_code = app_mpu_magnetometer_init(&magnetometer_config);
    APP_ERROR_CHECK(ret_code); // Check for errors in return value
}



/**
 * @brief Function for main application entry.
 */
int main(void)
{    
    uint32_t err_code;
    
    // Initialize.
    log_init();
	NRF_LOG_INFO("\033[2J\033[;H"); // Clear screen
    
    mpu_init();
    
    // Start execution.
    NRF_LOG_INFO("MPU Magnetometer example.");
    
    accel_values_t acc_values;
	magn_values_t magn_values;
    uint32_t sample_number = 0;
    
    const uint8_t MAG_DATA_SIZE = 10;
    uint8_t magn_data[MAG_DATA_SIZE];
    memset(magn_data, 0, MAG_DATA_SIZE);
    
    while(1)
    {
        if(NRF_LOG_PROCESS() == false)
        {
            // Read accelerometer sensor values
            err_code = app_mpu_read_accel(&acc_values);
            APP_ERROR_CHECK(err_code);
            // Clear terminal and print values
            NRF_LOG_INFO("\033[3;1HAccel Sample # %d\r\nX: %06d\r\nY: %06d\r\nZ: %06d\r\n", ++sample_number, acc_values.x, acc_values.y, acc_values.z);
            
            // Read and print magnetometer sensor values
            err_code = app_mpu_read_magnetometer(&magn_values, NULL);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("\n\rMagno Sample # %d\r\nX: %06d\r\nY: %06d\r\nZ: %06d\r\n", sample_number, magn_values.x, magn_values.y, magn_values.z);
     
            nrf_delay_ms(200);
        }
    }
}

/** @} */

