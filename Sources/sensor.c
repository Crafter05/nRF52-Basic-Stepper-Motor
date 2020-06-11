/***************************************************************************** 
* File Name:        sensor.c 
*
* Date Created:     6/8/20
*
* Author:           Michael Anderson				 
*
* Description:      source c file for sensor reading
*                 
* Function List:  
*                   Public
*                       sensor_init()
*                       sensor_read()
*                       sensor_saadc_handler()
*                   Private
*
*****************************************************************************/

#include "nrf_delay.h"
#include "sensor.h"
#include "boards.h"
#include "nrf_drv_saadc.h"
#include "ble_com.h"
#include "projectManager.h"
#include "timers.h"
#include "nrfx_saadc.h"
#include "math.h"
#include <stdbool.h>

static nrf_saadc_value_t m_buffer_pool[SAADC_SAMPLES_IN_BUFFER];
static volatile uint16_t sensor_adc_reading;

// This is for tracking SAADC component being used, so we can unintialize only
// the relevant ones when switching from one to another.
#define USED_SAADC(idx) (1UL << idx)
#define SENSOR_INDEX  0
static uint8_t m_saadc_used = 0;    //make sure we are waiting our turn on using adc component




/*
 * sensor_init()
 * initalize the ADC for sensor input reading
 * Note: handler after sample read will unit this
 * return true-  successful
 *        false- saadc component is busy
 */
bool sensor_init(void)
{
    ret_code_t err_code;
    bool ret = true; //for now no issues

    //check if our saadc component is available
    if(m_saadc_used == 0){
        m_saadc_used |= USED_SAADC(SENSOR_INDEX);
        //turn on saadc driver so we can take an adc reading
        err_code = nrf_drv_saadc_init(NULL, sensor_saadc_handler);
        APP_ERROR_CHECK(err_code);

        //setup ADC channel 0: Sensor input
        nrf_saadc_channel_config_t channel_config_0 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
        err_code = nrf_drv_saadc_channel_init(0, &channel_config_0);
        APP_ERROR_CHECK(err_code);
    
        //setup the sampling buffer in RAM to capture our for adc channel sample values
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool, SAADC_SINGLE_SAMPLE);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_saadc_sample();
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        ret = false;
    }

    return ret;
}

/*
 * @function sensor_read()
 * @brief reads the ADC sample value and sends it
 * over ble if connected
 * @param[in] val - raw ADC count value
 */
void sensor_read(uint16_t val)
{
    char response[20];
    
    //range check, this ADC can produce negative values that will be large in an unsigned format
    val = val > SAADC_MAX_COUNT ? 0 : val;

    if(ble_connected())
    {
        //send this to somebody if they are listening
        sprintf(response,"ADC: %4d", val);
        ble_com_send(response,strlen(response));
    }

}




/**********************
 * COMPONENT HANDLERS *
 **********************/


/*
 * Handler for app timer sensor read calls
 */
void sensor_app_timer_read_handler(void * p_context)
{       
    bool ret = sensor_init(); //setup ADC and take a measurement

    if(ret == false)
    {
        //we need to push another sensor_app_timer_read__handler() event onto scheduler
        //if the saadc component is busy and unable to take a reading yet.
        projectManagerSchedulerEventsT myEvent = {PRJ_SCHED_SENSOR_READ_EVENT, 0, 0}; 
        app_sched_event_put(&myEvent, sizeof(projectManagerSchedulerEventsT), project_manager_scheduler_handler);
    }
}

/*
 * @function sensor_saadc_handler()
 * @brief This runs on system_ticker CC[1] compare event that triggers
 * the ADC sample task that then triggers this handler when completed
 */
void sensor_saadc_handler(nrf_drv_saadc_evt_t const * p_event)
{   
    ret_code_t err_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        sensor_adc_reading = p_event->data.done.p_buffer[0];

        //release the saadc component
        nrf_drv_saadc_uninit();
        m_saadc_used &= ~USED_SAADC(SENSOR_INDEX);
        sensor_read(sensor_adc_reading);
    }
}