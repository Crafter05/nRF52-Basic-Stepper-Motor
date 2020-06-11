/***************************************************************************** 
* File Name:        sensor.h 
*
* Date Created:     6/8/20
*
* Author:           Michael Anderson				 
*
* Description:      Header for sensor input
*
*****************************************************************************/


#ifndef SENSOR_H
#define SENSOR_H 1

#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"

#define SAADC_SAMPLES_IN_BUFFER     1            //WARNING: the Callback handler will not be entered until the same number of nrf_drv_saadc_sample() are called to match this buffer size
#define SAADC_SINGLE_SAMPLE         1            //WARNING: the Callback handler will not be entered until the same number of nrf_drv_saadc_sample() are called to match this buffer size
#define SAADC_MAX_COUNT             4095

void sensor_saadc_handler(nrf_drv_saadc_evt_t const * p_event);
void sensor_app_timer_read_handler(void * p_context);
void sensor_read(uint16_t val);
bool sensor_init(void);

#endif //SENSOR_H