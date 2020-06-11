/***************************************************************************** 
* File Name:        timers.h
*
* Date Created:     6/8/2020
*
* Author:           Michael Anderson
*
* Description:      header file for timers
*                 
*
* Notes:
*****************************************************************************/

#ifndef TIMERS_H
#define TIMERS_H 1

#include "app_timer.h"
#include "nrf_drv_timer.h"

//create app_timer_ID for taking sensor readings
APP_TIMER_DEF(app_timer_read_sensor);
APP_TIMER_DEF(app_timer_ble_disconnect_timeout);
APP_TIMER_DEF(app_timer_eeprom_WP_timeout);

typedef enum{
    TIMER_SENSOR_READ,
    TIMER_BLE_DISCONNECT_TIMEOUT,
    TIMER_EEPROM_WP_TIMEOUT,
    TIMER_MAX_SIZE
} TIMERS_T;

#define APP_TIMER_READ_SENSOR_MS        50      //ticks the timer will count until handler is called
#define APP_TIMER_BLE_TIMEOUT_MS        1000    //ticks till BLE timeouts after connection lost
#define APP_TIMER_EEPROM_TIMEOUT_MS     11      //ticks for EEPROM write available delay
#define APP_TIMER_MS_PER_SEC            1000    //tick conversion for seconds unit into timers required MS units



#define CC_CHANNEL_IDLE_FREQ  0xFFFFFFFF //High frequency value for CC channels that we want to just idle

void timers_init(void);
void system_timer_cc_0_1_init(void);
void timerStart(uint32_t id, uint32_t ticks);
void timerStop(uint32_t id);
bool timerRunning(uint32_t id);
void timerSetCC0(uint16_t val);
app_timer_id_t timerGetID(uint32_t id);

#endif