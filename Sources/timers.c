/***************************************************************************** 
* File Name:        timers.c 
*
* Date Created:     6/8/20
*
* Author:           Michael Anderson
*
* Description:      source file for timers
*                 
* Function List:  
*                   Public
*                       timers_init(void)
*                       system_timer_cc_0_1_init()
*                       timerSetCC0()
*                       timerStart()
*                       timerStop()
*                   Private
*                       system_timer_cc_0_1_handler()
*
* Notes:
*
*****************************************************************************/
#include "timers.h"
#include "sensor.h"
#include "boards.h"
#include "stepperMotor.h"
#include "projectManager.h"
#include "eeprom.h"

#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_ppi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* 
 * create timer instance for any system peripheral driven drivers that need 8MHz timing compare/capture events
 */
static nrf_drv_timer_t system_timer_cc_0_1 = NRF_DRV_TIMER_INSTANCE(1);

/*
 * create a PPI (peripheral to peripheral interface) channel that we can link up 
 * ISR events or tasks to
 */
static nrf_ppi_channel_t ppi_channel[2];


//private functions
void system_timer_cc_0_1_handler(nrf_timer_event_t event_type, void * p_context);


/**@brief Function for initializing the timer module.
 */
void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    //App Timer runs off RTC1 and can be used to make almost any number of 
    //application timers that are not needed for direct peripheral interaction.
    //for example to create an App timer call
    //app_timer_create(); then to start/stop use app_timer_start() app_timer_stop()
    //see app_timer.c/.h for more details
    err_code = app_timer_create(&app_timer_read_sensor, APP_TIMER_MODE_REPEATED, sensor_app_timer_read_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&app_timer_ble_disconnect_timeout, APP_TIMER_MODE_SINGLE_SHOT, ble_disconnect_disconnect_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&app_timer_eeprom_WP_timeout, APP_TIMER_MODE_SINGLE_SHOT, eeprom_wp_timeout_handler);
    APP_ERROR_CHECK(err_code);

    //setup a timer for our Stepper motor
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG; //default settings should use NRF_TIMER_FREQ_8MHz, adjustable in sdk_config.h
//    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&system_timer_cc_0_1, &timer_cfg, system_timer_cc_0_1_handler);
    APP_ERROR_CHECK(err_code);
    //note NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK clears out the current timer base value, good for the initilizer but not the next CC event handler
    nrf_drv_timer_extended_compare(&system_timer_cc_0_1, NRF_TIMER_CC_CHANNEL0, CC_CHANNEL_IDLE_FREQ, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    /* setup CC_1 for compare event every 20ms */
//    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&system_timer_cc_0_1, 20);
    //note NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK clears out the current timer base value, good for the initilizer but not the next CC event handler
//    nrf_drv_timer_extended_compare(&system_timer_cc_0_1, NRF_TIMER_CC_CHANNEL1, CC_CHANNEL_IDLE_FREQ, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, false);


    
    system_timer_cc_0_1_init(); //setup system timer events and tasks
}


/*
 * system_timer_cc_0_1_init()
 * timers_init() MUST be called before this
 * this is a system timer that can have each of its capture/compare channels
 * events linked up to other peripheral events/tasks if needed
 * Current channels used:
 * CC[0]-GPIOTE pin for Stepper Puslse
 * CC[1]-SAADC task sample (starts sampling for all enabled ADC channels)
 * CC[2]- unused
 * CC[3]- unused
 * CC[4]- unused
 * CC[5]- unused
 */
void system_timer_cc_0_1_init(void)
{
    uint32_t compare_evt_addr;
    uint32_t gpiote_task_addr;
    ret_code_t err_code;

    /* CC[0]
     * Setup PPI tasks for GPIOTE and system timer channel[0]
     * for the stepper motor
     */
    nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);

    //enable the pin for GPIOTE
    err_code = nrf_drv_gpiote_out_init(MOTOR_STEP_PIN_NUMBER, &config);
    APP_ERROR_CHECK(err_code);

    //setup a ppi channel to associate with the system_timer_cc_0_1 compare events
    err_code = nrf_drv_ppi_channel_alloc(&(ppi_channel[0]));
    APP_ERROR_CHECK(err_code);

    compare_evt_addr = nrf_drv_timer_event_address_get(&system_timer_cc_0_1, NRF_TIMER_EVENT_COMPARE0);
    nrfx_gpiote_init(); //pin_in_use_clear(MOTOR_STEP_PIN_NUMBER);
    gpiote_task_addr = nrf_drv_gpiote_out_task_addr_get(MOTOR_STEP_PIN_NUMBER);

    err_code = nrf_drv_ppi_channel_assign(ppi_channel[0], compare_evt_addr, gpiote_task_addr);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_enable(ppi_channel[0]);
    APP_ERROR_CHECK(err_code);


    
    /* CC[1]
     * Set PPI tasks for saadc and system timer channel[1]
     * for ADC channel readings
     */
//    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&system_timer_cc_0_1,
//                                                                                NRF_TIMER_CC_CHANNEL1);
//    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();
//
//    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
//    err_code = nrf_drv_ppi_channel_alloc(&(ppi_channel[1]));
//    APP_ERROR_CHECK(err_code);
//
//    err_code = nrf_drv_ppi_channel_assign(ppi_channel[1],
//                                          timer_compare_event_addr,
//                                          saadc_sample_task_addr);
//    APP_ERROR_CHECK(err_code);
//
//    //enable this through BLE commands for now
//    err_code = nrf_drv_ppi_channel_enable(ppi_channel[1]);
//    APP_ERROR_CHECK(err_code);

    //Once all system_timer_cc_0_1 shared peripherals are ready, enable the shared timer
    nrf_drv_timer_enable(&system_timer_cc_0_1);
}





/*
 * system_timer_cc_0_1_handler()
 * cc[0] match event handler on stepper motor timer
 * this is the place to set the next frequency for the stepper motor
 * inputs: event_type - determines what timer channel event this is
 *         p_context - any event details that may be expected
 */
void system_timer_cc_0_1_handler(nrf_timer_event_t event_type, void * p_context)
{
    bool a = false, b = false;
    static uint8_t pulse_polarity = 1; //start high
    switch(event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            pulse_polarity ^= 1;
            motorStateMachine(pulse_polarity); //handle all motor movement
            break;
        case NRF_TIMER_EVENT_COMPARE1: //CC[1] timing interrupt set to false so no trigger events here are necessary
            break;
        case NRF_TIMER_EVENT_COMPARE2: //we might want another channel compare in future
            //call our CC[2] event handler
            break;
        default:
            break;
      }
    
}


void timerSetCC0(uint16_t val)
{
    nrf_drv_timer_extended_compare(&system_timer_cc_0_1, NRF_TIMER_CC_CHANNEL0, val, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}

app_timer_id_t timerGetID(uint32_t id)
{
    app_timer_id_t ret;

    switch(id)
    {
        case TIMER_SENSOR_READ:
            ret = app_timer_read_sensor;
            break;
        case TIMER_BLE_DISCONNECT_TIMEOUT:
            ret = app_timer_ble_disconnect_timeout;
            break;
        case TIMER_EEPROM_WP_TIMEOUT:
            ret = app_timer_eeprom_WP_timeout;
            break;
        default:
            APP_ERROR_CHECK(NRF_ERROR_NOT_FOUND);
            break;
    }
        return ret;
}

void timerStart(uint32_t id, uint32_t ticks) 
{
    if(ticks == 0) //cannot run a zero timer, so simple do a single tick then call event
    {
        ticks++;
    }
    ret_code_t err_code = app_timer_start(timerGetID(id), APP_TIMER_TICKS(ticks), NULL);
    APP_ERROR_CHECK(err_code);
}


void timerStop(uint32_t id)
{
    ret_code_t err_code = app_timer_stop(timerGetID(id));
    APP_ERROR_CHECK(err_code);
}

bool timerRunning(uint32_t id)
{
  return app_timer_running(timerGetID(id));
}