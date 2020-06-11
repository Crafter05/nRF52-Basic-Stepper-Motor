/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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

/** @file
 *
 * @defgroup stepper_motor_example main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    main file for stepper motor controller running Nordic Uart BLE service application
 *			 on a NRF52832 SoC
 *
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 *
 */


#include <stdint.h>
#include <string.h>
//includes from Nordic SDK library
#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_delay.h"
//#include "app_timer.h"
#include "app_pwm.h"
#include "ble_nus.h"
#include "app_scheduler.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_assert.h"

//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
//#include "nrf_log_default_backends.h"

//includes for projects source folder
#include "projectManager.h"
#include "eeprom.h"
#include "ble_com.h"
#include "sensor.h"
#include "timers.h"
#include "stepperMotor.h"


static bool project_manager_EEPROM_WP_event_flag = false;       //flag for starting 5 ms EEPROM end timer and any related flash write tasks
static bool BLE_event_connection_flag = false;                  //flag for ble connection/disconnection events
static bool BLE_event_disconnection_flag = false;               //flag for ble connection/disconnection events

void  project_manager_event_handler(void * p_evt);
bool processDemoState(void);
void project_manager_event_flags_check(projectManagerSchedulerEventsT* myEvent);


/***************************************************
 * @brief section for handlers that pop events off *
 * the Nordic app scheduler                        *
 ***************************************************/
void project_manager_scheduler_handler(void * p_evt, uint16_t size )
{
    ret_code_t err_code;
    projectManagerSchedulerEventsT *current_event = p_evt;

    switch(current_event->event_type)
    {
        case PRJ_SCHED_STATE_MANAGER_EVENT:
            project_manager_event_handler(p_evt); 
            err_code = NRF_SUCCESS;  
            break; 
        case PRJ_SCHED_SENSOR_READ_EVENT:
            sensor_app_timer_read_handler(p_evt);
            err_code = NRF_SUCCESS;
            break;
        case PRJ_SCHED_BLE_MESSAGE_EVENT:
            ble_com_process_message_packet(p_evt);
            err_code = NRF_SUCCESS;
            break;
        case PRJ_SCHED_EEPROM_EVENT:
            eeprom_process_scheduled_packet(p_evt);
            err_code = NRF_SUCCESS;
            break;
        default:
            err_code = NRF_ERROR_NOT_FOUND;
            break;
    }
    APP_ERROR_CHECK(err_code);


}
/**@brief Function for handling Queued Write Module errors.n
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
void project_manager_event_flags_check(projectManagerSchedulerEventsT* myEvent)
{

    //after each EEPROM write, start a timer to shut off our write protection
    //any addictional writes will stop this timer
    if(project_manager_EEPROM_WP_event_flag == true)
    {
        project_manager_EEPROM_WP_event_flag = false;
        timerStart(TIMER_EEPROM_WP_TIMEOUT, APP_TIMER_EEPROM_TIMEOUT_MS);

    }
    //check if we have connect to BLE and are not in a critical prj_man_state_type
    else if(BLE_event_connection_flag == true && myEvent->command_type.prj_man_state_type == PRJ_MAN_STATE_IDLE)
    {
        BLE_event_connection_flag = false;
        myEvent->command_type.prj_man_state_type = PRJ_MAN_STATE_BLE_CONNECTED;
    }
}

/**@brief Function for managing the systems main thread state machine
 *
 * @details Handles the state of which the system will be in (startup,flash loading,idle, etc..)
 *
 * @param[in]   p_evt   pointer to the scheduler event.
 */
void  project_manager_event_handler(void * p_evt)
{
    ret_code_t err_code;
    projectManagerSchedulerEventsT *myEvent = p_evt;
    TIMERS_T timer_index;
    //check for any events that need to direct the app state
    project_manager_event_flags_check(myEvent);

    switch (myEvent->command_type.prj_man_state_type){
        case PRJ_MAN_STATE_LOAD_FLASH:
            if(eeprom_busy() == false){
                myEvent->command_type.prj_man_state_type = PRJ_MAN_STATE_STARTUP;
            }
            break;
        case PRJ_MAN_STATE_STARTUP:
            //perform any start up activities here
            motorOn();
            //like validating our data loaded in from flash etc...
            myEvent->command_type.prj_man_state_type = PRJ_MAN_STATE_IDLE;
            break;
        case PRJ_MAN_STATE_IDLE:
            //idle here until something interesting happens
            break;
        case PRJ_MAN_STATE_BLE_CONNECTED:
            //if we ever disconnect we can go back to idle state
            if(BLE_event_disconnection_flag == true){
                BLE_event_disconnection_flag = false;
                myEvent->command_type.prj_man_state_type = PRJ_MAN_STATE_IDLE;
            }
            break;
        case PRJ_MAN_STATE_UPDATING_FLASH:
            // let any EEPROM stuff clear up before the soft reset
            if(eeprom_busy()){
                break;
            }
            if(app_sched_event_on_stack(PRJ_SCHED_EEPROM_EVENT)){
                break;
            }
            //call our read function so we can verify it's saved correctly
            //can place a function here to perform an EEPROM read call to verify our data is written correctly
            //myEvent->command_type.prj_man_state_type = PRJ_MAN_STATE_VERIFYING_FLASH;
            //break;

            //if you do not want to verify flash, proceed with lines below if
            //Choice #1
            myEvent->command_type.prj_man_state_type = PRJ_MAN_STATE_IDLE;
            break;

            //Choice #2
            myEvent->command_type.prj_man_state_type = PRJ_MAN_STATE_SOFT_RESET;
            BLE_event_connection_flag = true; //stay connected after we go through soft reset
            break;
        case PRJ_MAN_STATE_VERIFYING_FLASH:
            ///spin untill we have read in all our stuff
            if(eeprom_busy() || app_sched_event_on_stack(PRJ_SCHED_EEPROM_EVENT)){
                break;
            }
            //call our verify check function
            if(false){ //This would be a handler that follows up the data validation, if it has an error - true, send data again
                //we have an issue, failed updating Flash, try again
                myEvent->command_type.prj_man_state_type = PRJ_MAN_STATE_UPDATING_FLASH;
                break;
            }
            
            myEvent->command_type.prj_man_state_type = PRJ_MAN_STATE_SOFT_RESET;
            BLE_event_connection_flag = true; //stay connected after we go through soft reset
            break;
        case PRJ_MAN_STATE_SOFT_RESET:
            //do a soft reset after we have been programmed or lose BLE connection
            // halt any TIMERS we do not wish to have running
            for(timer_index = TIMER_SENSOR_READ; timer_index < TIMER_MAX_SIZE; timer_index++){
                timerStop(timer_index);
            }
            //go back to our startup state
            myEvent->command_type.prj_man_state_type = PRJ_MAN_STATE_LOAD_FLASH;

            break;
        default:
            break;
    }

    //keep the state machine going forever
    app_sched_event_put(myEvent, sizeof(projectManagerSchedulerEventsT), project_manager_scheduler_handler);

}

/**@brief event call to alert projectManager to activate a timeout for EEPROM write protection.
 *  There may still be more EEPROM writes pending on scheduler that will pause the timeout
 */ 
void project_manager_EEPROM_WP_event(void){
    project_manager_EEPROM_WP_event_flag = true;
}

/**@brief alerts project manager of any BLE state changes
 */
void project_manager_ble_event(bool connected){
    if(connected){
        BLE_event_connection_flag = true;
        //stop the timeout timer from any quick unintentional disconnects
        timerStop(TIMER_BLE_DISCONNECT_TIMEOUT);
    }
    else{ //disconnected
        //start a timeout to let projectManager know we truely are disconnected from BLE service
        timerStart(TIMER_BLE_DISCONNECT_TIMEOUT,APP_TIMER_BLE_TIMEOUT_MS);
    }
}


/**@brief timeout after ble is disconnected (with no auto reconnection happening from app)
 */
void ble_disconnect_disconnect_timeout_handler(void * p_context){
    BLE_event_disconnection_flag = true;

}

/**@brief Setup GPIO pins for NRF52 Controller
 */
static void pinInit(void){
    nrf_gpio_cfg_output(MOTOR_DIR_PIN_NUMBER); //configure this as output pin      
    nrf_gpio_cfg_output(EEPROM_WP_PIN_NUMBER); //configure this as output pin
    nrf_gpio_pin_set(EEPROM_WP_PIN_NUMBER); // set high for WP disable
}

/**@brief Application main function.
 */
int main(void){
    ret_code_t err_code;
    char flashMemoryBuffer[EEPROM_24LC01_MEMORY_SIZE];
    // setup pins we use
    pinInit();

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);


    // Initialize board peripherals, timers etc...
    timers_init();                    // setup our timers
    ble_com_init();            // Setup BLE Service and Protocol

    // prepare our scheduler to handle our application management state machine
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE); //turns on our application scheduler

    projectManagerSchedulerEventsT myEvent = {PRJ_SCHED_STATE_MANAGER_EVENT, PRJ_MAN_STATE_LOAD_FLASH, 0}; 
    app_sched_event_put(&myEvent, sizeof(projectManagerSchedulerEventsT), project_manager_scheduler_handler);
    
    //Start off by reading EEPROM Flash for any saved data
    APP_ERROR_CHECK(eeprom_read(EEPROM_START_FLASH_ADDRESS, flashMemoryBuffer, EEPROM_24LC01_MEMORY_SIZE));

    NRF_LOG_INFO("Stepper Motor App Started....");

    // Enter main loop.
    for (;;){
        app_sched_execute(); // pop any tasks/processess off the scheduler que
        idle_state_handle();
    }
}


/**
 * @}
 */
