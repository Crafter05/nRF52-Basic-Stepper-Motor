/***************************************************************************** 
* File Name:        projectManager.h 
*
* Date Created:     6/8/20
*
* Author:           Michael Anderson				 
*
* Description:      Header for defines and macros used for the projects main application
*                   manager which is running Nordics event scheduler
*
*****************************************************************************/

#ifndef PROJECT_MANAGER_H
#define PROJECT_MANAGER_H 1

#include <stdbool.h>
#include "sdk_config.h"
#include "nordic_common.h"
#include "nrf.h"


//NOTE: Careful when using these, will stop all event interrupts
#define interrupts() __enable_irq()
#define noInterrupts() __disable_irq()

// Scheduler event types.
typedef enum 
{
    PRJ_SCHED_STATE_MANAGER_EVENT,
    PRJ_SCHED_SENSOR_READ_EVENT,
    PRJ_SCHED_BLE_MESSAGE_EVENT,
    PRJ_SCHED_EEPROM_EVENT
} appEventT;

// Possible application states.
typedef enum 
{
    PRJ_MAN_STATE_STARTUP,
    PRJ_MAN_STATE_IDLE,
    PRJ_MAN_STATE_LOAD_FLASH,
    PRJ_MAN_STATE_UPDATING_FLASH,
    PRJ_MAN_STATE_VERIFYING_FLASH,
    PRJ_MAN_STATE_BLE_CONNECTED,
    PRJ_MAN_STATE_SOFT_RESET
} projectManagerStateT;

typedef enum
{
    PRJ_MAN_EEPROM_WRITE_EVENT,
    PRJ_MAN_EEPROM_READ_EVENT
} projectManagerEepromEventT;

// Possible application events.
typedef enum 
{
    BLE_COM_EVENT_PULL_DATA,
    BLE_COM_EVENT_PUSH_DATA,
    BLE_COM_EVENT_MAX     //always keep this at end
} projectManagerBLEMessageEventT;

typedef struct
{
    uint16_t address;
    uint16_t size;
    uint8_t *pdata;
}eepromDataT;

#define SCHEDULER_EVENT_PAYLOAD_SIZE 40 //make plenty of room for our scheduler to run

typedef struct {
    appEventT event_type;
    union {
        uint16_t default_type;
        projectManagerStateT  prj_man_state_type;
        projectManagerBLEMessageEventT ble_msg_type;
        projectManagerEepromEventT eeprom_event_type;
    }command_type;
    union {
        uint8_t ble_packet[SCHEDULER_EVENT_PAYLOAD_SIZE];
        eepromDataT eeprom_data;
    }payload_type;
} projectManagerSchedulerEventsT;

/****************  TEMPLATE TO BUILDING A SCHEDULER EVENT TO PUT ON STACK **********************************
    //construct our event
    projectManagerSchedulerEventsT const eepromEvent_ = 
    {
        .event_type = PRJ_SCHED_EEPROM_EVENT,
        .command_type =
         {
            .eeprom_event_type = PRJ_MAN_EEPROM_WRITE_EVENT
         },
         .payload_type =
         {
            .eeprom_data = 
            {
                .address = 0,
                .size = 0,
                .pdata = 0
            }
         }
    };
    //push onto stack
    app_sched_event_put(&newEvent, sizeof(projectManagerSchedulerEventsT), project_manager_scheduler_handler);
******************************************************************************************************************/

void project_manager_scheduler_handler(void * p_evt, uint16_t size );
void ble_disconnect_disconnect_timeout_handler(void * p_context);
void project_manager_ble_event(bool connected);
void project_manager_EEPROM_WP_event(void);
//uint8_t hex2int(uint8_t hex1, uint8_t hex2);
#endif //PROJECT_MANAGER_H