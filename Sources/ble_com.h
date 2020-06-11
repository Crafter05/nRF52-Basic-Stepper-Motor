/***************************************************************************** 
* File Name:        ble_com.h 
*
* Date Created:     6/8/20
*
* Author:           Michael Anderson				 
*
* Description:      Header for all BLE communication related functions 
*                   and definitions.
*
*
*****************************************************************************/

#ifndef BLE_COM_H
#define BLE_COM_H 1

#include <stdint.h>
#include <string.h>
#include "projectManager.h"
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
#include "app_timer.h"
//#include "fstorage.h"
#include "fds.h"
#include "peer_manager.h"
#include "ble_conn_state.h"
#include "peer_manager_handler.h"
#include "app_pwm.h"
#include "ble_nus.h"
#include "app_scheduler.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_gpiote.h" 
#include "sensorsim.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


//COMMANDS FOR ATTRIBUTE PACKETS
#define BLE_COM_PULL_DATA       0x4C5550 // value for "PUL" NOTE: LSB is the first ASCII Character
#define BLE_COM_PUSH_DATA       0x535550 // value for "PUS"

/*
 * BLE connection and advertisement parameters and variables
 */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "NRF52_EXAMPLE"                             /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                6000                                       /**< The advertising duration (60 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  1                                       /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/*
 * END of BLE variables
 */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define SCHED_MAX_EVENT_DATA_SIZE   sizeof(projectManagerSchedulerEventsT) //MAX(sizeof(ble_nus_evt_t)*2, APP_TIMER_SCHED_EVENT_DATA_SIZE)
#define SCHED_QUEUE_SIZE            20 // Note: this can eat up lots of RAM, limit up to 20 events on our scheduler



#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif



void ble_com_init(void);
void ble_com_send(char* message,uint16_t length);
void BLE_scheduler_event_handler(void * p_evt, uint16_t size);
void idle_state_handle(void);
bool ble_connected(void);
void ble_com_request_message(uint8_t *dataPtr, projectManagerBLEMessageEventT message);
void ble_com_process_message_packet(void * p_evt);
//void advertising_start(void);

#endif //BLE_COM_H