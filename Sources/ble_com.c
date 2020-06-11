/***************************************************************************** 
* File Name:        ble_com.c 
*
* Date Created:     6/8/20
*
* Author:           Michael Anderson
*
* Description:      source file for ble_com.c
*                 
* Function List:  
*                   Public 
*                       ble_com_init()
*                       ble_com_send()
*                       ble_connected(); 
*                       advertising_start()
*                       idle_state_handle()
*
*                   Private 
*                       nus_data_handler() 
*                       gap_params_init() 
*                       nrf_qwr_error_handler() 
*                       services_init() 
*                       on_adv_evt() *
*                       on_conn_params_evt()  
*                       conn_params_error_handler()  
*                       conn_params_init()  
*                       sleep_mode_enter()  
*                       ble_evt_handler()  
*                       ble_stack_init()  
*                       gatt_init() 
*                       bsp_event_handler()
*                       advertising_init()
*                       buttons_leds_init()
*                       log_init()
*                       power_management_init()
*                       nus_event_handler()
*
* Notes:
*****************************************************************************/

#include "ble_com.h"
#include <stdbool.h>
#include "stepperMotor.h"
#include "projectManager.h"
#include "timers.h"
#include "sensor.h"
#include "nrf_assert.h"
#include "eeprom.h"
#include <string.h>
#include <ctype.h>
//private data



BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

static char attribute_messages[4][20];
static bool currently_connected = false;
//functions
bool parseAttributePacket(char * packet, int size);
void nus_event_handler(ble_nus_evt_t * p_evt);


static void advertising_start(bool erase_bonds);

//static pm_evt_id_t bondingLog[100];
//static int bondingLogIndex = 0;


bool ble_connected(void)
{
    return currently_connected;
}
/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;
        case PM_EVT_CONN_SEC_START:
            break;
        case PM_EVT_CONN_SEC_CONFIG_REQ:
            break;
        case PM_EVT_CONN_SEC_PARAMS_REQ:
            break;
        case PM_EVT_CONN_SEC_FAILED:
            break;
        case PM_EVT_BONDED_PEER_CONNECTED:
            break;
        case PM_EVT_CONN_SEC_SUCCEEDED:
            break;
        case PM_EVT_ERROR_UNEXPECTED:
            break;
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            break;
        case PM_EVT_PEER_DATA_UPDATE_FAILED:
            break;
        case PM_EVT_PEER_DELETE_SUCCEEDED:
            break;
        case PM_EVT_PEER_DELETE_FAILED:
            break;
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
            break;
        case PM_EVT_PEERS_DELETE_FAILED:
            break;
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            break;
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
            break;
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
            break;
        case PM_EVT_SLAVE_SECURITY_REQ:
            break;
        case PM_EVT_FLASH_GARBAGE_COLLECTED:
            break;
        case PM_EVT_STORAGE_FULL:
            break;
        case PM_EVT_FLASH_GARBAGE_COLLECTION_FAILED:
            break;
        default:
            // unknown error : p_evt->evt_id
            ASSERT(false);
            break;
    }
//    bondingLog[(bondingLogIndex++) % 99] = p_evt->evt_id;
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    const uint8_t * nameptr = (const uint8_t *) DEVICE_NAME ;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          nameptr,
                                          strlen(nameptr));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.n
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function called after receiving full count of attribute ble messages
 *  will break down each payload and update all attributable variables
 */
void ble_com_process_message_packet(void * p_evt){

    ret_code_t err_code;
    projectManagerSchedulerEventsT *myEvent = p_evt;
    static bool flash_effect = false;
    uint8_t * packetptr = &(myEvent->payload_type.ble_packet[3]); //point past the command type to the actual packet payload
    switch(myEvent->command_type.ble_msg_type)
    {
        case BLE_COM_EVENT_PULL_DATA:
            ble_com_request_message(packetptr, BLE_COM_EVENT_PULL_DATA);
            err_code = NRF_SUCCESS; 
            break;
        case BLE_COM_EVENT_PUSH_DATA:
            // do something with packetptr
            err_code = NRF_SUCCESS;
            break;
        default:
            err_code = NRF_ERROR_NOT_FOUND;
    }
    APP_ERROR_CHECK(err_code);
}


/*****************************************************************************
*
* Function:     ble_com_request_message()
*
* Description:  replies back to Mobile app with requested data
*                                        
* Input:        UInt8_t *dataPtr - Pointer that points to byte #1 of one of 
*                                  the CAN attribute messages.
*               UInt8_t message  - Indicates which attribute message received.
*                                    
* Output:       None
*
* Notes:        
*                      
*****************************************************************************/
void ble_com_request_message(uint8_t *dataPtr, projectManagerBLEMessageEventT message)
{
    ret_code_t err_code;
    int i;
    char response[20];
    uint32_t id;

    switch (message)
    {
        case BLE_COM_EVENT_PULL_DATA:
            sprintf(response,"%s",DEVICE_NAME);
            ble_com_send(response,strlen(response));
            err_code = NRF_SUCCESS;
            break;
        default:
            err_code = NRF_ERROR_NOT_FOUND;
    }
    APP_ERROR_CHECK(err_code);


}

/**@snippet [Handling the data received over BLE] */

/* parseAttributePacket()
 * @brief reads in the attribute packet, validates it, and passes
 * it onto scheduler to handle
 *
 * @param[in] void* p_evt - data packet
 * @param[in] int size - size of packet
 */
static void nus_data_handler(void * p_evt, uint16_t size)
{
    uint8_t* p_data = p_evt;
    uint32_t err_code, tempcolor;
    uint16_t length;
    uint32_t* packetHeader = p_evt; //
    uint32_t  command = 0xFFFFFF & *packetHeader; //mask off the first 3 ascii character values in packet header

    projectManagerSchedulerEventsT myEvent = {PRJ_SCHED_BLE_MESSAGE_EVENT, BLE_COM_EVENT_MAX,0};

    switch(command) //p_data[0]) //(p_evt->params.rx_data.p_data[0])
    {
        case BLE_COM_PULL_DATA:
            myEvent.command_type.ble_msg_type = BLE_COM_EVENT_PULL_DATA;
            break;
        case BLE_COM_PUSH_DATA:
            myEvent.command_type.ble_msg_type = BLE_COM_EVENT_PUSH_DATA;
            break;
        default: //if we get a lot of empty packets ignore them here
            break;
    }

    if( myEvent.command_type.ble_msg_type != BLE_COM_EVENT_MAX) //if we are valid msg_type then add to scheduler
    {
        memcpy(myEvent.payload_type.ble_packet,p_evt,size < SCHEDULER_EVENT_PAYLOAD_SIZE ? size : SCHEDULER_EVENT_PAYLOAD_SIZE);
        app_sched_event_put(&myEvent, sizeof(projectManagerSchedulerEventsT), project_manager_scheduler_handler);
    }

    //Clear out our event from scheduler
    memset(p_data, 0,size);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_event_handler; //Pushes onto Scheduler //nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) 
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)  
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
    //TODO: REMOVE THIS CODE!!!! or have it shutdown BLE service only
    // NOTE: FOR NOW ALL CALLS TO SLEEP_MODE_ENTER HAVE BEEN COMMENTED OUT
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
//            sleep_mode_enter(); //TODO: determine what to do when no advertisement is no longer going on.
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) 
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
//            NRF_LOG_INFO("Connected");
            currently_connected = true;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            project_manager_ble_event(true);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            project_manager_ble_event(false);
            currently_connected = false;
//            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
//            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            //When we get here: structure is Header event ID: 0x13
            // evt -> gap_evt -> params -> sec_info_request -> lesc_dhkey_request (0x01) ; auth status (0x00) sec_request (bond 0x01; mitm 0x01; lesc 0x01; keypress 0x00)
            // Pairing not supported
//            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            // try supporting it
//            ble_gap_sec_params_t mGapSecParams;

//            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_SUCCESS, NULL, NULL);
//            APP_ERROR_CHECK(err_code); //error 0x08 : invalid state.
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            project_manager_ble_event(false);
            currently_connected = false;
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;
    err_code = nrf_sdh_disable_request(); //make sure its not running first
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
//        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
//    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
//                  p_gatt->att_mtu_desired_central,
//                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

//    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
//            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
//            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//            if (err_code != NRF_ERROR_INVALID_STATE)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
//            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
//            {
//                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
//                if (err_code != NRF_ERROR_INVALID_STATE)
//                {
//                    APP_ERROR_CHECK(err_code);
//                }
//            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds) 
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) 
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
void idle_state_handle(void) 
{
//    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

/*
 * nus_event_handler()
 * This guy will just add the nus event onto our scheduler
 */
void nus_event_handler(ble_nus_evt_t * p_evt)
{
    app_sched_event_put(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length, nus_data_handler);
}

/*
 * ble_com_send()
 * sends a string of data over BLE UART service
 */
void ble_com_send(char* message,uint16_t length)
{
    ret_code_t err_code;

    err_code = ble_nus_data_send(&m_nus, message, &length, m_conn_handle);
    APP_ERROR_CHECK(err_code); //WARNING: if we get caught here, com sends are filling up GATT que, need to slow down com sends
}


/*
 * setup all Nordic SDK and custom (when created) 
 * components for the application BLE service
 */
void ble_com_init(void)
{
    bool erase_bonds;

    buttons_leds_init(&erase_bonds);
    bsp_board_leds_off();
    power_management_init();
    
    log_init();

    /*BLE INITIALIZERS*/
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();
    erase_bonds = true;
    advertising_start(erase_bonds);
    /* END */

}