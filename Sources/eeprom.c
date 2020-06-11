/***************************************************************************** 
* File Name:        eeprom.c 
*
* Date Created:     6/8/20
*
* Author:           Michael Anderson
*
* Description:      Functions to handle the EEPROM chip 24LC01 over I2S bus
*                 
* Function List:  
*                   Public
*                       eeprom_busy()
*                       eeprom_init()
*                       eeprom_process_scheduled_packet()
*                       eeprom_read()
*                       eeprom_write()
*                       eeprom_wp_timeout_handler()
*                   Private
*                       twi_event_handler()
*                       twi_master_init()
*
* Notes:
*
*****************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "math.h"
#include "boards.h"
#include "eeprom.h"
#include "nrf_assert.h"
#include "projectManager.h"
#include "timers.h"
#include "nrf_drv_twi.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "app_scheduler.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static uint8_t eeprom_write_package[EEPROM_ADDRESS_LEN_BYTES + EEPROM_24LC01_SEQ_WRITE_MAX_BYTES]; /* Addr + data */ 
static volatile bool eeprom_event_busy_flag;
static volatile bool eeprom_event_read_flag;
static volatile bool eeprom_event_write_flag;
static volatile bool eeprom_test_flag;
/**
 * @brief TWI master instance.
 *
 * Instance of TWI master driver that will be used for communication with simulated
 * EEPROM memory.
 */
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);

/**
 * @brief Tells if EEPROM is busy reading or writing.
 *
 * Function uses the TWI interface to write data into EEPROM memory.
 *
 * @param     none.
 *
 * @return true if busy, false if not.
 *
 * @note  do not put this in a while loop, it breaks handler for some reason will never get called
 */
bool eeprom_busy()
{
    return eeprom_event_busy_flag;
}

/**
 * @brief Write data to simulated EEPROM.
 *
 * Function uses the TWI interface to write data into EEPROM memory.
 *
 * @param     addr  Start address to write.
 * @param[in] pdata Pointer to data to send.
 * @param     size  Byte count of data to send.
 * @attention       Maximum number of bytes that may be written is @ref EEPROM_SIM_SEQ_WRITE_MAX.
 *                  In sequential write, all data must be in the same page
 *                  (higher address bits do not change).
 *
 * @return NRF_SUCCESS or reason of error.
 *
 * @attention If you wish to communicate with real EEPROM memory chip, check its readiness
 * after writing the data.
 */
ret_code_t eeprom_write(uint16_t addr, uint8_t const * pdata, size_t size)
{
    ret_code_t ret;
    projectManagerSchedulerEventsT const newEvent = 
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
                .address = addr,
                .size = size,
                .pdata = (uint8_t *) pdata
            }
         }
    };

    ASSERT(addr <= EEPROM_END_FLASH_ADDRESS);
    
    /* Memory device supports only a limited number of bytes written in sequence */
    if (size > (EEPROM_24LC01_SEQ_WRITE_MAX_BYTES))
    { 
        return NRF_ERROR_INVALID_LENGTH;
    }
    if(eeprom_busy() == true)
    {
        ret = app_sched_event_put(&newEvent, sizeof(projectManagerSchedulerEventsT), project_manager_scheduler_handler);
        APP_ERROR_CHECK(ret);
        return NRF_ERROR_BUSY;
    }
    do
    {
        eeprom_event_busy_flag = true;
        eeprom_event_write_flag = true;
        nrf_gpio_pin_clear(EEPROM_WP_PIN_NUMBER);  // set low to disable Write Protection

        memcpy(eeprom_write_package, &addr, EEPROM_ADDRESS_LEN_BYTES);
        memcpy(eeprom_write_package + EEPROM_ADDRESS_LEN_BYTES, pdata, size);
        ret = nrf_drv_twi_tx(&m_twi_master, EEPROM_CHIP_ADDR, eeprom_write_package, size + EEPROM_ADDRESS_LEN_BYTES, false);
        if (NRF_SUCCESS != ret)
        {
           eeprom_event_busy_flag = false;
           eeprom_event_write_flag = false;
           nrf_gpio_pin_set(EEPROM_WP_PIN_NUMBER);  // set high to enable Write Protection
           break;
        }
    }while (0);
    return ret;
}


/**
 * @brief Read data from simulated EEPROM.
 *
 * Function uses the TWI interface to read data from EEPROM memory.
 *
 * @param     addr  Start address to read.
 * @param[in] pdata Pointer to the buffer to fill with data.
 * @param     size  Byte count of data to read.
 *
 * @return NRF_SUCCESS or reason of error.
 */
ret_code_t eeprom_read(uint16_t addr, uint8_t * pdata, size_t size)
{
    ret_code_t ret;
    int timeout = 20000;
    projectManagerSchedulerEventsT const newEvent = 
    {
        .event_type = PRJ_SCHED_EEPROM_EVENT,
        .command_type =
         {
            .eeprom_event_type = PRJ_MAN_EEPROM_READ_EVENT
         },
         .payload_type =
         {
            .eeprom_data = 
            {
                .address = addr,
                .size = size,
                .pdata = pdata
            }
         }
    };

    ASSERT(addr < EEPROM_END_FLASH_ADDRESS);

    if (size > (EEPROM_24LC01_MEMORY_SIZE - addr))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    if(eeprom_busy())
    {
        ret = app_sched_event_put(&newEvent, sizeof(projectManagerSchedulerEventsT), project_manager_scheduler_handler);
        APP_ERROR_CHECK(ret);
        return NRF_ERROR_BUSY;
    }
    do
    {
       uint16_t addr16 = addr;
       eeprom_event_busy_flag = true;
       eeprom_event_read_flag = true;
       ret = nrf_drv_twi_tx(&m_twi_master, EEPROM_CHIP_ADDR, (uint8_t *)&addr16, EEPROM_ADDRESS_LEN_BYTES, true);
       if (NRF_SUCCESS != ret)
       {
          eeprom_event_busy_flag = false;
          eeprom_event_read_flag = false;
          break;
       }
       while(eeprom_event_read_flag){}; //NOTE: IF we successfully wrote to TWI then we will get an interrupt trigger to reset this flag
       timerStop(TIMER_EEPROM_WP_TIMEOUT);
       eeprom_event_busy_flag = true;
       ret = nrf_drv_twi_rx(&m_twi_master, EEPROM_CHIP_ADDR, pdata, size);
       if (NRF_SUCCESS != ret)
       {
          eeprom_event_busy_flag = false;
          break;
       }
    }while (0);
    return ret;
}

// Called after a write event finishes
//future: implement an EEPROM_READ/WRITE_COMPLETE event that stores a possible call back handler that can be pushed onto scheduler
static void twi_event_handler(nrf_drv_twi_evt_t const * p_event,
                              void *                    p_context)
{
    ASSERT(p_event != NULL);
    ret_code_t result;
    if(eeprom_event_write_flag)
    {
        eeprom_event_write_flag = false;
        project_manager_EEPROM_WP_event();
    }
    else
    {
        eeprom_event_read_flag = false;               // turn off read flag
        eeprom_event_busy_flag = false;               // turn off busy flag
    }
    if (p_event->type == NRF_DRV_TWI_EVT_DONE)
    {
        result = NRF_SUCCESS;
    }
    else
    {
        result = NRF_ERROR_INTERNAL;
    }

}
/**@brief Timer timeout handler to disable write protection on
 *        EEPROM chip once we have finished writing to it
 */
void eeprom_wp_timeout_handler(void * p_context)
{   
    //It takes 5ms for every page write to process by EEPOM, so do not free up our TWI until we are sure its ready
    nrf_gpio_pin_set(EEPROM_WP_PIN_NUMBER);    // set high to enable WP
    eeprom_event_busy_flag = false;            // turn off busy flag
}


/**@brief Handler for processing read or write packets from the scheduler
 *
 * @param[in] p_evt Pointer to EEPROM event and the data payload
 */
void eeprom_process_scheduled_packet(void * p_evt){

    ret_code_t err_code;
    projectManagerSchedulerEventsT *myEvent = p_evt;

    switch(myEvent->command_type.eeprom_event_type)
    {
        case PRJ_MAN_EEPROM_WRITE_EVENT:
            eeprom_write(myEvent->payload_type.eeprom_data.address,
                         myEvent->payload_type.eeprom_data.pdata,
                         myEvent->payload_type.eeprom_data.size);
            break;
        case PRJ_MAN_EEPROM_READ_EVENT:
            eeprom_read(myEvent->payload_type.eeprom_data.address,
                         myEvent->payload_type.eeprom_data.pdata,
                         myEvent->payload_type.eeprom_data.size);
            break;
        default:
            APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
            break;
    }
}
/**
 * @brief Initialize the master TWI.
 *
 * Function used to initialize the master TWI interface that would communicate with simulated EEPROM.
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
static ret_code_t twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    ret = nrf_drv_twi_init(&m_twi_master, &config, twi_event_handler, NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi_master);
    }

    return ret;
}


/**
 * @brief Initialize the EEPROM Chip.
 *
 * Function used to initialize the EEPROM Chip and its PHY interface.
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
bool eeprom_init(void)
{

    uint32_t err_code = NRF_SUCCESS;
    eeprom_event_busy_flag = false;
    NRF_LOG_INFO("TWI init...");

    /* Initializing TWI master interface for EEPROM */
    err_code = twi_master_init();
    APP_ERROR_CHECK(err_code);
    return err_code;
}

//
//void eeprom_flash_test(void)
//{
//    static uint8_t buffer_a[128];
//    static uint8_t buffer_b[128];
//    int i;
//    uint8_t * ptr = buffer_a, addr = 0;
//    eeprom_test_flag = true; //no scheduling of events or any of that for writes and reads
//
//    for(i=0;i<128;i++)
//    {
//        buffer_a[i] = i;
//
//        if((i + 1) % 8 == 0)
//        {
//            eeprom_write(addr,ptr, 8);
//            nrf_delay_ms(6);
//            ptr += 8; //inc our address and buffer to send
//            addr += 8;
//        }
//
//    }
//    ptr = buffer_b;
//    eeprom_read(0,ptr,128);
//    nrf_delay_ms(10); //give some time for the reading
//    i = 0; // catch our break point here
//
//
//}

    

/** @} */