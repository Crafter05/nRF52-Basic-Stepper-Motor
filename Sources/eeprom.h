/***************************************************************************** 
* File Name:        eeprom.h
*
* Date Created:     6/8/20
*
* Author:           Michael Anderson
*
* Description:      header file for the EEPROM chip 24LC01 over I2C bus
*                   using Nordics TWI driver
*                 
* Function List:  
*                   Public
*                       eeprom_busy()
*                       eeprom_init()
*                       eeprom_process_scheduled_packet()
*                       eeprom_read()
*                       eeprom_write()
*                       eeprom_wp_timeout_handler()
*
* Notes:
*
*****************************************************************************/

#ifndef EEPROM_H
#define EEPROM_H

#include "boards.h"
#include "sdk_errors.h"
#define EEPROM_24LC01_MEMORY_SIZE                   (128u) //!< EEPROM chip size.

/* Maximum number of bytes writable to this slave emulator in one sequential access including
 * address of the slave memory. Maximum allowed is one block = 8 bytes.
 */
#define EEPROM_24LC01_ADDR_BLOCK_SIZE        8
#define EEPROM_24LC01_SEQ_WRITE_MAX_BYTES    EEPROM_24LC01_ADDR_BLOCK_SIZE

#define EEPROM_CHIP_ADDR                   0x50    //!< 24LC01 EEPROM TWI slave address.


/* Flash start address to load the RAM with at startup */
#define EEPROM_START_FLASH_ADDRESS  0x00
#define EEPROM_END_FLASH_ADDRESS    0x7F // 128 Bytes of space ends at address 127

/* Slave memory addressing byte length */
#define EEPROM_ADDRESS_LEN_BYTES    1

/* if EEPROM_ADDRESS_LEN_BYTES == 2, below will configure which byte is sent first by master */
/**
 * @enum address_byte_endian
 * @brief Endianness of the address byte that is received from master.
 */
typedef enum
{
    /*lint -save -e30*/
    BIG_ENDIAN = 0,   /**< MSB is sent first by master for address. */
    LITTLE_ENDIAN,    /**< LSB is sent first by master for address. */
} address_byte_endian;
 
#define TWI_ADDRESS_CONFIG    BIG_ENDIAN //LITTLE_ENDIAN

/* Master Configuration */
#define MASTER_TWI_INST     0       //!< TWI interface used as a master accessing EEPROM memory.
#define TWI_SCL_M           EEPROM_SCL_PIN_NUMBER       //!< Master SCL pin.
#define TWI_SDA_M           EEPROM_SDA_PIN_NUMBER       //!< Master SDA pin.


ret_code_t eeprom_write(uint16_t addr, uint8_t const * pdata, size_t size);
ret_code_t eeprom_read(uint16_t addr, uint8_t * pdata, size_t size);
bool eeprom_init(void);
bool eeprom_busy(void);
void eeprom_wp_timeout_handler(void * p_context);
void eeprom_process_scheduled_packet(void * p_evt);
//void eeprom_flash_test(void); //uncomment this and .c file related code if additional testing is needed
#endif