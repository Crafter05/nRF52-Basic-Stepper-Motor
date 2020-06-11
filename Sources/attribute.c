/***************************************************************************** 
* File Name:        attribute.c 
*
* Date Created:     7/24/19
*
* Author:           Michael Anderosn
*
* Description:      Handles all storing and reading for gauge's attribute object.
* 
* Function List:  
*                   Public
*                       bool attributeInit(void)
*                       bool attributeCheckNvmBlock(void)
*                       void attributeFillRAMCopy(const UInt8_t*, UInt8_t)
*                       void attributeFlashSaveGaugeBlock(void)
*                       void attributeFlashLoadGaugeBlock(void)
*                       void attributeFlashSave(void)
*                       void attributeFlashLoad(void)
*                       void attributeVerifyNewData(void)
*                       bool attributeVerifyNewDataStatus(void)
*                   Private
*                       bool attributeVerifyDeviceSaveResult(void)
*                       void attributeVerifyDeviceSave(void)
*                       bool attributeVerifyFlashSaveResult(void)
*                       void attributeVerifyFlashSave(void)
*
*              
* CSRCS Log
* $Log: attribute.c,v $
* Revision 1.11  2020-03-26 12:02:02-07  manderson
* Add PCA Board ID type definitions,
* Added Sensor Scan and Backlight refresh rates to attributes and system,
* Added sensor connection startup check, motor stays on pointers top if fails.
* Updated Sensor PCA curves.
* Cleaned up code.
* Rolled Revision to 00.002.00
*
* Revision 1.10  2020-03-06 11:48:39-08  manderson
* <>
*
* Revision 1.9  2020-02-18 11:00:47-08  manderson
* Initial Release of EV3 Attribute Programmer.
* Ver: 00.001.00
*
* Revision 1.8  2020-02-06 14:35:49-08  manderson
* Added Toggle preview commands for warning light and flash effect.
* Modified demo mode to extend viewing of main effects
*
* Revision 1.7  2020-01-29 13:28:40-08  manderson
* Fixed Bug Issue with lighting colors not reseting to proper attributed values after disconnecting from mobile app
*
* Revision 1.6  2020-01-24 11:14:57-08  manderson
* Updated Output Driver. Added more sensor Curves. Added EEPROM write verifications. Added warning Flash effect to lighting.c/.h
*
* Revision 1.5  2019-12-20 12:22:23-08  manderson
* Updated for R00.01.00 release
*
* Revision 1.4  2019-12-17 14:44:53-08  manderson
* Updated EEPROM read and writing and BLE commands and gauge curves
*
* Revision 1.3  2019-11-21 14:03:35-08  manderson
* Added NVM support for EEPROM chip. Added Curves.h/.c and few other files for handlling sensor data
*
* Revision 1.2  2019-10-02 14:21:12-07  manderson
* Updated App Scheduler to include attribute messages. Added full support for Attribute programming from Mobile App
*
* Revision 1.1  2019-09-26 14:50:21-07  manderson
* Added 2 demo modes for warn or sweep mode, revised BLE commands,pushed recieved messages onto app scheduler and added handler functions.
*
* Revision 1.0  2019-07-30 12:20:02-07  manderson
* Initial revision
*
*
* Notes:
*
* Copyright (c) 2019  ISSPRO Inc.
********************************************************************************/

#include <string.h>
#include "attribute.h"
#include "projectManager.h"
#include "isspro_ble_com.h"
#include "lighting.h"
#include "eeprom.h"
#include "nrf_gpio.h"
#define BIT_0_MASK       0x01
#define BITS_7_TO_6_MASK 0xC0
#define BITS_7_TO_4_MASK 0xF0
#define BITS_7_TO_1_MASK 0xFE
#define BITS_1_TO_0_MASK 0x03
#define BITS_3_TO_2_MASK 0x0C
#define BITS_5_TO_0_MASK 0x3F

// Private Functions 
static uint8_t nvmComputeCheckSum(const uint8_t *dataPtr, uint8_t byteCount);

char Packets[5][21]; //can hold up to 5 packets with length of 20 characters and 1 null
// Private Data
#define ATTRIBUTE_CLEAR_MASK 0
#define ATTRIBUTE_MSG_1_MASK 0x01
#define ATTRIBUTE_MSG_2_MASK 0x02
#define ATTRIBUTE_MSG_3_MASK 0x04
#define ATTRIBUTE_MSG_4_MASK 0x08
#define ATTRIBUTE_MSG_5_MASK 0x10
#define ATTRIBUTE_MSG_6_MASK 0x20
#define ATTRIBUTE_MSG_FULL_MASK  (ATTRIBUTE_MSG_1_MASK | ATTRIBUTE_MSG_2_MASK | ATTRIBUTE_MSG_3_MASK | ATTRIBUTE_MSG_4_MASK | ATTRIBUTE_MSG_5_MASK | ATTRIBUTE_MSG_6_MASK)

static uint16_t attributeMessagesReceived;
//#define CUSTOM_GAUGE_SETTING
#ifdef CUSTOM_GAUGE_SETTING
// Store custom gauge data into ram (defined in attributes.h)
static Flash_Memory nvm_block_verifier, NVMBlock = {
                                ATT_GAUGE_FUEL_PRESS_100_100,    //gauge type
                                0,    //home
                                30,  //full
                                135,  //sweep
                                0,    //valid low
                                50,  //valid max
                                10,    //sensor curve
                                200,    //pointer weight
                                0,    //Hysteresis
                                20,   //sensorScanRate
                                0x3f8ccccd,    //coeffecient0 :  1.1
                                0xbcf5c28f,    //coeffecient1 : -0.03
                                0,    //modeTypeFlash // 16, 32, 48 - enabled test, sweep, warn modes
                                101,  //backlightFlash: 50% hi zone
                                181,  //backlightTopV: 18.1V
                                78,   //backlightTopV: 7.8V
                                0,    //backlightRed
                                0,    //backlightGreen
                                255,  //backlightBlue
                                0,    //backlightWhite
                                0,    //pointerRed
                                0,    //pointerGreen
                                0,    //pointerBlue
                                255,  //pointerxWhite
                                41,   //warningLightBot: 20% zone hi
                                160,  //warningLightTop: 80% zone low
                                0,    //outputDriverBot
                                0, //101,  //outputDriverTop: 50% zone 1
                                64,   //outputTiming 15 sec startup delay
                                200,  //backlight refresh in msec time
                                0,    //empty filler
                                0,    //empty filler
                                0,    //CHECKSUM_VALUE 
                                };
#else
// RAM copy of what is stored in NVM (will get filled on init).
static Flash_Memory NVMBlock, nvm_block_verifier;
#endif
static Flash_Memory_Device NVMDevice, nvm_device_verifier;
static bool attributes_invalid;
static bool attributes_factory_save = false;
static bool attributes_new_block_save_flag = false;
static bool attributes_new_device_save_flag = false;

static bool attributeVerifyDeviceSaveResult(void);
static void attributeVerifyDeviceSave(void);
static bool attributeVerifyFlashSaveResult(void);
static void attributeVerifyFlashSave(void);

/*****************************************************************************
*
* Function:     attributeInit()
*
* Description:  Checks NVM and initializes the attribute settings.
*
* Input:        None
*
* Output:       FALSE - Attributes set and valid in NVM block.
*               TRUE  - Attributes not set and/or not valid in NVM block.
*
* Notes:        After a successful attributeInit(), the RAM and NVM attribute 
*               values will be the same and both should have a valid checksum.
*
*               The caller needs to verify the output of this function
*               and determine what action to occur if error detected.
*
*****************************************************************************/
bool attributeInit(void)
{
    attributes_invalid = false; //assume data is good at first
    // Check NVM to verify that the attribute values are valid.
    if (attributeCheckNvmBlock())
    {
#ifdef CUSTOM_GAUGE_SETTING
        attributeFlashSave();
#else
        attributes_invalid = true;          // NVM data not valid - error
#endif
        //pfMarkLatch(TRUE);
    }
   
    return attributes_invalid;
}


/*****************************************************************************
*
* Function:     attributeCheckStatus()
*
* Description:  Tells other libraries if attributes are invalid or not
*
* Input:        None
*
* Output:       FALSE - Attributes set and good
*               TRUE  - Attributes not set and/or not valid in NVM block.
*
* Notes:        After a successful attributeInit(), the RAM and NVM attribute 
*               values will be the same and both should have a valid checksum.
*
*               The caller needs to verify the output of this function
*               and determine what action to occur if error detected.
*
*****************************************************************************/
bool attributeCheckStatus(void)
{
    return attributes_invalid;
}

void attributeStorePacket(char* packet, int index)
{
    index = index >= 0 && index <=4 ? index : 0; //range check
    memcpy(&Packets[index],packet,20);
}
/*****************************************************************************
*
* Function:     attributeCheckNvmBlock()
*
* Description:  Checks if attributes found in NVM Flash are Ok            
*
* Input:        None
*
* Output:       FALSE - No Error Detected while checking the NVM Block
*               TRUE  - Failure Detected while checking the NVM Block.
*
* Notes:        Compute Checksum from the attributes stored in NVM Block
*               and compares against Checksum stored in NVM Block.    
*           
*****************************************************************************/
bool attributeCheckNvmBlock(void)
{
    // Including the checksum in the calculation should produce a zero result.
    if (nvmComputeCheckSum((uint8_t *) &NVMBlock, NUM_ATTRIBUTES_BYTES) == 0 /*&& NVMBlock.checkSum != 0*/)    
    {   
        return (false);             // Ok - No Error Detected
    }
    else
    {    
        return (true);              // Not Ok - Error Detected 
    }
}


/**
 * hex2int
 * take a hex string and convert it to a 32bit number (max 8 hex digits)
 */
uint8_t hex2int(uint8_t hex1, uint8_t hex2) 
{
    uint8_t byte;
    
    if (hex1 >= '0' && hex1 <= '9')
    {
        hex1 = hex1 - '0';
    }
    else if (hex1 >= 'a' && hex1 <='f')
    {
        hex1 = hex1 - 'a' + 10;
    }
    else if (hex1 >= 'A' && hex1 <='F')
    {
        hex1 = hex1 - 'A' + 10;
    }

    if (hex2 >= '0' && hex2 <= '9')
    {
        hex2 = hex2 - '0';
    }
    else if (hex2 >= 'a' && hex2 <='f')
    {
        hex2 = hex2 - 'a' + 10;
    }
    else if (hex2 >= 'A' && hex2 <='F')
    {
        hex2 = hex2 - 'A' + 10;
    }
    
    byte =  (uint8_t) ((hex1 << 4) | (hex2 & 0xF));
    return byte;
}


/*****************************************************************************
*
* Function:     attributeFillGaugeNVMBlock()
*
* Description:  Stores received configuration data into the RAM copy of
*               NVM (NVMBlock) in preparation for storing into NVM.
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
void attributeFillGaugeNVMBlock(uint8_t *dataPtr, appBLEMessageEventT message)
{
    ret_code_t err_code = NRF_SUCCESS;
    int i;
    uint8_t  packetBytes[8];

    //NVMDevice
    switch (message)
    {
        case APP_BLE_NAME_CHANGE_EVENT:
            //NOTE: this will fully trust AP always passes correct characters and length to the gauge
            dataPtr++; //increment our pointer to skip the space before ascii characters
            memcpy(NVMDevice.device_name,dataPtr,DEVICE_NAME_SIZE);
            attributeFlashSaveGaugeBlock();
            break;
        case APP_BLE_PRG_SN_EVENT:
            //convert our ASCII characters into integers
            for(i=0;i<8;i++)
            {
                packetBytes[i] = hex2int(dataPtr[2*i], dataPtr[(2*i)+1]);
            }
            NVMDevice.device_SN = (uint32_t) (((packetBytes[0] << 24) | (packetBytes[1] << 16)  | (packetBytes[2] << 16) | packetBytes[3] ));
            //NVMBlock.coeffecient1 = (uint32_t) (((packetBytes[4] << 24) | (packetBytes[5] << 16)  | (packetBytes[6] << 16) | packetBytes[7] ));
            attributeFlashSaveGaugeBlock();
            break;
         case APP_BLE_PRG_ATTRS_EVENT:
            //setup a flag so that attributes loading in will not be stored in Custome Slot
            attributes_factory_save = true;
            //do not break here, we also have to garenteer our attribute_slot is set to false
         case APP_BLE_REQ_FACT_RESET_EVENT:
            NVMDevice.attribute_slot = false; //use slot for Factory attributes
            attributeFlashSaveGaugeBlock();
            
            // Attributes will be reloaded in softreset state in main.c 
            app_manager_attribute_programmed_event();
            break;
        default:
            err_code = NRF_ERROR_NOT_FOUND;
    }
    APP_ERROR_CHECK(err_code);

}
/*****************************************************************************
*
* Function:     attributeFillRAMCopy()
*
* Description:  Stores received configuration data into the RAM copy of
*               NVM (NVMBlock) in preparation for storing into NVM.
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
void attributeFillRAMCopy(uint8_t *dataPtr, appBLEMessageEventT message)
{
    ret_code_t err_code;
    int i;
    bool update_check;
    uint8_t  packetBytes[8];
    RGB_ColorT c1, c2;
    //convert our ASCII characters into integers
    for(i=0;i<8;i++)
    {
        packetBytes[i] = hex2int(dataPtr[2*i], dataPtr[(2*i)+1]);
    }
    // NOTE: SCHEDULER CAN SCRAMBLE MESSAGE ORDER UP, attributeMessagesReceived will monitor this
    switch (message)
    {
        case APP_BLE_ATTR_0_MSG_EVENT:
            c1.red    = packetBytes[0];
            c1.green  = packetBytes[1];
            c1.blue   = packetBytes[2];
            c1.white  = packetBytes[3];
            c2.red    = packetBytes[4];
            c2.green  = packetBytes[5];
            c2.blue   = packetBytes[6];
            c2.white  = packetBytes[7];
            lightingSetTestColor(c1,c2);
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_ATTR_1_MSG_EVENT:
            // First attribute message received.
            NVMBlock.gaugeType = (uint16_t) (((packetBytes[0] << 8) | packetBytes[1])); //packetBytes[0];
            NVMBlock.gaugeHome = (uint16_t) (((packetBytes[2] << 8) | packetBytes[3]));
            NVMBlock.gaugeFull = (uint16_t) (((packetBytes[4] << 8) | packetBytes[5]));
            NVMBlock.minValidReading = (uint16_t) (((packetBytes[6] << 8) | packetBytes[7]));

            attributeMessagesReceived |= ATTRIBUTE_MSG_1_MASK;
            err_code = NRF_SUCCESS;  

            break;

        case APP_BLE_ATTR_2_MSG_EVENT:
            // Second attribute message received.
            NVMBlock.maxValidReading = (uint16_t) (((packetBytes[0] << 8) | packetBytes[1]));
            NVMBlock.sensorCurve = packetBytes[2];
            NVMBlock.pointerWeight = (uint16_t) (((packetBytes[3] << 8) | packetBytes[4]));
            NVMBlock.hysteresis = packetBytes[5];
            NVMBlock.sensorScanRate = packetBytes[6];
            NVMBlock.gaugeSweep = packetBytes[7];

            attributeMessagesReceived |= ATTRIBUTE_MSG_2_MASK;
            err_code = NRF_SUCCESS;  

            break;

        case APP_BLE_ATTR_3_MSG_EVENT:
            // Third attribute message received.
            NVMBlock.modeTypeFlash = packetBytes[0];
            NVMBlock.backlightFlash = packetBytes[1];
            NVMBlock.backlightTopV = (uint16_t) (((packetBytes[2] << 8) | packetBytes[3]));
            NVMBlock.backlightBotV = (uint16_t) (((packetBytes[4] << 8) | packetBytes[5]));
            NVMBlock.warningLightBot = packetBytes[6];
            NVMBlock.warningLightTop = packetBytes[7];

            attributeMessagesReceived |= ATTRIBUTE_MSG_3_MASK;
            err_code = NRF_SUCCESS;  
            break;

        case APP_BLE_ATTR_4_MSG_EVENT:
            // Fourth attribute message received.
            NVMBlock.backlightRed = packetBytes[0];
            NVMBlock.backlightGreen = packetBytes[1];
            NVMBlock.backlightBlue = packetBytes[2];
            NVMBlock.backlightWhite = packetBytes[3];
            NVMBlock.pointerRed = packetBytes[4];
            NVMBlock.pointerGreen = packetBytes[5];
            NVMBlock.pointerBlue = packetBytes[6];
            NVMBlock.pointerWhite = packetBytes[7];

            attributeMessagesReceived |= ATTRIBUTE_MSG_4_MASK;
            err_code = NRF_SUCCESS;  
            break;

        case APP_BLE_ATTR_5_MSG_EVENT:
            // Fifth attribute message received.
            NVMBlock.coeffecient0 = (uint32_t) (((packetBytes[0] << 24) | (packetBytes[1] << 16)  | (packetBytes[2] << 8) | packetBytes[3] ));
            NVMBlock.coeffecient1 = (uint32_t) (((packetBytes[4] << 24) | (packetBytes[5] << 16)  | (packetBytes[6] << 8) | packetBytes[7] ));

            attributeMessagesReceived |= ATTRIBUTE_MSG_5_MASK;
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_ATTR_6_MSG_EVENT:
            NVMBlock.outputTiming = packetBytes[0];
            NVMBlock.outputDriverBot = packetBytes[1];
            NVMBlock.outputDriverTop = packetBytes[2];
            NVMBlock.backlightScanRate = packetBytes[3];

            attributeMessagesReceived |= ATTRIBUTE_MSG_6_MASK;
            err_code = NRF_SUCCESS;  
            break;
        default:
            err_code = NRF_ERROR_NOT_FOUND;
    }
    if(attributeMessagesReceived == ATTRIBUTE_MSG_FULL_MASK )
    {
        attributeMessagesReceived = ATTRIBUTE_CLEAR_MASK;
        update_check = NVMDevice.attribute_slot;
        //Update device Info to ensure we load now from correct attribute slot
        if(attributes_factory_save == false){
            NVMDevice.attribute_slot = true; //use slot for customer attribute programmer
        }
        else{
            attributes_factory_save = false;
        }

        //Lets store our Attributes in Flash now
        attributeFlashSave();
        if(update_check != NVMDevice.attribute_slot)
        {
            attributeFlashSaveGaugeBlock();
        }
        app_manager_attribute_programmed_event(); //will cause a reset and load in new attributes now that we received final message.
    }

    APP_ERROR_CHECK(err_code);
}

/**@brief Function called after receiving full count of attribute ble messages
 *  will break down each payload and update all attributable variables
 */
void attribute_process_message_packet(void * p_evt){

    ret_code_t err_code;
    customSchedulerEventsT *myEvent = p_evt;
    static bool flash_effect = false;
    uint8_t * packetptr = &(myEvent->payload_type.ble_packet[3]); //skip the header that got us here
    switch(myEvent->command_type.ble_msg_type)
    {
        case APP_BLE_ATTR_0_MSG_EVENT:
            attributeFillRAMCopy(packetptr, APP_BLE_ATTR_0_MSG_EVENT); //store our attributes
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_ATTR_1_MSG_EVENT:
            attributeFillRAMCopy(packetptr, APP_BLE_ATTR_1_MSG_EVENT); //store our attributes
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_ATTR_2_MSG_EVENT:
            attributeFillRAMCopy(packetptr, APP_BLE_ATTR_2_MSG_EVENT); //store our attributes
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_ATTR_3_MSG_EVENT:
            attributeFillRAMCopy(packetptr, APP_BLE_ATTR_3_MSG_EVENT); //store our attributes
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_ATTR_4_MSG_EVENT:
            attributeFillRAMCopy(packetptr, APP_BLE_ATTR_4_MSG_EVENT); //store our attributes
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_ATTR_5_MSG_EVENT:
            attributeFillRAMCopy(packetptr, APP_BLE_ATTR_5_MSG_EVENT); //store our attributes
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_ATTR_6_MSG_EVENT:
            attributeFillRAMCopy(packetptr, APP_BLE_ATTR_6_MSG_EVENT); //store our attributes
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_NAME_CHANGE_EVENT:
            attributeFillGaugeNVMBlock(packetptr, APP_BLE_NAME_CHANGE_EVENT);
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_PRG_ATTRS_EVENT:
            attributeFillGaugeNVMBlock(packetptr, APP_BLE_PRG_ATTRS_EVENT);
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_PRG_SN_EVENT:
            attributeFillGaugeNVMBlock(packetptr, APP_BLE_PRG_SN_EVENT);
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_REQ_WARN_FLASH_TOGGLE_EVENT:
            flash_effect = flash_effect ? false : true;
            lighting_warning_light_flash_set(flash_effect);
            break;
        case APP_BLE_REQ_WARN_TOGGLE_EVENT:
            nrf_gpio_pin_toggle(WARN_LIGHT_PIN_NUMBER);
            break;
        case APP_BLE_REQ_ATTRS_EVENT:
            isspro_ble_com_request_message(packetptr, APP_BLE_REQ_ATTRS_EVENT);
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_REQ_SN_BIN_EVENT:
            isspro_ble_com_request_message(packetptr, APP_BLE_REQ_SN_BIN_EVENT);
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_REQ_FACT_RESET_EVENT:
            attributeFillGaugeNVMBlock(packetptr, APP_BLE_REQ_FACT_RESET_EVENT);
            err_code = NRF_SUCCESS;  
            break;
        case APP_BLE_REQ_REV_EVENT:
            isspro_ble_com_request_message(packetptr, APP_BLE_REQ_REV_EVENT);
            err_code = NRF_SUCCESS;  
            break;
        default:
            err_code = NRF_ERROR_NOT_FOUND;
    }
    APP_ERROR_CHECK(err_code);
}

/*****************************************************************************
*
* Function:     attributeReadGaugeSelect()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
AttGaugeTypeT attributeReadGaugeSelect(void)
{
    return (AttGaugeTypeT)(NVMBlock.gaugeType);
}

/*****************************************************************************
*
* Function:     attributeReadGaugeHome()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint16_t attributeReadGaugeHome(void)
{
    return NVMBlock.gaugeHome;
}

/*****************************************************************************
*
* Function:     attributeReadGaugeFull()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint16_t attributeReadGaugeFull(void)
{
    return NVMBlock.gaugeFull;
}

/*****************************************************************************
*
* Function:     attributeReadGaugeSweep()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadGaugeSweep(void)
{
    return NVMBlock.gaugeSweep;
}

/*****************************************************************************
*
* Function:     attributeReadGaugeMin()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
int16_t attributeReadGaugeMin(void)
{
    return (int16_t) NVMBlock.minValidReading;
}

/*****************************************************************************
*
* Function:     attributeReadGaugeMax()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
int16_t attributeReadGaugeMax(void)
{
    return (int16_t) NVMBlock.maxValidReading;
}

/*****************************************************************************
*
* Function:     attributeReadGaugeMax()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadSensorCurve(void)
{
    return (uint8_t) NVMBlock.sensorCurve;
}

/*****************************************************************************
*
* Function:     attributeReadPointerWeight()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint16_t attributeReadPointerWeight(void)
{
    return (uint16_t) NVMBlock.pointerWeight;
}

/*****************************************************************************
*
* Function:     attributeReadGaugeHysteresis()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadGaugeHysteresis(void)
{
    return (uint8_t) NVMBlock.hysteresis;

}

/*****************************************************************************
*
* Function:     attributeReadSensorScanRate()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadSensorScanRate(void)
{
    return (uint8_t) NVMBlock.sensorScanRate;
}

/*****************************************************************************
*
* Function:     attributeReadSensorCoeff0()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
float attributeReadSensorCoeff0(void)
{
    return (float) NVMBlock.coeffecient0;
}

/*****************************************************************************
*
* Function:     attributeReadSensorCoeff1()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
float attributeReadSensorCoeff1(void)
{
    return (float) NVMBlock.coeffecient1;
}

/*****************************************************************************
*
* Function:     attributeGetCoeffPtr()
*
* Description:  Returns pointer to coefficient attribute data in NVM Flash.
*                           
* Input:        None
*                           
* Output:       float* - Returns pointer to coefficient data in NVM Flash.
*                    
* Notes:        
* 
*****************************************************************************/
float * attributeGetCoeffPtr(void)
{
    return (float *)(&NVMBlock.coeffecient0); 
}

/*****************************************************************************
*
* Function:     attributeGetDeviceNamePtr()
*
* Description:  Returns pointer to devices name that BLE advertises.
*                           
* Input:        None
*                           
* Output:       char* - Returns pointer to coefficient data in NVM Flash.
*                    
* Notes:        
* 
*****************************************************************************/
char * attributeGetDeviceNamePtr(void)
{
    static char Gauge_Name[13];
    memcpy(Gauge_Name,NVMDevice.device_name,12);
    Gauge_Name[12] = 0; //null character for end of string
    return (char *)(&Gauge_Name); 
}

/*****************************************************************************
*
* Function:     attributeReadGaugeOperationMode()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       AttGaugeModeT - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
AttGaugeModeT attributeReadGaugeOperationMode(void)
{
    return (AttGaugeModeT) ((NVMBlock.modeTypeFlash & BITS_7_TO_4_MASK) >> 4);
}

/*****************************************************************************
*
* Function:     attributeReadPointerType()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       AttPointerStyleT - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
AttPointerStyleT attributeReadPointerType(void)
{
    return (AttPointerStyleT) ((NVMBlock.modeTypeFlash & BITS_3_TO_2_MASK) >> 2);
}

/*****************************************************************************
*
* Function:     attributeReadBacklightFlashIntensity()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadBacklightFlashIntensity(void)
{
    return (uint8_t) (NVMBlock.modeTypeFlash & BITS_1_TO_0_MASK);
}

/*****************************************************************************
*
* Function:     attributeReadBacklightFlashThreshold()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadBacklightFlashThreshold(void)
{
    uint8_t percentLevel;               // 0 - 100 %

    percentLevel = (uint8_t)((NVMBlock.backlightFlash & BITS_7_TO_1_MASK) >> 1);
//    assert(percentLevel <= 100);
    
    return percentLevel;
}

/*****************************************************************************
*
* Function:     attributeReadBacklightFlashZone()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadBacklightFlashZone(void)
{
    return (uint8_t) (NVMBlock.backlightFlash & BIT_0_MASK);
}

/*****************************************************************************
*
* Function:     attributeReadBacklightBottomVoltage()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
float attributeReadBacklightBottomVoltage(void)
{
    return (float) (NVMBlock.backlightBotV * BACKLIGHT_V_SCALAR);
}

/*****************************************************************************
*
* Function:     attributeReadBacklightTopVoltage()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
float attributeReadBacklightTopVoltage(void)
{
    return (float) (NVMBlock.backlightTopV * BACKLIGHT_V_SCALAR);
}

/*****************************************************************************
*
* Function:     attributeReadBacklightRGBW()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       RGB_ColorT - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
RGB_ColorT attributeReadBacklightRGBW(void)
{
    RGB_ColorT ret = {NVMBlock.backlightWhite,NVMBlock.backlightBlue,NVMBlock.backlightRed,NVMBlock.backlightGreen};
    return ret;
}

/*****************************************************************************
*
* Function:     attributeReadPointerRGBW()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       RGB_ColorT - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
RGB_ColorT attributeReadPointerRGBW(void)
{
    RGB_ColorT ret = {NVMBlock.pointerWhite,NVMBlock.pointerBlue,NVMBlock.pointerRed,NVMBlock.pointerGreen}; 
    return ret;
}

/*****************************************************************************
*
* Function:     attributeReadBottomWarningLightThreshold()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadBottomWarningLightThreshold(void)
{
    uint8_t percentLevel;               // 0 - 100 %

    percentLevel = (uint8_t)((NVMBlock.warningLightBot & BITS_7_TO_1_MASK) >> 1);
//    assert(percentLevel <= 100);
    
    return percentLevel;
}

/*****************************************************************************
*
* Function:     attributeReadBottomWarningLightZone()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadBottomWarningLightZone(void)
{
    return (uint8_t)(NVMBlock.warningLightBot & BIT_0_MASK);
}

/*****************************************************************************
*
* Function:     attributeReadTopWarningLightThreshold()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadTopWarningLightThreshold(void)
{
    uint8_t percentLevel;               // 0 - 100 %

    percentLevel = (uint8_t)((NVMBlock.warningLightTop & BITS_7_TO_1_MASK) >> 1);
//    assert(percentLevel <= 100);
    
    return percentLevel;
}

/*****************************************************************************
*
* Function:     attributeReadTopWarningLightZone()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadTopWarningLightZone(void)
{
    return (uint8_t)(NVMBlock.warningLightTop & BIT_0_MASK);
}

/*****************************************************************************
*
* Function:     attributeReadBottomOutputDriverThreshold()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadBottomOutputDriverThreshold(void)
{
    uint8_t percentLevel;               // 0 - 100 %

    percentLevel = (uint8_t)((NVMBlock.outputDriverBot & BITS_7_TO_1_MASK) >> 1);
//    assert(percentLevel <= 100);
    
    return percentLevel;
}

/*****************************************************************************
*
* Function:     attributeReadBottomOutputDrivertZone()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadBottomOutputDrivertZone(void)
{
    return (uint8_t)(NVMBlock.outputDriverBot & BIT_0_MASK);
}

/*****************************************************************************
*
* Function:     attributeReadTopOutputDriverThreshold()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadTopOutputDriverThreshold(void)
{
    uint8_t percentLevel;               // 0 - 100 %

    percentLevel = (uint8_t)((NVMBlock.outputDriverTop & BITS_7_TO_1_MASK) >> 1);
//    assert(percentLevel <= 100);
    
    return percentLevel;
}

/*****************************************************************************
*
* Function:     attributeReadTopOutputDriverZone()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadTopOutputDriverZone(void)
{
    return (uint8_t)(NVMBlock.outputDriverTop & BIT_0_MASK);
}

/*****************************************************************************
*
* Function:     attributeReadOutputDriverStartupDelay()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadOutputDriverStartupDelay(void)
{
    return (uint8_t)((NVMBlock.outputTiming & BITS_7_TO_6_MASK) >> 6);
}

/*****************************************************************************
*
* Function:     attributeReadOutputDriverActivationDelay()
*
* Description:  Returns indicated attribute value from NVM Flash block.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint8_t attributeReadOutputDriverActivationDelay(void)
{
    return (uint8_t)(NVMBlock.outputTiming & BITS_5_TO_0_MASK);
}

/*****************************************************************************
*
* Function:     attributeGetDeviceSN()
*
* Description:  Returns indicated attribute SN form NVMDevice memory.
*                           
* Output:       UIntX_t - attribute variable value
*                    
* Notes:        
* 
*****************************************************************************/
uint32_t attributeGetDeviceSN(void)
{
    return NVMDevice.device_SN;
}

/*****************************************************************************
*
* Function:     nvmComputeCheckSum()
*
* Description:  Computes checksum value for attribute table in NVM.
*              
* Input:        UInt8_t* dataPtr - pointer to attribute data to checksum
*               UInt8_t  byteCount - length in bytes to checksum
*
* Output:       UInt8_t - checksum calculated from data in NVM.
*
* Notes:        If the byteCount (length) includes the checksum value, 
*               then the output should be 0 for a valid set of values.
*
*****************************************************************************/
static uint8_t nvmComputeCheckSum(const uint8_t *dataPtr, uint8_t byteCount)
{
    uint8_t  i;
    uint16_t checksum = 0;          // Reset checksum value

    for (i = 0; i < byteCount; i++)
    {
        checksum += *dataPtr++;     // Add NVM data, minus checksum
    }

    checksum ^= 0xFFFF;
    ++checksum;                     // Perform 2's compliment

    checksum &= 0xFF;               // Remove upper byte value
  
    return ((uint8_t) checksum);
}


void attributeModeDemoInc(void)
{
    static uint8_t counter = 0;
    switch(attributeReadGaugeOperationMode())
    {
        case ATT_GAUGE_MODE_TEST:
            NVMBlock.modeTypeFlash = 32;
            break;
        case ATT_GAUGE_MODE_SWEEP_DEMO:
            NVMBlock.modeTypeFlash = 48;
            break;
        case ATT_GAUGE_MODE_WARN_DEMO:
        default:
            NVMBlock.modeTypeFlash = (counter++ & 0x1) ? 16 : 32; //skip warn demo every other time, it nothing special16;
            break;
    }
}

/*****************************************************************************
*
* Function:     attributeFlashLoad()
*
* Description:  Computes checksum value for attribute table in NVM.
*              
* Input:
*
* Output:       bool true - failure, false - success.
*
* Notes:        
*
*****************************************************************************/
bool attributeFlashLoad(void)
{
    uint8_t const addr = (NVMDevice.attribute_slot == false) ? ATT_FACTORY_FLASH_LOCATION : ATT_AP_SLOT_FLASH_LOCATION;
    bool ret = false;
    uint8_t * pdata = (uint8_t *) &NVMBlock;
    eeprom_read(addr, pdata, NUM_ATTRIBUTES_BYTES);
    //read will complete after eeprom interrupt handler is done

    return ret;

}

/*****************************************************************************
*
* Function:     attributeFlashSave()
*
* Description:  Computes checksum value for attribute table in NVM.
*              
* Input:        None.
*
* Output:       none.
*
* Notes:        will perform its own CRC call
*
*****************************************************************************/
void attributeFlashSave(void)
{
    uint8_t const addr = (NVMDevice.attribute_slot == false) ? ATT_FACTORY_FLASH_LOCATION : ATT_AP_SLOT_FLASH_LOCATION;
    int bytes_to_send = NUM_ATTRIBUTES_BYTES;
    int offset, size;
    uint8_t * pdata = (uint8_t *) &NVMBlock;
    
    NVMBlock.checkSum = nvmComputeCheckSum(pdata, NUM_ATTRIBUTES_BYTES-1);
    attributes_new_block_save_flag = true;

    //Assumption: at the start bytes_to_send will always be larger than 8
    //Check: make sure if we are not starting on a key address apge (0,8,16, etc..)
    //       that we limit the size, after which page writes can be size 8
    size = EEPROM_24LC01_ADDR_BLOCK_SIZE - (addr % EEPROM_24LC01_ADDR_BLOCK_SIZE);
    do
    {
        offset = NUM_ATTRIBUTES_BYTES - bytes_to_send;
        eeprom_write(addr + offset, pdata + offset, size);
        bytes_to_send -= size;
        size = bytes_to_send > EEPROM_24LC01_ADDR_BLOCK_SIZE ? EEPROM_24LC01_ADDR_BLOCK_SIZE : bytes_to_send;
    }while (bytes_to_send);
    bytes_to_send = -1;
}

/*****************************************************************************
*
* Function:     attributeVerifyFlashSaveResult()
*
* Description:  Check EEPROM data and verifies it was stored properly
*              
* Input:
*
* Output:       bool true - failure, false - success.
*
* Notes:        
*
*****************************************************************************/
bool attributeVerifyFlashSaveResult(void)
{
    return memcmp(&nvm_block_verifier,&NVMBlock,NUM_ATTRIBUTES_BYTES);
}
/*****************************************************************************
*
* Function:     attributeVerifyFlashSave()
*
* Description:  Load in EEPROM data into temp buffer to check if saved correctly
*              
* Input:        none.
*
* Output:       none.
*
* Notes:        
*
*****************************************************************************/
void attributeVerifyFlashSave(void)
{
    uint8_t const addr = (NVMDevice.attribute_slot == false) ? ATT_FACTORY_FLASH_LOCATION : ATT_AP_SLOT_FLASH_LOCATION;
    bool ret = false;
    uint8_t * pdata = (uint8_t *) &nvm_block_verifier;
    eeprom_read(addr, pdata, NUM_ATTRIBUTES_BYTES);
    //read will complete after eeprom interrupt handler is done

    //return ret;
}


/*****************************************************************************
*
* Function:     attributeFlashLoad()
*
* Description:  Computes checksum value for attribute table in NVM.
*              
* Input:
*
* Output:       bool true - failture, false - success.
*
* Notes:        
*
*****************************************************************************/
bool attributeFlashLoadGaugeBlock(void)
{
    uint8_t const addr = ATT_DEVICE_FLASH_LOCATION;
    uint8_t empty_array[8];
    bool ret = false;
    uint8_t * pdata = (uint8_t *) &NVMDevice;
    eeprom_read(addr, pdata, NUM_DEVICE_BYTES);
    //read will complete after eeprom interrupt handler is done

    return ret;
}

/*****************************************************************************
*
* Function:     attributeFlashSaveGaugeBlock()
*
* Description:  Computes checksum value for attribute table in NVM.
*              
* Input:        none.
*
* Output:       none.
*
* Notes:        will perform its own CRC call
*
*****************************************************************************/
void attributeFlashSaveGaugeBlock(void)
{
    uint8_t const addr = ATT_DEVICE_FLASH_LOCATION;
    int bytes_to_send = NUM_DEVICE_BYTES;
    int offset, size;
    uint8_t * pdata = (uint8_t *) &NVMDevice;
    
    NVMDevice.checkSum = nvmComputeCheckSum(pdata, NUM_DEVICE_BYTES-1);
    attributes_new_device_save_flag = true;

    //Assumption: at the start bytes_to_send will always be larger than 8
    //Check: make sure if we are not starting on a key address apge (0,8,16, etc..)
    //       that we limit the size, after which page writes can be size 8
    size = EEPROM_24LC01_ADDR_BLOCK_SIZE - (addr % EEPROM_24LC01_ADDR_BLOCK_SIZE);
    do
    {
        offset = NUM_DEVICE_BYTES - bytes_to_send;
        eeprom_write(addr + offset, pdata + offset, size);
        bytes_to_send -= size;
        size = bytes_to_send > EEPROM_24LC01_ADDR_BLOCK_SIZE ? EEPROM_24LC01_ADDR_BLOCK_SIZE : bytes_to_send;
    }while (bytes_to_send);

}

/*****************************************************************************
*
* Function:     attributeVerifyDeviceSaveResult()
*
* Description:  Check EEPROM data and verifies it was stored properly
*              
* Input:
*
* Output:       bool true - failure, false - success.
*
* Notes:        
*
*****************************************************************************/
bool attributeVerifyDeviceSaveResult(void)
{
    return memcmp(&nvm_device_verifier,&NVMDevice,NUM_DEVICE_BYTES);
}

/*****************************************************************************
*
* Function:     attributeVerifyDeviceSave()
*
* Description:  Load in EEPROM data into temp buffer to check if saved correctly
*              
* Input:
*
* Output:       none.
*
* Notes:        
*
*****************************************************************************/
void attributeVerifyDeviceSave(void)
{
    uint8_t const addr = ATT_DEVICE_FLASH_LOCATION;
    bool ret = false;
    uint8_t * pdata = (uint8_t *) &nvm_device_verifier;
    eeprom_read(addr, pdata, NUM_DEVICE_BYTES);
    //read will complete after eeprom interrupt handler is done

    //return ret;
}

/*****************************************************************************
*
* Function:     attributeNVMBlockCopy()
*
* Description:  fills in the pointer to a memory block with a copy of current loaded NVMBlock
*              
* Input:        Flash_Memory * memblock - location copy will be
*
* Output:       none.
*
* Notes:        
*
*****************************************************************************/
void attributeNVMBlockCopy(Flash_Memory * memblock)
{
    memcpy(memblock,&NVMBlock,NUM_ATTRIBUTES_BYTES);
}

/*****************************************************************************
*
* Function:     attributeVerifyNewData()
*
* Description:  determines what data needs to be verified and reads it from 
*               memory
*              
* Input:        none.
*
* Output:       none.
*
* Notes:        
*
*****************************************************************************/
void attributeVerifyNewData(void)
{
    if(attributes_new_device_save_flag == true)
    {
        attributeVerifyDeviceSave();
    }
    if(attributes_new_block_save_flag == true)
    {
        attributeVerifyFlashSave();
    }
}

/*****************************************************************************
*
* Function:     attributeVerifyNewDataStatus()
*
* Description:  returns status of new data verification
*              
* Input:        none.
*
* Output:       false - success, true - failure.
*
* Notes:        
*
*****************************************************************************/
bool attributeVerifyNewDataStatus()
{
    static uint8_t attempts = 0;
    bool result_1, result_2, ret = false;

    result_1 = attributes_new_device_save_flag == true ? attributeVerifyDeviceSaveResult() : false;
    result_2 = attributes_new_block_save_flag == true ? attributeVerifyFlashSaveResult() : false;

    //check if we are good, or need to retry writing attempts
    if((result_1 == true || result_2 == true))
    {
        ret = true;
        if(++attempts >= ATTRIBUTE_FAILURE_RETRY_COUNT){
            attempts = 0;
            //if we get here, need to fire off the failed lights
        }
        else
        {
           if(result_1)
           {
              attributeFlashSaveGaugeBlock();
           }
           if(result_2)
           {
              attributeFlashSave();
           }
        }
    }
    else
    {
        attributes_new_device_save_flag = false;
        attributes_new_block_save_flag = false;
        attempts = 0;
    }

    return ret;
}

/*****************************************************************************
*
* Function:     attributeGetGaugeMode()
*
* Description:  returns mask for if gauge is progammed as a demo gauge or normal type
*              
* Input:        none.
*
* Output:       false - success, true - failure.
*
* Notes:        
*
*****************************************************************************/
int attributeGetGaugeTypeMask(void)
{
    int ret = 0;
    if(attributeReadGaugeSelect() == ATT_GAUGE_DEMO)
    {
        ret = ATTRIBUTE_GAUGE_TYPE_DEMO_MASK;
    }

    return ret;

}


/*****************************************************************************
*
* Function:     attributeGetBacklightScanRate()
*
* Description:  returns rate of backlight refresh
*              
* Input:        none.
*
* Output:       ticks - mSec amount of time until new backlight adc reading is taken.
*
* Notes:        
*
*****************************************************************************/
uint32_t attributeGetBacklightScanRate(void)
{
    uint32_t ret = NVMBlock.backlightScanRate;
    return ret;
}
