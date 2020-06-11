/***************************************************************************** 
* File Name:        attribute.h 
*
* Date Created:     7/24/19
*
* Author:           Michael Anderosn
*
* Description:      Header for gauge's attribute object.
*                 
* CSRCS Log
* $Log: attribute.h,v $
* Revision 1.8  2020-03-26 12:02:05-07  manderson
* Add PCA Board ID type definitions,
* Added Sensor Scan and Backlight refresh rates to attributes and system,
* Added sensor connection startup check, motor stays on pointers top if fails.
* Updated Sensor PCA curves.
* Cleaned up code.
* Rolled Revision to 00.002.00
*
* Revision 1.7  2020-03-06 11:48:39-08  manderson
* <>
*
* Revision 1.6  2020-01-24 11:14:54-08  manderson
* Updated Output Driver. Added more sensor Curves. Added EEPROM write verifications. Added warning Flash effect to lighting.c/.h
*
* Revision 1.5  2019-12-20 12:22:23-08  manderson
* Updated for R00.01.00 release
*
* Revision 1.4  2019-12-17 14:44:55-08  manderson
* Updated EEPROM read and writing and BLE commands and gauge curves
*
* Revision 1.3  2019-11-21 14:03:35-08  manderson
* Added NVM support for EEPROM chip. Added Curves.h/.c and few other files for handlling sensor data
*
* Revision 1.2  2019-10-02 14:21:13-07  manderson
* Updated App Scheduler to include attribute messages. Added full support for Attribute programming from Mobile App
*
* Revision 1.1  2019-09-26 14:50:22-07  manderson
* Added 2 demo modes for warn or sweep mode, revised BLE commands,pushed recieved messages onto app scheduler and added handler functions.
*
* Revision 1.0  2019-07-30 12:20:03-07  manderson
* Initial revision
*
*
* Notes:
*
* Copyright (c) 2019  ISSPRO Inc.
********************************************************************************/
#ifndef ATTRIBUTE_H
#define ATTRIBUTE_H

#include <stdint.h>
#include <stdbool.h>
#include "projectManager.h"
#include "nordic_common.h"


#define ATT_DRIVER_STARTUP_SCALAR 15000 // timers expect time to be in ms ( 1unit = 15s then scalar needs to be make 1unit = 15000 ms)
#define ATT_GAUGE_SWEEP_DEGREE_SCALAR 2
#define WEIGHT_SCALAR   0.00025
#define BACKLIGHT_V_SCALAR   0.1    // .1V per unit for backlight top/bot V attributes

/*
 *  Attributes are Stored on a 128x8 bits memory chip
 *  NVMblock is around 
 *
 */
#define ATT_FACTORY_FLASH_LOCATION      0x00    // reserve first half of memory block for FACTORY FLASH
#define ATT_AP_SLOT_FLASH_LOCATION      0x35    // second half of memory block is for custom Attribute Programmer flash
#define ATT_DEVICE_FLASH_LOCATION       0x6A    // small space for additional device info

#define ATTRIBUTE_FAILURE_RETRY_COUNT   5 //retry up to 5 times to save NVM data
#define ATTRIBUTE_GAUGE_TYPE_DEMO_MASK  0x80000000    //used as a mask to pair with gauge mode in checks to also see if gauge is programmed as demo gauge

typedef enum
{
    ATT_GAUGE_MODE_NORMAL = 0,
    ATT_GAUGE_MODE_TEST,
    ATT_GAUGE_MODE_SWEEP_DEMO,
    ATT_GAUGE_MODE_WARN_DEMO,
    ATT_GAUGE_MODE_MAX_SIZE
} AttGaugeModeT;

#define ATT_GAUGE_MODE_NORMAL_TYPE_DEMO (ATT_GAUGE_MODE_NORMAL | ATTRIBUTE_GAUGE_TYPE_DEMO_MASK)
// Definitions for available types that gauge can be configured.
// NOTE: Max # of gauge types is not defined here because this value
// can be set to something higher to indicate a demo/test mode.
typedef enum
{
    ATT_GAUGE_DEMO,
    ATT_GAUGE_FUEL_LEVEL,
    ATT_GAUGE_VOLT_18V,
    ATT_GAUGE_COOL_TEMP_100_280,
    ATT_GAUGE_REAR_AXLE_TEMP_100_280,
    ATT_GAUGE_TRANS_OIL_TEMP_100_280,
    ATT_GAUGE_PRI_AIR_PRESS_175,
    ATT_GAUGE_OIL_PRESS_100,  
    ATT_GAUGE_EXHAUST_BACK_PRESS_60,
    ATT_GAUGE_EXHAUST_BACK_PRESS_100,
    ATT_GAUGE_TURBO_BOOST_40,
    ATT_GAUGE_TURBO_BOOST_60,
    ATT_GAUGE_TURBO_BOOST_100,
    ATT_GAUGE_FUEL_PRESS_30_30,         //Fuel gauge 30 psi using 30 psi sensor
    ATT_GAUGE_FUEL_PRESS_100_30,        //Fuel gauge 30 psi using 100 psi  sensor
    ATT_GAUGE_FUEL_PRESS_100_40,        //Fuel gauge 40 psi using 100 psi  sensor
    ATT_GAUGE_FUEL_PRESS_100_100,       //Fuel gauge 100 psi using 100 psi  sensor
    ATT_GAUGE_FUEL_RAIL_PRESS_29k_30k,
    ATT_GAUGE_VOLT_36V,
//  ATT_GAUGE_NEW_GAUGE_HERE,
    ATT_GAUGE_NONE = 65535
}   AttGaugeTypeT;

typedef enum
{
    ATT_POINTER_NORMAL = 0,
    ATT_POINTER_FAT
}   AttPointerStyleT;

typedef enum
{
    ATT_METER_KM = 0,                   // Kilometers
    ATT_METER_MI                        // Miles
}   AttMeterUnitsT;

// RGBWS interface uses this order of color 0xGGRRBBWW
typedef struct
{
    uint8_t white;
    uint8_t blue;
    uint8_t red;
    uint8_t green;
} RGB_ColorT;

// Quadrant/Gauge attribute data stored in NVM flash memory.
//#pragma pack(push, 1)   // compiler adds pad bytes to struct if don't do this!
typedef struct
{
    uint16_t  gaugeType;        // 0-65535, selects gauge type
    uint16_t  gaugeHome;        // 0-65535, Zero position on face dial
    uint16_t  gaugeFull;        // 0-65535, Max position on face dial
    uint8_t   gaugeSweep;       // 1-180, (in 2 degree units: 2-360 Degrees) Angular degrees of pointer sweep
    uint16_t  minValidReading;  // -32768 to +32767, minimum valid sensor reading in face dial units 
    uint16_t  maxValidReading;  // -32768 to +32767, maximum valid sensor reading in face dial units 
    uint8_t   sensorCurve;      // 0-255, sensor Curve utlize
    uint16_t  pointerWeight;    // 0-4000 (in .00025 units: 0.0-1.0) weight of each reading relative to total
    uint8_t   hysteresis;       // 0-255 Hysteresis band in ADC counts in both directions
    uint8_t   sensorScanRate;   // 0-255 (in mSec) sample rate to read the sensor
    uint32_t  coeffecient0;     // IEE 754 single precision float, First coefficient of linear sensor curve
    uint32_t  coeffecient1;     // IEE 754 single precision float, Second coefficient of linear sensor curve
    uint8_t   modeTypeFlash;    // bits 7-4: Mode of operation (0 = normal, 1= test, 2 = sweep demo, 3= warning demo) 
                                // bits 3-2: Pointer type (0 = standard, 1 = fat)
                                // bits 1-0: Backlight Flash intesisty (0 = subtle, 1 = medium, 2 = extreme)
    uint8_t   backlightFlash;   // bits 7-1: Backlight Flash level (0 : disabled, 1-100 %)
                                // bit 0:    Backlight Flash Zone (0 = low, 1 = high)
    uint16_t  backlightTopV;    // 1-65535 (in 0.1 units: 0.1-65553.5V) Dash input voltage represents highest intesnity
    uint16_t  backlightBotV;    // 1-65535 (in 0.1 units: 0.1-65553.5V) Dash input voltage represents lowest intesnity
    uint8_t   backlightRed;     // 0-255, red color setting for backlight RGB
    uint8_t   backlightGreen;   // 0-255, green color setting for backlight RGBW
    uint8_t   backlightBlue;    // 0-255, blue color setting for backlight RGBW
    uint8_t   backlightWhite;   // 0-255, white color setting for backlight RGBW
    uint8_t   pointerRed;       // 0-255, red color setting for pointer RGB
    uint8_t   pointerGreen;     // 0-255, green color setting for pointer RGBW
    uint8_t   pointerBlue;      // 0-255, blue color setting for pointer RGBW
    uint8_t   pointerWhite;     // 0-255, white color setting for pointer RGBW
    uint8_t   warningLightBot;  // bits 7-1: bottom level to turn on warning light (0= disabled,1-100%)
                                // bit 0: bottom level warning light zone (0 = low, 1 = high)
    uint8_t   warningLightTop;  // bits 7-1: Top level to turn on warning light (0= disabled,1-100%)
                                // bit 0: Top level warning light zone (0 = low, 1 = high)
    uint8_t   outputDriverBot;  // bits 7-1: bottom level to turn on output driver (0= disabled,1-100%)
                                // bit 0: bottom level output driver zone (0 = low, 1 = high)
    uint8_t   outputDriverTop;  // bits 7-1: Top level to turn on output driver (0= disabled,1-100%)
                                // bit 0: Top level output driver zone (0 = low, 1 = high)
    uint8_t   outputTiming;     // bits 7-6: Output driver activation delay during Startup (0 = disabled, 1-3 in 15 sec units: 15 - 45 seconds) 
                                // bits 5-0: Output driver activation delay during normal operation (0 = disbled, 1-60 in 1 sec units)
    uint8_t   backlightScanRate;// 0-255, msec time for backlight dimmer to be checked and brightness refreshed.
    uint32_t  emptyfiller_1;    
    uint32_t  emptyfiller_2;
    uint8_t   checkSum;         // 8 bit checksum for memory block
}  __attribute__((__packed__)) Flash_Memory;

#define DEVICE_NAME_SIZE  12
typedef struct
{
    char device_name[DEVICE_NAME_SIZE];       // Device name broadcasted on BLE advertisement, up to a 12 characters long
    bool attribute_slot;        // tells which slot location in NVM to load attributes from.
                                // 0: slot 1 (factory attributes) 1 : slot 2 (custom/attribute programmer)
    uint8_t empty[4];           // 5 filler bytes for future device attributes
    uint32_t device_SN;         // Unique 32-bit serial ID for each EV3 gauges
    uint8_t  checkSum;          // 8 bit checksum for memory block
} __attribute__((__packed__)) Flash_Memory_Device;

#define NUM_ATTRIBUTES_BYTES sizeof(Flash_Memory)     // 53 bytes NOTE: 53 Bytes is max size this can be, no more space on NVM chip
                                                      // 45 / 53 Bytes are currently being used, the 8 ones left are available to take
#define NUM_DEVICE_BYTES sizeof(Flash_Memory_Device)  // 22 bytes NOTE: 22 Bytes is max size this can be, no more space on NVM chip

bool attributeInit(void);
bool attributeCheckNvmBlock(void);
AttGaugeTypeT attributeReadGaugeSelect(void);
uint16_t attributeReadGaugeHome(void);
uint16_t attributeReadGaugeFull(void);
uint8_t attributeReadGaugeSweep(void);
int16_t attributeReadGaugeMin(void);
int16_t attributeReadGaugeMax(void);
uint8_t attributeReadSensorCurve(void);
uint16_t attributeReadPointerWeight(void);
uint8_t attributeReadGaugeHysteresis(void);
uint8_t attributeReadSensorScanRate(void);
float attributeReadSensorCoeff0(void);
float attributeReadSensorCoeff1(void);
AttGaugeModeT attributeReadGaugeOperationMode(void);
AttPointerStyleT attributeReadPointerType(void);    //TODO: implement this into code
uint8_t attributeReadBacklightFlashIntensity(void);
uint8_t attributeReadBacklightFlashThreshold(void);
uint8_t attributeReadBacklightFlashZone(void);
float attributeReadBacklightBottomVoltage(void);
float attributeReadBacklightTopVoltage(void);
RGB_ColorT attributeReadBacklightRGBW(void);
RGB_ColorT attributeReadPointerRGBW(void);
uint8_t attributeReadBottomWarningLightThreshold(void);
uint8_t attributeReadBottomWarningLightZone(void);
uint8_t attributeReadTopWarningLightThreshold(void);
uint8_t attributeReadTopWarningLightZone(void);
uint8_t attributeReadBottomOutputDriverThreshold(void);
uint8_t attributeReadBottomOutputDrivertZone(void);
uint8_t attributeReadTopOutputDriverThreshold(void);
uint8_t attributeReadTopOutputDriverZone(void);
uint8_t attributeReadOutputDriverStartupDelay(void);
uint8_t attributeReadOutputDriverActivationDelay(void);
void attributeFillRAMCopy(uint8_t *dataPtr, appBLEMessageEventT message);
void attributeFillGaugeNVMBlock(uint8_t *dataPtr, appBLEMessageEventT message);
void attributeStorePacket(char* packet, int index);
void attribute_process_message_packet(void * p_evt);
float * attributeGetCoeffPtr(void);
void attributeModeDemoInc(void);
void attributeFlashSaveGaugeBlock(void);
bool attributeFlashLoadGaugeBlock(void);
void attributeFlashSave(void);
bool attributeFlashLoad(void);
bool attributeCheckStatus(void);
char * attributeGetDeviceNamePtr(void);
uint32_t attributeGetDeviceSN(void);
void attributeVerifyNewData(void);
bool attributeVerifyNewDataStatus();
void attributeNVMBlockCopy(Flash_Memory * memblock);
int attributeGetGaugeTypeMask(void);
uint32_t attributeGetBacklightScanRate(void);

#endif //ATTRIBUTE_H