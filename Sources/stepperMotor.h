/***************************************************************************** 
* File Name:        stepperMotor.h 
*
* Date Created:     6/8/2020
*
* Author:           Michael Anderson				 
*
* Description:      Header for stepper motor control
*
*****************************************************************************/

#ifndef MOTOR_H
#define MOTOR_H 1

/***** Public Types *****/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

typedef enum
{
    DIR_CCW,
    DIR_CW
} stepperMotorDirectionT;

/***** Public Functions *****/

void     motor_init(bool start);
void     motorOff(void);
void     motorOn(void);
uint16_t motorReload(void);
void     motorSetDirection(stepperMotorDirectionT dir);
void     motorStateMachine(uint8_t);
bool     motorNeedsToShutOff(void);
bool     motorNeedsToStartup(void);

#endif

