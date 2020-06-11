/***************************************************************************** 
* File Name:        stepperMotor.c 
*
* Date Created:     6/08/20
*
* Author:           Michael Anderson				 
*
* Description:      Stepper motor control
*
* Function List:  
*                   Public
*                       motor_init()
*                       motorOff()
*                       motorOn()
*                       motorReload()
*                       motorStateMachine()
*                   Private
*
* Notes:            
*****************************************************************************/
/* Include shared modules, which are used for whole project*/
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrfx_gpiote.h"
#include "boards.h"
/* Include application files */
#include "motor.h"
#include "timers.h"
#include "projectManager.h"
#include "nrf_assert.h"

// Local Definitions

// Private Functions 

// Private Data


/*****************************************************************************
*
* Function:     motorInit()
*
* Description:  Initializes the stepper motor hardware/software on power up.
*             
* Input:        bool 0 - damping disabled, don't set its global variables
*                    1 - damping enabled, set its globals (ie NVM is good)
*
* Output:       None.
*
* Notes:        
*
*****************************************************************************/

void motor_init(bool start)
{

}



/*****************************************************************************
*
* Function:     motorOff()
*
* Description:  Turns stepper motor off by disabling timer. 
*             
* Input:        void
*
* Output:       None.
*
* Notes:        
*
*****************************************************************************/

void motorOff(void)
{
}


/*****************************************************************************
*
* Function:     motorOn()
*
* Description:  Turns stepper motor on (or keeps it running). This only 
*               initiates power. It does not generate the actual steps 
*               (done in state machine).
*             
* Input:        void
*
* Output:       None.
*
* Notes:        
*
*****************************************************************************/

void motorOn(void)
{
   motorSetDirection(motorInfo.direction);
}


/*****************************************************************************
*
* Function:     motorReload()
*
* Description:  Gets next stepper period value and reloads into timer.
*             
* Input:        None.
*
* Output:       period to reload the pwm will
*
* Notes:        This requires the current speed to be updated prior to call.
*
*****************************************************************************/

uint16_t motorReload(void)
{
    timerSetCC0(period);

    return period;

}


/*****************************************************************************
*
* Function:     motorSetDirection()
*
* Description:  Sets the DIR signal on the VID66 chip to indicate direction
*               motor is to run.
*             
* Input:        stepperMotorDirectionT dir - direction to run motor (CCW or CW)
*
* Output:       None.
*
* Notes:        This should be called prior to running the motor.
*
*****************************************************************************/

void motorSetDirection(stepperMotorDirectionT dir)
{
    if (dir == DIR_CCW)
    {
        nrf_gpio_pin_clear(MOTOR_DIR_PIN_NUMBER); // Toggle GPIO pin
    }
    else
    {
        nrf_gpio_pin_set(MOTOR_DIR_PIN_NUMBER); // Toggle GPIO pin
    }
}

/*****************************************************************************
*
* Function:     motorNeedsToStartup()
*
* Description:  Lets timer know that motor needs to be started up again
*             
* Input:        void
*
* Output:       bool - true motor needs to shut off, false ignore this check
*
* Notes:        None.
*
*****************************************************************************/
bool motorNeedsToStartup(void){
    bool ret = false;

    if(true){ //(motorOnFlag){
      ret = true;
      //enable our GPIO pin associated with CC[0] functionality
      nrf_drv_gpiote_out_task_enable(MOTOR_STEP_PIN_NUMBER);
    }
    return ret;
}

/*****************************************************************************
*
* Function:     motorNeedsToShutOff()
*
* Description:  Lets timer know that motor needs to be shut off after the last
*               pulse  wave, allows pre-emptive shutoff during half pulse transition
*             
* Input:        void
*
* Output:       bool - true motor needs to shut off, false ignore this check
*
* Notes:        None.
*
*****************************************************************************/
bool motorNeedsToShutOff(void){
    bool ret = false;
    if(true){ //motorOffFlag
      ret = true;
      //Disable our GPIO pin associated with CC[0] functionality
      nrf_drv_gpiote_out_task_disable(MOTOR_STEP_PIN_NUMBER);
      //motorOff();
    }
    return ret;
}

/*****************************************************************************
*
* Function:     motorStateMachine()
*
* Description:  Updates the motor signals based on the current state.
*               Keeps track of the motor position and determines when it 
*               needs to ramp up, ramp down or turn off.
*             
* Input:        pulse_polarity - 1 high pulse of wave, 0 low pulse
*
* Output:       None.
*
* Notes:        With the VID066, only need to generate a pulse for each step. 
*               Step is generated on the rising edge, so set pulse high then
*               before exiting, clear it. This results in a 2.5 uSec pulse,
*               which is within spec (min 450 nSec).
*
*****************************************************************************/

void motorStateMachine(uint8_t pulse_polarity)
{

    if(pulse_polarity){
        if (nrfx_gpiote_out_task_is_disable(MOTOR_STEP_PIN_NUMBER) ){ // don't bother doing any work if we disabled our PPI task PIN
            motorReload(); //reload the timer with new frequency
        }
    }
    else{
        //run some checks if we need to disable or renable CC[0] driver
        if(motorNeedsToStartup()){


        }
        else if(motorNeedsToShutOff()){

        }
    }
}

/*
 * Stepper Motor Functions End
 */

