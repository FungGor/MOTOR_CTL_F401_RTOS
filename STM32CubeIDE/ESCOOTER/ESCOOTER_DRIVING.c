/*
 * ESCOOTER_DRIVING.c
 *
 *  Created on: 18 Jul 2023
 *      Author: TerenceLeung
 */

#include "ESCOOTER_DRIVING.h"
#include "cmsis_os.h"
#include "mc_api.h"
#include "main.h"

ESCOOTER_Driving_State_t Driving_State;
ESCOOTER_BrakeANDThrottleInput_t modeControl;
ESCOOTER_Physical_State_t motorStatus;

void ESCOOTER_saveStatus (uint8_t state)
{
	  Driving_State = state;
}

ESCOOTER_Driving_State_t ESCOOTER_getStatus()
{
	return Driving_State;
}

void ESCOOTER_Set_Limit(ESCOOTER_BrakeANDThrottleInput_t *limitHandle)
{
     modeControl = *limitHandle;
}

void ESCOOTER_Set_PhysicalParam(ESCOOTER_Physical_State_t *motorParam)
{
     motorStatus = *motorParam;
}

int16_t throttle_Current = 0;
int16_t current_limits = 0;
int16_t speed_limits = 0;
uint16_t ramping = 0;
void ESCOOTER_Driving_Start()
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_SET);
	/*Set acceleration ramp*/
	throttle_Current = modeControl.TARGET_IQ;
	/*Those parameters will be used as Cruise Control*/
	current_limits = modeControl.IQ_LIMIT;
	speed_limits = modeControl.SPEED_LIMIT;
	ramping = modeControl.RAMP_DURATION;
}

void ESCOOTER_Driving_Stop()
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_RESET);
	/*Stop Motor!*/
}

void ESCOOTER_DrivingTaskControl(void const * argument)
{
     for(;;)
     {
           if(Driving_State == DRIVING_IDLE)
           {
        	   ESCOOTER_Driving_Stop();
           }
           else if (Driving_State == DRIVING_START)
           {
        	   ESCOOTER_Driving_Start();
           }
           else if(Driving_State == DRIVING_STOP)
           {
        	   ESCOOTER_Driving_Stop();
           }
     }
}
