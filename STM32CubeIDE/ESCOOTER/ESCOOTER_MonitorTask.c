/*
 * ESCOOTER_MonitorTask.c
 *
 *  Created on: 7 Jul 2023
 *      Author: TerenceLeung
 */
#include "ESCOOTER_MonitorTask.h"
#include "mc_api.h"
#include "main.h"

void ESCOOTER_PhysicalParameterInit(ESCOOTER_Physical_State_t *stateHandle)
{
	/*Initialize those fucking parameters*/
	stateHandle->current_speed = 0;
	stateHandle->phase_current = 0;
	stateHandle->phase_voltage = 0;
	stateHandle->motor_status  = 0;
}

ESCOOTER_Physical_State_t ESCOOTER_PhysicalParameterMonitoring(ESCOOTER_Physical_State_t *stateHandle)
{
	/*You could test it by inputing dummy data */
	/*To convert those parameters, please read the datasheet !!*/
    stateHandle->current_speed = MC_GetMecSpeedAverageMotor1(); //Need Conversion
    stateHandle->phase_current = MC_GetPhaseCurrentAmplitudeMotor1(); //Need Conversion
    stateHandle->phase_voltage = MC_GetPhaseVoltageAmplitudeMotor1(); //Need Conversion
    stateHandle->motor_status = (int32_t)MC_GetSTMStateMotor1(); //Need Conversion
    return *stateHandle;
}
