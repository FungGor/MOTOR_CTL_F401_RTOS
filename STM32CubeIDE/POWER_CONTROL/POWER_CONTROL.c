/*
 * POWER_CONTROL.c
 *
 *  Created on: 27 Jul 2023
 *      Author: TerenceLeung
 */
#include "POWER_CONTROL.h"
#include "POWER_CONTROL_LL.h"
#include "ESCOOTER_MainTask.h"
#include "main.h"
#include <stdint.h>

static POWER_Control_t *pwrControl;
static Power_Status_Indicator_t *tailLightControl;
static Power_sysProtocol_Handler_t *reTransMgnt;
POWER_State_t state_Handler;
Power_Control_Heartbeat_t protocolHandler;

void POWER_CONTROL_CONFG(POWER_Control_t *cmd)
{
	pwrControl = cmd;
}

void POWER_INDICATOR_CONFG(Power_Status_Indicator_t *indicator)
{
	tailLightControl = indicator;
}

void POWER_RETRANSMIT_CTL_CONFG(Power_sysProtocol_Handler_t *reTransTIM)
{
    reTransMgnt = reTransTIM;
}

void POWER_SET_DEFAULT_STATE(POWER_State_t state)
{
	state_Handler = state;
}

void POWER_CHANGE_STATE(POWER_State_t state)
{
	state_Handler = state;
}

void Stop_RetransmissionTimer()
{
	protocolHandler.RxPacketLossCount = 0;
	reTransMgnt->reTransmissionOff();
}

void retransmissionTimerStart()
{
	reTransMgnt->reTransmissionOn();
}


void PacketLossCount()
{
   protocolHandler.RxPacketLossCount++;
}

void POWER_PACKET_ACK()
{
	protocolHandler.RxPacketLossCount = 0;
}

void POWER_PROTOCOL_CHECKSTATUS()
{
    if(protocolHandler.RxPacketLossCount == 0)
    {
    	protocolHandler.protocolFailure = false;
    }
    else if(protocolHandler.RxPacketLossCount != 0)
    {
    	if(protocolHandler.RxPacketLossCount > MAXIMUM_PACKET_RETRANSMIT)
    	{
            protocolHandler.protocolFailure = true;
            ESCOOTER_SendReportStatus(true);
            /*If protocolFailure = true, E-Scooter must do the following tasks:
             * Case 1: If the E-Scooter is in DRIVING_START:
             * --> Stop the motor by changing the state from DRIVING_START to DRIVING_IDLE
             * --> Automatically Power Off
             *
             * Case 2: If the E-Scooter is in DRIVING_IDLE:
             * --> Automatically Power Off
             * */

    	}
    }
}

void POWER_SLEEP()
{
	pwrControl->sleep();
	//Peripheral_DeInit();
}

void POWER_WAKEUP()
{
	pwrControl->wake();
}

void POWER_CTL_MONITORING(void const *argument)
{
	for(;;)
	{
		switch(state_Handler)
		{
		    case POWER_OFF:
		    	pwrControl -> sleep();
		    	break;

		    case POWER_ON:
		    	pwrControl -> powerOn();
		    	break;

		    case WAKEUP:
		    	pwrControl -> wake();

		    default:
		    	break;
		}
	}
}
