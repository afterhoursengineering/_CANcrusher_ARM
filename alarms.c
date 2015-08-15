/*
 * alarms.c
 *
 *  Created on: Apr 28, 2015
 *      Author: After Hours Engineering, LLC
 */

#include "alarms.h"
#include "core_pins.h"



#define false 0
#define true (!false)

static uint32_t array_AlarmTime[NUM_OF_ALARMS];
static boolean array_AlarmTriggered[NUM_OF_ALARMS];


void alarms_Init(void)
{
	uint8_t i;
	for(i=0;i<NUM_OF_ALARMS;i++){ array_AlarmTriggered[i] = false; }
}

/**
 *  Call to set an alarm
 */
void SetAlarm(uint8_t alarmIndex, uint32_t triggerTime)
{
	if(alarmIndex < NUM_OF_ALARMS){
		array_AlarmTime[alarmIndex] = triggerTime;
	}
}

/**
 *  Return state of the alarm
 */
boolean AlarmIsTriggered(uint8_t alarmIndex)
{
	boolean retVal = false;

	if(alarmIndex < NUM_OF_ALARMS){
		if(array_AlarmTriggered[alarmIndex]){ retVal = true; }
	}
	return retVal;
}

/**
 *  Call to clear an alarm
 */
void ClearAlarm(uint8_t alarmIndex)
{
	if(alarmIndex < NUM_OF_ALARMS)
	{
		array_AlarmTriggered[alarmIndex] = false;
	}
}

/**
 *  Call periodically to update the alarm status
 */
void ProcessAlarms(void)
{
	int i;
	uint32_t currentTime = millis();

	for(i = 0; i < NUM_OF_ALARMS; i++)
	{
		if(array_AlarmTime[i] <= currentTime)
		{
			array_AlarmTriggered[i] = true;
		}
	}
}

