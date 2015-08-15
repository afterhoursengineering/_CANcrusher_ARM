/*
 * alarms.h
 *
 *  Created on: Apr 28, 2015
 *      Author: After Hours Engineering, LLC
 */

#ifndef ALARMS_H_
#define ALARMS_H_

#include "WProgram.h"

#define NUM_OF_ALARMS 16u
#define ALARM_1 0
#define ALARM_2 1
#define ALARM_3 2
#define ALARM_4 3
#define ALARM_SIM808_RX_TIMEOUT	4u
#define ALARM_SIM808_2	5u

#ifdef __cplusplus
extern "C"
{
#endif
void alarms_Init(void);
void SetAlarm(uint8_t alarmIndex, uint32_t triggerTime);
void ClearAlarm(uint8_t alarmIndex);
void ProcessAlarms(void);
boolean AlarmIsTriggered(uint8_t alarmIndex);


#ifdef __cplusplus
}
#endif

#endif /* ALARMS_H_ */
