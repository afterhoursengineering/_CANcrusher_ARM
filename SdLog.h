/*
 * SdLog.h
 *
 *  Created on: May 14, 2015
 *      Author: After Hours Engineering, LLC
 */

#ifndef SDLOG_H_
#define SDLOG_H_

#include "MCP2515.h"
#include "SIM808.h"

#ifdef __cplusplus
extern "C"
{
#endif
/**
 *  Place all C-accessible function prototypes here.
 */


void SdLog_Task(void);
void SdLog_setup(void);
boolean SdLog_OkToLog();
void SdLog_SetLogData(struct CAN_msg_t *msg);

boolean SdLog_SaveCanData(struct CAN_msg_t *msg);
boolean SdLog_SaveGpsData(struct SIM808_GPSPos_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* SDLOG_H_ */
