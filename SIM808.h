/******************************************************************************
 **   SIM808.h
 **   Copyright (C) 2015  David Burke (david@afterhoursengineering.net)
 **
 **   This program is free software: you can redistribute it and/or modify
 **   it under the terms of the GNU General Public License as published by
 **   the Free Software Foundation, either version 3 of the License, or
 **   (at your option) any later version.
 **
 **   This program is distributed in the hope that it will be useful,
 **   but WITHOUT ANY WARRANTY; without even the implied warranty of
 **   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 **   GNU General Public License for more details.
 **
 **   You should have received a copy of the GNU General Public License
 **   along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#ifndef BUILD_SIM808_H_
#define BUILD_SIM808_H_


/*---------------------------------------------------------------------------*/
/* System Includes                                                           */
/*---------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
/* Constant declarations                                                     */
/*---------------------------------------------------------------------------*/

enum enum_SIM808_State {
	E_SIM808_PWR_ON,
	E_SIM808_PWR_OFF,
	E_SIM808_IS_OFF,
	E_SIM808_WAIT_FOR_RESP,
	E_SIM808_READY_FOR_CMD,
	E_SIM808_ENABLE_GPS,
	E_SIM808_DISABLE_GPS,
};
typedef enum enum_SIM808_State SIM808_State_t;

enum enum_SIM808_ParseOkResponseState {
	E_SIM808_PARSEOK_NONE,
	E_SIM808_PARSEOK_ENABLE_GPS,
	E_SIM808_PARSEOK_DISABLE_GPS,
	E_SIM808_PARSEOK_QUERY_GPS_PWR_STATUS,
	E_SIM808_PARSEOK_QUERY_FIX_STATUS,
	E_SIM808_PARSEOK_GPS_LOCATION,
};
typedef enum enum_SIM808_ParseOkResponseState SIM808_ParseOkState_t;

/*---------------------------------------------------------------------------*/
/* Type declarations                                                         */
/*---------------------------------------------------------------------------*/

typedef struct SIM808_GPSPos_t {
	uint8_t 	u8Indicators;		/**
	 	 	 	 	 	 	 	 	 *   7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
	 	 	                         *   |   |   |
	 	 	                         *   |   |   +______________________ 1: E, 0: W
	 	 	                         *   |   +__________________________ 1: N, 0: S
	 	 	                         *   +______________________________ 1: GPS Data Indicator
	 	 	                         */
	uint8_t 	u8Lat_dd;			/** Latitude: degrees */
	uint8_t		u8Lon_dd;			/** Longitude: degrees */
	uint8_t     u8Placeholder;		/** Keep the struct size same as CAN_msg_t */
	float 		fLat_minutes;		/** Latitude: Minutes mm.mmmmmm         */
	float		fLon_minutes;		/** longitude: Minutes mm.mmmmmm	     */
	float		fUtcTime;			/** UTC Time hhmmss.sss as a float */
} SIM808_GPSPos_t;


/*---------------------------------------------------------------------------*/
/* Variable declarations                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Macro declarations                                                        */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Function prototypes                                                       */
/*---------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif
void SIM808_Init(void);
void SIM808_Task(void);
uint8_t SIM808_ClearRxBuffer(void);
uint8_t SIM808_EnableGps(uint8_t bEnable);
uint8_t SIM808_GetGpsIsOn(void);		// Returns the current flag holding the state.
uint8_t SIM808_RequestGpsPwrStatus(void);	// Begins process of reading state
uint8_t SIM808_RequestGpsFixStatus(void);
uint8_t SIM808_RequestGpsLocation(void);

#ifdef __cplusplus
}
#endif


#endif /* BUILD_SIM808_H_ */
