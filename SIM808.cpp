/******************************************************************************
 **   SIM808.cpp
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

/*---------------------------------------------------------------------------*/
/* System Includes                                                           */
/*---------------------------------------------------------------------------*/
#include "SIM808.h"
#include "HardwareSerial.h"
#include "mcp23018.h"
#include <core_pins.h>
#include <string.h>
#include "alarms.h"
#include "support.h"


/*---------------------------------------------------------------------------*/
/* Constant declarations                                                     */
/*---------------------------------------------------------------------------*/
#define ENABLE_SIM808_DIAGNOSTICS	1u

#define REG_ON		(uint8_t)0
#define REG_OFF		(uint8_t)1u

#define MIN_PWR_PIN_DURATION 2000UL
#define AT_RESP_MAX_DURATION 10000UL

/** MAX # of bytes that can be sent to the SIM808 at a time. */
#define SIM808_TX_BUFFER_MAX 511u	// TX_BUFFER of serial3 is 512. Must be one less.

/** MAX Size of the RX Buffer */
#define SIM808_RX_BUFFER_MAX 512u

/** When the GPS RMC sentence is tokenized, there should be 12 delimiters found */
#define RMC_DELIM_COUNT	12u

/*---------------------------------------------------------------------------*/
/* Type declarations                                                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Structure declarations                                                     */
/*---------------------------------------------------------------------------*/
/**
 * Status flags for the state of the SIM808 module.
 */
typedef struct SIM808_Stat_t {
	uint8_t bIsOn;		/** T/F whether the SIM808 is ON or not */
	uint8_t bIsBusy;		/** T/F whether SIM808 is busy */
	uint8_t bRespReady;	/** T/F whether a command response is ready. */
	uint8_t bTimedOut;	/** T/F last command timed out */
	uint8_t bOk;			/** T/F response received and OK received */
	uint8_t bError;		/** T/F response received but an ERROR occurred */
	uint8_t bRxBufOvf;	/** T/F whether rxBuf has overflowed */
	uint8_t bGpsIsOn;		/** T/F whether GPS is enabled/ON */
	uint8_t bCmdSuccess;	/** T/F whether last command was successful */
	uint8_t bGpsFixUnknown;
	uint8_t bGpsFixNotFix;
	uint8_t bGpsFix2dFix;
	uint8_t bGpsFix3dFix;
	uint16_t u16RxBufLen;	/** Length of the received response */
} SIM808_Stat_t;
static SIM808_Stat_t devStat;


/*---------------------------------------------------------------------------*/
/* Variable declarations                                                     */
/*---------------------------------------------------------------------------*/
const char * OK_STR = "OK\x0d\x0a";
const char * ERR_STR = "ERROR\x0d\x0a";
const char * GPS_PWR_STAT_ON = "+CGPSPWR: 1";
const char * GPS_PWR_STAT_OFF = "+CGPSPWR: 0";
const char * GPS_FIX_STAT_UNKNOWN = "+CGPSSTATUS: Location Unknown";
const char * GPS_FIX_STAT_NOT_FIX = "+CGPSSTATUS: Location Not Fix";
const char * GPS_FIX_STAT_2D_FIX  = "+CGPSSTATUS: Location 2D Fix";
const char * GPS_FIX_STAT_3D_FIX  = "+CGPSSTATUS: Location 3D Fix";
static SIM808_State_t state;
static SIM808_ParseOkState_t parseOkState;
static uint8_t SIM808_REG_EN_state = REG_OFF;
static uint8_t u8RxBuf[SIM808_RX_BUFFER_MAX];
static uint16_t u16RxIndex;
static uint32_t u32ProcessTimeout;

//-------------- GPS Position-Related --------------
static SIM808_GPSPos_t pos;
static char GPSPOS_pu8UtcTimeStr[11u];		// = "000000.000";
static char GPSPOS_pu8StatusStr[2u]; 		// = "V";
static char GPSPOS_pu8LatitudeStr[12u];		// = "0000.000000";
static char GPSPOS_pu8NSIndicatorStr[2u];	// = "N";
static char GPSPOS_pu8LongitudeStr[13u];	// = "00000.000000";
static char GPSPOS_pu8EWIndicatorStr[2u];	// = "E";
static char GPSPOS_pu8SpeedKnotsStr[7u];	// = "0.00";
static char GPSPOS_pu8DateStr[7u];			// = "010170";
static char GPSPOS_pu8CourseStr[7u];		// = "000.00";
// -------------------------------------------------
/*---------------------------------------------------------------------------*/
/* Macro declarations                                                        */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Function prototypes                                                       */
/*---------------------------------------------------------------------------*/
void SendDiagMessage(const char * msg);
void SendDiagMessageNoCRLF(const char * msg);
void SIM808_ParseOkResponse(const char * pResponse);
uint8_t SIM808_SendAndReceive(const char *pu8Cmd, uint32_t u32Timeout);
uint8_t SIM808_ParseGpsData(const char * pu8Position);

/******************************************************************************
 ** Function: SIM808_Init
 ** 
 ** Description: Initialize the SIM808 Driver
 **              Reference the following website for details on the SIM808
 **              <http://wiki.iteadstudio.com/SIM808_GSM/GPRS/GPS_Module>
 **
 ** Inputs:  NA
 ** Outputs: NA
 ** 
 *****************************************************************************/
void SIM808_Init(void)
{

/**
 *  Initialize the device status flags.
 */
	devStat.bError = false;
	devStat.bIsBusy = false;
	devStat.bIsOn = false;
	devStat.bOk = false;
	devStat.bRespReady = false;
	devStat.bTimedOut = false;
	devStat.bRxBufOvf = false;
	devStat.bCmdSuccess = false;
	devStat.u16RxBufLen = 0;
	devStat.bGpsFixUnknown = true;
	devStat.bGpsFixNotFix = false;
	devStat.bGpsFix2dFix = false;
	devStat.bGpsFix3dFix = false;

	/** Init the parsing state to NONE */
	parseOkState = E_SIM808_PARSEOK_NONE;

	serial3_begin(BAUD2DIV(115200u));
	serial3_format(SERIAL_8N1);

	/* Make sure SIM808_INDICATOR pin (U13.3 GPB0 of DEV_MISC) is an INPUT */
	MCP23018_ConfigAsInput(DEV_MISC, GP0, GPIOB);
	state = E_SIM808_PWR_ON;


}


/******************************************************************************
 ** Function: SIM808_Task
 **
 ** Description: Periodic Task for SIM808 Processing
 ** Inputs:  NA
 ** Outputs: NA
 **
 *****************************************************************************/
void SIM808_Task(void)
{
	static uint32_t currentTime = 0;
	static uint8_t rxBuf[64u];
	static uint16_t rxLen = 0;
	static uint8_t txBuf[64u];
	static int16_t txLen = 0;
	uint8_t * pu8Resp;
	const char * pu8StrToSearch;
	static const char * pu8OkResponseStr;
	const char * pu8CmdToSend;
	char * pu8MsgToSend;
	uint16_t i;

	switch(state)
	{
	/**
	 *  Attempting to turn the SIM808 ON
	 */
	case E_SIM808_PWR_ON:
		if(REG_OFF == SIM808_REG_EN_state)
		{
			/* Enable the regulator by pulling SIM808_REG_EN low */
			MCP23018_ClearPin(DEV_MISC,GP7,GPA);
			SIM808_REG_EN_state = REG_ON;

			/**
			 *  Prototype CANcrushers do not have a pull-up on the SIM808
			 *  Enable line and the control line from the MCP23018 is floating
			 *  by default. The SIM808 is likely powered when a RESET of the
			 *  Teensy occurs.  Must check the SIM808_INDICATOR before attempting
			 *  to cycle power of the SIM808.
			 */
			if(MCP23018_READ(DEV_MISC,GPIOB) & GP0)
			{
				/** SIM808 is already ON.  Go to READY state **/
				state = E_SIM808_READY_FOR_CMD;
				devStat.bIsOn = true;
				currentTime = millis();
				SendDiagMessage("SIM808 already ON.  Going to READY_FOR_CMD state");
			}
			else /** SIM808 NOT ON  or not present */
			{
				/* Apply 3.3V to the SIM808_PWR pin for at least 2 seconds */
				MCP23018_ClearPin(DEV_MISC,GP4,GPA);
				currentTime = millis();		// Capture a time sample
				SendDiagMessage("SIM808 Not ON. Toggling PWR pin for 2 sec.");
			}


		}

		if(((millis() - currentTime) > MIN_PWR_PIN_DURATION) || (currentTime > millis()))
		{
			/* Check the indicator state and turn off the SIM808_PWR pin */
			MCP23018_SetPin(DEV_MISC,GP4,GPA);
			if(MCP23018_READ(DEV_MISC,GPIOB) & GP0)	// Indicator says SIM808 is ON
			{
				state = E_SIM808_READY_FOR_CMD;
				SendDiagMessage("SIM808 is ON");
				devStat.bIsOn = true;
			}
			else
			{
				/* Indicator says SIM808 is still off... Something is wrong... */
				MCP23018_SetPin(DEV_MISC,GP7,GPA);	// Disable the Regulator
				SIM808_REG_EN_state = REG_OFF;
				state = E_SIM808_IS_OFF;
				devStat.bIsOn = false;
				SendDiagMessage("ERROR: SIM808 NOT ON.");
			}
		}

		break;

		/**
		 * Attempting to turn the SIM808 OFF
		 */
	case E_SIM808_PWR_OFF:
		/**
		 *  TODO: Implement E_SIM808_PWR_OFF case.
		 */
		break;

		/**
		 *  The SIM808 is currently off
		 */
	case E_SIM808_IS_OFF:
		devStat.bIsOn = false;
		break;

		/**
		 *  A command was sent. Listening for a response
		 *  terminated by either OK+CRLF or ERR+CRLF
		 */
	case E_SIM808_WAIT_FOR_RESP:

//		if(AlarmIsTriggered(ALARM_SIM808_RX_TIMEOUT))
		if(u32ProcessTimeout < millis())
		{
//			ClearAlarm(ALARM_SIM808_RX_TIMEOUT);
			devStat.bTimedOut = true;
			devStat.bIsBusy = false;
			state = E_SIM808_READY_FOR_CMD;

			SendDiagMessage("bTimedOut");
		}
		else
		{
			txLen = serial3_available();
			if(txLen>0)
			{
				//SendDiagMessage("SIM808 Responded...processing...");
				uint16_t u16StrtIndex = u16RxIndex;
				for(i=u16StrtIndex; i < (u16StrtIndex + txLen); i++){
					u8RxBuf[u16RxIndex++] = (uint8_t)serial3_getchar();
				}
				/**
				 * Check for the OK_STR or ERR_STR in the response
				 */
				pu8StrToSearch = (const char *)u8RxBuf;
				if(NULL != strstr(pu8StrToSearch,OK_STR))
				{
					/* OK Termination String found */
					devStat.bOk = true;
					devStat.bRespReady = true;
					devStat.bIsBusy = false;
					devStat.u16RxBufLen = u16RxIndex;
					state = E_SIM808_READY_FOR_CMD;

					pu8OkResponseStr = (const char *)u8RxBuf;

					//SendDiagMessage("OK_STR Found");
				}
				else if(NULL != strstr(pu8StrToSearch,ERR_STR))
				{
					/* ERR Termination String found */
					devStat.bError = true;
					devStat.bRespReady = true;
					devStat.bIsBusy = false;
					devStat.u16RxBufLen = u16RxIndex;
					state = E_SIM808_READY_FOR_CMD;

					//SendDiagMessage("ERR_STR Found");
				}
				else
				{
					/* Termination string not found yet */
					//SendDiagMessage("...still waiting");
				}
			}
		}

		break;

		/**
		 *  SIM808 is between processes and is ready for a command.
		 */
	case E_SIM808_READY_FOR_CMD:
		rxLen = serial2_available();
		if(rxLen > 0)
		{
			for(i=0;i<rxLen;i++){ rxBuf[i] = (uint8_t)serial2_getchar(); }

			if(rxLen < (SIM808_TX_BUFFER_MAX-1)){ rxBuf[rxLen] = 0; }
			pu8CmdToSend = (const char*)rxBuf;

			/**
			 *  Check for Diagnostic inputs
			 *  3030 --> Enable GPS
			 *  3031 --> Disable GPS
			 */
			if(0x30u == rxBuf[0])
			{
				switch(rxBuf[1u])
				{
				case 0x30u:	// Enable GPS
					if(SIM808_EnableGps(true)){ SendDiagMessage("Attempting to enable GPS"); }
					else{ SendDiagMessage("Failed to enable GPS"); }
					break;
				case 0x31u:	// Disable GPS
					if(SIM808_EnableGps(false)){ SendDiagMessage("Attempting to disable GPS"); }
					else{ SendDiagMessage("Failed to disable GPS"); }
					break;
				case 0x32u:	// Read the GPS power state
					if(SIM808_RequestGpsPwrStatus()){ SendDiagMessage("Attempting to read GPS PWR State."); }
					else{ SendDiagMessage("Failed to read GPS PWR state"); }
					break;
				case 0x33u: // Request GPS Fix Status
					if(SIM808_RequestGpsFixStatus()){ SendDiagMessage("Attempting to read GPS FIX Status."); }
					else{ SendDiagMessage("Failed to read GPS FIX Status"); }
					break;
				case 0x34u: // Request GPS Location
					if(SIM808_RequestGpsLocation()){ SendDiagMessage("Attempting to aquire GPS Location."); }
					else{ SendDiagMessage("Failed to read GPS Location"); }
					break;
				}
			}
			else if((rxBuf[0] == 'A') && (rxBuf[1] == 'T'))
			{
				parseOkState = E_SIM808_PARSEOK_NONE;
				if(!SIM808_SendAndReceive(pu8CmdToSend, 10000u))
				{
					//pu8MsgToSend = "ERROR: Could not send.\x0d\x0a";
					//serial2_write(pu8MsgToSend,(uint16_t)strlen(pu8MsgToSend));
					SendDiagMessage("ERROR: Could not send");
					//return;
				}
				else{
					//SendDiagMessage("RX'd Message. Sending the following:");
					//SendDiagMessage(pu8CmdToSend);
				}
			}
			else
			{
				SendDiagMessage("Send an 'AT' command or a 0x0n test command only.");
			}

		}

		if(devStat.bRespReady)
		{
			devStat.bRespReady = false;
			if(devStat.bOk)
			{
				devStat.bOk = false;
				//SendDiagMessage("bOk: Response to follow...");
				serial2_write(u8RxBuf, devStat.u16RxBufLen);

				SIM808_ParseOkResponse(pu8OkResponseStr);
			}
			else if(devStat.bError)
			{
				devStat.bError = false;
				//pu8MsgToSend = "bError: Response to follow...\x0d\x0a";
				//serial2_write(pu8MsgToSend, (uint16_t)strlen(pu8MsgToSend));
				//SendDiagMessage("bError: Response to follow...\x0d\x0a");
				serial2_write(u8RxBuf, devStat.u16RxBufLen);

				parseOkState = E_SIM808_PARSEOK_NONE;

				/** TODO: Add a "SIM808_ParseErrorResponse()" function */
			}
			else
			{
				SendDiagMessage("No flags were set...???");
				parseOkState = E_SIM808_PARSEOK_NONE;
				//pu8MsgToSend = "No flags were set...???\x0d\x0a";
				//serial2_write(pu8MsgToSend,(uint16_t)strlen(pu8MsgToSend));
			}
		}
		else if(devStat.bTimedOut || devStat.bRxBufOvf)
		{
			if(devStat.bTimedOut){devStat.bTimedOut = false; SendDiagMessage("devStat.bTimedOut"); }
			else{ devStat.bRxBufOvf = false; SendDiagMessage("devStat.bRxBufOvf"); }
			parseOkState = E_SIM808_PARSEOK_NONE;
		}
		break;

		/** Process to Enable the GPS feature of the SIM808 */
	case E_SIM808_ENABLE_GPS:

		break;

		/** Process to Disable the GPS feature of the SIM808 */
	case E_SIM808_DISABLE_GPS:

		break;

	default:

		break;
	}
}

/**
 *  Clear the u8RxBuf if the SIM808 isn't currently processing anything...
 */
uint8_t SIM808_ClearRxBuffer(void)
{
	uint8_t bOk = true;
	uint16_t i;

	if(devStat.bIsBusy){ return false; }
	else{ for(i=0;i<SIM808_TX_BUFFER_MAX;i++){ u8RxBuf[i] = 0; }}
	return bOk;
}

/**
 *  Send an AT command to the SIM808.
 *
 *  MAX size of the pu8Cmd data is 511 bytes
 */
uint8_t SIM808_SendAndReceive(const char *pu8Cmd, uint32_t u32Timeout)
{
	uint8_t bOk = true;
	uint16_t u16DataLen = (uint16_t)strlen(pu8Cmd);				// Calc. size of command
	uint16_t u16TxBufAvailable = serial3_write_buffer_free();	// Get the # of free bytes

	if(devStat.bIsBusy ||	// Don't transmit if SIM808 is busy
	  !devStat.bIsOn   ||	// Don't transmit if SIM808 is OFF
	  (u16DataLen > SIM808_TX_BUFFER_MAX))
	{
		SendDiagMessage("bIsBusy || !bIsOn");
		return false;
	} // Don't TX if command > 511 bytes

	// Don't transmit until the buffer is free to receive the command
	if(u16TxBufAvailable < u16DataLen)
	{
		SendDiagMessage("u16TxBufAvailable < u16DataLen");
		return false;
	}

	/** Clear the RX Buffer in preparation for a transaction */
	if(!SIM808_ClearRxBuffer())
	{
		SendDiagMessage("Could not ClearRxBuffer");
		return false;
	}

//	SetAlarm(ALARM_SIM808_RX_TIMEOUT, u32Timeout);
//	ClearAlarm(ALARM_SIM808_RX_TIMEOUT);
	u32ProcessTimeout = millis() + AT_RESP_MAX_DURATION;	// Timeout 10s from now

	/* Clear the status flags */
	devStat.bIsBusy = true;
	devStat.bError = false;
	devStat.bOk = false;
	devStat.bRespReady = false;
	devStat.bTimedOut = false;
	devStat.bRxBufOvf = false;
	devStat.u16RxBufLen = 0;

	/**
	 *  Data may arrive sporadically, slower then it can be pulled
	 *  from the buffer and processed.  Want to continually append
	 *  data into the rxBuffer until the timeout has occurred or
	 *  one of the termination strings shows up (OK, ERROR, etc)
	 */
	u16RxIndex = 0;

	//SendDiagMessage("Flags set. Message away.");

	serial3_clear();	// Clear the rxBuffer (tail = head)
	serial3_write(pu8Cmd,u16DataLen);	/* Send the command */

	state = E_SIM808_WAIT_FOR_RESP;

	return bOk;
}

void SendDiagMessageNoCRLF(const char * msg)
{
#ifdef ENABLE_SIM808_DIAGNOSTICS
	serial2_write(msg,(uint16_t)strlen(msg));
#endif
}

void SendDiagMessage(const char * msg)
{
#ifdef ENABLE_SIM808_DIAGNOSTICS
	const char * CRLF = "\x0d\x0a";
	serial2_write(msg,(uint16_t)strlen(msg));
	serial2_write(CRLF,(uint16_t)strlen(CRLF));
#endif
}

/**
 * Return GPS Enabled State Status
 *  T: if GPS is enabled. F: OT
 */
uint8_t SIM808_GetGpsIsOn(void)
{
	return devStat.bGpsIsOn;
}

/**
 *  Attempt to Enable or Disable the GPS feature of the SIM808.
 *  Returns T if possible, F Otherwise.
 *
 *  devStat flag bGpsEnabled must be checked for confirmation.
 */
uint8_t SIM808_EnableGps(uint8_t bEnable)
{
	uint8_t bOk = true;

	devStat.bCmdSuccess = false;
	if(bEnable)
	{
		if(!SIM808_SendAndReceive("AT+CGPSPWR=1\x0d\x0a", AT_RESP_MAX_DURATION)){ return false; }
		parseOkState = E_SIM808_PARSEOK_ENABLE_GPS;
	}
	else
	{
		if(!SIM808_SendAndReceive("AT+CGPSPWR=0\x0d\x0a", AT_RESP_MAX_DURATION)){ return false; }
		parseOkState = E_SIM808_PARSEOK_DISABLE_GPS;
	}

	return bOk;
}

/**
 * Begin process to get the GPS power/enable state.
 *  Returns T if the process can begin. Must read the devStat.bGpsIsOn flag
 *  to know the status. Also verify the devStat.bCmdSuccess flag to know the
 *  response arrived and was parsed properly.
 */
uint8_t SIM808_RequestGpsPwrStatus(void)
{
	uint8_t bOk = true;
	devStat.bCmdSuccess = false;
	if(!SIM808_SendAndReceive("AT+CGPSPWR?\x0d\x0a", AT_RESP_MAX_DURATION)){ return false; }
	parseOkState = E_SIM808_PARSEOK_QUERY_GPS_PWR_STATUS;
	return bOk;
}

/**
 * Begin process to get the GPS FIX Status.
 *  Returns T if the process can begin. Must read the .bGpsFix flags to know
 *  what state the GPS is currently in.
 *  	devStat.bGpsFix2dFix
 *      devStat.bGpsFix3dFix
 *      devStat.bGpsFixNotFix
 *      devStat.bGpsFixUnknown;
 *
 */
uint8_t SIM808_RequestGpsFixStatus(void)
{
	uint8_t bOk = true;
	devStat.bCmdSuccess = false;
	if(!SIM808_SendAndReceive("AT+CGPSSTATUS?\x0d\x0a",AT_RESP_MAX_DURATION)){ return false; }
	parseOkState = E_SIM808_PARSEOK_QUERY_FIX_STATUS;
	return bOk;
}

/**
 *  Begin process to get the GPS Location in RMC(Recommended Minimum Specific) Format.
 *  Example Output:
 *  +CGPSINF: 32,003820.000,A,4715.0965,N,09213.5966,W,0.000,0.00,020815,,,A
 *
 *  Returns: T if process can begin.
 *
 */
uint8_t SIM808_RequestGpsLocation(void)
{
	uint8_t bOk = true;
	devStat.bCmdSuccess = false;
	if(!SIM808_SendAndReceive("AT+CGPSINF=32\x0d\x0a",AT_RESP_MAX_DURATION)){ return false; }
	parseOkState = E_SIM808_PARSEOK_GPS_LOCATION;
	return bOk;
}

/**
 *  Search for expected SIM808 responses and set the appropriate actions and flags.
 */
void SIM808_ParseOkResponse(const char * pResponse)
{
	char  u8AsciiResult[16u];

	switch(parseOkState)
	{
	case E_SIM808_PARSEOK_NONE:

		break;

		/**
		 *  AT+CGPSPWR=0
		 *  OK<CR><LF>
		 */
	case E_SIM808_PARSEOK_DISABLE_GPS:
		if(NULL != strstr(pResponse,OK_STR))
		{
			devStat.bGpsIsOn = false;
			devStat.bCmdSuccess = true;
			parseOkState = E_SIM808_PARSEOK_NONE;
			SendDiagMessage("bGpsIsOn = false");
		}
		break;

		/**
		 *  AT+CGPSPWR=1
		 *  OK<CR><LF>
		 */
	case E_SIM808_PARSEOK_ENABLE_GPS:
		if(NULL != strstr(pResponse,OK_STR))
		{
			devStat.bGpsIsOn = true;
			devStat.bCmdSuccess = true;
			parseOkState = E_SIM808_PARSEOK_NONE;
			SendDiagMessage("bGpsIsOn = true");
		}
		break;

		/**
		 *  Parse the RMS GPS Sentence
		 */
	case E_SIM808_PARSEOK_GPS_LOCATION:

		if(SIM808_ParseGpsData(pResponse)){ devStat.bCmdSuccess = true; }
		parseOkState = E_SIM808_PARSEOK_NONE;

		break;

	case E_SIM808_PARSEOK_QUERY_FIX_STATUS:
		if(NULL != strstr(pResponse,GPS_FIX_STAT_3D_FIX)){
			devStat.bCmdSuccess = true;
			devStat.bGpsFix3dFix = true;
			devStat.bGpsFix2dFix = false;
			devStat.bGpsFixNotFix = false;
			devStat.bGpsFixUnknown = false;
			parseOkState = E_SIM808_PARSEOK_NONE;
			SendDiagMessage(".bGpsFix3dFix = true");
		}
		else if(NULL != strstr(pResponse,GPS_FIX_STAT_2D_FIX)){
			devStat.bCmdSuccess = true;
			devStat.bGpsFix3dFix = false;
			devStat.bGpsFix2dFix = true;
			devStat.bGpsFixNotFix = false;
			devStat.bGpsFixUnknown = false;
			parseOkState = E_SIM808_PARSEOK_NONE;
			SendDiagMessage(".bGpsFix2dFix = true");
		}
		else if(NULL != strstr(pResponse,GPS_FIX_STAT_NOT_FIX)){
			devStat.bCmdSuccess = true;
			devStat.bGpsFix3dFix = false;
			devStat.bGpsFix2dFix = false;
			devStat.bGpsFixNotFix = true;
			devStat.bGpsFixUnknown = false;
			parseOkState = E_SIM808_PARSEOK_NONE;
			SendDiagMessage(".bGpsFixNotFix = true");
		}
		else if(NULL != strstr(pResponse,GPS_FIX_STAT_UNKNOWN)){
			devStat.bCmdSuccess = true;
			devStat.bGpsFix3dFix = false;
			devStat.bGpsFix2dFix = false;
			devStat.bGpsFixNotFix = false;
			devStat.bGpsFixUnknown = true;
			parseOkState = E_SIM808_PARSEOK_NONE;
			SendDiagMessage(".bGpsFixUnknown = true");
		}
		break;

	case E_SIM808_PARSEOK_QUERY_GPS_PWR_STATUS:
		if(NULL != strstr(pResponse,GPS_PWR_STAT_ON)){
			devStat.bGpsIsOn = true;
			devStat.bCmdSuccess = true;
			parseOkState = E_SIM808_PARSEOK_NONE;
			SendDiagMessage("bGpsIsOn = true");
		}
		else if(NULL != strstr(pResponse,GPS_PWR_STAT_OFF)){
			devStat.bGpsIsOn = false;
			devStat.bCmdSuccess = true;
			parseOkState = E_SIM808_PARSEOK_NONE;
			SendDiagMessage("bGpsIsOn = false");
		}
		break;

	default:

		break;
	}
}

/**
 *	Convert the RMS GPS Sentence into a format that can be logged.
 *
 *
 *	Return T if there is no formatting issue.
 */
uint8_t SIM808_ParseGpsData(const char * pu8Pos)
{
	  const char * temp;
	  uint8_t bOk = true;
	  char *s[RMC_DELIM_COUNT];		/* array of string pointers */
	  uint8_t i;
	  char delim = ',';
	  uint16_t u16DelimCount;
	  uint16_t u16CopyLen;
	  char *ptr;			/* placeholder for strtol function */
	  char u8Segment[7u];

	  /**
	   *  Find the # of delimiters (",")
	   *  Count the # of commas
	   */
	  temp = pu8Pos;
	  for (u16DelimCount=0; temp[u16DelimCount]; temp[u16DelimCount]==',' ? u16DelimCount++ : *temp++);

	  /**
	   *  Check #1: For RMC Sentence there should be 12 delimiters
	   */
	  if(u16DelimCount != RMC_DELIM_COUNT){ SendDiagMessage("Not 12 delimiters"); return false; }

	  s[0] = strchr(pu8Pos,delim);	// 1st string starts at the first delimiter ",NNNN"

	  for(i=1;i<RMC_DELIM_COUNT;i++)
	  {
		  s[i] = strchr(&s[i-1][1],delim);	// Search after the delimiter
	  }

	  /**
	   * Process GPSPOS_pu8UtcTimeStr --> "000000.000"
	   */
	  u16CopyLen = s[1]-s[0];	// How long is the token?
	  if(u16CopyLen > 11u){ return false; }
	  memcpy(GPSPOS_pu8UtcTimeStr,&s[0][1],u16CopyLen-1); // Put the token in the placeholder
	  GPSPOS_pu8UtcTimeStr[u16CopyLen-1] = '\0'; // Append a NULL to complete the string
	  pos.fUtcTime = strtof(GPSPOS_pu8UtcTimeStr,NULL);

	  SendDiagMessageNoCRLF(" UTC:");SendDiagMessage(itoaX((uint32_t)(1000UL * pos.fUtcTime)));

	  /**
	   * Process GPSPOS_pu8StatusStr --> "V" or "A"
	   * "A" = Data Valid
	   */
	  u16CopyLen = s[2u]-s[1];	// How long is the token?
	  if(u16CopyLen > 2u){ return false; }
	  memcpy(GPSPOS_pu8StatusStr,&s[1][1],u16CopyLen-1); // Put the token in the placeholder
	  GPSPOS_pu8StatusStr[u16CopyLen-1] = '\0'; // Append a NULL to complete the string
	  if(GPSPOS_pu8StatusStr[0] != 'A'){ return false; }

	  /**
	   *  Process GPSPOS_pu8LatitudeStr: ddmm.mmmmmm
	   */
	  u16CopyLen = s[3u]-s[2u];	// How long is the token?
	  if(u16CopyLen > 12u){ return false; }
	  memcpy(GPSPOS_pu8LatitudeStr,&s[2u][1],u16CopyLen-1); // Put the token in the placeholder
	  GPSPOS_pu8LatitudeStr[u16CopyLen-1] = '\0'; // Append a NULL to complete the string
	  u8Segment[0] = GPSPOS_pu8LatitudeStr[0];
	  u8Segment[1] = GPSPOS_pu8LatitudeStr[1];
	  u8Segment[2] = '\0';
	  pos.u8Lat_dd = (uint8_t)strtol(u8Segment,&ptr,10u);
	  pos.fLat_minutes = strtof(&GPSPOS_pu8LatitudeStr[2],NULL);

	  SendDiagMessageNoCRLF("ddd:");SendDiagMessageNoCRLF(itoaX(pos.u8Lat_dd));
	  SendDiagMessageNoCRLF(" min:");SendDiagMessage(itoaX((uint32_t)(1000000UL * pos.fLat_minutes)));

	  u16CopyLen = s[4u]-s[3u];	// How long is the token?
	  if(u16CopyLen > 2u){ return false; }
	  memcpy(GPSPOS_pu8NSIndicatorStr,&s[3u][1],u16CopyLen-1); // Put the token in the placeholder
	  GPSPOS_pu8NSIndicatorStr[u16CopyLen-1] = '\0'; // Append a NULL to complete the string
	  if(GPSPOS_pu8NSIndicatorStr[0] == 'N'){ pos.u8Indicators |= 0x40u; }
	  else if(GPSPOS_pu8NSIndicatorStr[0] == 'S'){ pos.u8Indicators &= ~0x40u; }
	  else{ return false; }

	  u16CopyLen = s[5u]-s[4u];	// How long is the token?
	  if(u16CopyLen > 13u){ return false; }
	  memcpy(GPSPOS_pu8LongitudeStr,&s[4u][1],u16CopyLen-1); // Put the token in the placeholder
	  GPSPOS_pu8LongitudeStr[u16CopyLen-1] = '\0'; // Append a NULL to complete the string
	  u8Segment[0u] = GPSPOS_pu8LongitudeStr[0u];
	  u8Segment[1u] = GPSPOS_pu8LongitudeStr[1u];
	  u8Segment[2u] = GPSPOS_pu8LongitudeStr[2u];
	  u8Segment[3u] = '\0';
	  pos.u8Lon_dd = (uint8_t)strtol(u8Segment,&ptr,10u);
	  pos.fLon_minutes = strtof(&GPSPOS_pu8LongitudeStr[3u],NULL);

	  SendDiagMessageNoCRLF("ddd:");SendDiagMessageNoCRLF(itoaX(pos.u8Lon_dd));
	  SendDiagMessageNoCRLF(" min:");SendDiagMessage(itoaX((uint32_t)(1000000UL * pos.fLon_minutes)));

	  u16CopyLen = s[6u]-s[5u];	// How long is the token?
	  if(u16CopyLen > 2u){ return false; }
	  memcpy(GPSPOS_pu8EWIndicatorStr,&s[5u][1],u16CopyLen-1); // Put the token in the placeholder
	  GPSPOS_pu8EWIndicatorStr[u16CopyLen-1] = '\0'; // Append a NULL to complete the string
	  if(GPSPOS_pu8EWIndicatorStr[0] == 'E'){ pos.u8Indicators |= 0x20u; }
	  else if(GPSPOS_pu8EWIndicatorStr[0] == 'W'){ pos.u8Indicators &= ~0x20u; }
	  else{ return false; }

	  /**
	   *  T H E   R E S T   A R E   N O T   P R O C E S S E D
	   */
	  u16CopyLen = s[7u]-s[6u];	// How long is the token?
	  if(u16CopyLen > 7u){ return false; }
	  memcpy(GPSPOS_pu8SpeedKnotsStr,&s[6u][1],u16CopyLen-1); // Put the token in the placeholder
	  GPSPOS_pu8SpeedKnotsStr[u16CopyLen-1] = '\0'; // Append a NULL to complete the string

	  u16CopyLen = s[8u]-s[7u];	// How long is the token?
	  if(u16CopyLen > 7u){ return false; }
	  memcpy(GPSPOS_pu8DateStr,&s[7u][1],(size_t)u16CopyLen-1); // Put the token in the placeholder
	  GPSPOS_pu8DateStr[u16CopyLen-1] = '\0'; // Append a NULL to complete the string

	  u16CopyLen = s[9u]-s[8u];	// How long is the token?
	  if(u16CopyLen > 7u){ return false; }
	  memcpy(GPSPOS_pu8CourseStr,&s[8u][1],u16CopyLen-1); // Put the token in the placeholder
	  GPSPOS_pu8CourseStr[u16CopyLen-1] = '\0'; // Append a NULL to complete the string

	  SendDiagMessage(GPSPOS_pu8UtcTimeStr);
	  SendDiagMessage(GPSPOS_pu8StatusStr);
	  SendDiagMessage(GPSPOS_pu8LatitudeStr);
	  SendDiagMessage(GPSPOS_pu8NSIndicatorStr);
	  SendDiagMessage(GPSPOS_pu8LongitudeStr);
	  SendDiagMessage(GPSPOS_pu8EWIndicatorStr);
	  SendDiagMessage(GPSPOS_pu8SpeedKnotsStr);
	  SendDiagMessage(GPSPOS_pu8DateStr);
	  SendDiagMessage(GPSPOS_pu8CourseStr);

	  return bOk;
}

