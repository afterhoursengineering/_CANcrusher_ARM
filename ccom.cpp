/******************************************************************************
 **   ccom.cpp
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
#include "Arduino.h"
#include "usb_rawhid.h"
#include "HardwareSerial.h"

/*---------------------------------------------------------------------------*/
/* Constant declarations                                                     */
/*---------------------------------------------------------------------------*/
#define BT		Serial2
#define USB		Serial

/*---------------------------------------------------------------------------*/
/* Type declarations                                                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Structure declarations                                                     */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/* Variable declarations                                                     */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/* Macro declarations                                                        */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Function prototypes                                                       */
/*---------------------------------------------------------------------------*/
bool USB_isReady(void);


/**
 *  \brief Initialize the USB and Serial2 drivers
 */
void CCOM_Init(void)
{
	BT.begin(921600u,SERIAL_8N1);
	USB.begin(9600);	/* Pretty sure this does nothing... USB enumerates if connected on its own */
}

/**
 * \brief Returns whether USB enumerated with a computer and is connected.
 * @return T:CONNECTED, F:NOT CONNECTED
 */
bool USB_isReady(void)
{
	return USB;	/* Returns the state of the usb_configuration value */
}

void CCOM_Debug(const char * msg, boolean bCrlf)
{

}






