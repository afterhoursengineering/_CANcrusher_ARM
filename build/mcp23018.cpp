/******************************************************************************
 **   mcp23018.cpp
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

#include "mcp23018.h"
#include <Wire.h>
#include "HardwareSerial.h"

#define MISC_INT_PIN	2u
#define MISC_RST_PIN	3u
#define VEH_IO_INT_PIN	16u
#define VEH_IO_RST_PIN	4u

#define MISC_ADDR		0x20u
#define VEH_IO_ADDR		0x27u

static uint8_t state_LED1 = 0;
static uint8_t state_LED2 = 0;
static uint8_t state_LED3 = 0;
static uint8_t state_LED4 = 0;

static uint8_t state_IO_PIN1 = 1u;
static uint8_t state_IO_PIN2 = 1u;
static uint8_t state_IO_PIN3 = 1u;
static uint8_t state_IO_PIN4 = 1u;
static uint8_t state_IO_PIN5 = 1u;
static uint8_t state_IO_PIN6 = 1u;
static uint8_t state_IO_PIN7 = 1u;
static uint8_t state_IO_PIN8 = 1u;

TwoWire i2c;

void microDelay(uint32_t delay);

void microDelay(uint32_t delay)
{
	uint32_t start = micros();
	while((micros()-start) < delay);
}

void MCP23018_Init(void)
{
	pinMode(MISC_INT_PIN, INPUT);	// MISC_INT configured as INPUT
	pinMode(MISC_RST_PIN, INPUT);	// MISC_RST configured as INPUT (OUTPUT LOW to RESET)
	pinMode(VEH_IO_INT_PIN, INPUT);
	pinMode(VEH_IO_RST_PIN, INPUT);

	MCP23018_Reset(DEV_MISC);
	MCP23018_Reset(DEV_VEH_IO);

	i2c.begin();

	MCP23018_Init_IOCON();

	// VEH IO INPUT pins (MISC_IN1:MISC_IN8) always inputs
	MCP23018_WRITE(DEV_VEH_IO, (uint8_t)IODIRA, (uint8_t)0xFFu);
}

void MCP23018_Reset(uint8_t device)
{
	switch(device)
	{
	case DEV_MISC:
		pinMode(MISC_RST_PIN, OUTPUT);
		digitalWrite(MISC_RST_PIN, LOW);	// Drive pin LOW
		microDelay(25u);
		pinMode(MISC_RST_PIN, INPUT);		// Release the pin
		microDelay(25u);
		break;

	case DEV_VEH_IO:
		pinMode(VEH_IO_RST_PIN, OUTPUT);
		digitalWrite(VEH_IO_RST_PIN, LOW);	// Drive pin LOW
		microDelay(25u);
		pinMode(VEH_IO_RST_PIN, INPUT);		// Release the pin
		microDelay(25u);
		break;
	}
}

void MCP23018_Init_IOCON(void)
{
	delayMicroseconds(1000u);

	i2c.beginTransmission((uint8_t)MISC_ADDR);
	i2c.write(0x0Au);  // select IOCON
    i2c.write(0xA0u);  // BANK=1, ADDR pointer NOT incremented
	i2c.endTransmission();

	delayMicroseconds(1000u);

	i2c.beginTransmission((uint8_t)VEH_IO_ADDR);
	i2c.write(0x0Au);  // select IOCON
    i2c.write(0xA0u);  // BANK=1, ADDR pointer NOT incremented
	i2c.endTransmission();

}

uint8_t MCP23018_get_state_LED(uint8_t led)
{
	switch(led)
	{
	case 1u:
		return state_LED1;
		break;
	case 2u:
		return state_LED2;
		break;
	case 3u:
		return state_LED3;
		break;
	case 4u:
		return state_LED4;
		break;
	}

}

uint8_t MCP23018_get_state_IO_PIN(uint8_t pin)
{
	switch(pin)
	{
	case IO_PIN1:
		return state_IO_PIN1;
		break;
	case IO_PIN2:
		return state_IO_PIN2;
			break;
	case IO_PIN3:
		return state_IO_PIN3;
			break;
	case IO_PIN4:
		return state_IO_PIN4;
			break;
	case IO_PIN5:
		return state_IO_PIN5;
			break;
	case IO_PIN6:
		return state_IO_PIN6;
			break;
	case IO_PIN7:
		return state_IO_PIN7;
			break;
	case IO_PIN8:
		return state_IO_PIN8;
			break;
	}
}

void MCP23018_WRITE(uint8_t device, uint8_t reg, uint8_t data)
{
	uint8_t addr;

	if(DEV_MISC == device){ addr = MISC_ADDR; }
	else{ addr = VEH_IO_ADDR; }

	i2c.beginTransmission(addr);
	i2c.write(reg);
	i2c.write(data);
	i2c.endTransmission();
}

uint8_t MCP23018_READ(uint8_t device, uint8_t reg)
{
	uint8_t addr;
	uint8_t result;

	if(DEV_MISC == device){ addr = MISC_ADDR; }
	else{ addr = VEH_IO_ADDR; }

	i2c.beginTransmission(addr);
	i2c.write(reg);  				// IODIRA.BANK1 Address
	i2c.endTransmission();

	i2c.requestFrom(addr,(uint8_t)1u);
	while (i2c.available() < 1u);
	result = i2c.receive();

	return result;
}

void MCP23018_LED(uint8_t led, uint8_t state)
{

	uint8_t portDir = 0;
	uint8_t val_GPIOA = 0;

	switch(led)
	{
	case 1u:
		portDir = MCP23018_READ(DEV_MISC, IODIRA);	// Get the current state of IODIRA
		val_GPIOA = MCP23018_READ(DEV_MISC, GPIOA);	// Get the current state of GPIOA

		if(state > 0)	// Turn ON LED
		{
			portDir &= ~IO0;		// Make GPAx an output
			val_GPIOA &= ~GP0;		// Drive pin low
			MCP23018_WRITE(DEV_MISC, (uint8_t)IODIRA, portDir);
			MCP23018_WRITE(DEV_MISC, (uint8_t)GPIOA, val_GPIOA);
			state_LED1 = 1u;
		}
		else // Turn OFF LED
		{
			portDir |= IO0;		// Make GPAx an input
			MCP23018_WRITE(DEV_MISC, (uint8_t)IODIRA, portDir);
			state_LED1 = 0u;
		}
		break;

	case 2u:
		portDir = MCP23018_READ(DEV_MISC, IODIRA);	// Get the current state of IODIRA
		val_GPIOA = MCP23018_READ(DEV_MISC, GPIOA);	// Get the current state of GPIOA

		if(state > 0)	// Turn ON LED
		{
			portDir &= ~IO1;		// Make GPAx an output
			val_GPIOA &= ~GP1;		// Drive pin low
			MCP23018_WRITE(DEV_MISC, (uint8_t)IODIRA, portDir);
			MCP23018_WRITE(DEV_MISC, (uint8_t)GPIOA, val_GPIOA);
			state_LED2 = 1u;
		}
		else // Turn OFF LED
		{
			portDir |= IO1;		// Make GPAx an input
			MCP23018_WRITE(DEV_MISC, (uint8_t)IODIRA, portDir);
			state_LED2 = 0u;
		}
		break;

	case 3u:
		portDir = MCP23018_READ(DEV_MISC, IODIRA);	// Get the current state of IODIRA
		val_GPIOA = MCP23018_READ(DEV_MISC, GPIOA);	// Get the current state of GPIOA

		if(state > 0)	// Turn ON LED
		{
			portDir &= ~IO2;		// Make GPAx an output
			val_GPIOA &= ~GP2;		// Drive pin low
			MCP23018_WRITE(DEV_MISC, (uint8_t)IODIRA, portDir);
			MCP23018_WRITE(DEV_MISC, (uint8_t)GPIOA, val_GPIOA);
			state_LED3 = 1u;
		}
		else // Turn OFF LED
		{
			portDir |= IO2;		// Make GPAx an input
			MCP23018_WRITE(DEV_MISC, (uint8_t)IODIRA, portDir);
			state_LED3 = 0u;
		}
		break;

	case 4u:
		portDir = MCP23018_READ(DEV_MISC, IODIRA);	// Get the current state of IODIRA
		val_GPIOA = MCP23018_READ(DEV_MISC, GPIOA);	// Get the current state of GPIOA

		if(state > 0)	// Turn ON LED
		{
			portDir &= ~IO3;		// Make GPAx an output
			val_GPIOA &= ~GP3;		// Drive pin low
			MCP23018_WRITE(DEV_MISC, (uint8_t)IODIRA, portDir);
			MCP23018_WRITE(DEV_MISC, (uint8_t)GPIOA, val_GPIOA);
			state_LED4 = 1u;
		}
		else // Turn OFF LED
		{
			portDir |= IO3;		// Make GPAx an input
			MCP23018_WRITE(DEV_MISC, (uint8_t)IODIRA, portDir);
			state_LED4 = 0u;
		}
		break;
	}
}

void MCP23018_SetCompPower(uint8_t state)
{
	uint8_t portDir = 0;
	uint8_t val_GPIO = 0;

	portDir = MCP23018_READ(DEV_MISC,IODIRB);
	val_GPIO = MCP23018_READ(DEV_MISC, GPIOB);

	if(COMP_ON == state)
	{
		// Make pin an output
		portDir &= ~IO1;
		MCP23018_WRITE(DEV_MISC, IODIRB, portDir);
		val_GPIO &= ~GP1;
		MCP23018_WRITE(DEV_MISC, GPIOB, val_GPIO);
	}
	else
	{
		// Make pin an INPUT
		portDir |= IO1;
		MCP23018_WRITE(DEV_MISC, IODIRB, portDir);
	}
}

/**
 *  Setting a pin means changing it to an INPUT since the MCP23018 has open
 *  collector outputs and requires a pull-up on a floating pin for a HIGH
 *
 *  Pull-up for the pin is enabled and pin configured as input...
 */
void MCP23018_SetPin(uint8_t device, uint8_t pinMask, uint8_t port)
{
	uint8_t GPPUx, IODIRx, devAddr;
	uint8_t portDir, portPullupState;

	if(GPA == port){ GPPUx = GPPUA; IODIRx = IODIRA; }
	else{ GPPUx = GPPUB; IODIRx = IODIRB; }

	portPullupState = MCP23018_READ(device, GPPUx);	// Read current pullup state
	portDir = MCP23018_READ(device, IODIRx);	// Read current port direction state

	portPullupState |= pinMask;		// Setting bit to 1 enables pull-up on pin
	portDir |= pinMask;				// Setting bit to 1 configures pin to INPUT

	MCP23018_WRITE(device, GPPUx, portPullupState);
	MCP23018_WRITE(device, IODIRx, portDir);

}

/******************************************************************************
 ** Function: MCP23018_ConfigAsInput(uint8_t device, uint8_t pinMask, uint8_t port)
 **
 ** Description: Configure the defined pin as an INPUT with no pullup enabled.
 ** Inputs:  uint8_t device --> DEV_MISC or DEV_VEH_IO
 **          uint8_t pinMask --> The pin to configure
 **          uint8_t port    --> GPIOA or GPIOB
 ** Outputs: NA
 **
 *****************************************************************************/
void MCP23018_ConfigAsInput(uint8_t device, uint8_t pinMask, uint8_t port)
{
	uint8_t GPPUx, IODIRx, devAddr;
	uint8_t portDir, portPullupState;

	if(GPA == port){ GPPUx = GPPUA; IODIRx = IODIRA; }
	else{ GPPUx = GPPUB; IODIRx = IODIRB; }

	portPullupState = MCP23018_READ(device, GPPUx);	// Read current pullup state
	portDir = MCP23018_READ(device, IODIRx);	// Read current port direction state

	portPullupState &= ~pinMask;	// Clearing bit to 0 disables pull-up on pin
	portDir |= pinMask;				// Setting bit to 1 configures pin to INPUT

	MCP23018_WRITE(device, GPPUx, portPullupState);
	MCP23018_WRITE(device, IODIRx, portDir);
}

/**
 *  Disable pull-up if enabled, configure pin as output, set to low (pull-down)
 */
void MCP23018_ClearPin(uint8_t device, uint8_t pinMask, uint8_t port)
{
	uint8_t GPIOx, IODIRx, GPPUx, devAddr;
	uint8_t portDir, portState, portPullupState;

	if(GPA == port){ GPIOx = GPIOA; IODIRx = IODIRA; GPPUx = GPPUA; }
	else{ GPIOx = GPIOB; IODIRx = IODIRB; GPPUx = GPPUB; }

	portState = MCP23018_READ(device, GPIOx);	// Read current PORT state
	portDir = MCP23018_READ(device, IODIRx);	// Read current port direction state
	portPullupState = MCP23018_READ(device, GPPUx);	// Read current port pullup state

	portState &= ~pinMask;		// Clearing bit to 0 pulls pin down when configured as an output
	portDir &= ~pinMask;		// Clearing bit to 0 configures pin to OUTPUT
	portPullupState &= ~pinMask;// Disable pull-up on the pin (0: disables)

	MCP23018_WRITE(device, GPIOx, portState);
	MCP23018_WRITE(device, GPPUx, portPullupState);
	MCP23018_WRITE(device, IODIRx, portDir);
}

void MCP23018_Set_IO(uint8_t ioPin, uint8_t dir, uint8_t state)
{
	static uint8_t portDirB = 0;
	static uint8_t val_GPIOB = 0;
	static uint8_t dirMask;
	static uint8_t pinMask;

	portDirB = MCP23018_READ(DEV_VEH_IO, IODIRB);	// Get the current state of IODIRB
	val_GPIOB = MCP23018_READ(DEV_VEH_IO, GPIOB);	// Get the current state of GPIOB

	switch(ioPin)
	{
	case IO_PIN1:
		dirMask = IO0;
		pinMask = GP0;
		if((IO_INPUT == dir) || (state > 0)){ state_IO_PIN1 = 1u; }
		else{ state_IO_PIN1 = 0; }
		break;
	case IO_PIN2:
		dirMask = IO1;
		pinMask = GP1;
		if((IO_INPUT == dir) || (state > 0)){ state_IO_PIN2 = 1u; }
		else{ state_IO_PIN2 = 0; }
		break;
	case IO_PIN3:
		dirMask = IO2;
		pinMask = GP2;
		if((IO_INPUT == dir) || (state > 0)){ state_IO_PIN3 = 1u; }
		else{ state_IO_PIN3 = 0; }
		break;
	case IO_PIN4:
		dirMask = IO3;
		pinMask = GP3;
		if((IO_INPUT == dir) || (state > 0)){ state_IO_PIN4 = 1u; }
		else{ state_IO_PIN4 = 0; }
		break;
	case IO_PIN5:
		dirMask = IO4;
		pinMask = GP4;
		if((IO_INPUT == dir) || (state > 0)){ state_IO_PIN5 = 1u; }
		else{ state_IO_PIN5 = 0; }
		break;
	case IO_PIN6:
		dirMask = IO5;
		pinMask = GP5;
		if((IO_INPUT == dir) || (state > 0)){ state_IO_PIN6 = 1u; }
		else{ state_IO_PIN6 = 0; }
		break;
	case IO_PIN7:
		dirMask = IO6;
		pinMask = GP6;
		if((IO_INPUT == dir) || (state > 0)){ state_IO_PIN7 = 1u; }
		else{ state_IO_PIN7 = 0; }
		break;
	case IO_PIN8:
		dirMask = IO7;
		pinMask = GP7;
		if((IO_INPUT == dir) || (state > 0)){ state_IO_PIN8 = 1u; }
		else{ state_IO_PIN8 = 0; }
		break;
	default:
		return;
		break;
	}

	if((IO_INPUT == dir) || (state > 0))
	{
		portDirB |= dirMask;
		MCP23018_WRITE(DEV_VEH_IO, (uint8_t)IODIRB, portDirB);
	}
	else
	{
		portDirB &= ~dirMask;
		val_GPIOB &= ~pinMask;
		MCP23018_WRITE(DEV_VEH_IO, (uint8_t)IODIRB, portDirB);
		MCP23018_WRITE(DEV_VEH_IO, (uint8_t)GPIOB, val_GPIOB);
	}
}


uint8_t MCP23018_Get_VEH_IO(uint8_t pinMask)
{
	uint8_t retVal = 0;

	/* Turn ON Comparator, READ PORT, Turn OFF COmparator */
	MCP23018_SetCompPower(COMP_ON);
	retVal = MCP23018_READ(DEV_VEH_IO,GPIOA);
	MCP23018_SetCompPower(COMP_OFF);

	if((retVal & pinMask) > 0){ retVal = 1u; }
	else{ retVal = 0; }

	return retVal;
}


void DEBUG_ReadAllRegisters(void)
{
	static uint8_t txBuf[100u];
	uint8_t i = 0;
	uint8_t j = 0;



	j = 0;
//	for(i=0; i<0x10u; i++)
//	{
//		txBuf[j] = i; j++;
//		txBuf[j] = MCP23018_READ(DEV_MISC, i); j++;
//		delayMicroseconds(50u);
//	}
//
//	for(i=0x10u; i<0x1Bu; i++)
//	{
//		txBuf[j] = i; j++;
//		txBuf[j] = MCP23018_READ(DEV_MISC, i); j++;
//		delayMicroseconds(50u);
//	}
//
//	txBuf[j] = 0xA5u; j++;
//	txBuf[j] = 0x5Au; j++;

	for(i=0; i<0x10u; i++)
	{
		txBuf[j] = i; j++;
		txBuf[j] = MCP23018_READ(DEV_VEH_IO, i); j++;
		delayMicroseconds(50u);
	}

	for(i=0x10u; i<0x1Bu; i++)
	{
		txBuf[j] = i; j++;
		txBuf[j] = MCP23018_READ(DEV_VEH_IO, i); j++;
		delayMicroseconds(50u);
	}

	txBuf[j] = 0xA5u; j++;
	txBuf[j] = 0x5Au; j++;
	serial2_write(txBuf,j);


}




