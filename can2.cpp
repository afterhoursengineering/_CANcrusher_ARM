/*
 * can2.cpp
 *
 *  Created on: Jul 16, 2015
 *      Author: David
 */

#include "can2.h"
#include <SPI.h>
#include "HardwareSerial.h"
#include "SdLog.h"


#define CAN2_CS				5u			// PTD7(21) is CAN2 chip select
#define CAN2_INT			6u			// PTD4(6) is CAN2 INT line

/* Uncomment this to dump all SPI traffic out the USART port (Bluetooth USART) */
//#define DEBUG_CAN2_SPI	1u

static uint8_t currentBaudRate = 0;
static CAN_msg_t msg0;
static CAN_msg_t msg1;

static bool bCanDataIsReady = false;
static bool bRx0DataReady = false;
static bool bRx1DataReady = false;

SPIClass can2;// = new SPIClass;

void CAN2_Process_RX0(void);
void CAN2_Process_RX1(void);
void CAN2_SpiFlushRxFifo(void);

void CAN2_Init(void)
{
	pinMode(CAN2_CS, OUTPUT);	// CS configured as an output
	digitalWrite(CAN2_CS,HIGH);	// Initialize CS as high
	pinMode(12u,INPUT_PULLUP);	// MISO configured with internal pullup
	pinMode(CAN2_INT,INPUT_PULLUP);	// CAN2_INT configured as input with internal pullup
	can2.begin();
}

void CAN2_Task(void)
{
	uint16_t rxLen;
	uint16_t i;
	uint8_t rxBuffer[32];
	uint8_t txBuffer[32];
	uint8_t intResult = 0;

	// Determine if an interrupt occurred
	if(!CAN2_get_INT_Pin())
	{
		// What interrupt occurred
		intResult = CAN2_getIntStatus();
		switch(intResult){
		case CAN_INT_NONE:

			break;
		case CAN_INT_ERR:

			break;
		case CAN_INT_WAK:

			break;
		case CAN_INT_TXB0:
			CAN2_bitModify(CANINTF, ~TX0IF, TX0IF);	// Clear the flag
			break;
		case CAN_INT_TXB1:
			CAN2_bitModify(CANINTF, ~TX1IF, TX1IF);	// Clear the flag
			break;
		case CAN_INT_TXB2:
			CAN2_bitModify(CANINTF, ~TX2IF, TX2IF);	// Clear the flag
			break;
		case CAN_INT_RXB0:
			CAN2_getRxBuf(0,&msg0);
			bCanDataIsReady = true;
			bRx0DataReady = true;
			CAN2_Process_RX0();
			CAN2_bitModify(CANINTF, ~RX0IF, RX0IF);	// Clear the flag
			break;
		case CAN_INT_RXB1:
			CAN2_getRxBuf(1u,&msg1);
			bCanDataIsReady = true;
			bRx1DataReady = true;
			CAN2_Process_RX1();
			CAN2_bitModify(CANINTF, ~RX1IF, RX1IF);	// Clear the flag
			break;
		}
	}
}

uint8_t CAN2_DataIsReady( struct CAN_msg_t *msg)
{
	uint8_t retVal = false;
	uint8_t i;

	if(bCanDataIsReady)
	{
		retVal = true;
		bCanDataIsReady = false;
		if(bRx0DataReady)
		{
			bRx0DataReady = false;
			if(msg0.stat & STAT_EXT){ msg0.id &= (uint32_t)ID_29BIT_MASK; }
			else{ msg0.id &= (uint32_t)ID_11BIT_MASK; }
			msg->id = msg0.id;
			msg->len = msg0.len;
			msg->stat = msg0.stat;
			//for(i=0;i<msg0.len;i++){ msg->buf[i] = msg0.buf[i]; }
			for(i=0;i<CAN_DATA_BUF_SIZE;i++){ msg->buf[i] = msg0.buf[i]; }
		}
		else if(bRx1DataReady)
		{
			bRx1DataReady = false;
			if(msg1.stat & STAT_EXT){ msg1.id &= (uint32_t)ID_29BIT_MASK; }
			else{ msg1.id &= (uint32_t)ID_11BIT_MASK; }
			msg->id = msg1.id;
			msg->len = msg1.len;
			msg->stat = msg1.stat;
			//for(i=0;i<msg1.len;i++){ msg->buf[i] = msg1.buf[i]; }
			for(i=0;i<CAN_DATA_BUF_SIZE;i++){ msg->buf[i] = msg1.buf[i]; }
		}
		else
		{
			return false;
		}
	}

	return retVal;
}

void CAN2_Process_RX0(void)
{
	uint16_t i;
	uint8_t txBuffer[15u];

	for(i=0;i<13u;i++){ txBuffer[i] = 0; }

	if(msg0.stat & STAT_EXT){	// Message has an extended ID (29-bit)
		txBuffer[0u] = (uint8_t)(msg0.id >> 24u);
		txBuffer[1u] = (uint8_t)(msg0.id >> 16u);
		txBuffer[2u] = (uint8_t)(msg0.id >> 8u);
		txBuffer[3u] = (uint8_t)msg0.id;
	}
	else{
		txBuffer[0u] = 0x00;	//(uint8_t)(msg0.id >> 24u);
		txBuffer[1u] = 0x00; 	//(uint8_t)(msg0.id >> 16u);
		txBuffer[2u] = 0x07u & (uint8_t)(msg0.id >> 8u);
		txBuffer[3u] = (uint8_t)msg0.id;
	}

	txBuffer[4u] = msg0.stat;
	for(i=0;i<msg0.len;i++){ txBuffer[i+5u] = msg0.buf[i]; }
	txBuffer[13u] = 0xA5u;
	txBuffer[14u] = 0x5Au;
	serial2_write(txBuffer,15u);
}

void CAN2_Process_RX1(void)
{
	uint16_t i;
	uint8_t txBuffer[15u];

	for(i=0;i<13u;i++){ txBuffer[i] = 0; }

	if(msg0.stat & STAT_EXT){	// Message has an extended ID (29-bit)
		txBuffer[0u] = (uint8_t)(msg1.id >> 24u);
		txBuffer[1u] = (uint8_t)(msg1.id >> 16u);
		txBuffer[2u] = (uint8_t)(msg1.id >> 8u);
		txBuffer[3u] = (uint8_t)msg1.id;
	}
	else{
		txBuffer[0u] = 0x00;	//(uint8_t)(msg0.id >> 24u);
		txBuffer[1u] = 0x00; 	//(uint8_t)(msg0.id >> 16u);
		txBuffer[2u] = 0x07u & (uint8_t)(msg1.id >> 8u);
		txBuffer[3u] = (uint8_t)msg1.id;
	}

	txBuffer[4u] = msg1.stat;
	for(i=0;i<msg1.len;i++){ txBuffer[i+5u] = msg1.buf[i]; }
	txBuffer[13u] = 0xA5u;
	txBuffer[14u] = 0x5Au;
	serial2_write(txBuffer,15u);
}

uint8_t CAN2_get_INT_Pin(void)
{
	if(0 == digitalRead(CAN2_INT)){ return 0; }
	else{ return 1u; }
}

void CAN2_MCP2515_Reset(void)
{
	uint8_t response;
#ifdef DEBUG_CAN2_SPI
	uint8_t txBuf[4];
#endif

	can2.beginTransaction(SPISettings(MCP2515_MAXSPEED,MSBFIRST,SPI_MODE0));
	digitalWrite(CAN2_CS,LOW);		// pull down CS
	response = can2.transfer(MCP2515_RESET);
	digitalWrite(CAN2_CS,HIGH);		// Release the CS
	can2.endTransaction();

#ifdef DEBUG_CAN2_SPI
	txBuf[0] = MCP2515_RESET;
	txBuf[1] = response;
	txBuf[2] = 0xA5u;
	txBuf[3] = 0x5A;
	serial2_write(txBuf,4);
#endif
}

/**
 *  Set the bus baudrate. Options are as follows:
 *  33333  -> 33.333kbps
 *  50000  -> 50kbps
 *  83333  -> 83.333kbps
 *  100000 -> 100kbps
 *  125000 -> 125kbps
 *  250000 -> 250kbps
 *  500000 -> 500kbps
 *
 *  Tq for each configuration is currently set at 20.
 *  SJW1:SJW0 = 00 = 1xTq = 20
 *  Calculation based on a 20MHz OSC.
 *
 *  Sets the CNF1, CNF2, CNF3 registers.
 *  Following WRITE, function READs back the expected values.
 *  Returns T if all values set properly.
 *  Returns F if one or more of values not set properly or
 *  if un undefined baud is selected.
 */
uint8_t CAN2_SetBaud(CANbaud_t baud)
{
	uint8_t retVal = false;
	uint8_t txBuf[22];
	uint8_t CNF1_val,CNF2_val,CNF3_val;


	switch(baud){
	case BAUD_33p333kbps:
		CNF1_val = 0x0Eu;
		CNF2_val = 0xBAu;
		CNF3_val = 0x07u;
		break;

	case BAUD_50kbps:
		CNF1_val = 0x07u;
		CNF2_val = 0xBAu;
		CNF3_val = 0x07u;
		break;

	case BAUD_83p333kbps:
		CNF1_val = 0x05u;
		CNF2_val = 0xBAu;
		CNF3_val = 0x07u;
		break;

	case BAUD_100kbps:
		CNF1_val = 0x04u;
		CNF2_val = 0xBAu;
		CNF3_val = 0x07u;
		break;

	case BAUD_125kbps:
		CNF1_val = 0x03u;
		CNF2_val = 0xBAu;
		CNF3_val = 0x07u;
		break;

	case BAUD_250kbps:
		CNF1_val = 0x01u;
		CNF2_val = 0xBAu;
		CNF3_val = 0x07u;
		break;

	case BAUD_500kbps:
		CNF1_val = 0x00u;
		CNF2_val = 0xBAu;
		CNF3_val = 0x07u;
		break;

	default:
		return retVal;
		break;
	}

	// Write the CNFx registers

	can2.beginTransaction(SPISettings(MCP2515_MAXSPEED,MSBFIRST,SPI_MODE0));

	CAN2_SpiFlushRxFifo();

	digitalWrite(CAN2_CS,LOW);		// pull down CS
	txBuf[1] = can2.transfer(MCP2515_WRITE);
	txBuf[3] = can2.transfer(CNF3);	// Addr: 0x28
	txBuf[5] = can2.transfer(CNF3_val);	// Setting for CNF3 register
	/**
	 *  MCP2515 allows writing and reading consecutive registers if the CS is
	 *  held low.  By addressing the lowest register first (CNF3 = 0x28), we can
	 *  write all 3 CNF registers without sending WRITE and ADDR commands for
	 *  each one.
	 */
	txBuf[7] = can2.transfer(CNF2_val);	// Setting for CNF2 register
	txBuf[9] = can2.transfer(CNF1_val);	// Setting for CNF1 register
	digitalWrite(CAN2_CS,HIGH);		// Release the CS
	can2.endTransaction();

	// Create a small 2us delay
	uint32_t start = micros();
	while((micros()-start) < 2u);

	// READ back all the CNFx registers for VERIFICATION

	can2.beginTransaction(SPISettings(MCP2515_MAXSPEED,MSBFIRST,SPI_MODE0));
	digitalWrite(CAN2_CS,LOW);		// pull down CS
	txBuf[11] = can2.transfer(MCP2515_READ);
	txBuf[13] = can2.transfer(CNF3);	// Addr: 0x28
	txBuf[15] = can2.transfer(DUMMY);	// READ Setting for CNF3 register
	txBuf[17] = can2.transfer(DUMMY);	// READ Setting for CNF2 register
	txBuf[19] = can2.transfer(DUMMY);	// READ Setting for CNF1 register
	digitalWrite(CAN2_CS,HIGH);		// Release the CS
	can2.endTransaction();

	if((CNF3_val == txBuf[15]) && (CNF2_val == txBuf[17]) && (CNF1_val == txBuf[19]))
	{
		currentBaudRate = baud;
		retVal = true;
	}

#ifdef DEBUG_CAN2_SPI
	txBuf[0]  = MCP2515_WRITE;
	txBuf[2]  = CNF3;
	txBuf[4]  = CNF3_val;
	txBuf[6]  = CNF2_val;
	txBuf[8]  = CNF1_val;
	txBuf[10] = MCP2515_READ;
	txBuf[12] = CNF3;
	txBuf[14] = DUMMY;
	txBuf[16] = DUMMY;
	txBuf[18] = DUMMY;
	txBuf[20] = 0xA5u;
	txBuf[21] = 0x5Au;
	serial2_write(txBuf,22u);
#endif

	return retVal;
}

/**
 *  NOTE: Only certain registers can be bit-modified. All others will
 *        have the mask automatically changed to 0xFF so the register
 *        will be written as a full byte instead.
 *
 *        mask --> Determines which bytes will change
 *                 Ex: mask 0b01101010
 *                     bits 6,5,3,and 1 will be editable.
 *        value--> The value the selected bits will change to.
 *                 Ex: value =  0b01001000
 *                 prev_value = 0b10000111
 *                 ------------------------
 *                 new_value  = 0b11001100
 */
uint8_t CAN2_bitModify( uint8_t reg, uint8_t value, uint8_t mask  )
{
	uint8_t retVal = false;
	uint8_t txBuf[16u];
	uint8_t u8Data = 0;

	can2.beginTransaction(SPISettings(MCP2515_MAXSPEED,MSBFIRST,SPI_MODE0));

	CAN2_SpiFlushRxFifo();

	digitalWrite(CAN2_CS,LOW);		// pull down CS
	txBuf[1u] = can2.transfer(MCP2515_BIT_MODIFY);
	txBuf[3u] = can2.transfer(reg);	// Choose which Register to modify
	txBuf[5u] = can2.transfer(mask);	// MASK 1's select which bits to change
	txBuf[7u] = can2.transfer(value);	// value for each selected bit to change to
	digitalWrite(CAN2_CS,HIGH);		// Release the CS
	can2.endTransaction();

	// Create a small 2us delay
	uint32_t start = micros();
	while((micros()-start) < 2u);

	// Verify the result

	can2.beginTransaction(SPISettings(MCP2515_MAXSPEED,MSBFIRST,SPI_MODE0));
	digitalWrite(CAN2_CS,LOW);		// pull down CS
	txBuf[9u]  = can2.transfer(MCP2515_READ);
	txBuf[11u] = can2.transfer(reg);	// Choose which Register to READ
	u8Data = can2.transfer(DUMMY);	// DUMMY bye to clock out value
	digitalWrite(CAN2_CS,HIGH);		// Release the CS
	can2.endTransaction();

	if((mask & u8Data) == (mask & value)){ retVal = true; }

#ifdef DEBUG_CAN2_SPI
	txBuf[0]   = MCP2515_BIT_MODIFY;
	txBuf[2u]  = reg;
	txBuf[4u]  = mask;
	txBuf[6u]  = value;
	txBuf[8u]  = MCP2515_READ;
	txBuf[10u] = reg;
	txBuf[12u] = DUMMY;
	txBuf[13u] = u8Data;
	txBuf[14u] = 0xA5u;
	txBuf[15u] = 0x5Au;
	serial2_write(txBuf,16u);
#endif
	return retVal;
}

/**
 *  Read common STATUS bits
 *  Bit  Flag
 *   7   CANINTF.TX2IF
 *   6   TXB2CNTRL.TXREQ
 *   5   CANINTF.TX1IF
 *   4   TXB1CNTRL.TXREQ
 *   3   CANINTF.TX0IF
 *   2   TXB0CNTRL.TXREQ
 *   1   CANINTFL.RX1IF
 *   0   CANINTF.RX0IF
 *
 *   Return: Result
 */
uint8_t CAN2_readStatus(void)
{
	static uint8_t txBuf[8];
	uint8_t retVal = 0;

	can2.beginTransaction(SPISettings(MCP2515_MAXSPEED,MSBFIRST,SPI_MODE0));

	CAN2_SpiFlushRxFifo();

	digitalWrite(CAN2_CS,LOW);		// pull down CS
	txBuf[1u] = can2.transfer(MCP2515_STATUS);
	retVal = can2.transfer(DUMMY);	// Choose which Register to READ
	txBuf[5u] = can2.transfer(DUMMY);	// DUMMY bye to clock out value
	digitalWrite(CAN2_CS,HIGH);		// Release the CS
	can2.endTransaction();

#ifdef DEBUG_CAN2_SPI
	txBuf[0]   = MCP2515_STATUS;
	txBuf[2u]  = DUMMY;
	txBuf[3u]  = retVal;
	txBuf[4u]  = DUMMY;
	txBuf[6u] = 0xA5u;
	txBuf[7u] = 0x5Au;
	serial2_write(txBuf,8u);
#endif

	return retVal;
}

/**
 *  Change the CAN controller Mode:
 *  MODE_CONFIG
 *  MODE_NORMAL
 *  MODE_SLEEP
 *  MODE_LISTEN_ONLY
 *  MODE_LOOPBACK
 */
uint8_t CAN2_setMode(CANmode_t mode)
{
	uint8_t retVal = false;
	switch(mode)
	{
	case MODE_CONFIG:		// '100xxxxx
		if(CAN2_bitModify(CANCTRL,0x80u,0xE0u)){ retVal = true; }
		break;

	case MODE_NORMAL:		// '000xxxxx
		if(CAN2_bitModify(CANCTRL,0x00u,0xE0u)){ retVal = true; }
		break;

	case MODE_SLEEP:		// '001xxxxx
		if(CAN2_bitModify(CANCTRL,0x20u,0xE0u)){ retVal = true; }
		break;

	case MODE_LISTEN_ONLY:	// '011xxxxx
		if(CAN2_bitModify(CANCTRL,0x60u,0xE0u)){ retVal = true; }
		break;

	case MODE_LOOPBACK:		// '010xxxxx
		if(CAN2_bitModify(CANCTRL,0x40u,0xE0u)){ retVal = true; }
		break;

	default:

		break;
	}
	return retVal;
}

/**
 *  Set the pin state of the RX1BF pin of the MCP2515
 *  RXnBF_OFF --> High Impedance state
 *  RXnBF_HI  --> 3.3V
 *  RXnBF_LO  --> 0V
 *
 *  SN65HVD230 Transceiver RS Pin
 *  0 = High Speed Mode. TX/RX Enabled. ~10mA Quiescent
 *  1 = Listen-only Mode. RX only enabled. ~370uA Quiescent
 *
 *	TH8056KDC-AAA-008-RE Mode Pins (CAN3 ONLY) SWCAN
 *	M0 M1  Mode
 *	------------
 *	0  0   Sleep Mode
 *	0  1   High-Voltage Wakeup
 *	1  0   High Speed
 *	1  1   Normal Mode
 */
uint8_t CAN2_RX1BF_Set(CANpinState_t state)
{
	uint8_t retVal = false;
	uint8_t mask = 0x2Au;
	switch(state)
	{
	case RXnBF_OFF:	// mask:0x2A, val:0x00
		if(CAN2_bitModify(BFPCTRL,0x00u,mask)){ retVal = true; }
		break;

	case RXnBF_HI:	// mask:0x2A, val:0x28
		if(CAN2_bitModify(BFPCTRL,0x28u,mask)){ retVal = true; }
		break;

	case RXnBF_LO:	// mask:0x2A, val:0x08
		if(CAN2_bitModify(BFPCTRL,0x08u,mask)){ retVal = true; }
		break;

	default:
		return retVal;
		break;
	}
	return retVal;
}


/**
 *  Set the pin state of the RX0BF pin of the MCP2515
 *  RXnBF_OFF --> High Impedance state
 *  RXnBF_HI  --> 3.3V
 *  RXnBF_LO  --> 0V
 *
 *	TH8056KDC-AAA-008-RE Mode Pins (CAN3 ONLY) SWCAN
 *	M0 M1  Mode
 *	------------
 *	0  0   Sleep Mode
 *	0  1   High-Voltage Wakeup
 *	1  0   High Speed
 *	1  1   Normal Mode
 */
uint8_t CAN2_RX0BF_Set(CANpinState_t state)
{
	uint8_t retVal = false;
	uint8_t mask = 0x15u;
	switch(state)
	{
	case RXnBF_OFF:	// mask:0x15, val:0x00
		if(CAN2_bitModify(BFPCTRL,0x00u,mask)){ retVal = true; }
		break;

	case RXnBF_HI:	// mask:0x15, val:0x14
		if(CAN2_bitModify(BFPCTRL,0x14u,mask)){ retVal = true; }
		break;

	case RXnBF_LO:	// mask:0x15, val:0x04
		if(CAN2_bitModify(BFPCTRL,0x04u,mask)){ retVal = true; }
		break;

	default:
		return retVal;
		break;
	}
	return retVal;
}

/**
 *  Load Tx Buffer
 *  buf:		TXBx (0,1,2)
 *  priority: 	(0-3) 0: lowest, 3: highest
 *  rtr:		(0,1) 0=transmitted as data frame, 1=transmitted as remote transmit request
 *  id:			11-bit or 29-bit CAN Arbitration ID (will only use bits 0-28 of id)
 *  dlc:		# of bytes to send (1-8)
 *  *data:		pointer to the buffer of data bytes (up to 8 bytes)
 *
 *  Returns  Description
 *  --------------------
 *        0  No Error. Data Loaded. TXREQ bit set
 *        1  ERROR: TXBnCTRL.TXREQ bit was set. Could not load message for transmission
 *        2  ERROR: Undefined message format (one of the arguments was out of range)
 *        3  ERROR: Data did not load correctly.
 */
uint8_t CAN2_LoadTxBuffer(uint8_t buf, uint8_t priority, uint8_t rtr, uint32_t id, uint8_t dlc, uint8_t *data )
{
	uint8_t retVal =  0;
	uint8_t status;
	uint8_t sidh, sidl, eid8, eid0;
	uint8_t TXBnDLC;
	uint8_t TXBnCTRL;
	uint8_t TXBnCTRL_data;
	uint8_t writeCmd;
	uint8_t readAddr;
	uint8_t txBuf[33u];
	uint8_t i = 0;

	// Range Verification
	if((buf > 2u)||(priority > 3u)||(rtr > 1u)||(dlc > 8u)){ return 2u; }

	// Read TXBnCTRL.TXREQ bit
	status = CAN2_readStatus();
	switch(buf)
	{
	case 0:
		if(status & 0x04u){ return 1; } // TXB0CNTRL.TXREQ is set
		writeCmd = MCP2515_LOAD_TX_BUF_0;
		readAddr = TXB0CTRL + 1u;
		TXBnCTRL = TXB0CTRL;
		break;
	case 1u:
		if(status & 0x10u){ return 1; } // TXB1CNTRL.TXREQ is set
		writeCmd = MCP2515_LOAD_TX_BUF_1;
		readAddr = TXB1CTRL + 1u;
		TXBnCTRL = TXB1CTRL;
		break;
	case 2u:
		if(status & 0x40u){ return 1; } // TXB2CNTRL.TXREQ is set
		writeCmd = MCP2515_LOAD_TX_BUF_2;
		readAddr = TXB2CTRL + 1u;
		TXBnCTRL = TXB2CTRL;
		break;
	}

	// Data is in range and no TXREQ bits are set... Load the data.

	/* parse the id bits */
	if(id & 0xFFFFF800u){	// Extended Frame
		sidh = (uint8_t)(id >> 3u);
		sidl = (uint8_t)(id << 5u) | 0x80u | (uint8_t)(id >> 27u);
		eid0 = (uint8_t)(id >> 11u);
		eid8 = (uint8_t)(id >> 19u);
	}
	else{	// Standard Frame
		sidh = (uint8_t)(id >> 3u);
		sidl = (uint8_t)(id << 5u);
		eid0 = 0x00u;
		eid8 = 0x00u;
	}

	/* OR the dlc with the rtr bit shifted to bit 6 */
	TXBnDLC = dlc | (rtr << 6u);

	// Begin WRITING the TXBn Buffers

	can2.beginTransaction(SPISettings(MCP2515_MAXSPEED,MSBFIRST,SPI_MODE0));

	CAN2_SpiFlushRxFifo();

	digitalWrite(CAN2_CS,LOW);		// pull down CS
	can2.transfer(writeCmd);	// Point to the correct TX Buffer. Starts at TXBnSIDH
	can2.transfer(sidh);
	can2.transfer(sidl);
	can2.transfer(eid8);
	can2.transfer(eid0);
	can2.transfer(TXBnDLC);	// dlc OR'd with the rtr bit
	for(i=0; i<dlc; i++){
		can2.transfer(data[i]);
	}
	digitalWrite(CAN2_CS,HIGH);		// Release the CS
	can2.endTransaction();

#ifdef DEBUG_CAN2_SPI
	txBuf[0]   = writeCmd;
	txBuf[1u]  = sidh;
	txBuf[2u]  = sidl;
	txBuf[3u]  = eid8;
	txBuf[4u]  = eid0;
	txBuf[5u]  = TXBnDLC;
	for(i=0;i<8;i++){ txBuf[i+6u] = 0; }
	for(i=0;i<dlc;i++){ txBuf[i+6u] = data[i]; }
	txBuf[14u] = 0xA5u;
	txBuf[15u] = 0x5Au;
	//serial2_write(txBuf,8u);
#endif

	// Create a small 2us delay
	uint32_t start = micros();
	while((micros()-start) < 2u);

#ifdef DEBUG_CAN2_SPI
	txBuf[16u] = MCP2515_READ;
	txBuf[17u] = readAddr;
#endif

	// Verify the data...
	can2.beginTransaction(SPISettings(MCP2515_MAXSPEED,MSBFIRST,SPI_MODE0));
	digitalWrite(CAN2_CS,LOW);		// pull down CS
	can2.transfer(MCP2515_READ);	// Read the TXBn buffers
	can2.transfer(readAddr);
	txBuf[18u] = can2.transfer(DUMMY);	// sidh
	txBuf[19u] = can2.transfer(DUMMY);	// sidl
	txBuf[20u] = can2.transfer(DUMMY);	// eid8
	txBuf[21u] = can2.transfer(DUMMY);	// eid0
	txBuf[22u] = can2.transfer(DUMMY);	// TXBnDLC
	for(i=0; i<dlc; i++){
		txBuf[i+23u] = can2.transfer(DUMMY);
	}
	digitalWrite(CAN2_CS,HIGH);		// Release the CS
	can2.endTransaction();

	if((txBuf[18u] != sidh) ||
	   (txBuf[19u] != sidl) ||
	   (txBuf[20u] != eid8) ||
	   (txBuf[21u] != eid0) ||
	   (txBuf[22u] != TXBnDLC) ){ return 3u; } // Data did not load properly.
	for(i=0; i<dlc; i++){
		if(txBuf[i+23u] != data[i]){ return 3u; } // Data did not load properly
	}

#ifdef DEBUG_CAN2_SPI
	txBuf[31u] = 0xA5u;
	txBuf[32u] = 0x5Au;
	serial2_write(txBuf,33u);
#endif

	// Set the TXREQ bit and the message priority
	TXBnCTRL_data = 0 | 0x08u | priority;	// TXREG = 1, TXP1:TXP0 = priority
	CAN2_bitModify(TXBnCTRL,TXBnCTRL_data,0x0Bu);	// mask = 0x0B = 0b00001011

	return retVal;
}

/**
 *
 */
uint8_t CAN2_getRxBuf(uint8_t buf, struct CAN_msg_t *msg)
{
	uint8_t stat = 0;
	uint32_t id = 0;
	uint8_t retVal = true;
	uint8_t i;
	uint8_t sidh,sidl,eid8,eid0,dlc;
	uint8_t ext = 0;
#ifdef DEBUG_CAN2_SPI
	uint8_t txBuf[16u];
#endif

	// clear the data bytes
	for(i=0;i<8u;i++){ msg->buf[i] = 0; }

	// Begin READing the RXBn Buffers
	can2.beginTransaction(SPISettings(MCP2515_MAXSPEED,MSBFIRST,SPI_MODE0));

	CAN2_SpiFlushRxFifo();

	digitalWrite(CAN2_CS,LOW);		// pull down CS
	if(0 == buf){ can2.transfer(MCP2515_READ_RX_BUF_0); }
	else{ can2.transfer(MCP2515_READ_RX_BUF_1); }
	sidh = can2.transfer(DUMMY);
	sidl = can2.transfer(DUMMY);
	eid8 = can2.transfer(DUMMY);
	eid0 = can2.transfer(DUMMY);
	dlc = can2.transfer(DUMMY);
	for(i=0;i<dlc;i++){ msg->buf[i] = can2.transfer(DUMMY); }
	digitalWrite(CAN2_CS,HIGH);		// Release the CS
	can2.endTransaction();

	// Pack the ID
	id = ((uint32_t)(sidl & 0x03u) << 27u) | 	// EID17:EID16 are bits 1:0 of SIDL
		 ((uint32_t)eid8 << 19u) | 			    // EID15:EID8 is in byte EID8
		 ((uint32_t)eid0 << 11u) |              // EID7:EID0 is in byte EID0
		 ((uint32_t)sidh << 3u)  |              // SID10:SID3 is in byte sidh
		 ((uint32_t)(sidl & 0xE0u) >> 5u);      // SID2:SID0 are in bits 7:5 of SIDL

	// Update the stat field
	if(sidl & 0x08u){ ext = CAN_ID_29BIT; } else{ ext = CAN_ID_11BIT; }
	stat = CAN_CH_CAN2 | CAN_RX | currentBaudRate | ext;

	msg->id = id;
	msg->len = dlc;
	msg->stat = stat;

#ifdef DEBUG_CAN2_SPI
	// id id id id stat dlc buf[0] buf[1] ... buf[7] 0xA5 0x5A
	txBuf[0]   = (uint8_t)(id >> 24u);
	txBuf[1u]  = (uint8_t)(id >> 16u);
	txBuf[2u]  = (uint8_t)(id >> 8u);
	txBuf[3u]  = (uint8_t)id;
	txBuf[4u]  = stat;
	txBuf[5u]  = dlc;
	for(i=0;i<8;i++){ txBuf[i+6u] = msg->buf[i]; }
	txBuf[14u] = 0xA5u;
	txBuf[15u] = 0x5Au;
	serial2_write(txBuf,16u);
#endif


	return retVal;
}

/**
 *  READ the TEC, REC, and EFLG buffers.
 */
uint8_t CAN2_getErrors(struct CAN_err_t *err)
{
	uint8_t retVal = true;
#ifdef DEBUG_CAN2_SPI
	uint8_t txBuf[5u];
#endif

	// Read the Error information

	can2.beginTransaction(SPISettings(MCP2515_MAXSPEED,MSBFIRST,SPI_MODE0));

	CAN2_SpiFlushRxFifo();

	digitalWrite(CAN2_CS,LOW);		// pull down CS
	can2.transfer(MCP2515_READ);	// READ command
	can2.transfer(TEC);				// Start address
	err->tec = can2.transfer(DUMMY);
	err->rec = can2.transfer(DUMMY);
	can2.transfer(MCP2515_READ);
	can2.transfer(EFLG);
	err->eflg = can2.transfer(DUMMY);
	digitalWrite(CAN2_CS,HIGH);		// Release the CS
	can2.endTransaction();

#ifdef DEBUG_CAN2_SPI
	// id id id id stat dlc buf[0] buf[1] ... buf[7] 0xA5 0x5A
	txBuf[0]   = err->tec;
	txBuf[1u]  = err->rec;
	txBuf[2u]  = err->eflg;
	txBuf[3u] = 0xA5u;
	txBuf[4u] = 0x5Au;
	serial2_write(txBuf,5u);
#endif

	return retVal;
}

/**
 *  Return the result of the ICODx bits
 *  ICD2:0
 *     000	No Interrupt
 *     001	Error Interrupt
 *     010	Wake-up Interrupt
 *     011	TXB0 Interrupt
 *     100	TXB1 Interrupt
 *     101	TXB2 Interrupt
 *     110	RXB0 Interrupt
 *     111	RXB1 Interrupt
 */
uint8_t CAN2_getIntStatus(void)
{
	uint8_t retVal = 0;
#ifdef DEBUG_CAN2_SPI
	uint8_t txBuf[3u];
#endif

	// Read the CANSTAT Interrupt Information

	can2.beginTransaction(SPISettings(MCP2515_MAXSPEED,MSBFIRST,SPI_MODE0));

	CAN2_SpiFlushRxFifo();

	digitalWrite(CAN2_CS,LOW);		// pull down CS
	can2.transfer(MCP2515_READ);	// READ command
	can2.transfer(CANSTAT);				// Start address
	retVal = (can2.transfer(DUMMY) >> 1u) & 0x07u;
	digitalWrite(CAN2_CS,HIGH);		// Release the CS
	can2.endTransaction();

#ifdef DEBUG_CAN2_SPI
	txBuf[0]   = retVal;
	txBuf[1u] = 0xA5u;
	txBuf[2u] = 0x5Au;
	serial2_write(txBuf,3u);
#endif

	return retVal;
}

void CAN2_SpiFlushRxFifo(void)
{
	uint32_t u32_SPI0_MCR_copy;
	/* Flush the RX FIFO */
	u32_SPI0_MCR_copy = SPI0_MCR;	// Grab a copy of the current register value
	SPI0_MCR |= SPI_MCR_CLR_RXF;
}

/**
 * \brief Pack the provided CAN TX message data into CAN_msg_t format.
 * @param msg The packed CAN message stored via the provided CAN_msg_t pointer
 * @param id 11 or 29-bit CAN Id
 * @param dlc # of data bytes in the packet
 * @param data
 * @return F if one of the values is out of range, T otherwise
 */
uint8_t CAN2_PackTxCanMsgType(CAN_msg_t *pMsg, uint32_t u32Id, uint8_t u8Dlc, uint8_t *pu8Data )
{
	uint8_t bOk =  true;
	uint8_t i = 0;

	// Range Verification
	if(u8Dlc > 8u){ return false; }
	if(u32Id > 0x1FFFFFFFul){ return false; }

	/* pack the stat flags */
	if(u32Id & 0xFFFFF800u){	// Extended Frame
		pMsg->stat = CAN_ID_29BIT | currentBaudRate | CAN_CH_CAN2 | CAN_TX ;
	}
	else{	// Standard Frame
		pMsg->stat = CAN_ID_11BIT | currentBaudRate | CAN_CH_CAN2 | CAN_TX ;
	}

	for(i=0;i<u8Dlc;i++){ pMsg->buf[i] = pu8Data[i]; }
	pMsg->len = u8Dlc;
	pMsg->id = u32Id;

	return bOk;
}




