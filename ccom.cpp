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
#include "usb_seremu.h"
#include "HardwareSerial.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ccom.h"
#include "can1.h"
#include "can2.h"


/*---------------------------------------------------------------------------*/
/* Constant declarations                                                     */
/*---------------------------------------------------------------------------*/
#define BT		Serial2
#define USB		Serial

#define MAX_FRAME_SIZE  64u		/* TX frames are sent in 64 byte packets     */
#define MAX_BUFFER_SIZE 8u      /* 8 buffers of 64 bytes                     */
#define RBUF_SIZE    MAX_BUFFER_SIZE
#define ID00_IO_ONLY	0
#define ID00_CAN_MSG1_AND_IO 1u
#define ID00_CAN_MSG1_AND_2_AND_IO 2u

/** To send A55A as the header, need to store the data in little endian format
 *  since the struct variable for the header is a uint16_t and all data is
 *  stored (and therefore sent) in little endian format as well.
 */
#define ID00_HEADER (uint16_t)0x5AA5u
#define ID00        (uint8_t)0x00

/*---------------------------------------------------------------------------*/
/* Type declarations                                                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Structure declarations                                                     */
/*---------------------------------------------------------------------------*/
/**
 *  Structure defines a typedef for a FIFO ring buffer.  Structure contains the
 *  pointer buffer of size RBUF_SIZE, the index of the (head) and (tail), and the
 *  total number of 32-bit addresses currently being used in the buffer (count).
 */
typedef struct  ringBufS
{
    uint32_t au32Addr[RBUF_SIZE]; /* 32bit address buffer of size RBUF_SIZE           */
    int16_t head;                 /* The index of the newest data in the buffer       */
    int16_t tail;                 /* The index of the oldest data in the buffer       */
    int16_t count;                /* The total # of addresses currently used by the buffer */
} T_RING_BUF;


/*---------------------------------------------------------------------------*/
/* Variable declarations                                                     */
/*---------------------------------------------------------------------------*/
static uint8_t au8Data[MAX_BUFFER_SIZE][MAX_FRAME_SIZE];
static T_RING_BUF ccomBuf;

/*---------------------------------------------------------------------------*/
/* Macro declarations                                                        */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Function prototypes                                                       */
/*---------------------------------------------------------------------------*/
bool USB_isReady(void);

void LoadOutput(uint8_t *inBuf, uint8_t *outBuf, uint16_t len);
void Clear_ID00_Packet(T_CCOM_ID00 * id00Packet);
uint8_t Checksum_ID00_Packet(T_CCOM_ID00 * id00Packet);

uint16_t modulo_inc (const uint16_t value, const uint16_t modulus);
void     rbuf_Init  (T_RING_BUF *_this, uint8_t * initAddress, uint16_t u16ArrayLen);
uint8_t  rbuf_empty (T_RING_BUF *_this);
uint8_t  rbuf_full  (T_RING_BUF *_this);
uint8_t  rbuf_get   (T_RING_BUF *_this, uint32_t * pu32Val);
void     rbuf_put   (T_RING_BUF *_this, uint32_t * pu32Val);
void     rbuf_flush (T_RING_BUF *_this);
void Parse_USB_rawHID(uint8_t * buf);
void Process_RxCmd_80(uint8_t * buf);

/**
 * \brief Load an output buffer with the bytes of the input buffer.
 * @param inBuf  pointer to the input buffer
 * @param outBuf pointer to the output buffer
 * @param len length of the data to transfer, from 0 to (len-1)
 */
void LoadOutput(uint8_t *inBuf, uint8_t *outBuf, uint16_t len)
{
    uint16_t i;
    for(i=0;i<len;i++){ outBuf[i] = inBuf[i]; }
}

/**
 *  \brief Initialize the USB and Serial2 drivers
 */
void CCOM_Init(void)
{
	uint8_t i;

	BT.begin(921600u,SERIAL_8N1);
	USB.begin(9600);	/* Pretty sure this does nothing... USB enumerates if connected on its own */

	rbuf_Init(&ccomBuf, &au8Data[0][0], (uint16_t)MAX_FRAME_SIZE);
}

/**
 * \brief Returns 8-bit checksum of all bytes in the packet up to the CKSM
 *
 * Checksum is the summation of all bytes, inverted, subtract 1
 *
 * @param id00Packet
 * @return
 */
uint8_t Checksum_ID00_Packet(T_CCOM_ID00 * id00Packet)
{
	uint64_t u64Cksm = 0;
	uint8_t i;

	for(i=0;i<(MAX_FRAME_SIZE-1);i++){ u64Cksm += id00Packet->au8AllData[i]; }

	return (~((uint8_t)u64Cksm))-1;
}

void Clear_ID00_Packet(T_CCOM_ID00 * id00Packet)
{
	uint8_t i;

	for(i=0;i<64u;i++){ id00Packet->au8AllData[i] = 0; }
}

/**
 * \brief Periodic process which will send any data currently loaded for transmission
 */
void CCOM_Task(void)
{
	uint32_t u32GetAddr;
	uint8_t *pu8Data;
	uint8_t i;
	uint8_t n;	// # of bytes received via USB
	uint8_t au8Buf[MAX_FRAME_SIZE];

	if(!rbuf_empty(&ccomBuf))
	{
		rbuf_get(&ccomBuf,&u32GetAddr);

		pu8Data = (uint8_t *)u32GetAddr;
		//if(USB_isReady()){ USB.write(pu8Data,MAX_FRAME_SIZE); }
		usb_rawhid_send(pu8Data, MAX_FRAME_SIZE);

		//for(i=0;i<MAX_FRAME_SIZE;i++){ printf("%02x,",*pu8Data++); } printf("\n\r");
	}

	/* Process any received data */
	n = usb_rawhid_recv(au8Buf, 0);
	if(MAX_FRAME_SIZE == n)
	{
		Parse_USB_rawHID(au8Buf);
	}

}

/**
 * \brief Parses any incoming rawHID USB traffic
 * @param buf
 */
void Parse_USB_rawHID(uint8_t * buf)
{
	uint8_t i;

	//Serial.println("Received a packet.");
	//for(i=0; i<MAX_FRAME_SIZE;i++){ Serial.printf("%02x",buf[i]); } Serial.printf("\n\r");


	// Process data if header (A55A) and command
	if((buf[0] == 0xA5u) && (buf[1] == 0x5Au))
	{
		switch(buf[2])
		{
		case 0x80u:
			//Serial.println("Process_RxCmd_80");
			Process_RxCmd_80(buf);
			break;

		default:

			break;
		}
	}

}

void Process_RxCmd_80(uint8_t * buf)
{
	uint8_t i;
	CAN_msg_t msg;
	uint8_t u8Data[8];
	uint32_t u32Id = 0;
	uint8_t u8Dlc;

	for(i=0;i<8;i++){ u8Data[i] = 0; }	// Init CAN data

	// Get the DLC
	u8Dlc = buf[8u];
	if(u8Dlc > 8)
	{
		Serial.println("ERROR: u8Dlc > 8");
		return;
	}	// Range checking
	// Get CAN data
	for(i=0;i<u8Dlc;i++){ u8Data[i] = buf[i+9]; } // Can Data is held in bytes 9-16
	// Get ARB ID (bytes 4:7, LSB = byte 4)
	u32Id =   (uint32_t)buf[4u]          |
			(((uint32_t)buf[5u]) << 8u)  |
			(((uint32_t)buf[6u]) << 16u) |
			(((uint32_t)buf[7u]) << 24u);

	// Select the channel to send the data out from
	switch(buf[3])
	{
	// CAN1
	case 1:
		//Serial.println("Attempting to send CAN1 message");
		CAN1_PackTxCanMsgType(&msg, u32Id,u8Dlc,u8Data); // Pack the message
		CAN1_LoadTxBuffer(TX_BUF_0,			/* Send the data using TX buffer 0 */
				          TX_PRIORITY_3,	/* Send the data using the highest priority */
						  DATA_FRAME,       /* Send message as data frame */
						  msg.id,           /* Arbitration ID */
						  msg.len,    		/* # of bytes to send */
						  &msg.buf[0]);  		/* Pass the address of the data */

		// Tell the UI what message was just sent...
		CCOM_TxCmd00(&msg,0,0,0,0,0);
		break;
	// CAN2
	case 2:
		//Serial.println("Attempting to send CAN2 message");
		CAN2_PackTxCanMsgType(&msg, u32Id,u8Dlc,u8Data); // Pack the message
		CAN2_LoadTxBuffer(TX_BUF_0,			/* Send the data using TX buffer 0 */
				          TX_PRIORITY_3,	/* Send the data using the highest priority */
						  DATA_FRAME,       /* Send message as data frame */
						  msg.id,           /* Arbitration ID */
						  msg.len,    		/* # of bytes to send */
						  &msg.buf[0]);  		/* Pass the address of the data */

		// Tell the UI what message was just sent...
		CCOM_TxCmd00(&msg,0,0,0,0,0);
		break;
	// SWCAN
	case 3:
		// TODO: Add SWCAN functionality
		break;

	default:
		Serial.println("ERROR: buf[3] wasn't 1-3");
		break;
	}

}

/**
 * \brief Passes CAN message data and VEH_IO data to the output stream
 * @param canMsg1 pointer to a packet of CAN message data
 * @param u64Can1EpochTime1 Time the message arrived
 * @param canMsg2 pointer to a packet of CAN message data
 * @param u64CanEpochTime2 Time the message arrived
 * @param u8IoDir 8-bit value representing the direction of the IO pins (Input:1, Output:0)
 * @param u8IoVal 8-bit value representing the value currently on the IO pins.
 * @return T if data is in a valid format and if there is space available in the output stream buffers
 */
uint8_t CCOM_TxCmd00(CAN_msg_t *canMsg1,
		             uint64_t u64CanEpochTime1,
					 CAN_msg_t *canMsg2,
					 uint64_t u64CanEpochTime2,
					 uint8_t u8IoDir,
					 uint8_t u8IoVal)
{
	uint8_t bOk = 1;
	T_CCOM_ID00 id00Packet;
	uint8_t i;
	uint32_t u32PutAddr;

	Clear_ID00_Packet(&id00Packet);

	/* Check to see if there is room in the output stream buffers */
	if(rbuf_full(&ccomBuf)){ return 0; }

	id00Packet.u16Header = ID00_HEADER;
	id00Packet.u8Id = ID00;
	id00Packet.u8IoDir = u8IoDir;
	id00Packet.u8IoVal = u8IoVal;
	id00Packet.u8BufSize = ccomBuf.count;

	if(NULL == canMsg1)
	{
		id00Packet.u8Dat = ID00_IO_ONLY;
	}
	else if(canMsg2 != NULL)
	{
		id00Packet.u8Dat = ID00_CAN_MSG1_AND_2_AND_IO;

		/* CAN MESSAGE 1 */
		id00Packet.u32CanId1 = canMsg1->id;
		id00Packet.u8CanDlc1 = canMsg1->len;
		id00Packet.u8CanStat1 = canMsg1->stat;
		for(i=0;i<canMsg1->len;i++){ id00Packet.au8CanData1[i] = canMsg1->buf[i]; }
		id00Packet.u64CanEpochTime1 = u64CanEpochTime1;

		/* CAN MESSAGE 2 */
		id00Packet.u32CanId2 = canMsg2->id;
		id00Packet.u8CanDlc2 = canMsg2->len;
		id00Packet.u8CanStat2 = canMsg2->stat;
		for(i=0;i<canMsg2->len;i++){ id00Packet.au8CanData2[i] = canMsg2->buf[i]; }
		id00Packet.u64CanEpochTime2 = u64CanEpochTime2;
	}
	else
	{
		id00Packet.u8Dat = ID00_CAN_MSG1_AND_IO;

		/* CAN MESSAGE 1 */
		id00Packet.u32CanId1 = canMsg1->id;
		id00Packet.u8CanDlc1 = canMsg1->len;
		id00Packet.u8CanStat1 = canMsg1->stat;
		for(i=0;i<canMsg1->len;i++){ id00Packet.au8CanData1[i] = canMsg1->buf[i]; }
		id00Packet.u64CanEpochTime1 = u64CanEpochTime1;
	}

	id00Packet.u8Cksm = Checksum_ID00_Packet(&id00Packet);

	/* Packet is put together... Now stuff it in a bucket for sending */
	rbuf_put(&ccomBuf, &u32PutAddr);	// which available array should we use?
	LoadOutput(&id00Packet.au8AllData[0],(uint8_t *)u32PutAddr,MAX_FRAME_SIZE);


	//for(i=0;i<MAX_FRAME_SIZE;i++){ printf("%02x,",au8Data[0][i]); } printf("\n\r");

	return bOk;
}

/**
 * \brief Initialize the Ring Buffer
 *
 * Initialize the ring buffer such that all elements = 0 (buf, head, tail, count),
 * then populate the array with the addresses provided.
 *
 * @param _this
 */
void rbuf_Init(T_RING_BUF *_this, uint8_t * initAddress, uint16_t u16ArrayLen)
{
    uint16_t i;
    //memset (_this, 0, sizeof (*_this));
    _this->count = 0;
    _this->head = 0;
    _this->tail = 0;
    for(i=0; i<RBUF_SIZE;i++){ _this->au32Addr[i] = (uint32_t)initAddress + i * u16ArrayLen; }
}

/**
 * \brief Get the oldest data in the buffer (First In).
 * @param _this
 * @param pu8Val --> au32Addr[tail] returned here.
 * @return T if there is data in the buffer, F otherwise
 */
uint8_t rbuf_get (T_RING_BUF *_this, uint32_t * pu32Val)
{
	uint8_t bOk = 1;

    if (_this->count>0)
    {

    	*pu32Val = _this->au32Addr[_this->tail];
      _this->tail = modulo_inc (_this->tail, RBUF_SIZE);
      --_this->count;
    }
    else{ bOk = 0; }
    return bOk;
}

/**
 * \brief Add data to one of the arrays pointed to by the address
 *        at au32Addr[head].
 *
 *  If .count < RBUF_SIZE, head is incremented, count is incremented
 *  and the next available 32-bit address is returned.
 *
 * @param _this pointer to the ring buffer
 * @param pu32Val 32-bit address at au32Addr[head] is returned
 */
void rbuf_put (T_RING_BUF *_this, uint32_t * pu32Val)
{
    if (_this->count < RBUF_SIZE)
    {
    	*pu32Val = _this->au32Addr[_this->head];
      _this->head = modulo_inc (_this->head, RBUF_SIZE);
      ++_this->count;
    }
}

/**
 * \brief Returns whether Ring Buffer is FULL(T) or NOT_FULL(F)
 *
 * @param _this pointer to the ring buffer
 * @return FULL-->TRUE, NOT_FULL-->FALSE
 */
uint8_t rbuf_full (T_RING_BUF *_this)
{
    return (_this->count>=RBUF_SIZE);
}

/**
 * \brief Flush the buffer
 *
 * count, head, and tail are cleared to 0.
 *
 * @param _this pointer to the ring buffer
 */
void rbuf_flush (T_RING_BUF *_this)
{
  _this->count  = 0;
  _this->head   = 0;
  _this->tail   = 0;
}

/**
 * \brief Returns whether Ring Buffer is EMPTY(T) or NOT_EMPTY(F)
 *
 * @param _this pointer to the ring buffer
 * @return EMPTY(TRUE), NOT_EMPTY(FALSE)
 */
uint8_t rbuf_empty (T_RING_BUF *_this)
{
    return (0==_this->count);
}

/**
 *\brief Calculate the modulus of 2 numbers
 *
 * If value++ >= modulus, return 0, else return value++.
 *
 * @param value The value compared against the provided modulus
 * @param modulus The modulus
 * @return value++ if value++ < modulus, else 0
 */
uint16_t modulo_inc (const uint16_t value, const uint16_t modulus)
{
	uint16_t my_value = value + 1;
    if (my_value >= modulus)
    {
      my_value  = 0;
    }
    return (my_value);
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






