/*
 * os.c
 *
 *  Created on: Apr 28, 2015
 *      Author: After Hours Engineering, LLC
 */

#include "os.h"
#include "usb_seremu.h"
#include "usb_rawhid.h"
#include "alarms.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
//#include "sdfi.h"
#include "SdLog.h"

#include "HardwareSerial.h"
#include "MCP2515.h"
#include "can1.h"
#include "can2.h"
#include "MCP23018.h"
#include "SIM808.h"
#include "ccom.h"

void OS_Run(void);
void Serial2_Init(void);
void BtDebug(const char * msg, boolean bCrlf);
void OS_GetSpiRegisters(uint32_t * u32RegSet);

byte buffer[64];
byte inBuffer[64];

#define CAN_ENABLED 1u
//#define SD_ENABLED 1u

#define DELAY_BTWN_TX 2000u
#define T_1MS 	1u
#define T_10MS 	10u
#define T_500MS 500u

unsigned int packetCount = 0;

static boolean cardInitialized;

static uint32_t au32SpiRegSet[4];

//boolean SD_Init(void);

void OS_Init(void)
{

	//Serial2_Init();
	CCOM_Init();


	MCP23018_Init();

	SIM808_Init();

	alarms_Init();

#ifdef CAN_ENABLED
	CAN1_Init();
	CAN2_Init();
#endif

#ifdef SD_ENABLED
	delay(5000u);

	SdLog_setup();

	BtDebug("Following SdLog_setup()",true);
	OS_GetSpiRegisters(au32SpiRegSet);	// Debug what's going on with the SPI registers
#endif
	delay(5000u);

	/**
	 *  Call the OS Loop -- Runs forever...
	 */
	OS_Run();
}

/******************************************************************************
 ** Function: <Function Name>
 **
 ** Description: <Function Description>
 ** Inputs:  NA
 ** Outputs: NA
 **
 *****************************************************************************/
void Serial2_Init(void)
{
	serial2_begin(BAUD2DIV(921600u));
	serial2_format(SERIAL_8N1);
}

void Serial3_Init(void)
{
	serial3_begin(BAUD2DIV(115200u));
	serial3_format(SERIAL_8N1);
}



void OS_Run(void)
{
	uint32_t i;
	uint8_t errResult = 0;
	uint8_t data1[8u] = {0,0,0,0,0,0,0,0};
	uint8_t data2[8u] = {0,0,0,0,0,0,0,0};
	boolean bCan1Ok = true;
	boolean bCan2Ok = true;
	static CAN_msg_t msgCan1Tx,msgCan2Tx,msgCan1Rx,msgCan2Rx;

	/** TEST CCOM */
    CAN_msg_t msg1,msg2;
	uint8_t  u8IoDir = 0xFFu;
	uint8_t  u8IoVal = 0xAAu;
	uint64_t u64CanEpochTime1 = 0x1122334455667788ul;
	uint64_t u64CanEpochTime2 = 0x8877665544332211ul;
	uint8_t bOk = 1;
	/** END CCOM TEST VARIABLES */

	/** Init CCOM test variables */
    msg1.id = (uint32_t)0x751;
    msg1.len = 8u;
    msg1.stat = 0xFFu;
    for(i=0;i<8;i++){ msg1.buf[i] = i; }

    msg2.id = (uint32_t)0x3A7;
    msg2.len = 8u;
    msg2.stat = 0xFFu;
    for(i=0;i<8;i++){ msg2.buf[i] = i-8u; }
    /** END CCOM variable init */


	SetAlarm(1u,millis() + T_10MS);
	SetAlarm(2u,millis() + T_1MS);
	SetAlarm(3u,millis() + T_500MS);
	ClearAlarm(2u);
	ClearAlarm(1u);
	ClearAlarm(3u);


	/**
	 *  MCP2515 CAN Setup
	 */
#ifdef CAN_ENABLED

	// RESET the chips
	CAN1_MCP2515_Reset();
	CAN2_MCP2515_Reset();
	delay(10);

	// Set the chips in CONFIG mode
	if(bCan1Ok){ if(!CAN1_setMode(MODE_CONFIG)){ bCan1Ok = false; }}
	if(bCan2Ok){ if(!CAN2_setMode(MODE_CONFIG)){ bCan2Ok = false; }}

	// Set baud rate
	if(bCan1Ok){ if(!CAN1_SetBaud(BAUD_500kbps)){ bCan1Ok = false; }}
	if(bCan2Ok){ if(!CAN2_SetBaud(BAUD_500kbps)){ bCan2Ok = false; }}

	// On SN65HVD230 transceivers (CAN1 & CAN2) enable it...
	if(bCan1Ok){ if(!CAN1_RX1BF_Set(RXnBF_LO)){ bCan1Ok = false; }}
	if(bCan2Ok){ if(!CAN2_RX1BF_Set(RXnBF_LO)){ bCan2Ok = false; }}

	// Clear the interrupt flags
	if(bCan1Ok)
	{
		if(!CAN1_bitModify(CANINTF,										// Register
			               ~TX2IF & ~TX1IF & ~TX0IF & ~RX1IF & ~RX0IF,	// Value to change to
				            TX2IF |  TX1IF |  TX0IF |  RX1IF |  RX0IF))  // Bit Mask
		{
			bCan1Ok = false;
		}
	}
	if(bCan2Ok)
	{
		if(!CAN2_bitModify(CANINTF,						// Register
	                       ~TX2IF & ~TX1IF & ~TX0IF & ~RX1IF & ~RX0IF,	// Value to change to
				            TX2IF |  TX1IF |  TX0IF |  RX1IF |  RX0IF))  // Bit Mask
		{
			bCan2Ok = false;
		}
	}

	// Enable TX0IE, TX1IE, TX2IE, RX0, RX1 Interrupts (INT pin will go low when interrupted)
	if(bCan1Ok)
	{
		if(!CAN1_bitModify(CANINTE,							        // Register
			               TX2IE | TX1IE | TX0IE | RX1IE | RX0IE, 	// Value to change to
				           TX2IE | TX1IE | TX0IE | RX1IE | RX0IE))	// Bit mask
		{
			bCan1Ok = false;
		}
	}

	if(bCan2Ok)
	{
		if(!CAN2_bitModify(CANINTE,									// Register
						   TX2IE | TX1IE | TX0IE | RX1IE | RX0IE, 	// Value to change to
				           TX2IE | TX1IE | TX0IE | RX1IE | RX0IE))	// Bit mask
		{
			bCan2Ok = false;
		}
	}

	// Configure as NORMAL mode before continuing
	if(bCan1Ok){ if(!CAN1_setMode(MODE_NORMAL)){ bCan1Ok = false; }}
	if(bCan2Ok){ if(!CAN2_setMode(MODE_NORMAL)){ bCan2Ok = false; }}

	if(!bCan1Ok){ BtDebug("FAILED: CAN1 Setup",true); }
	if(!bCan2Ok){ BtDebug("FAILED: CAN2 Setup",true); }

#endif
	/**
	 *  END MCP2515 Setup
	 */


	/*
	 * Create 2 CAN transmit messages in CAN_msg_t format.
	 */
	CAN1_PackTxCanMsgType(&msgCan1Tx, 0x00000123ul,8u,data1);
	CAN2_PackTxCanMsgType(&msgCan2Tx, 0x00000456ul,8u,data2);


	/**
	 *  MAIN SYSTEM LOOP
	 */
	while(1)
	{
		/* Update the alarms periodically */
		ProcessAlarms();
#ifdef SD_ENABLED
		//SdLog_Task();
#endif

		/* Process any messages waiting to be sent via USB or BT */
		CCOM_Task();

		/**
		 *  Process CAN RX as fast as possible. Some HSCAN messages
		 *  can arrive as fast as every 170us-200us.
		 */
		CAN1_Task();
		CAN2_Task();

		/* Check if there is received CAN data available and log it */
		if(CAN1_DataIsReady(&msgCan1Rx))
		{
			bOk = CCOM_TxCmd00(&msgCan1Rx,u64CanEpochTime1,0,0,0,0);
			u64CanEpochTime1++;


//			if(!SdLog_SaveCanData(&msgCan1Rx))
//			{
//				BtDebug("ERROR: Could not log msgCan1Rx... Trying again",true);
//				if(!SdLog_SaveCanData(&msgCan1Rx)){ BtDebug("ERROR: Retry failed.",true); }
//			}
		}




		/**************************
		 *  5 0 0 M S   T A S K S
		 *************************/
		if(AlarmIsTriggered(3u))
		{
			ClearAlarm(3u);
			SetAlarm(3u, millis() + T_500MS);

			/* Flash the LEDs */
			if(MCP23018_get_state_LED(1u)){ MCP23018_LED(LED1, 0); MCP23018_LED(LED2, 1u); }
			else if(MCP23018_get_state_LED(2u)){ MCP23018_LED(LED2, 0); MCP23018_LED(LED3, 1u); }
			else if(MCP23018_get_state_LED(3u)){ MCP23018_LED(LED3, 0); MCP23018_LED(LED4, 1u); }
			else if(MCP23018_get_state_LED(4u)){ MCP23018_LED(LED4, 0); MCP23018_LED(LED1, 1u); }
			else{ MCP23018_LED(LED1, 1u); }

			/** CCOM TEST */
//		    do
//		    {
//		    	bOk = CCOM_TxCmd00(&msg1,u64CanEpochTime1,&msg2,u64CanEpochTime2,u8IoDir,u8IoVal);
//		    	u64CanEpochTime1++;
//		    	u64CanEpochTime2++;
//		    	msg1.buf[0]++;
//		    	msg2.buf[0]++;
//		    }while(bOk);
			/** END CCOM TEST */
		}

		/**************************
		 *  1 M S   T A S K S
		 *************************/
		if(AlarmIsTriggered(2))
		{
			ClearAlarm(2);
			SetAlarm(2,millis() + T_1MS);

#ifdef CAN_ENABLED
//			CAN1_Task();
//			CAN2_Task();
//
//			/* Check if there is received CAN data available and log it */
//			if(CAN1_DataIsReady(&msgCan1Rx))
//			{
//				if(!SdLog_SaveCanData(&msgCan1Rx))
//				{
//					BtDebug("ERROR: Could not log msgCan1Rx... Trying again",true);
//					if(!SdLog_SaveCanData(&msgCan1Rx)){ BtDebug("ERROR: Retry failed.",true); }
//				}
//			}
//			if(CAN2_DataIsReady(&msgCan2Rx))
//			{
//				if(!SdLog_SaveCanData(&msgCan2Rx))
//				{
//					BtDebug("ERROR: Could not log msgCan2Rx... Trying again",true);
//					if(!SdLog_SaveCanData(&msgCan2Rx)){ BtDebug("ERROR: Retry failed.",true); }
//				}
//			}

#endif
			SIM808_Task();

		}

		/**************************
		 *  1 0 M S   T A S K S
		 *************************/
		if(AlarmIsTriggered(1))
		{
			ClearAlarm(1);					// Clear the alarm
			SetAlarm(1,millis()+T_10MS);	// Reset to trigger 500ms from now


#ifdef CAN_ENABLED
//			/* Transmit some CAN messages */
//			errResult = CAN1_LoadTxBuffer(TX_BUF_0,			/* Send the data using TX buffer 0 */
//					                      TX_PRIORITY_3,	/* Send the data using the highest priority */
//										  DATA_FRAME,       /* Send message as data frame */
//										  msgCan1Tx.id,     /* Arbitration ID */
//										  msgCan1Tx.len,    /* # of bytes to send */
//										  &msgCan1Tx.buf);  /* Pass the address of the data */
//
//			if(errResult){ BtDebug("CAN1 LOAD errResult: ",false); BtDebug(itoaX((uint32_t)errResult),true); }
//
			errResult = CAN2_LoadTxBuffer(TX_BUF_0,			/* Send the data using TX buffer 0 */
					                      TX_PRIORITY_3,	/* Send the data using the highest priority */
										  DATA_FRAME,       /* Send message as data frame */
										  msgCan2Tx.id,     /* Arbitration ID */
										  msgCan2Tx.len,    /* # of bytes to send */
										  &msgCan2Tx.buf);  /* Pass the address of the data */

			if(errResult){ BtDebug("CAN2 LOAD errResult: ",false); BtDebug(itoaX((uint32_t)errResult),true); }
//
//			/* Log the transmit messages */
//			SdLog_SaveCanData(&msgCan1Tx);	/* Pass the address of the CAN message */
//			SdLog_SaveCanData(&msgCan2Tx);
//
//			/* Change the TX data a bit for next time */
//			msgCan1Tx.buf[7]++; msgCan1Tx.buf[0]--;
			msgCan2Tx.buf[7]--; msgCan2Tx.buf[0]++;
#endif



		}



	} /* END while(1) */

}


/**
 *  Get the current value of the set of SPI register
 */
void OS_GetSpiRegisters(uint32_t * u32RegSet)
{
	u32RegSet[0] = SPI0_MCR;
	u32RegSet[1] = SIM_SCGC6;
	u32RegSet[2] = SPI0_CTAR0;
	u32RegSet[3] = SPI0_CTAR1;

	BtDebug("SPIO_MCR: ",false); BtDebug(itoaX(u32RegSet[0]),true);
	BtDebug("SIM_SCGC6: ",false); BtDebug(itoaX(u32RegSet[1]),true);
	BtDebug("SPI0_CTAR0: ",false); BtDebug(itoaX(u32RegSet[2]),true);
	BtDebug("SPI0_CTAR1: ",false); BtDebug(itoaX(u32RegSet[3]),true);
}

void BtDebug(const char * msg, boolean bCrlf)
{

	const char * CRLF = "\x0d\x0a";
	serial2_write(msg,(uint16_t)strlen(msg));
	if(bCrlf) serial2_write(CRLF,(uint16_t)strlen(CRLF));
}





