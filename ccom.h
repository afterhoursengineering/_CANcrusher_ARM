/******************************************************************************
 **   ccom.h
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

#ifndef CCOM_H_
#define CCOM_H_


/*---------------------------------------------------------------------------*/
/* System Includes                                                           */
/*---------------------------------------------------------------------------*/
#include <stdint.h>
#include "MCP2515.h"

/*---------------------------------------------------------------------------*/
/* Constant declarations                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Type declarations                                                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Structure declarations                                                     */
/*---------------------------------------------------------------------------*/

/**
 *  IMPORTANT: The structure is packed specifically to afford 8-byte alignment to
 *             make its physical size only 64 bytes.  Do not change the order.
 */
typedef union {
	struct {
		uint16_t u16Header;   /* 0xA55A */
		uint8_t  u8Id;		  /* 0x00 */
		uint8_t  u8Dat;		  /* 00: IO only,
							   * 01: CAN message 1 and IO
							   * 02: Both CAN Messages and IO
							   */
		uint8_t  au8Unused1[4u]; /* unused placeholder for 8-byte alignment */


		uint64_t u64CanEpochTime1; /* EPOCH time --> milliseconds
								* since 1/1/1970 00:00:00:000
								*/
		uint8_t  au8CanData1[8u];

		uint32_t u32CanId1;
		uint8_t  u8CanDlc1;
		uint8_t  u8CanStat1;   /* See CAN_msg_t for details */

		uint8_t  au8Unused2[2u];	/* unused placeholder for 8-byte alignment */

		uint64_t u64CanEpochTime2;
		uint8_t  au8CanData2[8u];

		uint32_t u32CanId2;
		uint8_t  u8CanDlc2;
		uint8_t  u8CanStat2;
		uint8_t  au8Unused3[2u];	/* placeholder */

		uint8_t  u8IoDir;	    /* VEH IO pin direction (1:input, 0:output) */
		uint8_t  u8IoVal;		/* VEH IO pin value */

		uint8_t  au8Unused4[4u];	/* Unused bytes */
		uint8_t  u8BufSize;		/*
								 * Current # of buffers used to
								 * hold the 64-byte packets.
								 */
		uint8_t  u8Cksm;			/* 8-bit Cksm of u16Header to u8Unused[7] */
	};

	uint8_t au8AllData[64u];

} T_CCOM_ID00;

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
/**
 *  Place all C-accessible function prototypes here.
 */

void CCOM_Init(void);

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
					 uint8_t u8IoVal);

void CCOM_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* CCOM_H_ */
