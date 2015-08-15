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

/*---------------------------------------------------------------------------*/
/* Constant declarations                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Type declarations                                                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Structure declarations                                                     */
/*---------------------------------------------------------------------------*/
typedef struct CCOM_ID00_Struct
{
	uint16_t u16Header;   /* 0xA55A */
	uint8_t  u8Id;		  /* 0x00 */
	uint8_t  u8Dat;		  /* 00: IO only,
	                       * 01: CAN message 1 only
	                       * 02: Both CAN Messages only
	                       * 03: Both CAN + IO
	                       */
	uint64_t u64Can1EpochTime; /* EPOCH time --> milliseconds
	                        * since 1/1/1970 00:00:00:000
	                        */
	uint32_t u32Can1Id;
	uint8_t  u8Can1Dlc;
	uint8_t  u8Can1Stat;   /* See CAN_msg_t for details */
	uint8_t  au8Can1Data[8u];

	uint64_t u64Can2EpochTime;
	uint32_t u32Can2Id;
	uint8_t  u8Can2Dlc;
	uint8_t  u8Can2Stat;
	uint8_t  au8Can2Data[8u];

	uint8_t  u8IoDir;	    /* VEH IO pin direction (1:input, 0:output) */
	uint8_t  u8IoVal;		/* VEH IO pin value */
	uint8_t  u8BufSize;		/*
	                         * Current # of buffers used to
	                         * hold the 64-byte packets.
	                         */
	uint8_t  u8Unused[12u];	/* Unused bytes */

	uint8_t  u8Crc;			/* 8-bit CRC of u16Header to u8Unused[7] */
}T_CCOM_ID00;
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

#ifdef __cplusplus
}
#endif

#endif /* CCOM_H_ */
