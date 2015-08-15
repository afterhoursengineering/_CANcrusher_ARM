/*
 * MCP2515.h
 *
 *  Created on: Jul 18, 2015
 *      Author: David Burke
 */

#ifndef MCP2515_H_
#define MCP2515_H_

//#define MCP2515_MAXSPEED	10000000u	// MCP2515 MAX SCK = 10MHz
#define MCP2515_MAXSPEED	250000u
// BAUDRATE OPTIONS
/**
 * xxx0 00xx 00 BAUD_33p333kbps
 * xxx0 01xx 04 BAUD_50kbps
 * xxx0 10xx 08 BAUD_83p333kbps
 * xxx0 11xx 0C BAUD_100kbps
 * xxx1 00xx 10 BAUD_125kbps
 * xxx1 01xx 14 BAUD_250kbps
 * xxx1 10xx 18 BAUD_500kbps
 * xxx1 11xx 1C (unused)
 */
enum enum_CANbaud {
	BAUD_33p333kbps = 0x00u,
	BAUD_50kbps     = 0x04u,
	BAUD_83p333kbps = 0x08u,
	BAUD_100kbps    = 0x0Cu,
	BAUD_125kbps    = 0x10u,
	BAUD_250kbps    = 0x14u,
	BAUD_500kbps    = 0x18u
};
typedef enum enum_CANbaud CANbaud_t;

// MODE OPTIONS
enum enum_CANmodes {
	MODE_CONFIG,
	MODE_NORMAL,
	MODE_SLEEP,
	MODE_LISTEN_ONLY,
	MODE_LOOPBACK
};
typedef enum enum_CANmodes CANmode_t;

#define CAN_CH_LIN 		0x00u
#define CAN_CH_CAN1   	0x20u
#define CAN_CH_CAN2   	0x40u
#define CAN_CH_CANSW  	0x60u
#define CAN_ID_11BIT 	0x00u
#define CAN_ID_29BIT 	0x01u
#define CAN_TX       	0x80u
#define CAN_RX       	0x00u

/* Transmit request priority. 3 = HIGHEST */
#define TX_PRIORITY_0	0
#define TX_PRIORITY_1	1
#define TX_PRIORITY_2	2
#define TX_PRIORITY_3	3

/* Transmit buffer Selection */
#define TX_BUF_0		0
#define TX_BUF_1		1
#define TX_BUF_2		2

/* RTR bit configurations */
#define DATA_FRAME			0	/* Send the message as a data frame */
#define REMOTE_TRANS_RQST	1   /* Send the message as a remote transmit request */

#define CAN_DATA_BUF_SIZE	8u

/**
 * element size description
 * ------------------------
 *  id     32   CAN arbitration ID (11-bit or 29-bit)
 *  len    8	DLC/# of data bytes sent or received
 *  buf    8x8  8 bytes of data
 *  stat   8    encoded data (extended, CAN channel, TX/RX, baud)
 *
 *  stat_bit  encoding
 *  7         TX/RX
 *  6:5       CHANNEL (00-KLINE/LIN, 01-CAN1, 10-CAN2, 11-SW-CAN
 *  4:2       baud (000 - 33.333kbps
 *                  001 - 50kbps
 *                  010 - 83.333kbps
 *                  011 - 100kbps
 *                  100 - 125kbps
 *                  101 - 250kbps
 *                  110 - 500kbps)
 *  1          unused
 *  0          0:11-bit, 1:29-bit (extended)
 */
typedef struct CAN_msg_t {
  uint32_t id; // can identifier
  uint8_t stat; // 8-bits of message status information
  uint8_t len; // length of data
  uint8_t buf[CAN_DATA_BUF_SIZE];
} CAN_msg_t;

// CAN_msg_t stat Bit Masks
#define STAT_TXRX	0x80u	// 1000 0000
#define STAT_CHAN	0x60u	// 0110 0000
#define STAT_BAUD	0x1Cu	// 0001 1100
#define STAT_EXT	0x01u	// 0000 0001

#define ID_11BIT_MASK  0x000007FFul
#define ID_29BIT_MASK  0x1FFFFFFFul

/**
 *  tec:  Transmit Error Counter
 *  rec:  Receiver Error Counter
 *  eflg: Error Flag
 *        (7) (R/W) RX1OVR Receive Buffer 1 Overflow Flag bit
 *        (6) (R/W) RX0OVR Receive Buffer 0 Overflow Flag bit
 *        (5) (R) TXBO Bus-Off Error Flag bit
 *        (4) (R) TXEP Transmit Error-Passive Flag bit
 *        (3) (R) RXEP Receive Error-Passive Flag bit
 *        (2) (R) TXWAR Transmit Error Warning Flag bit
 *        (1) (R) RXWAR Receive Error Warning Flag bit
 *        (0) (R) EWARN Error Warning Flag bit
 */
typedef struct CAN_err_t {
	uint8_t tec;	// Transmit Error Counter
	uint8_t rec;	// Receiver Error Counter
	uint8_t eflg;	// Error Flag
} CAN_err_t;

// RXnBF PIN STATE
enum enum_CANpinState{
	RXnBF_HI,
	RXnBF_LO,
	RXnBF_OFF
};
typedef enum enum_CANpinState CANpinState_t;

// CANSTAT - INTERRUPT STATUS
#define CAN_INT_NONE	0x00u
#define CAN_INT_ERR		0x01u
#define CAN_INT_WAK		0x02u
#define CAN_INT_TXB0	0x03u
#define CAN_INT_TXB1	0x04u
#define CAN_INT_TXB2	0x05u
#define CAN_INT_RXB0	0x06u
#define CAN_INT_RXB1	0x07u

// SPI Commands
#define DUMMY			0xFFu			// Dummy byte used during SPI reads
#define MCP2515_READ	0x03u			// MCP2515 READ command
#define MCP2515_STATUS 	0xA0u			// MCP2515 STATUS command
#define MCP2515_RESET	0xC0u			// MCP2515 RESET command
#define MCP2515_WRITE	0x02u			// WRITE command
#define MCP2515_RX_STATUS	0xB0u		// RX_READ Status
#define MCP2515_BIT_MODIFY	0x05u		// BIT_MODIFY
#define MCP2515_LOAD_TX_BUF_0 0x40u		// Start loading data in TXB0SIDH (0x31)
#define MCP2515_LOAD_TX_BUF_1 0x42u		// Start loading data in TXB1SIDH (0x41)
#define MCP2515_LOAD_TX_BUF_2 0x44u		// Start loading data in TXB2SIDH (0x51)
#define MCP2515_READ_RX_BUF_0 0x90u		// Read RXBUF0 starting at RXB0SIDH (0x61)
#define MCP2515_READ_RX_BUF_1 0x94u      // Read RXBUF1 starting at RXB1SIDH (0x71)

// Bit Masks
#define BIT7		0x80u
#define BIT6		0x40u
#define BIT5		0x20u
#define BIT4		0x10u
#define BIT3		0x08u
#define BIT2		0x04u
#define BIT1		0x02u
#define BIT0		0x01u

// BFPCTRL
#define B1BFS		BIT5
#define B0BFS		BIT4
#define B1BFE		BIT3
#define B0BFE		BIT2
#define B1BFM		BIT1
#define B0BFM		BIT0

// TXRTSCTRL
#define B2RTS		BIT5
#define B1RTS		BIT4
#define B0RTS		BIT3
#define B2RTSM		BIT2
#define B1RTSM		BIT1
#define B0RTSM		BIT0

// CANSTAT
#define OPMOD2		BIT7
#define OPMOD1		BIT6
#define OPMOD0		BIT5
#define ICOD2		BIT3
#define ICOD1		BIT2
#define ICOD0		BIT1

// CANCTRL
#define REQOP2		BIT7
#define REQOP1		BIT6
#define REQOP0		BIT5
#define ABAT		BIT4
#define OSM			BIT3
#define CLKEN		BIT2
#define CLKPRE1		BIT1
#define CLKPRE0		BIT0

// CNF3
#define SOF			BIT7
#define WAKFIL		BIT6
#define PHSEG22		BIT2
#define PHSEG21		BIT1
#define PHSEG20		BIT0

// CNF2
#define BTLMODE		BIT7
#define SAM			BIT6
#define PHSEG12		BIT5
#define PHSEG11		BIT4
#define PHSEG10		BIT3
#define PRSEG2		BIT2
#define PRSEG1		BIT1
#define PRSEG0		BIT0

// CNF1
#define SJW1		BIT7
#define SJW0		BIT6
#define BRP5		BIT5
#define BRP4		BIT4
#define BRP3		BIT3
#define BRP2		BIT2
#define BRP1		BIT1
#define BRP0		BIT0

// TXBnCTRL
#define ABTF		BIT6
#define MLOA		BIT5
#define TXERR		BIT4
#define TXREQ		BIT3
#define TXP1		BIT1
#define TXP0		BIT0

// RXB0CTRL & RXB1CTRL
#define RXM1		BIT6
#define RXM0		BIT5
#define RXRTR		BIT3
#define BUKT		BIT2
#define FILHIT2		BIT2
#define BUKT1		BIT1
#define FILHIT1		BIT1
#define FILHIT0		BIT0

// EFLG Error Flag Register
#define RX1OVR		BIT7
#define RX0OVR		BIT6
#define	TXBO		BIT5
#define TXEP		BIT4
#define RXEP		BIT3
#define TXWAR		BIT2
#define RXWAR		BIT1
#define EWARN		BIT0

// CANINTE Interrupt Enable (0x2B)
#define MERRE		BIT7
#define WAKIE		BIT6
#define ERRIE		BIT5
#define TX2IE		BIT4
#define TX1IE		BIT3
#define TX0IE		BIT2
#define RX1IE		BIT1
#define RX0IE		BIT0

// CANINTF Interrupt Flag (0x2C)
#define MERRF		BIT7
#define WAKIF		BIT6
#define ERRIF		BIT5
#define TX2IF		BIT4
#define TX1IF		BIT3
#define TX0IF		BIT2
#define RX1IF		BIT1
#define RX0IF		BIT0

//Registers
#define RXF0SIDH	0x00u
#define RXF0SIDL	0x01u
#define RXF0EID8	0x02u
#define RXF0EID0	0x03u
#define RXF1SIDH	0x04u
#define RXF1SIDL	0x05u
#define RXF1EID8	0x06u
#define RXF1EID0	0x07u
#define RXF2SIDH	0x08u
#define RXF2SIDL	0x09u
#define RXF2EID8	0x0Au
#define RXF2EID0	0x0Bu
#define BFPCTRL 	0x0Cu
#define TXRTSCTRL	0x0Du
#define CANSTAT 	0x0Eu // Status register
#define CANCTRL 	0x0Fu //Mode control register
#define RXF3SIDH	0x10u
#define RXF3SIDL	0x11u
#define RXF3EID8	0x12u
#define RXF3EID0	0x13u
#define RXF4SIDH	0x14u
#define RXF4SIDL	0x15u
#define RXF4EID8	0x16u
#define RXF4EID0	0x17u
#define RXF5SIDH	0x18u
#define RXF5SIDL	0x19u
#define RXF5EID8	0x1Au
#define RXF5EID0	0x1Bu
#define TEC         0x1Cu
#define REC         0x1Du

#define RXM0SIDH	0x20u
#define RXM0SIDL	0x21u
#define RXM0EID8	0x22u
#define RXM0EID0	0x23u
#define RXM1SIDH	0x24u
#define RXM1SIDL	0x25u
#define RXM1EID8	0x26u
#define RXM1EID0	0x27u
#define CNF3 		0x28u
#define CNF2 		0x29u
#define CNF1 		0x2Au
#define CANINTE 	0x2Bu // Interrupt Enable
#define CANINTF 	0x2Cu // Interrupt Flag
#define EFLG 		0x2Du // Error Register address

#define TXB0CTRL 	0x30u
#define TXB0SIDH	0x31u
#define TXB0SIDL	0x32u
#define TXB0EID8	0x33u
#define TXB0EID0	0x34u
#define TXB0DLC		0x35u
#define TXB0D0		0x36u
#define TXB0D1		0x37u
#define TXB0D2		0x38u
#define TXB0D3		0x39u
#define TXB0D4		0x3Au
#define TXB0D5		0x3Bu
#define TXB0D6		0x3Cu
#define TXB0D7		0x3Du

#define TXB1CTRL 	0x40u
#define TXB1SIDH	0x41u
#define TXB1SIDL	0x42u
#define TXB1EID8	0x43u
#define TXB1EID0	0x44u
#define TXB1DLC		0x45u
#define TXB1D0		0x46u
#define TXB1D1		0x47u
#define TXB1D2		0x48u
#define TXB1D3		0x49u
#define TXB1D4		0x4Au
#define TXB1D5		0x4Bu
#define TXB1D6		0x4Cu
#define TXB1D7		0x4Du

#define TXB2CTRL 	0x50u //TRANSMIT BUFFER CONTROL REGISTER
#define TXB2SIDH	0x51u
#define TXB2SIDL	0x52u
#define TXB2EID8	0x53u
#define TXB2EID0	0x54u
#define TXB2DLC		0x55u
#define TXB2D0		0x56u
#define TXB2D1		0x57u
#define TXB2D2		0x58u
#define TXB2D3		0x59u
#define TXB2D4		0x5Au
#define TXB2D5		0x5Bu
#define TXB2D6		0x5Cu
#define TXB2D7		0x5Du

#define RXB0CTRL 	0x60u
#define RXB0SIDH	0x61u
#define RXB0SIDL	0x62u
#define RXB0EID8 	0x63u
#define RXB0EID0	0x64u
#define RXB0DLC		0x65u
#define RXB0D0		0x66u
#define RXB0D1		0x67u
#define RXB0D2		0x68u
#define RXB0D3		0x69u
#define RXB0D4		0x6Au
#define RXB0D5		0x6Bu
#define RXB0D6		0x6Cu
#define RXB0D7		0x6Du

#define RXB1CTRL 	0x70u
#define RXB1SIDH	0x71u
#define RXB1SIDL	0x72u
#define RXB1EID8 	0x73u
#define RXB1EID0	0x74u
#define RXB1DLC		0x75u
#define RXB1D0		0x76u
#define RXB1D1		0x77u
#define RXB1D2		0x78u
#define RXB1D3		0x79u
#define RXB1D4		0x7Au
#define RXB1D5		0x7Bu
#define RXB1D6		0x7Cu
#define RXB1D7		0x7Du


#endif /* MCP2515_H_ */
