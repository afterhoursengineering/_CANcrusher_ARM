/*
 * can1.h
 *
 *  Created on: Jul 16, 2015
 *      Author: David
 */

#ifndef CAN2_H_
#define CAN2_H_

#include <stdint.h>
#include <MCP2515.h>

#ifdef __cplusplus
extern "C"
{
#endif
/**
 *  Place all C-accessible function prototypes here.
 */

void CAN2_Init(void);
void CAN2_MCP2515_Reset(void);
void CAN2_Task(void);
uint8_t CAN2_SetBaud(CANbaud_t baud);
uint8_t CAN2_bitModify( uint8_t reg, uint8_t value, uint8_t mask  );
uint8_t CAN2_readStatus(void);
uint8_t CAN2_setMode(CANmode_t mode);
uint8_t CAN2_RX1BF_Set(CANpinState_t state);
uint8_t CAN2_RX0BF_Set(CANpinState_t state);
uint8_t CAN2_LoadTxBuffer(uint8_t buf, uint8_t priority, uint8_t rtr, uint32_t id, uint8_t dlc, uint8_t *data );
uint8_t CAN2_getRxBuf(uint8_t buf, struct CAN_msg_t *msg);
uint8_t CAN2_getErrors(struct CAN_err_t *err);
uint8_t CAN2_getIntStatus(void);
uint8_t CAN2_get_INT_Pin(void);
uint8_t CAN2_DataIsReady( struct CAN_msg_t *msg);
uint8_t CAN2_PackTxCanMsgType(CAN_msg_t *pMsg, uint32_t u32Id, uint8_t u8Dlc, uint8_t *pu8Data );

#ifdef __cplusplus
}
#endif

#endif /* CAN2_H_ */
