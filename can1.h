/*
 * can1.h
 *
 *  Created on: Jul 16, 2015
 *      Author: David
 */

#ifndef CAN1_H_
#define CAN1_H_

#include <stdint.h>
#include <MCP2515.h>

#ifdef __cplusplus
extern "C"
{
#endif
/**
 *  Place all C-accessible function prototypes here.
 */

void CAN1_Init(void);
void CAN1_MCP2515_Reset(void);
void CAN1_Task(void);
uint8_t CAN1_SetBaud(CANbaud_t baud);
uint8_t CAN1_bitModify( uint8_t reg, uint8_t value, uint8_t mask  );
uint8_t CAN1_readStatus(void);
uint8_t CAN1_setMode(CANmode_t mode);
uint8_t CAN1_RX1BF_Set(CANpinState_t state);
uint8_t CAN1_RX0BF_Set(CANpinState_t state);
uint8_t CAN1_LoadTxBuffer(uint8_t buf, uint8_t priority, uint8_t rtr, uint32_t id, uint8_t dlc, uint8_t *data );
uint8_t CAN1_getRxBuf(uint8_t buf, struct CAN_msg_t *msg);
uint8_t CAN1_getErrors(struct CAN_err_t *err);
uint8_t CAN1_getIntStatus(void);
uint8_t CAN1_get_INT_Pin(void);
uint8_t CAN1_DataIsReady( struct CAN_msg_t *msg);

/**
 * \brief Pack the provided CAN TX message data into CAN_msg_t format.
 * @param msg The packed CAN message stored via the provided CAN_msg_t pointer
 * @param id 11 or 29-bit CAN Id
 * @param dlc # of data bytes in the packet
 * @param data
 * @return F if one of the values is out of range, T otherwise
 */
uint8_t CAN1_PackTxCanMsgType(CAN_msg_t *pMsg, uint32_t u32Id, uint8_t u8Dlc, uint8_t *pu8Data );

#ifdef __cplusplus
}
#endif




#endif /* CAN1_H_ */
