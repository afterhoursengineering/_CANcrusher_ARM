/*
 * mcp23018.h
 *
 *  Created on: Jul 25, 2015
 *      Author: David
 */

#ifndef BUILD_MCP23018_H_
#define BUILD_MCP23018_H_

#include <stdint.h>

// Device Selections
#define DEV_MISC	(uint8_t)0	// U13 -> Controls Bluetooth IO, SIM808, LEDs, LIN control, and VEH_IO comparator
#define DEV_VEH_IO	(uint8_t)1u	// U27 -> Controls all vehicle IO

// Comparator
#define COMP_ON	 (uint8_t)1u
#define COMP_OFF (uint8_t)0u

// LEDs
#define LED1 (uint8_t)1u
#define LED2 (uint8_t)2u
#define LED3 (uint8_t)3u
#define LED4 (uint8_t)4u

// VEH IO PINS
#define IO_PIN1	(uint8_t)1u
#define IO_PIN2	(uint8_t)2u
#define IO_PIN3 (uint8_t)3u
#define IO_PIN4	(uint8_t)4u
#define IO_PIN5	(uint8_t)5u
#define IO_PIN6	(uint8_t)6u
#define IO_PIN7	(uint8_t)7u
#define IO_PIN8	(uint8_t)8u

#define IO_INPUT	(uint8_t)1u
#define IO_OUTPUT	(uint8_t)0u

#define GPA	(uint8_t)0u
#define GPB	(uint8_t)1u

// BIT MASKS
#define GP0	(uint8_t)0x01u
#define GP1	(uint8_t)0x02u
#define GP2	(uint8_t)0x04u
#define GP3	(uint8_t)0x08u
#define GP4	(uint8_t)0x10u
#define GP5	(uint8_t)0x20u
#define GP6	(uint8_t)0x40u
#define GP7	(uint8_t)0x80u

#define IO0	(uint8_t)0x01u
#define IO1	(uint8_t)0x02u
#define IO2	(uint8_t)0x04u
#define IO3	(uint8_t)0x08u
#define IO4	(uint8_t)0x10u
#define IO5	(uint8_t)0x20u
#define IO6	(uint8_t)0x40u
#define IO7	(uint8_t)0x80u

// REGISTER ADDRESSES (BANK = 1)
#define IODIRA 		(uint8_t)0x00u
#define IODIRB 		(uint8_t)0x10u
#define IPOLA		(uint8_t)0x01u
#define IPOLB		(uint8_t)0x11u
#define GPINTENA	(uint8_t)0x02u
#define GPINTENB	(uint8_t)0x12u
#define DEFVALA		(uint8_t)0x03u
#define DEFVALB		(uint8_t)0x13u
#define INTCONA		(uint8_t)0x04u
#define INTCONB		(uint8_t)0x14u
#define IOCON		(uint8_t)0x05u
#define GPPUA		(uint8_t)0x06u
#define GPPUB		(uint8_t)0x16u
#define INTFA		(uint8_t)0x07u
#define INTFB		(uint8_t)0x17u
#define INTCAPA		(uint8_t)0x08u
#define INTCAPB		(uint8_t)0x18u
#define GPIOA		(uint8_t)0x09u
#define GPIOB		(uint8_t)0x19u
#define OLATA		(uint8_t)0x0Au
#define OLATB		(uint8_t)0x1Au

#ifdef __cplusplus
extern "C"
{
#endif
/**
 *  Place all C-accessible function prototypes here.
 */
void MCP23018_Init(void);
void MCP23018_Reset(uint8_t device);
void MCP23018_Init_IOCON(void);
void MCP23018_LED(uint8_t led, uint8_t state);
uint8_t MCP23018_get_state_LED(uint8_t led);
uint8_t MCP23018_READ(uint8_t device, uint8_t reg);
void MCP23018_WRITE(uint8_t device, uint8_t reg, uint8_t data);
void MCP23018_Set_IO(uint8_t ioPin, uint8_t dir, uint8_t state);
uint8_t MCP23018_get_state_IO_PIN(uint8_t pin);
void DEBUG_ReadAllRegisters(void);
void MCP23018_SetCompPower(uint8_t state);
uint8_t MCP23018_Get_VEH_IO(uint8_t pinMask);
void MCP23018_SetPin(uint8_t device, uint8_t pinMask, uint8_t port);
void MCP23018_ClearPin(uint8_t device, uint8_t pinMask, uint8_t port);
void MCP23018_ConfigAsInput(uint8_t device, uint8_t pinMask, uint8_t port);

#ifdef __cplusplus
}
#endif

#endif /* BUILD_MCP23018_H_ */
