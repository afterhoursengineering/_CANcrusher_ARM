/*
 * FlexCAN.h
 *
 *  Created on: May 6, 2015
 *      Author: After Hours Engineering, LLC
 *
 *  Ported from C++ version
 *  https://github.com/teachop/FlexCAN_Library
 */

#ifndef FLEXCAN_H_
#define FLEXCAN_H_

#include <Arduino.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct CAN_message_t {
  uint32_t id; // can identifier
  uint8_t ext; // identifier is extended
  uint8_t len; // length of data
  uint16_t timeout; // milliseconds, zero will disable waiting
  uint8_t buf[8];
} CAN_message_t;

typedef struct CAN_filter_t {
  uint8_t rtr;
  uint8_t ext;
  uint32_t id;
} CAN_filter_t;

// -------------------------------------------------------------

  struct CAN_filter_t defaultMask;

  void FlexCAN_init(uint32_t baud);
  void FlexCAN_begin(const CAN_filter_t mask);
  void FlexCAN_setFilter(const CAN_filter_t filter, uint8_t n);
  void FlexCAN_end(void);
  int  FlexCAN_available(void);
  int  FlexCAN_write(const CAN_message_t msg);
  int  FlexCAN_read(struct CAN_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* FLEXCAN_H_ */
