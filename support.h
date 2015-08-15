/*
 * support.h
 *
 *  Created on: Apr 28, 2015
 *      Author: After Hours Engineering, LLC
 */

#ifndef SUPPORT_H_
#define SUPPORT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

char* itoa(int value, char* result, int base);
char *strtok_single (char * str, char const * delims);
char * itoaX(int32_t s32Int);


#ifdef __cplusplus
}
#endif
#endif /* SUPPORT_H_ */
