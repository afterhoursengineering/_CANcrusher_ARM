#include "support.h"
#include <string.h>

#ifndef NULL
#define NULL 0
#endif

/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.
 */
char* itoa(int value, char* result, int base) {
	// check that the base if valid
	if (base < 2 || base > 36) { *result = '\0'; return result; }

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
	} while ( value );

	// Apply negative sign
	if (tmp_value < 0) *ptr++ = '-';
	*ptr-- = '\0';
	while(ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr--= *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

/**
 * author: Filip Roseen
 *
 * Works similar to strtok function, but can be used with strings which
 * have repeated delimiters. strtok skips entries with repeated delimiters.
 *
 * Solution found on www.stackoverflow.com:
 * http://stackoverflow.com/questions/8705844/need-to-know-when-no-data-appears-between-two-token-separators-using-strtok
 */
char *strtok_single (char * str, char const * delims)
{
  static char  * src = NULL;
  char  *  p,  * ret = 0;

  if (str != NULL){ src = str; }

  if (src == NULL){ return NULL; }

  if ((p = strpbrk (src, delims)) != NULL) {
    *p  = 0;
    ret = src;
    src = ++p;
  }

  return ret;
}

/**
 *  Return a string pointer to the INT32 converted to ASCII format.
 */
char * itoaX(int32_t s32Int)
{
	static uint8_t u8AsciiResult[11u];
	itoa(s32Int,u8AsciiResult,10u);
	return u8AsciiResult;
}
