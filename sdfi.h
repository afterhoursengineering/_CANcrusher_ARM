/*
 * sdfi.h
 *
 *  Created on: May 11, 2015
 *      Author: After Hours Engineering, LLC
 */

#ifndef SDFI_H_
#define SDFI_H_

#ifdef __cplusplus
extern "C"
{
#endif
	boolean SD_Init(void);
	uint16_t SD_NumberOfFiles(void);
	void SD_ListFiles(void);
	uint32_t SD_GetFileSize(void);
	boolean SD_ReadFile(const char *filename);
#ifdef __cplusplus
}
#endif

#endif /* SDFI_H_ */
