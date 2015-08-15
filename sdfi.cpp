/*
 * sdfi.cpp
 * sdFat Interface - C++ interface to C++ sdFat library for C-code access
 *
 *  Created on: May 11, 2015
 *      Author: After Hours Engineering, LLC
 */
#include "Arduino.h"
#include "sdfi.h"
#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include "usb_rawhid.h"
//#include "support.h"

const uint8_t chipSelect = SS;
byte buffer[64];

SdFile file;
SdFat sdf;

boolean SD_Init(void)
{
	return sdf.begin(chipSelect,SPI_FULL_SPEED);
}

uint16_t SD_NumberOfFiles(void)
{
	uint16_t fileCount = 0;

	buffer[0] = 0x01;
	usb_rawhid_send(buffer, 100);
	while(file.openNext(sdf.vwd(), O_READ))
	{
		fileCount++;

		buffer[0]++;
		usb_rawhid_send(buffer, 100);

	}
	file.close();
	return fileCount;
}

uint32_t SD_GetFileSize(void)
{
	uint32_t retVal = 0;
	//TEST.TXT
	sdf.chdir();	// Change directory to root
	// OPEN the file for reading:
	if (!file.open("TEST.TXT", O_READ)) {
	sdf.errorHalt("opening FILE for read failed");
	}
	else
	{
		retVal = file.fileSize();
		file.close();

	}

	return retVal;
}

boolean SD_ReadFile(const char *filename)
{
	uint32_t t_arrival = millis();
	boolean retVal = false;
	sdf.chdir();
	if (!file.open(filename, O_READ)) {
	sdf.errorHalt("opening FILE for read failed");
	}
	else
	{
		uint32_t len = file.fileSize();
		uint8_t index = 0;
		for(uint32_t i=0; i< len; i++)
		{
			int intData = file.read();
			char byteData;
			int base = 10u;
			//itoa(intData,&byteData,base);	// Convert to ASCII
			buffer[index] = (uint8_t)intData;//byteData;
			if(index < 63u){ index++; }
			else{ usb_rawhid_send(buffer, 100u); index = 0; }

		}
		file.close();
		uint32_t t_exit = millis() - t_arrival;
		buffer[62] = (uint8_t)((t_exit & 0x000000FF)>>8);
		buffer[63] = (uint8_t)t_exit;
		usb_rawhid_send(buffer,100u);

	}
}

void SD_ListFiles(void)
{

	  Serial.begin(9600);
	  while (!Serial) {}
	  delay(1000);
	  Serial.println();
	  while (file.openNext(sdf.vwd(), O_READ)) {
	    file.printFileSize(&Serial);
	    Serial.write(' ');
	    file.printModifyDateTime(&Serial);
	    Serial.write(' ');
	    file.printName(&Serial);
	    if (file.isDir()) {
	      // Indicate a directory.
	      Serial.write('/');
	    }
	    Serial.println();
	    file.close();
	  }
}






