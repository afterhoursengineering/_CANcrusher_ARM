/*
 * SdLog.cpp
 *
 *  Created on: May 14, 2015
 *      Author: After Hours Engineering, LLC
 */

#include "Arduino.h"
#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include "usb_rawhid.h"
#include "SdLog.h"
#include <time.h>

void logData_postProcess();
bool logData_init();
void logData_Task();

#define INIT_EPOCH_TIME_MS 1431719049000UL

//------------------------------------------------------------------------------
// Buffer definitions.
//
// The logger will use SdFat's buffer plus BUFFER_BLOCK_COUNT additional
// buffers.
//
#ifndef RAMEND
// Assume ARM. Use total of nine 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 8;
//
#elif RAMEND < 0X8FF
#error Too little SRAM
//
#elif RAMEND < 0X10FF
// Use total of two 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 1;
//
#elif RAMEND < 0X20FF
// Use total of five 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;
//
#else  // RAMEND
// Use total of 13 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 12;
#endif  // RAMEND

const uint8_t ADC_DIM = 4;

/** 24 byte structure used to hold CAN data for logging */
struct data_can_t {
  uint64_t time;			// (8 bytes) EPOCH time --> milliseconds since 1/1/1970 00:00:00:000
  uint8_t CAN_data[8u];		// (8 bytes)
  uint32_t CAN_id;			// (4 bytes) 4-byte CAN ID
  uint8_t CAN_stat;			// (1 byte) ID Extended?
  uint8_t CAN_dlc;			// (1 byte) # of bytes in the CAN message
//uint8_t dummy1;			// (1 unused memory byte)
//uint8_t dummy2;			// (1 unused memory byte)
};

/**
 * 24 byte structure used to hold GPS data for logging.
 * MSB of byte 19 indicates GPS data instead of CAN data for binary parsing.
 * uint64_t(8 bytes) and float(4 bytes) are stored in little-endian format in memory.
 */
struct data_gps_t {
	uint64_t 	time;			/** (8 bytes){0:7}  */
	float		fUtcTime;		/** (4 bytes){8:11} UTC Time hhmmss.sss as a float */
	float		fLon_minutes;	/** (4 bytes){12:15} longitude: Minutes mm.mmmmmm	     */

	uint8_t 	u8Lat_dd;		/** (1 byte){16} Latitude: degrees */
	uint8_t		u8Lon_dd;		/** (1 byte){17} Longitude: degrees */
	uint8_t     u8Placeholder;	/** (1 byte){18} Keep the struct size same as CAN_msg_t */
	uint8_t 	u8Indicators;	/** (1 byte){19}
								 *   7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
								 *   |   |   |
								 *   |   |   +______________________ 1: E, 0: W
								 *   |   +__________________________ 1: N, 0: S
								 *   +______________________________ 1: GPS Data Indicator
								 */
	float 		fLat_minutes;	/** (4 bytes){20:23} Latitude: Minutes mm.mmmmmm         */
};

// Number of data records in a block.
const uint16_t DATA_DIM = (512u - 4u)/sizeof(data_can_t);

//Compute fill so block size is 512 bytes.  FILL_DIM may be zero.
const uint16_t FILL_DIM = 512u - 4u - DATA_DIM*sizeof(data_can_t);

struct block_t {
  data_can_t data[DATA_DIM];
  uint8_t fill[FILL_DIM];
  uint16_t overrun;
  uint16_t count;
};

static uint16_t demoSample = 0;
static bool okToLog = false;


// Until a suitable function is created to get the current time, use
// a placeholder for the current time in "epoch" format
// 1431719049 = (5/15/2015 3:44:09 PM)
static uint64_t initEpochTime = INIT_EPOCH_TIME_MS; // 0x0000000055564C89
static uint32_t initSystTime;

// max number of blocks to erase per erase call
static uint32_t const ERASE_SIZE = 262144L; // = 512 x 512

static uint32_t bgnBlock, endBlock;
// Allocate extra buffer space.
static block_t block[BUFFER_BLOCK_COUNT];
static block_t* curBlock = 0;
static uint8_t* cache;
static uint32_t bgnErase;
static uint32_t endErase;
static uint32_t bn;
static uint32_t t0;
static uint32_t t1;
static uint32_t overrun;
static uint32_t overrunTotal;
static uint32_t count;
static uint32_t maxLatency;
static int32_t diff;
//uint32_t logTime;
static bool closeFile;

struct CAN_msg_t stLogData;

static bool bCanDataIsReady = false;



//==============================================================================
// Start of configuration constants.
//==============================================================================
//Interval between data records in microseconds.
const uint32_t LOG_INTERVAL_USEC = 2000;
//------------------------------------------------------------------------------
// Pin definitions.
//
// SD chip select pin.
const uint8_t SD_CS_PIN = SS;
//
// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for SD write
// overrun errors and logging continues.
const int8_t ERROR_LED_PIN = -1;
//------------------------------------------------------------------------------
// File definitions.
//
// Maximum file size in blocks.
// The program creates a contiguous file with FILE_BLOCK_COUNT 512 byte blocks.
// This file is flash erased using special SD commands.  The file will be
// truncated if logging is stopped early.

//const uint32_t FILE_BLOCK_COUNT = 256000;
const uint32_t FILE_BLOCK_COUNT = 2500;	// 1,280,000 bytes (2500 * 512 bytes)

// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "data"

//==============================================================================
// End of configuration constants.
//==============================================================================
// Temporary log file.  Will be deleted if a reset or power failure occurs.
#define TMP_FILE_NAME "tmp_log.bin"

// Size of file base name.  Must not be larger than six.
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

SdFat sd;

SdBaseFile binFile;

char binName[13] = FILE_BASE_NAME "00.bin";

const uint8_t QUEUE_DIM = BUFFER_BLOCK_COUNT + 2;

block_t* emptyQueue[QUEUE_DIM];
uint8_t emptyHead;
uint8_t emptyTail;

block_t* fullQueue[QUEUE_DIM];
uint8_t fullHead;
uint8_t fullTail;

// Advance queue index.
inline uint8_t queueNext(uint8_t ht) {
  return ht < (QUEUE_DIM - 1) ? ht + 1 : 0;
}
//==============================================================================
// Error messages stored in flash.
#define error(msg) errorFlash(F(msg))
//------------------------------------------------------------------------------
void errorFlash(const __FlashStringHelper* msg) {
  sd.errorPrint(msg);
  //fatalBlink();
}
//------------------------------------------------------------------------------


/**
 *
 */
void SdLog_Task(void)
{
	if(SdLog_OkToLog()){ logData_Task(); }
}


bool logData_init()
{
	bool retVal = true;

//	Serial.println();
//
//	Serial.print(F("DATA_DIM "));
//	Serial.println(DATA_DIM);
//	Serial.print(F("sizeof(data_can_t) "));
//	Serial.println(sizeof(data_can_t));
//	Serial.print(F("sizeof(block_t)= "));
//	Serial.println(sizeof(block_t));

	// Find unused file name.
	if (BASE_NAME_SIZE > 6){ error("FILE_BASE_NAME too long"); retVal = false; }
	while (sd.exists(binName))
	{
		if (binName[BASE_NAME_SIZE + 1] != '9') { binName[BASE_NAME_SIZE + 1]++; }
		else
		{
			binName[BASE_NAME_SIZE + 1] = '0';
			if (binName[BASE_NAME_SIZE] == '9') { error("Can't create file name"); retVal = false; }
			binName[BASE_NAME_SIZE]++;
		}
	}
	// Delete old tmp file.
	if (sd.exists(TMP_FILE_NAME))
	{
//		Serial.println(F("Deleting tmp file"));
		if (!sd.remove(TMP_FILE_NAME)) { error("Can't remove tmp file"); retVal = false; }
	}
	// Create new file.
//	Serial.println(F("Creating new file"));
	binFile.close();
	if (!binFile.createContiguous(sd.vwd(),
								TMP_FILE_NAME, 512 * FILE_BLOCK_COUNT)) { error("createContiguous failed"); retVal = false; }
	// Get the address of the file on the SD.
	if (!binFile.contiguousRange(&bgnBlock, &endBlock)) { error("contiguousRange failed"); retVal = false; }
	// Use SdFat's internal buffer.
	//uint8_t* cache = (uint8_t*)sd.vol()->cacheClear();
	cache = (uint8_t*)sd.vol()->cacheClear();
	if (cache == 0) { error("cacheClear failed"); retVal = false; }

	// Flash erase all data in the file.
//	Serial.println(F("Erasing all data"));
	//uint32_t bgnErase = bgnBlock;
	bgnErase = bgnBlock;
	//uint32_t endErase;
	while (bgnErase < endBlock)
	{
		endErase = bgnErase + ERASE_SIZE;
		if (endErase > endBlock) { endErase = endBlock; }
		if (!sd.card()->erase(bgnErase, endErase)) { error("erase failed"); retVal = false; }
		bgnErase = endErase + 1;
	}
	// Start a multiple block write.
	if (!sd.card()->writeStart(bgnBlock, FILE_BLOCK_COUNT)) { error("writeBegin failed"); retVal = false; }
	// Initialize queues.
	emptyHead = emptyTail = 0;
	fullHead = fullTail = 0;

	// Use SdFat buffer for one block.
	emptyQueue[emptyHead] = (block_t*)cache;
	emptyHead = queueNext(emptyHead);

	// Put rest of buffers in the empty queue.
	for (uint8_t i = 0; i < BUFFER_BLOCK_COUNT; i++)
	{
		emptyQueue[emptyHead] = &block[i];
		emptyHead = queueNext(emptyHead);
	}
	if(retVal)
	{
//		Serial.println(F("Logging..."));
	}
	else{
//		Serial.println(F("INIT Failed..."));
	}

	// Wait for Serial Idle.
//	Serial.flush();
	delay(10);

	bn = 0;
	t0 = millis();
	t1 = t0;
	overrun = 0;
	overrunTotal = 0;
	count = 0;
	maxLatency = 0;

	closeFile = false;

	initSystTime = millis();

	return retVal;
}


int Get_bCanDataIsReady()
{
	if(bCanDataIsReady)
	{
		bCanDataIsReady = false;
		return true;
	}
	else{ return false; }
}

void SdLog_SetCanLogData(struct CAN_msg_t *msg)
{
	stLogData.id = msg->id;
	stLogData.len = msg->len;
	stLogData.stat = msg->stat;
	stLogData.buf[0] = msg->buf[0];
	stLogData.buf[1] = msg->buf[1];
	stLogData.buf[2] = msg->buf[2];
	stLogData.buf[3] = msg->buf[3];
	stLogData.buf[4] = msg->buf[4];
	stLogData.buf[5] = msg->buf[5];
	stLogData.buf[6] = msg->buf[6];
	stLogData.buf[7] = msg->buf[7];
	bCanDataIsReady = true;
}

int GetLogData(struct CAN_msg_t *msg)
{
	msg->id = stLogData.id;
	msg->stat = stLogData.stat;
	msg->len = stLogData.len;
	msg->buf[0] = stLogData.buf[0];
	msg->buf[1] = stLogData.buf[1];
	msg->buf[2] = stLogData.buf[2];
	msg->buf[3] = stLogData.buf[3];
	msg->buf[4] = stLogData.buf[4];
	msg->buf[5] = stLogData.buf[5];
	msg->buf[6] = stLogData.buf[6];
	msg->buf[7] = stLogData.buf[7];
	return 1;
}

/**
 *  acquireData --> Collect data sample
 *
 *
 */
void acquireData(data_can_t* data)
{
	struct CAN_msg_t rxmsg;

	// This equation is only good for 49 days after power-up. Then
	// the 32-bit millis() result will roll-over.  May need to change
	// this equation to something that accounts for the rollover.
	data->time = initEpochTime + (uint64_t)(millis() - initSystTime);

	if(GetLogData(&rxmsg))
	{
		data->CAN_id = rxmsg.id;
		data->CAN_dlc = rxmsg.len;
		data->CAN_stat = rxmsg.stat;
		data->CAN_data[0] = rxmsg.buf[0];
		data->CAN_data[1] = rxmsg.buf[1];
		data->CAN_data[2] = rxmsg.buf[2];
		data->CAN_data[3] = rxmsg.buf[3];
		data->CAN_data[4] = rxmsg.buf[4];
		data->CAN_data[5] = rxmsg.buf[5];
		data->CAN_data[6] = rxmsg.buf[6];
		data->CAN_data[7] = rxmsg.buf[7];
	}
}

void acquireCanData(data_can_t* data, CAN_msg_t * rxmsg)
{
	//struct CAN_msg_t rxmsg;

	// This equation is only good for 49 days after power-up. Then
	// the 32-bit millis() result will roll-over.  May need to change
	// this equation to something that accounts for the rollover.
	data->time = initEpochTime + (uint64_t)(millis() - initSystTime);

//	if(GetLogData(&rxmsg))
//	{
		data->CAN_id = rxmsg->id;
		data->CAN_dlc = rxmsg->len;
		data->CAN_stat = rxmsg->stat;
		data->CAN_data[0] = rxmsg->buf[0];
		data->CAN_data[1] = rxmsg->buf[1];
		data->CAN_data[2] = rxmsg->buf[2];
		data->CAN_data[3] = rxmsg->buf[3];
		data->CAN_data[4] = rxmsg->buf[4];
		data->CAN_data[5] = rxmsg->buf[5];
		data->CAN_data[6] = rxmsg->buf[6];
		data->CAN_data[7] = rxmsg->buf[7];
//	}
}



boolean SdLog_SaveCanData(struct CAN_msg_t *msg)
{
	boolean bOk = true;

	if(!okToLog){ return false; }

	if (closeFile)
	{
		if (curBlock != 0 && curBlock->count >= 0)
		{
			// Put buffer in full queue.
			fullQueue[fullHead] = curBlock;
			fullHead = queueNext(fullHead);
			curBlock = 0;
		}
	}
	else
	{
		if (curBlock == 0 && emptyTail != emptyHead)
		{
			curBlock = emptyQueue[emptyTail];
			emptyTail = queueNext(emptyTail);
			curBlock->count = 0;
			curBlock->overrun = overrun;
			overrun = 0;
		}

		if (curBlock == 0) { overrun++; }
		else
		{
			bCanDataIsReady = true;
			/**
			 *  Look for available CAN message then acquire it for the LOG
			 */
			if(Get_bCanDataIsReady())
			{
				acquireCanData(&curBlock->data[curBlock->count++],msg);
				if (curBlock->count == DATA_DIM)
				{
					fullQueue[fullHead] = curBlock;
					fullHead = queueNext(fullHead);
					curBlock = 0;
				}
			}
		}
	}

	if (fullHead == fullTail)
	{
		// Exit loop if done.
		if (closeFile)
		{
			logData_postProcess();
			if(!okToLog){ return false; }
		}
	}
	else if (!sd.card()->isBusy())
	{
		// Get address of block to write.
		block_t* pBlock = fullQueue[fullTail];
		fullTail = queueNext(fullTail);
		// Write block to SD.
		uint32_t usec = micros();
		if (!sd.card()->writeData((uint8_t*)pBlock)){ error("write data failed"); return false; }
		usec = micros() - usec;
		t1 = millis();
		if (usec > maxLatency) { maxLatency = usec; }
		count += pBlock->count;

		// Add overruns and possibly light LED.
		if (pBlock->overrun)
		{
			overrunTotal += pBlock->overrun;
			return false;
			//if (ERROR_LED_PIN >= 0) { digitalWrite(ERROR_LED_PIN, HIGH); }
		}
		// Move block to empty queue.
		emptyQueue[emptyHead] = pBlock;
		emptyHead = queueNext(emptyHead);
		bn++;
		if (bn == FILE_BLOCK_COUNT)
		{
			// File full so stop and create new file
			logData_postProcess();
			if(!okToLog){ return false; }
		}
	}
	else{ return false; }

	return bOk;
}


boolean SdLog_SaveGpsData(struct SIM808_GPSPos_t *msg)
{

}


void logData_Task()
{
	// Time for next data record.
	//logTime += LOG_INTERVAL_USEC;

	if (closeFile)
	{
		if (curBlock != 0 && curBlock->count >= 0)
		{
			// Put buffer in full queue.
			fullQueue[fullHead] = curBlock;
			fullHead = queueNext(fullHead);
			curBlock = 0;
		}
	}
	else
	{
		if (curBlock == 0 && emptyTail != emptyHead)
		{
			curBlock = emptyQueue[emptyTail];
			emptyTail = queueNext(emptyTail);
			curBlock->count = 0;
			curBlock->overrun = overrun;
			overrun = 0;
		}

		//do { diff = logTime - micros(); } while(diff > 0);
		//if (diff < -10) { error("LOG_INTERVAL_USEC too small"); }

		if (curBlock == 0) { overrun++; }
		else
		{
			/**
			 *  Look for available CAN message then acquire it for the LOG
			 */
			if(Get_bCanDataIsReady())
			{
				acquireData(&curBlock->data[curBlock->count++]);
				if (curBlock->count == DATA_DIM)
				{
					fullQueue[fullHead] = curBlock;
					fullHead = queueNext(fullHead);
					curBlock = 0;
				}
			}
		}
	}

	if (fullHead == fullTail)
	{
		// Exit loop if done.
		if (closeFile) { logData_postProcess(); }
	}
	else if (!sd.card()->isBusy())
	{
		// Get address of block to write.
		block_t* pBlock = fullQueue[fullTail];
		fullTail = queueNext(fullTail);
		// Write block to SD.
		uint32_t usec = micros();
		if (!sd.card()->writeData((uint8_t*)pBlock)){ error("write data failed"); }
		usec = micros() - usec;
		t1 = millis();
		if (usec > maxLatency) { maxLatency = usec; }
		count += pBlock->count;

		// Add overruns and possibly light LED.
		if (pBlock->overrun)
		{
			overrunTotal += pBlock->overrun;
			if (ERROR_LED_PIN >= 0) { digitalWrite(ERROR_LED_PIN, HIGH); }
		}
		// Move block to empty queue.
		emptyQueue[emptyHead] = pBlock;
		emptyHead = queueNext(emptyHead);
		bn++;
		if (bn == FILE_BLOCK_COUNT)
		{
			// File full so stop
			logData_postProcess();
		}
	}

}


void logData_postProcess()
{
	if (!sd.card()->writeStop()) {
	error("writeStop failed");
	}
	// Truncate file if recording stopped early.
	if (bn != FILE_BLOCK_COUNT) {
//	Serial.println(F("Truncating file"));
	if (!binFile.truncate(512L * bn)) {
	  error("Can't truncate file");
	}
	}
	if (!binFile.rename(sd.vwd(), binName)) {
	error("Can't rename file");
	}
//	Serial.print(F("File renamed: "));
//	Serial.println(binName);
//	Serial.print(F("Max block write usec: "));
//	Serial.println(maxLatency);
//	Serial.print(F("Record time sec: "));
//	Serial.println(0.001*(t1 - t0), 3);
//	Serial.print(F("Sample count: "));
//	Serial.println(count);
//	Serial.print(F("Samples/sec: "));
//	Serial.println((1000.0)*count/(t1-t0));
//	Serial.print(F("Overruns: "));
//	Serial.println(overrunTotal);
//	Serial.println(F("Done"));

	//okToLog = false;
	/** Create a new file and continue logging */
	if(logData_init()){ okToLog = true; }
	else{ okToLog = false; }
}

boolean SdLog_OkToLog(){ return okToLog; }



//------------------------------------------------------------------------------
void SdLog_setup(void) {
  if (ERROR_LED_PIN >= 0) {
    pinMode(ERROR_LED_PIN, OUTPUT);
  }
//  Serial.begin(9600);
//  while (!Serial) {}

  delay(1000);

  /**
   *  EPOCH TIME EXAMPLE --> Given in seconds. Multiply by 1000 to get milliseconds
   *  Include <time.h> to use the struct tm and mktime function.
   */
  time_t t_of_day;
  struct tm t;
  t.tm_year = 2015-1900;
  t.tm_mon = 5;
  t.tm_mday = 15;
  t.tm_hour = 9;
  t.tm_min = 40;
  t.tm_sec = 15;
  t.tm_isdst = -1;		// -1 if you don't know,
  t_of_day = mktime(&t);


//  Serial.print(F("FreeRam: "));
//  Serial.println(FreeRam());
//  Serial.print(F("Records/block: "));
//  Serial.println(DATA_DIM);
  if (sizeof(block_t) != 512) {
    error("Invalid block size");
  }
  // initialize file system.
  if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
    sd.initErrorPrint();
    //fatalBlink();
  }

  if(logData_init()){ okToLog = true; }
}


//------------------------------------------------------------------------------



void fatalBlink() { }
//==============================================================================
// Convert binary file to csv file.
void binaryToCsv() { }

//------------------------------------------------------------------------------
// read data file and check for overruns
void checkOverrun() { }
//------------------------------------------------------------------------------
// dump data file to Serial
void dumpData() { }

void printData(Print* pr, data_can_t* data) { }

void printHeader(Print* pr) { }

void logData() { }



