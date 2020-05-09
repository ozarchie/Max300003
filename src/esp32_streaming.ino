#include "..\include\permission.h"
#include "..\include\version.h"
#include "..\include\MAX30003.h"
#include "..\include\max30003_fns.h"

#include "..\include\esp32_rpc.h"
#include "..\include\esp32_rpcserver.h"
#include "..\include\esp32_fifo.h"
#include "..\include\esp32_streaming.h"
#include "..\include\esp32_string.h"
#include "..\include\esp32_helpers.h"

 
bool streaming = false;
bool dataLogging = false;
 
/**
* @brief Encodes a 0x55 0xAA signature and a simple checksum to the id byte in the 32 bit field
* @param id Streaming ID
*/
uint32_t StreamIdChecksumCalculate(uint32_t id) {
  uint32_t sum;
  uint32_t calculated;
  sum = 0x55;
  sum += 0xAA;
  sum += id;
  sum &= 0xFF;
  sum = sum << 8;
  calculated = 0x55AA0000 + sum + id;
  return calculated;
}
 
/**
* @brief Creates a packet that will be streamed via USB or saved into flash datalog memory
* @brief the packet created will be inserted into a fifo to be streamed at a later time
* @param id Streaming ID
* @param buffer Pointer to a uint32 array that contains the data to include in the packet
* @param number Number of elements in the buffer
*/
void StreamPacketUint32(uint32_t id, uint32_t *buffer, uint32_t number) {
  uint32_t checksumId;
  if (streaming == true || dataLogging == true) {
    checksumId = StreamIdChecksumCalculate(id);
    StreamFifoId(checksumId);
    StreamFifoTimeStamp();
    StreamFifoLength(number);
    StreamFifoUint32Array(buffer, number);
  }
}
 
/**
* @brief Insert a buffer into the outgoing fifo
* @param buffer Array of uint32 to send to the fifo
* @param len Length of the array
*/
int StreamFifoUint32Array(uint32_t buffer[], uint32_t len) {
  int status;
  uint32_t i;
  for (i = 0; i < len; i++) {
    status = fifo_put32(GetStreamOutFifo(), buffer[i]);
    if (status == -1) {
      printf("FIFO_OF!");
      fflush(stdout);
      while (1)
        ;
    }
  }
  return 0;
}
 
/**
* @brief Insert a timestamp into the outgoing fifo
*/
int StreamFifoTimeStamp(void) {
  int status;
  // uint32_t timer = timestamp_GetCurrent(); //RTC_GetVal();
  uint32_t timer = micros();
  status = fifo_put32(GetStreamOutFifo(), timer);
  if (status == -1) {
    printf("FIFO_OF!");
    fflush(stdout);
    while (1)
      ;
  }
  return 0;
}
 
/**
* @brief Insert a packet id into the out going fifo
* @param id The uint32 packet id
*/
int StreamFifoId(uint32_t id) {
  int status;
  status = fifo_put32(GetStreamOutFifo(), id);
  if (status == -1) {
    printf("FIFO_OF!");
    fflush(stdout);
    while (1)
      ;
  }
  return 0;
}
 
/**
* @brief Insert a length value into the out going fifo
* @param length A uint32 number representing a length
*/
int StreamFifoLength(uint32_t length) {
  int status;
  status = fifo_put32(GetStreamOutFifo(), length);
  if (status == -1) {
    printf("FIFO_OF!");
    fflush(stdout);
    while (1)
      ;
  }
  return 0;
}
 
/**
* @brief Return a value that indicates if the system is streaming data
* @returns Returns a one or zero value
*/
uint8_t IsStreaming(void) { return streaming; }
 
/**
* @brief Set a flag to indicate if streaming is enabled
* @param state A one or zero value
*/
void SetStreaming(uint8_t state) { streaming = state; }
 
/**
* @brief Set a flag to indicate if datalogging is enabled
* @param state A one or zero value
*/
void SetDataLoggingStream(uint8_t state) { dataLogging = state; }
