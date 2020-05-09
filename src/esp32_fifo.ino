#include "..\include\permission.h"
#include "..\include\MAX30003.h"

#include "..\include\esp32_rpc.h"
#include "..\include\esp32_rpcserver.h"
#include "..\include\esp32_fifo.h"
#include "..\include\esp32_streaming.h"
#include "..\include\esp32_string.h"
#include "..\include\esp32_helpers.h"

#define __disable_irq() delay(1)
#define __enable_irq()  delay(1)

/****************************************************************************/
void fifo_init(fifo_t *fifo, void *mem, unsigned int length) {
  // atomic FIFO access
  __disable_irq();
 
  fifo->rindex = 0;
  fifo->windex = 0;
  fifo->data = mem;
  fifo->length = length;
 
  __enable_irq();
}
 
/****************************************************************************/
int fifo_put8(fifo_t *fifo, uint8_t element) {
  // Check if FIFO is full
  if ((fifo->windex == (fifo->rindex - 1)) ||
      ((fifo->rindex == 0) && (fifo->windex == (fifo->length - 1)))) {
    return -1;
  }
 
  // atomic FIFO access
  __disable_irq();
 
  // Put data into FIFO
  ((uint8_t *)(fifo->data))[fifo->windex] = element;
 
  // Increment pointer
  fifo->windex++;
  if (fifo->windex == fifo->length) {
    fifo->windex = 0;
  }
 
  __enable_irq();
 
  return 0;
}
 
/****************************************************************************/
int fifo_get8(fifo_t *fifo, uint8_t *element) {
  // Check if FIFO is empty
  if (fifo->rindex == fifo->windex)
    return -1;
 
  // atomic FIFO access
  __disable_irq();
 
  // Get data from FIFO
  *element = ((uint8_t *)(fifo->data))[fifo->rindex];
 
  // Increment pointer
  fifo->rindex++;
  if (fifo->rindex == fifo->length) {
    fifo->rindex = 0;
  }
 
  __enable_irq();
 
  return 0;
}
 
/****************************************************************************/
int fifo_put16(fifo_t *fifo, uint16_t element) {
  // Check if FIFO is full
  if ((fifo->windex == (fifo->rindex - 1)) ||
      ((fifo->rindex == 0) && (fifo->windex == (fifo->length - 1)))) {
    return -1;
  }
 
  // atomic FIFO access
  __disable_irq();
 
  // Put data into FIFO
  ((uint16_t *)(fifo->data))[fifo->windex] = element;
 
  // Increment pointer
  fifo->windex++;
  if (fifo->windex == fifo->length) {
    fifo->windex = 0;
  }
 
  __enable_irq();
 
  return 0;
}
 
/****************************************************************************/
int fifo_get16(fifo_t *fifo, uint16_t *element) {
  // Check if FIFO is empty
  if (fifo->rindex == fifo->windex)
    return -1;
 
  // atomic FIFO access
  __disable_irq();
 
  // Get data from FIFO
  *element = ((uint16_t *)(fifo->data))[fifo->rindex];
 
  // Increment pointer
  fifo->rindex++;
  if (fifo->rindex == fifo->length) {
    fifo->rindex = 0;
  }
 
  __enable_irq();
 
  return 0;
}
 
/****************************************************************************/
int fifo_put32(fifo_t *fifo, uint32_t element) {
  // Check if FIFO is full
  if ((fifo->windex == (fifo->rindex - 1)) ||
      ((fifo->rindex == 0) && (fifo->windex == (fifo->length - 1)))) {
    return -1;
  }
 
  // atomic FIFO access
  __disable_irq();
 
  // Put data into FIFO
  ((uint32_t *)(fifo->data))[fifo->windex] = element;
 
  // Increment pointer
  fifo->windex++;
  if (fifo->windex == fifo->length) {
    fifo->windex = 0;
  }
 
  __enable_irq();
 
  return 0;
}
 
/****************************************************************************/
int fifo_get32(fifo_t *fifo, uint32_t *element) {
  // Check if FIFO is empty
  if (fifo->rindex == fifo->windex)
    return -1;
 
  // atomic FIFO access
  __disable_irq();
 
  // Get data from FIFO
  *element = ((uint32_t *)(fifo->data))[fifo->rindex];
 
  // Increment pointer
  fifo->rindex++;
  if (fifo->rindex == fifo->length) {
    fifo->rindex = 0;
  }
 
  __enable_irq();
 
  return 0;
}
/****************************************************************************/
void fifo_clear(fifo_t *fifo) {
  // atomic FIFO access
  __disable_irq();
 
  fifo->rindex = 0;
  fifo->windex = 0;
 
  __enable_irq();
}
 
/****************************************************************************/
int fifo_empty(fifo_t *fifo) { return (fifo->rindex == fifo->windex); }
 
/****************************************************************************/
int fifo_full(fifo_t *fifo) {
  int retval;
 
  // atomic FIFO access
  __disable_irq();
  retval = ((fifo->windex == (fifo->rindex - 1)) ||
            ((fifo->rindex == 0) && (fifo->windex == (fifo->length - 1))));
  __enable_irq();
 
  return retval;
}
 
/****************************************************************************/
unsigned int fifo_level(fifo_t *fifo) {
  uint16_t value;
 
  // atomic FIFO access
  __disable_irq();
 
  if (fifo->windex >= fifo->rindex) {
    value = fifo->windex - fifo->rindex;
  } else {
    value = fifo->length - fifo->rindex + fifo->windex;
  }
 
  __enable_irq();
 
  return value;
}
 
/****************************************************************************/
unsigned int fifo_remaining(fifo_t *fifo) {
  uint16_t value;
 
  // atomic FIFO access
  __disable_irq();
 
  if (fifo->rindex > fifo->windex) {
    value = fifo->rindex - fifo->windex - 1;
  } else {
    value = fifo->length - fifo->windex + fifo->rindex - 1;
  }
 
  __enable_irq();
 
  return value;
}

// this will create a packet and insert it on the "jFifo" to be streamed out or
// saved in flash
void PacketFifo_InsertPacket(uint32_t packetId, uint32_t *buffer,
                             uint32_t numberInBuffer) {
  StreamPacketUint32(packetId, buffer, numberInBuffer);
}
 
// clears the packet fifo "jFifo"
void PacketFifo_Clear(void) { fifo_clear(GetStreamOutFifo()); }
 
// returns one if fifo is empty, zero if not empty
uint32_t PacketFifo_Empty(void) { return fifo_empty(GetStreamOutFifo()); }
 
// returns a uint32 from the fifo, this uint32 is destined to be streamed out
// USB or saved in flash
uint32_t PacketFifo_GetUint32(void) {
  uint32_t val;
  fifo_get32(GetStreamOutFifo(), &val);
  return val;
}
