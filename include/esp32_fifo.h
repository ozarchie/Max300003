#pragma once

/// Structure used for FIFO management
typedef struct {
  unsigned int length; ///< FIFO size (number of elements)
  void *data;          ///< pointer to the FIFO buffer
  unsigned int rindex; ///< current FIFO read index
  unsigned int windex; ///< current FIFO write index
} fifo_t;
 
/**
* @param    fifo     FIFO on which to perform the operation
* @param    mem      memory buffer to use for FIFO element storage
* @param    length   number of elements that the memory buffer can contain
* @returns  0 if successful, -1 upon failure
*/
void fifo_init(fifo_t *fifo, void *mem, unsigned int length);
 
/**
* @brief    Adds and 8-bit element to the FIFO
* @param    fifo     FIFO on which to perform the operation
* @param    element  element to add to the FIFO
* @returns  0 if successful, -1 upon failure
*/
int fifo_put8(fifo_t *fifo, uint8_t element);
 
/**
* @brief    Gets the next 8-bit element to the FIFO
* @param    fifo     FIFO on which to perform the operation
* @param    element  pointer to where to store the element from the FIFO
* @returns  0 if successful, -1 upon failure
*/
int fifo_get8(fifo_t *fifo, uint8_t *element);
 
/**
* @brief    Adds the next 16-bit element to the FIFO
* @param    fifo     FIFO on which to perform the operation
* @param    element  element to add to the FIFO
* @returns  0 if successful, -1 upon failure
*/
int fifo_put16(fifo_t *fifo, uint16_t element);
 
/**
* @brief    Gets the next 16-bit element to the FIFO
* @param    fifo     FIFO on which to perform the operation
* @param    element  pointer to where to store the element from the FIFO
* @returns  0 if successful, -1 upon failure
*/
int fifo_get16(fifo_t *fifo, uint16_t *element);
 
/**
* @brief    Adds the next 16-bit element to the FIFO
* @param    fifo     FIFO on which to perform the operation
* @param    element  element to add to the FIFO
* @returns  0 if successful, -1 upon failure
*/
int fifo_put32(fifo_t *fifo, uint32_t element);
 
/**
* @brief    Gets the next 16-bit element to the FIFO
* @param    fifo     FIFO on which to perform the operation
* @param    element  pointer to where to store the element from the FIFO
* @returns  0 if successful, -1 upon failure
*/
int fifo_get32(fifo_t *fifo, uint32_t *element);
 
/**
* @brief    Immediately resets the FIFO to the empty state
* @param    fifo   FIFO on which to perform the operation
*/
void fifo_clear(fifo_t *fifo);
 
/**
* @brief    Determines if the FIFO is empty
* @param    fifo   FIFO on which to perform the operation
* @returns  #TRUE if FIFO is empty, #FALSE otherwise
*/
int fifo_empty(fifo_t *fifo);
 
/**
* @brief    FIFO status function
* @param    fifo   FIFO on which to perform the operation
* @returns  #TRUE if FIFO is full, #FALSE otherwise
*/
int fifo_full(fifo_t *fifo);
 
/**
* @brief    FIFO status function
* @param    fifo   FIFO on which to perform the operation
* @returns  the number of elements currently in the FIFO
*/
unsigned int fifo_level(fifo_t *fifo);
 
/**
* @brief    FIFO status function
* @param    fifo   FIFO on which to perform the operation
* @returns  the remaining elements that can be added to the FIFO
*/
unsigned int fifo_remaining(fifo_t *fifo);

/**
* this will create a packet and insert it into an outbound fifo to be streamed out or saved in flash
* @param packetId number id to assign to this packet
* @param buffer a 32-bit buffer that contains data that will be used in the packet
* @param numberInBuffer the number of 32-bit elements to be copied from the buffer
*/
void PacketFifo_InsertPacket(uint32_t packetId, uint32_t *buffer, uint32_t numberInBuffer);
 
/**
* clears the packet outbound fifo
*/
void PacketFifo_Clear(void);
 
/**
* returns one if outbound fifo is empty, zero if not empty
*/
uint32_t PacketFifo_Empty(void);
 
/** 
* returns a uint32 from the fifo, this uint32 is destined to be streamed out USB or saved in flash
*/
uint32_t PacketFifo_GetUint32(void);
 