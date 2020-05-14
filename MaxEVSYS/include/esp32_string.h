#pragma once

#include <arduino.h>

/// indicates that a string up to a CRLF is being accumulated
#define GETLINE_WAITING 1
/// indicates that a string is being processes
#define GETLINE_PROCESSING 2
/// indicates that a CRLF string has been buffered and can be processed
#define GETLINE_DONE 3
 
/**
* @brief Clear the incoming USB read fifo
*/
void clearOutReadFifo(void);
/**
* @brief Block until a character can be read from the USB
* @return the character read
*/
char getch(void);
/**
* @brief Place incoming USB characters into a fifo
* @param lineBuffer buffer to place the incoming characters
* @param bufferLength length of buffer
* @return GETLINE_WAITING if still waiting for a CRLF, GETLINE_DONE
*/
int getLine(char *lineBuffer, int bufferLength);
/**
* @brief Block until a fixed number of characters has been accumulated from the
* incoming USB
* @param lineBuffer buffer to place the incoming characters
* @param maxLength length of buffer
*/
void getStringFixedLength(uint8_t *lineBuffer, int maxLength);
/**
* @brief Output a string out the USB serial port
* @param str output this str the USB channel
*/
int putStr(const char *str);
/**
* @brief Place incoming USB characters into a fifo
* @param data_IN buffer of characters
* @param len length of data
*/
int fifoIncomingChars(uint8_t data_IN[], unsigned int len);
/**
* @brief Outut an array of bytes out the USB serial port
* @param data buffer to output
* @param length length of buffer
*/
int putBytes(uint8_t *data, uint32_t length);
/**
* @brief Outut 256 byte blocks out the USB serial using writeBlock bulk
* transfers
* @param data buffer of blocks to output
* @param length length of 256-byte blocks
*/
int putBytes256Block(uint8_t *data, int numberBlocks);
