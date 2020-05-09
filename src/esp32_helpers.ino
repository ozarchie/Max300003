#include "..\include\permission.h"
#include "..\include\version.h"

#include "..\include\MAX30003.h"
#include "..\include\max30003_fns.h"

#include "..\include\esp32_rpc.h"
#include "..\include\esp32_rpcserver.h"
#include "..\include\esp32_fifo.h"
#include "..\include\esp32_string.h"
#include "..\include\esp32_helpers.h"

/**
 ===================================================================================================================
    String Helpers
 * /


/**
* @brief Process an array of hex alpha numeric strings representing arguments of
* type uint8
* @param args Array of strings to process
* @param argsUintPtr Pointer of uint8 to save converted arguments
* @param numberOf Number of strings to convert
*/
void ProcessArgs(char args[32][32], uint8_t *argsUintPtr, int numberOf) {
  int i;
  int val;
  for (i = 0; i < numberOf; i++) {
    sscanf(args[i], "%x", &val);
    argsUintPtr[i] = (uint8_t)val;
  }
}
 
/**
* @brief Process an array of hex alpha numeric strings representing arguments of
* type uint32
* @param args Array of strings to process
* @param argsUintPtr Pointer of uint32 to save converted arguments
* @param numberOf Number of strings to convert
*/
void ProcessArgs32(char args[32][32], uint32_t *argsUintPtr, int numberOf) {
  int i;
  int val;
  for (i = 0; i < numberOf; i++) {
    sscanf(args[i], "%x", &val);
    argsUintPtr[i] = val;
  }
}
 
/**
* @brief Process an array of decimal numeric strings representing arguments of
* type uint32
* @param args Array of strings to process
* @param argsUintPtr Pointer of uint32 to save converted arguments
* @param numberOf Number of strings to convert
*/
void ProcessArgs32Dec(char args[32][32], uint32_t *argsUintPtr, int numberOf) {
  int i;
  int val;
  for (i = 0; i < numberOf; i++) {
    sscanf(args[i], "%d", &val);
    argsUintPtr[i] = val;
  }
}
 
/**
* @brief Parse a single string in decimal format to a uint32 number
* @param str String to process
* @returns Returns the converted number of type uint32
*/
uint32_t ParseAsciiDecU32(char *str) {
  uint32_t val;
  sscanf(str, "%d", &val);
  return val;
}
 
/**
* @brief Parse a single string in hex format to a uint32 number
* @param str String to process
* @returns Returns the converted number of type uint32
*/
uint32_t ParseAsciiHexU32(char *str) {
  uint32_t val;
  sscanf(str, "%x", &val);
  return val;
}
 
/**
* @brief Process a string that is "embedded" within another string using the /"
* delimiters
* @brief Extract the string
* @param str String to process
* @returns Returns the string
*/
void ProcessString(char args[32][32], uint8_t *str, int numberOf) {
  int i;
  int start;
  uint8_t *ptr;
  uint8_t *strPtr;
  ptr = (uint8_t *)args;
  strPtr = str;
  start = 0;
  for (i = 0; i < numberOf; i++) {
    if ((start == 0) && (*ptr == '\"')) {
      start = 1;
      ptr++;
      continue;
    }
    if ((start == 1) && (*ptr == '\"')) {
      break;
    }
    if (start == 1) {
      *strPtr = *ptr;
      strPtr++;
    }
    ptr++;
  }
  // terminate the string
  *strPtr = 0;
}
 
/**
* @brief Parse an incoming string until a CRLF is encountered, dst will contain
* the parsed string
* @param src source string to process
* @param dst destination string to contain the parsed incoming string
* @param length length of incoming src string
* @returns updated pointer in src string
*/
char *ParseUntilCRLF(char *src, char *dst, int length) {
  int i;
  char *srcPtr;
 
  srcPtr = src;
  i = 0;
  *dst = 0;
  while ((*srcPtr != 0) && (*srcPtr != 13)) {
    dst[i] = *srcPtr;
    i++;
    if (i >= length)
      break;
    srcPtr++;
  }
  if (*srcPtr == 13)
    srcPtr++;
  if (*srcPtr == 10)
    srcPtr++;
  dst[i] = 0; // terminate the string
  return srcPtr;
}
 
/**
* @brief Parse an incoming string hex value into an 8-bit value
* @param str string to process
* @returns 8-bit byte that is parsed from the string
*/
uint8_t StringToByte(char str[32]) {
  int val;
  uint8_t byt;
  sscanf(str, "%x", &val);
  byt = (uint8_t)val;
  return byt;
}
 
/**
* @brief Parse an incoming string hex value into 32-bit value
* @param str string to process
* @returns 32-bit value that is parsed from the string
*/
uint32_t StringToInt(char str[32]) {
  int val;
  uint32_t byt;
  sscanf(str, "%x", &val);
  byt = (uint32_t)val;
  return byt;
}
 
/**
* @brief Format a binary 8-bit array into a string
* @param uint8Ptr byte buffer to process
* @param numberOf number of bytes in the buffer
* @param reply an array of strings to place the converted array values into
*/
void FormatReply(uint8_t *uint8Ptr, int numberOf, char reply[32][32]) {
  int i;
  for (i = 0; i < numberOf; i++) {
    sprintf(reply[i], "%02X", uint8Ptr[i]);
  }
  strcpy(reply[i], "\0");
}
 
/**
* @brief Format a binary 32-bit array into a string
* @param uint32Ptr 32-bit value buffer to process
* @param numberOf number of values in the buffer
* @param reply an array of strings to place the converted array values into
*/
void FormatReply32(uint32_t *uint32Ptr, int numberOf, char reply[32][32]) {
  int i;
  for (i = 0; i < numberOf; i++) {
    sprintf(reply[i], "%02X", uint32Ptr[i]);
  }
  strcpy(reply[i], "\0");
}
 
/**
* @brief Format a binary 8-bit array into a string
* @param data 8-bit value buffer to process
* @param length number of values in the buffer
* @param str output string buffer
* @return output string
*/
char *BytesToHexStr(uint8_t *data, uint32_t length, char *str) {
  uint32_t i;
  char tmpStr[8];
  str[0] = 0;
  for (i = 0; i < length; i++) {
    sprintf(tmpStr, "%02X ", data[i]);
    strcat(str, tmpStr);
  }
  return str;
}

static uint8_t flags[4];
 
int MAX30003_Helper_IsStreaming(eFlags flag) { 
 return flags[(uint32_t)flag]; 
}
 
void MAX30003_Helper_SetStreamingFlag(eFlags flag, uint8_t state) {
  flags[(uint32_t)flag] = state;
}
 
void MAX30003_Helper_Stop(void) {
  if (flags[(uint32_t)eStreaming_ECG] == 1) {
    max30003_Stop_ECG();
  }
  if (flags[(uint32_t)eStreaming_RtoR] == 1) {
    max30003_Stop_RtoR();
  }
  MAX30003_Helper_ClearStreamingFlags();
}
 
int MAX30003_AnyStreamingSet(void) {
  uint32_t i;
  for (i = 0; i < 4; i++) {
    if (flags[i] == 1) return 1;
  }
  return 0;
}
 
void MAX30003_Helper_StartSync(void) {
  if (MAX30003_AnyStreamingSet() == 1) {
    max30003_synch();
  }
}
 
void MAX30003_Helper_ClearStreamingFlags(void) {
  uint32_t i;
  for (i = 0; i < 4; i++) {
    flags[i] = 0;
  }
}
 
void MAX30003_Helper_Debug_ShowStreamFlags(void) {
  putStr("\r\n");
  if (flags[(uint32_t)eStreaming_ECG] == 1) {
    putStr("eStreaming_ECG, ");
  }
  if (flags[(uint32_t)eStreaming_RtoR] == 1) {
    putStr("eStreaming_RtoR, ");
  }
  putStr("\r\n");
}
 
void MAX30003_Helper_SetupInterrupts() {
    // We removed this baed on the assumption that user provides a INT_assignment command
    /*
    Peripherals::MAX30003()->MAX30003_INT_assignment(MAX30003::MAX30003_INT_B,    MAX30003::MAX30003_NO_INT,   MAX30003::MAX30003_NO_INT,  //  en_enint_loc,      en_eovf_loc,   en_fstint_loc,
                                                         MAX30003::MAX30003_INT_2B,   MAX30003::MAX30003_INT_2B,   MAX30003::MAX30003_NO_INT,  //  en_dcloffint_loc,  en_bint_loc,   en_bovf_loc,
                                                         MAX30003::MAX30003_INT_2B,   MAX30003::MAX30003_INT_2B,   MAX30003::MAX30003_NO_INT,  //  en_bover_loc,      en_bundr_loc,  en_bcgmon_loc,
                                                         MAX30003::MAX30003_INT_B,    MAX30003::MAX30003_NO_INT,   MAX30003::MAX30003_NO_INT,  //  en_pint_loc,       en_povf_loc,   en_pedge_loc,
                                                         MAX30003::MAX30003_INT_2B,   MAX30003::MAX30003_INT_B,    MAX30003::MAX30003_NO_INT,  //  en_lonint_loc,     en_rrint_loc,  en_samp_loc,
                                                         MAX30003::MAX30003_INT_ODNR, MAX30003::MAX30003_INT_ODNR);                            //  intb_Type,         int2b_Type)
    */
}
