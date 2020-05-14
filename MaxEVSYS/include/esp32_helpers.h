#pragma once
#include <arduino.h>

typedef enum eFlags {
  eStreaming_ECG,
  eStreaming_PACE,
  eStreaming_BIOZ,
  eStreaming_RtoR
} eFlags;

void ProcessArgs(char (*)[32], uint8_t *, int );
void FormatReply(uint8_t *, int , char args[32][32]);
void ProcessArgs32(char (*)[32], uint32_t *, int );
void ProcessArgs32Dec(char (*)[32], uint32_t *, int );
 void FormatReply32(uint32_t *, int , char (*)[32]);
uint8_t StringToByte(char str[]);
uint32_t StringToInt(char str[]);
uint32_t ParseAsciiDecU32(char *);
uint32_t ParseAsciiHexU32(char *);
char *BytesToHexStr(uint8_t *, uint32_t , char *);
char *ParseUntilCRLF(char *, char *, int );
 
int MAX30003_Helper_IsStreaming(eFlags flag);
void MAX30003_Helper_SetStreamingFlag(eFlags flag, uint8_t state);
void MAX30003_Helper_Stop(void);
void MAX30003_Helper_ClearStreamingFlags(void);
int MAX30003_AnyStreamingSet(void);
void MAX30003_Helper_Debug_ShowStreamFlags(void);
void MAX30003_Helper_StartSync(void);
void MAX30003_Helper_SetupInterrupts(void);
uint8_t *MAX30003_Helper_getVersion(void);