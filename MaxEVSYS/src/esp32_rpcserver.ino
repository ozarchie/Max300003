#include <arduino.h>
#include "..\include\esp32_rpcserver.h"
#include "..\include\esp32_helpers.h"
#include "..\include\version.h"
 
/// define the version string that is reported with a RPC "ReadVer" command
#define FW_VERSION_STRING "MAX30001 FW Ver"
 
char args[32][32];
char results[32][32];

static fifo_t fifo;                           // define a fifo for incoming USB data
static uint8_t fifoBuffer[128];               // define a buffer for incoming USB data
static fifo_t fifoStreamOut;                  // define stream out fifo
static uint32_t streamOutBuffer[0xC000 / 4];  // allocate a large fifo buffer for streaming out
 
//******************************************************************************
fifo_t *GetUSBIncomingFifo(void) { return &fifo; }
 
//******************************************************************************
fifo_t *GetStreamOutFifo(void) { return &fifoStreamOut; }
 
//******************************************************************************
int System_ReadVer(char argStrs[32][32], char replyStrs[32][32]) {
  char version[32];
  snprintf(version, sizeof(version)/sizeof(char), "%s %d.%d.%d %02d/%02d/%02d", 
  FW_VERSION_STRING, VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH,
  VERSION_MONTH, VERSION_DAY, VERSION_SHORT_YEAR);
  strcpy(replyStrs[0], version);
  strcpy(replyStrs[1], "\0");
  return 0;
}
 
//******************************************************************************
int System_ReadBuildTime(char argStrs[32][32], char replyStrs[32][32]) {
  // strcpy(replyStrs[0],buildTime);
  // strcpy(replyStrs[1],"\0");
  return 0;
}
 
//******************************************************************************
int System_SystemCoreClock(char argStrs[32][32], char replyStrs[32][32]) {
  sprintf(replyStrs[0], "SystemCoreClock = %d", (int) millis());
  strcpy(replyStrs[1], "\0");
  return 0;
}
 
//******************************************************************************
int System_GetTimestamp(char argStrs[32][32], char replyStrs[32][32]) {
  sprintf(replyStrs[0], "GetTimestamp = %d", 0);
  strcpy(replyStrs[1], "\0");
  return 0;
}
 
static struct RPC_Object RPC_Procedures = {NULL, NULL};
 
//******************************************************************************
void RPC_addProcedure(struct RPC_registeredProcedure *procedure) {
  struct RPC_Object *obj = &RPC_Procedures;
  if (obj->last != NULL) {
    obj->last->next = procedure;
  }
  if (obj->head == NULL) {
    obj->head = procedure;
  }
  procedure->next = NULL;
  obj->last = procedure;
}
 
//******************************************************************************
void RPC_init(void) {
 
  fifo_init(&fifo, fifoBuffer, sizeof(fifoBuffer));
  fifo_init(&fifoStreamOut, streamOutBuffer,
            sizeof(streamOutBuffer) / sizeof(uint32_t));
 
  // MAX30003
  RPC_addProcedure(&Define_MAX30003_WriteReg);
  RPC_addProcedure(&Define_MAX30003_ReadReg);
  RPC_addProcedure(&Define_MAX30003_Start);
  RPC_addProcedure(&Define_MAX30003_Stop);
  RPC_addProcedure(&Define_MAX30003_Read_LeadON);
  RPC_addProcedure(&Define_MAX30003_StartTest);
  RPC_addProcedure(&Define_MAX30003_INT_assignment);
  RPC_addProcedure(&Define_MAX30003_Rbias_FMSTR_Init);
  RPC_addProcedure(&Define_MAX30003_CAL_InitStart);
  RPC_addProcedure(&Define_MAX30003_ECG_InitStart);
  RPC_addProcedure(&Define_MAX30003_ECGFast_Init);
  RPC_addProcedure(&Define_MAX30003_RtoR_InitStart);
  RPC_addProcedure(&Define_MAX30003_Stop_ECG);
  RPC_addProcedure(&Define_MAX30003_Stop_RtoR);
  RPC_addProcedure(&Define_MAX30003_Stop_Cal);

  // led
  RPC_addProcedure(&Define_Led_On);
  RPC_addProcedure(&Define_Led_Off);
  RPC_addProcedure(&Define_Led_BlinkHz);
  RPC_addProcedure(&Define_Led_BlinkPattern);
 
  // System
  RPC_addProcedure(&Define_System_ReadVer);
  RPC_addProcedure(&Define_System_ReadBuildTime);
}
 
//******************************************************************************
struct RPC_registeredProcedure *RPC_lookup(char *objectName, char *methodName) {
  struct RPC_registeredProcedure *ptr;
  ptr = RPC_Procedures.head;  // lookup all registered methods
  while (ptr != NULL) {
    if (strcmp(ptr->objectName, objectName) == 0 &&
        strcmp(ptr->methodName, methodName) == 0) {
      return ptr;      // we found a match... return with it
    }
    ptr = ptr->next;
  }
  return NULL;
}
 
//******************************************************************************
char *GetToken(char *inStr, char *outStr, int start, char ch) {
  int i;
  int index = 0;
  int length = strlen(inStr);
  for (i = start; i < length; i++) {
    if (inStr[i] != ch) {
      outStr[index++] = inStr[i];
    } else {
      break;
    }
  }
  outStr[index++] = 0;
  return outStr;
}
 
//******************************************************************************
void SendCommandList(char *reply) {
  struct RPC_registeredProcedure *ptr;
  reply[0] = 0;
  ptr = RPC_Procedures.head;
  while (ptr != NULL) {
    strcat(reply, "/");
    strcat(reply, ptr->objectName);
    strcat(reply, "/");
    strcat(reply, ptr->methodName);
    strcat(reply, ",");
    ptr = ptr->next;
  }
  strcat(reply, "\r\n");
}
 
//******************************************************************************
int CheckForDoubleQuote(char *str) {
  int doubleQuoteFound;
  // scan through arguments, see if there is a double quote for a string
  // argument
  doubleQuoteFound = 0;
  while (*str != 0) {
    if (*str == '\"') {
      doubleQuoteFound = 1;
      break;
    }
    str++;
  }
  return doubleQuoteFound;
}
 
//******************************************************************************
void ExtractDoubleQuoteStr(char *src, char *dst) {
  int start;
 
  dst[0] = 0;
  start = 0;
  while (*src != 0) {
    // look for start
    if ((*src == '\"') && (start == 0)) {
      start = 1;
      src++;
      continue;
    }
    // look for end
    if ((*src == '\"') && (start == 1)) {
      *dst = 0; // terminate the string
      break;
    }
    if (start == 1) {
      *dst = *src;
      dst++;
    }
    src++;
  }
}
 
//******************************************************************************
void RPC_call_test(void) {
  int doubleQuoteFound;
  char doubleQuoteStr[64];
  char *request = (char*)"/Logging/AppendMissionCmd \"BMP280 InitStart 1\"";
 
  // scan through arguments, see if there is a double quote for a string
  // argument
  doubleQuoteFound = CheckForDoubleQuote(request);
  if (doubleQuoteFound) {
    ExtractDoubleQuoteStr(request, doubleQuoteStr);
  }
}
 
//******************************************************************************
void RPC_call(char *request, char *reply) {
  const char slash[2] = "/";
  const char space[2] = " ";
  char *objectName;
  char *methodName;
  char doubleQuoteStr[128];
  char requestCpy[256];
  char *token;
  int argIndex;
  int resultIndex;
  int doubleQuoteFound;
  struct RPC_registeredProcedure *procedurePtr;
 
  reply[0] = 0;                   // clear out the reply
  strcpy(requestCpy, request);    // copy the request for scanning and extraction later
  if (request[0] != '/') {        // check for beginning forward slash
    return;
  }
  if (request[0] == '/' && request[1] == 0) {  // check for only a forward slash
    SendCommandList(reply);
    return;
  }
  strcat(request, " ");             // get the object name
  token = strtok(request, slash);   // token = GetToken(request, tokenBuffer, 1, '/');
  objectName = token;
  if (objectName == NULL)           // must have an object name
    return;
  token = strtok(NULL, space);      // get the method name
  methodName = token;
  if (methodName == NULL)           // must have a method name
    return;
 
// scan through arguments, see if there is a double quote for a string argument
  doubleQuoteFound = CheckForDoubleQuote(requestCpy);
 
  if (doubleQuoteFound == 0) {      // walk through arguments
    argIndex = 0;
    token = strtok(NULL, space);
    while (token != NULL) {             // save this arg in array
      strcpy(args[argIndex++], token);  // read next token arg if any
      token = strtok(NULL, space);
    }
    strcpy(args[argIndex], "\0");       // terminate the end of the string array with an empty string
    strcpy(results[0], "\0");
  } else {
    ExtractDoubleQuoteStr(requestCpy, doubleQuoteStr);    // grab out the double quote string
    argIndex = 0;
    strcpy(args[argIndex++], doubleQuoteStr);    // token = strtok(NULL, quote);
  }
 
  // alias the MAX30003 and MAX30003 names
  if (strcmp(objectName, MAX30003_NAME) == 0) {
    strcpy(objectName, MAX30003_NAME);
  }
 
  procedurePtr = RPC_lookup(objectName, methodName);
  if (procedurePtr != NULL) {
  //  Serial.printf("RPC_call: %s processing\n",requestCpy);
    procedurePtr->func(args, results);
  } else {
    Serial.printf("RPC_call: %s not found\n", requestCpy);
    Serial.printf("Unable to lookup %s %s", objectName, methodName);
  }
 
  // loop while (if) there are results to return
  resultIndex = 0;
  strcpy(reply, "\0");
  while (results[resultIndex][0] != '\0') {
    strcat(reply, results[resultIndex++]);
    strcat(reply, " ");
  }
  strcat(reply, "\r\n");
}
 
//******************************************************************************
void RPC_ProcessCmds(char *cmds) {
  char cmd[32 * 32];
  char *ptrCmds;
  char reply[512];
  ptrCmds = cmds;
 
  while (*ptrCmds != 0) {
    ptrCmds = ParseUntilCRLF(ptrCmds, cmd, sizeof(cmd));
    if (*cmd != 0) {
      RPC_call(cmd, reply);
    }
  }
}
