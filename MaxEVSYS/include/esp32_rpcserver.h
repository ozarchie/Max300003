#pragma once
#include "..\include\esp32_fifo.h"
#include "..\include\max30003_fns.h"

/**
* @brief Reads the Version of the HSP FCache_Writel
*/
int System_ReadVer(char argStrs[32][32], char replyStrs[32][32]);
/**
* @brief Reads the built time of the RPC FW
*/
int System_ReadBuildTime(char argStrs[32][32], char replyStrs[32][32]);

/// define the parts of a RPC.  ObjectName, MethodName and function
struct RPC_registeredProcedure {
    const char *objectName;
    const char *methodName;
    //enum eArgType argTypes[4];
    int (*func)(char args[32][32], char results[32][32]);
    struct RPC_registeredProcedure *next;
};
 
/// used to keep track of the head of the list and the end of a list
struct RPC_Object {
    struct RPC_registeredProcedure *head;
    struct RPC_registeredProcedure *last;
};

//example /I2c/WriteRead 1 A0 3 11 22 33 2
#define System_NAME "System"
#define MAX30003_NAME "MAX30001"
#define LED_NAME "Led"

/**
* @brief  /System/ReadVer
* @details Returns the version string of the FW that is currently running
* @details Example: /System/ReadVer
* @details The command returns a version string similar to this: "HSP FW Version 2.0.1f 8/23/16"
*/
struct RPC_registeredProcedure  Define_System_ReadVer = { System_NAME, "ReadVer", System_ReadVer };
/**
* @brief  /System/ReadBuildTime
* @details Returns the build string of the FW that is currently running, this is the time and date that the firmware was built
* @details Example: /System/ReadBuildTime
* @details The command returns a build string similar to this: "Build Time: Fri Jul 1 15:48:31 2016"
*/
struct RPC_registeredProcedure  Define_System_ReadBuildTime = { System_NAME, "ReadBuildTime", System_ReadBuildTime };

struct RPC_registeredProcedure  Define_MAX30003_WriteReg = { MAX30003_NAME, "WriteReg", MAX30003_WriteReg };
struct RPC_registeredProcedure  Define_MAX30003_ReadReg = { MAX30003_NAME, "ReadReg", MAX30003_ReadReg };
struct RPC_registeredProcedure  Define_MAX30003_Start = { MAX30003_NAME, "Start", MAX30003_Start };
struct RPC_registeredProcedure  Define_MAX30003_Stop = { MAX30003_NAME, "Stop", MAX30003_Stop };
struct RPC_registeredProcedure  Define_MAX30003_Rbias_FMSTR_Init = { MAX30003_NAME, "Rbias_FMSTR_Init", MAX30003_Rbias_FMSTR_Init };
struct RPC_registeredProcedure  Define_MAX30003_CAL_InitStart = { MAX30003_NAME, "CAL_InitStart", MAX30003_CAL_InitStart };
struct RPC_registeredProcedure  Define_MAX30003_ECG_InitStart = { MAX30003_NAME, "ECG_InitStart", MAX30003_ECG_InitStart };
struct RPC_registeredProcedure  Define_MAX30003_ECGFast_Init = { MAX30003_NAME, "ECGFast_Init", MAX30003_ECGFast_Init };
struct RPC_registeredProcedure  Define_MAX30003_RtoR_InitStart = { MAX30003_NAME, "RtoR_InitStart", MAX30003_RtoR_InitStart };
struct RPC_registeredProcedure  Define_MAX30003_Stop_ECG = { MAX30003_NAME, "Stop_ECG", MAX30003_Stop_ECG };
struct RPC_registeredProcedure  Define_MAX30003_Stop_RtoR = { MAX30003_NAME, "Stop_RtoR", MAX30003_Stop_RtoR };
struct RPC_registeredProcedure  Define_MAX30003_Stop_Cal = { MAX30003_NAME, "Stop_Cal", MAX30003_Stop_Cal };
struct RPC_registeredProcedure  Define_MAX30003_Enable_ECG_LeadON = { MAX30003_NAME, "Enable_ECG_LeadON", MAX30003_Enable_ECG_LeadON };
struct RPC_registeredProcedure  Define_MAX30003_Read_LeadON = { MAX30003_NAME, "Read_LeadON", MAX30003_Read_LeadON };
struct RPC_registeredProcedure  Define_MAX30003_StartTest = { MAX30003_NAME, "StartTest", MAX30003_StartTest };

struct RPC_registeredProcedure  Define_Led_On = { LED_NAME, "On", Led_On };
struct RPC_registeredProcedure  Define_Led_Off = { LED_NAME, "Off", Led_Off };
struct RPC_registeredProcedure  Define_Led_BlinkHz = { LED_NAME, "Blink", Led_BlinkHz };
struct RPC_registeredProcedure  Define_Led_BlinkPattern = { LED_NAME, "Pattern", Led_BlinkPattern };struct RPC_registeredProcedure  Define_MAX30003_INT_assignment = { MAX30003_NAME, "INT_assignment", MAX30003_INT_assignment };

void RPC__init(void);
void RPC__call(char *request, char *reply);
fifo_t *GetUSBIncomingFifo(void);
fifo_t *GetStreamOutFifo(void);
/**
* @brief Batch process RPC commands
*/
void RPC_ProcessCmds(char *cmds);
/**
* @brief Initialize the RPC server with all of the commands that it supports
*/
void RPC_init(void);
/**
* @brief Initialize the RPC server with all of the commands that it supports
*/
void RPC_call(char *request, char *reply);
