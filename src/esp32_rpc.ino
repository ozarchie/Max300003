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

/*
RPC Commands
============
Format:
/<Object name>/<Method name> <Arguments separated by spaces><CR LF>
All arguments are expected to be in a hexadecimal format unless otherwise noted.
The return data is delimited by spaces and are in hexadecimal format unless otherwise noted.
The return data is terminated by a <CR LF>

Streaming Data
==============
Format:
Packet ID	TimeStamp	Number In Payload	Payload

The payload data is delimited by spaces and are in hexadecimal format unless otherwise noted.
The return data is terminated by a <CR LF>

Example:
Packet ID	12
TimeStamp	11223344
#Payload	22
Payload     1BF 15C 1C4 165 1CD 
            167 1D7 16E 1C9 15D 
            1B6 158 1BC 15E 1CD 
            173 1D3 16B 1CD 15D 
            1BF 158 1B9 155 1BA 
            15C 1CD 170 1D1 162 
            1CB 15F 1C0 157

12 is the hex string for the Packet ID
11223344 is the timestamp
22 is the hex string for number of data items in payload (34 decimal)

Packet ID
=========
ID	    Data Type	            Payload Format
--      ---------               --------------
0x11	MAX30101, 1 active LED	RED RED...
0x12	MAX30101, 2 active LED	RED IR RED IR...
0x13	MAX30101, 3 active LED	RED IR GREEN RED IR GREEN...
0x20	LIS2DH Accelerometer	x y z x y z...
0x30	MAX30003 ECG	        ECG ECG...

0x32	MAX30003 RtoR	        RtoR

0x34	MAX30003 Lead Off DC	
0x35	MAX30003 Lead Off AC	
0x36	MAX30003 BCGMON	
0x37	MAX30003 ACLEADON

Commands
========

/System/ReadVer                         ->  "HSP FW Version 3.0.0 10/14/16" <cr> <lf>

MAX30003
========

/MAX30003/CAL_InitStart En_Vcal Vmode Vmag Fcal Thigh Fifty
Parameter	Description
En_ecg	    ECG Channel Enable <CNFG_GEN register="" bits>
Openp	    Open the ECGN Input Switch (most often used for testing and calibration studies) <CNFG_EMUX register="" bits>
Openn	    Open the ECGN Input Switch (most often used for testing and calibration studies) <CNFG_EMUX register="" bits>
Calp_sel	ECGP Calibration Selection <CNFG_EMUX register="" bits>
Caln_sel	ECGN Calibration Selection <CNFG_EMUX register="" bits>
E_fit	    ECG FIFO Interrupt Threshold (issues EINT based on number of unread FIFO records) <CNFG_GEN register="" bits>
Clr_rrint	RTOR R Detect Interrupt (RRINT) Clear Behavior <CNFG_GEN register="" bits>
Rate	    ECG Data Rate
Gain	    ECG Channel Gain Setting
Dhpf	    ECG Channel Digital High Pass Filter Cutoff Frequency
Dlpf	    ECG Channel Digital Low Pass Filter Cutoff Frequency

/MAX30003/ECGFast_Init Clr_Fast Fast Fast_Th
Parameter	Description
Clr_Fast	FAST MODE Interrupt Clear Behavior <MNGR_INT register>
Fast	    ECG Channel Fast Recovery Mode Selection (ECG High Pass Filter Bypass) <MNGR_DYN register>
Fast_Th	    Automatic Fast Recovery Threshold

/MAX30003/Enable_ECG_LeadON

/MAX30003/INT_assignment en_enint_loc en_eovf_loc en_fstint_loc en_dcloffint_loc en_bint_loc en_bovf_loc en_bover_loc en_bundr_loc en_bcgmon_loc en_pint_loc en_povf_loc en_pedge_loc en_lonint_loc en_rrint_loc en_samp_loc intb_Type int2b_Type
Parameter	        Description
en_enint_loc	    either INTB or INT2B or NONE
en_eovf_loc	        either INTB or INT2B or NONE
en_fstint_loc	    either INTB or INT2B or NONE
en_dcloffint_loc	either INTB or INT2B or NONE
en_bint_loc	        either INTB or INT2B or NONE
en_bovf_loc	        either INTB or INT2B or NONE
en_bover_loc	    either INTB or INT2B or NONE
en_bundr_loc	    either INTB or INT2B or NONE
en_bcgmon_loc	    either INTB or INT2B or NONE
en_pint_loc	        either INTB or INT2B or NONE
en_povf_loc	        either INTB or INT2B or NONE
en_pedge_loc	    either INTB or INT2B or NONE
en_lonint_loc	    either INTB or INT2B or NONE
en_rrint_loc	    either INTB or INT2B or NONE
en_samp_loc	        either INTB or INT2B or NONE
intb_Type	        INTB Port Type (EN_INT Selections).
int2b_Type	        INT2B Port Type (EN_INT2 Selections)

/MAX30003/Rbias_FMSTR_Init En_rbias Rbias Rbiasp Rbiasn Fmstr
Parameter	Description
En_rbias	Enable and Select Resitive Lead Bias Mode
Rbias	    Resistive Bias Mode Value Selection
Rbiasp	    Enables Resistive Bias on Positive Input
Rbiasn	    Enables Resistive Bias on Negative Input
Fmstr	    Selects Master Clock Frequency

/MAX30003/Read_LeadON

/MAX30003/ReadReg address
Parameter	Description
address	    Register address to read

/MAX30003/WriteReg address data
Parameter	Description
address	    Register address to write
data	    Data to write

/MAX30003/RtoR_InitStart En_rtor Wndw Gain Pavg Ptsf Hoff Ravg Rhsf Clr_rrint
Parameter	Description
En_rtor	    ECG RTOR Detection Enable <RTOR1 register>
Gain	    R to R Window Averaging (Window Width = RTOR_WNDW[3:0]*8mS) <RTOR1 register>
Gain	    R to R Gain (where Gain = 2^RTOR_GAIN[3:0], plus an auto-scale option) <RTOR1 register>
Pavg	
Ptsf	    R to R Peak Averaging Weight Factor <RTOR1 register>
Hoff	    R to R minimum Hold Off <RTOR2 register>
Ravg	    R to R Interval Averaging Weight Factor <RTOR2 register>
Rhsf	    R to R Interval Hold Off Scaling Factor <RTOR2 register>
Clr_rrint	RTOR Detect Interrupt Clear behaviour <MNGR_INT register>

/MAX30003/Start
Start streaming MAX30003 data. The data streamed depends on the previous MAX30003 InitStart RPC commands.

/MAX30003/StartTest
Start the self test for the MAX30003, interrupts are verified using this test

/MAX30003/Stop
Stop streaming and interrupts of the MAX30003 device

/MAX30003/Stop_Cal
Stop MAX30003 Cal mode

/MAX30003/Stop_ECG
Stop ECG streaming and interrupts For the in depth meaning of these parameters please refer to the MAX30003 API document

/MAX30003/Stop_RtoR
Stop RtoR streaming and interrupts

/Testing/Test_MAX30003
Start a testing sequence for this device, returns PASS and FAIL strings and detailed results of the test

*/

uint32_t max30003_RegRead(MAX30003_REG_map_t addr) {
  uint32_t data;
  max30003_reg_read(addr, &data);
  return data;
}
 
void max30003_RegWrite(MAX30003_REG_map_t addr, uint32_t data) {
  max30003_reg_write(addr, data);
}
 
int MAX30003_WriteReg(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t args[2];
  uint32_t reply[1];

  ProcessArgs32(argStrs, args, sizeof(args) / sizeof(uint32_t));
  max30003_RegWrite((MAX30003_REG_map_t)args[0], args[1]);
  reply[0] = 0x80;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}
 
int MAX30003_ReadReg(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t args[1];
  uint32_t reply[1];

  ProcessArgs32(argStrs, args, sizeof(args) / sizeof(uint32_t));
  reply[0] = max30003_RegRead((MAX30003_REG_map_t)args[0]);
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}
 
int MAX30003_Rbias_FMSTR_Init(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t args[5];
  uint32_t reply[1];
  uint32_t value;
  ProcessArgs32(argStrs, args, sizeof(args) / sizeof(uint32_t));
  value = max30003_Rbias_FMSTR_Init(args[0],  // En_rbias
                                    args[1],  // Rbiasv
                                    args[2],  // Rbiasp
                                    args[3],  // Rbiasn
                                    args[4]); // Fmstr
  reply[0] = value;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}
 
int MAX30003_CAL_InitStart(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t args[6];
  uint32_t reply[1];
  uint32_t value;
  ProcessArgs32(argStrs, args, sizeof(args) / sizeof(uint32_t));
  value = max30003_CAL_InitStart(args[0], // En_Vcal
                                                          args[1], // Vmag
                                                          args[2], // Fcal
                                                          args[3], // Thigh
                                                          args[4], // Fifty
                                                          args[5]); // Vmode
  reply[0] = value;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}
 
int MAX30003_ECG_InitStart(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t args[11];
  uint32_t reply[1];
  uint32_t value;
  ProcessArgs32(argStrs, args, sizeof(args) / sizeof(uint32_t));
  value = max30003_ECG_InitStart( args[0], // En_ecg
                                                          args[1], // Openp
                                                          args[2], // Openn
                                                          args[3], // Pol
                                                          args[4], // Calp_sel
                                                          args[5], // Caln_sel
                                                          args[6], // E_fit
                                                          args[7], // Rate
                                                          args[8], // Gain
                                                          args[9], // Dhpf
                                                          args[10]); // Dlpf
  MAX30003_Helper_SetStreamingFlag(eStreaming_ECG, 1);
  reply[0] = value;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}
 
int MAX30003_ECGFast_Init(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t args[3];
  uint32_t reply[1];
  uint32_t value;
  ProcessArgs32(argStrs, args, sizeof(args) / sizeof(uint32_t));
  value = max30003_ECGFast_Init(args[0], // Clr_Fast
                                                         args[1], // Fast
                                                         args[2]); // Fast_Th
  reply[0] = value;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}
 
int MAX30003_RtoR_InitStart(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t args[9];
  uint32_t reply[1];
  uint32_t value;
  ProcessArgs32(argStrs, args, sizeof(args) / sizeof(uint32_t));
  value =  max30003_RtoR_InitStart(args[0], // En_rtor
                                                            args[1], // Wndw
                                                            args[2], // Gain
                                                            args[3], // Pavg
                                                            args[4], // Ptsf
                                                            args[5], // Hoff
                                                            args[6], // Ravg
                                                            args[7], // Rhsf
                                                            args[8]); // Clr_rrint
  MAX30003_Helper_SetStreamingFlag(eStreaming_RtoR, 1);
  reply[0] = value;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}
 
int MAX30003_Stop_ECG(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t reply[1];
  max30003_Stop_ECG();
  reply[0] = 0x80;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}

int MAX30003_Stop_RtoR(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t reply[1];
  max30003_Stop_RtoR();
  reply[0] = 0x80;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}

int MAX30003_Stop_Cal(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t reply[1];
  // max30003_Stop_Cal();
  reply[0] = 0x80;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}

int MAX30003_Start(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t reply[1];
  uint32_t all;

  fifo_clear(GetUSBIncomingFifo());
  max30003_synch();
  max30003_ServiceStreaming();
  highDataRate = 0;
  max30003_reg_read(STSREG, &all);

  reply[0] = 0x80;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}
 
int MAX30003_Stop(char argStrs[32][32], char replyStrs[32][32]) {
  /*    uint32_t args[1];
          uint32_t reply[1];
          uint32_t value;
          //ProcessArgs32(argStrs, args, sizeof(args) / sizeof(uint32_t));
          MAX30003_StopTest();
          reply[0] = 0x80;
          FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);*/
  return 0;
}
 
int MAX30003_INT_assignment(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t args[17];
  uint32_t reply[1];
  ProcessArgs32(argStrs, args, sizeof(args) / sizeof(uint32_t));
  /*
  printf("MAX30003_INT_assignment ");
  printf("%d ",args[0]);
  printf("%d ",args[1]);
  printf("%d ",args[2]);
  printf("%d ",args[3]);
  printf("%d ",args[4]);
  printf("%d ",args[5]);
  printf("%d ",args[6]);
  printf("%d ",args[7]);
  printf("%d ",args[8]);
  printf("%d ",args[9]);
  printf("%d ",args[10]);
  printf("%d ",args[11]);
  printf("%d ",args[12]);
  printf("%d ",args[13]);
  printf("%d ",args[14]);
  printf("%d ",args[15]);
  printf("%d ",args[16]);
  printf("\n");
  fflush(stdout);
  */
 
  max30003_INT_assignment(
      (max30003_intrpt_Location_t)args[0],
      (max30003_intrpt_Location_t)args[1],
      (max30003_intrpt_Location_t)args[2],
      (max30003_intrpt_Location_t)args[3],
      (max30003_intrpt_Location_t)args[4],
      (max30003_intrpt_Location_t)args[5],
      (max30003_intrpt_Location_t)args[6],
      (max30003_intrpt_Location_t)args[7],
      (max30003_intrpt_Location_t)args[8],
      (max30003_intrpt_Location_t)args[9],
      (max30003_intrpt_Location_t)args[10],
      (max30003_intrpt_Location_t)args[11],
      (max30003_intrpt_Location_t)args[12],
      (max30003_intrpt_Location_t)args[13],
      (max30003_intrpt_Location_t)args[14],
      (max30003_intrpt_type_t)args[15],
      (max30003_intrpt_type_t)args[16]);
      
  reply[0] = 0x80;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}
 
int MAX30003_StartTest(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t reply[1];
  // ProcessArgs32(argStrs, args, sizeof(args) / sizeof(uint32_t));
 
  /*** Set FMSTR over here ****/
 
  /*** Set and Start the VCAL input ***/
  /* NOTE VCAL must be set first if VCAL is to be used */
  max30003_CAL_InitStart(0b1, 0b1, 0b1, 0b011, 0x7FF, 0b0);
 
  /**** ECG Initialization ****/
  max30003_ECG_InitStart(0b1, 0b1, 0b1, 0b0, 0b10, 0b11, 31, 0b00, 0b00, 0b0, 0b01);
 
  /*** Set RtoR registers ***/
  max30003_RtoR_InitStart(
      0b1, 0b0011, 0b1111, 0b00, 0b0011, 0b000001, 0b00, 0b000, 0b01);
 
  /*** Set Rbias & FMSTR over here ****/
  max30003_Rbias_FMSTR_Init(0b01, 0b10, 0b1, 0b1, 0b00);
 
  /**** Interrupt Setting  ****/
 
  /*** Set ECG Lead ON/OFF ***/
  // max30003_ECG_LeadOnOff();
 
  /**** Do a Synch ****/
  max30003_synch();
  fifo_clear(GetUSBIncomingFifo());
  max30003_ServiceStreaming();
 
  reply[0] = 0x80;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}

int MAX30003_Enable_ECG_LeadON(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t reply[1];

  max30003_Enable_LeadON(0b01);   // switch to ECG DC Lead ON
  reply[0] = 0x80;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}

int MAX30003_Read_LeadON(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t reply[1];
  
  reply[0] = max30003_LeadOn; // return the MAX30003_LeadOn var from the MAX30003 driver
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}

 int Led_On(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t reply[1];
  setLedOn(rLedRed);
  reply[0] = 0x80;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}
 
int Led_Off(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t reply[1];
  setLedOff(rLedRed);
  reply[0] = 0x80;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}
 
int Led_BlinkHz(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t args[1];
  uint32_t reply[1];
  ProcessArgs32(argStrs, args, sizeof(args) / sizeof(uint32_t));
  setLedBlink(rLedRed);
  reply[0] = 0x80;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}
 
int Led_BlinkPattern(char argStrs[32][32], char replyStrs[32][32]) {
  uint32_t args[2];
  uint32_t reply[1];
  ProcessArgs32(argStrs, args, sizeof(args) / sizeof(uint32_t));
  setLedPattern(rLedRed);
  reply[0] = 0x80;
  FormatReply32(reply, sizeof(reply) / sizeof(uint32_t), replyStrs);
  return 0;
}