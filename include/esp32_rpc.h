#pragma once
#include <arduino.h>
#include "..\include\permission.h"
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
0x31	MAX30003 Pace	        Pace Pace...
0x32	MAX30003 RtoR	        RtoR
0x33	MAX30003 BioZ	        BioZ BioZ...
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

///MAX30003 Registers
typedef enum
{
    NO_OP          = 0x00,
    STSREG         = 0x01,
    EN_INT         = 0x02,
    EN_INT2        = 0x03,
    MNGR_INT       = 0x04,
    MNGR_DYN       = 0x05,
    SW_RST         = 0x08,
    SYNCH          = 0x09,
    FIFO_RST       = 0x0A,
    INFO           = 0x0F,
    CNFG_GEN       = 0x10,
    CNFG_CAL       = 0x12,
    CNFG_EMUX      = 0x14,
    CNFG_ECG       = 0x15,
    CNFG_RTOR1     = 0x1D,
    CNFG_RTOR2     = 0x1E,
    ECG_FIFO_BURST = 0x20,
    ECG_FIFO       = 0x21,
    RTOR           = 0x25,
    NO_OP2         = 0x7F
} MAX30003_REG_map_t;

typedef enum {
  MAX30003_NO_INT = 0,  // No interrupt
  MAX30003_INT_B  = 1,  // INTB selected for interrupt
  MAX30003_INT_2B = 2   // INT2B selected for interrupt
} max30003_intrpt_Location_t;

typedef enum {
  MAX30003_INT_DISABLED = 0b00,
  MAX30003_INT_CMOS     = 0b01,
  MAX30003_INT_ODN      = 0b10,
  MAX30003_INT_ODNR     = 0b11
} max30003_intrpt_type_t;

union max30003_info_reg {
    uint32_t all;
    struct {
      uint32_t serial    : 12;
      uint32_t part_id   : 2;
      uint32_t sample    : 1;
      uint32_t reserved1 : 1;
      uint32_t rev_id    : 4;
      uint32_t pattern   : 4;
      uint32_t reserved  : 8;
    } bit;
 
  } max30003_info;

 union max30003_status_reg {
    uint32_t all;
 
    struct {
      uint32_t loff_nl : 1;
      uint32_t loff_nh : 1;
      uint32_t loff_pl : 1;
      uint32_t loff_ph : 1;
 
      uint32_t bcgmn     : 1;
      uint32_t bcgmp     : 1;
      uint32_t reserved1 : 1;
      uint32_t reserved2 : 1;
 
      uint32_t pllint : 1;
      uint32_t samp   : 1;
      uint32_t rrint  : 1;
      uint32_t lonint : 1;
 
      uint32_t pedge  : 1;
      uint32_t povf   : 1;
      uint32_t pint   : 1;
      uint32_t bcgmon : 1;
 
      uint32_t bundr : 1;
      uint32_t bover : 1;
      uint32_t bovf  : 1;
      uint32_t bint  : 1;
 
      uint32_t dcloffint : 1;
      uint32_t fstint    : 1;
      uint32_t eovf      : 1;
      uint32_t eint      : 1;
 
      uint32_t reserved : 8;
 
    } bit;
 
  } max30003_status;


 union max30003_en_int_reg {
    uint32_t all;
 
    struct {
      uint32_t intb_type : 2;
      uint32_t reserved1 : 1;
      uint32_t reserved2 : 1;
 
      uint32_t reserved3 : 1;
      uint32_t reserved4 : 1;
      uint32_t reserved5 : 1;
      uint32_t reserved6 : 1;
 
      uint32_t en_pllint : 1;
      uint32_t en_samp   : 1;
      uint32_t en_rrint  : 1;
      uint32_t en_lonint : 1;
 
      uint32_t en_pedge  : 1;
      uint32_t en_povf   : 1;
      uint32_t en_pint   : 1;
      uint32_t en_bcgmon : 1;
 
      uint32_t en_bundr : 1;
      uint32_t en_bover : 1;
      uint32_t en_bovf  : 1;
      uint32_t en_bint  : 1;
 
      uint32_t en_dcloffint : 1;
      uint32_t en_fstint    : 1;
      uint32_t en_eovf      : 1;
      uint32_t en_eint      : 1;
 
      uint32_t reserved : 8;
 
    } bit;
 
  } max30003_en_int;
 
 union max30003_en_int2_reg {
    uint32_t all;
 
    struct {
      uint32_t intb_type : 2;
      uint32_t reserved1 : 1;
      uint32_t reserved2 : 1;
 
      uint32_t reserved3 : 1;
      uint32_t reserved4 : 1;
      uint32_t reserved5 : 1;
      uint32_t reserved6 : 1;
 
      uint32_t en_pllint : 1;
      uint32_t en_samp   : 1;
      uint32_t en_rrint  : 1;
      uint32_t en_lonint : 1;
 
      uint32_t en_pedge  : 1;
      uint32_t en_povf   : 1;
      uint32_t en_pint   : 1;
      uint32_t en_bcgmon : 1;
 
      uint32_t en_bundr  : 1;
      uint32_t en_bover  : 1;
      uint32_t en_bovf   : 1;
      uint32_t en_bint   : 1;
 
      uint32_t en_dcloffint : 1;
      uint32_t en_fstint    : 1;
      uint32_t en_eovf      : 1;
      uint32_t en_eint      : 1;
 
      uint32_t reserved : 8;
 
    } bit;
 
  } max30003_en_int2;
 
union max30003_mngr_int_reg {
    uint32_t all;
 
    struct {
      uint32_t samp_it   : 2;
      uint32_t clr_samp  : 1;
      uint32_t clr_pedge : 1;
      uint32_t clr_rrint : 2;
      uint32_t clr_fast  : 1;
      uint32_t reserved1 : 1;
      uint32_t reserved2 : 4;
      uint32_t reserved3 : 4;
      uint32_t b_fit     : 3;
      uint32_t e_fit     : 5;
      uint32_t reserved : 8;
    } bit;
  } max30003_mngr_int;
 
union max30003_mngr_dyn_reg {
    uint32_t all;
 
    struct {
      uint32_t bloff_lo_it : 8;
      uint32_t bloff_hi_it : 8;
      uint32_t fast_th     : 6;
      uint32_t fast        : 2;
      uint32_t reserved    : 8;
    } bit;
  } max30003_mngr_dyn;

 
 union max30003_cnfg_gen_reg {
    uint32_t all;
    struct {
      uint32_t rbiasn     : 1;
      uint32_t rbiasp     : 1;
      uint32_t rbiasv     : 2;
      uint32_t en_rbias   : 2;
      uint32_t vth        : 2;
      uint32_t imag       : 3;
      uint32_t ipol       : 1;
      uint32_t en_dcloff  : 2;
      uint32_t en_bloff   : 2;
      uint32_t reserved1  : 1;
      uint32_t en_pace    : 1;
      uint32_t en_bioz    : 1;
      uint32_t en_ecg     : 1;
      uint32_t fmstr      : 2;
      uint32_t en_ulp_lon : 2;
      uint32_t reserved : 8;
    } bit;
 
  } max30003_cnfg_gen;

union max30003_cnfg_cal_reg {
    uint32_t all;
    struct {
      uint32_t thigh     : 11;
      uint32_t fifty     : 1;
      uint32_t fcal      : 3;
      uint32_t reserved1 : 5;
      uint32_t vmag      : 1;
      uint32_t vmode     : 1;
      uint32_t en_vcal   : 1;
      uint32_t reserved2 : 1;
      uint32_t reserved  : 8;
    } bit;
  } max30003_cnfg_cal;

union max30003_cnfg_ecg_reg {
    uint32_t all;
    struct {
      uint32_t reserved1 : 12;
      uint32_t dlpf      : 2;
      uint32_t dhpf      : 1;
      uint32_t reserved2 : 1;
      uint32_t gain      : 2;
      uint32_t reserved3 : 4;
      uint32_t rate      : 2;
      uint32_t reserved  : 8;
    } bit;
  } max30003_cnfg_ecg;

  union max30003_cnfg_emux_reg {
    uint32_t all;
    struct {
      uint32_t reserved1 : 16;
      uint32_t caln_sel  : 2;
      uint32_t calp_sel  : 2;
      uint32_t openn     : 1;
      uint32_t openp     : 1;
      uint32_t reserved2 : 1;
      uint32_t pol       : 1;
      uint32_t reserved : 8;
    } bit;
 
  } max30003_cnfg_emux;

  union max30003_cnfg_rtor1_reg {
    uint32_t all;
    struct {
      uint32_t reserved1 : 8;
      uint32_t ptsf      : 4;
      uint32_t pavg      : 2;
      uint32_t reserved2 : 1;
      uint32_t en_rtor   : 1;
      uint32_t gain      : 4;
      uint32_t wndw      : 4;
      uint32_t reserved : 8;
    } bit;
 
  } max30003_cnfg_rtor1;
 
  union max30003_cnfg_rtor2_reg {
    uint32_t all;
    struct {
      uint32_t reserved1 : 8;
      uint32_t rhsf      : 3;
      uint32_t reserved2 : 1;
      uint32_t ravg      : 2;
      uint32_t reserved3 : 2;
      uint32_t hoff      : 6;
      uint32_t reserved4 : 2;
      uint32_t reserved : 8;
    } bit;
 
  } max30003_cnfg_rtor2;
 
int MAX30003_WriteReg(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_ReadReg(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_Start(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_Stop(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_Rbias_FMSTR_Init(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_CAL_InitStart(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_ECG_InitStart(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_ECGFast_Init(char argStrs[32][32], char replyStrs[32][32]);
//int MAX30003_PACE_InitStart(char argStrs[32][32], char replyStrs[32][32]);
//int MAX30003_BIOZ_InitStart(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_RtoR_InitStart(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_Stop_ECG(char argStrs[32][32], char replyStrs[32][32]);
//int MAX30003_Stop_PACE(char argStrs[32][32], char replyStrs[32][32]);
//int MAX30003_Stop_BIOZ(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_Stop_RtoR(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_Stop_Cal(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_Enable_ECG_LeadON(char argStrs[32][32], char replyStrs[32][32]);
//int MAX30003_Enable_BIOZ_LeadON(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_Read_LeadON(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_StartTest(char argStrs[32][32], char replyStrs[32][32]);
int MAX30003_INT_assignment(char argStrs[32][32], char replyStrs[32][32]);

int Led_On(char argStrs[32][32], char replyStrs[32][32]);
int Led_Off(char argStrs[32][32], char replyStrs[32][32]);
int Led_BlinkHz(char argStrs[32][32], char replyStrs[32][32]);
int Led_BlinkPattern(char argStrs[32][32], char replyStrs[32][32]);
