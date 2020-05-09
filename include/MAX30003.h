#pragma once

#include <arduino.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>

#include "..\include\permission.h"
#include "..\include\esp32_rpc.h"
#include "..\include\esp32_fifo.h"
#include "..\include\esp32_helpers.h"
#include "..\include\max30003_fns.h"

#define WREG 0x00
#define RREG 0x01

#define   MAX_STATUS          0x01
#define   MAX_EN_INT          0x02
#define   MAX_EN_INT2         0x03
#define   MAX_MNGR_INT        0x04
#define   MAX_MNGR_DYN        0x05
#define   MAX_SW_RST          0x08
#define   MAX_SYNCH           0x09
#define   MAX_FIFO_RST        0x0A
#define   MAX_INFO            0x0F
#define   MAX_CNFG_GEN        0x10
#define   MAX_CNFG_CAL        0x12
#define   MAX_CNFG_EMUX       0x14
#define   MAX_CNFG_ECG        0x15
#define   MAX_CNFG_RTOR1      0x1D
#define   MAX_CNFG_RTOR2      0x1E
#define   MAX_NO_OP           0x7F

    // Data locations
#define   MAX_ECG_FIFO_BURST    0x20
#define   MAX_ECG_FIFO          0x21
#define   MAX_RTOR              0x25

#define   MAX_FIFOIRQ           0x80
#define   MAX_OVFIRQ            0x40
#define   MAX_R2RIRQ            0x04

const int EINT_STATUS_MASK =  1 << 23;

const int FIFO_VALID_SAMPLE =   0x000;
const int FIFO_FAST_SAMPLE =    0x001;
const int FIFO_VALID_LAST =     0x002;
const int FIFO_FAST_LAST =      0x003;
const int FIFO_IS_EMPTY =       0x006;
const int FIFO_OVF =            0x007;

const int ETAG_BITS_MASK = 0x7;
const int FIFO_MAX_SAMPLES = 32;
const int FIFO_EFIT = 15;

const uint8_t MAX30003_CS_PIN = 25;    // Digital25 attached to CSB   of MAX30003
const uint8_t MAX30003_INT2B_PIN = 26; // Digital26 attached to INT2B of MAX30003
const uint8_t MAX30003_INTB_PIN = 27;  // Digital27 attached to INTB  of MAX30003

// FCLK (Use PWM)
const uint16_t FCLKPin = 13;            // Digital13 attached to FCLK  of MAX30003
//const uint16_t FCLKFreq = 32000;        // 32000 Hz
//const float    FCLKtSample = 8;         // mS
const uint16_t FCLKFreq = 32768;        // 32768 Hz
const float    FCLKtSample = 7.8125;    // mS
const uint16_t FCLKChannel = 0;         // PWM LEDC Ch0 (LEDC is a PWM timer)
const uint16_t FCLKResolution = 8;      // Need resolution < 12 for 32kHz
const uint16_t FCLKDuty = 127;          // Square wave (50% duty cycle, ((2^8 - 1) / 2) = 127)

// NeoPixel LED
#define LED_TYPE      WS2812B
const uint16_t PIXEL_PIN        = 14;   // Digital14 attached to NeoPixel data
const uint16_t NUMPIXELS        = 1;
const uint16_t BRIGHTNESS       = 25;
const uint16_t FORMAT           = NEO_GRB + NEO_KHZ800;

const uint8_t  rLedOff    = 0;
const uint8_t  rLedRed    = 1;
const uint8_t  rLedGreen  = 2;
const uint8_t  rLedBlue   = 4;
const uint8_t  rLedUpdate = 8;

#define R2R_LED_ON    BRIGHTNESS
#define R2R_LED_OFF   0
#define R2R_LED_FLASH 16

// Data Packet
const uint8_t  PacketLength     = 19;

//int MAX30003_Write_Reg (MAX30003_REG_map_t , uint32_t);
//int MAX30003_Read_Reg(MAX30003_REG_map_t , uint32_t);
void MAX30003_Read_Burst(int );
void MAX30003_begin( void );

void PrintHex8(uint8_t *, uint8_t );
void r2rLed(uint8_t);
void setLed(uint8_t, uint8_t );
void setLedOn(uint8_t );
void setLedOff(uint8_t );
void setLedBlink(uint8_t );
void setLedPattern(uint8_t );

void max30003_ServiceStreaming();

int highDataRate = 0;
// Data
uint32_t max30003_ECG_FIFO_buffer[32]; // (303 for internal test)
uint32_t max30003_RtoR_data;  // This holds the RtoR data
uint32_t max30003_DCLeadOff;  // This holds the LeadOff data, Last 4 bits give
                              // the status, BIT3=LOFF_PH, BIT2=LOFF_PL,
                              // BIT1=LOFF_NH, BIT0=LOFF_NL
                              // 8th and 9th bits tell Lead off is due to ECG  
                              // 0b01 = ECG Lead Off
uint32_t max30003_LeadOn;     // This holds the LeadOn data, BIT1 = BIOZ Lead ON,
                              // BIT0 = ECG Lead ON, BIT8= Lead On Status Bit
uint32_t max30003_timeout;    // If the PLL does not respond, timeout and get out.

int inputState;               // local input state of the RPC
char request[128];            // RPC request buffer
char reply[128];              // RPC reply buffer
