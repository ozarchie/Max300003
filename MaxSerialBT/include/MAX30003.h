#include <arduino.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>

#define WREG 0x00
#define RREG 0x01

#define   STATUS          0x01
#define   EN_INT          0x02
#define   EN_INT2         0x03
#define   MNGR_INT        0x04
#define   MNGR_DYN        0x05
#define   SW_RST          0x08
#define   SYNCH           0x09
#define   FIFO_RST        0x0A
#define   INFO            0x0F
#define   CNFG_GEN        0x10
#define   CNFG_CAL        0x12
#define   CNFG_EMUX       0x14
#define   CNFG_ECG        0x15
#define   CNFG_RTOR1      0x1D
#define   CNFG_RTOR2      0x1E
#define   NO_OP           0x7F

    // Data locations
#define   ECG_FIFO_BURST    0x20
#define   ECG_FIFO          0x21
#define   RTOR              0x25

#define FIFOIRQ 0x80
#define OVFIRQ  0x40
#define R2RIRQ  0x04

const int EINT_STATUS_MASK =  1 << 23;

const int FIFO_VALID_SAMPLE =  0x000;
const int FIFO_FAST_SAMPLE =  0x001;
const int FIFO_VALID_LAST =  0x002;
const int FIFO_FAST_LAST =  0x003;
const int FIFO_EMPTY =  0x6;
const int FIFO_OVF =  0x7;

const int ETAG_BITS_MASK = 0x7;
const int FIFO_MAX_SAMPLES = 32;
const int FIFO_EFIT = 15;

const uint8_t MAX30003_CS_PIN = 25;    // Digital25 attached to CSB   of MAX30003
const uint8_t MAX30003_INT2B_PIN = 26; // Digital26 attached to INT2B of MAX30003
const uint8_t MAX30003_INTB_PIN = 27;  // Digital27 attached to INTB  of MAX30003

// FCLK (Use PWM)
const uint16_t FCLKPin = 13;            // Digital13 attached to FCLK  of MAX30003
//const uint16_t FCLKFreq = 32000;        // 32000 Hz
//const float    FCLKPeriod = 8;          // at 256sps
const uint16_t FCLKFreq = 32768;        // 32768 Hz
const float    FCLKPeriod = (7.8125*2); // at 128sps
const uint16_t FCLKChannel = 0;         // PWM LEDC Ch0 (LEDC is a PWM timer)
const uint16_t FCLKResolution = 8;      // Need resolution < 12 for 32kHz
const uint16_t FCLKDuty = 127;          // Square wave (50% duty cycle, ((2^8 - 1) / 2) = 127)

// NeoPixel LED
#define LED_TYPE      WS2812B
const uint16_t PIXEL_PIN        = 14;   // Digital14 attached to NeoPixel data
const uint16_t PIXEL_COUNT      = 1;
const uint16_t PIXEL_FORMAT     = NEO_GRB + NEO_KHZ800;

const uint8_t  cLedOff    = 0;
const uint8_t  cLedRed    = 1;
const uint8_t  cLedGreen  = 2;
const uint8_t  cLedBlue   = 4;

const uint16_t BRIGHTNESS       = 128;
#define LED_ON          BRIGHTNESS
#define LED_OFF         0
#define LED_FLASH       2
#define R2R_LED_FLASH   1
#define BLINK_RATE      1000

void max30003_synch(void);
void MAX30003_Write_Reg (uint8_t , uint32_t );
uint32_t MAX30003_Read_Reg(uint8_t );
void MAX30003_Read_Burst(int );
void MAX30003_begin();

void Hex8(uint8_t *, uint8_t *, uint8_t );     // formats 8-bit data in hex
void Led(uint8_t , uint8_t );                  // HAndle 3-color pixel