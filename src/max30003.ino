#include <arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>

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

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#undef  CALIBRATION
#define ECG_READ

#undef  SERIAL_MONITOR
#undef SERIAL_PLOTTER

BluetoothSerial SerialBT;
String BTname = "HeartSensor";
// char *BTpin = "1234";
char BTSendBuffer[32];
bool BTconnected;
int BTCount = 0;
String BTcallback = "";
String BTreadData = "";

uint8_t rLED = rLedOff;       // Red Status LED
uint8_t gLED = rLedOff;       // Green Status LED
uint8_t bLED = rLedOff;       // Blue Status LED
Adafruit_NeoPixel *rLedPixel;

uint16_t i=0;
uint32_t startMillis = 0;
uint32_t currentMillis = 0;
uint8_t printGrid = 0;

uint8_t DataPacketHeader[PacketLength] =      // Response Packet
{ 0x0A, 0xFA, 0x0C, 0x00, 0x02,               // Header
  0x00, 0x00, 0x00, 0x00,                     // ecgdata
  0x00, 0x00, 0x00, 0x00,                     // RR data
  0x00, 0x00, 0x00, 0x00,                     // HR data
  0x00, 0x0B };                               // Footer

int maxresult;
uint32_t maxinfo;                           // Max30003 INFO
uint16_t maxstatus;                         // Max30003 status register (built from 32bit STATUS)
uint32_t regdata;                           // Max30003 register data
int32_t sregdata;
uint8_t etag;                               // Max30003 etag data
int16_t ecgSampleCount = 0;
int32_t ecgSample[FIFO_MAX_SAMPLES];        // ecg sample is 18 bits
uint8_t etagSample[FIFO_MAX_SAMPLES];       // etag sample is 3 bits
int16_t R2RSample[FIFO_MAX_SAMPLES];
int16_t HRSample[FIFO_MAX_SAMPLES];
uint32_t  rtor = 0x1FFF;                    // Max30003 rtor  default value
float     hr =    80;                       // Max30003 hr    default value
int16_t   hrAvg = 0;                        // Max30003 hr    average value
int16_t   hrRolling;

volatile uint8_t SPI_TX_Buff[4];
volatile uint8_t SPI_RX_Buff[10];
volatile uint8_t *SPI_RX_Buff_Ptr;
uint8_t SPI_temp_Burst[100];

portMUX_TYPE ecgINTBMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE ecgINT2BMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t  ecgFIFOIrqCount = 0;
volatile uint8_t  ecgFIFOIrq = 0;
uint32_t rtor_data = 0;
volatile uint8_t  rtor_detected  = 0;
volatile uint8_t  rtor_count  = 0;
uint8_t INTBstatus = 0;

void IRAM_ATTR ecgINTBIRQ()  {            // ECG FIFO IRQ
  portENTER_CRITICAL_ISR(&ecgINTBMux);
  ecgFIFOIrqCount += 1;
  portEXIT_CRITICAL_ISR(&ecgINTBMux);
} 

void IRAM_ATTR ecgINT2BIRQ()  {            // R2R IRQ
  portENTER_CRITICAL_ISR(&ecgINT2BMux);
  rtor_detected += 1;
  portEXIT_CRITICAL_ISR(&ecgINT2BMux);
}

void BT_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    if(event == ESP_SPP_SRV_OPEN_EVT) {
        BTcallback = "BT Connection established";
        Serial.println("");
        for (int i = 0; i < 6; i++) {
          Serial.printf("%02X", param->srv_open.rem_bda[i]);
          if (i < 5) {
           Serial.print(":");
          }
        }
        Serial.println("");
        BTconnected = true;
    }
    else if(event == ESP_SPP_CLOSE_EVT) {
        BTcallback = "BT Connection closed";
        BTconnected = false;
        delay(100);
//        ESP.restart();
    }
    else if(event == ESP_SPP_DATA_IND_EVT) {
        BTcallback = "BT Data received";

    }
    else if(event == ESP_SPP_WRITE_EVT) {
        BTcallback = "BT Data sent";
    }
}

void setup()
{
  Serial.begin(115200);                   // USB Serial (RPC)

  SerialBT.register_callback(BT_callback);
  BTconnected = SerialBT.begin(BTname);   // Bluetooth Serial (MAX)

// Display RPC start banner
  Serial.printf("MAX30003 HeartSensor %d.%d.%d %02d/%02d/%02d\n", 
    VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, 
    VERSION_MONTH, VERSION_DAY, VERSION_SHORT_YEAR);

// Initialize the RPC server
  Serial.printf("Init RPC Server...\n");
  RPC_init();
  inputState = 0;

//  Serial.println("Initializing FCLK ..");
// Use LEDC(PWM) to set FCLK to 32768 Hz
  pinMode(FCLKPin, OUTPUT);
  ledcSetup(FCLKChannel, FCLKFreq, FCLKResolution);
  ledcAttachPin(FCLKPin, FCLKChannel);
  ledcWrite(FCLKChannel, FCLKDuty);

//  Serial.println("Initializing SPI ..");
  pinMode(MAX30003_CS_PIN, OUTPUT);     // Also SPI SS
  digitalWrite(MAX30003_CS_PIN, HIGH);  //disable device

// Initialise vspi with default pins : SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  SPI.begin();
  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  
  rLedPixel = new Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, FORMAT);
  rLedPixel->begin();
  rLedPixel->show();
  r2rLed(rLedBlue);

  startMillis = millis();

//  Serial.print("Initializing MAX30003 ..");
  MAX30003_begin();   // initialize MAX30003
}

void loop() {

  r2rLed(rLedUpdate);                           // Update LED state

  // Handle RPC
//  Serial.printf("Checking for RPC commands ...\n");
  inputState = getLine(request, sizeof(request));  // get a RPC string if one is available
  if (inputState == GETLINE_DONE) {                // if a string has been captured, process string
    Serial.printf(request);                             // Send request to debug port
    Serial.print(" : ");
    RPC_call(request, reply);                           // process the RPC string
    Serial.printf(reply);                               // Send reply to debug port
    putStr(reply);                                      // output the reply
  }

// Handle BT connect
  do {
    if (BTcallback != "") {
      if (BTcallback == "BT Data sent") {
        BTcallback = "";
      }
      else if (BTcallback == "BT Data received") {
        BTcallback = "";
      }
      else if (BTcallback == "BT Connection established") {
        Serial.println(BTcallback);
        BTcallback = "";
      }
      else if (BTcallback == "BT Connection closed") {
        Serial.println(BTcallback);
        BTcallback = "";
        delay(100);
        BTconnected = SerialBT.begin(BTname);   // Bluetooth Serial (MAX)
        BTconnected = true;
      }
    }
    delay(20);
  } while (!BTconnected); 

  currentMillis = millis();
  if ((currentMillis - startMillis) >= 1000) {  // Test for 1s
    startMillis = currentMillis;
    DataPacketHeader[11] += 1;                  // Set time in packet data
    if ((DataPacketHeader[11] & 0x01) == 0x01)  // Flash Blue LED
      bLED = BRIGHTNESS;
    else
      bLED = rLedOff;
  }

  maxresult = max30003_reg_read(STSREG, &max30003_status.all); // Read status register
  maxstatus = (((max30003_status.all >> 8) & 0x000000F0) | (max30003_status.all & 0x0000000F));

// Handle R2R
  if ((maxstatus & MAX_R2RIRQ) == MAX_R2RIRQ) {
    maxresult = max30003_reg_read(RTOR, &rtor_data);
    rtor = ((rtor_data >> 10) & 0x3FFF);
    hr =  (float)60 / ((float)rtor * ((float)FCLKtSample)/1000);
    if (hrAvg == 0)                           // Initialize first hrAvg
     hrAvg = (int16_t) hr;
    rtor_count += 1;
    gLED = BRIGHTNESS;                        // Turn on Green LED
    rtor_detected = 0;
  }

  if (rtor_count) {
    rtor_count +=1;
    if (rtor_count > R2R_LED_FLASH) {
      rtor_count = 0;
      gLED = rLedOff;                       // Turn off Green LED
    }
  } 

  if ((maxstatus & MAX_OVFIRQ) == MAX_OVFIRQ){          // Check if FIFO has overflowed                  
    max30003_reg_write(FIFO_RST, 0);                    // Reset FIFO
    rLED = BRIGHTNESS;                                  // overflow occured
  }    
  else if ((maxstatus & MAX_FIFOIRQ) == MAX_FIFOIRQ) {    
    ecgSampleCount = 0;                                                 // Reset sample counter 
    do {
        maxresult = max30003_reg_read(ECG_FIFO, &regdata);               // Read FIFO
        ecgSample[ecgSampleCount] = (regdata >> 6) & 0x03FFFF;          // 18 bit  ecg voltage data
//            ecgSample[ecgSampleCount] = (regdata >> 8) & 0x00FFFF;        // integer ecg voltage data (divide by 4)
        etagSample[ecgSampleCount] = ( regdata >> 3 ) & ETAG_BITS_MASK; // Isolate ETAG
        R2RSample[ecgSampleCount] = rtor;
        HRSample[ecgSampleCount] = (uint16_t)hr;
        ecgSampleCount += 1;                                        // Increment sample counter
        ecgSampleCount &= (FIFO_MAX_SAMPLES - 1);                   // Limit to FIFO_MAX_SAMPLES

    } while  ((etagSample[ecgSampleCount - 1] != FIFO_VALID_LAST) &&
              (etagSample[ecgSampleCount - 1] != FIFO_FAST_LAST));   // &&
//                  (etagSample[ecgSampleCount - 1] != FIFO_IS_EMPTY) &&
//                  (etagSample[ecgSampleCount - 1] != FIFO_OVF));

    if (etagSample[ecgSampleCount - 1] == FIFO_OVF){                // Check if FIFO has overflowed                  
      max30003_reg_write(FIFO_RST, 0);                              // Reset FIFO
      rLED = rLedOff;                                               // overflow occured
    }
    if ((ecgSampleCount >= 1) && (etagSample[ecgSampleCount - 1] != FIFO_IS_EMPTY)) { // First data already overflowed?
      for ( int idx = 0; idx < ecgSampleCount; idx++ ) {
        hrRolling += HRSample[idx];  // Calculate rolling average
      }
      hrAvg = (hrAvg + hrRolling) / (ecgSampleCount + 1);
      for ( int idx = 0; idx < ecgSampleCount; idx++ ) {   // Store valid results and print
        DataPacketHeader[5] = ecgSample[idx];               // ecg data
        DataPacketHeader[6] = ecgSample[idx] >> 8;
        DataPacketHeader[7] = ecgSample[idx] >> 16;
        DataPacketHeader[8] = (etagSample[idx] | (ecgSampleCount << 4));              // ecg data tag and sample count
        DataPacketHeader[9] = R2RSample[idx];               // R2R
        DataPacketHeader[10] = R2RSample[idx] >> 8;
        DataPacketHeader[13] = HRSample[idx];               // HR
        DataPacketHeader[14] = hrAvg;               // HRAvg

//        DataPacketHeader[14] = HRSample[idx] >> 8;  

#ifdef SERIAL_PLOTTER
        sregdata = ((DataPacketHeader[7] << 16) + (DataPacketHeader[6] << 8) + DataPacketHeader[5]); // ecg data only
        if (sregdata > 0x20000) sregdata |= 0xFFFC0000;
//        Serial.print((int32_t)sregdata, DEC);                  // ecg data
//        Serial.print(",");
        SerialBT.print((int32_t)sregdata, DEC);
        SerialBT.print(",");

//        Serial.print(DataPacketHeader[13]*100, DEC);      //  HR
//        Serial.print(",");
        SerialBT.print(DataPacketHeader[13]*100, DEC);    //  HR
        SerialBT.print(",");

//        Serial.print(DataPacketHeader[14]*100, DEC);      //  HRAvg
//        Serial.print(",");
        SerialBT.print(DataPacketHeader[14]*100, DEC);    //  HRAvg
        SerialBT.print(",");

        if ((DataPacketHeader[11] & 0x01) == 0x01) {  // seconds counter
//          Serial.print(-5000);
          SerialBT.print(-5000);
        }
        else {
//          Serial.print(15000);
          SerialBT.print(15000);
        }
//        Serial.print(',');
        SerialBT.print(',');

//        Serial.print(DataPacketHeader[9]*100, DEC);       //  R2R
//        Serial.print(',');
        SerialBT.print(DataPacketHeader[9]*100, DEC);     //  R2R
        SerialBT.print(',');

//        Serial.print(DataPacketHeader[8], DEC);       //  ecg etag
//          Serial.print(',');
        SerialBT.print(DataPacketHeader[8], DEC);     //  ecg etag
//          SerialBT.print(',');          
  
#endif
#ifdef SERIAL_MONITOR
        for(i=0; i<PacketLength; i++) {               // transmit the data packet
          PrintHex8(&DataPacketHeader[i], 1);
          Serial.print(" ");
        }
#endif
//        Serial.println("");
        SerialBT.println("");
      }
    }
  }
}

/*
int max30003_reg_write (MAX30003_REG_map_t Write_Address, uint32_t data) {
  SPI_TX_Buff[0] = ((Write_Address << 1 ) | WREG);
  SPI_TX_Buff[1] = (0x00FF0000 & data) >> 16;
  SPI_TX_Buff[2] = (0x0000FF00 & data) >> 8;
  SPI_TX_Buff[3] = (0x000000FF & data);

  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
  digitalWrite(MAX30003_CS_PIN, LOW);        // CS Low
  for ( i = 0; i < 4; i++) {                 // Send Command and Data
     SPI.transfer(SPI_TX_Buff[i]);
  }
  digitalWrite(MAX30003_CS_PIN, HIGH);       // CS High
  SPI.endTransaction();
  return 0;
}

int MAX30003_Read_Reg(MAX30003_REG_map_t Read_Address, uint32_t data) {
  uint32_t data = 0;
  SPI_TX_Buff[0] = ((Read_Address << 1 ) | RREG);
  SPI_TX_Buff[1] = 0x00;
  SPI_TX_Buff[2] = 0x00;
  SPI_TX_Buff[3] = 0x00;

  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
  digitalWrite(MAX30003_CS_PIN, LOW);            // CS Low
  // SPI.transfer(SPI_TX_Buff[0]);               // Send (command + register address)
  for ( i = 0; i < 4; i++) {                     // Read the three byte response
     SPI_RX_Buff[i] = SPI.transfer(SPI_TX_Buff[i]);
  }
  data =( (0x00FF0000 & (uint32_t) SPI_RX_Buff[1] << 16) | 
          (0x0000FF00 & (uint32_t) SPI_RX_Buff[2] << 8) | 
          (0x000000FF & (uint32_t) SPI_RX_Buff[3]) );
  digitalWrite(MAX30003_CS_PIN, HIGH);           // CS High
  SPI.endTransaction();
  return 0;
}
*/

void MAX30003_Read_Burst(int num_samples) {
  SPI_TX_Buff[0] = (ECG_FIFO_BURST << 1 ) | RREG;
  if (num_samples > FIFO_EFIT) num_samples = FIFO_EFIT;

  digitalWrite(MAX30003_CS_PIN, LOW);             // CS Low
  SPI.transfer(SPI_TX_Buff[0]);                   // Send (READ + ECG Burst)
  for ( i = 0; i < num_samples*3; ++i) {          // Read data
    SPI_temp_Burst[i] = SPI.transfer(0x00);
  }
  digitalWrite(MAX30003_CS_PIN, HIGH);            // CS High  
}

void MAX30003_begin() {
#ifdef SERIAL_MONITOR
  Serial.print("\nReset .. STATUS: ");
#endif
  max30003_reg_write((MAX30003_REG_map_t)SW_RST, 0x000000);
   delay(100);
  maxresult = max30003_reg_read(STSREG, &maxinfo);   // Read INFO cannot be the first register operation
#ifdef SERIAL_MONITOR
  Serial.println(maxinfo, HEX);                   //  so do a STATUS read first
  Serial.print("Reset .. INFO  : ");
#endif
  maxresult = max30003_reg_read(INFO, &maxinfo);              // Read INFO cannot be the first register operation
#ifdef SERIAL_MONITOR
  Serial.println(maxinfo, HEX);
  if ((maxinfo & 0xF0F000) == 0x503000) {
    Serial.print(" found MAX30003, REV_ID = ");
    Serial.println(((maxinfo >> 16) & 0x00000F), HEX);
    Serial.print("          Serial Number = ");
    Serial.println((maxinfo & 0x000FFF), HEX);
  }
  else 
    Serial.println(" MAX30003 not found)");
#endif

//    Serial.print("CNFG_GEN ..");
/* General config register setting
    MAX30003::GeneralConfiguration_u CNFG_GEN_r;
    CNFG_GEN_r.bits.en_ecg = 1;     // Enable ECG channel
    CNFG_GEN_r.bits.rbiasn = 1;     // Enable resistive bias on negative input
    CNFG_GEN_r.bits.rbiasp = 1;     // Enable resistive bias on positive input
    CNFG_GEN_r.bits.en_rbias = 1;   // Enable resistive bias
    CNFG_GEN_r.bits.imag = 2;       // Current magnitude = 10nA
    CNFG_GEN_r.bits.en_dcloff = 0;  // Disable DC lead-off detection 
*/
#ifdef CALIBRATION
  max30003_reg_write(CNFG_GEN, 0x080200);   // No ULP, 32768, EN_ECG, No DCLOFF, 10nA, Resistive Bias (100M)
#endif
#ifdef ECG_READ
  max30003_reg_write(CNFG_GEN, 0x080217);   // No ULP, 32768, EN_ECG, No DCLOFF, 10nA, Resistive Bias (100M)
#endif
  delay(100);

//    Serial.print("MNGR_INT ..");
/* Manage interrupts register setting
    MAX30003::ManageInterrupts_u MNG_INT_r;
    MNG_INT_r.bits.efit = 0b00011;          // Assert EINT w/ 4 unread samples
    MNG_INT_r.bits.clr_rrint = 0b01;        // Clear R-to-R on RTOR reg. read back
*/
  max30003_reg_write(MNGR_INT, 0x180014);   // FIFO_EFIT = 4
   delay(100);

//    Serial.print("MNGR_DYN ..");
/* Dyanmic modes config
    MAX30003::ManageDynamicModes_u MNG_DYN_r;
    MNG_DYN_r.bits.fast = 0;                // Fast recovery mode disabled
*/
  max30003_reg_write(MNGR_DYN, 0x000000);   //
   delay(100);

//    Serial.print("CNFG_CAL ..");
#ifdef CALIBRATION
  max30003_reg_write(CNFG_CAL, 0x704800);   //
#endif
#ifdef ECG_READ
  max30003_reg_write(CNFG_CAL, 0x004800);   //
#endif
   delay(100);

//    Serial.print("CNFG_EMUX ..");
/* MUX Config
    MAX30003::MuxConfiguration_u CNFG_MUX_r;
    CNFG_MUX_r.bits.openn = 0;          // Connect ECGN to AFE channel
    CNFG_MUX_r.bits.openp = 0;          // Connect ECGP to AFE channel
*/
#ifdef CALIBRATION
  max30003_reg_write(CNFG_EMUX,0x3B0000);   //
#endif
#ifdef ECG_READ
  max30003_reg_write(CNFG_EMUX,0x000000);   //
#endif  
   delay(100);

//    Serial.print("CNFG_ECG ..");
/* ECG Config register setting
    MAX30003::ECGConfiguration_u CNFG_ECG_r;
    CNFG_ECG_r.bits.dlpf = 1;       // Digital LPF cutoff = 40Hz
    CNFG_ECG_r.bits.dhpf = 1;       // Digital HPF cutoff = 0.5Hz
    CNFG_ECG_r.bits.gain = 3;       // ECG gain = 160V/V
    CNFG_ECG_r.bits.rate = 2;       // Sample rate = 128 sps
    */
#ifdef CALIBRATION
  max30003_reg_write(CNFG_ECG, 0x805000);  // 128sps (d23 - d22 : 01 for 250sps , 00:500 sps); 20V/V
#endif
#ifdef ECG_READ
  max30003_reg_write(CNFG_ECG, 0x825000);  // 128sps (d23 - d22 : 01 for 250sps , 00:500 sps); 160V/V
#endif  
   delay(100);

//    Serial.print("CNFG_RTOR1 ..");
/* R-to-R configuration
    MAX30003::RtoR1Configuration_u CNFG_RTOR_r;
    CNFG_RTOR_r.bits.wndw = 0b0011;         // WNDW = 96ms
    CNFG_RTOR_r.bits.rgain = 0b1111;        // Auto-scale gain
    CNFG_RTOR_r.bits.pavg = 0b11;           // 16-average
    CNFG_RTOR_r.bits.ptsf = 0b0011;         // PTSF = 4/16
    CNFG_RTOR_r.bits.en_rtor = 1;           // Enable R-to-R detection
*/
  max30003_reg_write(CNFG_RTOR1,0x3fA300);
   delay(100);

//    Serial.print("CNFG_RTOR2 ..");
  max30003_reg_write(CNFG_RTOR2,0x202400);
   delay(100);
//    Serial.print("SYNCH ..");
  max30003_reg_write(SYNCH, 0x000000);
    delay(100);

//    Serial.print("interrupts ..");
/* Enable interrupts register setting
    MAX30003::EnableInterrupts_u EN_INT_r;
    EN_INT_r.bits.en_eint = 1;              // Enable EINT interrupt
    EN_INT_r.bits.en_rrint = 1;             // Enable R-to-R interrupt
    EN_INT_r.bits.intb_type = 3;            // Open-drain NMOS with internal pullup
*/
  max30003_reg_write(EN_INT,0x800003);    // Enable FIFOIRQ,  PUR
  max30003_reg_write(EN_INT2,0x000403);   // Enable R2RIRQ,   PUR
    delay(100);

  pinMode(MAX30003_INTB_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt( MAX30003_INTB_PIN ), ecgINTBIRQ, FALLING);   // Handle INTB
  pinMode(MAX30003_INT2B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt( MAX30003_INT2B_PIN ), ecgINT2BIRQ, FALLING);   // Handle INTB
 
//    Serial.println(" all done.");
}

void PrintHex8(uint8_t *data, uint8_t length) { // prints 8-bit data in hex
 char tmp[length*2+1];
 byte first ;
 int j=0;
 for (uint8_t i=0; i<length; i++)
 {
   first = (data[i] >> 4) | 48;
   if (first > 57) tmp[j] = first + (byte)39;
   else tmp[j] = first ;
   j++;

   first = (data[i] & 0x0F) | 48;
   if (first > 57) tmp[j] = first + (byte)39;
   else tmp[j] = first;
   j++;
 }
 tmp[length*2] = 0;
 Serial.print(tmp);
}

void setLedOn(uint8_t color) {
  setLed(color, BRIGHTNESS);
}

void setLedOff(uint8_t color) {
  setLed(color, rLedOff);
}

void setLedBlink(uint8_t color) {
  setLed(color, BRIGHTNESS);
}

void setLedPattern(uint8_t color) {
  setLed(color, rLedOff);
}

void setLed(uint8_t color, uint8_t intensity){
  if (color == rLedRed)    rLED = intensity;
  if (color == rLedGreen)  gLED = intensity;
  if (color == rLedBlue)   bLED = intensity; 
  rLedPixel->setPixelColor(0, rLED, gLED, bLED);       // PixelNumber, R, G, B
  rLedPixel->show();
}

void r2rLed(uint8_t color){
  if (color & rLedRed)    rLED = BRIGHTNESS;
  if (color & rLedGreen)  gLED = BRIGHTNESS;
  if (color & rLedBlue)   bLED = BRIGHTNESS; 
  rLedPixel->setPixelColor(0, rLED, gLED, bLED);       // PixelNumber, R, G, B
  rLedPixel->show();
}

#define SerialHSP   SerialBT

/// a running index that keeps track of where an incoming string has been
/// buffered to
static int lineBuffer_index = 0;
/// a flag that keeps track of the state of accumulating a string
static int getLine_State = GETLINE_WAITING;
 
/**
* @brief Place incoming USB characters into a fifo
* @param data_IN buffer of characters
* @param len length of data
*/
int fifoIncomingChars(uint8_t data_IN[], unsigned int len) {
  int i;
  for (i = 0; i < len; i++) {
    fifo_put8(GetUSBIncomingFifo(), data_IN[i]);
  }
  return 0;
}
 
/**
* @brief Check the USB incoming fifo to see if there is data to be read
* @return 1 if there is data to be read, 0 if data is not available
*/
int isReadReady(void) {
  if (fifo_empty(GetUSBIncomingFifo()) == 0)
    return 1;
  return 0;
}
 
/**
* @brief Clear the incoming USB read fifo
*/
void clearOutReadFifo(void) { fifo_clear(GetUSBIncomingFifo()); }
 
/**
* @brief Block until a character can be read from the USB
* @return the character read
*/
char getch(void) {
  uint8_t ch;
  // block until char is ready
  while (isReadReady() == 0) {
  }
  // read a char from buffer
  fifo_get8(GetUSBIncomingFifo(), &ch);
  return ch;
}
 
/**
* @brief Place incoming USB characters into a fifo
* @param lineBuffer buffer to place the incoming characters
* @param bufferLength length of buffer
* @return GETLINE_WAITING if still waiting for a CRLF, GETLINE_DONE
*/
int getLine(char *lineBuffer, int bufferLength) {
  uint8_t ch;
 
  if (getLine_State == GETLINE_DONE) {
    getLine_State = GETLINE_WAITING;
  }
  if (SerialHSP.available() != 0) {
    ch = SerialHSP.read();
    Serial.write(ch);
    if (ch != 0x0A && ch != 0x0D) {
      lineBuffer[lineBuffer_index++] = ch;
    }
    if (ch == 0x0D) {
      lineBuffer[lineBuffer_index++] = 0;
      lineBuffer_index = 0;
      getLine_State = GETLINE_DONE;
    }
    if (lineBuffer_index > bufferLength) {
      lineBuffer[bufferLength - 1] = 0;
      getLine_State = GETLINE_DONE;
    }
  }
  return getLine_State;
}
 
/**
* @brief Block until a fixed number of characters has been accumulated from the
* incoming USB
* @param lineBuffer buffer to place the incoming characters
* @param maxLength length of buffer
*/
void getStringFixedLength(uint8_t *lineBuffer, int maxLength) {
  uint8_t ch;
  int index = 0;
  // block until maxLength is captured
  while (1) {
    ch = getch();
    lineBuffer[index++] = ch;
    if (index == maxLength)
      return;
  }
}
 
/**
* @brief Output a string out the USB serial port
* @param str output this str the USB channel
*/
int putStr(const char *str) {
  SerialHSP.printf("%s", str); // fflush(stdout);
  // uint8_t *ptr;
  // uint8_t buffer[256];
  // int index = 0;
  /*    int length;
          ptr = (uint8_t *)str;
          length = strlen(str);
 
          Peripherals::usbSerial()->writeBlock(ptr,length); */
  return 0;
}
 
/**
* @brief Outut an array of bytes out the USB serial port
* @param data buffer to output
* @param length length of buffer
*/
int putBytes(uint8_t *data, uint32_t length) {
  int sendThis = 64;
  int sent = 0;
  int thisLeft;
  uint8_t *ptr = data;
  if (length < 64)
    sendThis = length;
  do {
    SerialHSP.write(ptr, sendThis);
    sent += sendThis;
    ptr += sendThis;
    thisLeft = length - sent;
    sendThis = 64;
    if (thisLeft < 64)
      sendThis = thisLeft;
  } while (sent != length);
  return 0;
}
 
/**
* @brief Outut 256 byte blocks out the USB serial using writeBlock bulk transfers
* @param data buffer of blocks to output
* @param length length of 256-byte blocks
*/
int putBytes256Block(uint8_t *data, int numberBlocks) {
  int i;
  uint8_t *ptr;
  ptr = data;
  const int BLOCK_SIZE = 32;
  const int FLASH_PAGE_SIZE = 256;
  for (i = 0; i < numberBlocks * (FLASH_PAGE_SIZE / BLOCK_SIZE); i++) {
    SerialHSP.write(ptr, BLOCK_SIZE);
    ptr += BLOCK_SIZE;
  }
  return 0;
}

void max30003_ServiceStreaming() {
//  char ch;
  uint32_t val;
  fifo_clear(GetStreamOutFifo());
  SetStreaming(true);
  clearOutReadFifo();
  while (IsStreaming() == true) {
    if (fifo_empty(GetStreamOutFifo()) == 0) {
      fifo_get32(GetStreamOutFifo(), &val);
      SerialHSP.printf("%02X ", val);
    }
    if (SerialHSP.available()) {     // clear any serial incoming fifo
//      ch = 
      SerialHSP.read();
      MAX30003_Helper_Stop();
      SetStreaming(false);
      fifo_clear(GetUSBIncomingFifo());
      fifo_clear(GetStreamOutFifo());
    }
  }
}
