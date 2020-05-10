#include <arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "..\include\MAX30003.h"
#include "..\include\secrets.h"

const char* ssid = SSID;
const char* password = PASS;

//variables for blinking an LED with Millis
const int led = 2;                              // ESP32 Pin to which onboard LED is connected
unsigned long previousMillis = 0;               // will store last time LED was updated
const long interval = 1000;                     // interval at which to blink (milliseconds)
int ledState = LOW;                             // ledState used to set the LED

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#undef  CALIBRATION
#define ECG_READ

#undef  SERIAL_MONITOR
#define SERIAL_PLOTTER

BluetoothSerial SerialBT;
String BTname = "HeartSensor";
// char *BTpin = "1234";
char BTSendBuffer[32];
bool BTconnected;
int BTCount = 0;
String BTcallback = "";
String BTreadData = "";

uint8_t rLED = rLedOff;       // R2R Status LED
Adafruit_NeoPixel *rLedPixel;

uint16_t i=0;
uint32_t startMillis = 0;
uint32_t currentMillis = 0;
uint8_t printGrid = 0;

uint8_t DataPacketHeader[20] =                // Response Packet
{ 0x0A, 0xFA, 0x0C, 0x00, 0x02,               // Header
  0x00, 0x00, 0x00, 0x00,                     // ecgdata
  0x00, 0x00, 0x00, 0x00,                     // RR data
  0x00, 0x00, 0x00, 0x00,                     // HR data
  0x00, 0x0B };                               // Footer

int32_t maxinfo;                            // Max30003 INFO
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
float     hr = 80;                          // Max30003 hr    default value

volatile uint8_t SPI_TX_Buff[4];
volatile uint8_t SPI_RX_Buff[10];
volatile uint8_t *SPI_RX_Buff_Ptr;
uint8_t SPI_temp_Burst[100];

portMUX_TYPE ecgINTBMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE ecgINT2BMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t  ecgFIFOIrqCount = 0;
volatile uint8_t  ecgFIFOIrq = 0;
volatile uint32_t rtor_data = 0;
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

void setupMAX30003(void){
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
  
  rLED |= rLedBlue;
  rLedPixel = new Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, FORMAT);
  rLedPixel->begin();
  startMillis = millis();

//  Serial.print("Initializing MAX30003 ..");
  MAX30003_begin();   // initialize MAX30003
}

void setupOTA(void) {
  Serial.println("Checking OTA .. ");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Restarting...");
    delay(5000);
    ESP.restart();
  }

   ArduinoOTA.setPort(3232);                // Port defaults to 3232
   ArduinoOTA.setHostname("HeartSensor");   // Hostname defaults to esp3232-[MAC]
   ArduinoOTA.setPassword("max30003");      // No authentication by default

  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3  // Password can be set with it's md5 value as well
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();               // For loop() ?

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

}

void setupBT(){
  SerialBT.register_callback(BT_callback);
  BTconnected = SerialBT.begin(BTname);   // Bluetooth Serial (MAX)
}

void setup() {
  Serial.begin(115200);                   // USB Serial

  setupOTA();
  setupBT();
  setupMAX30003();

}

void loop() {
  ArduinoOTA.handle();  
    
  currentMillis = millis();
  if ((currentMillis - startMillis) >= 1000) {  //test for 1s
    startMillis = currentMillis;
    DataPacketHeader[11] += 1;  
  }
  r2rLed(rLED);                             // Update LED state

// Handle BT connect/disconnect
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

/*
    do {
      MAX30003_Read_Reg (STATUS);             // Read status register
      maxstatus = ((SPI_RX_Buff[1] & 0xF0) | (SPI_RX_Buff[2] & 0x0F));
      portENTER_CRITICAL_ISR(&ecgINTBMux);    // Refresh loop copy of current IRQ Count
      ecgFIFOIrq = ecgFIFOIrqCount;           // (Cannot do this in loop test)
      portEXIT_CRITICAL_ISR(&ecgINTBMux);
    } while (((maxstatus & FIFOIRQ) != FIFOIRQ) && (ecgFIFOIrq == 0));       // Wait for FIFO IRQ
    
    portENTER_CRITICAL_ISR(&ecgINTBMux);    // Refresh loop copy of current IRQ Count
    ecgFIFOIrqCount = 0;                    // Clear IRQ Flag
    portEXIT_CRITICAL_ISR(&ecgINTBMux);
*/

    MAX30003_Read_Reg(STATUS);              // Read status register
    maxstatus = ((SPI_RX_Buff[1] & 0xF0) | (SPI_RX_Buff[2] & 0x0F));
/*
//#ifdef SERIAL_MONITOR
    if (((maxstatus & R2RIRQ) == R2RIRQ) || ((maxstatus & FIFOIRQ) == FIFOIRQ) || ((maxstatus & OVFIRQ) == OVFIRQ)) {
     Serial.print("status: "); Serial.print(maxstatus, HEX);
     Serial.print(", "); Serial.println(ecgFIFOIrq);
    }
//#endif
*/

    if ((maxstatus & R2RIRQ) == R2RIRQ) {
      rtor_data = MAX30003_Read_Reg(RTOR);
      rtor = ((rtor_data >> 10) & 0x3FFF);
      hr =  (float)60 / ((float)rtor * ((float)FCLKPeriod)/1000);
      rtor_count += 1;
      rLED |= rLedGreen;        // Turn on Green LED
      rtor_detected = 0;
    }

    if (rtor_count) {
      rtor_count +=1;
      if (rtor_count > R2R_LED_FLASH) {
        rtor_count = 0;
        rLED &= ~rLedGreen;      // Turn on Green LED
      }
    } 

    if ((maxstatus & OVFIRQ) == OVFIRQ){                  // Check if FIFO has overflowed                  
      MAX30003_Write_Reg(FIFO_RST, 0);                    // Reset FIFO
      rLED |= rLedRed;                                    // overflow occured
    }    
    else if ((maxstatus & FIFOIRQ) == FIFOIRQ) {    
        ecgSampleCount = 0;                                             // Reset sample counter 
        do {
            regdata = MAX30003_Read_Reg(ECG_FIFO);                      // Read FIFO
            ecgSample[ecgSampleCount] = (regdata >> 6) & 0x03FFFF;          // Isolate voltage data
//            ecgSample[ecgSampleCount] = (regdata >> 8) & 0x00FFFF;          // Isolate voltage data
            etagSample[ecgSampleCount] = ( regdata >> 3 ) & ETAG_BITS_MASK; // Isolate ETAG
            R2RSample[ecgSampleCount] = rtor; 
            HRSample[ecgSampleCount] = (uint16_t)hr;
            ecgSampleCount += 1;                                        // Increment sample counter
            ecgSampleCount &= (FIFO_MAX_SAMPLES - 1);                   // Limit to FIFO_MAX_SAMPLES

        } while  ((etagSample[ecgSampleCount - 1] != FIFO_VALID_LAST) &&
                  (etagSample[ecgSampleCount - 1] != FIFO_FAST_LAST));   // &&
//                  (etagSample[ecgSampleCount - 1] != FIFO_EMPTY) &&
//                  (etagSample[ecgSampleCount - 1] != FIFO_OVF));

        if (etagSample[ecgSampleCount - 1] == FIFO_OVF){                // Check if FIFO has overflowed                  
          MAX30003_Write_Reg(FIFO_RST, 0);                              // Reset FIFO
          rLED |= rLedRed;                                               // overflow occured
        }
        if ((ecgSampleCount >= 1) && (etagSample[ecgSampleCount - 1] != FIFO_EMPTY)) { // First data already overflowed?
          for ( int idx = 0; idx < ecgSampleCount; idx++ ) {   // Store valid results and print
            DataPacketHeader[5] = ecgSample[idx];              // ecgdata
            DataPacketHeader[6] = ecgSample[idx] >> 8;
            DataPacketHeader[7] = ecgSample[idx] >> 16;
            DataPacketHeader[8] = etagSample[idx];
            DataPacketHeader[9] = R2RSample[idx];              // R2R
            DataPacketHeader[10] = R2RSample[idx] >> 8;
            DataPacketHeader[13] = HRSample[idx];              // HR
            DataPacketHeader[14] = HRSample[idx] >> 8;  
          }

#ifdef SERIAL_PLOTTER
          sregdata = ((DataPacketHeader[7] << 16) + (DataPacketHeader[6] << 8) + DataPacketHeader[5]); // ecg data only
          if (sregdata > 0x20000) sregdata = -(0x40000 - sregdata);
          Serial.print((int32_t)sregdata, DEC);
          Serial.print(",");
          
          sprintf(BTSendBuffer, "%d", (int32_t)sregdata);
          for (int i = 0; (BTSendBuffer[i] != 0); i++) { 
            SerialBT.write(BTSendBuffer[i]);      
          }
          SerialBT.write(',');
          
          Serial.print(-DataPacketHeader[9]*100, DEC);   //  R2R
          Serial.print(',');

          sprintf(BTSendBuffer, "%d", (-DataPacketHeader[9]*100));
          for (int i = 0; (BTSendBuffer[i] != 0); i++) { SerialBT.write(BTSendBuffer[i]); }
          SerialBT.write(',');
          
          Serial.print(DataPacketHeader[13]*100, DEC);   //  HR
          Serial.print(",");

          sprintf(BTSendBuffer, "%d", (DataPacketHeader[13]*100));
          for (int i = 0; (BTSendBuffer[i] != 0); i++) { SerialBT.write(BTSendBuffer[i]); }
          SerialBT.write(',');

          if ((DataPacketHeader[11] & 0x01) == 0x01) {
            Serial.print(-16000);
            sprintf(BTSendBuffer, "%d", (-16000));
            for (int i = 0; (BTSendBuffer[i] != 0); i++) { SerialBT.write(BTSendBuffer[i]); }
          }
          else {
            Serial.print(16000);
            sprintf(BTSendBuffer, "%d", (-16000));
            for (int i = 0; (BTSendBuffer[i] != 0); i++) { SerialBT.write(BTSendBuffer[i]); }
          }

          Serial.print(",");
          SerialBT.write(',');

//          }
      
#endif
#ifdef SERIAL_MONITOR
          for(i=5; i<15; i++) {                   // transmit the data packet
            PrintHex8(&DataPacketHeader[i], 1);
            Serial.print(" ");
          }
#endif
          Serial.println(";");
          SerialBT.write(';');
          SerialBT.write(0x0d);
          SerialBT.write(0x0a);
        }
    }
}

void max30003_sw_reset(void)
{
  MAX30003_Write_Reg(SW_RST, 0x000000);     
  delay(100);
}

void max30003_synch(void)
{
  MAX30003_Write_Reg(SYNCH, 0x000000);
}

void MAX30003_Write_Reg (uint8_t Write_Address, uint32_t data) {
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
}

uint32_t MAX30003_Read_Reg(uint8_t Read_Address) {
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
  digitalWrite(MAX30003_CS_PIN, HIGH);           // CS High
  SPI.endTransaction();

  data = ((0x00FF0000 & (uint32_t) SPI_RX_Buff[1] << 16) | 
          (0x0000FF00 & (uint32_t) SPI_RX_Buff[2] << 8) | 
          (0x000000FF & (uint32_t) SPI_RX_Buff[3]));
  return (data);
}

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
  MAX30003_Write_Reg(SW_RST, 0x000000);
   delay(100);
  maxinfo = MAX30003_Read_Reg(STATUS);            // Read INFO cannot be the first register operation
#ifdef SERIAL_MONITOR
  Serial.println(maxinfo, HEX);                   //  so do a STATUS read first
  Serial.print("Reset .. INFO  : ");
#endif
  maxinfo = MAX30003_Read_Reg(INFO);              // Read INFO cannot be the first register operation
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
  MAX30003_Write_Reg(CNFG_GEN, 0x080200);   // No ULP, 32768, EN_ECG, No DCLOFF, 10nA, Resistive Bias (100M)
#endif
#ifdef ECG_READ
  MAX30003_Write_Reg(CNFG_GEN, 0x080217);   // No ULP, 32768, EN_ECG, No DCLOFF, 10nA, Resistive Bias (100M)
#endif
  delay(100);
//    Serial.print("MNGR_INT ..");
    //Manage interrupts register setting
    /*
    MAX30003::ManageInterrupts_u MNG_INT_r;
    MNG_INT_r.bits.efit = 0b00011;          // Assert EINT w/ 4 unread samples
    MNG_INT_r.bits.clr_rrint = 0b01;        // Clear R-to-R on RTOR reg. read back
    */
  MAX30003_Write_Reg(MNGR_INT, 0x180014);   // FIFO_EFIT = 4
   delay(100);
//    Serial.print("MNGR_DYN ..");
/*
    //Dyanmic modes config
    MAX30003::ManageDynamicModes_u MNG_DYN_r;
    MNG_DYN_r.bits.fast = 0;                // Fast recovery mode disabled
    */
  MAX30003_Write_Reg(MNGR_DYN, 0x000000);   //
   delay(100);
//    Serial.print("CNFG_CAL ..");
#ifdef CALIBRATION
  MAX30003_Write_Reg(CNFG_CAL, 0x704800);   //
#endif
#ifdef ECG_READ
  MAX30003_Write_Reg(CNFG_CAL, 0x004800);   //
#endif
   delay(100);
//    Serial.print("CNFG_EMUX ..");
/*
    // MUX Config
    MAX30003::MuxConfiguration_u CNFG_MUX_r;
    CNFG_MUX_r.bits.openn = 0;          // Connect ECGN to AFE channel
    CNFG_MUX_r.bits.openp = 0;          // Connect ECGP to AFE channel
    */
#ifdef CALIBRATION
  MAX30003_Write_Reg(CNFG_EMUX,0x3B0000);   //
#endif
#ifdef ECG_READ
  MAX30003_Write_Reg(CNFG_EMUX,0x000000);   //
#endif  
   delay(100);
//    Serial.print("CNFG_ECG ..");
    // ECG Config register setting
    /*
    MAX30003::ECGConfiguration_u CNFG_ECG_r;
    CNFG_ECG_r.bits.dlpf = 1;       // Digital LPF cutoff = 40Hz
    CNFG_ECG_r.bits.dhpf = 1;       // Digital HPF cutoff = 0.5Hz
    CNFG_ECG_r.bits.gain = 3;       // ECG gain = 160V/V
    CNFG_ECG_r.bits.rate = 2;       // Sample rate = 128 sps
    */
#ifdef CALIBRATION
  MAX30003_Write_Reg(CNFG_ECG, 0x805000);  // 128sps (d23 - d22 : 01 for 250sps , 00:500 sps); 20V/V
#endif
#ifdef ECG_READ
  MAX30003_Write_Reg(CNFG_ECG, 0x835000);  // 128sps (d23 - d22 : 01 for 250sps , 00:500 sps); 160V/V
#endif  

   delay(100);
//    Serial.print("CNFG_RTOR1 ..");
    //R-to-R configuration
    /*
    MAX30003::RtoR1Configuration_u CNFG_RTOR_r;
    CNFG_RTOR_r.bits.wndw = 0b0011;         // WNDW = 96ms
    CNFG_RTOR_r.bits.rgain = 0b1111;        // Auto-scale gain
    CNFG_RTOR_r.bits.pavg = 0b11;           // 16-average
    CNFG_RTOR_r.bits.ptsf = 0b0011;         // PTSF = 4/16
    CNFG_RTOR_r.bits.en_rtor = 1;           // Enable R-to-R detection
    */
  MAX30003_Write_Reg(CNFG_RTOR1,0x3fB300);
   delay(100);
//    Serial.print("CNFG_RTOR2 ..");
  MAX30003_Write_Reg(CNFG_RTOR2,0x202400);
   delay(100);
//    Serial.print("SYNCH ..");
  MAX30003_Write_Reg(SYNCH, 0x000000);
    delay(100);
//    Serial.print("interrupts ..");

/*
        //Enable interrupts register setting
    MAX30003::EnableInterrupts_u EN_INT_r;
    EN_INT_r.bits.en_eint = 1;              // Enable EINT interrupt
    EN_INT_r.bits.en_rrint = 1;             // Enable R-to-R interrupt
    EN_INT_r.bits.intb_type = 3;            // Open-drain NMOS with internal pullup
*/
  MAX30003_Write_Reg(EN_INT,0x800003);    // Enable FIFOIRQ,  PUR
  MAX30003_Write_Reg(EN_INT2,0x000403);   // Enable R2RIRQ,   PUR
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

void r2rLed(uint8_t color){
  uint8_t r, g, b = 0;
  if (color & rLedRed)    r = BRIGHTNESS;
  if (color & rLedGreen)  g = BRIGHTNESS;
  if (color & rLedBlue)   b = BRIGHTNESS;
  rLedPixel->setPixelColor(0, r, g, b);
  rLedPixel->show();
}
