#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>

#include "..\include\secrets.h"

const char* ssid = SSID;                // Github immune secrets file
const char* password = PASS;

// NeoPixel LED
#define PIXEL_TYPE      WS2812B

const uint16_t PIXEL_PIN        = 14;   // Digital14 attached to NeoPixel data
const uint16_t PIXEL_COUNT      = 1;
const uint16_t BRIGHTNESS       = 128;
const uint16_t PIXEL_FORMAT     = NEO_GRB + NEO_KHZ800;

const uint8_t  cLedOff    = 0;
const uint8_t  cLedRed    = 1;
const uint8_t  cLedGreen  = 2;
const uint8_t  cLedBlue   = 4;

#define LED_ON    BRIGHTNESS
#define LED_OFF   0
#define LED_FLASH 16

uint8_t cLED = cLedOff;
uint8_t bLED = BRIGHTNESS;
uint8_t r, g, b;
Adafruit_NeoPixel *LedPixel;

void Led(uint8_t color, uint8_t brightness){
  r = 0; b = 0; g = 0;
  if (color & cLedRed)    r = brightness;
  if (color & cLedGreen)  g = brightness;
  if (color & cLedBlue)   b = brightness;
  LedPixel->setPixelColor(0, r, g, b);
  LedPixel->show();
}

uint32_t startMillis;
uint32_t currentMillis;

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  cLED |= cLedBlue;
  LedPixel = new Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_FORMAT);
  LedPixel->begin();
  LedPixel->show();
  startMillis = millis();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
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

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  ArduinoOTA.handle();

  currentMillis = millis();
  if ((currentMillis - startMillis) >= 1000) {  //test for 1s
    startMillis = currentMillis;
    ((cLED += 1) &= 0x07);                      // Change colour, rollover
    ((bLED += 1) |= BRIGHTNESS);                // Increase brightness, rollover
    Led(cLED, bLED);                            // Update LED state
  }
}
