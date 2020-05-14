#include <arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>

#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/apb_ctrl_reg.h"
#include "rom/rtc.h"

#include "../include/secrets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "esp_task_wdt.h"
#include "esp32-hal.h"
#include "esp_attr.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_partition.h"

#include "esp_wifi.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"

#include "esp_bt.h"
// bluedroid
#include "esp_bt_main.h"            // implements initialization and enabling of the Bluedroid stack
#include "esp_bt_defs.h"
#include "esp_gap_bt_api.h"
#include "esp_gap_ble_api.h"        // implements GAP configuration, such as advertising and connection parameters
#include "esp_gatts_api.h"          // implements GATT configuration, such as creating services and characteristics
#include "esp_gatt_defs.h"

#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/sdmmc_host.h"

#include "esp_log.h"
#include "MaxBle.h"
#include "max30003.h"
#include "tcp_server.h"
#include "ble.h"
#include "adc.h"

#define CONFIG_BLE_MODE_ENABLE
#undef  CONFIG_WIFIMODE_ENABLE

#define TAG1 "HeartSensor:"
#define delay_ms(ms) vTaskDelay((ms) / portTICK_RATE_MS)
#define BUF_SIZE  1000

extern xSemaphoreHandle print_mux;
char uart_data[50];
const int uart_num = UART_NUM_1;
uint8_t* db;

unsigned int global_heartRate ;
BluetoothSerial SerialBT;

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;
extern QueueHandle_t xQueue_tcp;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
  switch (event->event_id) {
  case SYSTEM_EVENT_STA_START:
    esp_wifi_connect();
    break;
  case SYSTEM_EVENT_STA_GOT_IP:
    xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    esp_wifi_connect();
    xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    break;
  default:
    break;
  }
  return ESP_OK;
}

void wifi_init(void)
{
  tcpip_adapter_init();
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

  wifi_config_t wifi_config {
    .sta = {SSIDString, PASSString},
  };

/*
        .sta = {
            .ssid = SSID,
            .password = PASS
        }
  };
*/

  ESP_LOGI(TAG1, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
}

void ecgStart(void) {

// Initialize NVS.
  esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

  xQueue_tcp = xQueueCreate(20, sizeof( struct Tcp_Message *));
  if( xQueue_tcp==NULL ){
    ESP_LOGI(TAG1, "Failed to create Queue..!");
  }

// Initialize and start the ECG chip
  Serial.println("Max30003 SPI  ..");
  max30003_initchip(PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_SCK, PIN_SPI_CS);
  Serial.println("Max30003 chip ..");
  start_max30003();           // MAX3003 initialization
  Serial.println("adc           ..");
  adc_start();                // Battery monitor task create and start
  
  vTaskDelay(2000/ portTICK_PERIOD_MS);   //give sometime for max to settle

#ifdef CONFIG_BLE_MODE_ENABLE
  ble_Init();
#endif
  
#ifdef CONFIG_WIFIMODE_ENABLE             //configure the ssid/password in secrets.h

  Serial.println("wifi          ..");
  wifi_init();
    /* Wait for WiFI to show as connected */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,false, true, portMAX_DELAY);
    
  /*only hr and rr is sending through tcp, to plot ECG, you need to configure the max30003 to read ecg data */
  vTaskDelay(500/ portTICK_PERIOD_MS);
  Serial.println("tcp           ..");

  tcp_start();
#endif
}

void setup() {
  
  Serial.begin(115200); //start serial for debug
  Serial.println(__func__);
  Serial.println("-------------");
  Serial.println("API  Started!");
  Serial.println("-------------");
  Serial.println("Starting ecg!");
  Serial.println("-------------");
  ecgStart();
};

void loop() {
};
