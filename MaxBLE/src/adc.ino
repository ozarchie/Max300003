#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/adc.h"

#include <esp_log.h>
#include <esp_adc_cal.h>

#include "adc.h"
#include "ble.h"

#define TAG4 "HSBattery:"
volatile int adc_val = 0;
uint32_t battery = 0;
uint32_t samples = 10;
float bat;
static bool startup_flag = true;
static int bat_prev = 100;
QueueHandle_t xQueue_battery;
int bt_rem = 0;
static uint8_t bat_percent = 100;
static esp_adc_cal_characteristics_t *adc_chars;
static const adc1_channel_t BatteryV = ADC1_CHANNEL_0;        // Battery monitor attached to VP (GPIO36, ADC1_Ch0)
                                                            //  via 10k/10k divider (wastes 4/20 mA)

void adc1task(void* arg)
{
	int bat_count=0;
  adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));   // call esp_adc_cal_characterize() to initialise

	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(BatteryV, ADC_ATTEN_DB_11);
//  esp_adc_cal_value_t val_type = 
           esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
	vTaskDelay(500/portTICK_PERIOD_MS);

	while(1)
	{
		adc_val = adc1_get_raw(BatteryV);                     // Read battery data
		battery += adc_val;                                   // Accumulate
		if(bat_count == (samples - 1)){                       // taking average of 'samples measurements
      battery /= samples;                                 // Calculate average
      battery = esp_adc_cal_raw_to_voltage(battery, adc_chars); // Fix ESP32 non-linearity
      battery *= 2;                                       // Account for /2 resistor divider

      if (battery > 4100) battery = 4100;                 // Constrain range for later (0% - 100%) mapping
      else if (battery < 3600 ) battery = 3600;

      if (startup_flag == true) {                         // First measurement
        bat_prev = battery;
        startup_flag = false;
      }

  ESP_LOGI(TAG4, "Battery %%: %d V", battery);
      if((battery/10)      >= 410) battery = 100;         // Map battery volts to %Full
      else if((battery/10) >= 405) battery = 90;
      else if((battery/10) >= 400) battery = 80;
      else if((battery/10) >= 395) battery = 70;
      else if((battery/10) >= 390) battery = 60;
      else if((battery/10) >= 385) battery = 50;
      else if((battery/10) >= 380) battery = 45;
      else if((battery/10) >= 375) battery = 40;
      else if((battery/10) >= 370) battery = 30;
      else if((battery/10) >= 365) battery = 25;
      else if((battery/10) <  365) battery = 20;
  ESP_LOGI(TAG4, "Battery %%: %d %%", battery);

/*    bt_rem = (battery % 100);                           // Update current held value ?? 
      if((bt_rem > 80) && (bt_rem < 99) && (bat_prev != 0)) battery = bat_prev;
      bat_prev = battery;
*/

      bat_percent = (uint8_t) battery;
      update_bat(bat_percent);		
      bat_count=0;
      battery=0;

      }
		else bat_count++;
		vTaskDelay(4000/portTICK_PERIOD_MS);                  // 4s
  }
}

void adc_start(void)
{
	xQueue_battery = xQueueCreate(2, sizeof( int ));
	if( xQueue_battery==NULL )
	{
		printf( "Failed to create xQueue_battery..!");
	}
	xTaskCreate(adc1task, "adc1task", 4096, NULL, 2, NULL);
}
