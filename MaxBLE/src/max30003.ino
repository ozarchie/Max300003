#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <arduino.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_bt.h"
#include "esp_wifi.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "driver/ledc.h"
#include "driver/uart.h"

#include "max30003.h"
#include "ble.h"
#include "arrhythmia.h"

#define TAG3 "MAX30003:"

extern unsigned int global_heartRate ;

static xQueueHandle max30003intSemaphore;
xQueueHandle rgbLedSemaphore;

SemaphoreHandle_t updateR2RSemaphore = NULL;
volatile uint16_t HR;
volatile uint16_t R2R;

uint8_t SPI_TX_Buff[4];                     // 32 bits
uint8_t SPI_temp_32b[4];
uint8_t SPI_RX_Buff[10];
uint8_t DataPacketHeader[MSGMAX];
spi_device_handle_t spi;

signed long ecgdata;
unsigned long data;
int flag;
unsigned long ps;
uint8_t rtor_detected  = 0;

unsigned int array[MSGMAX];
int rear = -1;
float rmssd;
float mean_f;
float rmssd_f;
int count = 0;
float per_pnn;
int sqsum;
int hist[] = {0};
float sdnn;
float sdnn_f;
int k=0;
int min_f=0;
int max_f=0;
int max_t=0;
int min_t=0;
float pnn_f=0;
float tri =0;
float array_temp[MSGMAX]={0.0};
extern uint8_t arrhythmiadetector;

void IRAM_ATTR gpio_isr_handler(void* arg) {
    BaseType_t mustYield=false;
    xSemaphoreGiveFromISR(max30003intSemaphore, &mustYield);
}


void max30003_spi_pre_transfer_callback(spi_transaction_t *t) {
    ;
}

void max30003_spi_post_transfer_callback(spi_transaction_t *t) {
    ;
}

// Configure the FCLK clock to 32768 Hz using ledc timer 0, ledc channel 0
void max30003_start_timer(void) {
    ledc_timer_config_t FCLK_timer =
    {
      .speed_mode = LEDC_HIGH_SPEED_MODE,       //timer mode,
      {.duty_resolution = LEDC_TIMER_10_BIT},   //resolution of PWM duty
      .timer_num = LEDC_TIMER_0,                //timer index
      .freq_hz = 32768                          //set frequency of pwm
    };

    ledc_timer_config(&FCLK_timer);

    ledc_channel_config_t FCLK_channel =
    {
        .gpio_num = PIN_NUM_FCLK,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 512
    };

    ledc_channel_config(&FCLK_channel);
}

void MAX30003_Reg_Write (unsigned char WRITE_ADDRESS, uint32_t data)
{
    uint8_t wRegName = (WRITE_ADDRESS << 1) | WREG;
    uint8_t txData[4];

    txData[0] = wRegName;
    txData[1] = (data >> 16);
    txData[2] = (data >> 8);
    txData[3] = (data);

    esp_err_t ret;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));           //Zero out the transaction

    t.length = 32;                      //Length is in bytes, transaction length is in bits.
    t.tx_buffer = &txData;              //Data
    ret=spi_device_transmit(spi, &t);   //Transmit!
    assert(ret == ESP_OK);              //Should have had no issues.
}


void MAX30003_ReadID(void)
{
   uint8_t SPI_TX_Buff[4];
   uint8_t SPI_RX_Buff[10];

   uint8_t Reg_address=INFO;

   SPI_TX_Buff[0] = (Reg_address << 1 ) | RREG;
   SPI_TX_Buff[1] = 0x00;
   SPI_TX_Buff[2] = 0x00;
   SPI_TX_Buff[3] = 0x00;

   esp_err_t ret;
   spi_transaction_t t;
   memset(&t, 0, sizeof(t));       // Zero out the transaction

   t.length = 32;                  // Command is 8 bits, 24 bits of data
   t.rxlength = 32;
   t.tx_buffer = &SPI_TX_Buff;     // The data is the cmd itself
   t.rx_buffer = &SPI_RX_Buff;

   t.user=(void*)0;                     //D/C needs to be set to 0
   ret=spi_device_transmit(spi, &t);    //Transmit!
   assert(ret == ESP_OK);               //Should have had no issues.
}

void max30003_Reg_Read(unsigned char READ_ADDRESS)
{
    uint8_t rRegName = READ_ADDRESS;

    SPI_TX_Buff[0] = (rRegName << 1 ) | RREG;
    SPI_TX_Buff[1] = 0x00;
    SPI_TX_Buff[2] = 0x00;
    SPI_TX_Buff[3] = 0x00;

    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));           //Zero out the transaction

    t.length = 32;
    t.rxlength = 32;
    t.tx_buffer = &SPI_TX_Buff;
    t.rx_buffer = &SPI_RX_Buff;

    t.user=(void*)0;
    ret=spi_device_transmit(spi, &t);
    assert(ret == ESP_OK);              //Should have had no issues.

    SPI_temp_32b[0] = SPI_RX_Buff[1];
    SPI_temp_32b[1] = SPI_RX_Buff[2];
    SPI_temp_32b[2] = SPI_RX_Buff[3];
}

void read_data(void *pvParameters)
{
    while(1)
    {
//        max30003_Reg_Read(STATUS);
//        uint32_t data = (SPI_RX_Buff[1] << 16) | (SPI_RX_Buff[2] << 8) | SPI_RX_Buff[3];
//        ESP_LOGI(TAG, "Wait for RRIRQ Loop, STATUS = 0x%.8X", data);
        xSemaphoreTake(max30003intSemaphore, portMAX_DELAY); //Wait until slave is ready
        max30003_read_send_data();
        xSemaphoreGive(updateR2RSemaphore);
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void max30003_sw_reset(void) {
    MAX30003_Reg_Write(SW_RST, 0x000000);
}

void max30003_synch(void) {
    MAX30003_Reg_Write(SYNCH, 0x000000);
}

void max30003_drdy_interrupt_enable() {              // MAX30003 DRDY (INTB) isr

    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_NEGEDGE;                          //GPIO_PIN_INTR_NEGEDGE;
    io_conf.pin_bit_mask = 1 << MAX30003_INTB_PIN;
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    gpio_set_intr_type(MAX30003_INTB_PIN, GPIO_INTR_NEGEDGE);       //GPIO_PIN_INTR_NEGEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_remove(MAX30003_INTB_PIN);
    gpio_isr_handler_add(MAX30003_INTB_PIN, gpio_isr_handler, (void*) MAX30003_INTB_PIN);
}

void max30003_initchip(int pin_miso, int pin_mosi, int pin_sck, int pin_cs )
{
    esp_err_t ret;

    ESP_LOGE(TAG3, "%s - initialising SPI controller\n", __func__);

    spi_bus_config_t buscfg = {                             // Pin values for MAX30003
        .mosi_io_num = pin_mosi,
        .miso_io_num = pin_miso,
        .sclk_io_num = pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        0,0,0,                                              // uint8_t command_bits, uint8_t address_bits, uint8_t dummy_bits
        .mode = (uint8_t)0,                               	// SPI mode 0
        0,0,0,                                              // uint8_t duty_cycle_pos, uint8_t cs_ena_pretrans, uint8_t cs_ena_posttrans
        .clock_speed_hz = (int)4000000,             	    // Clock out at 10 MHz
        0,                                                  // int input_delay_ns
        .spics_io_num = (int)pin_cs,              		    // CS pin
        0,                                                  // uint32_t flags (Bitwise OR of SPI_DEVICE_* flags)
        .queue_size = (int)7,                                    // queue 7 transactions at a time
        .pre_cb = (transaction_cb_t)max30003_spi_pre_transfer_callback,         // Specify pre-transfer callback to handle D/C line, before a transmission is started
        .post_cb = (transaction_cb_t)max30003_spi_post_transfer_callback        // Specify post-transfer callback to handle (Not sure), after a transmission has completed
    };

    //Initialize the SPI bus
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0);    // MAX30003 is on the HSPI bus, with CS = GPIO25
    assert(ret == ESP_OK);
    //Attach the MAX30003 to the SPI bus
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret == ESP_OK);
    
    max30003_start_timer();                             // Starts the MAX30003 FCLK timer at 32768Hz
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGE(TAG3, "%s - initialising MAX registers\n", __func__);

    max30003_sw_reset();                                // Reset MAX30003 
    vTaskDelay(100 / portTICK_PERIOD_MS);
    MAX30003_Reg_Write(CNFG_GEN, 0x080217);             // No ULP, 32768, EN_ECG, No DCLOFF, 10nA, Resistive Bias (100M)
    vTaskDelay(100 / portTICK_PERIOD_MS);
    MAX30003_Reg_Write(CNFG_CAL, 0x004800);             // Calib sources disabled, bipolar, 50mV
    vTaskDelay(100 / portTICK_PERIOD_MS);
    MAX30003_Reg_Write(CNFG_EMUX, 0x000000);            // ECGP = isolated; ECGN = isolated
    vTaskDelay(100 / portTICK_PERIOD_MS);
    MAX30003_Reg_Write(CNFG_ECG, 0x835000);             // 128sps; DHPF = .5Hz, DLPF = 40Hz 
    vTaskDelay(100 / portTICK_PERIOD_MS);
    MAX30003_Reg_Write(CNFG_RTOR1, 0x3fb300);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    MAX30003_Reg_Write(MNGR_INT, 0x180014);             // FIFO buffer = 4; Clear RRIRQ on RTOR Read, Clear SAMP IRQ
    vTaskDelay(100 / portTICK_PERIOD_MS);
	MAX30003_Reg_Write(EN_INT, 0x000401);               // Enable RRIRQ; CMOS
    vTaskDelay(100 / portTICK_PERIOD_MS);
    max30003_synch();

    ESP_LOGE(TAG3, "%s - enabling interrupts\n", __func__);

	max30003_drdy_interrupt_enable();                   // Enable INTB from MAX (RRIRQ)
}

uint8_t* max30003_read_send_data(void)
{	
    update_beat_blr();                                              // Heartbeat flag
    max30003_Reg_Read(RTOR);                                        // Read the rtor data

    unsigned long RTOR_msb = (unsigned long) (SPI_temp_32b[0]);
    unsigned char RTOR_lsb = (unsigned char) (SPI_temp_32b[1]);

    unsigned long rtor = (RTOR_msb << 8 | RTOR_lsb);
    rtor = ((rtor >> 2) & 0x3fff) ;

    float hr =  60 / ((float)rtor * RTOR_LSB_RES);                  // Use period for 32768Hz (0.0078125)
    ESP_LOGI(TAG3, "HR: %2.1f", hr);

    global_heartRate = (unsigned int)hr ; 
    HR = (unsigned int)hr;
    R2R = (unsigned int)((float)rtor * (RTOR_LSB_RES * 1000)) ;     // (7.8125ms)
    k++;
 
    update_hr(HR);                                                  // <ble_data_update.hr=value;>
    update_rr(R2R);                                                 // <ble_data_update.rr=value;>

    if (rear == (MSGMAX - 1)) {
        for(int i=0; i<(MSGMAX - 1); i++)
        {
            array[i] = array[i + 1];
        }
        array[MSGMAX - 1] = R2R;	  
    }
    else {		  
      rear++;
      array[rear] = R2R;
    }
	
    if(k >= MSGMAX) {	
      max_f = arraymax(array);
      min_f = arraymin(array);
      mean_f = mean(array);
      sdnn_f = sdnn_ff(array);
      pnn_f = pnn_ff(array);

      update_mean((int)(mean_f * 100));
      update_sdnn((uint16_t)(sdnn_f * 100));
      update_pnn((uint16_t)(per_pnn * 100));

      for(int i=0; i<MSGMAX; i++) {
       array_temp[i] = ((float)(array[i]) / 1000);
      }
      challenge(array_temp);
      update_arrhythmia(arrhythmiadetector); 
    }
   return  DataPacketHeader;
}

int arraymax(unsigned int array[])
{  
    for(int i=0;i<MSGMAX;i++) {
        if(array[i] > max_t) {
            max_t = array[i];
        }
    }
    return max_t;
}

int arraymin(unsigned int array[])
{   
  	min_t = max_f;
  	for(int i=0;i<MSGMAX;i++) { 
        if(array[i] < min_t) {
  		    min_t = array[i];	
  		}
  	}
  	return min_t;
}

float mean(unsigned int array[])
{ 
    int sum = 0;
    float mean_rr;

    for(int i=0;i<(MSGMAX);i++) {
      sum = sum + array[i];
    }
    mean_rr = (((float)sum) / MSGMAX);	
    return mean_rr;
}	

float sdnn_ff(unsigned int array[])
{
  	int sumsdnn = 0;
  	int diff;
  	
    for(int i=0;i<(MSGMAX);i++) {
      diff = (array[i] - (mean_f));
      diff = diff * diff;
      sumsdnn = sumsdnn + diff;		
    }
    sdnn = (sqrt(sumsdnn / (MSGMAX)));
    return sdnn;
}

float pnn_ff(unsigned int array[])
{ 
    unsigned int pnn50[MSGMAX];
    count = 0;
    sqsum = 0;
    
    for(int i=0;i<(MSGMAX-2);i++) {
      pnn50[i] = abs(array[i+1] - array[i]);
      sqsum = sqsum + (pnn50[i] * pnn50[i]);
      if(pnn50[i] > 50) {
       count += 1;		 
      }
    }
    per_pnn = ((float)count / MSGMAX) * 100;
    rmssd = sqrt(sqsum / (MSGMAX - 1));
    update_rmssd((uint16_t)(rmssd * 100));
    return per_pnn;
}

void start_max30003(void)
{

    ESP_LOGE(TAG3, "%s - starting controller\n", __func__);

    max30003intSemaphore = xSemaphoreCreateBinary();
    updateR2RSemaphore = xSemaphoreCreateMutex();
    rgbLedSemaphore = xSemaphoreCreateBinary();
    xTaskCreate(&read_data, "read_data", 4096, NULL, 4, NULL);
}

/*
void send_data(uint8_t *dataToSend, int dataLength)
{

}
*/