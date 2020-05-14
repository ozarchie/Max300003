#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"                 // implements BT controller and VHCI configuration procedures from the host side

// bluedroid
#include "esp_bt_main.h"            // implements initialization and enabling of the Bluedroid stack
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"        // implements GAP configuration, such as advertising and connection parameters
#include "esp_gatts_api.h"          // implements GATT configuration, such as creating services and characteristics
#include "esp_gatt_defs.h"

#include "ble.h"
#include "arrhythmia.h"

/*
  (https://blog.linuxc.com.cn/tutorial/Gatt_Server_Example_Walkthrough.html)
  This implements a Bluetooth Low Energy (BLE) Generic Attribute Profile (GATT) Server on the HeartSensor.
  This code is designed around three Application Profiles and a series of events that are handled in order to execute a sequence of configuration steps,
  such as defining advertising parameters, updating connection parameters and creating services and characteristics.
  In addition, this code handles read and write events, including a Write Long Characteristic request,
  which divides the incoming data into chunks so that the data can fit in the Attribute Protocol (ATT) message.
 */

uint8_t rr_service_uuid[16]= {0xd0,0x36,0xba,0x8c,0xda,0xd1,0x4c,0xae,0xb8,0x7d,0x48,0x44,0x91,0x74,0x5c,0xcd};   //custom 128bit service UUID for R2R
uint8_t rr_char_uuid[16]= {0xdc,0xad,0x7f,0xc4,0x23,0x90,0x4d,0xd4,0x96,0x8d,0x0f,0x97,0x6f,0xa8,0xbf,0x01};

static void gatts_profile_hr_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_hrv_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_bat_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

uint8_t beat_flag = 0;
uint16_t attr_handle_hr   = 0x002a;
uint16_t attr_handle_rr   = 0x002a;
uint16_t attr_handle_bat  = 0x002a;

uint16_t conn_id_indicate_hr  = 0;
uint16_t conn_id_indicate_rr  = 0;
uint16_t conn_id_indicate_bat = 0;

esp_gatt_if_t gatts_if_for_hr   = ESP_GATT_IF_NONE;
esp_gatt_if_t gatts_if_for_rr   = ESP_GATT_IF_NONE;
esp_gatt_if_t gatts_if_for_bat  = ESP_GATT_IF_NONE;

// Format of data from HeartSensor
extern unsigned char DataPacketHeader[];
struct ble_data_att
{
  uint16_t  pnn;                  // Mean times in 1hr in which two successive interval changes exceed 50mS                
  uint8_t   bat;                  // Batttery level: % 100%=(4.1V) to 20%=(3.6V)
  uint16_t  stress;               //
  uint16_t  hr;                   // HeartRate
  uint16_t  rr;                   // R-R            
  uint16_t  rmssd;                // Root Mean Square diference between successive R-R
  int       mean;                 // Mean R-R over last 30 samples
  uint16_t  sdnn;                 // Standard Deviation of R-R
  uint8_t   arrhythmiadetector; 

} ble_data_update;

uint8_t char1_str[] = {0x11,0x22,0x33};

esp_attr_value_t gatts_demo_char1_val = 
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06, 0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
        0x45, 0x4d, 0x4f, 0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};
static uint8_t raw_scan_rsp_data[] = {
        0x02, 0x01, 0x06, 0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
        0x45, 0x4d, 0x4f, 0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};

#else

// Heart Rate
static uint8_t hr_service_uuid128[32] =
{
  //first uuid, 16bit, [12],[13] is the value
  0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xAB, 0xCD, 0x00, 0x00,
  //second uuid, 32bit, [12], [13], [14], [15] is the value
  0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xAB, 0xCD, 0xAB, 0xCD,
};

// Advertising Data

/*
  The esp_ble_adv_data_t data structure for advertising data has the following definition:

  typedef struct {
      bool set_scan_rsp;            // Set this advertising data as scan response or not
      bool include_name;            // Advertising data include device name or not 
      bool include_txpower;         // Advertising data include TX power 
      int min_interval;             // Advertising data show advertising min interval
      int max_interval;             // Advertising data show advertising max interval
      int appearance;               // External appearance of device
      uint16_t manufacturer_len;    // Manufacturer data length
      uint8_t *p_manufacturer_data; // Manufacturer data point
      uint16_t service_data_len;    // Service data length
      uint8_t *p_service_data;      // Service data point
      uint16_t service_uuid_len;    // Service uuid length
      uint8_t *p_service_uuid;      // Service uuid array point
      uint8_t flag;                 // Advertising flag of discovery mode, see BLE_ADV_DATA_FLAG detail
  } esp_ble_adv_data_t;

  The minimum and maximum advertisement intervals are set as multiples of 1.25 ms.
  In this code, the minimum advertisement interval is defined as    0x20 * 1.25 ms = 40 ms,
  and the maximum advertisement interval is initialized as         0x40 * 1.25 ms = 80 ms.

  An advertising payload can be up to 31 bytes of data.
  It is possible the parameter data is large enough to surpass the 31-byte advertisement packet limit,
  which causes the stack to cut the advertisement packet and leave some of the parameters out.

  It is possible to also advertise customized raw data using the esp_ble_gap_config_adv_data_raw()
  and esp_ble_gap_config_scan_rsp_data_raw() functions,
  which require to create and pass a buffer for both advertising data and scanning response data.
  In this code, the raw data is represented by the raw_adv_data[] and raw_scan_rsp_data[] arrays.

*/

static esp_ble_adv_data_t test_adv_data = 
{
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, 					
    .p_manufacturer_data =  NULL, 			
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = hr_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif 


// Advertising Parameters
static esp_ble_adv_params_t test_adv_params = 
{
    .adv_int_min        = (uint16_t)0x20,
    .adv_int_max        = (uint16_t)0x40,
    .adv_type           = (esp_ble_adv_type_t)ADV_TYPE_IND,
    .own_addr_type      = (esp_ble_addr_type_t)BLE_ADDR_TYPE_PUBLIC,
    .peer_addr          = {0,0,0,0,0,0},
    .peer_addr_type     = (esp_ble_addr_type_t)BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = (esp_ble_adv_channel_t)ADV_CHNL_ALL,
    .adv_filter_policy  = (esp_ble_adv_filter_t)ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


    esp_bd_addr_t           peer_addr;          /*!< Peer device bluetooth device address */
    esp_ble_addr_type_t     peer_addr_type;     /*!< Peer device bluetooth device address type, only support public address type and random address type */

/*
  Each profile is defined as a struct where the struct members depend on the services and characteristics that are implemented in that Application Profile.
  The members also include a GATT interface, Application ID, Connection ID and a callback function to handle profile events.
  In this appln, each profile is composed by:
    GATT interface
    Application ID
    Connection ID
    Service handle
    Service ID
    Characteristic handle
    Characteristic UUID
    Attribute permissions
    Characteristic properties
    Client Characteristic Configuration descriptor handle
    Client Characteristic Configuration descriptor UUID

  This profile was designed to have one service and one characteristic, and that the characteristic has one descriptor.
  The service has a handle and an ID, in the same manner that each characteristic has a handle, an UUID, attribute permissions and properties.
  In addition, if the characteristic supports notifications or indications, it must implement a Client Characteristic Configuration descriptor (CCCD),
   which is an additional attribute that describes if the notifications or indications are enabled and defines how the characteristic may be configured by a specific client.
   This descriptor also has a handle and an UUID.  
*/

struct gatts_profile_inst 
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/*
  The Application Profiles are stored in an array and corresponding callback functions "gatts_profile_xxx_event_handler()" are assigned.
  Different applications on the GATT client use different interfaces, represented by the gatts_if parameter.
    For initialization, this parameter is set to ESP_GATT_IF_NONE, which means that the Application Profile is not linked to any client yet.
*/
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = 
{
// Heart Rate
    [PROFILE_A_APP_ID] = 
  {
        .gatts_cb = gatts_profile_hr_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,   
    },
// Heart Rate Variability  
  [PROFILE_C_APP_ID] = 
  {
        .gatts_cb = gatts_profile_hrv_event_handler,                  
        .gatts_if = ESP_GATT_IF_NONE,      
    },
// Battery  
  [PROFILE_D_APP_ID] = 
  {
        .gatts_cb = gatts_profile_bat_event_handler,                  
        .gatts_if = ESP_GATT_IF_NONE,      
    },

};

/*
  GAP Event Handler
  Once the advertising data have been set, the GAP event ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT is triggered.
  For the case of raw advertising data set, the event triggered is ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT.
  Additionally when the raw scan response data is set, ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT event is triggered.
*/
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) 
  {			
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&test_adv_params);
        break;
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&test_adv_params);
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&test_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:        
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }
        break;
    default:
        break;
    }
}

/*
The register application event is the first one that is triggered during the lifetime of the program.
This code uses the Profile A GATT event handle to configure the advertising parameters upon registration.

This code has the option to use both standard Bluetooth Core Specification advertising parameters or a customized raw buffer.
The option can be selected with the CONFIG_SET_RAW_ADV_DATA define.
 The raw advertising data can be used to implement iBeacons, Eddystone or other proprietaries,
  and custom frame types such as the ones used for Indoor Location Services that are different from the standard specifications.
The function used to configure standard Bluetooth Specification advertisement parameters is esp_ble_gap_config_adv_data(),
 which takes a pointer to an esp_ble_adv_data_t structure.
*/

/*
  To set the device name, the esp_ble_gap_set_device_name() function is used.
  The registering event handler follows:
*/

static void gatts_profile_hr_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event)
  {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_HR;
        esp_ble_gap_set_device_name(TEST_DEVICE_NAME);

#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
#else
        esp_ble_gap_config_adv_data(&test_adv_data);
#endif    
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_HR);
        break;
      
    case ESP_GATTS_READ_EVT: 
    {
      ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
      esp_gatt_rsp_t rsp;
      memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
      rsp.attr_value.handle = param->read.handle;
      rsp.attr_value.len = 2;
      rsp.attr_value.value[0] = ( 0 ); 							
      rsp.attr_value.value[1] = (uint8_t)ble_data_update.hr;				

      esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                    ESP_GATT_OK, &rsp);
      break;
    }

    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;
    }

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_HR;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);

        esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY, 
                               &gatts_demo_char1_val, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT: {
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

    attr_handle_hr = param->add_char.attr_handle;		
        esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
                   
        break;
    }

    case ESP_GATTS_CONNECT_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;		
        conn_id_indicate_hr = 	param->connect.conn_id;			
        gatts_if_for_hr = gatts_if;		
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        esp_ble_gap_start_advertising(&test_adv_params);		
        gatts_if_for_hr = ESP_GATT_IF_NONE;		
        break;

    default:
        break;
    }
}


static void gatts_profile_bat_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {

    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_D_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_D_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_D_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_D_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_BAT;
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_D_APP_ID].service_id, GATTS_NUM_HANDLE_HR);
        break;

    case ESP_GATTS_READ_EVT: {
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 1;
        rsp.attr_value.value[0] = ((ble_data_update.bat));
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;
    }

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_D_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_D_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_D_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_BAT;
        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_D_APP_ID].service_handle);
        esp_ble_gatts_add_char(gl_profile_tab[PROFILE_D_APP_ID].service_handle, &gl_profile_tab[PROFILE_D_APP_ID].char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                               NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:

        gl_profile_tab[PROFILE_D_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_D_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_D_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_D_APP_ID].service_handle, &gl_profile_tab[PROFILE_D_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     NULL, NULL);
        attr_handle_bat = param->add_char.attr_handle;

        break;

    case ESP_GATTS_CONNECT_EVT:
        gl_profile_tab[PROFILE_D_APP_ID].conn_id = param->connect.conn_id;
        gatts_if_for_bat = gatts_if;
        conn_id_indicate_bat = param->connect.conn_id;	
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        gatts_if_for_bat = ESP_GATT_IF_NONE;
    break;

    default:
        break;
    }
}

static void gatts_profile_hrv_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {

    case ESP_GATTS_REG_EVT:       
        gl_profile_tab[PROFILE_C_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_C_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_C_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_128;
        for(int i=0; i<16; i++){
          gl_profile_tab[PROFILE_C_APP_ID].service_id.id.uuid.uuid.uuid128[i]=rr_service_uuid[i];
        }    
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_C_APP_ID].service_id, GATTS_NUM_HANDLE_HR);
        break;

    case ESP_GATTS_READ_EVT: {
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 1;
        rsp.attr_value.value[0] = ((ble_data_update.mean));
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;
    }

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_C_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_C_APP_ID].char_uuid.len = ESP_UUID_LEN_128;
        for(int i=0; i<16; i++){
            gl_profile_tab[PROFILE_C_APP_ID].char_uuid.uuid.uuid128[i] = rr_char_uuid[i];
        }
        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_C_APP_ID].service_handle);
        esp_ble_gatts_add_char(gl_profile_tab[PROFILE_C_APP_ID].service_handle,
                               &gl_profile_tab[PROFILE_C_APP_ID].char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                               NULL, NULL);
        break;
  
    case ESP_GATTS_ADD_CHAR_EVT:      
        gl_profile_tab[PROFILE_C_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_C_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_C_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        attr_handle_rr = param->add_char.attr_handle;
        esp_ble_gatts_add_char_descr( gl_profile_tab[PROFILE_C_APP_ID].service_handle,
                                      &gl_profile_tab[PROFILE_C_APP_ID].descr_uuid,
                                      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                      NULL, NULL);
        break;           
  
    case ESP_GATTS_CONNECT_EVT:
        gl_profile_tab[PROFILE_C_APP_ID].conn_id = param->connect.conn_id;		
        gatts_if_for_rr = gatts_if;			
        conn_id_indicate_rr = param->connect.conn_id;	
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        gatts_if_for_rr = ESP_GATT_IF_NONE;
    break;

    default:
        break;
    }
}

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
    }
    else {
      ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id, 
                    param->reg.status);
      return;
    }
  }
  do {
    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++) {
      if (gatts_if == ESP_GATT_IF_NONE ||  gatts_if == gl_profile_tab[idx].gatts_if) 
      {
        if (gl_profile_tab[idx].gatts_cb)
        {
          gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
        }
      }
    }
  } while (0);
}

void update_ble_atts(void(*update)(uint16_t),uint16_t value) {
  update(value);
}

void update_stress(uint16_t value) {
  ble_data_update.stress=value;	
}

void update_rmssd(uint16_t value) {
  ble_data_update.rmssd=value;	
}

void update_hr(uint16_t value) {
  ble_data_update.hr=value;
}

void update_rr(uint16_t value) {
  ble_data_update.rr=value;
}

void update_mean(int value) {
  ble_data_update.mean=value;	
}

void update_sdnn(uint16_t value) {
  ble_data_update.sdnn=value;	
}

void update_bat(uint8_t value) {
  ble_data_update.bat=value;	
  esp_ble_gatts_send_indicate(gatts_if_for_bat, conn_id_indicate_bat, attr_handle_bat, 1, &(ble_data_update.bat), false);		//updates in every 40s
}

void update_pnn(uint16_t value) {
  ble_data_update.pnn=value;	
}

void update_beat_blr(void) {
  beat_flag = 1;
}

void update_arrhythmia(uint8_t value) {
    ble_data_update.arrhythmiadetector=value;
}

static void notify_task(void* arg) {
    
  while (true) {
      
    while (gatts_if_for_hr == ESP_GATT_IF_NONE) {			//checking the connection
        vTaskDelay(5000/ portTICK_PERIOD_MS);
    }
                
    vTaskDelay(10/ portTICK_PERIOD_MS);
       
    uint8_t value_arr_hr[4];
    uint8_t value_arr_hrv_anls[15];	 			  			  
         
    value_arr_hr[0] =( 0x10 );	
    value_arr_hr[1] = (uint8_t)ble_data_update.hr;
    value_arr_hr[2] = ble_data_update.rr;
    value_arr_hr[3] = ble_data_update.rr>>8;
        
    value_arr_hrv_anls[0] = (ble_data_update.mean);
    value_arr_hrv_anls[1] = (ble_data_update.mean>>8);
    value_arr_hrv_anls[2] = (ble_data_update.mean>>16);
    value_arr_hrv_anls[3] = (ble_data_update.mean>>24);
    value_arr_hrv_anls[4] = (ble_data_update.sdnn);
    value_arr_hrv_anls[5] = (ble_data_update.sdnn>>8);
     
    value_arr_hrv_anls[6] = (ble_data_update.pnn);
    value_arr_hrv_anls[7] = (ble_data_update.pnn>>8);
    value_arr_hrv_anls[8] = (ble_data_update.stress);
    value_arr_hrv_anls[9] = (ble_data_update.stress>>8);
    value_arr_hrv_anls[10] = (ble_data_update.rmssd);
    value_arr_hrv_anls[11] = (ble_data_update.rmssd>>8);
    //value_arr_hrv_anls[12] = (uint8_t)ble_data_update.hr;
    value_arr_hrv_anls[12] = ble_data_update.arrhythmiadetector;
    // ESP_LOGE(GATTS_TAG, "arr val%d \n", ble_data_update.arrhythmiadetector);
                            
    if(beat_flag) {
      esp_ble_gatts_send_indicate(gatts_if_for_hr, conn_id_indicate_hr, attr_handle_hr, 4, value_arr_hr, false);
      esp_ble_gatts_send_indicate(gatts_if_for_rr, conn_id_indicate_rr, attr_handle_rr, 13, value_arr_hrv_anls, false);
      beat_flag = 0;
    }
  }
}

void ble_Init(void)
{
  esp_err_t ret;
/*
  BT Controller and Stack Initialization
  The main function also initializes the BT controller by first creating a BT controller configuration structure named esp_bt_controller_config_t
   with default settings generated by the BT_CONTROLLER_INIT_CONFIG_DEFAULT() macro.
  The BT controller implements the Host Controller Interface (HCI) on the controller side, the Link Layer (LL) and the Physical Layer (PHY).
  The BT Controller is invisible to the user applications and deals with the lower layers of the BLE stack.
  The controller configuration includes setting the BT controller stack size, priority and HCI baud rate.

  With the settings created, the BT controller is initialized and enabled with the esp_bt_controller_init() function:
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
  
  Next, the controller is enabled in BLE Mode:
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    The controller should be enabled in ESP_BT_MODE_BTDM, if you want to use the dual mode (BLE + BT).

  There are four Bluetooth modes supported:
    ESP_BT_MODE_IDLE: Bluetooth not running
    ESP_BT_MODE_BLE: BLE mode
    ESP_BT_MODE_CLASSIC_BT: BT Classic mode
    ESP_BT_MODE_BTDM: Dual mode (BLE + BT Classic)

  After the initialization of the BT controller, the Bluedroid stack, which includes the common definitions and APIs for both BT Classic and BLE,
  is initialized and enabled by using:
      ret = esp_bluedroid_init();
      ret = esp_bluedroid_enable();
  The Bluetooth stack is up and running at this point in the program flow.
*/
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

  ret = esp_bt_controller_init(&bt_cfg);
  if (ret != ESP_OK) {
      ESP_LOGE(GATTS_TAG, "%s initialise controller failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }
  ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
  if (ret != ESP_OK) {
      ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }
  ret = esp_bluedroid_init();
  if (ret != ESP_OK) {
      ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }
  ret = esp_bluedroid_enable();
  if (ret != ESP_OK) {
      ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }
  ESP_LOGE(GATTS_TAG, "%s Initialise bluetooth dual mode, bluedroid succeeded\n", __func__);

/*
  The functionality of the application is defined by reacting to events such as what happens when another device tries to read or write parameters and establish a connection.
  The two main managers of events are the GAP and GATT event handlers.
  The application needs to register a callback function for each event handler in order to let the application know which functions are going to handle the GAP and GATT events:
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
  The functions gatts_event_handler() and gap_event_handler() handle all the events that are pushed to the application from the BLE stack.
*/

  ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret){
    ESP_LOGE(GATTS_TAG, "%s gatts register error, error code = %x", __func__, ret);
    return;
  }
  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret){
    ESP_LOGE(GATTS_TAG, "%s gap register error, error code = %x", __func__, ret);
    return;
  }
  
  /*
    Finally, the Application Profiles are registered using the Application ID, which is an user-assigned number to identify each profile.
    In this way, multiple Application Profiles can run in one server.
  */

  ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
  if (ret){
    ESP_LOGE(GATTS_TAG, "gatts app register error - PROFILE_A_APP_ID, error code = %x", ret);
    return;
  }
  ret = esp_ble_gatts_app_register(PROFILE_C_APP_ID);
  if (ret){
    ESP_LOGE(GATTS_TAG, "gatts app register error - PROFILE_C_APP_ID, error code = %x", ret);
    return;
  }
  ret = esp_ble_gatts_app_register(PROFILE_D_APP_ID);
  if (ret){
    ESP_LOGE(GATTS_TAG, "gatts app register error - PROFILE_D_APP_ID, error code = %x", ret);
    return;
  }

  xTaskCreate(notify_task, "notify_task", 1024*4, NULL, 3, NULL);
}
