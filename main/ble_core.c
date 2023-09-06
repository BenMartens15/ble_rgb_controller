
/* INCLUDES *******************************************************************/
#include "ble_core.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "led.h"
#include "nvs_flash.h"
#include "rgb_controller.h"
#include "switch_controller.h"
/******************************************************************************/

/* DEFINES ********************************************************************/
#define BLE_TAG                     "BLE_CORE"
#define BLE_LOG_LEVEL               ESP_LOG_INFO

#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "Lightning-LC2444"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)
/******************************************************************************/

/* ENUMS **********************************************************************/
/******************************************************************************/

/* STRUCTURES *****************************************************************/
/******************************************************************************/

/* GLOBALS ********************************************************************/
static uint8_t adv_config_done = 0;

uint16_t attribute_handle_table[NUM_ATTRIBUTES];

// lightning service UUID: f19a2444-96fe-4d87-a476-68a7a8d0b7ba
static uint8_t lightning_srvc_uuid[ESP_UUID_LEN_128] = {
    0xba, 0xb7, 0xd0, 0xa8, 0xa7, 0x68, 0x76, 0xa4,
    0x87, 0x4d, 0xfe, 0x96, 0x44, 0x24, 0x9a, 0xf1
};

// control characteristic UUID: f19a2445-96fe-4d87-a476-68a7a8d0b7ba
static uint8_t control_char_uuid[ESP_UUID_LEN_128] = {
    0xba, 0xb7, 0xd0, 0xa8, 0xa7, 0x68, 0x76, 0xa4,
    0x87, 0x4d, 0xfe, 0x96, 0x45, 0x24, 0x9a, 0xf1
};

// info characteristic UUID: f19a2446-96fe-4d87-a476-68a7a8d0b7ba
static uint8_t info_char_uuid[ESP_UUID_LEN_128] = {
    0xba, 0xb7, 0xd0, 0xa8, 0xa7, 0x68, 0x76, 0xa4,
    0x87, 0x4d, 0xfe, 0x96, 0x46, 0x24, 0x9a, 0xf1
};

ble_mfg_adv_data_t mfg_adv_data = {
    .company_id = 0xffff,
    .light_state = false,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = false,
    .appearance          = 0x00,
    .manufacturer_len    = 9, 
    .p_manufacturer_data = (uint8_t*)&mfg_adv_data,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 0,
    .p_service_uuid      = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x640, // 1s advertising interval
    .adv_int_max         = 0x640,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t cccd_init_value[2]            = {0x00, 0x00};
static const uint8_t char_init_value[7]            = {'U', 'n', 'n', 'a', 'm', 'e', 'd'}; 

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[NUM_ATTRIBUTES] =
{
    // Service Declaration
    [LIGHTNING_SRVC] = {
        {
            .auto_rsp = ESP_GATT_AUTO_RSP
        }, 
        {
            .uuid_length = ESP_UUID_LEN_16, 
            .uuid_p = (uint8_t *)&primary_service_uuid, 
            .perm = ESP_GATT_PERM_READ,
            .max_length = ESP_UUID_LEN_128, 
            .length = ESP_UUID_LEN_128, 
            .value = (uint8_t *)&lightning_srvc_uuid
        }
    },

    // Characteristic Declaration
    [LIGHTNING_CONTROL_CHAR] = {
        {
            .auto_rsp = ESP_GATT_AUTO_RSP
        }, 
        {
            .uuid_length = ESP_UUID_LEN_16, 
            .uuid_p = (uint8_t *)&character_declaration_uuid, 
            .perm = ESP_GATT_PERM_READ,
            .max_length = CHAR_DECLARATION_SIZE, 
            .length = CHAR_DECLARATION_SIZE, 
            .value = (uint8_t *)&char_prop_read_write_notify
        }
    },

    // Characteristic Value
    [LIGHTNING_CONTROL_CHAR_VALUE] = {
        {
            .auto_rsp = ESP_GATT_AUTO_RSP
        }, 
        {
            .uuid_length = ESP_UUID_LEN_128, 
            .uuid_p = (uint8_t *)&control_char_uuid, 
            .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            .max_length = GATTS_DEMO_CHAR_VAL_LEN_MAX, 
            .length = sizeof(char_init_value), 
            .value = (uint8_t *)char_init_value
        }
    },

    // Client Characteristic Configuration Descriptor
    [LIGHTNING_CONTROL_CHAR_CCCD] = {
        {
            .auto_rsp = ESP_GATT_AUTO_RSP
        }, 
        {
            .uuid_length = ESP_UUID_LEN_16, 
            .uuid_p = (uint8_t *)&character_client_config_uuid, 
            .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            .max_length = sizeof(uint16_t), 
            .length = sizeof(cccd_init_value), 
            .value = (uint8_t *)cccd_init_value
        }
    },

    // Characteristic Declaration
    [LIGHTNING_INFO_CHAR] = {
        {
            .auto_rsp = ESP_GATT_AUTO_RSP
        }, 
        {
            .uuid_length = ESP_UUID_LEN_16, 
            .uuid_p = (uint8_t *)&character_declaration_uuid, 
            .perm = ESP_GATT_PERM_READ,
            .max_length = CHAR_DECLARATION_SIZE, 
            .length = CHAR_DECLARATION_SIZE, 
            .value = (uint8_t *)&char_prop_read
        }
    },

    // Characteristic Value
    [LIGHTNING_INFO_CHAR_VALUE] = {
        {
            .auto_rsp = ESP_GATT_AUTO_RSP
        }, 
        {
            .uuid_length = ESP_UUID_LEN_128, 
            .uuid_p = (uint8_t *)&info_char_uuid, 
            .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            .max_length = GATTS_DEMO_CHAR_VAL_LEN_MAX, 
            .length = sizeof(char_init_value), 
            .value = (uint8_t *)char_init_value
        }
    },
};
/******************************************************************************/

/* PROTOTYPES *****************************************************************/
static void control_char_write(esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
/******************************************************************************/

/* PUBLIC FUNCTIONS ***********************************************************/
void ble_init(void)
{
    esp_err_t ret;

    esp_log_level_set(BLE_TAG, BLE_LOG_LEVEL);

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    mfg_adv_data.device_type = CONFIG_DEVICE_TYPE;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(BLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(BLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(BLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(BLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}

void ble_update_adv_data(void)
{
    mfg_adv_data.light_state = switch_control_get_state();

    esp_ble_gap_config_adv_data(&adv_data);
}
/******************************************************************************/ 

/* PRIVATE FUNCTIONS **********************************************************/
static void control_char_write(esp_ble_gatts_cb_param_t *param)
{
    ble_cmd_request_t request = {0x0};

    if (param->write.len < 2) {
        ESP_LOGE(BLE_TAG, "Not enough data supplied. Must specify at least one byte for the command and one byte for the data size.");
        return;
    }
    request.command = param->write.value[0];
    request.data_size = param->write.value[1];
    if (request.data_size != param->write.len - 2) {
        ESP_LOGE(BLE_TAG, "Specified data size and actual data size do not match. Specified: %d, actual: %d", request.data_size, param->write.len - 2);
        return;
    }
    memcpy(&request.data.rgb_colour, &param->write.value[2], request.data_size);
    ESP_LOGI(BLE_TAG, "BLE command - command: %d, size: %d", request.command, request.data_size);
    ESP_LOGI(BLE_TAG, "Data: ");
    esp_log_buffer_hex(BLE_TAG, request.data.rgb_colour, request.data_size);

    switch (request.command) {
        case BLE_CMD_SET_LIGHT_STATE:
            ESP_LOGI(BLE_TAG, "Command: BLE_CMD_SET_LIGHT_STATE (%d)", BLE_CMD_SET_LIGHT_STATE);

            if (request.data_size == 1) {
                if (request.data.on_off_state > 1) {
                    ESP_LOGE(BLE_TAG, "Invalid switch state: %d - must be either 1 or 0", request.data.on_off_state);
                } else {
                    switch_control_set_switch_state(request.data.on_off_state);
                }
            } else {
                ESP_LOGE(BLE_TAG, "Invalid data size: %d", request.data_size);
            }
            break;
        case BLE_CMD_SET_RGB_COLOUR:
            ESP_LOGI(BLE_TAG, "Command: BLE_CMD_SET_RGB_COLOUR (%d)", BLE_CMD_SET_RGB_COLOUR);
            
            if (request.data_size == 3) {
                rgb_control_set_colour(request.data.rgb_colour);

                if (request.data.rgb_colour[0] != 0 || request.data.rgb_colour[1] != 0 || request.data.rgb_colour[2] != 0) {
                    mfg_adv_data.light_state = true;
                    esp_ble_gap_config_adv_data(&adv_data);
                } else {
                    mfg_adv_data.light_state = false;
                    esp_ble_gap_config_adv_data(&adv_data);
                }
            } else {
                ESP_LOGE(BLE_TAG, "Invalid data size: %d", request.data_size);
            }
            break;
        case BLE_CMD_SET_RGB_BRIGHTNESS:
            ESP_LOGI(BLE_TAG, "Command: BLE_CMD_SET_RGB_BRIGHTNESS (%d)", BLE_CMD_SET_RGB_BRIGHTNESS);
            break;
        case BLE_CMD_SET_DEVICE_NAME:
            ESP_LOGI(BLE_TAG, "Command: BLE_CMD_SET_DEVICE_NAME (%d)", BLE_CMD_SET_DEVICE_NAME);

            if (request.data_size <= 100) {
                esp_ble_gatts_set_attr_value(attribute_handle_table[LIGHTNING_INFO_CHAR_VALUE], request.data_size, request.data.device_name);            
            } else {
                ESP_LOGE(BLE_TAG, "Name to large. Max length is 100 characters.");
            }
            break;
        case BLE_CMD_SET_MOTION_TIMEOUT:
            ESP_LOGI(BLE_TAG, "Command: BLE_CMD_SET_MOTION_TIMEOUT (%d)", BLE_CMD_SET_MOTION_TIMEOUT);

            if (request.data_size == 2) {
                switch_control_set_motion_timeout(__htons(request.data.motion_timeout));
            } else {
                ESP_LOGE(BLE_TAG, "Invalid data size: %d", request.data_size);
            }
            break;
        case BLE_CMD_SET_PIR_STATE:
            ESP_LOGI(BLE_TAG, "Command: BLE_CMD_SET_PIR_STATE (%d)", BLE_CMD_SET_PIR_STATE);

            if (request.data_size == 1) {
                if (request.data.on_off_state > 1) {
                    ESP_LOGE(BLE_TAG, "Invalid PIR state: %d - must be either 1 or 0", request.data.on_off_state);
                } else {
                    switch_control_set_pir_state(request.data.on_off_state);
                }
            } else {
                ESP_LOGE(BLE_TAG, "Invalid data size: %d", request.data_size);
            }
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(BLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(BLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(BLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(BLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(BLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(BLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(BLE_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, NUM_ATTRIBUTES, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(BLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(BLE_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        case ESP_GATTS_WRITE_EVT:
            // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
            esp_log_buffer_hex(BLE_TAG, param->write.value, param->write.len);

            if (param->write.handle == attribute_handle_table[LIGHTNING_CONTROL_CHAR_VALUE]) {
                ESP_LOGI(BLE_TAG, "Write to control characteristic");

                control_char_write(param);
            }

            // handle enabling and disabling of notification/indications
            if (attribute_handle_table[LIGHTNING_CONTROL_CHAR_CCCD] == param->write.handle && param->write.len == 2){
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    ESP_LOGI(BLE_TAG, "notify enable");
                    uint8_t notify_data[15];
                    for (int i = 0; i < sizeof(notify_data); ++i)
                    {
                        notify_data[i] = i % 0xff;
                    }
                    //the size of notify_data[] need less than MTU size
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, attribute_handle_table[LIGHTNING_CONTROL_CHAR_VALUE],
                                            sizeof(notify_data), notify_data, false);
                }else if (descr_value == 0x0002){
                    ESP_LOGI(BLE_TAG, "indicate enable");
                    uint8_t indicate_data[15];
                    for (int i = 0; i < sizeof(indicate_data); ++i)
                    {
                        indicate_data[i] = i % 0xff;
                    }
                    //the size of indicate_data[] need less than MTU size
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, attribute_handle_table[LIGHTNING_CONTROL_CHAR_VALUE],
                                        sizeof(indicate_data), indicate_data, true);
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(BLE_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(BLE_TAG, "unknown descr value");
                    esp_log_buffer_hex(BLE_TAG, param->write.value, param->write.len);
                }

                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
      	    break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(BLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(BLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(BLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(BLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(BLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);

            // turn on the connection LED
            led_on();
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(BLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);

            // turn off the connection LED
            led_off();
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(BLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != NUM_ATTRIBUTES){
                ESP_LOGE(BLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, NUM_ATTRIBUTES);
            }
            else {
                ESP_LOGI(BLE_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(attribute_handle_table, param->add_attr_tab.handles, sizeof(attribute_handle_table));
                esp_ble_gatts_start_service(attribute_handle_table[LIGHTNING_SRVC]);
            }
            break;
        }
        case ESP_GATTS_EXEC_WRITE_EVT:
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}
/******************************************************************************/
