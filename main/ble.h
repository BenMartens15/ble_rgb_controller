
#ifndef BLE_H
#define BLE_H

/* DEFINES ********************************************************************/
#define GATTS_TAG "BLE"

#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4

#define TEST_DEVICE_NAME            "RGB CONTROLLER"
#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
/******************************************************************************/


/* INCLUDES *******************************************************************/
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
/******************************************************************************/


/* STRUCTURES *****************************************************************/
struct gatts_profile_inst {
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

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;
/******************************************************************************/


/* PROTOTYPES *****************************************************************/
void ble_init();
void write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
/******************************************************************************/

#endif /* #ifndef BLE_H */