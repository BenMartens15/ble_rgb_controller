#ifndef BLE_CORE_H
#define BLE_CORE_H

/* INCLUDES *******************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
/******************************************************************************/

/* DEFINES ********************************************************************/
#define BLE_CMD_SET_LIGHT_STATE             0x01
#define BLE_CMD_SET_RGB_COLOUR              0x02
#define BLE_CMD_SET_RGB_BRIGHTNESS          0x03
#define BLE_CMD_SET_DEVICE_NAME             0x04
#define BLE_CMD_SET_MOTION_TIMEOUT          0x05
#define BLE_CMD_SET_PIR_STATE               0x06
/******************************************************************************/

/* ENUMS **********************************************************************/
enum
{
    LIGHTNING_SRVC,

    LIGHTNING_CONTROL_CHAR,
    LIGHTNING_CONTROL_CHAR_VALUE,
    LIGHTNING_CONTROL_CHAR_CCCD,

    LIGHTNING_INFO_CHAR,
    LIGHTNING_INFO_CHAR_VALUE,

    NUM_ATTRIBUTES,
};
/******************************************************************************/

/* STRUCTURES *****************************************************************/
typedef struct {
    uint16_t company_id;
    bool light_state;
    bool pir_enabled;
    uint8_t device_type;
    uint8_t data[4];
} ble_mfg_adv_data_t;

typedef struct {
    uint8_t command;
    uint8_t data_size;
    union {
        uint8_t device_name[100];
        uint8_t rgb_colour[3];
        uint8_t on_off_state;
        uint16_t motion_timeout;
    } data;
} ble_cmd_request_t;
/******************************************************************************/

/* GLOBALS ********************************************************************/
/******************************************************************************/

/* PROTOTYPES *****************************************************************/
void ble_init(void);
void ble_update_adv_data(void);
/******************************************************************************/

#endif /* #ifndef BLE_CORE_H */
