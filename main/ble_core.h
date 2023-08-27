#ifndef FILE_H
#define FILE_H

/* INCLUDES *******************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
/******************************************************************************/

/* DEFINES ********************************************************************/
#define DEVICE_TYPE_RGB_CONTROLLER          0
#define DEVICE_TYPE_LIGHT_SWITCH            1

#define BLE_CMD_SET_LIGHT_STATE             0x01
#define BLE_CMD_SET_RGB_COLOUR              0x02
#define BLE_CMD_SET_RGB_BRIGHTNESS          0x03
#define BLE_CMD_SET_DEVICE_NAME             0x04
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
    uint8_t device_type;
    uint8_t data[4];
} ble_mfg_adv_data_t;

typedef struct {
    uint8_t command;
    uint8_t data_size;
    union {
        uint8_t rgb_colour[3];
    } data;
} ble_cmd_request_t;
/******************************************************************************/

/* GLOBALS ********************************************************************/
/******************************************************************************/

/* PROTOTYPES *****************************************************************/
void ble_init();
/******************************************************************************/

#endif /* #ifndef FILE_H */
