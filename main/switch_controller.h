
#ifndef SWITCH_CONTROLLER_H
#define SWITCH_CONTROLLER_H

/* INCLUDES *******************************************************************/
#include <stdint.h>
/******************************************************************************/

/* DEFINES ********************************************************************/
/******************************************************************************/

/* ENUMS **********************************************************************/
typedef enum {
    OFF,
    ON
} on_off_state_e;
/******************************************************************************/

/* STRUCTURES *****************************************************************/
/******************************************************************************/

/* GLOBALS ********************************************************************/
/******************************************************************************/

/* PROTOTYPES *****************************************************************/
void switch_control_init(void);
void switch_control_pir_init(void);
void switch_control_set_switch_state(on_off_state_e state);
void switch_control_set_pir_state(on_off_state_e state);
on_off_state_e switch_control_get_switch_state(void);
on_off_state_e switch_control_get_pir_state(void);
void switch_control_set_motion_timeout(uint16_t timeout);
/******************************************************************************/

#endif /* #ifndef SWITCH_CONTROLLER_H */
