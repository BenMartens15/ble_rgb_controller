
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
} switch_state_e;
/******************************************************************************/

/* STRUCTURES *****************************************************************/
/******************************************************************************/

/* GLOBALS ********************************************************************/
/******************************************************************************/

/* PROTOTYPES *****************************************************************/
void switch_control_init(void);
void switch_control_intr_init(void);
void switch_control_set_state(switch_state_e state);
switch_state_e switch_control_get_state(void);
void switch_control_set_motion_timeout(uint16_t timeout);
/******************************************************************************/

#endif /* #ifndef SWITCH_CONTROLLER_H */
