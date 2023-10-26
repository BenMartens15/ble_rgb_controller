
#ifndef SWITCH_CONTROLLER_H
#define SWITCH_CONTROLLER_H

/* INCLUDES *******************************************************************/
#include <stdint.h>
/******************************************************************************/

/* DEFINES ********************************************************************/
#define DEVICE_TYPE_SWITCH_CONTROLLER 1
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
#if (CONFIG_DEVICE_TYPE == DEVICE_TYPE_SWITCH_CONTROLLER)
    void switch_control_init(void);
    void switch_control_pir_init(void);
    void switch_control_set_switch_state(on_off_state_e state);
    void switch_control_set_pir_state(on_off_state_e state);
    on_off_state_e switch_control_get_switch_state(void);
    on_off_state_e switch_control_get_pir_state(void);
    void switch_control_set_motion_timeout(uint16_t timeout);
#else
    static inline void switch_control_init(void) { return; }
    static inline void switch_control_pir_init(void) { return; };
    static inline void switch_control_set_switch_state(on_off_state_e state) { return; };
    static inline void switch_control_set_pir_state(on_off_state_e state) { return; };
    static inline on_off_state_e switch_control_get_switch_state(void) { return OFF; };
    static inline on_off_state_e switch_control_get_pir_state(void) { return OFF; };
    static inline void switch_control_set_motion_timeout(uint16_t timeout) { return; };
#endif
/******************************************************************************/

#endif /* #ifndef SWITCH_CONTROLLER_H */
