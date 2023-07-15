
#ifndef PWM_H
#define PWM_H

/* INCLUDES *******************************************************************/
#include <stdint.h>
/******************************************************************************/

/* DEFINES ********************************************************************/
/******************************************************************************/

/* ENUMS **********************************************************************/
/******************************************************************************/

/* STRUCTURES *****************************************************************/
/******************************************************************************/

/* GLOBALS ********************************************************************/
/******************************************************************************/

/* PROTOTYPES *****************************************************************/
void rgb_control_pwm_init();
void rgb_control_set_colour(uint8_t * value);
/******************************************************************************/

#endif /* #ifndef PWM_H */
