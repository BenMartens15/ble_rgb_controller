
#ifndef PWM_H
#define PWM_H

/* INCLUDES *******************************************************************/
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
/******************************************************************************/


/* DEFINES ********************************************************************/
#define PWM_TAG     "PWM"

#define PWM_PIN1    14
#define PWM_PIN2    12
#define PWM_PIN3    13
#define PWM_CHANNEL 0

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz
/******************************************************************************/


/* STRUCTURES *****************************************************************/
/******************************************************************************/


/* PROTOTYPES *****************************************************************/
void pwm_init();
void set_color(uint8_t * value, uint16_t len);
/******************************************************************************/

#endif /* #ifndef PWM_H */