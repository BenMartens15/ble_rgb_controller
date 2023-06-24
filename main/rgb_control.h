
#ifndef PWM_H
#define PWM_H

/* DEFINES ********************************************************************/
#define PWM_TAG                 "PWM"

#define RED_CTRL_PIN            14
#define GREEN_CTRL_PIN          12
#define BLUE_CTRL_PIN           13
#define PWM_CHANNEL             0

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 8 bits
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz
/******************************************************************************/


/* STRUCTURES *****************************************************************/
/******************************************************************************/


/* PROTOTYPES *****************************************************************/
void rgb_control_pwm_init();
void rgb_control_set_colour(uint8_t * value);
/******************************************************************************/

#endif /* #ifndef PWM_H */