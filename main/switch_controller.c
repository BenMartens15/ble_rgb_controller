
/* INCLUDES *******************************************************************/
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "switch_controller.h"
/******************************************************************************/

/* DEFINES ********************************************************************/
#define SWITCH_CONTROLLER_TAG           "SWITCH_CONTROLLER"

#define SERVO_MIN_PULSEWIDTH_US         500 // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US         2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE                -180 // Minimum angle
#define SERVO_MAX_DEGREE                180 // Maximum angle

#define SERVO_PIN                       13 // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ    1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD           20000 // 20000 ticks, 20ms

#define NEUTRAL_POS                     90
#define PUSH_SWITCH_POS                 40
/******************************************************************************/

/* ENUMS **********************************************************************/
/******************************************************************************/

/* STRUCTURES *****************************************************************/
/******************************************************************************/

/* GLOBALS ********************************************************************/
mcpwm_cmpr_handle_t m_comparator = NULL;
/******************************************************************************/

/* PROTOTYPES *****************************************************************/
static inline uint32_t example_angle_to_compare(uint8_t angle);
/******************************************************************************/

/* PUBLIC FUNCTIONS ***********************************************************/
void switch_control_init(void) {
    ESP_LOGI(SWITCH_CONTROLLER_TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(SWITCH_CONTROLLER_TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(SWITCH_CONTROLLER_TAG, "Create comparator and generator from the operator");
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &m_comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PIN,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m_comparator, example_angle_to_compare(NEUTRAL_POS)));

    ESP_LOGI(SWITCH_CONTROLLER_TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m_comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(SWITCH_CONTROLLER_TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void switch_control_set_state(switch_state state) {
    uint8_t servo_angle = 0;

    ESP_LOGI(SWITCH_CONTROLLER_TAG, "Setting switch position: %d", state);
    if (state == ON) {
        servo_angle = NEUTRAL_POS + PUSH_SWITCH_POS;
    } else {
        servo_angle = NEUTRAL_POS - PUSH_SWITCH_POS;
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m_comparator, example_angle_to_compare(servo_angle)));
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m_comparator, example_angle_to_compare(NEUTRAL_POS)));
}
/******************************************************************************/ 

/* PRIVATE FUNCTIONS **********************************************************/
static inline uint32_t example_angle_to_compare(uint8_t angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}
/******************************************************************************/
