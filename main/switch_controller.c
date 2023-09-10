
/* INCLUDES *******************************************************************/
#include "ble_core.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "switch_controller.h"
/******************************************************************************/

/* DEFINES ********************************************************************/
#define SWITCH_CONTROLLER_TAG           "SWITCH_CONTROLLER"
#define SWITCH_CONTROLLER_LOG_LEVEL     ESP_LOG_INFO

#define SERVO_MIN_PULSEWIDTH_US         500 // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US         2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE                -180 // Minimum angle
#define SERVO_MAX_DEGREE                180 // Maximum angle

#define SERVO_PIN                       13 // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ    1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD           20000 // 20000 ticks, 20ms

#define NEUTRAL_POS                     90
#define PUSH_SWITCH_POS                 40

#define PIR_INT_PIN                     14
#define PIR_INT_PIN_MASK                (1 << PIR_INT_PIN)

#define SECONDS_TO_MICROSECONDS(s)      (s * 1000000)
/******************************************************************************/

/* ENUMS **********************************************************************/
/******************************************************************************/

/* STRUCTURES *****************************************************************/
/******************************************************************************/

/* GLOBALS ********************************************************************/
mcpwm_cmpr_handle_t m_comparator = NULL;
on_off_state_e m_switch_state = OFF;
on_off_state_e m_pir_state = OFF;
esp_timer_handle_t m_motion_timer;
uint16_t m_motion_timeout_period = 120;

static QueueHandle_t gpio_evt_queue = NULL;
/******************************************************************************/

/* PROTOTYPES *****************************************************************/
static void motion_timeout(void * unused);
static inline uint32_t example_angle_to_compare(uint8_t angle);
static void pir_sensor_task(void* arg);
static void pir_sensor_interrupt_handler(void * arg);
/******************************************************************************/

/* PUBLIC FUNCTIONS ***********************************************************/
void switch_control_init(void)
{
    esp_log_level_set(SWITCH_CONTROLLER_TAG, SWITCH_CONTROLLER_LOG_LEVEL);

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

    // initialize motion timer     
    const esp_timer_create_args_t motion_timer_args = {
        .callback = &motion_timeout,
        .name = "motion_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&motion_timer_args, &m_motion_timer));
}

void switch_control_pir_init(void)
{
    gpio_config_t pin_cfg = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = PIR_INT_PIN_MASK,
        .pull_up_en = 0
    };

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(pir_sensor_task, "pir_sensor_task", 2048, NULL, 10, NULL);

    gpio_config(&pin_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIR_INT_PIN, pir_sensor_interrupt_handler, (void *)PIR_INT_PIN);

    m_pir_state = ON;
}

void switch_control_set_switch_state(on_off_state_e state)
{
    uint8_t servo_angle = 0;

    m_switch_state = state;
    ESP_LOGI(SWITCH_CONTROLLER_TAG, "Setting switch position: %d", state);
    if (state == ON) {
        servo_angle = NEUTRAL_POS + PUSH_SWITCH_POS;
        // start the motion timer
        if (m_pir_state == ON) {
            ESP_ERROR_CHECK(esp_timer_start_periodic(m_motion_timer, SECONDS_TO_MICROSECONDS(m_motion_timeout_period)));
        }
    } else {
        servo_angle = NEUTRAL_POS - PUSH_SWITCH_POS;
        // stop the motion timer
        if (m_pir_state == ON) {
            ESP_ERROR_CHECK(esp_timer_stop(m_motion_timer));
        }
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m_comparator, example_angle_to_compare(servo_angle)));
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m_comparator, example_angle_to_compare(NEUTRAL_POS)));

    ble_update_adv_data();
}

void switch_control_set_pir_state(on_off_state_e state)
{
    m_pir_state = state;
    if (state == ON) {
        ESP_LOGI(SWITCH_CONTROLLER_TAG, "Enabling PIR sensor");
        gpio_intr_enable(PIR_INT_PIN);
    } else {
        ESP_LOGI(SWITCH_CONTROLLER_TAG, "Disabling PIR sensor");
        gpio_intr_disable(PIR_INT_PIN);
        if (esp_timer_is_active(m_motion_timer) == true) {
            ESP_ERROR_CHECK(esp_timer_stop(m_motion_timer));
        }
    }

    ble_update_adv_data();
}

on_off_state_e switch_control_get_switch_state(void)
{
    return m_switch_state;
}

on_off_state_e switch_control_get_pir_state(void)
{
    return m_pir_state;
}


void switch_control_set_motion_timeout(uint16_t timeout)
{
    ESP_LOGI(SWITCH_CONTROLLER_TAG, "Setting motion timeout to %d seconds", timeout);
    m_motion_timeout_period = timeout;
}
/******************************************************************************/ 

/* PRIVATE FUNCTIONS **********************************************************/
static void motion_timeout(void * unused)
{
    if (m_switch_state == ON) {
        ESP_LOGI(SWITCH_CONTROLLER_TAG, "No motion detected in the last %d seconds. Turning switch off.", m_motion_timeout_period);
        switch_control_set_switch_state(OFF);
    }
}

static inline uint32_t example_angle_to_compare(uint8_t angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

static void pir_sensor_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // reset the motion timer every time movement is detected
            if (esp_timer_is_active(m_motion_timer) == true) {
                ESP_ERROR_CHECK(esp_timer_restart(m_motion_timer, SECONDS_TO_MICROSECONDS(m_motion_timeout_period)));
                ESP_LOGD(SWITCH_CONTROLLER_TAG, "Motion detected - timer reset");
            }

            if (m_switch_state == OFF) {
                ESP_LOGI(SWITCH_CONTROLLER_TAG, "Motion detected - turning switch on");
                switch_control_set_switch_state(ON);
            }
        }
    }
}

static void IRAM_ATTR pir_sensor_interrupt_handler(void * arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
/******************************************************************************/
