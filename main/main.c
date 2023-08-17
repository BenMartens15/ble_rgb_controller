// #include "ble.h"
#include "ble_core.h"
#include "led.h"
#include "rgb_controller.h"

void app_main(void)
{
    ble_init();
    led_init();
    rgb_control_pwm_init();
    
    return;
}
