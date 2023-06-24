#include "ble.h"
#include "led.h"
#include "rgb_control.h"

void app_main(void)
{
    ble_init();
    led_init();
    rgb_control_pwm_init();
    
    return;
}
