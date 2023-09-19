// #include "ble.h"
#include "ble_core.h"
#include "led.h"
#include "rgb_controller.h"
#include "switch_controller.h"

void app_main(void)
{
    ble_init();
    led_init();
#if (CONFIG_DEVICE_TYPE == DEVICE_TYPE_RGB_CONTROLLER)
    rgb_control_pwm_init();
#elif (CONFIG_DEVICE_TYPE == DEVICE_TYPE_SWITCH_CONTROLLER)
    switch_control_init();
    switch_control_pir_init();
#endif
    
    return;
}
