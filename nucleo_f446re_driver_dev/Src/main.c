#include "stm32f446xx.h"
#include "stm32f446xx.gpio.h"

void delay() {
    int i = 0;
    for (i = 0; i < 500000; i++) {

    }
 }

int main() {
    GPIO_HANDLE_t handle = {0};
    handle.gpiox = GPIOA;
    handle.config.pin_number = GPIO_PIN_5;
    handle.config.pin_output_type = GPIO_PORT_OUT_PUSH_PULL;
    handle.config.pin_mode = GPIO_PIN_MODE_OUTPUT;
    handle.config.pin_pull_up_down_cntrl = GPIO_PUPDR_NONE;
    handle.config.pin_speed = GPIO_PIN_SPEED_FAST;

    GPIO_clock_control(handle.gpiox, 1);
    GPIO_init(&handle);

    GPIO_HANDLE_t handle2 = {0};
    handle2.gpiox = GPIOC;
    handle2.config.pin_number = GPIO_PIN_13;
    handle2.config.pin_mode = GPIO_PIN_MODE_INPUT;
    handle2.config.pin_pull_up_down_cntrl = GPIO_PUPDR_NONE;
    handle2.config.pin_speed = GPIO_PIN_SPEED_FAST;

    GPIO_clock_control(handle2.gpiox, 1U);
    GPIO_init(&handle2);

    while(true) {

        if(GPIO_read_from_input_pin(handle2.gpiox, GPIO_PIN_13) == 0) {
            GPIO_toggle_output_pin(handle.gpiox, GPIO_PIN_5);
            delay();
        }
            
    }

    return 0;
    
}

void SystemInit() {

}