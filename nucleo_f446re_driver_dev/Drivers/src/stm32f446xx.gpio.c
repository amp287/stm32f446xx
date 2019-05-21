#include "stm32f446xx.gpio.h"

void GPIO_init(GPIO_HANDLE_t *handle) {
    GPIO_CONFIG_t config = handle->config;
    GPIO_t *gpio = handle->gpiox;

    switch (config.pin_mode) {
        case PIN_MODE_INPUT:
        case PIN_MODE_ALT:
        case PIN_MODE_ANALOG:
        case PIN_MODE_OUTPUT:
            gpio->MODER |= (config.pin_mode << (config.pin_number * 2));
    }

    switch (config.pin_output_type) {
        case PORT_OUT_OPEN_DRAIN:
        case PORT_OUT_PUSH_PULL:
            gpio->OTYPER |= (config.pin_output_type << config.pin_number);
    }
  
    switch (config.pin_speed) {
        case PIN_SPEED_LOW:
        case PIN_SPEED_MED:
        case PIN_SPEED_FAST:
        case PIN_SPEED_HIGH:
            gpio->OSPEEDER |= (config.pin_speed << (config.pin_number * 2));
    }

    switch (config.pin_pull_up_down_cntrl) {
        case PUPDR_NONE:
        case PUPDR_UP:
        case PUPDR_DOWN:
            gpio->PUPDR |= (config.pin_pull_up_down_cntrl << (config.pin_number * 2));
    }
    
}

void GPIO_deinit(GPIO_t *gpio) {

}

void GPIO_clock_control(GPIO_t *gpio, bool enable) {
    int bit_position = 0;

    if(gpio == GPIOA) bit_position = 0;
    else if(gpio == GPIOB) bit_position = 1;
    else if(gpio == GPIOC) bit_position = 2;
    else if(gpio == GPIOD) bit_position = 3;
    else if(gpio == GPIOE) bit_position = 4;
    else if(gpio == GPIOF) bit_position = 5;
    else if(gpio == GPIOG) bit_position = 6;
    else if(gpio == GPIOH) bit_position = 7;

    if(enable) {
        RCC->AHB1_ENR |= (1 << bit_position);
    } else {
        RCC->AHB1_ENR &= ~(1 << bit_position);
    }
}

uint8_t GPIO_read_from_input_pin(GPIO_t *gpio, uint8_t pin) {
    return gpio->IDR & (1 << pin);
}

uint16_t GPIO_read_from_input_port(GPIO_t *gpio) {
    return gpio->ODR;
}

void GPIO_write_to_output_pin(GPIO_t *gpio, uint8_t pin, bool value) {

}
 
void GPIO_write_to_output_port(GPIO_t *gpio, uint16_t value) {

}

void GPIO_toggle_output_pin(GPIO_t *gpio, uint8_t pin) {

}

void GPIO_irq_config(uint8_t irq_number, uint8_t irq_priority, bool enable) {

}

void GPIO_irq_handling(uint8_t pin) {

}