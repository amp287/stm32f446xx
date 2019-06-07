#include "stm32f446xx.gpio.h"

void GPIO_init(GPIO_HANDLE_t *handle) {
    GPIO_CONFIG_t config = handle->config;
    GPIO_t *gpio = handle->gpiox;

    switch (config.pin_mode) {
        case GPIO_PIN_MODE_INPUT:
        case GPIO_PIN_MODE_ALT:
        case GPIO_PIN_MODE_ANALOG:
        case GPIO_PIN_MODE_OUTPUT:
            gpio->MODER &= ~(3U << (config.pin_number * 2U));
            gpio->MODER |= (config.pin_mode << (config.pin_number * 2U));;
    }

    switch (config.pin_output_type) {
        case GPIO_PORT_OUT_OPEN_DRAIN:
        case GPIO_PORT_OUT_PUSH_PULL:
            gpio->OTYPER &= ~(3U << (config.pin_number * 2U));
            gpio->OTYPER |= (config.pin_output_type << config.pin_number);
    }
  
    switch (config.pin_speed) {
        case GPIO_PIN_SPEED_LOW:
        case GPIO_PIN_SPEED_MED:
        case GPIO_PIN_SPEED_FAST:
        case GPIO_PIN_SPEED_HIGH:
            gpio->OSPEEDER &= ~(3U << (config.pin_number * 2U));
            gpio->OSPEEDER |= (config.pin_speed << (config.pin_number * 2U));
    }

    switch (config.pin_pull_up_down_cntrl) {
        case GPIO_PUPDR_NONE:
        case GPIO_PUPDR_UP:
        case GPIO_PUPDR_DOWN:
            gpio->PUPDR &= ~(3U << (config.pin_number * 2U));
            gpio->PUPDR |= (config.pin_pull_up_down_cntrl << (config.pin_number * 2U));
    }
    
    if (config.pin_mode == GPIO_PIN_MODE_ALT) {
        if(config.pin_alt_mode >= GPIO_ALT_FN_0 && config.pin_alt_mode <= GPIO_ALT_FN_15) {
            if(config.pin_number < 8U) {
                gpio->AFR[0] &= ~(0xFU << (config.pin_number * 4U));
                gpio->AFR[0] |= (config.pin_alt_mode << (config.pin_number * 4U));
            } else {
                gpio->AFR[1] &= ~(0xFU << (config.pin_number - 8U * 4U));
                gpio->AFR[1] = (config.pin_alt_mode << ((config.pin_number - 8U) * 4U));
            }
        }
    }
}

void GPIO_deinit(GPIO_t *gpio) {
    
}

void GPIO_clock_control(GPIO_t *gpio, bool enable) {
    int bit_position = 0;

    if(gpio == GPIOA) bit_position = 0U;
    else if(gpio == GPIOB) bit_position = 1U;
    else if(gpio == GPIOC) bit_position = 2U;
    else if(gpio == GPIOD) bit_position = 3U;
    else if(gpio == GPIOE) bit_position = 4U;
    else if(gpio == GPIOF) bit_position = 5U;
    else if(gpio == GPIOG) bit_position = 6U;
    else if(gpio == GPIOH) bit_position = 7U;

    if(enable) {
        RCC->AHB1_ENR |= (1U << bit_position);
    } else {
        RCC->AHB1_ENR &= ~(1U << bit_position);
    }
}

uint8_t GPIO_read_from_input_pin(GPIO_t *gpio, uint8_t pin) {
    if((gpio->IDR & (1U << pin)) != 0)
        return 1;
    else
        return 0;
}

uint16_t GPIO_read_from_input_port(GPIO_t *gpio) {
    return gpio->ODR;
}

void GPIO_write_to_output_pin(GPIO_t *gpio, uint8_t pin, bool value) {
    gpio->ODR &= ~(1U << pin);
    gpio->ODR |= (value << pin);
}

void GPIO_write_to_output_pin_atomic(GPIO_t *gpio, uint8_t pin, bool value) {
    uint8_t offset = 0;

    if(value == 0) offset = 16U;

    gpio->BSRR = 1 << (pin + offset);
}
 
void GPIO_write_to_output_port(GPIO_t *gpio, uint16_t value) {
    gpio->ODR = value;
}

void GPIO_toggle_output_pin(GPIO_t *gpio, uint8_t pin) {
    gpio->ODR ^= (1U << pin);
}

void GPIO_irq_config(uint8_t irq_number, uint8_t irq_priority, bool enable) {

}

void GPIO_irq_handling(uint8_t pin) {
    
}