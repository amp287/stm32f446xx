#ifndef INC_STM32F446XX_GPIO
#define INC_STM32F446XX_GPIO

#include "stm32f446xx.h"
#include <stdbool.h>

/*
* @GPIO_PIN_MODES
*/
#define GPIO_PIN_MODE_INPUT  0
#define GPIO_PIN_MODE_OUTPUT 1
#define GPIO_PIN_MODE_ALT    2
#define GPIO_PIN_MODE_ANALOG 3

#define GPIO_PORT_OUT_PUSH_PULL  0
#define GPIO_PORT_OUT_OPEN_DRAIN 1

#define GPIO_PIN_SPEED_LOW  0
#define GPIO_PIN_SPEED_MED  1
#define GPIO_PIN_SPEED_FAST 2
#define GPIO_PIN_SPEED_HIGH 3

// Pull up / pull down register control values
#define GPIO_PUPDR_NONE  0
#define GPIO_PUPDR_UP    1
#define GPIO_PUPDR_DOWN  2

/* @GPIO_ALT_FUNC_MODES
* Alternate function options ()
*/
#define GPIO_ALT_FN_0    0
#define GPIO_ALT_FN_1    1
#define GPIO_ALT_FN_2    2
#define GPIO_ALT_FN_3    3
#define GPIO_ALT_FN_4    4
#define GPIO_ALT_FN_5    5
#define GPIO_ALT_FN_6    6
#define GPIO_ALT_FN_7    7
#define GPIO_ALT_FN_8    8
#define GPIO_ALT_FN_9    9
#define GPIO_ALT_FN_10   10
#define GPIO_ALT_FN_11   11
#define GPIO_ALT_FN_12   12
#define GPIO_ALT_FN_13   13
#define GPIO_ALT_FN_14   14
#define GPIO_ALT_FN_15   15

/* @GPIO_PIN_NUMS
 */

#define GPIO_PIN_0  0
#define GPIO_PIN_1  1
#define GPIO_PIN_2  2
#define GPIO_PIN_3  3
#define GPIO_PIN_4  4
#define GPIO_PIN_5  5
#define GPIO_PIN_6  6
#define GPIO_PIN_7  7
#define GPIO_PIN_8  8
#define GPIO_PIN_9  9
#define GPIO_PIN_10 10
#define GPIO_PIN_11 11
#define GPIO_PIN_12 12
#define GPIO_PIN_13 13
#define GPIO_PIN_14 14
#define GPIO_PIN_15 15


typedef struct {
    // possible values from @GPIO_PIN_NUMS
    uint8_t pin_number;
    // possible values from @GPIO_PIN_MODES
    uint8_t pin_mode;
    uint8_t pin_speed;
    uint8_t pin_pull_up_down_cntrl;
    uint8_t pin_output_type;
    // possible values from @GPIO_ALT_FUNC_MODES
    uint8_t pin_alt_mode;
} GPIO_CONFIG_t;

typedef struct {
    GPIO_t *gpiox;
    GPIO_CONFIG_t config;
} GPIO_HANDLE_t;

void GPIO_init(GPIO_HANDLE_t *handle);
void GPIO_deinit(GPIO_t *gpio);

/*
 *  @fn         GPIO_clock_control
 *  
 *  @brief      Enables or disables clock for a given gpio
 * 
 *  @param[in]  gpio reference
 *  @param[in]  true to enable, false to disable
 * 
 *  @return     none  
 */
void GPIO_clock_control(GPIO_t *gpio, bool enable);

/*
 *  @fn         GPIO_read_from_input_pin
 *  
 *  @brief      Read from a single gpio pin
 * 
 *  @param[in]  gpio reference
 *  @param[in]  pin number to read
 * 
 *  @return     pin value
 */
uint8_t GPIO_read_from_input_pin(GPIO_t *gpio, uint8_t pin);

/*
 *  @fn         GPIO_read_from_input_port
 *  
 *  @brief      Read from a gpio port (16 pins)
 * 
 *  @param[in]  gpio reference
 *  
 *  @return     port value
 */
uint16_t GPIO_read_from_input_port(GPIO_t *gpio);

/*
 *  @fn         GPIO_write_to_output_pin
 *  
 *  @brief      Write value to pin 
 * 
 *  @param[in]  gpio reference
 *  @param[in]  pin number
 *  @param[in]  value to write
 *  
 *  @return     none
 */
void GPIO_write_to_output_pin(GPIO_t *gpio, uint8_t pin, bool value);

/*
 *  @fn         GPIO_write_to_output_port
 *  
 *  @brief      Write value to port 
 * 
 *  @param[in]  gpio reference
 *  @param[in]  value to write
 *  
 *  @return     none
 */
void GPIO_write_to_output_port(GPIO_t *gpio, uint16_t value);

/*
 *  @fn         GPIO_toggle_output_pin
 *  
 *  @brief      Toggle pin value 
 * 
 *  @param[in]  gpio reference
 *  @param[in]  pin number
 *  
 *  @return     none
 */
void GPIO_toggle_output_pin(GPIO_t *gpio, uint8_t pin);

/*
 *  @fn         GPIO_irq_config
 *  
 *  @brief      enable or disable irq 
 * 
 *  @param[in]  irq number
 *  @param[in]  irq priority
 *  @param[in]  true or false
 *  
 *  @return     none
 */
void GPIO_irq_config(uint8_t irq_number, uint8_t irq_priority, bool enable);
void GPIO_irq_handling(uint8_t pin);

#endif