// Header file defining registers and memory location of the STM32f446XX
#ifndef INC_STM32F446XX
#define INC_STM32F446XX

#include <stdint.h>

#define __vo volatile

#define FLASH_BASE_ADDR 0x08000000U
#define SRAM1_BASE_ADDR 0x20000000U

#define SRAM2_BASE_ADDR 0x0201C000U

// Defined as System Memory in Reference Manual pg.65
#define ROM_BASE_ADDR           0x1FFF0000U

#define OTP_BASE_ADDR           0x1FFF7800U

// Buses 

#define APB1_BASE_ADDR          0x40000000U
#define APB2_BASE_ADDR          0x40010000U
#define AHB1_BASE_ADDR          0x40020000U
#define AHB2_BASE_ADDR          0x50000000U
#define AHB3_BASE_ADDR          0x60000000U

/* AHB1 Perpherials START */

#define GPIOA_BASE_ADDR         (AHB1_BASE_ADDR + 0x0000U)
#define GPIOB_BASE_ADDR         (AHB1_BASE_ADDR + 0x0400U)
#define GPIOC_BASE_ADDR         (AHB1_BASE_ADDR + 0x0800U)
#define GPIOD_BASE_ADDR         (AHB1_BASE_ADDR + 0x0C00U)
#define GPIOE_BASE_ADDR         (AHB1_BASE_ADDR + 0x1000U)
#define GPIOF_BASE_ADDR         (AHB1_BASE_ADDR + 0x1400U)
#define GPIOG_BASE_ADDR         (AHB1_BASE_ADDR + 0x1800U)
#define GPIOH_BASE_ADDR         (AHB1_BASE_ADDR + 0x1C00U)
#define CRC_BASE_ADDR           (AHB1_BASE_ADDR + 0x3800U)
#define RCC_BASE_ADDR           (AHB1_BASE_ADDR + 0x3800U)

/* AHB1 Perpherials END */

/* APB1 Perpherials START */

#define TIM2_BASE_ADDR          (APB1_BASE_ADDR + 0x0000U)
#define TIM3_BASE_ADDR          (APB1_BASE_ADDR + 0x0400U)
#define TIM4_BASE_ADDR          (APB1_BASE_ADDR + 0x0800U)
#define TIM5_BASE_ADDR          (APB1_BASE_ADDR + 0x0C00U)
#define TIM6_BASE_ADDR          (APB1_BASE_ADDR + 0x1000U)
#define TIM7_BASE_ADDR          (APB1_BASE_ADDR + 0x1400U)
#define TIM12_BASE_ADDR         (APB1_BASE_ADDR + 0x1800U)
#define TIM13_BASE_ADDR         (APB1_BASE_ADDR + 0x1C00U)
#define TIM14_BASE_ADDR         (APB1_BASE_ADDR + 0x2000U)

#define RTC_BKP_BASE_ADDR       (APB1_BASE_ADDR + 0x2800U)

// Descrptive name
#define WWDG_BASE_ADDR          (APB1_BASE_ADDR + 0x2C00U)
#define IWDG_BASE_ADDR          (APB1_BASE_ADDR + 0x3000U)

#define SPI2_I2S2_BASE_ADDR     (APB1_BASE_ADDR + 0x3800U)
#define SPI3_I2S3_BASE_ADDR     (APB1_BASE_ADDR + 0x3C00U)

#define SPDIF_RX_BASE_ADDR      (APB1_BASE_ADDR + 0x4000U)

#define USART2_BASE_ADDR        (APB1_BASE_ADDR + 0x4400U)
#define USART3_BASE_ADDR        (APB1_BASE_ADDR + 0x4800U)

#define UART4_BASE_ADDR         (APB1_BASE_ADDR + 0x4C00U)
#define UART5_BASE_ADDR         (APB1_BASE_ADDR + 0x5000U)

#define I2C1_BASE_ADDR          (APB1_BASE_ADDR + 0x5400U)
#define I2C2_BASE_ADDR          (APB1_BASE_ADDR + 0x5800U)
#define I2C3_BASE_ADDR          (APB1_BASE_ADDR + 0x5C00U)

#define CAN1_BASE_ADDR          (APB1_BASE_ADDR + 0x6400U)
#define CAN2_BASE_ADDR          (APB1_BASE_ADDR + 0x6800U)

#define HDMI_CEC_BASE_ADDR      (APB1_BASE_ADDR + 0x6C00U)

#define PWR_BASE_ADDR           (APB1_BASE_ADDR + 0x7000U)

#define DAC_BASE_ADDR           (APB1_BASE_ADDR + 0x7400U)

/* APB1 Perpherials END */

/* APB2 Perpherials START */

#define TIM1_BASE_ADDR          (APB2_BASE_ADDR + 0x0000U)
#define TIM8_BASE_ADDR          (APB2_BASE_ADDR + 0x0400U)

#define USART1_BASE_ADDR        (APB2_BASE_ADDR + 0x1000U)
#define USART6_BASE_ADDR        (APB2_BASE_ADDR + 0x1400U)

#define ADC1_2_3_BASE_ADDR      (APB2_BASE_ADDR + 0x2000U)

#define SDMMC_BASE_ADDR         (APB2_BASE_ADDR + 0x2C00U)

#define SPI1_BASE_ADDR          (APB2_BASE_ADDR + 0x3000U)
#define SPI4_BASE_ADDR          (APB2_BASE_ADDR + 0x3400U)

#define SYSCFG_BASE_ADDR        (APB2_BASE_ADDR + 0x3800U)

// External Interrupt Controller base address
#define EXTI_BASE_ADDR          (APB2_BASE_ADDR + 0x3800U)

#define TIM9_BASE_ADDR          (APB2_BASE_ADDR + 0x4000U)
#define TIM10_BASE_ADDR         (APB2_BASE_ADDR + 0x4400U)
#define TIM11_BASE_ADDR         (APB2_BASE_ADDR + 0x4800U)

#define SAI1_BASE_ADDR          (APB2_BASE_ADDR + 0x5800U)
#define SAI2_BASE_ADDR          (APB2_BASE_ADDR + 0x5C00U)

/* APB2 Perpherials END */

/* GPIO START */
typedef struct GPIO_t {
    /* GPIO port mode register (pg 187). 00: Input (reset state)
    01: General purpose output mode 10: Alternate function mode 11: Analog mode */
    __vo uint32_t MODER;
    __vo /* GPIO port output type register (pg 188) 0: Output push-pull (reset state) 1: Output open-drain */
    __vo uint32_t OTYPER;
    __vo /* Port output speed resgister */
    __vo uint32_t OSPEEDER;
    __vo /* Port pull-up/pull-down register */
    __vo uint32_t PUPDR;
    __vo /* Port input data register */
    __vo uint32_t IDR;
    __vo /* Port output data register */
    __vo uint32_t ODR;
    __vo /* Port bit set/reset register */
    __vo uint32_t BSRR;
    __vo /* Port configuration lock register */
    __vo uint32_t LCKR;
    __vo /* Alternate function low/high registers (index 0 for low, index 1 for high) */
    __vo uint32_t AFR[2];
} GPIO_t;

#define GPIOA                   ((GPIO_t*)GPIOA_BASE_ADDR)
#define GPIOB                   ((GPIO_t*)GPIOB_BASE_ADDR)
#define GPIOC                   ((GPIO_t*)GPIOC_BASE_ADDR)
#define GPIOD                   ((GPIO_t*)GPIOD_BASE_ADDR)
#define GPIOE                   ((GPIO_t*)GPIOE_BASE_ADDR)
#define GPIOF                   ((GPIO_t*)GPIOF_BASE_ADDR)
#define GPIOG                   ((GPIO_t*)GPIOG_BASE_ADDR)
#define GPIOH                   ((GPIO_t*)GPIOH_BASE_ADDR)

/* GPIO END */

typedef struct RCC_t {
    /* Clock control register */
    __vo uint32_t CR;
    /* PLL configuration register */
    __vo uint32_t PLL_CFGR;
    /* Clock configuration register */
    __vo uint32_t CFGR;
    /* Clock interrupt register */
    __vo uint32_t CIR;
    /* AHB1 peripheral reset register */
    __vo uint32_t AHB1_RSTR;
    /* AHB2 peripheral reset register */
    __vo uint32_t AHB2_RSTR;
    /* AHB3 peripheral reset register */
    __vo uint32_t AHB3_RSTR;
    uint32_t RESERVED0;
    /* APB1 peripheral reset register */
    __vo uint32_t APB1_RSTR;
    /* APB2 peripheral reset register */
    __vo uint32_t APB2_RSTR;
    uint32_t RESERVED1[2];
    /* AHB1 peripheral clock enable register */
    __vo uint32_t AHB1_ENR;
    /* AHB2 peripheral clock enable register */
    __vo uint32_t AHB2_ENR;
     /* AHB3 peripheral clock enable register */
    __vo uint32_t AHB3_ENR;
    uint32_t RESERVED2;
     /* APB1 peripheral clock enable register */
    __vo uint32_t APB1_ENR;
     /* APB2 peripheral clock enable register */
    __vo uint32_t APB2_ENR;
    uint32_t RESERVED3[2];
    /* AHB1 peripheral clock enable in low power mode register */
    __vo uint32_t AHB1_LPENR;
    /* AHB2 peripheral clock enable in low power mode register */
    __vo uint32_t AHB2_LPENR;
    /* AHB3 peripheral clock enable in low power mode register */
    __vo uint32_t AHB3_LPENR;
    uint32_t RESERVED4;
    /* APB1 peripheral clock enable in low power mode register */
    __vo uint32_t APB1_LPENR;
    /* APB2 peripheral clock enable in low power mode register */
    __vo uint32_t APB2_LPENR;
    uint32_t RESERVED5[2];
    /* Backup domain control register */
    __vo uint32_t BDCR;
    /* Clock control & status register */
    __vo uint32_t CSR;
    uint32_t RESERVED6[2];
    /* Spread spectrum clock generation register */
    __vo uint32_t SS_CGR;
    /* PLLI2S configuration register */
    __vo uint32_t PLLI2_SCFGR;
    /* PLL configuration register */
    __vo uint32_t PLL_SAI_CFGR;
    /* Dedicated Clock Configuration Register */
    __vo uint32_t DCK_CFGR;
    /* Clocks gated enable register */
    __vo uint32_t CK_GATENR;
    /* Dedicated clocks configuration register 2 */
    __vo uint32_t DCK_CFGR2;

} RCC_t;

#define RCC                     ((RCC_t*)RCC_BASE_ADDR)
/* Clock Enable/Disable Macros START */
#define GPIOA_CLOCK_ENABLE()    (RCC->AHB1_ENR |= (1 << 0))
#define GPIOB_CLOCK_ENABLE()    (RCC->AHB1_ENR |= (1 << 1))
#define GPIOC_CLOCK_ENABLE()    (RCC->AHB1_ENR |= (1 << 2))
#define GPIOD_CLOCK_ENABLE()    (RCC->AHB1_ENR |= (1 << 3))
#define GPIOE_CLOCK_ENABLE()    (RCC->AHB1_ENR |= (1 << 4))
#define GPIOF_CLOCK_ENABLE()    (RCC->AHB1_ENR |= (1 << 5))
#define GPIOG_CLOCK_ENABLE()    (RCC->AHB1_ENR |= (1 << 6))
#define GPIOH_CLOCK_ENABLE()    (RCC->AHB1_ENR |= (1 << 7))
#define GPIOA_CLOCK_DISABLE()   (RCC->AHB1_ENR &= ~(1 << 0))
#define GPIOB_CLOCK_DISABLE()   (RCC->AHB1_ENR &= ~(1 << 1))  
#define GPIOC_CLOCK_DISABLE()   (RCC->AHB1_ENR &= ~(1 << 2))  
#define GPIOD_CLOCK_DISABLE()   (RCC->AHB1_ENR &= ~(1 << 3))  
#define GPIOE_CLOCK_DISABLE()   (RCC->AHB1_ENR &= ~(1 << 4))  
#define GPIOF_CLOCK_DISABLE()   (RCC->AHB1_ENR &= ~(1 << 5))  
#define GPIOG_CLOCK_DISABLE()   (RCC->AHB1_ENR &= ~(1 << 6))  
#define GPIOH_CLOCK_DISABLE()   (RCC->AHB1_ENR &= ~(1 << 7)) 

#define SPI1_CLOCK_ENABLE()     (RCC->APB2_ENR |= (1 << 12))
#define SPI2_CLOCK_ENABLE()     (RCC->APB1_ENR |= (1 << 14))
#define SPI3_CLOCK_ENABLE()     (RCC->APB1_ENR |= (1 << 15))
#define SPI4_CLOCK_ENABLE()     (RCC->APB2_ENR |= (1 << 13))
#define SPI1_CLOCK_DISABLE()    (RCC->APB2_ENR &= ~(1 << 12))
#define SPI2_CLOCK_DISABLE()    (RCC->APB1_ENR &= ~(1 << 14))
#define SPI3_CLOCK_DISABLE()    (RCC->APB1_ENR &= ~(1 << 15))
#define SPI4_CLOCK_DISABLE()    (RCC->APB2_ENR &= ~(1 << 13))

#define UART2_CLOCK_ENABLE()    (RCC->APB1_ENR |= (1 << 17))
#define UART3_CLOCK_ENABLE()    (RCC->APB1_ENR |= (1 << 18))
#define UART4_CLOCK_ENABLE()    (RCC->APB1_ENR |= (1 << 19))
#define UART5_CLOCK_ENABLE()    (RCC->APB1_ENR |= (1 << 20))
#define UART2_CLOCK_DISABLE()   (RCC->APB1_ENR &= ~(1 << 17))
#define UART3_CLOCK_DISABLE()   (RCC->APB1_ENR &= ~(1 << 18))
#define UART4_CLOCK_DISABLE()   (RCC->APB1_ENR &= ~(1 << 19))
#define UART5_CLOCK_DISABLE()   (RCC->APB1_ENR &= ~(1 << 20))

#define I2C1_CLOCK_ENABLE()     (RCC->APB1_ENR |= (1 << 21))
#define I2C2_CLOCK_ENABLE()     (RCC->APB1_ENR |= (1 << 22))
#define I2C3_CLOCK_ENABLE()     (RCC->APB1_ENR |= (1 << 23))
#define I2C1_CLOCK_DISABLE()    (RCC->APB1_ENR &= ~(1 << 21))
#define I2C2_CLOCK_DISABLE()    (RCC->APB1_ENR &= ~(1 << 22))
#define I2C3_CLOCK_DISABLE()    (RCC->APB1_ENR &= ~(1 << 23))

#define USART1_CLOCK_ENABLE()   (RCC->APB2_ENR |= (1 << 4))
#define USART6_CLOCK_ENABLE()   (RCC->APB2_ENR |= (1 << 5))
#define USART1_CLOCK_DISABLE()  (RCC->APB2_ENR &= ~(1 << 4))
#define USART6_CLOCK_DISABLE()  (RCC->APB2_ENR &= ~(1 << 5))

#define SYSCFG_CLOCK_ENABLE()   (RCC->APB2_ENR |= (1 << 14))
#define SYSCFG_CLOCK_DISABLE()  (RCC->APB2_ENR &= ~(1 << 14))

/* Clock Enable/Disable Macros END */
#endif
