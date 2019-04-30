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

#define GPIOA_BASE_ADDR         AHB1_BASE_ADDR + 0x0000U
#define GPIOB_BASE_ADDR         AHB1_BASE_ADDR + 0x0400U
#define GPIOC_BASE_ADDR         AHB1_BASE_ADDR + 0x0800U
#define GPIOD_BASE_ADDR         AHB1_BASE_ADDR + 0x0C00U
#define GPIOE_BASE_ADDR         AHB1_BASE_ADDR + 0x1000U
#define GPIOF_BASE_ADDR         AHB1_BASE_ADDR + 0x1400U
#define GPIOG_BASE_ADDR         AHB1_BASE_ADDR + 0x1800U
#define GPIOH_BASE_ADDR         AHB1_BASE_ADDR + 0x1C00U

/* AHB1 Perpherials END */

/* APB1 Perpherials START */

#define TIM2_BASE_ADDR          APB1_BASE_ADDR + 0x0000U
#define TIM3_BASE_ADDR          APB1_BASE_ADDR + 0x0400U
#define TIM4_BASE_ADDR          APB1_BASE_ADDR + 0x0800U
#define TIM5_BASE_ADDR          APB1_BASE_ADDR + 0x0C00U
#define TIM6_BASE_ADDR          APB1_BASE_ADDR + 0x1000U
#define TIM7_BASE_ADDR          APB1_BASE_ADDR + 0x1400U
#define TIM12_BASE_ADDR         APB1_BASE_ADDR + 0x1800U
#define TIM13_BASE_ADDR         APB1_BASE_ADDR + 0x1C00U
#define TIM14_BASE_ADDR         APB1_BASE_ADDR + 0x2000U

#define RTC_BKP_BASE_ADDR       APB1_BASE_ADDR + 0x2800U

// Descrptive name
#define WWDG_BASE_ADDR          APB1_BASE_ADDR + 0x2C00U
#define IWDG_BASE_ADDR          APB1_BASE_ADDR + 0x3000U

#define SPI2_I2S2_BASE_ADDR     APB1_BASE_ADDR + 0x3800U
#define SPI3_I2S3_BASE_ADDR     APB1_BASE_ADDR + 0x3C00U

#define SPDIF_RX_BASE_ADDR      APB1_BASE_ADDR + 0x4000U

#define USART2_BASE_ADDR        APB1_BASE_ADDR + 0x4400U
#define USART3_BASE_ADDR        APB1_BASE_ADDR + 0x4800U

#define UART4_BASE_ADDR         APB1_BASE_ADDR + 0x4C00U
#define UART5_BASE_ADDR         APB1_BASE_ADDR + 0x5000U

#define I2C1_BASE_ADDR          APB1_BASE_ADDR + 0x5400U
#define I2C2_BASE_ADDR          APB1_BASE_ADDR + 0x5800U
#define I2C3_BASE_ADDR          APB1_BASE_ADDR + 0x5C00U

#define CAN1_BASE_ADDR          APB1_BASE_ADDR + 0x6400U
#define CAN2_BASE_ADDR          APB1_BASE_ADDR + 0x6800U

#define HDMI_CEC_BASE_ADDR      APB1_BASE_ADDR + 0x6C00U

#define PWR_BASE_ADDR           APB1_BASE_ADDR + 0x7000U

#define DAC_BASE_ADDR           APB1_BASE_ADDR + 0x7400U

/* APB1 Perpherials END */

/* APB2 Perpherials START */

#define TIM1_BASE_ADDR          APB2_BASE_ADDR + 0x0000U
#define TIM8_BASE_ADDR          APB2_BASE_ADDR + 0x0400U

#define USART1_BASE_ADDR        APB2_BASE_ADDR + 0x1000U
#define USART6_BASE_ADDR        APB2_BASE_ADDR + 0x1400U

#define ADC1_2_3_BASE_ADDR      APB2_BASE_ADDR + 0x2000U

#define SDMMC_BASE_ADDR         APB2_BASE_ADDR + 0x2C00U

#define SPI1_BASE_ADDR          APB2_BASE_ADDR + 0x3000U
#define SPI4_BASE_ADDR          APB2_BASE_ADDR + 0x3400U

#define SYSCFG_BASE_ADDR        APB2_BASE_ADDR + 0x3800U

// External Interrupt Controller base address
#define EXTI_BASE_ADDR          APB2_BASE_ADDR + 0x3800U

#define TIM9_BASE_ADDR          APB2_BASE_ADDR + 0x4000U
#define TIM10_BASE_ADDR         APB2_BASE_ADDR + 0x4400U
#define TIM11_BASE_ADDR         APB2_BASE_ADDR + 0x4800U

#define SAI1_BASE_ADDR          APB2_BASE_ADDR + 0x5800U
#define SAI2_BASE_ADDR          APB2_BASE_ADDR + 0x5C00U

/* APB2 Perpherials END */

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

#define GPIOA                   (GPIO_t*)GPIOA_BASE_ADDR
#define GPIOB                   (GPIO_t*)GPIOB_BASE_ADDR
#define GPIOC                   (GPIO_t*)GPIOC_BASE_ADDR
#define GPIOD                   (GPIO_t*)GPIOD_BASE_ADDR
#define GPIOE                   (GPIO_t*)GPIOE_BASE_ADDR
#define GPIOF                   (GPIO_t*)GPIOF_BASE_ADDR
#define GPIOG                   (GPIO_t*)GPIOG_BASE_ADDR
#define GPIOH                   (GPIO_t*)GPIOH_BASE_ADDR


typedef struct RCC_t {
    __vo uint32_t RCC_CR;
    __vo uint32_t RCC_PLL_CFGR;
    __vo uint32_t RCC_CFGR;
    __vo uint32_t RCC_CIR;
    __vo uint32_t RCC_AHB1_RSTR;
    __vo uint32_t RCC_AHB2_RSTR;
    __vo uint32_t RCC_AHB3_RSTR;
    __vo uint32_t RESERVED;
    __vo uint32_t RCC_APB1_RSTR;
    __vo uint32_t RCC_APB2_RSTR;
    __vo uint32_t RESERVED;
    __vo uint32_t RESERVED;
    __vo uint32_t RCC_AHB2_ENR;
    __vo uint32_t RCC_AHB3_ENR;
    __vo uint32_t RESERVED;
    __vo uint32_t RCC_APB1_ENR;
    __vo uint32_t RCC_APB2_ENR;
    __vo uint32_t RESERVED;
    __vo uint32_t RESERVED;
    __vo uint32_t RCC_AHB1_LPENR;
    __vo uint32_t RCC_AHB2_LPENR;
    __vo uint32_t RCC_AHB3_LPENR;
    __vo uint32_t RESERVED;
    __vo uint32_t RCC_APB1_LPENR;
    __vo uint32_t RCC_APB2_LPENR;
    __vo uint32_t RESERVED;
    __vo uint32_t RESERVED;
    __vo uint32_t RCC_BDCR;
    __vo uint32_t RCC_CSR;
    __vo uint32_t RESERVED;
    __vo uint32_t RESERVED;
    __vo uint32_t RCC_SS_CGR;
    __vo uint32_t RCC_PLLI2_SCFGR;
    __vo uint32_t RCC_PLL_SAI_CFGR;
    __vo uint32_t RCC_DCK_CFGR;
    __vo uint32_t RCC_CK_GATENR;
    __vo uint32_t RCC_DCK_CFGR2;

} RCC_t;

#endif
