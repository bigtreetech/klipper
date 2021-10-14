// Code to setup clocks and gpio on stm32f0
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_REF_FREQ
#include "board/armcm_boot.h" // armcm_main
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // enable_pclock
#include "sched.h" // sched_main

#define FREQ_PERIPH 64000000

typedef struct
{
    uint32_t periph_addr;
    volatile uint32_t* rcc_src;
    uint8_t rcc_bit;
}periph_map_t;

// const periph_map_t apb_map[] = {
//     {TIM2_BASE, &RCC->APBENR1, 0},
//     {TIM3_BASE, &RCC->APBENR1, 1},
//     {TIM4_BASE, &RCC->APBENR1, 2},
//     {TIM6_BASE, &RCC->APBENR1, 4},
//     {TIM7_BASE, &RCC->APBENR1, 5},
//     {LPUART2_BASE, &RCC->APBENR1, 7},
//     {USART5_BASE, &RCC->APBENR1, 8},
//     {USART6_BASE, &RCC->APBENR1, 9},
//     {RTC_BASE, &RCC->APBENR1, 10},
//     {WWDG_BASE, &RCC->APBENR1, 11},
//     {FDCAN1_BASE, &RCC->APBENR1, 12},
//     {FDCAN2_BASE, &RCC->APBENR1, 12},
//     {USB_BASE, &RCC->APBENR1, 13},
//     {SPI2_BASE, &RCC->APBENR1, 14},
//     {SPI3_BASE, &RCC->APBENR1, 15},
//     {CRS_BASE, &RCC->APBENR1, 16},
//     {USART2_BASE, &RCC->APBENR1, 17},
//     {USART3_BASE, &RCC->APBENR1, 18},
//     {USART4_BASE, &RCC->APBENR1, 19},
//     {LPUART1_BASE, &RCC->APBENR1, 20},
//     {I2C1_BASE, &RCC->APBENR1, 21},
//     {I2C2_BASE, &RCC->APBENR1, 22},
//     {I2C3_BASE, &RCC->APBENR1, 23},
//     {CEC_BASE, &RCC->APBENR1, 24},
//     {UCPD1_BASE, &RCC->APBENR1, 25},
//     {UCPD2_BASE, &RCC->APBENR1, 26},
//     {DBG_BASE, &RCC->APBENR1, 27},
//     {PWR_BASE, &RCC->APBENR1, 28},
//     {DAC1_BASE, &RCC->APBENR1, 29},
//     {LPTIM2_BASE, &RCC->APBENR1, 30},
//     {LPTIM1_BASE, &RCC->APBENR1, 31},

//     {SYSCFG_BASE, &RCC->APBENR2, 0},
//     {TIM1_BASE, &RCC->APBENR2, 11},
//     {SPI1_BASE, &RCC->APBENR2, 12},
//     {USART1_BASE, &RCC->APBENR2, 14},
//     {TIM14_BASE, &RCC->APBENR2, 15},
//     {TIM15_BASE, &RCC->APBENR2, 16},
//     {TIM16_BASE, &RCC->APBENR2, 17},
//     {TIM17_BASE, &RCC->APBENR2, 18},
//     {ADC1_BASE, &RCC->APBENR2, 20},
// };

const periph_map_t apb_mess_map[] = {
    {LPUART2_BASE, &RCC->APBENR1, 7},
    {USART5_BASE, &RCC->APBENR1, 8},
    {USART6_BASE, &RCC->APBENR1, 9}, // > SYSCFG_BASE but in APB1
    {FDCAN1_BASE, &RCC->APBENR1, 12},
    {FDCAN2_BASE, &RCC->APBENR1, 12},
    {USB_BASE, &RCC->APBENR1, 13},
    {CRS_BASE, &RCC->APBENR1, 16},
    {LPUART1_BASE, &RCC->APBENR1, 20},
    {I2C3_BASE, &RCC->APBENR1, 23},
    {CEC_BASE, &RCC->APBENR1, 24},
    {UCPD1_BASE, &RCC->APBENR1, 25},
    {UCPD2_BASE, &RCC->APBENR1, 26},
    {DBG_BASE, &RCC->APBENR1, 27}, // > SYSCFG_BASE but in APB1
    {LPTIM2_BASE, &RCC->APBENR1, 30},

    {TIM14_BASE, &RCC->APBENR2, 15}, // < SYSCFG_BASE but in APB2
    {ADC1_BASE, &RCC->APBENR2, 20},
};

// Enable a peripheral clock
void
enable_pclock(uint32_t periph_base)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(apb_mess_map); i++) {
        if (periph_base == apb_mess_map[i].periph_addr) {
            *apb_mess_map[i].rcc_src |= 1 << apb_mess_map[i].rcc_bit;
            return;
        }
    }
    if (periph_base < SYSCFG_BASE) {
        uint32_t pos = (periph_base - APBPERIPH_BASE) / 0x400;
        RCC->APBENR1 |= 1 << pos;
        RCC->APBENR1;
    } else if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - SYSCFG_BASE) / 0x400;
        RCC->APBENR2 |= 1 << pos;
        RCC->APBENR2;
    } else if (periph_base < IOPORT_BASE){
        uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
        RCC->AHBENR |= 1 << pos;
        RCC->AHBENR;
    } else{
        uint32_t pos = (periph_base - IOPORT_BASE) / 0x400;
        RCC->IOPENR |= 1 << pos;
        RCC->IOPENR;
    }
}

// Check if a peripheral clock has been enabled
int
is_enabled_pclock(uint32_t periph_base)
{
    if (periph_base < SYSCFG_BASE) {
        uint32_t pos = (periph_base - APBPERIPH_BASE) / 0x400;
        return RCC->APBENR1 & (1 << pos);
    } else if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - SYSCFG_BASE) / 0x400;
        return RCC->APBENR2 & (1 << pos);
    } else if (periph_base < IOPORT_BASE){
        uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
        return RCC->AHBENR & 1 << pos;
    } else{
        uint32_t pos = (periph_base - IOPORT_BASE) / 0x400;
        return RCC->IOPENR & 1 << pos;
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return FREQ_PERIPH;
}

// Enable a GPIO peripheral clock
void
gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - IOPORT_BASE) / 0x400;
    RCC->IOPENR |= 1 << rcc_pos;
    RCC->IOPENR;
}

// Set the mode and extended function of a pin
void
gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];

    // Enable GPIO clock
    gpio_clock_enable(regs);

    // Configure GPIO
    uint32_t mode_bits = mode & 0xf, func = (mode >> 4) & 0xf, od = mode >> 8;
    uint32_t pup = pullup ? (pullup > 0 ? 1 : 2) : 0;
    uint32_t pos = gpio % 16, af_reg = pos / 8;
    uint32_t af_shift = (pos % 8) * 4, af_msk = 0x0f << af_shift;
    uint32_t m_shift = pos * 2, m_msk = 0x03 << m_shift;

    regs->AFR[af_reg] = (regs->AFR[af_reg] & ~af_msk) | (func << af_shift);
    regs->MODER = (regs->MODER & ~m_msk) | (mode_bits << m_shift);
    regs->PUPDR = (regs->PUPDR & ~m_msk) | (pup << m_shift);
    regs->OTYPER = (regs->OTYPER & ~(1 << pos)) | (od << pos);
    regs->OSPEEDR = (regs->OSPEEDR & ~m_msk) | (0x02 << m_shift);
}

#define USB_BOOT_FLAG_ADDR (CONFIG_RAM_START + CONFIG_RAM_SIZE - 1024)
#define USB_BOOT_FLAG 0x55534220424f4f54 // "USB BOOT"

// Handle USB reboot requests
void
usb_request_bootloader(void)
{
    irq_disable();
    *(uint64_t*)USB_BOOT_FLAG_ADDR = USB_BOOT_FLAG;
    NVIC_SystemReset();
}

#if !CONFIG_STM32_CLOCK_REF_INTERNAL
DECL_CONSTANT_STR("RESERVE_PINS_crystal", "PF0,PF1");
#endif

// Configure and enable the PLL as clock source
static void
pll_setup(void)
{
    /**************** PWR ********************/
    /* Modify voltage scaling range */
    PWR->CR1 &= (~PWR_CR1_VOS | PWR_CR1_VOS_0);
    /* Wait until VOSF is reset */
    while (PWR->SR2 & PWR_SR2_VOSF)
        ;

    RCC->PLLCFGR = 0x00001000;
    RCC->CFGR = 0x00000000;
    RCC->CR = 0x00000500;

    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U)
        ;
    // Disable PLL
    RCC->CR &= ~RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) != 0U)
        ;
    // Set PLL
    MODIFY_REG(RCC->PLLCFGR,
                (RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN |
                    RCC_PLLCFGR_PLLP   | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLR),
                ((uint32_t) (RCC_PLLCFGR_PLLSRC_HSE)
                    | (uint32_t) (0x00000000U)
                    | (uint32_t) ((16) << RCC_PLLCFGR_PLLN_Pos)
                    | (uint32_t) (RCC_PLLCFGR_PLLP_0)
                    | (uint32_t) (RCC_PLLCFGR_PLLQ_0)
                    | (uint32_t) (RCC_PLLCFGR_PLLR_0)));
    // Enable PLL
    RCC->CR |=RCC_CR_PLLON;
    // Enable PLLR Clock output
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
    // Wait till PLL is ready
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U)
        ;
    // Enable HSI48
    RCC->CR |= RCC_CR_HSI48ON;
    while ((RCC->CR &RCC_CR_HSI48RDY) == 0U)
        ;
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_1);
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_1)
        ;
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, \
               (RCC_CFGR_PPRE_2 | RCC_CFGR_PPRE_1 | RCC_CFGR_PPRE_0));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, 0x00000000U);
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U)
        ;
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_1);
    // while ((RCC->CFGR & RCC_CFGR_SWS) != (RCC_CFGR_SW_1 << RCC_CFGR_SWS_Pos))
    //     ;

    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, 0x00000000U);

    volatile uint32_t wait_loop_index = 20000;
    while(wait_loop_index != 0)
    wait_loop_index--;

    /* Configure the ADC clock source */
    // MODIFY_REG(RCC->CCIPR, RCC_CCIPR_ADCSEL, (uint32_t)(RCC_CCIPR_ADCSEL_1));
    /* HSI48 oscillator clock selected as USB clock */
    MODIFY_REG(RCC->CCIPR2, RCC_CCIPR2_USBSEL, 0x00000000U);
}

void
armcm_main(void)
{
    if (CONFIG_USBSERIAL && *(uint64_t*)USB_BOOT_FLAG_ADDR == USB_BOOT_FLAG) {
        *(uint64_t*)USB_BOOT_FLAG_ADDR = 0;
        uint32_t *sysbase = (uint32_t*)0x1fff0000;
        asm volatile("mov sp, %0\n bx %1"
                     : : "r"(sysbase[0]), "r"(sysbase[1]));
    }

    // Run SystemInit() and then restore VTOR
    SystemInit();
    SCB->VTOR = (uint32_t)VectorTable;

    pll_setup();

    sched_main();
}
