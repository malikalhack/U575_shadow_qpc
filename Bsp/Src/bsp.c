/**
 * @file    bsp.c
 * @version 1.0.0
 * @authors Anton Chernov
 * @date    02/06/2026
 * @date    @showdate "%m/%d/%Y"
 */

/******************************** Included files ******************************/
#include <stm32u5xx.h>
#include "bsp.h"
/********************************* Definitions ********************************/
#ifndef VECT_TAB_OFFSET
/**
 * @def VECT_TAB_OFFSET
 * @brief Vector Table base offset field. This value must be a multiple of 0x200.
 */
#define VECT_TAB_OFFSET         0x00000000UL
#endif /* VECT_TAB_OFFSET */

#if defined (Q_SPY) || defined (BENCHMARKS)
#define UART_ENABLED
#endif /* Q_SPY || BENCHMARKS */
/**
 * @def NVIC_PRIORITYGROUP_0
 * @brief 0 bit for pre-emption priority, 4 bits for subpriority
 */
#define NVIC_PRIORITYGROUP_0    0x7U

/**
 * @def NVIC_PRIORITYGROUP_1
 * @brief 1 bit for pre-emption priority, 3 bits for subpriority
 */
#define NVIC_PRIORITYGROUP_1    0x6U

/**
 * @def NVIC_PRIORITYGROUP_2
 * @brief 2 bits for pre-emption priority, 2 bits for subpriority
 */
#define NVIC_PRIORITYGROUP_2    0x5U

/**
 * @def NVIC_PRIORITYGROUP_3
 * @brief 3 bits for pre-emption priority, 1 bit for subpriority
 */
#define NVIC_PRIORITYGROUP_3    0x4U

/**
 * @def NVIC_PRIORITYGROUP_4
 * @brief 4 bits for pre-emption priority, 0 bit for subpriority
 */
#define NVIC_PRIORITYGROUP_4    0x3U

/// System core clock frequency in Hz
#define SYSTEM_CLOCK_HZ 2812500U

/// HSI16 clock frequency in Hz (used for USART1 as clock source)
#define HSI16_CLOCK_HZ  16000000U

/// UART baud rate for printf retargeting and QSPY
#define BAUD_RATE 115200U

/// USART1 BRR value for the specified baud rate and clock frequency
#define BRR_VALUE (HSI16_CLOCK_HZ / BAUD_RATE)

#ifdef TEST_IRQ
#define TEST_IRQn           EXTI0_IRQn
#endif

/****************************** Private prototypes ****************************/

/** @brief Resets Clock Control Registers group (RCC). */
static void rcc_reset(void);

/** @brief Configurs flash. */
static void flash_config(void);

/**
 * @brief Configurs Clock Control Registers group (RCC).
 * @note Configures PLL1: MSIS(4MHz) / 1 * 45 / 4 = 45 MHz
 * SYSCLK = PLL1 = 45 MHz
 * HCLK = SYSCLK / 16 = 2.8125 MHz
 */
static void rcc_config(void);

/**
 * @brief Configurs General Purpose Input Output register group (GPIO).
 * @note Enables GPIOC, GPIOB, GPIOG clocks.
 * Configures PC7, PB7, PG2 as outputs for LEDs.
 */
static void gpio_config(void);

/**
 * @brief Configurs 24-bits SysTick timer.
 * @param[in] ticks - ticks per second.
 */
static void systick_config (unsigned int ticks);

/**
 * @brief Initializes the Data Watchpoint and Trace (DWT) unit.
 * @note Enables the cycle counter for performance measurement.
 */
static void dwt_init(void);

#ifdef UART_ENABLED
/**
 * @brief Configures UART for printf retargeting and QSPY.
 * Configure USART1 with the following settings:
 * - Baud rate: 115200
 * - Data bits: 8
 * - Parity: None
 * - Stop bits: 1
 * - Clock source: HSI16 (16 MHz)
 * @note This function configures the USART1 for UART communication.
 */
static void uart_config (void);
#endif /* UART_ENABLED */

/********************* Application Programming Interface **********************/

/** @brief System initialization function */
void SystemInit(void) {
/* Enable debug in low-power modes and freeze peripherals during debug halt */
/* This MUST be done BEFORE any clock changes to maintain debugger connection */
#ifdef DEBUG
    DBGMCU->CR |= DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP | DBGMCU_APB1FZR1_DBG_WWDG_STOP;
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM2_STOP | DBGMCU_APB1FZR1_DBG_TIM3_STOP;
#endif

    /* Disable interrupts during initialization */
    __disable_irq();

    /* FPU settings ---------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    /* set CP10 and CP11 Full Access */
    SCB->CPACR |= ((3UL << 20U)|(3UL << 22U));
#endif

    /* Reset RCC to default state */
    rcc_reset();

    /* Configure Flash wait states */
    flash_config();

    /* Configure system clocks (PLL1 -> 2.8125 MHz) */
    rcc_config();

    /* Configure GPIO ports and LED pins */
    gpio_config();

    /* Initialize DWT for performance measurement */
    dwt_init();

    /* Configure low-power memory settings */
    /* SRAM1 retention only - SRAM2 and SRAM3 powered down in stop mode */
    PWR->CR2 |= PWR_CR2_SRAM2PDS1 | PWR_CR2_SRAM2PDS2;
    /* SRAM2 powered down in Stop mode */
    /* TODO: SRAM3 power down may require CR3 or other register - check STM32U5 reference manual */

    /* Configure EXTI for wake-up sources */
    /* Enable EXTI lines for wake-up from low-power modes */
    EXTI->IMR1 = 0x00000000U;  /* TODO: Configure specific lines as needed */
    EXTI->EMR1 = 0x00000000U;

    /* Configure the Vector Table location add offset address ---------------*/
#ifdef VECT_TAB_SRAM
        /* Vector Table Relocation in Internal SRAM */
        SCB->VTOR = SRAM1_BASE | VECT_TAB_OFFSET;
#else
        /* Vector Table Relocation in Internal FLASH */
        SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; 
#endif

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* Configure SysTick timer for 1 ms interrupts */
    systick_config(SYSTEM_CLOCK_HZ / BSP_TICKS_PER_SEC);

#ifdef UART_ENABLED
    /* Configure UART for printf retargeting and QSPY */
    uart_config();
#endif /* UART_ENABLED */

#ifdef TEST_IRQ
    /* Enable the EXTI0 interrupt for testing */
    /* Set priority (lower number = higher priority) */
    NVIC_SetPriority(TEST_IRQn, 10U);
    NVIC_EnableIRQ(TEST_IRQn);
#endif /* TEST_IRQ */
}

/** @fn get_system_core_clock */
unsigned int get_system_core_clock(void) {
    return SYSTEM_CLOCK_HZ;
}
#ifdef TEST_IRQ
void start_pending_irq(void) {
    NVIC_SetPendingIRQ(TEST_IRQn);
}

void clear_pending_irq(void) {
    NVIC_ClearPendingIRQ(TEST_IRQn);
}
#endif /* TEST_IRQ */

/****************************** Private functions ****************************/

/** @fn rcc_reset */
static void rcc_reset(void) {
    /* Set MSISON bit */
    RCC->CR = RCC_CR_MSISON;

    /* Reset CFGR registers */
    RCC->CFGR1 = 0U;
    RCC->CFGR2 = 0U;
    RCC->CFGR3 = 0U;

    /* Reset HSEON, CSSON, HSION, PLLxON bits */
    RCC->CR &= ~(
        RCC_CR_HSEON|RCC_CR_CSSON|RCC_CR_PLL1ON|RCC_CR_PLL2ON|RCC_CR_PLL3ON
    );

    /* Reset PLL1CFGR register */
    RCC->PLL1CFGR = 0U;

    /* Reset HSEBYP bit */
    RCC->CR &= ~(RCC_CR_HSEBYP);

    /* Disable all interrupts */
    RCC->CIER = 0U;
}

/** @fn flash_config */
static void flash_config(void) {
    /* Configure Flash prefetch, instruction cache, data cache */
    /* For 45 MHz PLL clock, 0 wait states are sufficient at voltage range 3 */
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_0WS;
  
    /* Enable prefetch */
    FLASH->ACR |= FLASH_ACR_PRFTEN;
}

/** @fn rcc_config */
static void rcc_config(void) {
    /* Enable MSI oscillator and configure range */
    RCC->CR |= RCC_CR_MSISON;
    while ((RCC->CR & RCC_CR_MSISRDY) == 0U);

    /* Select MSI range 4 MHz (Range 4 = 0b0100) */
    RCC->ICSCR1 &= ~RCC_ICSCR1_MSISRANGE;
    RCC->ICSCR1 |= (4U << RCC_ICSCR1_MSISRANGE_Pos);  /* 4 MHz */
    RCC->ICSCR1 |= RCC_ICSCR1_MSIRGSEL;  /* Use MSIRANGE from ICSCR1 */

    /* Configure PLL1 */
    /* PLL1 source = MSIS */
    RCC->PLL1CFGR &= ~RCC_PLL1CFGR_PLL1SRC;
    RCC->PLL1CFGR |= (0x1U << RCC_PLL1CFGR_PLL1SRC_Pos);  /* MSIS as source */

    /* PLL1M = 1 (divider = 1) */
    RCC->PLL1CFGR &= ~RCC_PLL1CFGR_PLL1M;
    RCC->PLL1CFGR |= (0x0U << RCC_PLL1CFGR_PLL1M_Pos);  /* M = 1 */

    /* PLL1N = 45 (multiplier = 45) */
    RCC->PLL1DIVR &= ~RCC_PLL1DIVR_PLL1N;
    RCC->PLL1DIVR |= ((45U - 1U) << RCC_PLL1DIVR_PLL1N_Pos);

    /* PLL1R = 4 (divider = 4) */
    RCC->PLL1DIVR &= ~RCC_PLL1DIVR_PLL1R;
    RCC->PLL1DIVR |= ((4U - 1U) << RCC_PLL1DIVR_PLL1R_Pos);

    /* Enable PLL1R output */
    RCC->PLL1CFGR |= RCC_PLL1CFGR_PLL1REN;

    /* Configure PLL1 input range (4 MHz -> Range 0: 4-8 MHz) */
    RCC->PLL1CFGR &= ~RCC_PLL1CFGR_PLL1RGE;
    RCC->PLL1CFGR |= (0x2U << RCC_PLL1CFGR_PLL1RGE_Pos);  /* 4-8 MHz range */

    /* Enable PLL1 */
    RCC->CR |= RCC_CR_PLL1ON;
    while ((RCC->CR & RCC_CR_PLL1RDY) == 0U);

    /* Configure AHB prescaler = 16 (HPRE = 0b1011 for /16 in STM32U5) */
    RCC->CFGR2 &= ~RCC_CFGR2_HPRE;
    RCC->CFGR2 |= (0xBU << RCC_CFGR2_HPRE_Pos);  /* Divide by 16: bits [3:0] = 1011 */

    /* Select PLL as system clock source */
    RCC->CFGR1 &= ~RCC_CFGR1_SW;
    RCC->CFGR1 |= RCC_CFGR1_SW_1 | RCC_CFGR1_SW_0;  /* PLL1 as SYSCLK */

    /* Wait until PLL is used as system clock */
    while ((RCC->CFGR1 & RCC_CFGR1_SWS) != (RCC_CFGR1_SWS_1 | RCC_CFGR1_SWS_0));

    /* Enable GPIOA clock for UART */
    RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOAEN;

    /* Enable HSI16 for USART1 clock source */
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U);  /* Wait for HSI16 ready */

    /* Select HSI16 as USART1 clock source (CCIPR1[1:0] = 0b10) */
    RCC->CCIPR1 &= ~RCC_CCIPR1_USART1SEL;
    RCC->CCIPR1 |= (0x2U << RCC_CCIPR1_USART1SEL_Pos);  /* HSI16 as source */

    /* Enable USART1 clock */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
}

/** @fn gpio_config */
static void gpio_config(void) {
    unsigned char pwrenabled = 0;
    /* Enable GPIO clocks */
    RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOCEN;  /* Enable GPIOC clock */
    RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOBEN;  /* Enable GPIOB clock */
    /* Enable VddIO2 for GPIOG (red LED) */
    if(READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_PWREN) == 0U) {
        __IO uint32_t tmpreg;
        SET_BIT(RCC->AHB3ENR, RCC_AHB3ENR_PWREN);
        /* Delay after an RCC peripheral clock enabling */
        tmpreg = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_PWREN);
        pwrenabled = 1;
        (void)tmpreg; /* Avoid warning */
    }
    SET_BIT(PWR->SVMCR, PWR_SVMCR_IO2SV);   /* Enable VddIO2 for GPIOG */

    if(pwrenabled) {
        CLEAR_BIT(RCC->AHB3ENR, RCC_AHB3ENR_PWREN);
    }
    SET_BIT(RCC->AHB2ENR1, RCC_AHB2ENR1_GPIOGEN);  /* Enable GPIOG clock */

    /* Small delay after clock enable */
    volatile uint32_t delay = 100;
    while (delay--);

    /* Configure PC7 as output (LED_GREEN) */
    GPIOC->MODER &= ~(3U << (7U * 2U));
    GPIOC->MODER |= (1U << (7U * 2U));  /* Output mode */
    GPIOC->OTYPER &= ~(1U << 7U);  /* Push-pull */
    GPIOC->OSPEEDR &= ~(3U << (7U * 2U));  /* Low speed */
    GPIOC->PUPDR &= ~(3U << (7U * 2U));  /* No pull-up/pull-down */
    GPIOC->ODR &= ~(1U << 7U);  /* LED OFF initially */

    /* Configure PB7 as output (LED_BLUE) */
    GPIOB->MODER &= ~(3U << (7U * 2U));
    GPIOB->MODER |= (1U << (7U * 2U));  /* Output mode */
    GPIOB->OTYPER &= ~(1U << 7U);  /* Push-pull */
    GPIOB->OSPEEDR &= ~(3U << (7U * 2U));  /* Low speed */
    GPIOB->PUPDR &= ~(3U << (7U * 2U));  /* No pull-up/pull-down */
    GPIOB->ODR &= ~(1U << 7U);  /* LED OFF initially */

    /* Configure PG2 as output (LED_RED) */
    GPIOG->MODER &= ~(3U << (2U * 2U));
    GPIOG->MODER |= (1U << (2U * 2U));  /* Output mode */
    GPIOG->OTYPER &= ~(1U << 2U);  /* Push-pull */
    GPIOG->OSPEEDR &= ~(3U << (2U * 2U));  /* Low speed */
    GPIOG->PUPDR &= ~(3U << (2U * 2U));  /* No pull-up/pull-down */
    GPIOG->ODR &= ~(1U << 2U);  /* LED OFF initially */

    /* USART1: PA9 (TX), PA10 (RX) */
    /* Configure PA9 as alternate function (USART1_TX) */
    GPIOA->MODER &= ~GPIO_MODER_MODE9;
    GPIOA->MODER |= GPIO_MODER_MODE9_1;  /* Alternate function mode */
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED9;  /* Clear speed bits */
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED9_0;  /* Medium speed (10 MHz) */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT9;  /* Push-pull */
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD9;  /* No pull-up/pull-down for TX */
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9;
    GPIOA->AFR[1] |= (7U << GPIO_AFRH_AFSEL9_Pos);  /* AF7 for USART1 */

    /* Configure PA10 as alternate function (USART1_RX) */
    GPIOA->MODER &= ~GPIO_MODER_MODE10;
    GPIOA->MODER |= GPIO_MODER_MODE10_1;  /* Alternate function mode */
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED10;  /* Clear speed bits */
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED10_0;  /* Medium speed (10 MHz) */
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD10;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD10_0;  /* Pull-up for RX */
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10;
    GPIOA->AFR[1] |= (7U << GPIO_AFRH_AFSEL10_Pos);  /* AF7 for USART1 */
}

/** @fn systick_config */
static void systick_config(unsigned int ticks) {
    /* Check if ticks value is valid (24-bit counter) */
    if ((ticks - 1U) > 0xFFFFFFU) {
        return;  /* Invalid reload value */
    }

    /* Set reload register */
    SysTick->LOAD = (unsigned int)(ticks - 1U);

    /* Set priority for SysTick interrupt */
    NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);

    /* Load the SysTick counter value */
    SysTick->VAL = 0U;

    /* Enable SysTick IRQ and SysTick Timer */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

/** @fn dwt_init */
static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

#ifdef UART_ENABLED
/** @fn uart_config */
static void uart_config (void) {
    /* Configure USART1 BRR register
    Actual baud rate = HSI16_CLOCK_HZ / 139 = 115107.9 (error 0.08%) */
    USART1->BRR = BRR_VALUE;

    /* Enable USART1, transmitter and receiver */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

#ifdef Q_SPY
    NVIC_EnableIRQ(USART1_IRQn); /* USART1 interrupt used for QS-RX */
#endif
}
#endif /* UART_ENABLED */
/*****************************************************************************/

/** @fn turn_on_led_green */
void turn_on_led_green(void) {
    GPIOC->ODR |= (1 << 7);  // LED_GREEN
}

/** @fn turn_on_led_blue */
void turn_on_led_blue(void) {
    GPIOB->ODR |= (1 << 7);  // LED_BLUE
}

/** @fn turn_on_led_red */
void turn_on_led_red(void) {
    GPIOG->ODR |= (1 << 2);  // LED_RED
}

/** @fn turn_off_led_green */
void turn_off_led_green(void) {
    GPIOC->ODR &= ~(1 << 7);  // LED_GREEN
}

/** @fn turn_off_led_blue */
void turn_off_led_blue(void) {
    GPIOB->ODR &= ~(1 << 7);  // LED_BLUE
}

/** @fn turn_off_led_red */
void turn_off_led_red(void) {
    GPIOG->ODR &= ~(1 << 2);  // LED_RED
}
/*****************************************************************************/
