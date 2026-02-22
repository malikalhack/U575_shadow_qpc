/**
 * @file    bsp.h
 * @version 1.0.0
 * @authors Anton Chernov
 * @date    02/06/2026
 * @date    @showdate "%m/%d/%Y"
 */

#ifndef BSP_H_
#define BSP_H_
/********************************* Definitions *******************************/

/**
 * @def BSP_TICKS_PER_SECOND
 * @brief SysTick timer frequency in Hz.
 */
#ifndef BENCHMARKS
#define BSP_TICKS_PER_SEC   1000U
#else
#define BSP_TICKS_PER_SEC   100U
#endif

/********************* Application Programming Interface *********************/

/**
 * @brief Get the system core clock frequency.
 * @return System core clock frequency in Hz.
 */
unsigned int get_system_core_clock(void);

#ifdef TEST_IRQ
/** @brief Start the pending interrupt. */
void start_pending_irq(void);

/** @brief Clear the pending interrupt. */
void clear_pending_irq(void);
#endif /* TEST_IRQ */

/*****************************************************************************/
// LED control functions

/** @brief Turn on the green LED. */
void turn_on_led_green(void);

/** @brief Turn on the blue LED. */
void turn_on_led_blue(void);

/** @brief Turn on the red LED. */
void turn_on_led_red(void);

/** @brief Turn off the green LED. */
void turn_off_led_green(void);

/** @brief Turn off the blue LED. */
void turn_off_led_blue(void);

/** @brief Turn off the red LED. */
void turn_off_led_red(void);

/*****************************************************************************/
#endif //! BSP_H_
