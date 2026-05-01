/**
 * @file    handlers.c
 * @version 1.0.0
 * @authors Anton Chernov
 * @date    02/13/2026
 * @date    @showdate "%m/%d/%Y"
 */

/********************************* Include files ******************************/
#include <stm32u5xx.h>
#ifdef TEST_IRQ
#include "tm_porting_layer.h"
#include "tm_counter_api.h"
#else
#include "qpc.h"
#endif // TEST_IRQ
/********************************* Definitions ********************************/

Q_DEFINE_THIS_FILE

/****************************** Private  variables ****************************/
#ifdef Q_SPY
static QSTimeCtr _QS_tickTime;
static QSTimeCtr _QS_tickPeriod;
#endif

/****************************** External variables ****************************/
#ifdef TEST_IRQ
/* The interrupt count. */
extern unsigned long tm_interrupt_handler_counter;
#endif // TEST_IRQ
/****************************** Interrupt handlers ****************************/

void SysTick_Handler(void) {
    QXK_ISR_ENTRY();   /* inform QXK about entering an ISR */
#ifdef Q_SPY
    _QS_tickTime += _QS_tickPeriod; /* account for the clock rollover */
#endif
    QTIMEEVT_TICK_X(0U, (void *)0); /* clock tick processing for rate 0 */
    QXK_ISR_EXIT();  /* inform QXK about exiting an ISR */
}
/*----------------------------------------------------------------------------*/

/**
 * @brief C-level HardFault handler that analyzes the fault and turns on
 * the red LED.
 * @details This function is called from the assembly HardFault_Handler after
 * determining the correct stack pointer (MSP or PSP). It disables interrupts,
 * turns on the red LED to indicate a fault, and in debug builds, captures
 * various fault status registers and the CPU register state at the time of the
 * fault for debugging purposes. Finally, it enters an infinite loop to halt
 * execution.
 * @param[in] sp Pointer to the stack frame at the time of the fault, containing
 * the CPU registers r0-r3, r12, lr, pc, and xPSR.
 */
void __NO_RETURN HardFault_C(uint32_t *sp) {
    __disable_irq();          // Disable interrupts
    GPIOC->ODR &= ~(1 << 7);  // Turn off LED_GREEN
    GPIOB->ODR &= ~(1 << 7);  // Turn off LED_BLUE
    GPIOG->ODR |= (1 << 2);   // Turn on LED_RED
#ifdef DEBUG
    volatile uint32_t CFSR  = SCB->CFSR;
    volatile uint32_t HFSR  = SCB->HFSR;
    volatile uint32_t MMFAR = SCB->MMFAR;
    volatile uint32_t BFAR  = SCB->BFAR;
    volatile uint32_t r0    = sp[0];
    volatile uint32_t r1    = sp[1];
    volatile uint32_t r2    = sp[2];
    volatile uint32_t r3    = sp[3];
    volatile uint32_t r12   = sp[4];
    volatile uint32_t lr    = sp[5];
    volatile uint32_t pc    = sp[6];
    volatile uint32_t xpsr  = sp[7];
    (void)CFSR; (void)HFSR; (void)MMFAR; (void)BFAR;
    (void)r0; (void)r1; (void)r2; (void)r3;
    (void)r12; (void)lr; (void)pc; (void)xpsr;
#endif
    while(1);
}

/**
 * @brief Naked HardFault handler that determines the active stack pointer and
 * then branches to the C-level handler.
 * @details This function is the actual HardFault handler that the processor
 * jumps to when a HardFault occurs. It uses inline assembly to check
 * bit 2 of the EXC_RETURN value in the LR register to determine which stack
 * pointer (MSP or PSP) was active at the time of the fault.
 * It then loads the appropriate stack pointer into R0 and branches to the
 * C-level handler HardFault_C, passing the stack pointer as an argument.
 */
__attribute__((naked)) void __NO_RETURN HardFault_Handler(void) {
    __asm volatile (
        "tst   lr, #4        \n"
        "ite   eq            \n"
        "mrseq r0, msp       \n"
        "mrsne r0, psp       \n"
        "b     HardFault_C   \n"
    );
}
/*----------------------------------------------------------------------------*/
#ifdef Q_SPY
void USART1_IRQHandler(void) {
    QXK_ISR_ENTRY();   /* inform QXK about entering an ISR */
    if(USART1->ISR & USART_ISR_RXNE) { /* is RX register NOT empty? */
        uint8_t b = (uint8_t)USART1->RDR; /* read the received byte */
        QS_RX_PUT(b); /* put into the QS receive buffer */
    }
    QXK_ISR_EXIT();  /* inform QXK about exiting an ISR */
}
#endif
/*----------------------------------------------------------------------------*/
#ifdef TEST_IRQ
/**
 * @brief EXTI0 interrupt handler that simulates an external interrupt for testing.
 * @details This function is called when the EXTI0 interrupt is triggered. It
 * clears the interrupt pending bit, increments the interrupt count, resets the
 * cycle counter, and releases a semaphore to signal a waiting thread. This is
 * used in the interrupt processing test to measure the time taken for a full
 * interrupt processing round-trip.
 */
void EXTI0_IRQHandler(void) {
    ISR_ENTRY();

    /* Clear the EXTI0 interrupt pending bit */
    EXTI->RPR1 = (1 << 0);

    /* Increment the interrupt count. */
    tm_interrupt_handler_counter++;

    /* Reset the cycle counter */
    tm_reset_counter();

    /* Release the semaphore to signal the waiting thread. */
    (void)tm_semaphore_put(0);

    ISR_EXIT();
}
#endif /* TEST_IRQ */

/******************************* Framework Handlers ***************************/

void QXK_onIdle(void) {
#ifdef Q_SPY
    QS_rxParse(); /* parse all the received bytes */
    if ((USART1->ISR & USART_ISR_TXE) != 0U) { /* is TXE empty? */
        QF_INT_DISABLE();
        uint16_t b = QS_getByte(); /* try to get next byte to transmit */
        QF_INT_ENABLE();
        if (b != QS_EOD) { /* not End-Of-Data? */
            USART1->TDR = (b & 0xFF); /* put into the DR register */
        }
    }
#endif
}
/*----------------------------------------------------------------------------*/

Q_NORETURN Q_onError(char const * const module, int_t const id) {
    QS_ASSERTION(module, id, 10000U);
#if DEBUG
    GPIOC->ODR &= ~(1 << 7);  // Turn off LED_GREEN
    GPIOG->ODR &= ~(1 << 2);  // Turn off LED_RED
    GPIOB->ODR |= (1 << 7);   // Turn on LED_BLUE
    while(1);
#else
    NVIC_SystemReset();
#endif
}
/*----------------------------------------------------------------------------*/

void QF_onStartup(void) {
#ifdef Q_SPY
    _QS_tickPeriod = get_system_core_clock() / BSP_TICKS_PER_SEC;
    _QS_tickTime = _QS_tickPeriod; /* to start the timestamp at zero */
#endif
    /* Enable interrupts after QXK kernel is fully initialized */
    __enable_irq();
}
/*----------------------------------------------------------------------------*/

void QF_onCleanup(void) {

}
/*----------------------------------------------------------------------------*/
#ifdef Q_SPY
uint8_t QS_onStartup(void const *arg) {
    Q_UNUSED_PAR(arg); /* avoid the "unused parameter" compiler warning */
    static uint8_t qs_tx_buf[2*1024]; /* buffer for QS-TX channel */
    static uint8_t qs_rx_buf[100];  /* buffer for QS-RX channel */
    QS_initBuf(qs_tx_buf, sizeof(qs_tx_buf));
    QS_rxInitBuf(qs_rx_buf, sizeof(qs_rx_buf));
    return 0;
}
/*----------------------------------------------------------------------------*/

QSTimeCtr QS_onGetTime(void) {
    QSTimeCtr result;
    // Code
    return result;
}
/*----------------------------------------------------------------------------*/

void QS_onFlush(void) {
    uint16_t b;
    QF_INT_DISABLE();
    while ((b = QS_getByte()) != QS_EOD) { /* not End-Of-Data? */
        QF_INT_ENABLE();
        while ((USART1->ISR & USART_ISR_TXE) == 0U); /* wait until TXE empty */
        USART1->TDR = (b & 0xFF); /* put into the DR register */
        QF_INT_DISABLE();
    }
    QF_INT_ENABLE();
}
/*----------------------------------------------------------------------------*/

void QS_onReset(void) {
    NVIC_SystemReset();
}
/*----------------------------------------------------------------------------*/

void QS_onCleanup(void) {

}
/*----------------------------------------------------------------------------*/

void QS_onCommand(
    uint8_t cmdId,
    uint32_t param1,
    uint32_t param2,
    uint32_t param3
) {
    (void)cmdId;
    (void)param1;
    (void)param2;
    (void)param3;
}
/******************************************************************************/
#endif
