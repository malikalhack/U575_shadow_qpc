/**
 * @file    main.c
 * @version 0.0.0
 * @authors Anton Chernov
 * @date    02/02/2026
 * @date    @showdate "%m/%d/%Y"
 */

/********************************* Include files ******************************/
#include "qpc.h"
/********************************* Definitions ********************************/

Q_DEFINE_THIS_FILE

/****************************** External prototypes ***************************/

extern void task_start(void);

/********************************* Entry point ********************************/

/** @brief  Main entry point of the application. */
int main(void) {
    QF_init();       // initialize the framework and the underlying RT kernel
    QXK_init();       // initialize the QXK kernel
#ifdef Q_SPY
    if (QS_INIT((void *)0) != 0U) { /* initialize the QS software tracing */
        Q_ERROR();
    }
#endif
    task_start();    // create the AOs and start Threads
    return QF_run(); // run the QF application
}
/******************************************************************************/
