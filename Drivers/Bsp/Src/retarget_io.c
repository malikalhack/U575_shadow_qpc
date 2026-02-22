/**
 * @file    retarget_io.c
 * @version 1.0.0
 * @authors Anton Chernov
 * @date    02/19/2026
 * @date    @showdate "%m/%d/%Y"
 * @note    Retargeting for printf/scanf to USART1 for ARM Compiler 6
 * @note    Uses USART1 (PA9=TX, PA10=RX) for stdio redirection
 */

/********************************* Include files ******************************/
#include <stdio.h>
#include <stm32u5xx.h>
/******************************************************************************/

/* Disable semihosting for ARM Compiler 6 */
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
__asm(".global __use_no_semihosting");

/* File handle stubs */
typedef int FILEHANDLE;
#define STDIN   0
#define STDOUT  1
#define STDERR  2

/* Minimal semihosting stubs required by ARM runtime library */
void _sys_exit(int return_code) {
    (void)return_code;
    while (1); /* Loop forever */
}
/*----------------------------------------------------------------------------*/

void _ttywrch(int ch) {
    (void)ch;
}
/*----------------------------------------------------------------------------*/

FILEHANDLE _sys_open(const char *name, int openmode) {
    (void)name;
    (void)openmode;
    return STDOUT;
}
/*----------------------------------------------------------------------------*/

int _sys_close(FILEHANDLE fh) {
    (void)fh;
    return 0;
}
/*----------------------------------------------------------------------------*/

int _sys_write(FILEHANDLE fh, const unsigned char *buf, unsigned len, int mode) {
    (void)fh;
    (void)mode;
    
    for (unsigned i = 0; i < len; i++) {
        while (!(USART1->ISR & USART_ISR_TXE_TXFNF));
        USART1->TDR = buf[i];
    }
    return 0;
}
/*----------------------------------------------------------------------------*/

int _sys_read(FILEHANDLE fh, unsigned char *buf, unsigned len, int mode) {
    (void)fh;
    (void)mode;
    
    for (unsigned i = 0; i < len; i++) {
        while (!(USART1->ISR & USART_ISR_RXNE_RXFNE));
        buf[i] = (unsigned char)(USART1->RDR & 0xFF);
    }
    return 0;
}
/*----------------------------------------------------------------------------*/

int _sys_istty(FILEHANDLE fh) {
    (void)fh;
    return 1;
}
/*----------------------------------------------------------------------------*/

long _sys_flen(FILEHANDLE fh) {
    (void)fh;
    return -1;
}
/*----------------------------------------------------------------------------*/

int _sys_seek(FILEHANDLE fh, long pos) {
    (void)fh;
    (void)pos;
    return -1;
}
/******************************************************************************/
#endif /* __ARMCC_VERSION */

/**
 * @brief Redirect single character output (most commonly used by printf)
 * @param ch Character to output
 * @param f File pointer (not used)
 * @return Character written
 */
int fputc(int ch, FILE *f) {
    (void)f;
    
    /* Wait until transmit data register is empty */
    while (!(USART1->ISR & USART_ISR_TXE_TXFNF));
    
    /* Send character */
    USART1->TDR = (ch & 0xFF);
    
    return ch;
}

/**
 * @brief Redirect single character input (for scanf)
 * @param f File pointer (not used)
 * @return Character read
 */
int fgetc(FILE *f) {
    (void)f;
    
    /* Wait until data is received */
    while (!(USART1->ISR & USART_ISR_RXNE_RXFNE));
    
    /* Read and return character */
    return (int)(USART1->RDR & 0xFF);
}
/******************************************************************************/
