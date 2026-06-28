#include <stdint.h>



#define UART_BASE 0x40000000

#define UART_TX_DATA  (*(volatile uint32_t *)(UART_BASE + 0x00))
#define UART_RX_DATA  (*(volatile uint32_t *)(UART_BASE + 0x04))
#define UART_STATUS   (*(volatile uint32_t *)(UART_BASE + 0x08))


#define TX_BUSY_BIT  0
#define RX_VALID_BIT 1


void uart_putc(char c) 
{
    while (UART_STATUS & (1 << TX_BUSY_BIT)) 
    {
    }
    UART_TX_DATA = (uint32_t)c;
}


char uart_getc() 
{
   
    while (!(UART_STATUS & (1 << RX_VALID_BIT))) 
    {
    }
    return (char)UART_RX_DATA;
}