#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include "uart_debug.h"
#define BAUD 9600UL
#include <util/setbaud.h>

void uart_debug_init()
{
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;

#if USE_2X
	UCSRA |= (1 << U2X);
#else
	UCSRA &= ~(1 << U2X);
#endif

	UCSRB |= (1 << TXEN);
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
}

static void uart_putchar(char c)
{
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = c;
}

static void put_flag(boolean_t b) {
    if (b) {
        uart_putchar('1');
    } else {
        uart_putchar('0');
    }
}

static void put_uint8(uint8_t val) {
    char buf[3];
    memset(buf, ' ', 3);
    itoa(val, buf, 10);
    uart_putchar(buf[0]);
    uart_putchar(buf[1]);
    uart_putchar(buf[2]);
}

static void put_uint16(uint16_t nr){
    char buf[5];
    memset(buf, ' ', 5);
    itoa(nr, buf, 10);
    uart_putchar(buf[0]);
    uart_putchar(buf[1]);
    uart_putchar(buf[2]);
    uart_putchar(buf[3]);
    uart_putchar(buf[4]);
}

void debug_appstate(appstate_t *appstate)
{
#ifdef DEBUGUART
    switch (appstate->charge_mode)
    {
        case CHARGE_NONE:
            uart_putchar('N');
            break;
        case CHARGE_A:
            uart_putchar('A');
            break;
        case CHARGE_B:
            uart_putchar('B');
            break;
    }  
    put_flag(appstate->discharge_a);
    put_flag(appstate->discharge_b);
    put_flag(appstate->light_requested);
    put_uint16(appstate->charge_a_value);
    put_uint16(appstate->charge_b_value);
    put_uint8(appstate->dynamo_frequency);
    uart_putchar('\r');
    uart_putchar('\n');
#endif
}