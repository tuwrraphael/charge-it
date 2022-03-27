#include <avr/io.h>
#include <string.h>
#include <stdlib.h>
#include "uart_debug.h"
#define BAUD 115200UL
#include <util/setbaud.h>

#define SUMMARY_OUTPUT (255)

typedef struct
{
    uint16_t freq_avg;
    uint8_t cycle;
    uint8_t charge_a_count;
    uint8_t charge_b_count;
    uint8_t shutoff_count;
    uint8_t discharge_a_count;
    uint8_t discharge_b_count;
    int16_t max_charge_val_avg;
    uint16_t charge_a_max;
    uint16_t charge_a_min;
    uint16_t charge_b_max;
    uint16_t charge_b_min;
} debug_summary_t;

debug_summary_t debug_summary_data = {
    .cycle = 0,
    .charge_a_count = 0,
    .charge_b_count = 0,
    .shutoff_count = 0,
    .discharge_a_count = 0,
    .discharge_b_count = 0,
    .max_charge_val_avg = 0,
    .charge_a_max = 0,
    .charge_a_min = 0,
    .charge_b_max = 0,
    .charge_b_min = 0};

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

static void put_flag(boolean_t b)
{
    if (b)
    {
        uart_putchar('1');
    }
    else
    {
        uart_putchar('0');
    }
}

static void put_uint16(uint16_t nr)
{
    char buf[5];
    memset(buf, ' ', 5);
    utoa(nr, buf, 10);
    uart_putchar(buf[0]);
    uart_putchar(buf[1]);
    uart_putchar(buf[2]);
    uart_putchar(buf[3]);
    uart_putchar(buf[4]);
}

static void put_uint8(uint8_t nr)
{
    char buf[3];
    memset(buf, ' ', 3);
    utoa(nr, buf, 10);
    uart_putchar(buf[0]);
    uart_putchar(buf[1]);
    uart_putchar(buf[2]);
}

static void output_percentage(uint8_t val, uint8_t max)
{
    uint16_t p = (uint16_t)val * 100;
    uint8_t per = p / max;
    put_uint8(per);
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
    put_flag(appstate->dynamo_shutoff);
    uart_putchar(' ');
    // put_uint16(appstate->charge_count);
    // uart_putchar(' ');
    put_uint16(appstate->charge_a_value);
    put_uint16(appstate->charge_b_value);
    put_uint16(appstate->max_discharge_value);
    put_uint16(appstate->dynamo_frequency);
    uart_putchar(' ');
    put_uint8(appstate->driving_state);
    uart_putchar('\r');
    uart_putchar('\n');
#endif
}

void debug_summary(appstate_t *appstate)
{
#ifdef DEBUGUART
    if (debug_summary_data.cycle == SUMMARY_OUTPUT)
    {
        output_percentage(debug_summary_data.charge_a_count, debug_summary_data.cycle);
        uart_putchar(' ');
        output_percentage(debug_summary_data.charge_b_count, debug_summary_data.cycle);
        uart_putchar(' ');
        output_percentage(debug_summary_data.shutoff_count, debug_summary_data.cycle);
        uart_putchar(' ');
        output_percentage(debug_summary_data.discharge_a_count, debug_summary_data.cycle);
        uart_putchar(' ');
        output_percentage(debug_summary_data.discharge_b_count, debug_summary_data.cycle);
        uart_putchar(' ');
        put_uint16(debug_summary_data.max_charge_val_avg);
        uart_putchar(' ');
        put_uint16(debug_summary_data.charge_a_max);
        uart_putchar(' ');
        put_uint16(debug_summary_data.charge_a_min);
        uart_putchar(' ');
        put_uint16(debug_summary_data.charge_b_max);
        uart_putchar(' ');
        put_uint16(debug_summary_data.charge_b_min);
        uart_putchar(' ');
        put_uint16(debug_summary_data.freq_avg);
        uart_putchar('\r');
        uart_putchar('\n');
        debug_summary_data.cycle = 0;
        debug_summary_data.charge_a_count = 0;
        debug_summary_data.charge_b_count = 0;
        debug_summary_data.shutoff_count = 0;
        debug_summary_data.discharge_a_count = 0;
        debug_summary_data.discharge_b_count = 0;
        debug_summary_data.max_charge_val_avg = 0;
        debug_summary_data.charge_a_max = 0;
        debug_summary_data.charge_a_min = 999;
        debug_summary_data.charge_b_max = 0;
        debug_summary_data.charge_b_min = 999;
        debug_summary_data.freq_avg = 0;
    }
    debug_summary_data.cycle++;
    switch (appstate->charge_mode)
    {
    case CHARGE_NONE:
        debug_summary_data.shutoff_count++;
        break;
    case CHARGE_A:
        debug_summary_data.charge_a_count++;
        break;
    case CHARGE_B:
        debug_summary_data.charge_b_count++;
        break;
    }
    if (appstate->discharge_a)
    {
        debug_summary_data.discharge_a_count++;
    }
    if (appstate->discharge_b)
    {
        debug_summary_data.discharge_b_count++;
    }
    debug_summary_data.max_charge_val_avg += ((int16_t)appstate->max_discharge_value - (int16_t)debug_summary_data.max_charge_val_avg) / debug_summary_data.cycle;
    debug_summary_data.charge_a_max = appstate->charge_a_value > debug_summary_data.charge_a_max ? appstate->charge_a_value : debug_summary_data.charge_a_max;
    debug_summary_data.charge_b_max = appstate->charge_b_value > debug_summary_data.charge_b_max ? appstate->charge_b_value : debug_summary_data.charge_b_max;
    debug_summary_data.charge_a_min = appstate->charge_a_value < debug_summary_data.charge_a_min ? appstate->charge_a_value : debug_summary_data.charge_a_min;
    debug_summary_data.charge_b_min = appstate->charge_b_value < debug_summary_data.charge_b_min ? appstate->charge_b_value : debug_summary_data.charge_b_min;
    debug_summary_data.freq_avg += (appstate->dynamo_frequency - debug_summary_data.freq_avg) / debug_summary_data.cycle;
#endif
}

void debug_powersave(appstate_t *appstate)
{
#ifdef DEBUGUART
    uart_putchar('P');
    uart_putchar('\r');
    uart_putchar('\n');
#endif
}