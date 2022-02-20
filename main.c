#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdio.h>

#include "pinout.h"
#include "boolean.h"
#include "app_state.h"
#include "uart_debug.h"

static appstate_t appstate = {
	.charge_mode = CHARGE_NONE,
	.discharge_a = FALSE,
	.discharge_b = FALSE,
	.light_requested = FALSE,
	.charge_a_value = 0,
	.charge_b_value = 0,
	.dynamo_frequency = 0};

#define ADMUX_BASE ((1 << REFS0) | (1 << REFS1))

static boolean_t next_measurement_a = TRUE;
static boolean_t adc_done = FALSE;

static void adc_measure()
{
	adc_done = FALSE;
	if (next_measurement_a)
	{
		ADMUX &= ~(1 << MUX0);
	}
	else
	{
		ADMUX |= (1 << MUX0);
	}
	ADCSRA |= (1 << ADSC);
}

static void adc_init()
{
	ADMUX = ADMUX_BASE;
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1<<ADIE);
	next_measurement_a = TRUE;
	adc_done = FALSE;
}

static void io_init()
{
	// PORTC = 0;
	// DDRC = ((1 << CHARGE_A_ON_PIN) |
	// 		(1 << CHARGE_B_ON_PIN) |
	// 		(1 << DISCHARGE_A_OFF_PIN) |
	// 		(1 << DISCHARGE_B_OFF_PIN));

	// PORTD = ((1 << REG5V_OFF_PIN) |
	// 		 (1 << LED_BACK_OFF_PIN) |
	// 		 (1 << LED_FRONT_ON_PIN) |
	// 		 (1 << LED_LIGHT_CHARGE_PIN));
	// DDRD = ((1 << REG5V_OFF_PIN) |
	// 		(1 << LED_BACK_OFF_PIN) |
	// 		(1 << LED_FRONT_ON_PIN));

	// PORTB = 0;
	// DDRB = ((1 << DYNAMO_OFF_PIN));

	// PORTC &= ~(1 << CHARGE_A_VALUE_PIN | 1 << CHARGE_B_VALUE_PIN);
	// DDRC |= (0 << CHARGE_A_VALUE_PIN) |
	// 		(0 << CHARGE_B_VALUE_PIN);
}

int main(void)
{
#ifdef DEBUGUART
	uart_debug_init();
#endif
	// appstate.charge_mode = CHARGE_B;
	io_init();
	adc_init();
	sei();
	adc_measure();
	debug_appstate(&appstate);
	while (TRUE)
	{
		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_mode();
		if (adc_done == TRUE)
		{
			if (next_measurement_a)
			{
				appstate.charge_a_value = (ADCL | (ADCH << 8));
				next_measurement_a = FALSE;
			}
			else
			{
				appstate.charge_b_value = (ADCL | (ADCH << 8));
				next_measurement_a = TRUE;
			}
			adc_measure();
			debug_appstate(&appstate);
		}
		
	}
	return 0;
}

ISR(ADC_vect)
{
	adc_done = TRUE;
}