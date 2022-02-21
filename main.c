#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdio.h>

#include "pinout.h"
#include "boolean.h"
#include "app_state.h"
#include "uart_debug.h"
#include "config.h"

static appstate_t appstate = {
	.charge_mode = CHARGE_NONE,
	.discharge_a = FALSE,
	.discharge_b = FALSE,
	.light_requested = FALSE,
	.dynamo_shutoff = FALSE,
	.charge_a_value = 0,
	.charge_b_value = 0,
	.dynamo_frequency = 0,
	.max_charge_value = MAX_CHARGE_VALUE};

#define ADMUX_BASE ((1 << REFS0) | (1 << REFS1))

static volatile boolean_t next_measurement_a = TRUE;
static volatile boolean_t adc_done = FALSE;

static uint16_t edge_before = 0;
static uint16_t frequency_measurement = 0;
static boolean_t frequency_measurement_done = FALSE;
static volatile boolean_t edge_before_valid = TRUE;
static uint8_t overflow_ctr = 0;
static volatile boolean_t timer_overflow = FALSE;

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
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADIE);
	next_measurement_a = TRUE;
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	adc_done = FALSE;
}

static void timer1_init()
{
	TCCR1B |= (1 << ICES1) | (1 << CS11) | (1 << ICNC1);
	TIMSK |= (1 << TOIE1) | (1 << TICIE1);
}

static void timer0_init()
{
	TCCR0 = (1 << CS01) | (1 << CS00);
	TIMSK |= (1 << TOIE0);
}

static void io_init()
{
	PORTC &= ~((1 << CHARGE_A_ON_PIN) |
			   (1 << CHARGE_B_ON_PIN) |
			   (1 << DISCHARGE_A_OFF_PIN) |
			   (1 << DISCHARGE_B_OFF_PIN));
	DDRC = ((1 << CHARGE_A_ON_PIN) |
			(1 << CHARGE_B_ON_PIN) |
			(1 << DISCHARGE_A_OFF_PIN) |
			(1 << DISCHARGE_B_OFF_PIN));

	PORTD |= ((1 << LED_FRONT_OFF_PIN) |
			  (1 << LED_LIGHT_CHARGE_PIN));

	PORTD &= ~(1 << REG5V_OFF_PIN);

#ifndef DEBUGUART
	PORTD |= (1 << LED_BACK_OFF_PIN);
#endif

	DDRD |= ((1 << REG5V_OFF_PIN) |
			 (1 << LED_BACK_OFF_PIN) |
			 (1 << LED_FRONT_OFF_PIN));

#ifdef DEBUGUART
	DDRD |= (1 << LED_BACK_OFF_PIN);
#endif

	DDRD &= ~(1 << LED_LIGHT_CHARGE_PIN);

	PORTB &= ~((1 << DYNAMO_OFF_PIN) | (1 << SENSE_DYNAMO_PIN));
	DDRB |= ((1 << DYNAMO_OFF_PIN));
	DDRB &= ~((1 << SENSE_DYNAMO_PIN));
}

static boolean_t is_driving()
{
	// return appstate.dynamo_frequency > 0;
	return TRUE;
}

static boolean_t is_charging()
{
	return appstate.charge_mode == CHARGE_A || appstate.charge_mode == CHARGE_B;
}

static void decrease_max_charge_value()
{
	if (appstate.max_charge_value > MIN_CHARGE_VALUE)
	{
		appstate.max_charge_value -= CHARGE_VOLTAGE_STEP;
	}
}

static void increase_max_charge_value()
{
	if (appstate.max_charge_value < MAX_CHARGE_VALUE)
	{
		appstate.max_charge_value += CHARGE_VOLTAGE_STEP;
	}
}

static void update_state()
{
	boolean_t changed = FALSE;
	if (appstate.charge_mode == CHARGE_A)
	{
		if (appstate.charge_a_value > appstate.max_charge_value)
		{
			if (appstate.charge_b_value > appstate.max_charge_value)
			{
				appstate.charge_mode = CHARGE_NONE;
				changed = TRUE;
			}
			else
			{
				appstate.charge_mode = CHARGE_B;
				changed = TRUE;
			}
		}
	}
	else if (appstate.charge_mode == CHARGE_B)
	{
		if (appstate.charge_b_value > appstate.max_charge_value)
		{
			if (appstate.charge_a_value > appstate.max_charge_value)
			{
				appstate.charge_mode = CHARGE_NONE;
				changed = TRUE;
			}
			else
			{
				appstate.charge_mode = CHARGE_A;
				changed = TRUE;
			}
		}
	}
	else
	{
		if (appstate.charge_a_value < appstate.max_charge_value)
		{
			appstate.charge_mode = CHARGE_A;
			changed = TRUE;
		}
		else if (appstate.charge_b_value < appstate.max_charge_value)
		{
			appstate.charge_mode = CHARGE_B;
			changed = TRUE;
		}
	}
	if (changed)
	{
		overflow_ctr = 0;
	}
	else if (overflow_ctr > 20 && is_driving() && is_charging())
	{
		overflow_ctr = 0;
		decrease_max_charge_value();
	}
	else if (overflow_ctr > DYNAMO_FREQUENCY_STEP_TIMING && is_driving() && !is_charging())
	{
		overflow_ctr = 0;
		increase_max_charge_value();
	}
	appstate.discharge_b = (appstate.charge_mode == CHARGE_A || appstate.charge_mode == CHARGE_NONE) &&
						   appstate.charge_b_value > MAX_DISCHARGE_VALUE;
	appstate.discharge_a = (appstate.charge_mode == CHARGE_B || appstate.charge_mode == CHARGE_NONE) &&
						   appstate.charge_a_value > MAX_DISCHARGE_VALUE;
	appstate.dynamo_shutoff = !is_charging() && appstate.dynamo_frequency < DYNAMO_FREQUENCY_DANGER_VOLTAGE;
}

static void apply_state()
{
	if (appstate.discharge_a)
	{
		PORTC &= ~(1 << DISCHARGE_A_OFF_PIN);
	}
	else
	{
		PORTC |= (1 << DISCHARGE_A_OFF_PIN);
	}
	if (appstate.discharge_b)
	{
		PORTC &= ~(1 << DISCHARGE_B_OFF_PIN);
	}
	else
	{
		PORTC |= (1 << DISCHARGE_B_OFF_PIN);
	}
	if (appstate.charge_mode == CHARGE_A)
	{
		PORTC &= ~(1 << CHARGE_B_ON_PIN);
		PORTC |= (1 << CHARGE_A_ON_PIN);
	}
	else if (appstate.charge_mode == CHARGE_B)
	{
		PORTC &= ~(1 << CHARGE_A_ON_PIN);
		PORTC |= (1 << CHARGE_B_ON_PIN);
	}
	else
	{
		PORTC &= ~((1 << CHARGE_A_ON_PIN) | (1 << CHARGE_B_ON_PIN));
	}
	if (appstate.dynamo_shutoff)
	{
		PORTB |= (1 << DYNAMO_OFF_PIN);
	}
	else
	{
		PORTB &= ~(1 << DYNAMO_OFF_PIN);
	}
}

int main(void)
{
#ifdef DEBUGUART
	uart_debug_init();
#endif
	cli();
	io_init();
	adc_init();
	timer1_init();
	timer0_init();
	sei();
	adc_measure();
	debug_appstate(&appstate);
	while (TRUE)
	{
		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_mode();
		if (adc_done == TRUE)
		{
			if (ADMUX & (1 << MUX0))
			{
				appstate.charge_b_value = ADC;
			}
			else
			{
				appstate.charge_a_value = ADC;
			}
			next_measurement_a = !next_measurement_a;
			adc_measure();
		}
		if (frequency_measurement_done == TRUE)
		{
			appstate.dynamo_frequency = frequency_measurement;
			frequency_measurement_done = FALSE;
		}
		if (timer_overflow)
		{
			overflow_ctr++;
			timer_overflow = FALSE;
		}
		update_state();
		apply_state();
		// if (++ctr == 0) {
		debug_appstate(&appstate);
		// }
	}
	return 0;
}

ISR(ADC_vect)
{
	adc_done = TRUE;
}

ISR(TIMER1_CAPT_vect)
{
	uint16_t newVal = ICR1L;
	newVal |= (ICR1H << 8);
	if (edge_before_valid)
	{
		frequency_measurement = newVal - edge_before;
		frequency_measurement_done = TRUE;
	}
	edge_before_valid = TRUE;
	edge_before = newVal;
}

ISR(TIMER1_OVF_vect)
{
	if (edge_before_valid == FALSE)
	{
		frequency_measurement = 0;
		frequency_measurement_done = TRUE;
	}
	edge_before_valid = FALSE;
}

ISR(TIMER0_OVF_vect)
{
	timer_overflow = TRUE;
}