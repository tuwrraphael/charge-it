#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdio.h>
#include <util/atomic.h>

#include "pinout.h"
#include "boolean.h"
#include "app_state.h"
#include "uart_debug.h"
#include "config.h"
#include "moving_average.h"
#include "convert_to_dynamo_rpm.h"
#include "update_state.h"

static moving_average_t deceleration_moving_average;

static appstate_t appstate = {
	.charge_mode = CHARGE_NONE,
	.discharge_a = FALSE,
	.discharge_b = FALSE,
	.light_requested = FALSE,
	.dynamo_shutoff = FALSE,
	.charge_a_value = 0,
	.charge_b_value = 0,
	.dynamo_frequency = 0,
	.max_discharge_value = MIN_CHARGE_VALUE,
	.driving_state = DRIVING_STATE_STOPPED,
	.is_braking = FALSE,
	.avg = 0,
	.driving_state_timing = 0,
	.cycle_count = 0,
	.back_off = 0,
	.max_charge_a_value = 0,
	.max_charge_b_value = 0,
	.turn_on_limit_us = TURN_ON_LIMIT_MAX};

#define ADMUX_BASE ((1 << REFS0) | (1 << REFS1))

static volatile boolean_t next_measurement_a = TRUE;

static uint16_t edge_before = 0;
static uint16_t frequency_measurement = 0;

static volatile uint8_t task_flags = 0;

#define FREQUENCY_MEASUREMENT_TASK_FLAG (1 << 0)
#define ADC_TASK_FLAG (1 << 1)
#define TIMER_TASK_FLAG (1 << 2)

static volatile boolean_t edge_before_valid = TRUE;

static boolean_t app_power_save = TRUE;
static volatile boolean_t leave_power_save_requested = TRUE;
uint8_t app_power_save_count = 0;

static void adc_measure()
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		task_flags &= ~ADC_TASK_FLAG;
	}
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
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADIE);
	next_measurement_a = TRUE;
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC))
		;
	task_flags &= ~ADC_TASK_FLAG;
}

static void timer1_init(boolean_t power_save)
{
	TCNT1 = 0;
	uint8_t timer_clk_src;
	if (power_save)
	{
		timer_clk_src = (1 << CS10) | (1 << CS12);
		TIMSK |= (1 << TICIE1);
		TIMSK &= ~(1 << TOIE1);
	}
	else
	{
		timer_clk_src = (1 << CS11);
		TIMSK |= (1 << TICIE1) | (1 << TOIE1);
	}
	TCCR1B = (1 << ICES1) | timer_clk_src | (1 << ICNC1);
}

static void timer2_init()
{
	TCCR2 = (1 << CS22);
	OCR2 = DIM_LIGHT_OCR;
	TIMSK |= (1 << TOIE2) | (1 << OCIE2);
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
			  (1 << LED_LIGHT_REQUEST_PIN));

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

	DDRD &= ~(1 << LED_LIGHT_REQUEST_PIN);

	PORTB &= ~((1 << DYNAMO_OFF_PIN) | (1 << SENSE_DYNAMO_PIN));
	DDRB |= ((1 << DYNAMO_OFF_PIN));
	DDRB &= ~((1 << SENSE_DYNAMO_PIN));

#ifdef LABBENCH
	PORTB |= ((1 << SCK_PIN) |
			  (1 << MOSI_PIN));
	PORTB &= ~(1 << MISO_PIN);
	DDRB &= ~((1 << MOSI_PIN) | (1 << SCK_PIN));
	DDRB |= (1 << MISO_PIN);
#endif
}

static void apply_state()
{

	if (appstate.dynamo_shutoff && (PORTB & (1 << DYNAMO_OFF_PIN)) == 0)
	{
		PORTB |= (1 << DYNAMO_OFF_PIN);
	}

	uint8_t new_portc = PORTC;

	if (appstate.discharge_a)
	{
		new_portc &= ~(1 << DISCHARGE_A_OFF_PIN);
	}
	else
	{
		new_portc |= (1 << DISCHARGE_A_OFF_PIN);
	}

	if (appstate.discharge_b)
	{
		new_portc &= ~(1 << DISCHARGE_B_OFF_PIN);
	}
	else
	{
		new_portc |= (1 << DISCHARGE_B_OFF_PIN);
	}

	if (appstate.charge_mode == CHARGE_A)
	{
		new_portc |= (1 << CHARGE_A_ON_PIN);
		new_portc &= ~(1 << CHARGE_B_ON_PIN);
	}
	else if (appstate.charge_mode == CHARGE_B)
	{
		new_portc |= (1 << CHARGE_B_ON_PIN);
		new_portc &= ~(1 << CHARGE_A_ON_PIN);
	}
	else
	{
		new_portc &= ~(1 << CHARGE_A_ON_PIN);
		new_portc &= ~(1 << CHARGE_B_ON_PIN);
	}
	PORTC = new_portc;

	if (!appstate.dynamo_shutoff && (PORTB & (1 << DYNAMO_OFF_PIN)) > 0)
	{
		PORTB &= ~(1 << DYNAMO_OFF_PIN);
		if (appstate.turn_on_limit_us < TURN_ON_LIMIT_MAX)
		{
			OCR1B = TCNT1 + appstate.turn_on_limit_us;
			TIMSK |= (1 << OCIE1B);
		}
	}

	if (appstate.driving_state == DRIVING_STATE_DRIVING || appstate.driving_state == DRIVING_STATE_STARTING)
	{
		PORTD &= ~(1 << REG5V_OFF_PIN);
	}
	else
	{
		PORTD |= (1 << REG5V_OFF_PIN);
	}

	if (appstate.light_requested && appstate.driving_state == DRIVING_STATE_DRIVING)
	{
		PORTD &= ~(1 << LED_FRONT_OFF_PIN);
	}
	else if (!appstate.light_requested || appstate.driving_state != DRIVING_STATE_STOPPING)
	{
		PORTD |= (1 << LED_FRONT_OFF_PIN);
	}
#ifndef DEBUGUART
	if (appstate.is_braking)
	{
		PORTD &= ~(1 << LED_BACK_OFF_PIN);
	}
	else if (appstate.light_requested == FALSE)
	{
		PORTD |= (1 << LED_BACK_OFF_PIN);
	}
#endif
}

static void read_inputs()
{
	appstate.light_requested = 0 == (PIND & (1 << LED_LIGHT_REQUEST_PIN));
}

int main(void)
{
#ifdef DEBUGUART
	uart_debug_init();
#endif
	cli();
	io_init();
	timer2_init();
	moving_average_init(&deceleration_moving_average);
	apply_state();
	sei();
	debug_appstate(&appstate);
	while (TRUE)
	{
		// PORTB |= (1 << MISO_PIN);
		if (app_power_save)
		{
#ifdef LABBENCH
			if (!(0 == (PINB & (1 << MOSI_PIN))))
#else
			if (!leave_power_save_requested)
#endif
			{
				read_inputs();
				boolean_t app_timer_elapsed = FALSE;
				if (task_flags & TIMER_TASK_FLAG)
				{
					app_timer_elapsed = TRUE;
					ATOMIC_BLOCK(ATOMIC_FORCEON)
					{
						task_flags &= ~TIMER_TASK_FLAG;
					}
				}
				update_state_powersave(&appstate, app_timer_elapsed);
				apply_state();
				debug_powersave(&appstate);
			}
			else
			{
				cli();
				app_power_save = FALSE;
				app_power_save_count = 0;
				adc_init();
				timer1_init(FALSE);
				sei();
				adc_measure();
			}
		}
		else
		{
			if (task_flags & ADC_TASK_FLAG)
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
			read_inputs();
			if (task_flags & FREQUENCY_MEASUREMENT_TASK_FLAG)
			{
				moving_average_add(&deceleration_moving_average, convert_to_dynamo_rpm(appstate.dynamo_frequency, frequency_measurement));
				appstate.avg = deceleration_moving_average.avg;
				appstate.dynamo_frequency = frequency_measurement;
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					task_flags &= ~FREQUENCY_MEASUREMENT_TASK_FLAG;
				}
			}
			boolean_t app_timer_elapsed = FALSE;
			if (task_flags & TIMER_TASK_FLAG)
			{
				app_timer_elapsed = TRUE;
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					task_flags &= ~TIMER_TASK_FLAG;
				}
			}
			update_state(&appstate, app_timer_elapsed);
			apply_state();
			if (appstate.driving_state == DRIVING_STATE_STOPPING)
			{
				if (app_power_save_count > 5)
				{
					cli();
					app_power_save = TRUE;
					leave_power_save_requested = FALSE;
					ADCSRA = 0;
					timer1_init(TRUE);
					sei();
				}
				else if (app_timer_elapsed == TRUE)
				{
					app_power_save_count++;
				}
			}
			else
			{
				app_power_save_count = 0;
			}
			// PORTB &= ~(1 << MISO_PIN);
			debug_summary(&appstate);
		}
		if (task_flags == 0)
		{
			set_sleep_mode(SLEEP_MODE_IDLE);
			sleep_mode();
		}
	}
	return 0;
}

ISR(ADC_vect)
{
	task_flags |= ADC_TASK_FLAG;
}

ISR(TIMER1_CAPT_vect)
{
	if (app_power_save)
	{
		leave_power_save_requested = TRUE;
	}
	else
	{
		uint16_t newVal = ICR1L;
		newVal |= (ICR1H << 8);
		if (edge_before_valid)
		{
			frequency_measurement = newVal - edge_before;
			task_flags |= frequency_measurement;
		}
		edge_before_valid = TRUE;
		edge_before = newVal;
	}
}

ISR(TIMER1_COMPB_vect)
{
	PORTB |= (1 << DYNAMO_OFF_PIN);
	TIMSK |= (1 << OCIE1B);
}

ISR(TIMER1_OVF_vect)
{
	if (edge_before_valid == FALSE)
	{
		frequency_measurement = 0;
		task_flags |= frequency_measurement;
	}
	edge_before_valid = FALSE;
}

ISR(TIMER2_OVF_vect)
{
	task_flags |= TIMER_TASK_FLAG;
	if (appstate.light_requested)
	{
#ifndef DEBUGUART
		if (!appstate.is_braking)
		{
			PORTD &= ~(1 << LED_BACK_OFF_PIN);
		}
#endif
		if (appstate.driving_state == DRIVING_STATE_STOPPING)
		{
			PORTD &= ~(1 << LED_FRONT_OFF_PIN);
		}
	}
}

ISR(TIMER2_COMP_vect)
{
	if (appstate.light_requested)
	{
#ifndef DEBUGUART
		if (!appstate.is_braking)
		{
			PORTD |= (1 << LED_BACK_OFF_PIN);
		}
		if (appstate.driving_state == DRIVING_STATE_STOPPING)
		{
			PORTD |= (1 << LED_FRONT_OFF_PIN);
		}
#endif
	}
}