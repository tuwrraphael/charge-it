#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
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
#include "charge_current.h"

static moving_average_t deceleration_moving_average;

static appstate_t appstate = {
	.charge_mode = CHARGE_NONE,
	.discharge_a = FALSE,
	.discharge_b = FALSE,
	.light_requested = FALSE,
	.dynamo_shutoff = FALSE,
	.charge_a_value = 0,
	.max_discharge_value = MAX_CHARGE_VALUE - CHARGE_DISCHARGE_SPAN,
	.driving_state = DRIVING_STATE_STOPPED,
	.is_braking = 0,
	.avg = 0,
	.driving_state_timing = 0,
	.cycle_count = 0,
	.back_off = 0,
	.max_charge_a_value = 0,
	.max_charge_b_value = 0,
	.turn_on_limit = TURN_ON_LIMIT_MAX,
	.mppt_direction_down = TRUE,
	.mppt_step_timing = 0,
	.mppt_step_down_size = MAX_MPPT_STEP,
	.mppt_step_up_size = MAX_MPPT_STEP,
	.output_voltage_sum = 0,
	.output_voltage_measurement_count = 0,
	.output_voltage_step_before = 0,
	.search_seconds = MIN_SEARCH_SCAN_INTERVAL_SECONDS,
	.search_seconds_count = 0,
	.search_seconds_counter = 0,
	.last_search_result = PWM_DUTY_CYCLE_MAX,
	.scan_mode = TRUE
};

#ifdef USE_SIMULATION
static volatile uint8_t debugstate[20] __attribute__((section(".mysection")));
#endif

#define ADMUX_BASE ((1 << REFS0) | (1 << REFS1))

static volatile boolean_t next_measurement_a = TRUE;

static uint16_t edge_before = 0;
static uint16_t frequency_measurement = 0;

static volatile uint8_t task_flags;

#define FREQUENCY_MEASUREMENT_TASK_FLAG (1 << 0)
#define TIMER_TASK_FLAG (1 << 2)
#define ADC_TASK_FLAG (1 << 3)

static volatile boolean_t edge_before_valid = TRUE;

static boolean_t app_power_save = TRUE;
static volatile boolean_t leave_power_save_requested = TRUE;
uint8_t app_power_save_count = 0;

static void adc_measure(void)
{
	if (next_measurement_a)
	{
		ADMUX &= ~(1 << MUX0);
	}
	else
	{
		ADMUX |= (1 << MUX0);
	}
	// while (ADCSRA & (1 << ADSC))
	// 	;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		task_flags &= ~(1 << ADC_TASK_FLAG);
	}
	ADCSRA |= (1 << ADSC);
}

static void adc_init(void)
{
	ADMUX = ADMUX_BASE;
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0) | (1 << ADPS1);
	next_measurement_a = TRUE;
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC))
		;
	ADCSRA |= (1 << ADIE);
}

static void timer1_init(boolean_t power_save)
{
	TCNT1 = 0;
	uint8_t timer_clk_src;
	if (power_save)
	{
		timer_clk_src = (1 << CS10) | (1 << CS12);
#ifdef USE_SIMULATION
		TIMSK1 |= (1 << ICIE1);
		TIMSK1 &= ~(1 << TOIE1);
#else
		TIMSK |= (1 << TICIE1);
		TIMSK &= ~(1 << TOIE1);
#endif
		TCCR1A = 0;
	}
	else
	{
		timer_clk_src = (1<< CS11);
#ifdef USE_SIMULATION
		TIMSK1 |= (1 << ICIE1) | (1 << TOIE1) | (1 << OCIE1A);
#else
		TIMSK |= (1 << TICIE1) | (1 << TOIE1) | (1 << OCIE1A);
#endif
		TCCR1A = (1 << WGM11);
		OCR1A = PWM_DUTY_CYCLE_MAX;
	}
	TCCR1B = (1 << ICES1) | timer_clk_src | (1 << ICNC1) | (1 << WGM12);
}

static void timer2_init(void)
{
#ifdef USE_SIMULATION
	TCCR2B = (1 << CS22);
	OCR2B = DIM_LIGHT_OCR;
	TIMSK2 |= (1 << TOIE2) | (1 << OCIE2B);
#else
	TCCR2 = (1 << CS22);
	OCR2 = DIM_LIGHT_OCR;
	TIMSK |= (1 << TOIE2) | (1 << OCIE2);
#endif
}

static void io_init(void)
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

static void apply_state(void)
{

	if (appstate.dynamo_shutoff && (PORTB & (1 << DYNAMO_OFF_PIN)) == 0)
	{
		PORTB |= (1 << DYNAMO_OFF_PIN);
	}

	uint8_t new_portc = PORTC;

	// if (appstate.discharge_a)
	// {
	// 	new_portc &= ~(1 << DISCHARGE_A_OFF_PIN);
	// }
	// else
	// {
	// 	new_portc |= (1 << DISCHARGE_A_OFF_PIN);
	// }

	// if (appstate.discharge_b)
	// {
	// 	new_portc &= ~(1 << DISCHARGE_B_OFF_PIN);
	// }
	// else
	// {
	// 	new_portc |= (1 << DISCHARGE_B_OFF_PIN);
	// }

	// if (appstate.charge_mode == CHARGE_A)
	// {
	// 	new_portc |= (1 << CHARGE_A_ON_PIN);
	// 	new_portc &= ~(1 << CHARGE_B_ON_PIN);
	// }
	// else if (appstate.charge_mode == CHARGE_B)
	// {
	// 	new_portc |= (1 << CHARGE_B_ON_PIN);
	// 	new_portc &= ~(1 << CHARGE_A_ON_PIN);
	// }
	// else
	// {
	// 	new_portc &= ~(1 << CHARGE_A_ON_PIN);
	// 	new_portc &= ~(1 << CHARGE_B_ON_PIN);
	// }
	PORTC = new_portc;

	if (!appstate.dynamo_shutoff && (PORTB & (1 << DYNAMO_OFF_PIN)) > 0)
	{
		PORTB &= ~(1 << DYNAMO_OFF_PIN);
		if (appstate.turn_on_limit < TURN_ON_LIMIT_MAX)
		{
			OCR1B = TCNT1 + appstate.turn_on_limit;
#ifdef USE_SIMULATION
			TIMSK1 |= (1 << OCIE1B);
#else
			TIMSK |= (1 << OCIE1B);
#endif
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
	if (appstate.is_braking > 0)
	{
		PORTD &= ~(1 << LED_BACK_OFF_PIN);
	}
	else if (appstate.light_requested == FALSE)
	{
		PORTD |= (1 << LED_BACK_OFF_PIN);
	}
#endif
}

static void read_inputs(void)
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
	moving_average_init(&appstate.output_voltage_noise_moving_average);
	appstate.output_voltage_noise_moving_average.calculate_variance = TRUE;
	apply_state();
	sei();
	debug_appstate(&appstate);
	while (TRUE)
	{
		// PORTB |= (1 << MISO_PIN);
		if (app_power_save)
		{
#ifdef LABBENCH
			if (FALSE)
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
				debug_powersave(&appstate);
				cli();
				app_power_save = FALSE;
				app_power_save_count = 0;
				adc_init();
				timer1_init(FALSE);
				sei();
				adc_measure();
				moving_average_init(&deceleration_moving_average);
			}
		}
		else
		{
			if (task_flags & ADC_TASK_FLAG)
			{
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					task_flags &= ~ADC_TASK_FLAG;
				}
				if (ADMUX & (1 << MUX0))
				{
					appstate.output_voltage_measurement_count++;
					appstate.output_voltage_sum += ADC_TO_MV_OUTPUT(ADC) / 100;
				}
				else
				{
					appstate.charge_a_value = ADC;
				}
			}
			read_inputs();
			if (task_flags & FREQUENCY_MEASUREMENT_TASK_FLAG)
			{
				appstate.diff = convert_to_dynamo_rpm(appstate.dynamo_frequency, frequency_measurement);
				if (appstate.diff < (10 * BRAKE_THRESHOLD_SCALE) && appstate.diff > (0 - (10 * BRAKE_THRESHOLD_SCALE)))
				{ // discard impossible outliers, 30 equals 12m/s^2 deceleration
					moving_average_add(&deceleration_moving_average, appstate.diff);
				}

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
			// if (appstate.charge_current_measurement_count == 99)
			// {
			// OCR1A = 50;
			// }
		}
#ifdef USE_SIMULATION
		debugstate[0] = appstate.driving_state;
		debugstate[1] = appstate.max_discharge_value & 0xFF;
		debugstate[2] = (appstate.max_discharge_value >> 8) & 0xFF;
		debugstate[4] = appstate.dynamo_frequency & 0xFF;
		debugstate[5] = (appstate.dynamo_frequency >> 8) & 0xFF;
		debugstate[3] = (appstate.output_voltage_measurement_count & 0xFF);
		debugstate[6] = appstate.output_voltage_sum & 0xFF;
		debugstate[7] = (appstate.output_voltage_sum >> 8) & 0xFF;
		// debugstate[8] = appstate.charge_current_measurement_sum & 0xFF;
		// debugstate[9] = (appstate.charge_current_measurement_sum >> 8) & 0xFF;
		debugstate[10] = OCR1A & 0xFF;
		debugstate[11] = (OCR1A >> 8) & 0xFF;
		debugstate[12] = appstate.output_voltage_step_before & 0xFF;
		debugstate[13] = (appstate.output_voltage_step_before >> 8) & 0xFF;
		debugstate[14] = appstate.mppt_direction_down;
		debugstate[15] = appstate.mppt_step_timing & 0xFF;
		debugstate[16] = (appstate.mppt_step_timing >> 8) & 0xFF;
		debugstate[17] = appstate.mppt_step_size;
#endif
		if (task_flags == 0)
		{
			set_sleep_mode(SLEEP_MODE_IDLE);
			sleep_mode();
		}
	}
	return 0;
}

ISR(ADC_vect, ISR_BLOCK)
{
	task_flags |= ADC_TASK_FLAG;
}

ISR(TIMER1_CAPT_vect, ISR_BLOCK)
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
			if (newVal > edge_before)
			{
				frequency_measurement = newVal - edge_before;
			}
			else
			{
				frequency_measurement = 65535 - edge_before + newVal;
			}
			task_flags |= FREQUENCY_MEASUREMENT_TASK_FLAG;
		}
		edge_before_valid = TRUE;
		edge_before = newVal;
	}
}

ISR(TIMER1_COMPA_vect, ISR_BLOCK)
{
	PORTC |= (1 << DISCHARGE_A_OFF_PIN | 1 << DISCHARGE_B_OFF_PIN);
}

ISR(TIMER1_COMPB_vect, ISR_BLOCK)
{
	PORTB |= (1 << DYNAMO_OFF_PIN);
#ifdef USE_SIMULATION
	TIMSK1 |= (1 << OCIE1B);
#else
	TIMSK |= (1 << OCIE1B);
#endif
}

ISR(TIMER1_OVF_vect, ISR_BLOCK)
{
	if (appstate.charge_mode != CHARGE_B)
	{
		appstate.charge_mode = CHARGE_B;
		PORTC = (PORTC & ~(1 << DISCHARGE_A_OFF_PIN | 1 << CHARGE_A_ON_PIN)) | (1 << DISCHARGE_B_OFF_PIN | 1 << CHARGE_B_ON_PIN);
	}
	else
	{
		appstate.charge_mode = CHARGE_A;
		PORTC = (PORTC & ~(1 << DISCHARGE_B_OFF_PIN | 1 << CHARGE_B_ON_PIN)) | (1 << DISCHARGE_A_OFF_PIN | 1 << CHARGE_A_ON_PIN);
	}
}

static boolean_t pressed;

ISR(TIMER2_OVF_vect, ISR_BLOCK)
{
	if (pressed && (PINB & (1 << MOSI_PIN)) == 0)
	{
		OCR1A -= 20;
		if (OCR1A > PWM_DUTY_CYCLE_MAX)
		{
			OCR1A = PWM_DUTY_CYCLE_MAX;
		}
	}
	pressed = PINB & (1 << MOSI_PIN);
	task_flags |= TIMER_TASK_FLAG;
	if (appstate.light_requested)
	{
#ifndef DEBUGUART
		if (appstate.is_braking == 0)
		{
			PORTD &= ~(1 << LED_BACK_OFF_PIN);
		}
#endif
		if (appstate.driving_state == DRIVING_STATE_STOPPING)
		{
			PORTD &= ~(1 << LED_FRONT_OFF_PIN);
		}
	}
	next_measurement_a = !next_measurement_a;
	adc_measure();
}

#ifdef USE_SIMULATION
ISR(TIMER2_COMPB_vect, ISR_BLOCK)
{
#else
ISR(TIMER2_COMP_vect, ISR_BLOCK)
{
#endif
	if (appstate.light_requested)
	{
#ifndef DEBUGUART
		if (appstate.is_braking == 0)
		{
			PORTD |= (1 << LED_BACK_OFF_PIN);
		}
#endif
		if (appstate.driving_state == DRIVING_STATE_STOPPING)
		{
			PORTD |= (1 << LED_FRONT_OFF_PIN);
		}
	}
}