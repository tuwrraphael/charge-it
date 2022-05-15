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

static appstate_t appstate = {
	.charge_mode = CHARGE_NONE,
	.discharge_a = FALSE,
	.discharge_b = FALSE,
	.light_requested = FALSE,
	.dynamo_shutoff_enable = FALSE,
	.charge_a_value = 0,
	.max_discharge_value = MAX_CHARGE_VALUE - CHARGE_DISCHARGE_SPAN,
	.driving_state = DRIVING_STATE_STOPPED,
	.braking_timing = 0,
	.is_braking = 0,
	.avg = 0,
	.driving_state_timing = 0,
	.cycle_count = 0,
	.back_off = 0,
	.max_charge_a_value = 0,
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
	.scan_mode = TRUE,
	.overvoltage_timing = 0,
	.limits_exceeded = 0,
	.speed_falling_ctr = 0};

#ifdef USE_SIMULATION
static volatile uint8_t debugstate[20] __attribute__((section(".mysection")));
#endif

#define ADMUX_BASE ((1 << REFS0) | (1 << REFS1))

static volatile boolean_t last_measurement_b = TRUE;

static volatile uint8_t pulse_counter = 0;
static volatile uint16_t first_pulse_time = 0;
static volatile uint16_t last_pulse_time = 0;
static volatile uint8_t last_pulse_repetition = 0;
static volatile uint8_t first_pulse_repetition = 0;
static volatile int16_t repetition_ctr;
static volatile uint8_t pulses = 0;
static volatile uint16_t pulses_time = 0;

static volatile uint8_t task_flags;

#define FREQUENCY_MEASUREMENT_TASK_FLAG (1 << 0)
#define TIMER_TASK_FLAG (1 << 2)
#define ADC_TASK_FLAG (1 << 3)

static boolean_t app_power_save = TRUE;
static volatile boolean_t leave_power_save_requested = TRUE;
uint8_t app_power_save_count = 0;

static void adc_measure(void)
{
	ADCSRA |= (1 << ADSC);
}

static void adc_init(void)
{
	ADMUX = ADMUX_BASE;
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
	last_measurement_b = TRUE;
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC))
		;
	ADCSRA |= (1 << ADIE);
}

static void timer1_init(boolean_t power_save)
{
	OCR1A = PWM_DUTY_CYCLE_MAX;
	OCR1B = PWM_DUTY_CYCLE_MAX;
	TCNT1 = 0;
	uint8_t timer_clk_src;
	if (power_save)
	{
		timer_clk_src = (1 << CS10) | (1 << CS12);
#ifdef USE_SIMULATION
		TIMSK1 |= (1 << ICIE1);
		TIMSK1 &= ~((1 << TOIE1) | (1 << OCIE1A) | (1 << OCIE1B));
#else
		TIMSK |= (1 << TICIE1);
		TIMSK &= ~((1 << TOIE1) | (1 << OCIE1A) | (1 << OCIE1B));
#endif
		TCCR1A = 0;
	}
	else
	{
		timer_clk_src = (1 << CS11);
#ifdef USE_SIMULATION
		TIMSK1 |= (1 << ICIE1) | (1 << TOIE1) | (1 << OCIE1A) | (1 << OCIE1B);
#else
		TIMSK |= (1 << TICIE1) | (1 << TOIE1) | (1 << OCIE1A) | (1 << OCIE1B);
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
				// adc_measure();
			}
		}
		else
		{
			if (task_flags & ADC_TASK_FLAG)
			{
				if (last_measurement_b)
				{
					appstate.output_voltage_measurement_count++;
					appstate.output_voltage_sum += ADC_TO_MV_OUTPUT(ADC) / 100;
				}
				else
				{
					appstate.charge_a_value = ADC;
				}
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					task_flags &= ~ADC_TASK_FLAG;
				}
			}
			read_inputs();
			if (task_flags & FREQUENCY_MEASUREMENT_TASK_FLAG)
			{
				if (pulses > 0)
				{
					uint32_t denom = (F_CPU / TIMER1_PRESCALER) / pulses_time;
					appstate.dynamo_frequency = pulses * (uint16_t)denom;
				}
				else
				{
					appstate.dynamo_frequency = 0;
				}
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
		debugstate[15] = appstate.is_braking & 0xFF;
		debugstate[16] = (appstate.is_braking >> 8) & 0xFF;
		debugstate[17] = appstate.mppt_step_down_size;
#endif
		if (task_flags == 0)
		{
			set_sleep_mode(SLEEP_MODE_IDLE);
			sleep_mode();
		}
	}
	return 0;
}

static volatile boolean_t mux_changed = FALSE;

ISR(ADC_vect, ISR_BLOCK)
{
	last_measurement_b = ADMUX & (1 << MUX0);
	if (last_measurement_b)
	{
		ADMUX = ADMUX_BASE;
	}
	else
	{
		ADMUX = ADMUX_BASE | (1 << MUX0);
	}
	mux_changed = TRUE;
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
		last_pulse_time = ICR1;
#ifdef USE_SIMULATION
		last_pulse_repetition = (last_pulse_time < TIMER1_TOP / 2 && TIFR1 & (1 << TOV1)) ? repetition_ctr + 1 : repetition_ctr;
#else
		last_pulse_repetition = (last_pulse_time < TIMER1_TOP / 2 && TIFR & (1 << TOV1)) ? repetition_ctr + 1 : repetition_ctr;
#endif
		if (pulse_counter == 0)
		{
			first_pulse_time = last_pulse_time;
			first_pulse_repetition = last_pulse_repetition;
		}
		pulse_counter++;
	}
}

ISR(TIMER1_COMPA_vect, ISR_BLOCK)
{
	PORTC |= (1 << DISCHARGE_A_OFF_PIN | 1 << DISCHARGE_B_OFF_PIN);
}

ISR(TIMER1_COMPB_vect, ISR_BLOCK)
{
	if (appstate.dynamo_shutoff_enable)
	{
		PORTB |= (1 << DYNAMO_OFF_PIN);
	}
}

ISR(TIMER1_OVF_vect, ISR_BLOCK)
{
	PORTB &= ~(1 << DYNAMO_OFF_PIN);
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
	repetition_ctr++;
	if (repetition_ctr >= FAST_FREQ_MEAS_REVS && pulse_counter > 1)
	{
		repetition_ctr = 0;
		uint16_t repetitions = last_pulse_repetition - first_pulse_repetition - 1;
		pulses_time = (TIMER1_TOP - first_pulse_time) + (repetitions * TIMER1_TOP) + last_pulse_time;
		pulses = pulse_counter - 1;
		task_flags |= FREQUENCY_MEASUREMENT_TASK_FLAG;
		pulse_counter = 0;
	}
	else if (repetition_ctr >= FREQ_MEAS_LIMIT_REVS)
	{
		repetition_ctr = 0;
		pulse_counter = 0;
		pulses = 0;
		task_flags |= FREQUENCY_MEASUREMENT_TASK_FLAG;
	}
}

ISR(TIMER2_OVF_vect, ISR_BLOCK)
{
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
	if (mux_changed)
	{
		mux_changed = FALSE;
	}
	else if ((task_flags & ADC_TASK_FLAG) == 0 && (ADCSRA & (1 << ADSC)) == 0)
	{
		adc_measure();
	}
	if (pulse_counter > 0)
	{
	}
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