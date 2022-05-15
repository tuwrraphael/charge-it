#include "update_state.h"
#include "uart_debug.h"
#include "boolean.h"
#include "app_state.h"
#include "config.h"
#include "moving_average.h"
#include "pinout.h"
#include <avr/io.h>

static boolean_t is_driving(appstate_t *appstate)
{
#ifndef LABBENCH
    return appstate->dynamo_frequency > 0;
#else
    return TRUE;
#endif
}

static boolean_t is_driving_fast(appstate_t *appstate)
{
#ifndef LABBENCH
    return appstate->dynamo_frequency > DYNAMO_FREQUENCY_DRIVING;
#else
    return TRUE;
#endif
}

static boolean_t driving_below_danger_voltage(appstate_t *appstate)
{
#ifndef LABBENCH
    return appstate->dynamo_frequency > DYNAMO_FREQUENCY_DANGER_VOLTAGE;
#else
    return TRUE;
#endif
}

static void update_driving_state(appstate_t *appstate, boolean_t app_timer_elapsed)
{
    switch (appstate->driving_state)
    {
    case DRIVING_STATE_STOPPED:
        if (is_driving(appstate))
        {
            appstate->driving_state = DRIVING_STATE_STARTING;
            appstate->driving_state_timing = 0;
        }
        break;
    case DRIVING_STATE_STARTING:
        if (appstate->driving_state_timing > DRIVING_THRESHOLD)
        {
            appstate->driving_state = DRIVING_STATE_DRIVING;
        }
        else if (app_timer_elapsed && is_driving_fast(appstate))
        {
            appstate->driving_state_timing++;
        }
        else if (!is_driving(appstate))
        {
            appstate->driving_state = DRIVING_STATE_STOPPED;
        }
        break;
    case DRIVING_STATE_DRIVING:
        if (!is_driving(appstate))
        {
            appstate->driving_state = DRIVING_STATE_STOPPING;
            appstate->driving_state_timing = 0;
        }
        break;
    case DRIVING_STATE_STOPPING:
        if (appstate->driving_state_timing > STOPPING_THRESHOLD)
        {
            appstate->driving_state = DRIVING_STATE_STOPPED;
        }
        else if (is_driving(appstate))
        {
            appstate->driving_state = DRIVING_STATE_DRIVING;
        }
        else if (app_timer_elapsed)
        {
            appstate->driving_state_timing++;
        }
    default:
        break;
    }
}

static void start_scan(appstate_t *appstate)
{
    appstate->scan_mode = TRUE;
    appstate->best_result = 0;
    appstate->sinking_ctr = 0;
    appstate->rising_ctr = 0;
    uint16_t search_start = (appstate->last_search_result * 40) / 100 + appstate->last_search_result;
    if (search_start > PWM_DUTY_CYCLE_MAX)
    {
        search_start = PWM_DUTY_CYCLE_MAX;
    }
    OCR1A = search_start;
}

void update_state(appstate_t *appstate,
                  boolean_t app_timer_elapsed)
{
    appstate->max_charge_a_value = appstate->charge_a_value > appstate->max_charge_a_value ? appstate->charge_a_value : appstate->max_charge_a_value;
    boolean_t is_in_overvoltage_protection = OCR1B < PWM_DUTY_CYCLE_MAX;
    if (app_timer_elapsed)
    {
        if (appstate->overvoltage_timing < OVERVOLTAGE_TIMING)
        {
            appstate->overvoltage_timing++;
        }
        else
        {
            appstate->overvoltage_timing = 0;
            appstate->overvoltage = appstate->max_charge_a_value > (is_in_overvoltage_protection ? MAX_CHARGE_VALUE_OVERVOLTAGE : MAX_CHARGE_VALUE);
            if (appstate->limits_exceeded < 8 && appstate->overvoltage)
            {
                appstate->limits_exceeded++;
            }
            if (!appstate->overvoltage && appstate->limits_exceeded > 0)
            {
                appstate->limits_exceeded--;
            }
            appstate->max_charge_a_value = 0;
        }
        if (appstate->is_braking > 0)
        {
            appstate->is_braking--;
        }
        if (appstate->braking_timing < BRAKING_TIMING)
        {
            appstate->braking_timing++;
        }
        else
        {
            debug_uint16(appstate->dynamo_frequency, appstate->is_braking);
            appstate->braking_timing = 0;
            int16_t frequency_diff = appstate->dynamo_frequency_before - appstate->dynamo_frequency;
            if (frequency_diff > 4)
            {
                if (appstate->speed_falling_ctr > 2) {
                appstate->is_braking = BRAKE_LIGHT_TIMING;
                }
                else {
                    appstate->speed_falling_ctr++;
                }
            } else {
                appstate->speed_falling_ctr = 0;
            }
            appstate->dynamo_frequency_before = appstate->dynamo_frequency;
        }
    }

    if (app_timer_elapsed && !appstate->scan_mode)
    {
        if (appstate->search_seconds_counter < SECONDS_TO_TIM2_REVS(1))
        {
            appstate->search_seconds_counter++;
        }
        else
        {
            appstate->search_seconds_counter = 0;
            appstate->search_seconds_count++;
            if (appstate->search_seconds_count >= appstate->search_seconds)
            {
                appstate->search_seconds_count = 0;
                // appstate->mppt_step_up_size = MAX_MPPT_STEP;
                // appstate->mppt_step_down_size = MAX_MPPT_STEP;
                // uint16_t diff = OCR1A > appstate->last_search_result ? OCR1A - appstate->last_search_result : appstate->last_search_result - OCR1A;
                if (!is_in_overvoltage_protection)
                {

                    start_scan(appstate);
                }
            }
        }
    }

    if (appstate->output_voltage_measurement_count >= 100)
    {
        appstate->output_voltage_measurement_count = 0;
        uint16_t mppt_exec = (appstate->scan_mode || appstate->limits_exceeded || is_in_overvoltage_protection) ? SECONDS_TO_TIM2_REVS(0.15) : MPPT_STEP;
        if (appstate->mppt_step_timing >= mppt_exec)
        {
            appstate->mppt_step_timing = 0;

            if (appstate->scan_mode)
            {
                if (appstate->output_voltage_noise_moving_average.avg > appstate->best_result && !appstate->limits_exceeded)
                {
                    appstate->best_result = appstate->output_voltage_noise_moving_average.avg;
                    appstate->best_result_at = OCR1A;
                }
                if (appstate->output_voltage_step_before > appstate->output_voltage_noise_moving_average.avg)
                {
                    appstate->sinking_ctr++;
                }
                else
                {
                    appstate->sinking_ctr = 0;
                    appstate->rising_ctr++;
                }
                boolean_t end_search = appstate->limits_exceeded || (appstate->sinking_ctr >= SINKING_THRESHOLD && appstate->rising_ctr >= RISING_THRESHOLD);
                if (!end_search && OCR1A >= 32)
                {
                    OCR1A -= SEARCH_STEP;
                }
                else
                {
                    appstate->scan_mode = FALSE;
                    if (end_search)
                    {
                        uint16_t new_ocr1a = OCR1A;
                        uint16_t singking_thres_mul = appstate->sinking_ctr > 0 ? appstate->sinking_ctr - 1 : 0;
                        new_ocr1a += (singking_thres_mul)*SEARCH_STEP + SEARCH_STEP / 2;
                        if (new_ocr1a > PWM_DUTY_CYCLE_MAX)
                        {
                            new_ocr1a = PWM_DUTY_CYCLE_MAX;
                        }
                        OCR1A = new_ocr1a;
                        uint16_t diff = new_ocr1a > appstate->last_search_result ? new_ocr1a - appstate->last_search_result : appstate->last_search_result - new_ocr1a;
                        if (diff < new_ocr1a / 10)
                        { // 10%
                            if (appstate->search_seconds < MAX_SEARCH_SCAN_INTERVAL_SECONDS)
                            {
                                appstate->search_seconds *= 2;
                            }
                        }
                        else
                        {
                            appstate->search_seconds = MIN_SEARCH_SCAN_INTERVAL_SECONDS;
                        }
                        appstate->last_search_result = new_ocr1a;
                    }
                    else
                    {
                        appstate->search_seconds = MIN_SEARCH_SCAN_INTERVAL_SECONDS;
                        appstate->last_search_result = PWM_DUTY_CYCLE_MAX;
                    }
                    appstate->last_search_result_voltage = appstate->output_voltage_noise_moving_average.avg;
                    appstate->mppt_step_down_size = MAX_MPPT_STEP;
                    appstate->mppt_step_up_size = MAX_MPPT_STEP;
                }
            }
            else
            {

                int16_t diff = appstate->output_voltage_step_before - appstate->output_voltage_noise_moving_average.avg;
                // uint32_t variance = moving_average_get_variance(&appstate->output_voltage_noise_moving_average);
                // int16_t diff_squared = diff * diff;
                // boolean_t is_significant = diff > 50;
                boolean_t switch_direction = diff > 0;
                if (switch_direction)
                {
                    if (appstate->mppt_step_down_size > MIN_MPPT_STEP)
                    {
                        appstate->mppt_step_down_size -= 1;
                        appstate->mppt_step_up_size -= 1;
                    }
                }
                else if ((diff < MIN_VOLTAGE_CHANGE && (0 - MIN_VOLTAGE_CHANGE) > diff) && appstate->mppt_step_down_size < MAX_MPPT_STEP)
                {
                    appstate->mppt_step_down_size += 1;
                    appstate->mppt_step_up_size += 1;
                }
                uint16_t ocra_step_size = appstate->mppt_step_down_size;
                boolean_t ov = appstate->overvoltage || is_in_overvoltage_protection;
                if (ov)
                {
                    switch_direction = appstate->mppt_direction_down;
                    ocra_step_size = 10;
                }
                boolean_t mppt_is_max = OCR1A >= (PWM_DUTY_CYCLE_MAX - ocra_step_size);
                boolean_t mppt_is_min = OCR1A <= ocra_step_size;

                if (!ov && ((appstate->mppt_direction_down && mppt_is_min) || (!appstate->mppt_direction_down && mppt_is_max)))
                {
                    switch_direction = TRUE;
                }

                if (switch_direction)
                {
                    appstate->mppt_direction_down = !appstate->mppt_direction_down;
                }

                if (appstate->mppt_direction_down)
                {
                    if (!mppt_is_min)
                    {
                        OCR1A -= ocra_step_size;
                    }
                    else
                    {
                        OCR1A = 1;
                    }
                }
                else
                {

                    if (!mppt_is_max)
                    {
                        OCR1A += ocra_step_size;
                    }
                    else
                    {
                        OCR1A = PWM_DUTY_CYCLE_MAX;
                    }
                }

                uint16_t deviation = appstate->last_search_result_voltage > appstate->output_voltage_noise_moving_average.avg ? appstate->last_search_result_voltage - appstate->output_voltage_noise_moving_average.avg : appstate->output_voltage_noise_moving_average.avg - appstate->last_search_result_voltage;
                if (deviation > 1000)
                {
                    // this will start scan after min seconds
                    appstate->search_seconds = MIN_SEARCH_SCAN_INTERVAL_SECONDS;
                }
                if (appstate->output_voltage_noise_moving_average.avg < 3000 && appstate->limits_exceeded)
                {
                    // if OCRA1 is very low but limits exceeded
                    appstate->search_seconds = MIN_SEARCH_SCAN_INTERVAL_SECONDS;
                    appstate->last_search_result = PWM_DUTY_CYCLE_MAX;
                }

                uint16_t step;
                if (OCR1B > 100)
                {
                    step = 40;
                }
                else if (OCR1B > 60)
                {
                    step = OCR1B * 2 / 100;
                }
                else
                {
                    step = 1;
                }
                step = step == 0 ? 1 : step;
                uint16_t newOcr1b = OCR1B;
                if (appstate->overvoltage)
                {
                    if (OCR1A > (PWM_DUTY_CYCLE_MAX - PWM_DUTY_CYCLE_MAX * 10 / 100))
                    {
                        if (newOcr1b > step)
                        {
                            newOcr1b -= step;
                        }
                    }
                }
                if (!appstate->limits_exceeded)
                {
                    if (OCR1B < PWM_DUTY_CYCLE_MAX)
                    {
                        newOcr1b += step;
                        if (newOcr1b > PWM_DUTY_CYCLE_MAX)
                        {
                            newOcr1b = PWM_DUTY_CYCLE_MAX;
                        }
                    }
                }
                OCR1B = newOcr1b;
            }
            appstate->output_voltage_step_before = appstate->output_voltage_noise_moving_average.avg;
        }
        moving_average_add(&appstate->output_voltage_noise_moving_average, appstate->output_voltage_sum);
        // debug_appstate(appstate);
        appstate->output_voltage_sum = 0;
    }

    appstate->dynamo_shutoff_enable = driving_below_danger_voltage(appstate);

    update_driving_state(appstate, app_timer_elapsed);

    if (app_timer_elapsed && appstate->mppt_step_timing < MPPT_STEP)
    {
        appstate->mppt_step_timing++;
    }
}

void update_state_powersave(appstate_t *appstate, boolean_t app_timer_elapsed)
{
    appstate->discharge_a = FALSE;
    appstate->discharge_b = FALSE;
    appstate->dynamo_shutoff_enable = FALSE;
    appstate->charge_mode = CHARGE_NONE;
    appstate->is_braking = FALSE;
    update_driving_state(appstate, app_timer_elapsed);
}