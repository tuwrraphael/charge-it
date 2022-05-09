#include "update_state.h"
#include "uart_debug.h"
#include "boolean.h"
#include "app_state.h"
#include "config.h"
#include "moving_average.h"
#ifdef LABBENCH
#include "pinout.h"
#include <avr/io.h>
#endif

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
    return appstate->dynamo_frequency < DYNAMO_FREQUENCY_DRIVING;
#else
    return TRUE;
#endif
}

static boolean_t is_charging(appstate_t *appstate)
{
    return appstate->charge_mode != CHARGE_NONE;
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
    uint16_t search_start = appstate->last_search_result + 60;
    if (search_start > PWM_DUTY_CYCLE_MAX)
    {
        search_start = PWM_DUTY_CYCLE_MAX;
    }
    OCR1A = search_start;
}

void update_state(appstate_t *appstate,
                  boolean_t app_timer_elapsed)
{
    uint16_t max_charge_value = (appstate->max_discharge_value + CHARGE_DISCHARGE_SPAN);

    // boolean_t a_full = appstate->charge_a_value >= max_charge_value;
    // boolean_t b_full = appstate->charge_b_value >= max_charge_value;
    // uint16_t charge_level = appstate->max_discharge_value;

    // boolean_t a_empty = appstate->charge_a_value < appstate->max_discharge_value;
    // boolean_t b_empty = appstate->charge_b_value < appstate->max_discharge_value;

    // if (a_empty || b_empty)
    // {
    //     if (appstate->charge_a_value < appstate->charge_b_value)
    //     {
    //         appstate->charge_mode = CHARGE_A;
    //     }
    //     else
    //     {
    //         appstate->charge_mode = CHARGE_B;
    //     }
    // }
    // else
    // {
    //     boolean_t a_full = appstate->charge_a_value < appstate->max_discharge_value;
    //     boolean_t b_full = appstate->charge_b_value < appstate->max_discharge_value;
    //     if (!(a_full && b_full))
    //     {
    //         if (appstate->charge_a_value < appstate->charge_b_value)
    //         {
    //             appstate->charge_mode = CHARGE_A;
    //         }
    //         else
    //         {
    //             appstate->charge_mode = CHARGE_B;
    //         }
    //     }
    //     else
    //     {
    //         appstate->charge_mode = CHARGE_NONE;
    //     }
    // }

    // appstate->max_charge_a_value = appstate->charge_a_value > appstate->max_charge_a_value ? appstate->charge_a_value : appstate->max_charge_a_value;
    // appstate->max_charge_b_value = appstate->charge_b_value > appstate->max_charge_b_value ? appstate->charge_b_value : appstate->max_charge_b_value;

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
                start_scan(appstate);
            }
        }
    }

    if (appstate->output_voltage_measurement_count >= 100)
    {
        appstate->output_voltage_measurement_count = 0;
        uint16_t mppt_exec = appstate->scan_mode ? SECONDS_TO_TIM2_REVS(0.15) : MPPT_STEP;
        if (appstate->mppt_step_timing >= mppt_exec)
        {
            appstate->mppt_step_timing = 0;

            if (appstate->scan_mode)
            {
                if (appstate->output_voltage_noise_moving_average.avg > appstate->best_result)
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
                boolean_t end_search = appstate->sinking_ctr >= SINKING_THRESHOLD && appstate->rising_ctr >= RISING_THRESHOLD;
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
                        new_ocr1a += (SINKING_THRESHOLD-1) * SEARCH_STEP + SEARCH_STEP / 2;
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

                boolean_t mppt_is_max = OCR1A >= (PWM_DUTY_CYCLE_MAX - appstate->mppt_step_up_size);
                boolean_t mppt_is_min = OCR1A <= appstate->mppt_step_down_size;

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

                // if (!is_significant && appstate->mppt_step_down_size < MAX_MPPT_STEP)
                // {
                //     appstate->mppt_step_down_size++;
                // }
                // else if (is_significant && appstate->mppt_step_down_size > MIN_MPPT_STEP)
                // {
                //     appstate->mppt_step_down_size--;
                // }
                // appstate->mppt_step_up_size = appstate->mppt_step_down_size;

                if ((switch_direction) || (appstate->mppt_direction_down && mppt_is_min) || (!appstate->mppt_direction_down && mppt_is_max))
                {
                    appstate->mppt_direction_down = !appstate->mppt_direction_down;
                }

                if (appstate->mppt_direction_down)
                {
                    if (!mppt_is_min)
                    {
                        OCR1A -= appstate->mppt_step_down_size;
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
                        OCR1A += appstate->mppt_step_up_size;
                    }
                    else
                    {
                        OCR1A = PWM_DUTY_CYCLE_MAX;
                    }
                }

                uint16_t deviation = appstate->last_search_result_voltage > appstate->output_voltage_noise_moving_average.avg ? appstate->last_search_result_voltage - appstate->output_voltage_noise_moving_average.avg : appstate->output_voltage_noise_moving_average.avg - appstate->last_search_result_voltage;
                if (deviation > 1000)
                {
                    appstate->search_seconds = MIN_SEARCH_SCAN_INTERVAL_SECONDS;
                    // start_scan(appstate);
                }
            }
            appstate->output_voltage_step_before = appstate->output_voltage_noise_moving_average.avg;
        }
        debug_appstate(appstate);

        moving_average_add(&appstate->output_voltage_noise_moving_average, appstate->output_voltage_sum);
        // if (appstate->output_voltage_max_avg < appstate->output_voltage_noise_moving_average.avg)
        // {
        //     appstate->output_voltage_max_avg = appstate->output_voltage_noise_moving_average.avg;
        // }
        // if (appstate->output_voltage_min_avg > appstate->output_voltage_noise_moving_average.avg)
        // {
        //     appstate->output_voltage_min_avg = appstate->output_voltage_noise_moving_average.avg;
        // }
        boolean_t limits_exeeded = appstate->max_charge_a_value > MAX_CHARGE_VALUE || appstate->max_charge_b_value > MAX_CHARGE_VALUE;
        if (limits_exeeded)
        {
            appstate->back_off = 20;
            if (appstate->turn_on_limit > TURN_ON_LIMIT_STEP)
            {
                appstate->turn_on_limit -= TURN_ON_LIMIT_STEP;
            }
        }
        else if (appstate->max_charge_a_value < max_charge_value && appstate->max_charge_b_value < max_charge_value && appstate->turn_on_limit < TURN_ON_LIMIT_MAX)
        {
            appstate->turn_on_limit += TURN_ON_LIMIT_STEP;
        }
        appstate->output_voltage_sum = 0;
        appstate->max_charge_a_value = 0;
        appstate->max_charge_b_value = 0;
    }

    // appstate->discharge_a = appstate->charge_mode != CHARGE_A && appstate->charge_a_value > MAX_DISCHARGE_VALUE;
    // appstate->discharge_b = appstate->charge_mode != CHARGE_B && appstate->charge_b_value > MAX_DISCHARGE_VALUE;

    appstate->dynamo_shutoff = !is_charging(appstate) && driving_below_danger_voltage(appstate);

    update_driving_state(appstate, app_timer_elapsed);

    if (appstate->avg > BRAKING_THRESHOLD)
    {
        appstate->is_braking = BRAKE_LIGHT_TIMING;
    }
    else if (app_timer_elapsed && appstate->is_braking > 0)
    {
        appstate->is_braking--;
    }

    if (app_timer_elapsed && appstate->mppt_step_timing < MPPT_STEP)
    {
        appstate->mppt_step_timing++;
    }

    // OCR1A = 250;
}

void update_state_powersave(appstate_t *appstate, boolean_t app_timer_elapsed)
{
    appstate->discharge_a = FALSE;
    appstate->discharge_b = FALSE;
    appstate->dynamo_shutoff = FALSE;
    appstate->charge_mode = CHARGE_NONE;
    appstate->is_braking = FALSE;
    update_driving_state(appstate, app_timer_elapsed);
}