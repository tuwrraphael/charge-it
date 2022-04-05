#include "update_state.h"
#include "uart_debug.h"
#include "boolean.h"
#include "app_state.h"
#include "config.h"
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

static void step_down(appstate_t *appstate)
{
    if (appstate->max_discharge_value > MIN_CHARGE_VALUE)
    {
        appstate->max_discharge_value -= VOLTS_TO_ADC(0.1);
    }
}

static void step_up(appstate_t *appstate)
{
    if (appstate->max_discharge_value < MAX_CHARGE_VALUE)
    {
        appstate->max_discharge_value += VOLTS_TO_ADC(0.5);
    }
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

void update_state(appstate_t *appstate,
                  boolean_t app_timer_elapsed)
{
    uint16_t max_charge_value = (appstate->max_discharge_value + CHARGE_DISCHARGE_SPAN);

    // boolean_t a_full = appstate->charge_a_value >= max_charge_value;
    // boolean_t b_full = appstate->charge_b_value >= max_charge_value;
    // uint16_t charge_level = appstate->max_discharge_value;

    boolean_t a_empty = appstate->charge_a_value < appstate->max_discharge_value;
    boolean_t b_empty = appstate->charge_b_value < appstate->max_discharge_value;

    if (a_empty || b_empty)
    {
        if (appstate->charge_a_value < appstate->charge_b_value)
        {
            appstate->charge_mode = CHARGE_A;
        }
        else
        {
            appstate->charge_mode = CHARGE_B;
        }
    }
    else
    {
        boolean_t a_full = appstate->charge_a_value < appstate->max_discharge_value;
        boolean_t b_full = appstate->charge_b_value < appstate->max_discharge_value;
        if (!(a_full && b_full))
        {
            if (appstate->charge_a_value < appstate->charge_b_value)
            {
                appstate->charge_mode = CHARGE_A;
            }
            else
            {
                appstate->charge_mode = CHARGE_B;
            }
        }
        else
        {
            appstate->charge_mode = CHARGE_NONE;
        }
    }

    appstate->max_charge_a_value = appstate->charge_a_value > appstate->max_charge_a_value ? appstate->charge_a_value : appstate->max_charge_a_value;
    appstate->max_charge_b_value = appstate->charge_b_value > appstate->max_charge_b_value ? appstate->charge_b_value : appstate->max_charge_b_value;

    if (appstate->charge_current_measurement_count >= 100)
    {
        appstate->charge_current_measurement_count = 0;
        if (appstate->back_off > 0)
        {
            appstate->back_off--;
        }
        else
        {
            if (appstate->max_charge_a_value < appstate->max_discharge_value)
            {
                step_down(appstate);
            }
            else
            {
                uint16_t max_discharge_mv = appstate->charge_voltage_sum;

                uint16_t power_mw = (max_discharge_mv / 100) * (appstate->charge_current_measurement_sum / 100);

                if (appstate->power_before > power_mw)
                {
                    // appstate->mppt_direction_change_ctr++;
                    // if (appstate->mppt_direction_change_ctr >= 3)
                    // {
                    //     appstate->mppt_direction_change_ctr = 0;
                    appstate->mppt_direction_down = !appstate->mppt_direction_down;
                    // }
                }
                // else if (appstate->mppt_direction_change_ctr > 0)
                // {
                //     appstate->mppt_direction_change_ctr--;
                // }
                appstate->power_before = power_mw;
                if (appstate->mppt_direction_down)
                {
                    step_down(appstate);
                }
                else
                {

                    step_up(appstate);
                }
            }
        }
        boolean_t limits_exeeded = appstate->max_charge_a_value > MAX_CHARGE_VALUE || appstate->max_charge_b_value > MAX_CHARGE_VALUE;
        if (limits_exeeded)
        {
            appstate->back_off = 20;
            step_down(appstate);
            if (appstate->turn_on_limit > TURN_ON_LIMIT_STEP)
            {
                appstate->turn_on_limit -= TURN_ON_LIMIT_STEP;
            }
        }
        else if (appstate->max_charge_a_value < max_charge_value && appstate->max_charge_b_value < max_charge_value && appstate->turn_on_limit < TURN_ON_LIMIT_MAX)
        {
            appstate->turn_on_limit += TURN_ON_LIMIT_STEP;
        }
        appstate->charge_current_measurement_sum = 0;
        appstate->max_charge_a_value = 0;
        appstate->max_charge_b_value = 0;
        appstate->charge_voltage_sum = 0;
    }

    appstate->discharge_a = appstate->charge_mode != CHARGE_A && appstate->charge_a_value > MAX_DISCHARGE_VALUE;
    appstate->discharge_b = appstate->charge_mode != CHARGE_B && appstate->charge_b_value > MAX_DISCHARGE_VALUE;

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