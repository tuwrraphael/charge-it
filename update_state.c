#include "update_state.h"
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
    return 0 == (PINB & (1 << MOSI_PIN));
#endif
}

static boolean_t is_driving_fast(appstate_t *appstate)
{
#ifndef LABBENCH
    return appstate->dynamo_frequency < DYNAMO_FREQUENCY_DRIVING;
#else
    return 0 == (PINB & (1 << SCK_PIN));
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
        appstate->max_discharge_value -= CHARGE_VOLTAGE_STEP;
    }
}

static void step_up(appstate_t *appstate)
{
    if (appstate->max_discharge_value < MAX_CHARGE_VALUE)
    {
        appstate->max_discharge_value += CHARGE_VOLTAGE_STEP;
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

    if (appstate->charge_a_value < appstate->max_discharge_value || appstate->charge_b_value < appstate->max_discharge_value)
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

    appstate->discharge_a = appstate->charge_mode != CHARGE_A && appstate->charge_a_value > MAX_DISCHARGE_VALUE;
    appstate->discharge_b = appstate->charge_mode != CHARGE_B && appstate->charge_b_value > MAX_DISCHARGE_VALUE;

    appstate->cycle_count = (appstate->cycle_count + 1) % 100;
    appstate->max_charge_a_value = appstate->charge_a_value > appstate->max_charge_a_value ? appstate->charge_a_value : appstate->max_charge_a_value;
    appstate->max_charge_b_value = appstate->charge_b_value > appstate->max_charge_b_value ? appstate->charge_b_value : appstate->max_charge_b_value;
    boolean_t limits_exeeded = appstate->max_charge_a_value > MAX_CHARGE_VALUE || appstate->max_charge_b_value > MAX_CHARGE_VALUE;

    if (appstate->cycle_count == 0)
    {

        if ((appstate->max_charge_a_value > max_charge_value || appstate->max_charge_b_value > max_charge_value) && !limits_exeeded)
        {
            if (appstate->back_off > 0)
            {
                appstate->back_off--;
            }
            else
            {
                step_up(appstate);
            }
        }
        else
        {
            step_down(appstate);
        }

        if (limits_exeeded)
        {
            appstate->back_off = 20;
            if (appstate->turn_on_limit_us > TURN_ON_LIMIT_STEP)
            {
                appstate->turn_on_limit_us -= TURN_ON_LIMIT_STEP;
            }
        }
        else if (appstate->max_charge_a_value < max_charge_value && appstate->max_charge_b_value < max_charge_value)
        {
            appstate->turn_on_limit_us = TURN_ON_LIMIT_MAX;
        }

        appstate->max_charge_a_value = 0;
        appstate->max_charge_b_value = 0;
        appstate->cycle_count = 0;
    }

    appstate->dynamo_shutoff = !is_charging(appstate) && driving_below_danger_voltage(appstate);

    update_driving_state(appstate, app_timer_elapsed);

    if (appstate->avg > BRAKING_THRESHOLD)
    {
        appstate->is_braking = TRUE;
    }
    else
    {
        appstate->is_braking = FALSE;
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