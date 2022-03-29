#ifndef APP_STATE_H
#define APP_STATE_H

#include "boolean.h"

typedef enum
{
    CHARGE_NONE,
    CHARGE_A,
    CHARGE_B
} charge_mode_t;

typedef enum
{
    DRIVING_STATE_STARTING,
    DRIVING_STATE_DRIVING,
    DRIVING_STATE_STOPPED,
    DRIVING_STATE_STOPPING
} driving_state_t;

typedef struct
{
    charge_mode_t charge_mode;
    boolean_t discharge_a;
    boolean_t discharge_b;
    boolean_t light_requested;
    boolean_t dynamo_shutoff;
    uint16_t charge_a_value;
    uint16_t charge_b_value;
    uint16_t max_discharge_value;
    uint16_t dynamo_frequency;

    driving_state_t driving_state;
    int16_t diff;
    int16_t avg;
    uint16_t is_braking;

    uint16_t driving_state_timing;

    uint8_t cycle_count;
    uint8_t back_off;
    uint16_t max_charge_a_value;
    uint16_t max_charge_b_value;
    uint8_t turn_on_limit;

} appstate_t;

#endif