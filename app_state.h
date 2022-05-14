#ifndef APP_STATE_H
#define APP_STATE_H

#include "boolean.h"
#include "moving_average.h"

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
    boolean_t dynamo_shutoff_enable;
    uint16_t charge_a_value;
    
    uint16_t output_voltage_sum;
    uint16_t output_voltage_step_before;
    uint16_t output_voltage_last_switch;
    uint16_t output_voltage_max_avg;
    uint16_t output_voltage_min_avg;
    moving_average_t output_voltage_noise_moving_average;
    uint8_t output_voltage_measurement_count;
    boolean_t mppt_direction_down;

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
    uint16_t turn_on_limit;
    uint16_t mppt_step_timing;
    uint16_t overvoltage_timing;
    uint8_t mppt_step_down_size;
    uint8_t mppt_step_up_size;

    uint8_t search_seconds;
    uint16_t search_seconds_counter;
    uint8_t search_seconds_count;
    uint16_t last_search_result;
    uint16_t last_search_result_voltage;
    uint16_t best_result;
    uint16_t best_result_at;
    boolean_t scan_mode;
    uint8_t sinking_ctr;
    uint8_t rising_ctr;

    boolean_t overvoltage;
    uint8_t limits_exceeded;
    uint16_t overvoltage_step_size;

} appstate_t;

#endif