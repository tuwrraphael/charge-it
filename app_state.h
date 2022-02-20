#ifndef APP_STATE_H
#define APP_STATE_H

#include "boolean.h"

typedef enum {
    CHARGE_NONE,
    CHARGE_A,
    CHARGE_B
} charge_mode_t;

typedef struct {
    charge_mode_t charge_mode;
    boolean_t discharge_a;
    boolean_t discharge_b;
    boolean_t light_requested;
    uint16_t charge_a_value;
    uint16_t charge_b_value;
    uint8_t dynamo_frequency;
} appstate_t;

#endif