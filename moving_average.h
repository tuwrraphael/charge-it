#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <stdint.h>
#include "boolean.h"

#define MOVING_AVERAGE_SIZE (8)

typedef struct
{
    boolean_t calculate_variance;
    uint8_t count;
    uint8_t index;
    int32_t sum;
    int16_t data[MOVING_AVERAGE_SIZE];
    int16_t avg;
    uint32_t var_sum;
} moving_average_t;


void moving_average_init(moving_average_t *moving_average);
void moving_average_add(moving_average_t *moving_average, int16_t data);
uint32_t moving_average_get_variance(moving_average_t *moving_average);

#endif