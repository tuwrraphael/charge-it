#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <stdint.h>

#define MOVING_AVERAGE_SIZE (32)

typedef struct
{
    uint8_t count;
    uint8_t index;
    int32_t sum;
    int16_t data[MOVING_AVERAGE_SIZE];
    int16_t avg;
} moving_average_t;


void moving_average_init(moving_average_t *moving_average);
void moving_average_add(moving_average_t *moving_average, int16_t data);

#endif