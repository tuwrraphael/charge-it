#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <stdint.h>

#define MOVING_AVERAGE_SIZE (8)

typedef struct
{
    uint8_t count;
    uint8_t index;
    uint16_t sum;
    uint16_t data[MOVING_AVERAGE_SIZE];
    uint16_t avg;
} moving_average_t;


void moving_average_init(moving_average_t *moving_average);
void moving_average_add(moving_average_t *moving_average, uint16_t data);

#endif