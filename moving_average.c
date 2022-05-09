#include "moving_average.h"
#include <string.h>
#include <stdio.h>

void moving_average_init(moving_average_t *moving_average)
{
    memset(moving_average, 0, sizeof(moving_average_t));
}

uint32_t moving_average_get_variance(moving_average_t *moving_average)
{
    if (moving_average->count >= MOVING_AVERAGE_SIZE)
    {
        uint32_t variance = 0;
        for (uint16_t i = 0; i < MOVING_AVERAGE_SIZE; i++)
        {
            int16_t diff = moving_average->data[i] - moving_average->avg;
            uint16_t diff_pos = diff > 0 ? diff : -diff;
            variance += diff_pos * diff_pos;
        }
        return variance / MOVING_AVERAGE_SIZE;
    }
    else
    {
        return 0;
    }
}

void moving_average_add(moving_average_t *moving_average, int16_t data)
{
    int16_t old_data = moving_average->data[moving_average->index];
    moving_average->data[moving_average->index] = data;
    moving_average->index = (moving_average->index + 1) % MOVING_AVERAGE_SIZE;

    if (moving_average->count >= MOVING_AVERAGE_SIZE)
    {
        int16_t delta = data - old_data;
        moving_average->sum += data;
        moving_average->sum -= old_data;
        int16_t new_avg = moving_average->sum / MOVING_AVERAGE_SIZE;
        if (moving_average->calculate_variance)
        {
            int64_t summand = (data + old_data - moving_average->avg - new_avg) * delta;
            // printf("%d\n", summand);
            if (summand < 0 && ((uint32_t)(-summand) > moving_average->var_sum))
            {
                moving_average->var_sum = 0;
            }
            else
            {
                moving_average->var_sum += summand;
            }
        }
        moving_average->avg = new_avg;
    }
    else
    {
        int16_t delta = data - moving_average->avg;
        moving_average->sum += data;
        moving_average->count++;
        moving_average->avg = moving_average->sum / moving_average->count;
        if (moving_average->calculate_variance)
        {
            moving_average->var_sum += (delta * (data - moving_average->avg));
        }
    }
}