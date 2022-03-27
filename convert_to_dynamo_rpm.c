#include "convert_to_dynamo_rpm.h"
#include "config.h"
#include <stdint.h>
#include <stdio.h>

#define SCALE (128)

uint16_t convert_to_dynamo_rpm(uint16_t oldFrequencyMeasurement, uint16_t newFrequencyMeasurement)
{
    if (oldFrequencyMeasurement > newFrequencyMeasurement || newFrequencyMeasurement == 0 || oldFrequencyMeasurement == 0)
    {
        return 0;
    }

    uint16_t next_value = 0;

    uint32_t a = (F_CPU / TIMER1_PRESCALER) / SCALE;
    uint16_t new_scaled = (newFrequencyMeasurement / SCALE);
    a = a / new_scaled;
    uint16_t diff = newFrequencyMeasurement - oldFrequencyMeasurement;
    a = a * diff;
    next_value = (a) / (oldFrequencyMeasurement / BRAKE_THRESHOLD_SCALE);

    return next_value;
}