#include "charge_current.h"
#include "config.h"

#define CAPACITY_UF (400)
#define SCALE_FACTOR ((F_CPU / 10000 / TIMER1_PRESCALER))

uint16_t get_charge_current_mA(uint16_t timer1_elapsed, uint16_t adc_voltage_before, uint16_t adc_voltage_after)
{
    if (adc_voltage_after > adc_voltage_before)
    {

        uint16_t timer1_elapsed_us = timer1_elapsed * 100 / SCALE_FACTOR;
        if (timer1_elapsed_us == 0)
        {
            return 0;
        }
        uint16_t voltage_mV = ADC_TO_MV(adc_voltage_after - adc_voltage_before);
        return (voltage_mV * CAPACITY_UF) / (timer1_elapsed_us);
    }
    return 0;
}