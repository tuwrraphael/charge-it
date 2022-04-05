#ifndef CHARGE_CURRENT_H
#define CHARGE_CURRENT_H

#include <stdint.h>

uint16_t get_charge_current_mA(uint16_t timer1_elapsed, uint16_t adc_voltage_before, uint16_t adc_voltage_after);

#endif
