#include "gtest/gtest.h"
#include "../charge_current.h"
#include "../config.h"

static uint32_t timer_100us = (100 * (F_CPU / (TIMER1_PRESCALER)) / 1000000);
static uint32_t timer_10us = (10 * (F_CPU / (TIMER1_PRESCALER)) / 1000000);
static uint32_t timer_1us = (1 * (F_CPU / (TIMER1_PRESCALER)) / 1000000);
static uint32_t timer_10ms = (1000 * (F_CPU / (TIMER1_PRESCALER)) / 100000);

TEST(charge_current, charge_800ma)
{
    uint16_t result = get_charge_current_mA((uint16_t)timer_100us, VOLTS_TO_ADC(8.0), VOLTS_TO_ADC(8.2));
    ASSERT_EQ(result, 824);
}

TEST(charge_current, no_charge_zero)
{
    uint16_t result = get_charge_current_mA((uint16_t)timer_100us, VOLTS_TO_ADC(8.0), VOLTS_TO_ADC(8.0));
    ASSERT_EQ(result, 0);
}

TEST(charge_current, discharge_zero)
{
    uint16_t result = get_charge_current_mA((uint16_t)timer_100us, VOLTS_TO_ADC(8.0), VOLTS_TO_ADC(7.6));
    ASSERT_EQ(result, 0);
}

TEST(charge_current, charge_in_10us)
{
    uint16_t result = get_charge_current_mA((uint16_t)timer_10us, VOLTS_TO_ADC(8.0), VOLTS_TO_ADC(8.5));
    ASSERT_EQ(result, 19920);
}

TEST(charge_current, charge_in_10ms)
{
    uint16_t result = get_charge_current_mA((uint16_t)timer_10ms, VOLTS_TO_ADC(8.0), VOLTS_TO_ADC(8.5));
    ASSERT_EQ(result, 19);
}

TEST(charge_current, charge_in_1us)
{
    uint16_t result = get_charge_current_mA(102, VOLTS_TO_ADC(8.0), VOLTS_TO_ADC(8.1));
    ASSERT_EQ(result, 570);
}