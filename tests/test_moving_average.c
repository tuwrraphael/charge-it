#include "gtest/gtest.h"
#include "../moving_average.h"


TEST(moving_average, not_all_values)
{
    moving_average_t moving_average;
    moving_average_init(&moving_average);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 20);
    moving_average_add(&moving_average, 10);
    ASSERT_EQ(moving_average.avg, 13);
}

TEST(moving_average, no_values)
{
    moving_average_t moving_average;
    moving_average_init(&moving_average);
    ASSERT_EQ(moving_average.avg, 0);
}

TEST(moving_average, more_than_max_values)
{
    moving_average_t moving_average;
    moving_average_init(&moving_average);
    moving_average_add(&moving_average, 1000);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    ASSERT_EQ(moving_average.avg, 10);
}

TEST(moving_average, exactly_max_values)
{
    moving_average_t moving_average;
    moving_average_init(&moving_average);
    moving_average_add(&moving_average, 1000);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    ASSERT_EQ(moving_average.avg, 133);
}