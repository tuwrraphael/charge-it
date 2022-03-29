#include "gtest/gtest.h"
#include "../moving_average.h"
#include "../convert_to_dynamo_rpm.h"
#include "../config.h"


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

static uint16_t convert_to_dynamo_rpm1(uint16_t oldFrequencyMeasurement, uint16_t newFrequencyMeasurement)
{
    if (oldFrequencyMeasurement > newFrequencyMeasurement || newFrequencyMeasurement == 0 || oldFrequencyMeasurement == 0)
    {
        return 0;
    }

    double old_rpm = (F_CPU / TIMER1_PRESCALER)/(double)oldFrequencyMeasurement;
    double new_rpm = (F_CPU / TIMER1_PRESCALER)/(double)newFrequencyMeasurement;
    uint16_t m = convert_to_dynamo_rpm(oldFrequencyMeasurement, newFrequencyMeasurement);
    printf("old: %f, new: %f, diff: %f, m: %d\n", old_rpm, new_rpm, (old_rpm - new_rpm) * BRAKE_THRESHOLD_SCALE, m);

    
    return m;
}


TEST(moving_average, stuff) {
    moving_average_t moving_average;
    moving_average_init(&moving_average);
    uint16_t val_before = 5337;

moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5337)); printf("%d, %d\n", moving_average.avg,263); val_before = 5337; 
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5299)); printf("%d, %d\n", moving_average.avg,263); val_before = 5299; 
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5330)); printf("%d, %d\n", moving_average.avg,277); val_before = 5330; 
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5331)); printf("%d, %d\n", moving_average.avg,252); val_before = 5331; 
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5299)); printf("%d, %d\n", moving_average.avg,243); val_before = 5299; 
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5296)); printf("%d, %d\n", moving_average.avg,53); val_before = 5296;  
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5326)); printf("%d, %d\n", moving_average.avg,66); val_before = 5326;  
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5336)); printf("%d, %d\n", moving_average.avg,31); val_before = 5336;  
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5320)); printf("%d, %d\n", moving_average.avg,31); val_before = 5320;  
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5320)); printf("%d, %d\n", moving_average.avg,31); val_before = 5320;  
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 635)); printf("%d, %d\n", moving_average.avg,31); val_before = 635;
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 4778)); printf("%d, %d\n", moving_average.avg,1816); val_before = 4778;
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5398)); printf("%d, %d\n", moving_average.avg,2108); val_before = 5398;
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5358)); printf("%d, %d\n", moving_average.avg,2108); val_before = 5358;
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5352)); printf("%d, %d\n", moving_average.avg,2108); val_before = 5352;
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5357)); printf("%d, %d\n", moving_average.avg,2097); val_before = 5357;
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5333)); printf("%d, %d\n", moving_average.avg,2093); val_before = 5333;
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5340)); printf("%d, %d\n", moving_average.avg,2096); val_before = 5340;
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5293)); printf("%d, %d\n", moving_average.avg,2096); val_before = 5293;
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5330)); printf("%d, %d\n", moving_average.avg,313); val_before = 5330; 
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5323)); printf("%d, %d\n", moving_average.avg,21); val_before = 5323;  
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5285)); printf("%d, %d\n", moving_average.avg,21); val_before = 5285;  
moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5321)); printf("%d, %d\n", moving_average.avg,36); val_before = 5321;  
}