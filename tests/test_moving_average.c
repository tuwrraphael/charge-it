#include "gtest/gtest.h"
#include "../moving_average.h"
#include "../convert_to_dynamo_rpm.h"
#include "../config.h"
#include "math.h"

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
    moving_average.calculate_variance = TRUE;
    moving_average_add(&moving_average, 0);
    moving_average_add(&moving_average, 1000);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);
    moving_average_add(&moving_average, 10);

    ASSERT_EQ(moving_average.avg, 133);
    ASSERT_EQ((uint16_t)sqrt(moving_average_get_variance(&moving_average)), 327);

    moving_average_add(&moving_average, 10);
    ASSERT_EQ(moving_average.avg, 10);
    ASSERT_EQ((uint16_t)sqrt(moving_average_get_variance(&moving_average)), 0);
}

TEST(moving_average, variance)
{
    moving_average_t moving_average;
    moving_average_init(&moving_average);
    moving_average.calculate_variance = TRUE;
    moving_average_add(&moving_average, 101);
    moving_average_add(&moving_average, 149);
    moving_average_add(&moving_average, 99);
    moving_average_add(&moving_average, 151);
    moving_average_add(&moving_average, 98);
    moving_average_add(&moving_average, 153);
    moving_average_add(&moving_average, 101);
    moving_average_add(&moving_average, 150);
    moving_average_add(&moving_average, 100);
    moving_average_add(&moving_average, 150);
    moving_average_add(&moving_average, 100);
    moving_average_add(&moving_average, 150);

    ASSERT_EQ(moving_average.avg, 125);
    ASSERT_EQ((uint16_t)sqrt(moving_average_get_variance(&moving_average)), 25);
}

static void add_and_print(moving_average_t *moving_average, uint16_t value)
{
    moving_average_add(moving_average, value);
    printf("%d\n", moving_average_get_variance(moving_average));
}

uint16_t values[] = {
4045,
4023,
4073,
4069,
4069,
4068,
4068,
4068,
4068,
4176,
4193,
4205,
4197,
4198,
4210,
4202,
4200,
4215,
4202,
4179,
4218,
4192,
4150,
4165,
4211,
4232,
4211,
4197,
4171,
4241,
4229,
4217,
4216,
4223,
4225,
4208,
4221,
4223,
4206,
4030,
4046,
4047,
4063,
4052,
4080,
4054,
4014,
3967,
3950,
4183,
4170,
4185,
4172,
4185,
4172,
4185,
4173,
4174,
4190,
4142,
4177,
4164,
4246,
4250,
4253,
4256,
4193,
4249,
4207,
4348,
4273,
4250,
4251,
4265,
4250,
4251,
4265,
4250,
4250,
4160,
4134,
4145,
4213,
4199,
4197,
4146,
4096,
4097,
4144,
4103,
4115,
4101,
4115,
4101,
4115,
4103,
4115,
4116,
4117,
4168,
4148,
4097,
4213,
4199,
4213,
4148,
4080,
4080,
4112,
4106,
4100,
4115,
4106,
4121,
4115,
4135,
4142,
4145,
4139,
4269,
4309,
4313,
4311,
4328,
4315,
4329,
4315,
4330,
4332,
4041,
4066,
4066,
4144,
4144,
4144,
4105,
4053,
4066,
4053,
4274,
4295,
4284,
4299,
4294,
4294,
4313,
4310,
4343,
4333,
4223,
4201,
4179,
4189,
4172,
4192,
4178,
4191,
4193,
4196,
4292,
4337,
4352,
4339,
4355,
4341,
4355,
4341,
4341,
4354,
4118,
4054,
4066,
4146,
4148,
4148,
4108,
4053,
4053,
4053,
4282,
4300,
4304,
4321,
4315,
4328,
4315,
4320,
4338,
4323,
4360,
4365,
4349,
4365,
4349,
4365,
4349,
4365,
4349,
4365,
4215,
4222,
4157,
4189,
4202,
4185,
4180,
4138,
4153,
4128,
4218,
4285,
4298,
4301,
4305,
4304,
4274,
4215,
4215,
4215,
4321,
4342,
4357,
4342,
4356,
4341,
4356,
4341,
4341,
4357,
4118,
4054,
4066,
4149,
4150,
4147,
4108,
4053,
4053,
4053,
4284,
4301,
4304,
4307,
4326,
4317,
4316,
4319,
4324,
4341,
4378,
4350,
4365,
4350,
4365,
4350,
4365,
4350,
4365,
4350,
4269,
4275,
4269,
4283,
4278,
4273,
4227,
4179,
4179,
4255,
4297,
4349,
4365,
4350,
4365,
4350,
4365,
4350
};

TEST(moving_average, variance2)
{
    moving_average_t moving_average;
    moving_average_init(&moving_average);
    moving_average.calculate_variance = TRUE;
    moving_average.var_sum = 8 * 18911;
    
    for (uint16_t i = 0; i < sizeof(values) / sizeof(uint16_t); i++)
    {
        add_and_print(&moving_average, values[i]);
    }
    // ASSERT_EQ(moving_average.avg, 125);
    // ASSERT_EQ((uint16_t)sqrt(moving_average_get_variance(&moving_average)), 25);
}

// static uint16_t convert_to_dynamo_rpm1(uint16_t oldFrequencyMeasurement, uint16_t newFrequencyMeasurement)
// {
//     if (oldFrequencyMeasurement > newFrequencyMeasurement || newFrequencyMeasurement == 0 || oldFrequencyMeasurement == 0)
//     {
//         return 0;
//     }

//     double old_rpm = (F_CPU / TIMER1_PRESCALER)/(double)oldFrequencyMeasurement;
//     double new_rpm = (F_CPU / TIMER1_PRESCALER)/(double)newFrequencyMeasurement;
//     uint16_t m = convert_to_dynamo_rpm(oldFrequencyMeasurement, newFrequencyMeasurement);
//     printf("old: %f, new: %f, diff: %f, m: %d\n", old_rpm, new_rpm, (old_rpm - new_rpm) * BRAKE_THRESHOLD_SCALE, m);

//     return m;
// }

// TEST(moving_average, stuff) {
//     moving_average_t moving_average;
//     moving_average_init(&moving_average);
//     uint16_t val_before = 5337;

// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5337)); printf("%d, %d\n", moving_average.avg,263); val_before = 5337;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5299)); printf("%d, %d\n", moving_average.avg,263); val_before = 5299;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5330)); printf("%d, %d\n", moving_average.avg,277); val_before = 5330;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5331)); printf("%d, %d\n", moving_average.avg,252); val_before = 5331;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5299)); printf("%d, %d\n", moving_average.avg,243); val_before = 5299;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5296)); printf("%d, %d\n", moving_average.avg,53); val_before = 5296;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5326)); printf("%d, %d\n", moving_average.avg,66); val_before = 5326;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5336)); printf("%d, %d\n", moving_average.avg,31); val_before = 5336;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5320)); printf("%d, %d\n", moving_average.avg,31); val_before = 5320;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5320)); printf("%d, %d\n", moving_average.avg,31); val_before = 5320;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 635)); printf("%d, %d\n", moving_average.avg,31); val_before = 635;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 4778)); printf("%d, %d\n", moving_average.avg,1816); val_before = 4778;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5398)); printf("%d, %d\n", moving_average.avg,2108); val_before = 5398;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5358)); printf("%d, %d\n", moving_average.avg,2108); val_before = 5358;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5352)); printf("%d, %d\n", moving_average.avg,2108); val_before = 5352;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5357)); printf("%d, %d\n", moving_average.avg,2097); val_before = 5357;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5333)); printf("%d, %d\n", moving_average.avg,2093); val_before = 5333;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5340)); printf("%d, %d\n", moving_average.avg,2096); val_before = 5340;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5293)); printf("%d, %d\n", moving_average.avg,2096); val_before = 5293;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5330)); printf("%d, %d\n", moving_average.avg,313); val_before = 5330;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5323)); printf("%d, %d\n", moving_average.avg,21); val_before = 5323;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5285)); printf("%d, %d\n", moving_average.avg,21); val_before = 5285;
// moving_average_add(&moving_average, convert_to_dynamo_rpm1(val_before, 5321)); printf("%d, %d\n", moving_average.avg,36); val_before = 5321;
// }