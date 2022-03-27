#include "gtest/gtest.h"
#include "../convert_to_dynamo_rpm.h"

TEST(convert_to_dynamo_rpm, no_overflow)
{
    uint16_t result = convert_to_dynamo_rpm(12947, 12297);
    ASSERT_EQ(result, 0);
}

TEST(convert_to_dynamo_rpm, ballpark)
{
    uint16_t result = convert_to_dynamo_rpm(12297, 12947);
    ASSERT_EQ(result, 392);
}