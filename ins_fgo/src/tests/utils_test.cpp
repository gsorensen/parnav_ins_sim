#include "../utils.hpp"
#include <gtest/gtest.h>

static constexpr double DOUBLE_THRESHOLD = 1e-4;

TEST(UtilsTest, SSATest)
{
    EXPECT_NEAR(2.41, ssa(2.41), DOUBLE_THRESHOLD);
    EXPECT_NEAR(-2.8832, ssa(3.4), DOUBLE_THRESHOLD);
    EXPECT_NEAR(2.8832, ssa(-3.4), DOUBLE_THRESHOLD);
    EXPECT_NEAR(0.0, ssa(0.0), DOUBLE_THRESHOLD);
    EXPECT_NEAR(-0.0032, ssa(6.28), DOUBLE_THRESHOLD);
    EXPECT_NEAR(0.0, ssa(2 * M_PI), DOUBLE_THRESHOLD);
    EXPECT_NEAR(-3.1416, ssa(M_PI), DOUBLE_THRESHOLD);
}
