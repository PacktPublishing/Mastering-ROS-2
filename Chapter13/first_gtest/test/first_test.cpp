#include <gtest/gtest.h>
#include <cmath>

// The function to be tested
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;

double degreesToRadians(double degrees) {
    double radians = degrees * DEG_TO_RAD;
    
    while (radians > PI) {
        radians -= 2.0 * PI;
    }
    while (radians < -PI) {
        radians += 2.0 * PI;
    }
    
    return radians;
}


TEST(AngleConversionTest, HandlesPositiveAngles) {
    EXPECT_NEAR(degreesToRadians(0), 0.0, 1e-9);
    EXPECT_NEAR(degreesToRadians(180), PI, 1e-9);
    EXPECT_NEAR(degreesToRadians(90), PI / 2.0, 1e-9);
    EXPECT_NEAR(degreesToRadians(360), 0.0, 1e-9);
}

TEST(AngleConversionTest, HandlesNegativeAngles) {
    EXPECT_NEAR(degreesToRadians(-180), -PI, 1e-9);
    EXPECT_NEAR(degreesToRadians(-90), -PI / 2.0, 1e-9);
    EXPECT_NEAR(degreesToRadians(-360), 0.0, 1e-9);
}

TEST(AngleConversionTest, HandlesOverflowAngles) {
    EXPECT_NEAR(degreesToRadians(720), 0.0, 1e-9);
    EXPECT_NEAR(degreesToRadians(540), -PI, 1e-9);
    EXPECT_NEAR(degreesToRadians(-540), PI, 1e-9);
}

TEST(AngleConversionTest, HandlesSmallAngles) {
    EXPECT_NEAR(degreesToRadians(1), DEG_TO_RAD, 1e-9);
    EXPECT_NEAR(degreesToRadians(-1), -DEG_TO_RAD, 1e-9);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
