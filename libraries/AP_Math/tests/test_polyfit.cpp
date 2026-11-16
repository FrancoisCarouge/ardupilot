#include <AP_gtest.h>
#include <AP_Common/AP_Common.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/polyfit.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Samples of a cubic in each axis are fitted exactly: the coefficients come
// back highest power first, as the temperature calibration evaluates them.
TEST(PolyFitTest, RecoversACubicPerAxis)
{
    const Vector3f coefficients[4] {
        {0.001f, -0.002f, 0.0005f},  // x^3
        {-0.01f, 0.02f, 0.003f},     // x^2
        {0.1f, -0.3f, 0.05f},        // x
        {0.5f, 1.5f, -9.8f},         // 1
    };
    PolyFit<4, double, Vector3f> fit {};
    for (int i = 0; i <= 80; i++) {
        const double x = -20.0 + 0.5 * i;
        Vector3f y;
        for (uint8_t k = 0; k < 4; k++) {
            y = y * float(x) + coefficients[k];
        }
        fit.update(x, y);
    }

    Vector3f result[4];
    ASSERT_TRUE(fit.get_polynomial(result));
    for (uint8_t k = 0; k < 4; k++) {
        EXPECT_NEAR(result[k].x, coefficients[k].x, 1e-4f + 1e-3f * fabsf(coefficients[k].x)) << "power " << 3 - k;
        EXPECT_NEAR(result[k].y, coefficients[k].y, 1e-4f + 1e-3f * fabsf(coefficients[k].y)) << "power " << 3 - k;
        EXPECT_NEAR(result[k].z, coefficients[k].z, 1e-4f + 1e-3f * fabsf(coefficients[k].z)) << "power " << 3 - k;
    }
}

// A singular normal matrix, all samples at one point, has no fit.
TEST(PolyFitTest, RejectsASingularFit)
{
    PolyFit<4, double, Vector3f> fit {};
    for (int i = 0; i < 10; i++) {
        fit.update(2.0, Vector3f{1.0f, 2.0f, 3.0f});
    }
    Vector3f result[4];
    EXPECT_FALSE(fit.get_polynomial(result));
}

AP_GTEST_MAIN()
