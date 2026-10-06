#include <AP_gtest.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/polyfit.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  fit the third order polynomial of the temperature calibration to
  samples taken over its temperature range, as differences to the
  reference temperature
 */
TEST(PolyFit, ThirdOrder)
{
    // coefficients of t^3, t^2, t and 1, the order of get_polynomial()
    const Vector3f coefficients[4] {
        {2.0e-6f, -1.0e-6f, 3.0e-6f},
        {1.0e-4f, 2.0e-4f, -1.5e-4f},
        {0.01f, -0.02f, 0.005f},
        {0.1f, -0.2f, 0.3f},
    };
    PolyFit<4, double, Vector3f> fit {};
    for (float t = -15.0f; t <= 25.0f; t += 0.5f) {
        const Vector3f y = ((coefficients[0] * t + coefficients[1]) * t + coefficients[2]) * t + coefficients[3];
        fit.update(t, y);
    }

    Vector3f fitted[4];
    ASSERT_TRUE(fit.get_polynomial(fitted));
    for (uint8_t i = 0; i < 4; i++) {
        for (uint8_t axis = 0; axis < 3; axis++) {
            EXPECT_NEAR(fitted[i][axis], coefficients[i][axis], fabsf(coefficients[i][axis]) * 1e-3f);
        }
    }
}

TEST(PolyFit, TooFewSamples)
{
    // three temperatures cannot determine four coefficients
    PolyFit<4, double, Vector3f> fit {};
    for (float t = 0.0f; t < 3.0f; t += 1.0f) {
        fit.update(t, Vector3f(1.0f, 2.0f, 3.0f));
    }

    Vector3f fitted[4];
    EXPECT_FALSE(fit.get_polynomial(fitted));
}

AP_GTEST_MAIN()
