#include <AP_gtest.h>
#include <AP_Common/AP_Common.h>

#include <AP_Compass/CompassCalibrator_fit.h>

#if COMPASS_CAL_ENABLED

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace compass_calibrator_fit;

namespace {

constexpr uint16_t sample_count = 50;

// The normal equations of the ellipsoid fit from the residuals' Jacobians
// and the residuals, the factors' derivatives and the residuals scaled by
// `field_scale`, as when the field's numbers are in a unit `field_scale`
// times smaller.
void normal_equations(const float (&jacobians)[sample_count][9], const float (&residuals)[sample_count],
                      float field_scale, Normal<ellipsoid> &JTJ, Gradient<ellipsoid> &JTFI)
{
    accumulate_normal_equations<ellipsoid>(JTJ, JTFI, sample_count,
        [&](uint16_t k) {
            float row[9];
            for (uint8_t j = 0; j < 9; j++) {
                row[j] = j < 3 ? jacobians[k][j] : field_scale * jacobians[k][j];
            }
            return typed_jacobian<ellipsoid>(row);
        },
        [&](uint16_t k) { return field_scale * residuals[k]; });
}

}

// The damped step of the ellipsoid fit does not depend on the unit of the
// field: with the field's numbers 10 times larger, the offsets' steps are 10
// times larger and the soft-iron factors' steps the same, as for an undamped
// Gauss-Newton step.
TEST(CompassCalibratorFit, EllipsoidStepIsIndependentOfTheFieldUnit)
{
    float jacobians[sample_count][9];
    float residuals[sample_count];
    uint32_t state = 7;
    const auto uniform = [&state] {
        state = state * 1664525u + 1013904223u;
        return float(int32_t(state >> 8) - (1 << 23)) / float(1 << 23);
    };
    for (uint16_t k = 0; k < sample_count; k++) {
        for (uint8_t j = 0; j < 9; j++) {
            jacobians[k][j] = uniform();
        }
        residuals[k] = uniform();
    }

    const float field_scale = 10.0f;
    const float lambda = 1.0f;
    Normal<ellipsoid> JTJ_a{}, JTJ_b{};
    Gradient<ellipsoid> JTFI_a{}, JTFI_b{};
    normal_equations(jacobians, residuals, 1.0f, JTJ_a, JTFI_a);
    normal_equations(jacobians, residuals, field_scale, JTJ_b, JTFI_b);

    float step_a[9], step_b[9];
    damped_step<ellipsoid>(JTJ_a, JTFI_a, lambda, step_a);
    damped_step<ellipsoid>(JTJ_b, JTFI_b, lambda, step_b);
    for (uint8_t j = 0; j < 9; j++) {
        const float expected = j < 3 ? field_scale * step_a[j] : step_a[j];
        EXPECT_NEAR(step_b[j], expected, 1e-4f + 1e-3f * fabsf(expected)) << "parameter " << int(j);
    }
}

#endif  // COMPASS_CAL_ENABLED

AP_GTEST_MAIN()
