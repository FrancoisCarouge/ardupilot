#include <AP_gtest.h>

#include <AP_AccelCal/AccelCalibrator.h>
#include <AP_Math/AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  feed an accelerometer calibrator with the samples that a sensor with the
  given errors reports in each of the given orientations, and check that the
  calibration recovers the errors
 */
static void check_calibration(accel_cal_fit_type_t fit_type, const Vector3f directions[], uint8_t num_samples,
                              const Vector3f &offset, const Vector3f &diag, const Vector3f &offdiag)
{
    // the calibration corrects a sample s to M * (s + offset)
    const Matrix3f M(diag.x, offdiag.x, offdiag.y,
                     offdiag.x, diag.y, offdiag.z,
                     offdiag.y, offdiag.z, diag.z);
    Matrix3f M_inverse;
    ASSERT_TRUE(M.inverse(M_inverse));

    const float sample_time = 0.5f;
    AccelCalibrator calibrator;
    calibrator.start(fit_type, num_samples, sample_time);
    for (uint8_t i = 0; i < num_samples; i++) {
        ASSERT_EQ(calibrator.get_status(), ACCEL_CAL_WAITING_FOR_ORIENTATION);
        calibrator.collect_sample();
        const Vector3f sample = M_inverse * (directions[i].normalized() * GRAVITY_MSS) - offset;
        const float dt = sample_time + 0.01f;
        calibrator.new_sample(sample * dt, dt);
    }
    ASSERT_EQ(calibrator.get_status(), ACCEL_CAL_SUCCESS);

    // the calibration reports the offset to add, the opposite of the fitted one
    Vector3f fitted_offset, fitted_diag, fitted_offdiag;
    calibrator.get_calibration(fitted_offset, fitted_diag, fitted_offdiag);
    for (uint8_t i = 0; i < 3; i++) {
        EXPECT_NEAR(fitted_offset[i], -offset[i], 1e-3f);
        EXPECT_NEAR(fitted_diag[i], diag[i], 1e-4f);
        EXPECT_NEAR(fitted_offdiag[i], offdiag[i], 1e-4f);
    }
}

TEST(AccelCalibrator, AxisAlignedEllipsoid)
{
    const Vector3f directions[] {
        {0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
    };
    check_calibration(ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, directions, ARRAY_SIZE(directions),
                      Vector3f(0.3f, -0.2f, 0.5f), Vector3f(1.02f, 0.97f, 1.05f), Vector3f());
}

TEST(AccelCalibrator, Ellipsoid)
{
    // the vertices of an icosahedron spread the samples over the sphere
    const float phi = (1.0f + sqrtf(5.0f)) * 0.5f;
    const Vector3f directions[] {
        {0, 1, phi}, {0, -1, phi}, {0, 1, -phi}, {0, -1, -phi},
        {1, phi, 0}, {-1, phi, 0}, {1, -phi, 0}, {-1, -phi, 0},
        {phi, 0, 1}, {-phi, 0, 1}, {phi, 0, -1}, {-phi, 0, -1},
    };
    check_calibration(ACCEL_CAL_ELLIPSOID, directions, ARRAY_SIZE(directions),
                      Vector3f(0.3f, -0.2f, 0.5f), Vector3f(1.02f, 0.97f, 1.05f), Vector3f(0.01f, -0.02f, 0.015f));
}

AP_GTEST_MAIN()
