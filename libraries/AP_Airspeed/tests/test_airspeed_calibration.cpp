#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Airspeed/Airspeed_Calibration.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// a global calibration, constructed during static initialization as the
// vehicle's is
static Airspeed_Calibration calibration;

/*
  circle at 18 m/s in a 5 m/s wind, with a sensor whose ratio is 1/0.8^2: at
  a ratio of 1 the square root of its pressure reads 0.8 of the true airspeed,
  and the calibration recovers the wind and the scale
 */
TEST(AirspeedCalibration, CircleInWind)
{
    using Calibration = Airspeed_Calibration;
    const auto metre_per_second = Calibration::MetresPerSecond::reference;
    calibration.init(2.0f * Calibration::Ratio::reference);
    const Vector3f wind(3.0f, -4.0f, 0.0f);
    Calibration::Scale scale{};
    for (int k = 0; k < 600; k++) {
        const float heading = k * 0.05f;
        const Vector3f air(18 * cosf(heading), 18 * sinf(heading), 0.0f);
        const Vector3f ground = air + wind;
        scale = calibration.update(0.8f * air.length() * Calibration::RootPressure::reference,
                                   ground.x * metre_per_second, ground.y * metre_per_second,
                                   ground.z * metre_per_second, 25.0f * metre_per_second);
    }
    const Calibration::Estimate state = calibration.get_state();
    EXPECT_NEAR(scale.numerical_value_in(Calibration::Scale::unit), 0.8f, 0.01f);
    EXPECT_NEAR(state.wind_north.numerical_value_in(metre_per_second), wind.x, 0.5f);
    EXPECT_NEAR(state.wind_east.numerical_value_in(metre_per_second), wind.y, 0.5f);
    EXPECT_NEAR(Calibration::ratio_of(scale).numerical_value_in(Calibration::Ratio::unit), 1 / sq(0.8f), 0.05f);
}

AP_GTEST_MAIN()
