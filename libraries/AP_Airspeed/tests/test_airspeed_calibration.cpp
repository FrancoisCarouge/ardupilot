#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Airspeed/AP_Airspeed.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// a global calibration, constructed during static initialization as the
// vehicle's is
static Airspeed_Calibration calibration;

/*
  circle at 18 m/s in a 5 m/s wind, with a sensor reading 0.8 of the true
  airspeed: the calibration recovers the wind and the scale factor
 */
TEST(AirspeedCalibration, CircleInWind)
{
    calibration.init(2.0f);
    const Vector3f wind(3.0f, -4.0f, 0.0f);
    float scale = 0.0f;
    for (int k = 0; k < 600; k++) {
        const float heading = k * 0.05f;
        const Vector3f air(18 * cosf(heading), 18 * sinf(heading), 0.0f);
        scale = calibration.update(0.8f * air.length(), air + wind, 25);
    }
    const Vector3f state = calibration.get_state();
    EXPECT_NEAR(scale, 0.8f, 0.01f);
    EXPECT_NEAR(state.x, wind.x, 0.5f);
    EXPECT_NEAR(state.y, wind.y, 0.5f);
}

AP_GTEST_MAIN()
