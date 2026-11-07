#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AC_PrecLand/PosVelEKF.h>

constexpr auto metre = mp_units::si::metre;
constexpr auto second = mp_units::si::second;
constexpr auto mps = metre / second;
constexpr auto m2 = metre * metre;
constexpr auto mps2 = mps * mps;

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  one prediction and one position fusion, checked against the Kalman
  equations evaluated by hand
 */
TEST(PosVelEKF, PredictFuse)
{
    PosVelEKF ekf;
    ekf.init(1.0f * metre, 0.5f * m2, 2.0f * mps, 0.25f * mps2);

    // x = F x + G dVel, P = F P F' + Q: P = |0.5025 0.025|
    //                                       |0.025  0.29 |
    ekf.predict(0.1f * second, 0.3f * mps, 0.2f * mps);
    EXPECT_FLOAT_EQ(ekf.getPos().numerical_value_in(metre), 1.2f);
    EXPECT_FLOAT_EQ(ekf.getVel().numerical_value_in(mps), 2.3f);

    // innovation 0.3, innovation variance 0.5025 + 0.1
    EXPECT_NEAR(ekf.getPosNIS(1.5f * metre, 0.1f * m2).numerical_value_in(mp_units::one), 0.1493776f, 1e-6f);

    // x = x + K * 0.3 with K = P H' / S
    ekf.fusePos(1.5f * metre, 0.1f * m2);
    EXPECT_NEAR(ekf.getPos().numerical_value_in(metre), 1.4502075f, 1e-6f);
    EXPECT_NEAR(ekf.getVel().numerical_value_in(mps), 2.3124481f, 1e-6f);
}

/*
  a filter in a global object, as AC_PrecLand is in the vehicle, is
  constructed during static initialization: it must still correct its
  estimate with a measurement
 */
static PosVelEKF global_ekf;

TEST(PosVelEKF, StaticStorage)
{
    global_ekf.init(0.0f * metre, 1.0f * m2, 0.0f * mps, 1.0f * mps2);
    global_ekf.fusePos(1.0f * metre, 1.0f * m2);
    EXPECT_NEAR(global_ekf.getPos().numerical_value_in(metre), 0.5f, 1e-6f);
}

AP_GTEST_MAIN()
