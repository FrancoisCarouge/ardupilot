#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AC_PrecLand/PosVelEKF.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  one prediction and one position fusion, checked against the Kalman
  equations evaluated by hand
 */
TEST(PosVelEKF, PredictFuse)
{
    PosVelEKF ekf;
    ekf.init(1.0f, 0.5f, 2.0f, 0.25f);

    // x = F x + G dVel, P = F P F' + Q: P = |0.5025 0.025|
    //                                       |0.025  0.29 |
    ekf.predict(0.1f, 0.3f, 0.2f);
    EXPECT_FLOAT_EQ(ekf.getPos(), 1.2f);
    EXPECT_FLOAT_EQ(ekf.getVel(), 2.3f);

    // innovation 0.3, innovation variance 0.5025 + 0.1
    EXPECT_NEAR(ekf.getPosNIS(1.5f, 0.1f), 0.1493776f, 1e-6f);

    // x = x + K * 0.3 with K = P H' / S
    ekf.fusePos(1.5f, 0.1f);
    EXPECT_NEAR(ekf.getPos(), 1.4502075f, 1e-6f);
    EXPECT_NEAR(ekf.getVel(), 2.3124481f, 1e-6f);
}

/*
  a filter in a global object, as AC_PrecLand is in the vehicle, is
  constructed during static initialization: it must still correct its
  estimate with a measurement
 */
static PosVelEKF global_ekf;

TEST(PosVelEKF, StaticStorage)
{
    global_ekf.init(0.0f, 1.0f, 0.0f, 1.0f);
    global_ekf.fusePos(1.0f, 1.0f);
    EXPECT_NEAR(global_ekf.getPos(), 0.5f, 1e-6f);
}

AP_GTEST_MAIN()
