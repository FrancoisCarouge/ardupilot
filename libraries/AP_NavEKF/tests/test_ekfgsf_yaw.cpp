#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_NavEKF/EKFGSF_yaw.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// an estimator constructed during static initialization
static EKFGSF_yaw static_estimator;

namespace {

const ftype IMU_DT = 0.0025;
const uint32_t GPS_PERIOD = 40;

// horizontal acceleration of the flight, north and east (m/s/s), from rest
Vector3F acceleration(ftype t)
{
    return Vector3F(3 * sinF(2.5 * t), 3 * sinF(1.7 * t), 0);
}

/*
  body to earth rotation of a multicopter: thrust along the specific force,
  nose at the given yaw
 */
Matrix3F attitude(ftype t, ftype yaw)
{
    const Vector3F specific_force = acceleration(t) - Vector3F(0, 0, GRAVITY_MSS);
    const Vector3F down = -specific_force.normalized();
    const Vector3F heading(cosF(yaw), sinF(yaw), 0);
    const Vector3F right = (down % heading).normalized();
    const Vector3F forward = right % down;
    Matrix3F R;
    for (uint8_t row = 0; row < 3; row++) {
        R[row][0] = forward[row];
        R[row][1] = right[row];
        R[row][2] = down[row];
    }
    return R;
}

struct Result {
    bool available;
    ftype yaw;
    ftype yaw_variance;
    ftype innovation_length;
};

/*
  one second at rest to align the tilt, then a flight of the given duration
  at a yaw turning from yaw0 at yaw_rate, with GPS velocities at 10 Hz
 */
Result fly(EKFGSF_yaw &estimator, ftype yaw0, ftype yaw_rate, ftype duration)
{
    for (uint32_t k = 0; k < 400; k++) {
        estimator.update(Vector3F(), Vector3F(0, 0, -GRAVITY_MSS * IMU_DT), IMU_DT, IMU_DT, false, 0);
    }

    Vector3F velocity;
    const uint32_t steps = duration / IMU_DT;
    for (uint32_t k = 0; k < steps; k++) {
        const ftype t = k * IMU_DT;
        const Matrix3F R0 = attitude(t, yaw0 + yaw_rate * t);
        const Matrix3F R1 = attitude(t + IMU_DT, yaw0 + yaw_rate * (t + IMU_DT));
        // the rotation over the step, in the body frame
        const Matrix3F dR = R0.transposed() * R1;
        const Vector3F delAng(0.5 * (dR[2][1] - dR[1][2]), 0.5 * (dR[0][2] - dR[2][0]), 0.5 * (dR[1][0] - dR[0][1]));
        const Vector3F a = acceleration(t + 0.5 * IMU_DT);
        const Vector3F delVel = R0.transposed() * (a - Vector3F(0, 0, GRAVITY_MSS)) * IMU_DT;
        velocity += a * IMU_DT;

        estimator.update(delAng, delVel, IMU_DT, IMU_DT, true, 0);
        if (k % GPS_PERIOD == GPS_PERIOD - 1) {
            estimator.fuseVelData(Vector2F(velocity.x, velocity.y), 0.3);
        }
    }

    Result result {};
    result.available = estimator.getYawData(result.yaw, result.yaw_variance) &&
                       estimator.getVelInnovLength(result.innovation_length);
    return result;
}

void expect_yaw(EKFGSF_yaw &estimator, ftype yaw0, ftype yaw_rate)
{
    const ftype duration = 20;
    const Result result = fly(estimator, yaw0, yaw_rate, duration);
    EXPECT_TRUE(result.available);
    EXPECT_NEAR(wrap_PI(result.yaw - (yaw0 + yaw_rate * duration)), 0, radians(5));
    EXPECT_LT(result.yaw_variance, sq(radians(5)));
    EXPECT_LT(result.innovation_length, 0.5);
}

} // namespace

// no estimate before the velocity fusion starts
TEST(EKFGSFYaw, Unavailable)
{
    EKFGSF_yaw estimator;
    ftype yaw, yaw_variance, innovation_length;
    EXPECT_FALSE(estimator.getYawData(yaw, yaw_variance));
    EXPECT_FALSE(estimator.getVelInnovLength(innovation_length));
    estimator.update(Vector3F(), Vector3F(0, 0, -GRAVITY_MSS * IMU_DT), IMU_DT, IMU_DT, false, 0);
    estimator.fuseVelData(Vector2F(1, 2), 0.3);
    EXPECT_FALSE(estimator.getYawData(yaw, yaw_variance));
}

// the yaw is found from the GPS velocity of an accelerating flight, whatever
// the yaw
TEST(EKFGSFYaw, ConstantYaw)
{
    for (const ftype yaw0 : {-3.0, -1.2, 0.0, 0.7, 2.5}) {
        EKFGSF_yaw estimator;
        expect_yaw(estimator, yaw0, 0);
    }
}

TEST(EKFGSFYaw, Turning)
{
    for (const ftype yaw_rate : {-0.3, 0.2}) {
        EKFGSF_yaw estimator;
        expect_yaw(estimator, 1.0, yaw_rate);
    }
}

TEST(EKFGSFYaw, StaticStorage)
{
    expect_yaw(static_estimator, -2.0, 0.1);
}

AP_GTEST_MAIN()
