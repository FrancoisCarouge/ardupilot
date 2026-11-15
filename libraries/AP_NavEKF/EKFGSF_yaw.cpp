/*
  Definition of functions used to provide a backup estimate of yaw angle
  that doesn't use a magnetometer. Uses a bank of 3-state EKF's organised
  as a Gaussian sum filter where states are velocity N,E (m/s) and yaw angle (rad)

  Written by Paul Riseborough <p_riseborough@live.com.au>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF/EKFGSF_yaw.h"
#include <AP_Math/typed_linear_algebra.h>
#include <new>
#include <type_traits>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

namespace {

namespace tla = typed_linear_algebra;

constexpr auto metre_per_second = mp_units::si::metre / mp_units::si::second;
constexpr auto radian = mp_units::si::radian;
constexpr auto one = mp_units::one;
constexpr auto speed_squared = metre_per_second * metre_per_second;
constexpr auto radian_squared = radian * radian;

using Velocity = tla::QuantityOf<metre_per_second, ftype>;
using Angle = tla::QuantityOf<radian, ftype>;
using VelocityVariance = tla::QuantityOf<speed_squared, ftype>;
using AngleVariance = tla::QuantityOf<radian_squared, ftype>;

// The state: Vel North, Vel East, and yaw.
using State = tla::VectorOf<ftype, Velocity, Velocity, Angle>;
// The output: the NE velocity measurement.
using Output = tla::VectorOf<ftype, Velocity, Velocity>;
namespace kf = fcarouge::kalman_filter::internal;
using Covariance = tla::MatrixOf<ftype, State, State>;
using OutputVariance = tla::MatrixOf<ftype, Output, Output>;
using OutputModel = kf::evaluate<kf::quotient<Output, State>>;
using StateTransition = kf::evaluate<kf::quotient<State, State>>;

// The velocity states are observed directly.
struct velocity_output_model {
    OutputModel operator()(const State &) const
    {
        OutputModel h{};
        h.at<0, 0>(ftype(1) * one);
        h.at<1, 1>(ftype(1) * one);
        return h;
    }
};

// The delta velocity noise, the same along the front and right axes, is the
// same along north and east, and the delta angle noise.
struct process_noise {
    Covariance operator()(const State &, const Velocity &, const Velocity &,
                          const VelocityVariance &d_vel_var, const AngleVariance &d_ang_var) const
    {
        Covariance Q{};
        Q.at<0, 0>(d_vel_var);
        Q.at<1, 1>(d_vel_var);
        Q.at<2, 2>(d_ang_var);
        return Q;
    }
};

// The Jacobian of the state prediction: a yaw error rotates the delta velocity.
struct prediction_jacobian {
    StateTransition operator()(const State &, const Velocity &del_vel_N, const Velocity &del_vel_E,
                               const VelocityVariance &, const AngleVariance &) const
    {
        StateTransition F{};
        F.at<0, 0>(ftype(1) * one);
        F.at<1, 1>(ftype(1) * one);
        F.at<2, 2>(ftype(1) * one);
        F.at<0, 2>(-del_vel_E / (ftype(1) * radian));
        F.at<1, 2>(del_vel_N / (ftype(1) * radian));
        return F;
    }
};

// The delta velocities summed in the earth frame.
struct velocity_sum {
    State operator()(const State &x, const Velocity &del_vel_N, const Velocity &del_vel_E,
                     const VelocityVariance &, const AngleVariance &) const
    {
        return State{x.at<0>() + del_vel_N, x.at<1>() + del_vel_E, x.at<2>()};
    }
};

struct velocity_observation {
    Output operator()(const State &x) const { return Output{x.at<0>(), x.at<1>()}; }
};

auto makeFilter()
{
    using namespace fcarouge;
    return kalman{
        state{State{}},
        output<Output>,
        estimate_uncertainty{Covariance{}},
        process_uncertainty{process_noise{}},
        // set by each correction
        output_uncertainty{OutputVariance{}},
        output_model{velocity_output_model{}},
        state_transition{prediction_jacobian{}},
        transition{velocity_sum{}},
        observation{velocity_observation{}},
        prediction_types<Velocity, Velocity, VelocityVariance, AngleVariance>,
        // P - K H P then (P + P^T) / 2, as the original's forceSymmetry
        simple_form, symmetrize};
}

// Constrain the variances, after the symmetrization.
void constrainVariances(Covariance &P)
{
    // fmaxF is a macro: the template arguments' commas stay out of it
    const ftype min_var = 1e-6f;
    const ftype P00 = P.at<0, 0>().numerical_value_in(speed_squared);
    const ftype P11 = P.at<1, 1>().numerical_value_in(speed_squared);
    const ftype P22 = P.at<2, 2>().numerical_value_in(radian_squared);
    P.at<0, 0>(fmaxF(P00, min_var) * speed_squared);
    P.at<1, 1>(fmaxF(P11, min_var) * speed_squared);
    P.at<2, 2>(fmaxF(P22, min_var) * radian_squared);
}

} // namespace

struct EKFGSF_yaw::Filter {
    decltype(makeFilter()) kalman{makeFilter()};
};

EKFGSF_yaw::Filter &EKFGSF_yaw::filter(const uint8_t mdl_idx)
{
    return *std::launder(static_cast<Filter *>(static_cast<void *>(filters[mdl_idx].bytes)));
}

const EKFGSF_yaw::Filter &EKFGSF_yaw::filter(const uint8_t mdl_idx) const
{
    return *std::launder(static_cast<const Filter *>(static_cast<const void *>(filters[mdl_idx].bytes)));
}

ftype EKFGSF_yaw::yawState(const uint8_t mdl_idx) const
{
    return filter(mdl_idx).kalman.x().at<2>().numerical_value_in(radian);
}

EKFGSF_yaw::EKFGSF_yaw()
{
    static_assert(sizeof(Filter) <= kFilterSize, "EKFGSF_yaw::kFilterSize is too small for the filter");
    static_assert(alignof(Filter) <= alignof(FilterStorage), "EKFGSF_yaw::FilterStorage is under-aligned for the filter");
    static_assert(std::is_trivially_destructible_v<Filter>, "EKFGSF_yaw never destroys its filters");
    for (FilterStorage &storage : filters) {
        new (storage.bytes) Filter;
    }
}

void EKFGSF_yaw::update(const Vector3F &delAng,
                        const Vector3F &delVel,
                        const ftype delAngDT,
                        const ftype delVelDT,
                        bool runEKF,
                        ftype TAS)
{

    // copy to class variables
    delta_angle = delAng;
    delta_velocity = delVel;
    angle_dt = delAngDT;
    velocity_dt = delVelDT;
    run_ekf_gsf = runEKF;
    true_airspeed = TAS;

    // Calculate a low pass filtered acceleration vector that will be used to keep the AHRS tilt aligned
    // The time constant of the filter is a fixed ratio relative to the time constant of the AHRS tilt correction loop
    const ftype filter_coef = fminF(EKFGSF_accelFiltRatio * delVelDT * EKFGSF_tiltGain, 1.0f);
    const Vector3F accel = delVel / fmaxF(delVelDT, 0.001f);
    ahrs_accel = ahrs_accel * (1.0f - filter_coef) + accel * filter_coef;

    // Iniitialise states and only when acceleration is close to 1g to prevent vehicle movement casuing a large initial tilt error
    if (!ahrs_tilt_aligned) {
        const ftype accel_norm_sq = accel.length_squared();
        const ftype upper_accel_limit = GRAVITY_MSS * 1.1f;
        const ftype lower_accel_limit = GRAVITY_MSS * 0.9f;
        const bool ok_to_align = ((accel_norm_sq > lower_accel_limit * lower_accel_limit &&
                                   accel_norm_sq < upper_accel_limit * upper_accel_limit));
        if (ok_to_align) {
            alignTilt();
            ahrs_tilt_aligned = true;
            ahrs_accel = accel;
        }
        return;
    }

    // Calculate common variables used by the AHRS prediction models
    ahrs_accel_norm = ahrs_accel.length();

    // Calculate AHRS acceleration fusion gain using a quadratic weighting function that is unity at 1g
    // and zero at the min and max g limits. This reduces the effect of large g transients on the attitude
    // estimates.
    ftype EKFGSF_ahrs_ng = ahrs_accel_norm / GRAVITY_MSS;
    if (EKFGSF_ahrs_ng > 1.0f) {
        if (is_positive(true_airspeed)) {
            // When flying in fixed wing mode we need to allow for more positive g due to coordinated turns
            // Gain varies from unity at 1g to zero at 2g
            accel_gain = EKFGSF_tiltGain * sq(MAX(2.0f - EKFGSF_ahrs_ng, 0.0f));
        } else if (EKFGSF_ahrs_ng <= 1.5f) {
            // Gain varies from unity at 1g to zero at 1.5g
            accel_gain = EKFGSF_tiltGain * sq(3.0f - 2.0f * EKFGSF_ahrs_ng);
        } else {
            // Gain is zero above max g
            accel_gain = 0.0f;
        }
    } else if (EKFGSF_ahrs_ng > 0.5f) {
        // Gain varies from zero at 0.5g to unity at 1g
        accel_gain = EKFGSF_tiltGain * sq(2.0f * EKFGSF_ahrs_ng - 1.0f);
    } else {
        // Gain is zero below min g
        accel_gain = 0.0f;
    }

    // Always run the AHRS prediction cycle for each model
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        predict(mdl_idx);
    }

    if (vel_fuse_running && !run_ekf_gsf) {
        vel_fuse_running = false;
    }

    // Calculate a composite yaw as a weighted average of the states for each model.
    // To avoid issues with angle wrapping, the yaw state is converted to a vector with legnth
    // equal to the weighting value before it is summed.
    Vector2F yaw_vector;
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        yaw_vector[0] += GSF.weights[mdl_idx] * cosF(yawState(mdl_idx));
        yaw_vector[1] += GSF.weights[mdl_idx] * sinF(yawState(mdl_idx));
    }
    GSF.yaw = atan2F(yaw_vector[1],yaw_vector[0]);

    // Example for future reference showing how a full GSF covariance matrix could be calculated if required
    /*
    memset(&GSF.P, 0, sizeof(GSF.P));
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        ftype delta[3];
        for (uint8_t row = 0; row < 3; row++) {
            delta[row] = EKF[mdl_idx].X[row] - GSF.X[row];
        }
        for (uint8_t row = 0; row < 3; row++) {
            for (uint8_t col = 0; col < 3; col++) {
                GSF.P[row][col] +=  GSF.weights[mdl_idx] * (EKF[mdl_idx].P[row][col] + delta[row] * delta[col]);
            }
        }
    }
    */

    GSF.yaw_variance = 0.0f;
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        ftype yawDelta = wrap_PI(yawState(mdl_idx) - GSF.yaw);
        GSF.yaw_variance +=  GSF.weights[mdl_idx] * (filter(mdl_idx).kalman.p().at<2, 2>().numerical_value_in(radian_squared) + sq(yawDelta));
    }
}

void EKFGSF_yaw::fuseVelData(const Vector2F &vel, const ftype velAcc)
{
    // convert reported accuracy to a variance, but limit lower value to protect algorithm stability
    const ftype velObsVar = sq(fmaxF(velAcc, 0.5f));

    // The 3-state EKF models only run when flying to avoid corrupted estimates due to operator handling and GPS interference
    if (run_ekf_gsf) {
        if (!vel_fuse_running) {
            // Perform in-flight alignment
            resetEKFGSF();
            for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
                // Use the firstGPS  measurement to set the velocities and corresponding variances
                auto &kalman = filter(mdl_idx).kalman;
                kalman.x(State{vel[0] * metre_per_second, vel[1] * metre_per_second, kalman.x().at<2>()});
                Covariance P{kalman.p()};
                P.at<0, 0>(velObsVar * speed_squared);
                P.at<1, 1>(velObsVar * speed_squared);
                kalman.p(P);
            }
            alignYaw();
            vel_fuse_running = true;
        } else {
            ftype total_w = 0.0f;
            ftype newWeight[(uint8_t)N_MODELS_EKFGSF];
            bool state_update_failed = false;
            for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
                // Update states and covariances using GPS NE velocity measurements fused as direct state observations
                if (!correct(mdl_idx, vel, velObsVar)) {
                    state_update_failed = true;
                }
            }

            if (!state_update_failed) {
                // Calculate weighting for each model assuming a normal error distribution
                const ftype min_weight = 1e-5f;
                n_clips = 0;
                for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
                    newWeight[mdl_idx] = gaussianDensity(mdl_idx) * GSF.weights[mdl_idx];
                    if (newWeight[mdl_idx] < min_weight) {
                        n_clips++;
                        newWeight[mdl_idx] = min_weight;
                    }
                    total_w += newWeight[mdl_idx];
                }

                // Normalise the sum of weights to unity
                // Reset the filters if all weights have underflowed due to excessive innovation variances
                if (vel_fuse_running && n_clips < N_MODELS_EKFGSF) {
                    ftype total_w_inv = 1.0f / total_w;
                    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
                        GSF.weights[mdl_idx]  = newWeight[mdl_idx] * total_w_inv;
                    }
                } else {
                    resetEKFGSF();
                }
            }
        }
    }
}

void EKFGSF_yaw::predictAHRS(const uint8_t mdl_idx)
{
    // Generate attitude solution using simple complementary filter for the selected model

    // Calculate 'k' unit vector of earth frame rotated into body frame
    const Vector3F k{AHRS[mdl_idx].R[2][0], AHRS[mdl_idx].R[2][1], AHRS[mdl_idx].R[2][2]};

    // Calculate angular rate vector in rad/sec averaged across last sample interval
    const Vector3F ang_rate_delayed_raw { delta_angle / angle_dt };

    // Perform angular rate correction using accel data and reduce correction as accel magnitude moves away from 1 g (reduces drift when vehicle picked up and moved).
    // During fixed wing flight, compensate for centripetal acceleration assuming coordinated turns and X axis forward

    Vector3F tilt_error_gyro_correction; // (rad/sec)

    if (accel_gain > 0.0f) {

        Vector3F accel = ahrs_accel;

        if (is_positive(true_airspeed)) {
            // Calculate centripetal acceleration in body frame from cross product of body rate and body frame airspeed vector
            // NOTE: this assumes X axis is aligned with airspeed vector
            const Vector3F centripetal_accel_vec_bf {
                0.0f,
                ang_rate_delayed_raw[2] * true_airspeed,
                - ang_rate_delayed_raw[1] * true_airspeed
            };
            // Correct measured accel for centripetal acceleration
            accel -= centripetal_accel_vec_bf;
        }

        tilt_error_gyro_correction = (k % accel) * (accel_gain / ahrs_accel_norm);

    }

    // Gyro bias estimation
    const ftype gyro_bias_limit = radians(5.0f);
    const ftype spinRate_squared = ang_rate_delayed_raw.length_squared();
    if (spinRate_squared < sq(0.175f)) {
        AHRS[mdl_idx].gyro_bias -= tilt_error_gyro_correction * (EKFGSF_gyroBiasGain * angle_dt);

        // sanity check
        if (AHRS[mdl_idx].gyro_bias.is_nan()) {
            AHRS[mdl_idx].gyro_bias.zero();
        }

        for (uint8_t i = 0; i < 3; i++) {
            AHRS[mdl_idx].gyro_bias[i] = constrain_ftype(AHRS[mdl_idx].gyro_bias[i], -gyro_bias_limit, gyro_bias_limit);
        }
    }

    // Calculate the corrected body frame rotation vector for the last sample interval and apply to the rotation matrix
    const Vector3F ahrs_delta_angle = delta_angle + (tilt_error_gyro_correction - AHRS[mdl_idx].gyro_bias) * angle_dt;
    AHRS[mdl_idx].R = updateRotMat(AHRS[mdl_idx].R, ahrs_delta_angle);

}

void EKFGSF_yaw::alignTilt()
{
    // Rotation matrix is constructed directly from acceleration measurement and will be the same for
    // all models so only need to calculate it once. Assumptions are:
    // 1) Yaw angle is zero - yaw is aligned later for each model when velocity fusion commences.
    // 2) The vehicle is not accelerating so all of the measured acceleration is due to gravity.

    // Calculate earth frame Down axis unit vector rotated into body frame
    Vector3F down_in_bf = -delta_velocity;
    down_in_bf.normalize();

    // Calculate earth frame North axis unit vector rotated into body frame, orthogonal to 'down_in_bf'
    // * operator is overloaded to provide a dot product
    const Vector3F i_vec_bf(1.0f,0.0f,0.0f);
    Vector3F north_in_bf = i_vec_bf - down_in_bf * (i_vec_bf * down_in_bf);
    north_in_bf.normalize();

    // Calculate earth frame East axis unit vector rotated into body frame, orthogonal to 'down_in_bf' and 'north_in_bf'
    // % operator is overloaded to provide a cross product
    const Vector3F east_in_bf = down_in_bf % north_in_bf;

    // Each column in a rotation matrix from earth frame to body frame represents the projection of the
    // corresponding earth frame unit vector rotated into the body frame, eg 'north_in_bf' would be the first column.
    // We need the rotation matrix from body frame to earth frame so the earth frame unit vectors rotated into body
    // frame are copied into corresponding rows instead to create the transpose.
    Matrix3F R;
    for (uint8_t col=0; col<3; col++) {
        R[0][col] = north_in_bf[col];
        R[1][col] = east_in_bf[col];
        R[2][col] = down_in_bf[col];
    }

    // record alignment
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        AHRS[mdl_idx].R = R;
        AHRS[mdl_idx].aligned = true;
    }
}

void EKFGSF_yaw::alignYaw()
{
    // Align yaw angle for each model
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        if (fabsF(AHRS[mdl_idx].R[2][0]) < fabsF(AHRS[mdl_idx].R[2][1])) {
            // get the roll, pitch, yaw estimates from the rotation matrix using a  321 Tait-Bryan rotation sequence
            ftype roll,pitch,yaw;
            AHRS[mdl_idx].R.to_euler(&roll, &pitch, &yaw);

            // set the yaw angle
            yaw = wrap_PI(yawState(mdl_idx));

            // update the body to earth frame rotation matrix
            AHRS[mdl_idx].R.from_euler(roll, pitch, yaw);

        } else {
            // Calculate the 312 Tait-Bryan rotation sequence that rotates from earth to body frame
            Vector3F euler312 = AHRS[mdl_idx].R.to_euler312();
            euler312[2] = wrap_PI(yawState(mdl_idx)); // first rotation (yaw) taken from EKF model state

            // update the body to earth frame rotation matrix
            AHRS[mdl_idx].R.from_euler312(euler312[0], euler312[1], euler312[2]);

        }
    }
}

// predict states and covariance for specified model index
void EKFGSF_yaw::predict(const uint8_t mdl_idx)
{
    // generate an attitude reference using IMU data
    predictAHRS(mdl_idx);

    // we don't start running the EKF part of the algorithm until there are regular velocity observations
    if (!vel_fuse_running) {
        return;
    }

    auto &kalman = filter(mdl_idx).kalman;

    // Calculate the yaw state using a projection onto the horizontal that avoids gimbal lock
    ftype yaw;
    if (fabsF(AHRS[mdl_idx].R[2][0]) < fabsF(AHRS[mdl_idx].R[2][1])) {
        // use 321 Tait-Bryan rotation to define yaw state
        yaw = atan2F(AHRS[mdl_idx].R[1][0], AHRS[mdl_idx].R[0][0]);
    } else {
        // use 312 Tait-Bryan rotation to define yaw state
        yaw = atan2F(-AHRS[mdl_idx].R[0][1], AHRS[mdl_idx].R[1][1]); // first rotation (yaw)
    }
    kalman.x(State{kalman.x().at<0>(), kalman.x().at<1>(), yaw * radian});

    // the delta velocity in the earth frame
    const Vector3F del_vel_NED = AHRS[mdl_idx].R * delta_velocity;

    // Use fixed values for delta velocity and delta angle process noise variances
    const ftype dvxVar = sq(EKFGSF_accelNoise * velocity_dt); // variance of forward and right delta velocity - (m/s)^2
    const ftype dazVar = sq(EKFGSF_gyroNoise * angle_dt); // variance of yaw delta angle - rad^2

    // sum delta velocities in earth frame, predict and symmetrize the covariance
    kalman.predict(del_vel_NED[0] * metre_per_second, del_vel_NED[1] * metre_per_second,
                   dvxVar * speed_squared, dazVar * radian_squared);

    Covariance P{kalman.p()};
    constrainVariances(P);
    kalman.p(P);
}

bool EKFGSF_yaw::correct(const uint8_t mdl_idx, const Vector2F &vel, const ftype velObsVar)
{
    auto &kalman = filter(mdl_idx).kalman;
    const State X{kalman.x()};
    const Covariance &P{kalman.p()};

    // calculate velocity observation innovations
    EKF[mdl_idx].innov[0] = X.at<0>().numerical_value_in(metre_per_second) - vel[0];
    EKF[mdl_idx].innov[1] = X.at<1>().numerical_value_in(metre_per_second) - vel[1];

    // copy covariance matrix to temporary variables
    const ftype P00 = P.at<0, 0>().numerical_value_in(speed_squared);
    const ftype P01 = P.at<0, 1>().numerical_value_in(speed_squared);
    const ftype P10 = P.at<1, 0>().numerical_value_in(speed_squared);
    const ftype P11 = P.at<1, 1>().numerical_value_in(speed_squared);

    // calculate innovation variance
    EKF[mdl_idx].S[0][0] = P00 + velObsVar;
    EKF[mdl_idx].S[1][1] = P11 + velObsVar;
    EKF[mdl_idx].S[0][1] = P01;
    EKF[mdl_idx].S[1][0] = P10;

    // Perform a chi-square innovation consistency test and calculate a compression scale factor that limits the magnitude of innovations to 5-sigma
    ftype S_det_inv = (EKF[mdl_idx].S[0][0]*EKF[mdl_idx].S[1][1] - EKF[mdl_idx].S[0][1]*EKF[mdl_idx].S[1][0]);
    ftype innov_comp_scale_factor = 1.0f;
    if (fabsF(S_det_inv) > 1E-6f) {
        // Calculate elements for innovation covariance inverse matrix assuming symmetry
        S_det_inv = 1.0f / S_det_inv;
        const ftype S_inv_NN = EKF[mdl_idx].S[1][1] * S_det_inv;
        const ftype S_inv_EE = EKF[mdl_idx].S[0][0] * S_det_inv;
        const ftype S_inv_NE = EKF[mdl_idx].S[0][1] * S_det_inv;

        // The following expression was derived symbolically from test ratio = transpose(innovation) * inverse(innovation variance) * innovation = [1x2] * [2,2] * [2,1] = [1,1]
        const ftype test_ratio = EKF[mdl_idx].innov[0]*(EKF[mdl_idx].innov[0]*S_inv_NN + EKF[mdl_idx].innov[1]*S_inv_NE) + EKF[mdl_idx].innov[1]*(EKF[mdl_idx].innov[0]*S_inv_NE + EKF[mdl_idx].innov[1]*S_inv_EE);

        // If the test ratio is greater than 25 (5 Sigma) then reduce the length of the innovation vector to clip it at 5-Sigma
        // This protects from large measurement spikes
        if (test_ratio > 25.0f) {
            innov_comp_scale_factor = sqrtF(25.0f / test_ratio);
        }
    } else {
        // skip this fusion step because calculation is badly conditioned
        return false;
    }

    // Fuse the velocity as a direct observation of the velocity states, the
    // innovations compressed: K S K^T removed from the covariance, symmetrized
    OutputVariance R{};
    R.at<0, 0>(velObsVar * speed_squared);
    R.at<1, 1>(velObsVar * speed_squared);
    kalman.r(R);
    const ftype yaw_prev = X.at<2>().numerical_value_in(radian);
    kalman.update(Output{X.at<0>() - EKF[mdl_idx].innov[0] * innov_comp_scale_factor * metre_per_second,
                         X.at<1>() - EKF[mdl_idx].innov[1] * innov_comp_scale_factor * metre_per_second});

    Covariance P_new{kalman.p()};
    constrainVariances(P_new);
    kalman.p(P_new);

    const ftype yaw_delta = yawState(mdl_idx) - yaw_prev;

    // apply the change in yaw angle to the AHRS taking advantage of sparseness in the yaw rotation matrix
    const ftype cos_yaw = cosF(yaw_delta);
    const ftype sin_yaw = sinF(yaw_delta);
    ftype  R_prev[2][3];
    memcpy(&R_prev, &AHRS[mdl_idx].R, sizeof(R_prev)); // copy first two rows from 3x3
    AHRS[mdl_idx].R[0][0] = R_prev[0][0] * cos_yaw - R_prev[1][0] * sin_yaw;
    AHRS[mdl_idx].R[0][1] = R_prev[0][1] * cos_yaw - R_prev[1][1] * sin_yaw;
    AHRS[mdl_idx].R[0][2] = R_prev[0][2] * cos_yaw - R_prev[1][2] * sin_yaw;
    AHRS[mdl_idx].R[1][0] = R_prev[0][0] * sin_yaw + R_prev[1][0] * cos_yaw;
    AHRS[mdl_idx].R[1][1] = R_prev[0][1] * sin_yaw + R_prev[1][1] * cos_yaw;
    AHRS[mdl_idx].R[1][2] = R_prev[0][2] * sin_yaw + R_prev[1][2] * cos_yaw;

    return true;
}

void EKFGSF_yaw::resetEKFGSF()
{
    memset(&GSF, 0, sizeof(GSF));
    vel_fuse_running = false;
    run_ekf_gsf = false;

    memset(&EKF, 0, sizeof(EKF));
    const ftype yaw_increment = M_2PI / (ftype)N_MODELS_EKFGSF;
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        auto &kalman = filter(mdl_idx).kalman;

        // evenly space initial yaw estimates in the region between +-Pi
        kalman.x(State{ftype(0) * metre_per_second, ftype(0) * metre_per_second,
                       (-M_PI + (0.5f * yaw_increment) + ((ftype)mdl_idx * yaw_increment)) * radian});

        // All filter models start with the same weight
        GSF.weights[mdl_idx] = 1.0f / (ftype)N_MODELS_EKFGSF;

        // Use half yaw interval for yaw uncertainty as that is the maximum that the best model can be away from truth
        GSF.yaw_variance = sq(0.5f * yaw_increment);
        Covariance P{};
        P.at<2, 2>(GSF.yaw_variance * radian_squared);
        kalman.p(P);
    }
}

// returns the probability of a selected model output assuming a gaussian error distribution
ftype EKFGSF_yaw::gaussianDensity(const uint8_t mdl_idx) const
{
    const ftype t2 = EKF[mdl_idx].S[0][0] * EKF[mdl_idx].S[1][1];
    const ftype t5 = EKF[mdl_idx].S[0][1] * EKF[mdl_idx].S[1][0];
    const ftype t3 = t2 - t5; // determinant
    const ftype t4 = 1.0f / MAX(t3, 1e-12f); // determinant inverse

    // inv(S)
    ftype invMat[2][2];
    invMat[0][0] =   t4 * EKF[mdl_idx].S[1][1];
    invMat[1][1] =   t4 * EKF[mdl_idx].S[0][0];
    invMat[0][1] = - t4 * EKF[mdl_idx].S[0][1];
    invMat[1][0] = - t4 * EKF[mdl_idx].S[1][0];

    // inv(S) * innovation
    ftype tempVec[2];
    tempVec[0] = invMat[0][0] * EKF[mdl_idx].innov[0] + invMat[0][1] * EKF[mdl_idx].innov[1];
    tempVec[1] = invMat[1][0] * EKF[mdl_idx].innov[0] + invMat[1][1] * EKF[mdl_idx].innov[1];

    // transpose(innovation) * inv(S) * innovation
    ftype normDist = tempVec[0] * EKF[mdl_idx].innov[0] + tempVec[1] * EKF[mdl_idx].innov[1];

    // convert from a normalised variance to a probability assuming a Gaussian distribution
    normDist = expf(-0.5f * normDist);
    normDist *= sqrtF(t4)/ M_2PI;
    return normDist;
}

// Apply a body frame delta angle to the body to earth frame rotation matrix using a small angle approximation
Matrix3F EKFGSF_yaw::updateRotMat(const Matrix3F &R, const Vector3F &g) const
{
    Matrix3F ret = R;
    ret[0][0] += R[0][1] * g[2] - R[0][2] * g[1];
    ret[0][1] += R[0][2] * g[0] - R[0][0] * g[2];
    ret[0][2] += R[0][0] * g[1] - R[0][1] * g[0];
    ret[1][0] += R[1][1] * g[2] - R[1][2] * g[1];
    ret[1][1] += R[1][2] * g[0] - R[1][0] * g[2];
    ret[1][2] += R[1][0] * g[1] - R[1][1] * g[0];
    ret[2][0] += R[2][1] * g[2] - R[2][2] * g[1];
    ret[2][1] += R[2][2] * g[0] - R[2][0] * g[2];
    ret[2][2] += R[2][0] * g[1] - R[2][1] * g[0];

    // Renormalise rows
    ftype rowLengthSq;
    for (uint8_t r = 0; r < 3; r++) {
        rowLengthSq = ret[r][0] * ret[r][0] + ret[r][1] * ret[r][1] + ret[r][2] * ret[r][2];
        if (is_positive(rowLengthSq)) {
            // Use linear approximation for inverse sqrt taking advantage of the row length being close to 1.0
            const ftype rowLengthInv = 1.5f - 0.5f * rowLengthSq;
            Vector3F &row = ret[r];
            row *= rowLengthInv;
        }
    }

    return ret;
}

// returns true if a yaw estimate is available.  yaw and its variance
// is returned, as well as the number of models which are *not* being
// used to snthesise the yaw.
bool EKFGSF_yaw::getYawData(ftype &yaw, ftype &yawVariance, uint8_t *_n_clips) const
{
    if (!vel_fuse_running) {
        return false;
    }
    yaw = GSF.yaw;
    yawVariance = GSF.yaw_variance;
    if (_n_clips != nullptr) {
        *_n_clips = n_clips;
    }
    return true;
}

bool EKFGSF_yaw::getVelInnovLength(ftype &velInnovLength) const
{
    if (!vel_fuse_running) {
        return false;
    }
    velInnovLength = 0.0f;
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        velInnovLength += GSF.weights[mdl_idx] * sqrtF((sq(EKF[mdl_idx].innov[0]) + sq(EKF[mdl_idx].innov[1])));
    }
    return true;
}

void EKFGSF_yaw::setGyroBias(Vector3f &gyroBias)
{
    for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
        AHRS[mdl_idx].gyro_bias = gyroBias.toftype();
    }
}
