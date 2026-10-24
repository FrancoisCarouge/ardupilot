#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"

#include "AP_NavEKF3_core.h"

#if EK3_FEATURE_OPTFLOW_FUSION

#include <GCS_MAVLink/GCS.h>
#include <AP_DAL/AP_DAL.h>

#include "AP_NavEKF3_typed.h"

#if EK3_FEATURE_OPTFLOW_AGL_KF
#include <AP_LinearAlgebra/AP_LinearAlgebra_Kalman.h>

#include <new>

namespace {

using NavEKF3_typed::Quantity;
using NavEKF3_typed::Position;
using NavEKF3_typed::Velocity;
constexpr auto metre = mp_units::si::metre;
constexpr auto second = mp_units::si::second;
constexpr auto metre_per_second = metre / second;
constexpr auto acceleration = metre / (second * second);
using Seconds = Quantity<second>;
using HeightVariance = Quantity<metre * metre>;
using VelocityVariance = Quantity<metre_per_second * metre_per_second>;

// state: AGL height and velocity, positive up; input: the vertical acceleration, positive down;
// output: the tilt corrected rangefinder height
using AglState = AP_LinearAlgebra::ColumnVector<ftype, Position, Velocity>;
using AglInput = AP_LinearAlgebra::ColumnVector<ftype, Quantity<acceleration>>;
using AglOutput = AP_LinearAlgebra::ColumnVector<ftype, Position>;
using AglCovariance = AP_LinearAlgebra::OuterProduct<AglState, AglState>;
using AglOutputVariance = AP_LinearAlgebra::OuterProduct<AglOutput, AglOutput>;
using AglTransition = AP_LinearAlgebra::Quotient<AglState, AglState>;
using AglControl = AP_LinearAlgebra::Quotient<AglState, AglInput>;

/*
  h(k+1) = h(k) + v(k)*dt, v(k+1) = v(k) - a*dt, so
  F = |1 dt|   G = |  0|   Q = |Qhgt     0|
      |0  1|       |-dt|       |   0  Qvel|
  and the rangefinder height measurement, H = [1 0]
 */
auto make_agl_filter()
{
    using namespace fcarouge;
    return kalman{
        state{AglState{}},
        output<AglOutput>,
        input<AglInput>,
        estimate_uncertainty{AglCovariance{}},
        process_uncertainty{[](const AglState &, const Seconds &, const HeightVariance &Qhgt, const VelocityVariance &Qvel) -> AglCovariance {
            AglCovariance q{};
            q.at<0, 0>(Qhgt);
            q.at<1, 1>(Qvel);
            return q;
        }},
        // set by each measurement update
        output_uncertainty{AglOutputVariance{}},
        state_transition{[](const AglInput &, const Seconds &dt, const HeightVariance &, const VelocityVariance &) -> AglTransition {
            AglTransition f{};
            f.at<0, 0>(ftype(1) * mp_units::one);
            f.at<0, 1>(dt);
            f.at<1, 1>(ftype(1) * mp_units::one);
            return f;
        }},
        input_control{[](const Seconds &dt, const HeightVariance &, const VelocityVariance &) -> AglControl {
            AglControl g{};
            g.at<1>(-dt);
            return g;
        }},
        prediction_types<Seconds, HeightVariance, VelocityVariance>};
}

} // namespace

// the filter does not exist until the variables are initialised: its values then read as zero,
// as the members of the zero allocated core it replaces did
struct NavEKF3_core::AglKf {
    decltype(make_agl_filter()) kalman{make_agl_filter()};
};

void NavEKF3_core::resetAglKf()
{
    static_assert(sizeof(AglKf) <= aglKfSize, "NavEKF3_core::aglKfSize is too small for the AGL filter");
    static_assert(alignof(AglKf) <= 8, "NavEKF3_core::aglKfStorage is under-aligned for the AGL filter");
    static_assert(std::is_trivially_destructible_v<AglKf>, "NavEKF3_core does not destroy the AGL filter");
    // constructed here, at run time, not with the object: the Kalman library's identity and zero values
    // are dynamically initialized variables
    aglKf = new (aglKfStorage) AglKf;
    auto &kalman = aglKf->kalman;
    // assume sitting on ground at minimum range, 5 m initial std-dev in height, 1 m/s in velocity
    kalman.x(AglState{rngOnGnd * metre, ftype(0) * metre_per_second});
    AglCovariance p{};
    p.at<0, 0>(ftype(25) * (metre * metre));
    p.at<1, 1>(ftype(1) * (metre_per_second * metre_per_second));
    kalman.p(p);
}

ftype NavEKF3_core::aglKfHeight() const
{
    if (aglKf == nullptr) {
        return 0;
    }
    return aglKf->kalman.x().at<0>().numerical_value_in(metre);
}

ftype NavEKF3_core::aglKfVelocity() const
{
    if (aglKf == nullptr) {
        return 0;
    }
    return aglKf->kalman.x().at<1>().numerical_value_in(metre_per_second);
}

ftype NavEKF3_core::aglKfHeightVariance() const
{
    if (aglKf == nullptr) {
        return 0;
    }
    return aglKf->kalman.p().at<0, 0>().numerical_value_in(metre * metre);
}

ftype NavEKF3_core::aglKfVelocityVariance() const
{
    if (aglKf == nullptr) {
        return 0;
    }
    return aglKf->kalman.p().at<1, 1>().numerical_value_in(metre_per_second * metre_per_second);
}
#endif // EK3_FEATURE_OPTFLOW_AGL_KF

#define P (const_cast<const Matrix24 &>(Pmut))

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of optical flow measurements
void NavEKF3_core::SelectFlowFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !optFlowFusionDelayed) {
        optFlowFusionDelayed = true;
        return;
    } else {
        optFlowFusionDelayed = false;
    }

    of_elements ofDataDelayed;      // OF data at the fusion time horizon

    // Check for data at the fusion time horizon
    const bool flowDataToFuse = storedOF.recall(ofDataDelayed, imuDataDelayed.time_ms);

    // Perform Data Checks
    // Check if the optical flow data is still valid
    flowDataValid = ((imuSampleTime_ms - flowValidMeaTime_ms) < 1000);
    // check is the terrain offset estimate is still valid - if we are using range finder as the main height reference, the ground is assumed to be at 0
    gndOffsetValid = ((imuSampleTime_ms - gndHgtValidTime_ms) < 5000) || (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER);
    // Perform tilt check
    bool tiltOK = (prevTnb.c.z > frontend->DCM33FlowMin);
    // Constrain measurements to zero if takeoff is not detected and the height above ground
    // is insufficient to achieve acceptable focus. This allows the vehicle to be picked up
    // and carried to test optical flow operation
    if (!takeOffDetected && ((terrainState - stateStruct.position.z) < 0.5f)) {
        ofDataDelayed.flowRadXYcomp.zero();
        ofDataDelayed.flowRadXY.zero();
        flowDataValid = true;
    }

    // if have valid flow or range measurements, fuse data into a 1-state EKF to estimate terrain height
    if (((flowDataToFuse && (frontend->_flowUse == FLOW_USE_TERRAIN)) || rangeDataToFuse) && tiltOK) {
        // Estimate the terrain offset (runs a one state EKF)
        EstimateTerrainOffset(ofDataDelayed);
    }

#if EK3_FEATURE_OPTFLOW_AGL_KF
    // Update the IMU-aided AGL KF every IMU step when enabled, regardless of flow/RF data presence.
    if (frontend->option_is_enabled(NavEKF3::Option::AglKfForOptflow)) {
        UpdateAglKf();
    }
#endif

    // Fuse optical flow data into the main filter
    if (flowDataToFuse && tiltOK) {
        const bool fuse_optflow = (frontend->_flowUse == FLOW_USE_NAV) && frontend->sources.useVelXYSource(AP_NavEKF_Source::SourceXY::OPTFLOW, core_index);
        // Set the flow noise used by the fusion processes
        R_LOS = sq(MAX(frontend->_flowNoise, 0.05f));
        // Fuse the optical flow X and Y axis data into the main filter sequentially
        FuseOptFlow(ofDataDelayed, fuse_optflow);
    }
}

/*
Estimation of terrain offset using a single state EKF
The filter can fuse motion compensated optical flow rates and range finder measurements
Equations generated using https://github.com/PX4/ecl/tree/master/EKF/matlab/scripts/Terrain%20Estimator
*/
void NavEKF3_core::EstimateTerrainOffset(const of_elements &ofDataDelayed)
{
    using namespace NavEKF3_typed;
    using mp_units::si::metre;
    using mp_units::si::second;
    using mp_units::si::radian;
    using Variance = Quantity<square(metre)>;
    using RateVariance = Quantity<square(radian / second)>;

    // horizontal velocity squared
    ftype velHorizSq = sq(stateStruct.velocity.x) + sq(stateStruct.velocity.y);

    // don't fuse flow data if LOS rate is misaligned, without GPS, or insufficient velocity, as it is poorly observable
    // don't fuse flow data if it exceeds validity limits
    // don't update terrain offset if ground is being used as the zero height datum in the main filter
    bool cantFuseFlowData = ((frontend->_flowUse != FLOW_USE_TERRAIN)
    || !gpsIsInUse
    || PV_AidingMode == AID_RELATIVE
    || velHorizSq < 25.0f
    || (MAX(ofDataDelayed.flowRadXY[0],ofDataDelayed.flowRadXY[1]) > frontend->_maxFlowRate));

    if ((!rangeDataToFuse && cantFuseFlowData) || (activeHgtSource == AP_NavEKF_Source::SourceZ::RANGEFINDER)) {
        // skip update
        inhibitGndState = true;
    } else {
        inhibitGndState = false;

        // the terrain vertical position (down) and its variance
        Position terrain = terrainState * metre;
        Variance variance = Popt * square(metre);
        const Position pd = stateStruct.position.z * metre;
        const Position rangeOnGround = rngOnGnd * metre;
        // the terrain is kept below the vehicle
        const Position lowestTerrain = pd + rangeOnGround;

        // propagate ground position state noise each time this is called using the difference in position since the last observations and an RMS gradient assumption
        // limit distance to prevent intialisation after bad gps causing bad numerical conditioning
        ftype distanceTravelledSq = sq(stateStruct.position[0] - prevPosN) + sq(stateStruct.position[1] - prevPosE);
        distanceTravelledSq = MIN(distanceTravelledSq, 100.0f);
        prevPosN = stateStruct.position[0];
        prevPosE = stateStruct.position[1];

        // in addition to a terrain gradient error model, we also have the growth in uncertainty due to the copter's vertical velocity
        const Quantity<second> timeLapsed = MIN(0.001f * (imuSampleTime_ms - timeAtLastAuxEKF_ms), 1.0f) * second;
        const Quantity<square(metre / second)> velDownVariance = P[6][6] * square(metre / second);
        const Unitless terrainGradient = ftype(frontend->_terrGradMax.get()) * mp_units::one;
        const Variance Pincrement = (distanceTravelledSq * square(metre)) * (terrainGradient * terrainGradient) + (timeLapsed * timeLapsed) * velDownVariance;
        variance += Pincrement;
        timeAtLastAuxEKF_ms = imuSampleTime_ms;

        // fuse range finder data
        if (rangeDataToFuse) {
            // reset terrain state if rangefinder data not fused for 5 seconds
            if (imuSampleTime_ms - gndHgtValidTime_ms > 5000) {
                terrain = std::max(rangeDataDelayed.rng * prevTnb.c.z * metre, rangeOnGround) + pd;
            }

            // predict range
            const Position predRngMeas = std::max(terrain - pd, rangeOnGround) / prevTnb.c.z;
            // Copy required states to local variable names
            ftype q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            ftype q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            ftype q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            ftype q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time

            // Set range finder measurement noise variance. TODO make this a function of range and tilt to allow for sensor, alignment and AHRS errors
            const Variance R_RNG = frontend->_rngNoise.get() * square(metre);

            // calculate Kalman gain: the range is the height divided by SK_RNG
            const ftype SK_RNG = sq(q0) - sq(q1) - sq(q2) + sq(q3);
            const Variance varInnov = R_RNG + variance / sq(SK_RNG);
            const Unitless K_RNG = variance / (SK_RNG * varInnov);

            // Calculate the innovation variance for data logging
            varInnovRng = varInnov.numerical_value_in(square(metre));

            // constrain terrain height to be below the vehicle
            terrain = std::max(terrain, lowestTerrain);

            // Calculate the measurement innovation
            const Position innov = predRngMeas - rangeDataDelayed.rng * metre;
            innovRng = innov.numerical_value_in(metre);

            // calculate the innovation consistency test ratio
            auxRngTestRatio = (innov * innov / varInnov).numerical_value_in(mp_units::one) / sq(MAX(0.01f * (ftype)frontend->_rngInnovGate, 1.0f));

            // Check the innovation test ratio and don't fuse if too large
            if (auxRngTestRatio < 1.0f) {
                // correct the state
                terrain -= K_RNG * innov;

                // constrain the state
                terrain = std::max(terrain, lowestTerrain);

                // correct the covariance
                variance = variance - variance * variance / (SK_RNG * varInnov * (sq(q0) - sq(q1) - sq(q2) + sq(q3)));

                // prevent the state variance from becoming negative
                variance = std::max(variance, ftype(0) * square(metre));

                // record the time we last updated the terrain offset state
                gndHgtValidTime_ms = imuSampleTime_ms;
            }
        }

        if (!cantFuseFlowData) {

            Vector3F relVelSensor;          // velocity of sensor relative to ground in sensor axes
            Vector2F losPred;               // predicted optical flow angular rate measurement
            ftype q0 = stateStruct.quat[0]; // quaternion at optical flow measurement time
            ftype q1 = stateStruct.quat[1]; // quaternion at optical flow measurement time
            ftype q2 = stateStruct.quat[2]; // quaternion at optical flow measurement time
            ftype q3 = stateStruct.quat[3]; // quaternion at optical flow measurement time
            const Velocity vn = stateStruct.velocity.x * NavEKF3_typed::metre_per_second;
            const Velocity ve = stateStruct.velocity.y * NavEKF3_typed::metre_per_second;
            const Velocity vd = stateStruct.velocity.z * NavEKF3_typed::metre_per_second;

            // predict range to centre of image
            const Position flowRngPred = std::max(terrain - pd, rangeOnGround) / prevTnb.c.z;

            // constrain terrain height to be below the vehicle
            terrain = std::max(terrain, lowestTerrain);

            // calculate relative velocity in sensor frame
            relVelSensor = prevTnb*stateStruct.velocity;

            // divide velocity by range, subtract body rates and apply scale factor to
            // get predicted sensed angular optical rates relative to X and Y sensor axes
            losPred.x =   relVelSensor.y / flowRngPred.numerical_value_in(metre);
            losPred.y = - relVelSensor.x / flowRngPred.numerical_value_in(metre);

            // calculate innovations, of both axes before either is fused
            auxFlowObsInnov = losPred - ofDataDelayed.flowRadXYcomp;
            constexpr auto rate = radian / second;

            // calculate observation jacobians: the rate is the velocity divided by the height
            ftype t2 = q0*q0;
            ftype t3 = q1*q1;
            ftype t4 = q2*q2;
            ftype t5 = q3*q3;
            Position t6 = pd - terrain;
            auto t7 = ftype(1) * radian / (t6*t6);
            ftype t8 = q0*q3*2;
            ftype t9 = t2-t3-t4+t5;

            // prevent the state variances from becoming badly conditioned
            const Variance minVariance = ftype(1E-6f) * square(metre);
            variance = std::max(variance, minVariance);

            // calculate observation noise variance from parameter
            const RateVariance flow_noise_variance = sq(MAX(frontend->_flowNoise, 0.05f)) * square(rate);
            const ftype gate = sq(MAX(0.01f * (ftype)frontend->_flowInnovGate, 1.0f));

            // Fuse Y axis data

            // Calculate observation partial derivative
            {
                const auto H_OPT = t7*t9*(-vd*(q0*q2*2-q1*q3*2)+vn*(t2+t3-t4-t5)+ve*(t8+q1*q2*2));

                // calculate innovation variance
                const RateVariance innovVar = H_OPT * variance * H_OPT + flow_noise_variance;

                // calculate Kalman gain
                const auto K_OPT = variance * H_OPT / innovVar;

                // calculate the innovation consistency test ratio
                const Quantity<rate> innov = auxFlowObsInnov.y * rate;
                auxFlowTestRatio.y = (innov * innov / innovVar).numerical_value_in(mp_units::one) / gate;

                // don't fuse if optical flow data is outside valid range
                if (auxFlowTestRatio.y < 1.0f) {

                    // correct the state
                    terrain -= K_OPT * innov;

                    // constrain the state
                    terrain = std::max(terrain, lowestTerrain);

                    // update intermediate variables used when fusing the X axis
                    t6 = pd - terrain;
                    t7 = ftype(1) * radian / (t6*t6);

                    // correct the covariance
                    variance = variance - K_OPT * H_OPT * variance;

                    // prevent the state variances from becoming badly conditioned
                    variance = std::max(variance, minVariance);

                    // record the time we last updated the terrain offset state
                    gndHgtValidTime_ms = imuSampleTime_ms;
                }
            }

            // fuse X axis data
            {
                const auto H_OPT = -t7*t9*(vd*(q0*q1*2+q2*q3*2)+ve*(t2-t3+t4-t5)-vn*(t8-q1*q2*2));

                // calculate innovation variances
                const RateVariance innovVar = H_OPT * variance * H_OPT + flow_noise_variance;

                // calculate Kalman gain
                const auto K_OPT = variance * H_OPT / innovVar;

                // calculate the innovation consistency test ratio
                const Quantity<rate> innov = auxFlowObsInnov.x * rate;
                auxFlowTestRatio.x = (innov * innov / innovVar).numerical_value_in(mp_units::one) / gate;

                // don't fuse if optical flow data is outside valid range
                if (auxFlowTestRatio.x < 1.0f) {

                    // correct the state
                    terrain -= K_OPT * innov;

                    // constrain the state
                    terrain = std::max(terrain, lowestTerrain);

                    // correct the covariance
                    variance = variance - K_OPT * H_OPT * variance;

                    // prevent the state variances from becoming badly conditioned
                    variance = std::max(variance, minVariance);
                }
            }
        }

        terrainState = terrain.numerical_value_in(metre);
        Popt = variance.numerical_value_in(square(metre));
    }
}

/*
 * Fuse angular motion compensated optical flow rates using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
 * Requires a valid terrain height estimate.
 *
 * really_fuse should be true to actually fuse into the main filter, false to only calculate variances
*/
void NavEKF3_core::FuseOptFlow(const of_elements &ofDataDelayed, bool really_fuse)
{
    using namespace NavEKF3_typed;
    // the line of sight rate, in rad/s
    using LosRate = Quantity<mp_units::si::radian / mp_units::si::second>;
    const Quantity<square(mp_units::si::radian / mp_units::si::second)> R_LOS_typed = R_LOS * square(mp_units::si::radian / mp_units::si::second);
    Vector2 losPred;

    // Copy required states to local variable names: the observation Jacobians of both axes use the
    // states before the fusion of the X axis
    const StateView states{AP_LinearAlgebra::MatrixView<ftype, 24, 1>{&statesArray[0]}};
    const Unitless q0 = states.at<0>();
    const Unitless q1 = states.at<1>();
    const Unitless q2 = states.at<2>();
    const Unitless q3 = states.at<3>();
    const Velocity vn = states.at<4>();
    const Velocity ve = states.at<5>();
    const Velocity vd = states.at<6>();
    ftype pd = stateStruct.position.z;

    // Default is the terrain estimator AGL (terrainState - pd, where pd is the main filter's vertical position)
    // constrain height above ground to be above range measured on ground
    ftype heightAboveGndEst = MAX((terrainState - pd), rngOnGnd);

#if EK3_FEATURE_OPTFLOW_SRTM
    // if ground offset (aka terrainState) is not valid, use SRTM altitude. terrain_srtm_alt
    // is positive up from the origin where pd and terrainState above are positive down
    terrain_srtm_alt_valid = (terrain_srtm_alt_ms != 0) &&
                             ((imuSampleTime_ms - terrain_srtm_alt_ms) < TERRAIN_SRTM_ALT_TIMEOUT_MS);
    if (!gndOffsetValid && terrain_srtm_alt_valid) {
        heightAboveGndEst = MAX((-pd) - terrain_srtm_alt, rngOnGnd);
    }
#endif

#if EK3_FEATURE_OPTFLOW_AGL_KF
    // AGL KF override: use the IMU-aided AGL KF estimate when enabled and valid,
    // instead of terrainState-pd which can drift when the main filter's vertical position
    // state is unreliable (e.g. poor altitude source, sensor outage, or ground effect).
    if (frontend->option_is_enabled(NavEKF3::Option::AglKfForOptflow) && aglKfValid) {
        heightAboveGndEst = MAX(aglKfHeight(), rngOnGnd);
    }
#endif

    // calculate range from ground plain to centre of sensor fov assuming flat earth
    ftype range = constrain_ftype((heightAboveGndEst/prevTnb.c.z),rngOnGnd,1000.0f);

    // correct range for flow sensor offset body frame position offset
    // the corrected value is the predicted range from the sensor focal point to the
    // centre of the image on the ground assuming flat terrain
    Vector3F posOffsetBody = ofDataDelayed.body_offset - accelPosOffset;
    if (!posOffsetBody.is_zero()) {
        Vector3F posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
        range -= posOffsetEarth.z / prevTnb.c.z;
    }
    
#if APM_BUILD_TYPE(APM_BUILD_Rover)
    // override with user specified height (if given, for rover)
    if (ofDataDelayed.heightOverride > 0) {
        range = ofDataDelayed.heightOverride;
    }
#endif

    // Fuse X and Y axis measurements sequentially assuming observation errors are uncorrelated
    for (uint8_t obsIndex=0; obsIndex<=1; obsIndex++) { // fuse X axis data first

        // calculate relative velocity in sensor frame including the relative motion due to rotation
        const Vector3F relVelSensor = (prevTnb * stateStruct.velocity) + (ofDataDelayed.bodyRadXYZ % posOffsetBody);

        // divide velocity by range to get predicted angular LOS rates relative to X and Y axes
        losPred[0] =  relVelSensor.y/range;
        losPred[1] = -relVelSensor.x/range;

        // calculate observation jacobians and Kalman gains
        const auto t2 = (ftype(1) * mp_units::si::radian) / (range * mp_units::si::metre);
        ObservationModel<LosRate> H_LOS{};
        if (obsIndex == 0) {
            // calculate X axis observation Jacobian
            H_LOS.at<0>(t2*(q1*vd*2+q0*ve*2-q3*vn*2));
            H_LOS.at<1>(t2*(q0*vd*2-q1*ve*2+q2*vn*2));
            H_LOS.at<2>(t2*(q3*vd*2+q2*ve*2+q1*vn*2));
            H_LOS.at<3>(-t2*(q2*vd*-2+q3*ve*2+q0*vn*2));
            H_LOS.at<4>(-t2*(q0*q3*2-q1*q2*2));
            H_LOS.at<5>(t2*(q0*q0-q1*q1+q2*q2-q3*q3));
            H_LOS.at<6>(t2*(q0*q1*2+q2*q3*2));
        } else {
            // calculate Y axis observation Jacobian
            H_LOS.at<0>(-t2*(q2*vd*-2+q3*ve*2+q0*vn*2));
            H_LOS.at<1>(-t2*(q3*vd*2+q2*ve*2+q1*vn*2));
            H_LOS.at<2>(t2*(q0*vd*2-q1*ve*2+q2*vn*2));
            H_LOS.at<3>(-t2*(q1*vd*2+q0*ve*2-q3*vn*2));
            H_LOS.at<4>(-t2*(q0*q0+q1*q1-q2*q2-q3*q3));
            H_LOS.at<5>(-t2*(q0*q3*2+q1*q2*2));
            H_LOS.at<6>(t2*(q0*q2*2-q1*q3*2));
        }

        // calculate the innovation variance and protect against a badly conditioned calculation
        const CovarianceView covariance{AP_LinearAlgebra::MatrixView<ftype, 24, 24>{&P[0][0]}};
        const auto PHt = covariance * fcarouge::transposed(H_LOS);
        const auto varInnov = (H_LOS * PHt).at() + R_LOS_typed;
        const bool badlyConditioned = !(varInnov > R_LOS_typed);
        if (obsIndex == 0) {
            faultStatus.bad_xflow = badlyConditioned;
        } else {
            faultStatus.bad_yflow = badlyConditioned;
        }
        if (badlyConditioned) {
            return;
        }
        flowVarInnov[obsIndex] = varInnov.numerical_value_in(varInnov.unit);

        // calculate the innovation
        // flowInnovTime_ms is updated when the Y axis innovation is calculated
        if (obsIndex == 0) {
            flowInnov[0] = losPred[0] - ofDataDelayed.flowRadXYcomp.x;
        } else {
            flowInnov[1] = losPred[1] - ofDataDelayed.flowRadXYcomp.y;
            flowInnovTime_ms = dal.millis();
        }

        // calculate the Kalman gains, of all quaternion, velocity and position states, and of the
        // other states not inhibited
        const auto K = PHt / varInnov;
        uint32_t kalman_mask = (1<<10) - 1;

        if (!inhibitDelAngBiasStates) {
            kalman_mask |= (1<<10) | (1<<11);
            if (recentYawFusion()) {
                kalman_mask |= (1<<12);
            }
        }

        if (!inhibitDelVelBiasStates && !badIMUdata) {
            for (uint8_t index = 0; index < 3; index++) {
                const uint8_t stateIndex = index + 13;
                if (!dvelBiasAxisInhibit[index]) {
                    kalman_mask |= (1<<stateIndex);
                }
            }
        }

        if (!inhibitMagStates) {
            kalman_mask |= (1<<16) | (1<<17) | (1<<18) | (1<<19) | (1<<20) | (1<<21);
        }

        if (!inhibitWindStates && !treatWindStatesAsTruth) {
            kalman_mask |= (1<<22) | (1<<23);
        }

        for (auto i=0; i<24; i++) {
            ftype res = 0;
            if (kalman_mask & (1<<i)) {
                res = K.data()(i);
            }
            Kfusion[i] = res;
        }


        // calculate the innovation consistency test ratio
        flowTestRatio[obsIndex] = sq(flowInnov[obsIndex]) / (sq(MAX(0.01f * (ftype)frontend->_flowInnovGate, 1.0f)) * flowVarInnov[obsIndex]);

        // Check the innovation for consistency and don't fuse if out of bounds or flow is too fast to be reliable
        if (really_fuse && (flowTestRatio[obsIndex]) < 1.0f && (ofDataDelayed.flowRadXY.x < frontend->_maxFlowRate) && (ofDataDelayed.flowRadXY.y < frontend->_maxFlowRate)) {
            // record the last time observations were accepted for fusion
            prevFlowFuseTime_ms = imuSampleTime_ms;
            // notify first time only
            if (!flowFusionActive) {
                flowFusionActive = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EKF3 IMU%u fusing optical flow",(unsigned)imu_index);
            }

            // correct the covariance P = (I - K*H)*P = P - K*H*P. K*H*P is written to its
            // array, its type checked against the covariance without being evaluated.
            const auto HP = H_LOS * covariance;
            static_assert(std::is_convertible_v<decltype(K * HP), Covariance>);
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=stateIndexLim; j++) {
                    KHP[i][j] = Kfusion[i] * HP.data()(j);
                }
            }

            // finish fusion from KHP and Kfusion
            if (FinishFusion(flowInnov[obsIndex])) {
                // fault, record bad axis
                if (obsIndex == 0) {
                    faultStatus.bad_xflow = true;
                } else if (obsIndex == 1) {
                    faultStatus.bad_yflow = true;
                }

            }
        }
    }

    // store optical flow rates for use in external calibration
    flowCalSample.timestamp_ms = imuSampleTime_ms;
    flowCalSample.flowRate.x = ofDataDelayed.flowRadXY.x;
    flowCalSample.flowRate.y = ofDataDelayed.flowRadXY.y;
    flowCalSample.bodyRate.x = ofDataDelayed.bodyRadXYZ.x;
    flowCalSample.bodyRate.y = ofDataDelayed.bodyRadXYZ.y;
    flowCalSample.losPred.x = losPred[0];
    flowCalSample.losPred.y = losPred[1];
}

// retrieve latest corrected optical flow samples (used for calibration)
bool NavEKF3_core::getOptFlowSample(uint32_t& timestamp_ms, Vector2f& flowRate, Vector2f& bodyRate, Vector2f& losPred) const
{
    if (flowCalSample.timestamp_ms != 0) {
        timestamp_ms = flowCalSample.timestamp_ms;
        flowRate = flowCalSample.flowRate;
        bodyRate = flowCalSample.bodyRate;
        losPred = flowCalSample.losPred;
        return true;
    }
    return false;
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

#if EK3_FEATURE_OPTFLOW_AGL_KF
/*
 * 2-state IMU-aided AGL Kalman filter
 *
 * State:       x  = [h_agl (m), v_agl (m/s)]'   (AGL is "up")
 * Transition:  F  = [[1, imuDt], [0, 1]]
 * Input:       u  = -velDotNED.z * imuDt             (gravity-included accel)
 * Noise:       Qvel = sq(EK3_ACC_P_NSE * imuDt)
 *              Qhgt = sq(EK3_TERR_GRAD) * horizDist²
 *
 * Prediction:  x(k+1) = F*x(k) + [0, u]'
 *              P(k+1) = F*P*F' + Q
 *
 * Observation: z = rng * prevTnb.c.z,  H = [1, 0],  R = sq(EK3_RNG_M_NSE)
 * Update:      innov = z - H*x,  innovVar = H*P*H' + R
 *              K = P*H' / innovVar
 *              x += K * innov
 *              P  = (I-KH)*P*(I-KH)' + K*R*K'
 */
void NavEKF3_core::UpdateAglKf()
{
    auto &kalman = aglKf->kalman;
    const ftype imuDt = imuDataDelayed.delVelDT;
    const uint32_t aglKfRngTimeout_ms = 5000;   // mark filter invalid / hard-reset after this gap without RF fusion

    // Process noise Q:
    //
    // Qhgt — terrain-induced AGL uncertainty during forward flight.
    //   As the vehicle moves horizontally by dist over terrain with unknown gradient "g",
    //   the true AGL changes by ~g * dist.  Since "g" is unknown, we treat it as zero-mean
    //   with std-dev terrGradMax, giving variance: Qhgt = terrGradMax² * horizDist²
    //   horizDist² = (vx²+vy²)*imuDt² is the squared horizontal distance travelled this step.
    //   Capped at 1 m² so a single large-velocity step can't blow up the covariance.
    //   The intended effect is that P[0][0] grows quickly during fast horizontal flight over rough terrain,
    //   allowing the RF measurement to pull h_agl back when the next reading arrives.
    //
    // Qvel — unmodelled vertical accelerations (IMU noise, vibration, model error).
    //   Uses the same accNoise parameter as CovariancePrediction (sq(imuDt*accNoise)),
    //   so the velocity uncertainty budget is consistent with the main EKF.
    //   The intended effect is that P[1][1] grows every step when RF is absent, reflecting accumulating IMU integration error in v_agl.
    //
    const ftype horizDistSq = MIN(sq(stateStruct.velocity.x * imuDt)
                                  + sq(stateStruct.velocity.y * imuDt), 1.0f);  // cap at 1 m²
    const VelocityVariance Qvel = sq(frontend->_accNoise * imuDt) * (metre_per_second * metre_per_second);   // matches CovariancePrediction: sq(imuDt*accNoise)
    const HeightVariance Qhgt = sq(frontend->_terrGradMax) * horizDistSq * (metre * metre);

    // h_agl(k+1) = h_agl(k) + v_agl(k)*imuDt
    // v_agl(k+1) = v_agl(k) - velDotNED.z*imuDt
    // velDotNED.z is NED-down acceleration (positive = downward, includes gravity).
    // Negate: downward acceleration reduces AGL rate.
    // and P = F*P*F' + Q
    kalman.predict(imuDt * second, Qhgt, Qvel, AglInput{velDotNED.z * acceleration});

    // First-order decay of v_agl toward zero when RF is absent (tau = 2 s).
    // Without range measurements v_agl is unobservable; accumulated IMU bias
    // error will cause it to drift, pulling h_agl to the floor during
    // subsequent climbs.  The decay limits that drift.
    // At the aglKfRngTimeout_ms validity timeout (5 s), |v| is at most
    // exp(-5/2) ~ 8% of its value at last RF fusion, so the hard reset finds v near zero.
    Velocity aglKfV = kalman.x().at<1>();
    if (!rangeDataToFuse) {
        const ftype tauV = 2.0f;
        aglKfV *= expf(-imuDt / tauV);
    }

    // AGL cannot go below the on-ground sensor reading
    kalman.x(AglState{MAX(kalman.x().at<0>().numerical_value_in(metre), rngOnGnd) * metre, aglKfV});

    // Cap covariance to prevent runaway during prolonged RF absence
    AglCovariance aglKfP{kalman.p()};
    aglKfP.at<0, 0>(MIN((aglKfP.at<0, 0>().numerical_value_in(metre * metre)), 100.0f) * (metre * metre));  // 10 m std-dev cap
    aglKfP.at<1, 1>(MIN((aglKfP.at<1, 1>().numerical_value_in(metre_per_second * metre_per_second)), 100.0f) * (metre_per_second * metre_per_second));  // 10 m/s std-dev cap
    kalman.p(aglKfP);

    // mark invalid if RF has been absent too long
    if (!rangeDataToFuse) {
        if (imuSampleTime_ms - lastAglRngFuseTime_ms > aglKfRngTimeout_ms) {
            aglKfValid = false;
        }
        return;
    }

    // Only fuse when vehicle tilt is within acceptable limits
    if (prevTnb.c.z < frontend->DCM33FlowMin) {
        return;
    }

    // After the timeout of IMU-only propagation, vertical velocity drift makes the
    // prediction unreliable.  Re-initialise directly from the rangefinder.
    if (imuSampleTime_ms - lastAglRngFuseTime_ms > aglKfRngTimeout_ms) {
        // Tilt-corrected AGL directly from rangefinder reading, assume stationary on reset
        kalman.x(AglState{MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd) * metre, ftype(0) * metre_per_second});
        AglCovariance p{};
        p.at<0, 0>(sq(frontend->_rngNoise) * (metre * metre)); // initialise h uncertainty to RF noise
        p.at<1, 1>(ftype(1) * (metre_per_second * metre_per_second)); // 1 m/s velocity uncertainty after reset
        kalman.p(p);
        lastAglRngFuseTime_ms = imuSampleTime_ms;
        aglKfValid = true;
        return;  // skip measurement update this cycle (just used the reading for reset)
    }

    // Measurement update — fuse tilt-corrected rangefinder reading
    //
    // Observation model: z = h_agl,  H = [1, 0]
    //   z_meas = rng * cos(tilt) = rng * prevTnb.c.z
    //
    const Position hgtMeas = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd) * metre;

    // Measurement noise variance R (reuse EK3_RNG_M_NSE)
    const HeightVariance measNoiseVar = sq(frontend->_rngNoise) * (metre * metre);

    // Innovation and innovation covariance, as the filter update computes them, to gate the measurement
    // hgtInnov = hgtMeas - H*x = hgtMeas - h_agl
    // innovVar = H*P*H' + R    = P[0][0] + R
    const Position hgtInnov = hgtMeas - kalman.x().at<0>();
    const HeightVariance innovVar = kalman.p().at<0, 0>() + measNoiseVar;

    // reject outliers (RF glitches, specular reflections, etc.)
    // gate is expressed as a multiplier on the 1-sigma bound.
    const ftype innovGate = MAX(0.01f * (ftype)frontend->_rngInnovGate, 1.0f);
    if (hgtInnov * hgtInnov > sq(innovGate) * innovVar) {
        // Innovation too large, likely a glitch.  Inflate both height and velocity
        // uncertainty so the next valid reading can correct both states more aggressively.
        AglCovariance p{kalman.p()};
        p.at<0, 0>(MIN((p.at<0, 0>().numerical_value_in(metre * metre)) * 2.0f, 100.0f) * (metre * metre));
        p.at<1, 1>(MIN((p.at<1, 1>().numerical_value_in(metre_per_second * metre_per_second)) * 2.0f, 100.0f) * (metre_per_second * metre_per_second));
        kalman.p(p);
        return;
    }

    // Kalman gain K = P*H' / innovVar, state update x += K * hgtInnov and covariance update in Joseph
    // form P = (I-KH)*P*(I-KH)' + K*R*K'
    kalman.r(AglOutputVariance{measNoiseVar});
    kalman.update(AglOutput{hgtMeas});

    // enforce physical constraint after update
    kalman.x(AglState{MAX(kalman.x().at<0>().numerical_value_in(metre), rngOnGnd) * metre, kalman.x().at<1>()});
    AglCovariance p{kalman.p()};
    p.at<0, 0>(MAX((p.at<0, 0>().numerical_value_in(metre * metre)), 0.0f) * (metre * metre));
    p.at<1, 1>(MAX((p.at<1, 1>().numerical_value_in(metre_per_second * metre_per_second)), 0.0f) * (metre_per_second * metre_per_second));
    kalman.p(p);

    lastAglRngFuseTime_ms = imuSampleTime_ms;
    aglKfValid = true;
}

#endif  // EK3_FEATURE_OPTFLOW_AGL_KF

#endif  //  EK3_FEATURE_OPTFLOW_FUSION
