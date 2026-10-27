#include <AP_HAL/AP_HAL.h>

#include "AP_NavEKF3.h"
#include "AP_NavEKF3_core.h"
#include "AP_NavEKF3_typed.h"
#include <AP_DAL/AP_DAL.h>

#define P (const_cast<const Matrix24 &>(Pmut))

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

/*
 * Fuse true airspeed measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
*/
void NavEKF3_core::FuseAirspeed()
{
    using namespace NavEKF3_typed;
    constexpr auto velocity_variance = metre_per_second * metre_per_second;

    // copy required states to local variable names
    const StateView states{AP_LinearAlgebra::MatrixView<ftype, 24, 1>{&statesArray[0]}};
    const Velocity vn = states.at<4>();
    const Velocity ve = states.at<5>();
    const Velocity vd = states.at<6>();
    const Velocity vwn = states.at<22>();
    const Velocity vwe = states.at<23>();

    // calculate the predicted airspeed
    const ftype VtasPred = norm((ve - vwe).numerical_value_in(metre_per_second), (vn - vwn).numerical_value_in(metre_per_second), vd.numerical_value_in(metre_per_second));
    // perform fusion of True Airspeed measurement
    if (VtasPred > 1.0f)
    {
        // calculate observation innovation and variance
        innovVtas = VtasPred - tasDataDelayed.tas;

        // calculate observation jacobians
        ObservationModel<Velocity> H_TAS{};
        H_TAS.at<4>((vn - vwn) / (VtasPred * metre_per_second));
        H_TAS.at<5>((ve - vwe) / (VtasPred * metre_per_second));
        H_TAS.at<6>(vd / (VtasPred * metre_per_second));
        H_TAS.at<22>(-(vn - vwn) / (VtasPred * metre_per_second));
        H_TAS.at<23>(-(ve - vwe) / (VtasPred * metre_per_second));

        // calculate the innovation variance and Kalman gains
        const CovarianceView covariance{AP_LinearAlgebra::MatrixView<ftype, 24, 24>{&P[0][0]}};
        const auto PHt = covariance * fcarouge::transposed(H_TAS);
        const auto S = (H_TAS * PHt).at() + tasDataDelayed.tasVariance * velocity_variance;
        const ftype temp = S.numerical_value_in(velocity_variance);
        if (temp >= tasDataDelayed.tasVariance) {
            faultStatus.bad_airspeed = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            faultStatus.bad_airspeed = true;
            return;
        }
        const auto K = PHt / S;

        uint32_t kalman_mask = 0; // values to calculate in Kfusion (others are set to zero)

        if (tasDataDelayed.allowFusion && !airDataFusionWindOnly) {
            kalman_mask = (1<<10)-1;
        }

        if (tasDataDelayed.allowFusion && !inhibitDelAngBiasStates && !airDataFusionWindOnly) {
            kalman_mask |= (1<<10) | (1<<11) | (1<<12);
        }

        if (tasDataDelayed.allowFusion && !inhibitDelVelBiasStates && !airDataFusionWindOnly) {
            for (uint8_t index = 0; index < 3; index++) {
                const uint8_t stateIndex = index + 13;
                if (!dvelBiasAxisInhibit[index]) {
                    kalman_mask |= (1<<stateIndex);
                }
            }
        }

        // zero Kalman gains to inhibit magnetic field state estimation
        if (tasDataDelayed.allowFusion && !inhibitMagStates && !airDataFusionWindOnly) {
            kalman_mask |= (1<<16) | (1<<17) | (1<<18) | (1<<19) | (1<<20) | (1<<21);
        }

        if (tasDataDelayed.allowFusion && !inhibitWindStates && !treatWindStatesAsTruth) {
            kalman_mask |= (1<<22) | (1<<23);
        }

        for (auto i=0; i<24; i++) {
            ftype res = 0;
            if (kalman_mask & (1<<i)) {
                res = K.data()(i);
            }
            Kfusion[i] = res;
        }

        // calculate measurement innovation variance
        varInnovVtas = temp;

        // calculate the innovation consistency test ratio
        tasTestRatio = sq(innovVtas) / (sq(MAX(0.01f * (ftype)frontend->_tasInnovGate, 1.0f)) * varInnovVtas);

        // fail if the ratio is > 1, but don't fail if bad IMU data
        const bool isConsistent = (tasTestRatio < 1.0f) || badIMUdata;
        tasTimeout = (imuSampleTime_ms - lastTasPassTime_ms) > frontend->tasRetryTime_ms;
        if (!isConsistent) {
            lastTasFailTime_ms = imuSampleTime_ms;
        } else {
            lastTasFailTime_ms = 0;
        }

        // test the ratio before fusing data, forcing fusion if airspeed and position are timed out as we have no choice but to try and use airspeed to constrain error growth
        if (tasDataDelayed.allowFusion && (isConsistent || (tasTimeout && posTimeout))) {
            // restart the counter
            lastTasPassTime_ms = imuSampleTime_ms;

            // correct the covariance P = (I - K*H)*P = P - K*H*P. K*H*P is written to its
            // array, its type checked against the covariance without being evaluated.
            const auto HP = H_TAS * covariance;
            static_assert(std::is_convertible_v<decltype(K * HP), Covariance>);
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=stateIndexLim; j++) {
                    KHP[i][j] = Kfusion[i] * HP.data()(j);
                }
            }

            // finish fusion from KHP and Kfusion
            FinishFusion(innovVtas, true); // forcing fusion is probably a bug
        }
    }
}

// select fusion of true airspeed measurements
void NavEKF3_core::SelectTasFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data locking out fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !airSpdFusionDelayed) {
        airSpdFusionDelayed = true;
        return;
    } else {
        airSpdFusionDelayed = false;
    }

    // get true airspeed measurement
    readAirSpdData();

    // if the filter is initialised, wind states are not inhibited and we have data to fuse, then perform TAS fusion

    if (tasDataToFuse && statesInitialised && !inhibitWindStates) {
        FuseAirspeed();
        tasDataToFuse = false;
        prevTasStep_ms = imuSampleTime_ms;
    }
}


// select fusion of synthetic sideslip measurements or body frame drag
// synthetic sidelip fusion only works for fixed wing aircraft and relies on the average sideslip being close to zero
// body frame drag only works for bluff body multi rotor vehices with thrust forces aligned with the Z axis
// it requires a stable wind for best results and should not be used for aerobatic flight
void NavEKF3_core::SelectBetaDragFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !sideSlipFusionDelayed) {
        sideSlipFusionDelayed = true;
        return;
    } else {
        sideSlipFusionDelayed = false;
    }

    // set true when the fusion time interval has triggered
    bool f_timeTrigger = ((imuSampleTime_ms - prevBetaDragStep_ms) >= frontend->betaAvg_ms);

    // use of air data to constrain drift is necessary if we have limited sensor data or are doing inertial dead reckoning
    bool is_dead_reckoning = ((imuSampleTime_ms - lastGpsPosPassTime_ms) > frontend->deadReckonDeclare_ms) &&
                             ((imuSampleTime_ms - lastVelPassTime_ms) > frontend->deadReckonDeclare_ms);
    const bool noYawSensor = !use_compass() && !using_noncompass_for_yaw();
    const bool f_required = (noYawSensor && (frontend->_betaMask & (1<<1))) || is_dead_reckoning;

    // set true when sideslip fusion is feasible (requires zero sideslip assumption to be valid and use of wind states)
    const bool f_beta_feasible = (assume_zero_sideslip() && !inhibitWindStates);

    // use synthetic sideslip fusion if feasible, required and enough time has lapsed since the last fusion
    if (f_beta_feasible && f_timeTrigger) {
        // unless air data is required to constrain drift, it is only used to update wind state estimates
        if (f_required || (frontend->_betaMask & (1<<0))) {
            // we are required to correct all states
            airDataFusionWindOnly = false;
        } else {
            // we are required to correct only wind states
            airDataFusionWindOnly = true;
        }
        FuseSideslip();
        prevBetaDragStep_ms = imuSampleTime_ms;
    }

#if EK3_FEATURE_DRAG_FUSION
    // fusion of XY body frame aero specific forces is done at a slower rate and only if alternative methods of wind estimation are not available
    if (!inhibitWindStates && storedDrag.recall(dragSampleDelayed,imuDataDelayed.time_ms)) {
        FuseDragForces();
    }
    dragTimeout = (imuSampleTime_ms - lastDragPassTime_ms) > frontend->dragFailTimeLimit_ms;
#endif
}

/*
 * Fuse sythetic sideslip measurement of zero using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/GenerateNavFilterEquations.m
*/
void NavEKF3_core::FuseSideslip()
{
    using namespace NavEKF3_typed;
    // the sideslip is observed as the ratio of the lateral to the forward wind relative velocity
    const Unitless R_BETA = 0.03f * mp_units::one; // assume a sideslip angle RMS of ~10 deg
    Vector3F vel_rel_wind;

    // copy required states to local variable names
    const StateView states{AP_LinearAlgebra::MatrixView<ftype, 24, 1>{&statesArray[0]}};
    const Unitless q0 = states.at<0>();
    const Unitless q1 = states.at<1>();
    const Unitless q2 = states.at<2>();
    const Unitless q3 = states.at<3>();
    const Velocity vn = states.at<4>();
    const Velocity ve = states.at<5>();
    const Velocity vd = states.at<6>();
    const Velocity vwn = states.at<22>();
    const Velocity vwe = states.at<23>();

    // calculate predicted wind relative velocity in NED
    vel_rel_wind.x = (vn - vwn).numerical_value_in(metre_per_second);
    vel_rel_wind.y = (ve - vwe).numerical_value_in(metre_per_second);
    vel_rel_wind.z = vd.numerical_value_in(metre_per_second);

    // rotate into body axes
    vel_rel_wind = prevTnb * vel_rel_wind;

    // perform fusion of assumed sideslip  = 0
    if (vel_rel_wind.x > 5.0f)
    {
        // Calculate observation jacobians
        const Velocity SH_BETA_0 = (vn - vwn)*(sq(q0) + sq(q1) - sq(q2) - sq(q3)) - vd*(2*q0*q2 - 2*q1*q3) + (ve - vwe)*(2*q0*q3 + 2*q1*q2);
        if (fabsF(SH_BETA_0.numerical_value_in(metre_per_second)) <= 1e-9f) {
            faultStatus.bad_sideslip = true;
            return;
        } else {
            faultStatus.bad_sideslip = false;
        }
        const Velocity SH_BETA_1 = (ve - vwe)*(sq(q0) - sq(q1) + sq(q2) - sq(q3)) + vd*(2*q0*q1 + 2*q2*q3) - (vn - vwn)*(2*q0*q3 - 2*q1*q2);
        const Velocity SH_BETA_2 = vn - vwn;
        const Velocity SH_BETA_3 = ve - vwe;
        const auto SH_BETA_4 = 1/(SH_BETA_0*SH_BETA_0);
        const auto SH_BETA_5 = 1/SH_BETA_0;
        const auto SH_BETA_6 = SH_BETA_5*(sq(q0) - sq(q1) + sq(q2) - sq(q3));
        const Unitless SH_BETA_7 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        const Velocity SH_BETA_8 = 2*q0*SH_BETA_3 - 2*q3*SH_BETA_2 + 2*q1*vd;
        const Velocity SH_BETA_9 = 2*q0*SH_BETA_2 + 2*q3*SH_BETA_3 - 2*q2*vd;
        const Velocity SH_BETA_10 = 2*q2*SH_BETA_2 - 2*q1*SH_BETA_3 + 2*q0*vd;
        const Velocity SH_BETA_11 = 2*q1*SH_BETA_2 + 2*q2*SH_BETA_3 + 2*q3*vd;
        const Unitless SH_BETA_12 = 2*q0*q3;

        ObservationModel<Unitless> H_BETA{};
        H_BETA.at<0>(SH_BETA_5*SH_BETA_8 - SH_BETA_1*SH_BETA_4*SH_BETA_9);
        H_BETA.at<1>(SH_BETA_5*SH_BETA_10 - SH_BETA_1*SH_BETA_4*SH_BETA_11);
        H_BETA.at<2>(SH_BETA_5*SH_BETA_11 + SH_BETA_1*SH_BETA_4*SH_BETA_10);
        H_BETA.at<3>(- SH_BETA_5*SH_BETA_9 - SH_BETA_1*SH_BETA_4*SH_BETA_8);
        H_BETA.at<4>(- SH_BETA_5*(SH_BETA_12 - 2*q1*q2) - SH_BETA_1*SH_BETA_4*SH_BETA_7);
        H_BETA.at<5>(SH_BETA_6 - SH_BETA_1*SH_BETA_4*(SH_BETA_12 + 2*q1*q2));
        H_BETA.at<6>(SH_BETA_5*(2*q0*q1 + 2*q2*q3) + SH_BETA_1*SH_BETA_4*(2*q0*q2 - 2*q1*q3));
        H_BETA.at<22>(SH_BETA_5*(SH_BETA_12 - 2*q1*q2) + SH_BETA_1*SH_BETA_4*SH_BETA_7);
        H_BETA.at<23>(SH_BETA_1*SH_BETA_4*(SH_BETA_12 + 2*q1*q2) - SH_BETA_6);

        // Calculate Kalman gains
        const CovarianceView covariance{AP_LinearAlgebra::MatrixView<ftype, 24, 24>{&P[0][0]}};
        const auto PHt = covariance * fcarouge::transposed(H_BETA);
        const Unitless S = (H_BETA * PHt).at() + R_BETA;
        if (S >= R_BETA) {
            faultStatus.bad_sideslip = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            faultStatus.bad_sideslip = true;
            return;
        }
        const auto K = PHt / S;

        uint32_t kalman_mask = 0; // values to calculate in Kfusion (others are set to zero)

        if (!airDataFusionWindOnly) {
            kalman_mask = (1<<10)-1;
        }

        if (!inhibitDelAngBiasStates && !airDataFusionWindOnly) {
            kalman_mask |= (1<<10) | (1<<11) | (1<<12);
        }

        if (!inhibitDelVelBiasStates && !airDataFusionWindOnly) {
            for (uint8_t index = 0; index < 3; index++) {
                const uint8_t stateIndex = index + 13;
                if (!dvelBiasAxisInhibit[index]) {
                    kalman_mask |= (1<<stateIndex);
                }
            }
        }

        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates && !airDataFusionWindOnly) {
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

        // calculate predicted sideslip angle and innovation using small angle approximation
        innovBeta = constrain_ftype(vel_rel_wind.y / vel_rel_wind.x, -0.5f, 0.5f);

        // correct the covariance P = (I - K*H)*P = P - K*H*P. K*H*P is written to its
        // array, its type checked against the covariance without being evaluated.
        const auto HP = H_BETA * covariance;
        static_assert(std::is_convertible_v<decltype(K * HP), Covariance>);
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                KHP[i][j] = Kfusion[i] * HP.data()(j);
            }
        }

        // finish fusion from KHP and Kfusion
        FinishFusion(innovBeta, true); // forcing fusion is probably a bug
    }
}

#if EK3_FEATURE_DRAG_FUSION
/*
 * Fuse X and Y body axis specific forces using explicit algebraic equations generated with SymPy.
 * See derivation/generate_2.py for derivation
 * Output for change reference: derivation/generated/acc_bf_generated.cpp
*/
void NavEKF3_core::FuseDragForces()
{
    using namespace NavEKF3_typed;
    using mp_units::si::kilogram;
    using mp_units::si::metre;
    using mp_units::si::second;
    constexpr auto acceleration = metre / second / second;
    using Acceleration = Quantity<acceleration>;
    using Rate = Quantity<mp_units::one / second>;

    // drag model parameters
    const Quantity<kilogram / (metre * metre)> bcoef_x = frontend->_ballisticCoef_x.get() * (kilogram / (metre * metre));
    const Quantity<kilogram / (metre * metre)> bcoef_y = frontend->_ballisticCoef_y.get() * (kilogram / (metre * metre));
    const Rate mcoef = frontend->_momentumDragCoef.get() * (mp_units::one / second);
    const bool using_bcoef_x = bcoef_x.numerical_value_in(kilogram / (metre * metre)) > 1.0f;
    const bool using_bcoef_y = bcoef_y.numerical_value_in(kilogram / (metre * metre)) > 1.0f;
    const bool using_mcoef = mcoef.numerical_value_in(mp_units::one / second) > 0.001f;

    ZERO_FARRAY(Kfusion);
    const Quantity<acceleration * acceleration> R_ACC = sq(fmaxF(frontend->_dragObsNoise, 0.5f)) * (acceleration * acceleration);
    const ftype density_ratio = 1.0f/sq(dal.get_EAS2TAS());
    const Quantity<kilogram / (metre * metre * metre)> rho = fmaxF(1.225f * density_ratio, 0.1f) * (kilogram / (metre * metre * metre)); // air density

    // the states, of which the first axis corrects the wind used by the second
    const StateView states{AP_LinearAlgebra::MatrixView<ftype, 24, 1>{&statesArray[0]}};
    const CovarianceView covariance{AP_LinearAlgebra::MatrixView<ftype, 24, 24>{&P[0][0]}};

    // predicted specific forces
    // calculate relative wind velocity in earth frame and rotate into body frame
    const Vector3F rel_wind_earth((states.at<4>() - states.at<22>()).numerical_value_in(metre_per_second),
                                  (states.at<5>() - states.at<23>()).numerical_value_in(metre_per_second),
                                  states.at<6>().numerical_value_in(metre_per_second));
    const Vector3F rel_wind_body = prevTnb * rel_wind_earth;

    // perform sequential fusion of XY specific forces
    for (uint8_t axis_index = 0; axis_index < 2; axis_index++) {
        // get latest estimated orientation, velocity and wind velocity in earth frame
        const Unitless q0 = states.at<0>();
        const Unitless q1 = states.at<1>();
        const Unitless q2 = states.at<2>();
        const Unitless q3 = states.at<3>();
        const Velocity vn = states.at<4>();
        const Velocity ve = states.at<5>();
        const Velocity vd = states.at<6>();
        const Velocity vwn = states.at<22>();
        const Velocity vwe = states.at<23>();

        // correct accel data for bias
        const Acceleration mea_acc = (dragSampleDelayed.accelXY[axis_index]  - stateStruct.accel_bias[axis_index] / dtEkfAvg) * acceleration;
        const Velocity rel_wind = rel_wind_body[axis_index] * metre_per_second;
        const Quantity<kilogram / (metre * metre)> bcoef = axis_index == 0 ? bcoef_x : bcoef_y;
        const bool using_bcoef = axis_index == 0 ? using_bcoef_x : using_bcoef_y;

        // Acceleration in m/s/s predicted using vehicle and wind velocity estimates
        // Initialised to measured value and updated later using available drag model
        Acceleration predAccel = mea_acc;

        // predicted sign of drag force
        const ftype dragForceSign = is_positive(rel_wind_body[axis_index]) ? -1.0f : 1.0f;

        // drag can be modelled as an arbitrary  combination of bluff body drag that proportional to
        // speed squared, and rotor momentum drag that is proportional to speed.
        Rate Kacc; // Derivative of specific force wrt airspeed
        const Rate min_Kacc = 1e-1f * (mp_units::one / second);
        if (using_mcoef && using_bcoef) {
            // mixed bluff body and propeller momentum drag
            const Velocity airSpd = (bcoef / rho) * (- mcoef + sqrtF((mcoef * mcoef + 2.0f * (rho / bcoef) * (fabsF(mea_acc.numerical_value_in(acceleration)) * acceleration)).numerical_value_in(mp_units::one / (second * second))) * (mp_units::one / second));
            Kacc = max(min_Kacc, Rate{(rho / bcoef) * airSpd + mcoef * density_ratio});
            predAccel = (0.5f / bcoef) * rho * rel_wind * rel_wind * dragForceSign - rel_wind * mcoef * density_ratio;
        } else if (using_mcoef) {
            // propeller momentum drag only
            Kacc = max(min_Kacc, Rate{mcoef * density_ratio});
            predAccel = - rel_wind * mcoef * density_ratio;
        } else if (using_bcoef) {
            // bluff body drag only
            const Velocity airSpd = sqrtF(((2.0f * bcoef * (fabsF(mea_acc.numerical_value_in(acceleration)) * acceleration)) / rho).numerical_value_in(metre_per_second * metre_per_second)) * metre_per_second;
            Kacc = max(min_Kacc, Rate{(rho / bcoef) * airSpd});
            predAccel = (0.5f / bcoef) * rho * rel_wind * rel_wind * dragForceSign;
        } else if (axis_index == 0) {
            // skip this axis
            continue;
        } else {
            // nothing more to do
            return;
        }

        // Observation Jacobians
        ObservationModel<Acceleration> Hfusion{};
        if (axis_index == 0) {
            const Velocity HK0 = vn - vwn;
            const Velocity HK1 = ve - vwe;
            const Velocity HK2 = HK0*q0 + HK1*q3 - q2*vd;
            const Velocity HK4 = HK0*q1 + HK1*q2 + q3*vd;
            const Velocity HK5 = HK0*q2 - HK1*q1 + q0*vd;
            const Velocity HK6 = -HK0*q3 + HK1*q0 + q1*vd;
            const Unitless HK7 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
            const Unitless HK9 = q0*q3 + q1*q2;
            const Unitless HK11 = q0*q2 - q1*q3;
            Hfusion.at<0>(-2*Kacc*HK2);
            Hfusion.at<1>(-2*Kacc*HK4);
            Hfusion.at<2>(2*Kacc*HK5);
            Hfusion.at<3>(-2*Kacc*HK6);
            Hfusion.at<4>(-Kacc*HK7);
            Hfusion.at<5>(-2*Kacc*HK9);
            Hfusion.at<6>(2*Kacc*HK11);
            Hfusion.at<22>(Kacc*HK7);
            Hfusion.at<23>(2*Kacc*HK9);
        } else {
            const Velocity HK0 = ve - vwe;
            const Velocity HK1 = vn - vwn;
            const Velocity HK2 = HK0*q0 - HK1*q3 + q1*vd;
            const Velocity HK4 = -HK0*q1 + HK1*q2 + q0*vd;
            const Velocity HK5 = HK0*q2 + HK1*q1 + q3*vd;
            const Velocity HK6 = HK0*q3 + HK1*q0 - q2*vd;
            const Unitless HK7 = q0*q3 - q1*q2;
            const Unitless HK9 = q0*q0 - q1*q1 + q2*q2 - q3*q3;
            const Unitless HK11 = q0*q1 + q2*q3;
            Hfusion.at<0>(-2*Kacc*HK2);
            Hfusion.at<1>(-2*Kacc*HK4);
            Hfusion.at<2>(-2*Kacc*HK5);
            Hfusion.at<3>(2*Kacc*HK6);
            Hfusion.at<4>(2*Kacc*HK7);
            Hfusion.at<5>(-Kacc*HK9);
            Hfusion.at<6>(-2*Kacc*HK11);
            Hfusion.at<22>(-2*Kacc*HK7);
            Hfusion.at<23>(Kacc*HK9);
        }

        // calculate innovation variance and exit if badly conditioned
        const auto PHt = covariance * fcarouge::transposed(Hfusion);
        const auto S = (Hfusion * PHt).at() + R_ACC;
        innovDragVar[axis_index] = S.numerical_value_in(acceleration * acceleration);
        if (S < R_ACC) {
            return;
        }

        // Kalman gains
        // Don't allow modification of any states other than wind velocity - we only need a wind estimate.
        // See derivation/generated/acc_bf_generated.cpp for un-implemented Kalman gain equations.
        const auto K = PHt / S;
        Kfusion[22] = K.data()(22);
        Kfusion[23] = K.data()(23);

        innovDrag[axis_index] = (predAccel - mea_acc).numerical_value_in(acceleration);
        dragTestRatio[axis_index] = sq(innovDrag[axis_index]) / (25.0f * innovDragVar[axis_index]);

        // if the innovation consistency check fails then don't fuse the sample
        if (dragTestRatio[axis_index] > 1.0f) {
            return;
        }

        // correct the covariance P = (I - K*H)*P = P - K*H*P. K*H*P is written to its
        // array, its type checked against the covariance without being evaluated.
        const auto HP = Hfusion * covariance;
        static_assert(std::is_convertible_v<decltype(K * HP), Covariance>);
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                KHP[i][j] = Kfusion[i] * HP.data()(j);
            }
        }

        // finish fusion from KHP and Kfusion
        FinishFusion(innovDrag[axis_index], true); // forcing fusion is probably a bug
    }

    // record time of successful fusion
    lastDragPassTime_ms = imuSampleTime_ms;
}
#endif // EK3_FEATURE_DRAG_FUSION

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

