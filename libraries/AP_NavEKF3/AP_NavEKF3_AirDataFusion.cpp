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
    // drag model parameters
    const ftype bcoef_x = frontend->_ballisticCoef_x.get();
    const ftype bcoef_y = frontend->_ballisticCoef_y.get();
    const ftype mcoef = frontend->_momentumDragCoef.get();
    const bool using_bcoef_x = bcoef_x > 1.0f;
    const bool using_bcoef_y = bcoef_y > 1.0f;
    const bool using_mcoef = mcoef > 0.001f;

    ZERO_FARRAY(Kfusion);
    Vector24 Hfusion; // Observation Jacobians
    const ftype R_ACC = sq(fmaxF(frontend->_dragObsNoise, 0.5f));
    const ftype density_ratio = 1.0f/sq(dal.get_EAS2TAS());
    const ftype rho = fmaxF(1.225f * density_ratio, 0.1f); // air density

    // get latest estimated orientation
    const ftype &q0 = stateStruct.quat[0];
    const ftype &q1 = stateStruct.quat[1];
    const ftype &q2 = stateStruct.quat[2];
    const ftype &q3 = stateStruct.quat[3];

    // get latest velocity in earth frame
    const ftype &vn = stateStruct.velocity.x;
    const ftype &ve = stateStruct.velocity.y;
    const ftype &vd = stateStruct.velocity.z;

    // get latest wind velocity in earth frame
    const ftype &vwn = stateStruct.wind_vel.x;
    const ftype &vwe = stateStruct.wind_vel.y;

    // predicted specific forces
    // calculate relative wind velocity in earth frame and rotate into body frame
    const Vector3F rel_wind_earth(vn - vwn, ve - vwe, vd);
    const Vector3F rel_wind_body = prevTnb * rel_wind_earth;

    // perform sequential fusion of XY specific forces
    for (uint8_t axis_index = 0; axis_index < 2; axis_index++) {
        // correct accel data for bias
        const ftype mea_acc = dragSampleDelayed.accelXY[axis_index]  - stateStruct.accel_bias[axis_index] / dtEkfAvg;

        // Acceleration in m/s/s predicted using vehicle and wind velocity estimates
        // Initialised to measured value and updated later using available drag model
        ftype predAccel = mea_acc;

        // predicted sign of drag force
        const ftype dragForceSign = is_positive(rel_wind_body[axis_index]) ? -1.0f : 1.0f;

        if (axis_index == 0) {
            // drag can be modelled as an arbitrary  combination of bluff body drag that proportional to
            // speed squared, and rotor momentum drag that is proportional to speed.
            ftype Kaccx; // Derivative of specific force wrt airspeed
            if (using_mcoef && using_bcoef_x) {
                // mixed bluff body and propeller momentum drag
                const ftype airSpd = (bcoef_x / rho) * (- mcoef + sqrtF(sq(mcoef) + 2.0f * (rho / bcoef_x) * fabsF(mea_acc)));
                Kaccx = fmaxF(1e-1f, (rho / bcoef_x) * airSpd + mcoef * density_ratio);
                predAccel = (0.5f / bcoef_x) * rho * sq(rel_wind_body[0]) * dragForceSign - rel_wind_body[0] * mcoef * density_ratio;
            } else if (using_mcoef) {
                // propeller momentum drag only
                Kaccx = fmaxF(1e-1f, mcoef * density_ratio);
                predAccel = - rel_wind_body[0] * mcoef * density_ratio;
            } else if (using_bcoef_x) {
                // bluff body drag only
                const ftype airSpd = sqrtF((2.0f * bcoef_x * fabsF(mea_acc)) / rho);
                Kaccx = fmaxF(1e-1f, (rho / bcoef_x) * airSpd);
                predAccel = (0.5f / bcoef_x) * rho * sq(rel_wind_body[0]) * dragForceSign;
            } else {
                // skip this axis
                continue;
            }

            // intermediate variables
            const ftype HK0 = vn - vwn;
            const ftype HK1 = ve - vwe;
            const ftype HK2 = HK0*q0 + HK1*q3 - q2*vd;
            const ftype HK3 = 2*Kaccx;
            const ftype HK4 = HK0*q1 + HK1*q2 + q3*vd;
            const ftype HK5 = HK0*q2 - HK1*q1 + q0*vd;
            const ftype HK6 = -HK0*q3 + HK1*q0 + q1*vd;
            const ftype HK7 = sq(q0) + sq(q1) - sq(q2) - sq(q3);
            const ftype HK8 = HK7*Kaccx;
            const ftype HK9 = q0*q3 + q1*q2;
            const ftype HK10 = HK3*HK9;
            const ftype HK11 = q0*q2 - q1*q3;
            const ftype HK12 = 2*HK9;
            const ftype HK13 = 2*HK11;
            const ftype HK14 = 2*HK4;
            const ftype HK15 = 2*HK2;
            const ftype HK16 = 2*HK5;
            const ftype HK17 = 2*HK6;
            const ftype HK18 = -HK12*P[0][23] + HK12*P[0][5] - HK13*P[0][6] + HK14*P[0][1] + HK15*P[0][0] - HK16*P[0][2] + HK17*P[0][3] - HK7*P[0][22] + HK7*P[0][4];
            const ftype HK19 = HK12*P[5][23];
            const ftype HK20 = -HK12*P[23][23] - HK13*P[6][23] + HK14*P[1][23] + HK15*P[0][23] - HK16*P[2][23] + HK17*P[3][23] + HK19 - HK7*P[22][23] + HK7*P[4][23];
            const ftype HK21 = sq(Kaccx);
            const ftype HK22 = HK12*HK21;
            const ftype HK23 = HK12*P[5][5] - HK13*P[5][6] + HK14*P[1][5] + HK15*P[0][5] - HK16*P[2][5] + HK17*P[3][5] - HK19 + HK7*P[4][5] - HK7*P[5][22];
            const ftype HK24 = HK12*P[5][6] - HK12*P[6][23] - HK13*P[6][6] + HK14*P[1][6] + HK15*P[0][6] - HK16*P[2][6] + HK17*P[3][6] + HK7*P[4][6] - HK7*P[6][22];
            const ftype HK25 = HK7*P[4][22];
            const ftype HK26 = -HK12*P[4][23] + HK12*P[4][5] - HK13*P[4][6] + HK14*P[1][4] + HK15*P[0][4] - HK16*P[2][4] + HK17*P[3][4] - HK25 + HK7*P[4][4];
            const ftype HK27 = HK21*HK7;
            const ftype HK28 = -HK12*P[22][23] + HK12*P[5][22] - HK13*P[6][22] + HK14*P[1][22] + HK15*P[0][22] - HK16*P[2][22] + HK17*P[3][22] + HK25 - HK7*P[22][22];
            const ftype HK29 = -HK12*P[1][23] + HK12*P[1][5] - HK13*P[1][6] + HK14*P[1][1] + HK15*P[0][1] - HK16*P[1][2] + HK17*P[1][3] - HK7*P[1][22] + HK7*P[1][4];
            const ftype HK30 = -HK12*P[2][23] + HK12*P[2][5] - HK13*P[2][6] + HK14*P[1][2] + HK15*P[0][2] - HK16*P[2][2] + HK17*P[2][3] - HK7*P[2][22] + HK7*P[2][4];
            const ftype HK31 = -HK12*P[3][23] + HK12*P[3][5] - HK13*P[3][6] + HK14*P[1][3] + HK15*P[0][3] - HK16*P[2][3] + HK17*P[3][3] - HK7*P[3][22] + HK7*P[3][4];
            // const ftype HK32 = Kaccx/(-HK13*HK21*HK24 + HK14*HK21*HK29 + HK15*HK18*HK21 - HK16*HK21*HK30 + HK17*HK21*HK31 - HK20*HK22 + HK22*HK23 + HK26*HK27 - HK27*HK28 + R_ACC);

            // calculate innovation variance and exit if badly conditioned
            innovDragVar.x = (-HK13*HK21*HK24 + HK14*HK21*HK29 + HK15*HK18*HK21 - HK16*HK21*HK30 + HK17*HK21*HK31 - HK20*HK22 + HK22*HK23 + HK26*HK27 - HK27*HK28 + R_ACC);
            if (innovDragVar.x < R_ACC) {
                return;
            }
            const ftype HK32 = Kaccx / innovDragVar.x;

            // Observation Jacobians
            Hfusion[0] = -HK2*HK3;
            Hfusion[1] = -HK3*HK4;
            Hfusion[2] = HK3*HK5;
            Hfusion[3] = -HK3*HK6;
            Hfusion[4] = -HK8;
            Hfusion[5] = -HK10;
            Hfusion[6] = HK11*HK3;
            Hfusion[22] = HK8;
            Hfusion[23] = HK10;

            // Kalman gains
            // Don't allow modification of any states other than wind velocity - we only need a wind estimate.
            // See derivation/generated/acc_bf_generated.cpp for un-implemented Kalman gain equations.
            Kfusion[22] = -HK28*HK32;
            Kfusion[23] = -HK20*HK32;


        } else if (axis_index == 1) {
            // drag can be modelled as an arbitrary combination of bluff body drag that proportional to
            // speed squared, and rotor momentum drag that is proportional to speed.
            ftype Kaccy; // Derivative of specific force wrt airspeed
            if (using_mcoef && using_bcoef_y) {
                // mixed bluff body and propeller momentum drag
                const ftype airSpd = (bcoef_y / rho) * (- mcoef + sqrtF(sq(mcoef) + 2.0f * (rho / bcoef_y) * fabsF(mea_acc)));
                Kaccy = fmaxF(1e-1f, (rho / bcoef_y) * airSpd + mcoef * density_ratio);
                predAccel = (0.5f / bcoef_y) * rho * sq(rel_wind_body[1]) * dragForceSign - rel_wind_body[1] * mcoef * density_ratio;
            } else if (using_mcoef) {
                // propeller momentum drag only
                Kaccy = fmaxF(1e-1f, mcoef * density_ratio);
                predAccel = - rel_wind_body[1] * mcoef * density_ratio;
            } else if (using_bcoef_y) {
                // bluff body drag only
                const ftype airSpd = sqrtF((2.0f * bcoef_y * fabsF(mea_acc)) / rho);
                Kaccy = fmaxF(1e-1f, (rho / bcoef_y) * airSpd);
                predAccel = (0.5f / bcoef_y) * rho * sq(rel_wind_body[1]) * dragForceSign;
            } else {
                // nothing more to do
                return;
            }

            // intermediate variables
            const ftype HK0 = ve - vwe;
            const ftype HK1 = vn - vwn;
            const ftype HK2 = HK0*q0 - HK1*q3 + q1*vd;
            const ftype HK3 = 2*Kaccy;
            const ftype HK4 = -HK0*q1 + HK1*q2 + q0*vd;
            const ftype HK5 = HK0*q2 + HK1*q1 + q3*vd;
            const ftype HK6 = HK0*q3 + HK1*q0 - q2*vd;
            const ftype HK7 = q0*q3 - q1*q2;
            const ftype HK8 = HK3*HK7;
            const ftype HK9 = sq(q0) - sq(q1) + sq(q2) - sq(q3);
            const ftype HK10 = HK9*Kaccy;
            const ftype HK11 = q0*q1 + q2*q3;
            const ftype HK12 = 2*HK11;
            const ftype HK13 = 2*HK7;
            const ftype HK14 = 2*HK5;
            const ftype HK15 = 2*HK2;
            const ftype HK16 = 2*HK4;
            const ftype HK17 = 2*HK6;
            const ftype HK18 = HK12*P[0][6] + HK13*P[0][22] - HK13*P[0][4] + HK14*P[0][2] + HK15*P[0][0] + HK16*P[0][1] - HK17*P[0][3] - HK9*P[0][23] + HK9*P[0][5];
            const ftype HK19 = sq(Kaccy);
            const ftype HK20 = HK12*P[6][6] - HK13*P[4][6] + HK13*P[6][22] + HK14*P[2][6] + HK15*P[0][6] + HK16*P[1][6] - HK17*P[3][6] + HK9*P[5][6] - HK9*P[6][23];
            const ftype HK21 = HK13*P[4][22];
            const ftype HK22 = HK12*P[6][22] + HK13*P[22][22] + HK14*P[2][22] + HK15*P[0][22] + HK16*P[1][22] - HK17*P[3][22] - HK21 - HK9*P[22][23] + HK9*P[5][22];
            const ftype HK23 = HK13*HK19;
            const ftype HK24 = HK12*P[4][6] - HK13*P[4][4] + HK14*P[2][4] + HK15*P[0][4] + HK16*P[1][4] - HK17*P[3][4] + HK21 - HK9*P[4][23] + HK9*P[4][5];
            const ftype HK25 = HK9*P[5][23];
            const ftype HK26 = HK12*P[5][6] - HK13*P[4][5] + HK13*P[5][22] + HK14*P[2][5] + HK15*P[0][5] + HK16*P[1][5] - HK17*P[3][5] - HK25 + HK9*P[5][5];
            const ftype HK27 = HK19*HK9;
            const ftype HK28 = HK12*P[6][23] + HK13*P[22][23] - HK13*P[4][23] + HK14*P[2][23] + HK15*P[0][23] + HK16*P[1][23] - HK17*P[3][23] + HK25 - HK9*P[23][23];
            const ftype HK29 = HK12*P[2][6] + HK13*P[2][22] - HK13*P[2][4] + HK14*P[2][2] + HK15*P[0][2] + HK16*P[1][2] - HK17*P[2][3] - HK9*P[2][23] + HK9*P[2][5];
            const ftype HK30 = HK12*P[1][6] + HK13*P[1][22] - HK13*P[1][4] + HK14*P[1][2] + HK15*P[0][1] + HK16*P[1][1] - HK17*P[1][3] - HK9*P[1][23] + HK9*P[1][5];
            const ftype HK31 = HK12*P[3][6] + HK13*P[3][22] - HK13*P[3][4] + HK14*P[2][3] + HK15*P[0][3] + HK16*P[1][3] - HK17*P[3][3] - HK9*P[3][23] + HK9*P[3][5];
            // const ftype HK32 = Kaccy/(HK12*HK19*HK20 + HK14*HK19*HK29 + HK15*HK18*HK19 + HK16*HK19*HK30 - HK17*HK19*HK31 + HK22*HK23 - HK23*HK24 + HK26*HK27 - HK27*HK28 + R_ACC);

            innovDragVar.y = (HK12*HK19*HK20 + HK14*HK19*HK29 + HK15*HK18*HK19 + HK16*HK19*HK30 - HK17*HK19*HK31 + HK22*HK23 - HK23*HK24 + HK26*HK27 - HK27*HK28 + R_ACC);
            if (innovDragVar.y < R_ACC) {
                // calculation is badly conditioned
                return;
            }
            const ftype HK32 = Kaccy / innovDragVar.y;

            // Observation Jacobians
            Hfusion[0] = -HK2*HK3;
            Hfusion[1] = -HK3*HK4;
            Hfusion[2] = -HK3*HK5;
            Hfusion[3] = HK3*HK6;
            Hfusion[4] = HK8;
            Hfusion[5] = -HK10;
            Hfusion[6] = -HK11*HK3;
            Hfusion[22] = -HK8;
            Hfusion[23] = HK10;

            // Kalman gains
            // Don't allow modification of any states other than wind velocity at this stage of development - we only need a wind estimate.
            // See derivation/generated/acc_bf_generated.cpp for un-implemented Kalman gain equations.
            Kfusion[22] = -HK22*HK32;
            Kfusion[23] = -HK28*HK32;
        }

        innovDrag[axis_index] = predAccel - mea_acc;
        dragTestRatio[axis_index] = sq(innovDrag[axis_index]) / (25.0f * innovDragVar[axis_index]);

        // if the innovation consistency check fails then don't fuse the sample
        if (dragTestRatio[axis_index] > 1.0f) {
            return;
        }

        // correct the covariance P = (I - K*H)*P = P - K*H*P. take advantage of
        // the zero elements of H to reduce the number of operations.
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            // j as the inner loop allows the compiler to hoist the KH product
            // to save computation, and do the inner indexing more efficiently.
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                ftype res = 0;
                res += (Kfusion[i] * Hfusion[0]) * P[0][j];
                res += (Kfusion[i] * Hfusion[1]) * P[1][j];
                res += (Kfusion[i] * Hfusion[2]) * P[2][j];
                res += (Kfusion[i] * Hfusion[3]) * P[3][j];
                res += (Kfusion[i] * Hfusion[4]) * P[4][j];
                res += (Kfusion[i] * Hfusion[5]) * P[5][j];
                res += (Kfusion[i] * Hfusion[6]) * P[6][j];
                res += (Kfusion[i] * Hfusion[22]) * P[22][j];
                res += (Kfusion[i] * Hfusion[23]) * P[23][j];
                KHP[i][j] = res;
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

