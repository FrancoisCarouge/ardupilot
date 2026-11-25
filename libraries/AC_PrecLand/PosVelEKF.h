#pragma once

/*
* This class implements a simple 1-D Kalman Filter to estimate the relative body frame position of the landing target and its relative velocity
* position and velocity of the target is predicted using delta velocity
* The predictions are corrected periodically using the landing target sensor(or camera)
*
* A typed Kalman filter: the position in metres, the velocity in metres per
* second, every covariance element in its own unit.
*/

#include <AP_Math/typed_linear_algebra.h>

class PosVelEKF {
public:
    using Time = typed_linear_algebra::Quantity<mp_units::si::second>;
    using Length = typed_linear_algebra::Length;
    using Speed = typed_linear_algebra::Speed;

    // The target: its position and velocity relative to the vehicle.
    using State = typed_linear_algebra::Vector<Length, Speed>;
    using Covariance = typed_linear_algebra::Matrix<State, State>;
    // The vehicle's change of velocity, which the target's relative velocity follows.
    using Input = typed_linear_algebra::Vector<Speed>;
    // The sensor's output: the target's position.
    using Output = typed_linear_algebra::Vector<Length>;
    using OutputVariance = typed_linear_algebra::Matrix<Output, Output>;

    // The variances of the position [m²] and of the velocity [(m/s)²].
    using PositionVariance = typed_linear_algebra::Element<Covariance, 0, 0>;
    using VelocityVariance = typed_linear_algebra::Element<Covariance, 1, 1>;

    using Transition = fcarouge::kalman_filter::internal::evaluate<
        fcarouge::kalman_filter::internal::quotient<State, State>>;
    using InputControl = fcarouge::kalman_filter::internal::evaluate<
        fcarouge::kalman_filter::internal::quotient<State, Input>>;

    // The target moves at its velocity for dt: F = |1 dt|
    //                                              |0  1|
    struct motion {
        Transition operator()(const Input &dvel, const Time &dt, const Speed &dvel_noise) const;
    };

    // The change of velocity adds to the velocity: B = |0 1|ᵀ.
    struct velocity_change {
        InputControl operator()(const Time &dt, const Speed &dvel_noise) const;
    };

    // The change of velocity is uncertain: Q = |0       0      |
    //                                          |0  dVelNoise²  |
    struct process_noise {
        Covariance operator()(const State &x, const Time &dt, const Speed &dvel_noise) const;
    };

    // The sensor measures the position: H = |1 0|, the filter's default
    // output model, the first row of the identity.
    using Filter = decltype(fcarouge::kalman{
        fcarouge::state{State{}}, fcarouge::output<Output>, fcarouge::input<Input>,
        fcarouge::estimate_uncertainty{Covariance{}}, fcarouge::process_uncertainty{process_noise{}},
        fcarouge::output_uncertainty{OutputVariance{}},
        fcarouge::state_transition{motion{}}, fcarouge::input_control{velocity_change{}},
        fcarouge::prediction_types<Time, Speed>, fcarouge::simple_form});

    // Initialize the covariance and state matrix
    // This is called when the landing target is located for the first time or it was lost, then relocated
    void init(Length pos, PositionVariance posVar, Speed vel, VelocityVariance velVar);

    // This functions runs the Prediction Step of the EKF
    // This is called at 400 hz
    void predict(Time dt, Speed dVel, Speed dVelNoise);

    // fuse the new sensor measurement into the EKF calculations
    // This is called whenever we have a new measurement available
    void fusePos(Length pos, PositionVariance posVar);

    // Get the EKF state position
    Length getPos() const { return _filter.x().at<0>(); }

    // Get the EKF state velocity
    Speed getVel() const { return _filter.x().at<1>(); }

    // get the normalized innovation squared
    float getPosNIS(Length pos, PositionVariance posVar) const;

private:
    // The covariance is symmetric: its upper element is kept, as the original
    // filter stored the upper triangle only.
    void keep_upper_triangle();

    Filter _filter{
        fcarouge::state{State{}}, fcarouge::output<Output>, fcarouge::input<Input>,
        fcarouge::estimate_uncertainty{Covariance{}}, fcarouge::process_uncertainty{process_noise{}},
        fcarouge::output_uncertainty{OutputVariance{}},
        fcarouge::state_transition{motion{}}, fcarouge::input_control{velocity_change{}},
        fcarouge::prediction_types<Time, Speed>, fcarouge::simple_form};
};
