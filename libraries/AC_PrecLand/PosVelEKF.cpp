#include "PosVelEKF.h"

using typed_linear_algebra::Element;

PosVelEKF::Transition PosVelEKF::motion::operator()(const Input &, const Time &dt, const Speed &) const
{
    Transition f{};
    f.at<0, 0>(1.0f * Element<Transition, 0, 0>::reference);
    f.at<0, 1>(dt);
    f.at<1, 1>(1.0f * Element<Transition, 1, 1>::reference);
    return f;
}

PosVelEKF::InputControl PosVelEKF::velocity_change::operator()(const Time &, const Speed &) const
{
    InputControl b{};
    b.at<1>(1.0f * Element<InputControl, 1, 0>::reference);
    return b;
}

PosVelEKF::Covariance PosVelEKF::process_noise::operator()(const State &, const Time &, const Speed &dvel_noise) const
{
    Covariance q{};
    q.at<1, 1>(dvel_noise * dvel_noise);
    return q;
}

// Initialize the covariance and state matrix
// This is called when the landing target is located for the first time or it was lost, then relocated
void PosVelEKF::init(Length pos, PositionVariance posVar, Speed vel, VelocityVariance velVar)
{
    _filter.x(State{pos, vel});
    Covariance p{};
    p.at<0, 0>(posVar);
    p.at<1, 1>(velVar);
    _filter.p(p);
}

// This functions runs the Prediction Step of the EKF
// This is called at 400 hz
void PosVelEKF::predict(Time dt, Speed dVel, Speed dVelNoise)
{
    // X = F X + B dVel, P = F P Fᵀ + Q
    _filter.predict(dt, dVelNoise, Input{dVel});
    keep_upper_triangle();
}

// fuse the new sensor measurement into the EKF calculations
// This is called whenever we have a new measurement available
void PosVelEKF::fusePos(Length pos, PositionVariance posVar)
{
    // the sensor measures the position directly, of variance R = posVar
    _filter.r(posVar);
    _filter.update(pos);
    keep_upper_triangle();
}

// Returns normalized innovation squared
float PosVelEKF::getPosNIS(Length pos, PositionVariance posVar) const
{
    // NIS = innovation_residual.Transpose * Innovation_Covariance.Inverse * innovation_residual
    const Length innovation_residual = pos - getPos();
    const PositionVariance innovation_covariance = _filter.p().at<0, 0>() + posVar;
    return (innovation_residual * innovation_residual / innovation_covariance).numerical_value_in(mp_units::one);
}

void PosVelEKF::keep_upper_triangle()
{
    Covariance p{_filter.p()};
    p.at<1, 0>(p.at<0, 1>());
    _filter.p(p);
}
