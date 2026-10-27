#include "PosVelEKF.h"

#include <AP_LinearAlgebra/AP_LinearAlgebra_Kalman.h>

#include <new>

namespace {

using AP_LinearAlgebra::Units::Metres;
using AP_LinearAlgebra::Units::MetresPerSecond;
using AP_LinearAlgebra::Units::Seconds;

constexpr auto metre = mp_units::si::metre;
constexpr auto second = mp_units::si::second;
constexpr auto metre_per_second = metre / second;

// state: position and velocity of the target relative to the vehicle
using State = AP_LinearAlgebra::ColumnVector<float, Metres, MetresPerSecond>;
// output: the measured relative position
using Output = AP_LinearAlgebra::ColumnVector<float, Metres>;
// input: the change of the relative velocity over the prediction
using Input = AP_LinearAlgebra::ColumnVector<float, MetresPerSecond>;
using Covariance = AP_LinearAlgebra::OuterProduct<State, State>;
using OutputVariance = AP_LinearAlgebra::OuterProduct<Output, Output>;
using StateTransition = AP_LinearAlgebra::Quotient<State, State>;
using InputControl = AP_LinearAlgebra::Quotient<State, Input>;

/*
  newState = F * oldState + G * dVel, with
  F = |1 dt|   G = |0|   Q = |0       0      |
      |0  1|       |1|       |0   dVelNoise^2|
  and a position measurement, H = [1 0]
 */
auto make_filter()
{
    using namespace fcarouge;
    return kalman{
        state{State{0.0f * metre, 0.0f * metre_per_second}},
        output<Output>,
        input<Input>,
        // set by init()
        estimate_uncertainty{Covariance{}},
        process_uncertainty{[](const State &, const Seconds &, const MetresPerSecond &dVelNoise) -> Covariance {
            Covariance q{};
            q.at<1, 1>(dVelNoise * dVelNoise);
            return q;
        }},
        // set by fusePos()
        output_uncertainty{OutputVariance{0.0f * (metre * metre)}},
        state_transition{[](const Input &, const Seconds &dt, const MetresPerSecond &) -> StateTransition {
            StateTransition f{};
            f.at<0, 0>(1.0f * mp_units::one);
            f.at<0, 1>(dt);
            f.at<1, 1>(1.0f * mp_units::one);
            return f;
        }},
        input_control{[](const Seconds &, const MetresPerSecond &) -> InputControl {
            return InputControl{0.0f * second, 1.0f * mp_units::one};
        }},
        prediction_types<Seconds, MetresPerSecond>,
        // the covariance corrected as (I - K H) P, as the original
        simple_form};
}

} // namespace

struct PosVelEKF::Filter {
    decltype(make_filter()) kalman{make_filter()};
};

PosVelEKF::PosVelEKF()
{
    static_assert(sizeof(Filter) <= filter_size, "PosVelEKF::filter_size is too small for the filter");
    static_assert(alignof(Filter) <= 8, "PosVelEKF::_storage is under-aligned for the filter");
    _filter = new (_storage) Filter;
}

PosVelEKF::~PosVelEKF()
{
    _filter->~Filter();
}

// Initialize the covariance and state matrix
// This is called when the landing target is located for the first time or it was lost, then relocated
void PosVelEKF::init(Metres pos, SquareMetres posVar, MetresPerSecond vel, SquareMetresPerSecondSquared velVar)
{
    auto &kalman = _filter->kalman;
    kalman.x(State{pos, vel});
    Covariance p{};
    p.at<0, 0>(posVar);
    p.at<1, 1>(velVar);
    kalman.p(p);
}

// This functions runs the Prediction Step of the EKF
// This is called at 400 hz
void PosVelEKF::predict(Seconds dt, MetresPerSecond dVel, MetresPerSecond dVelNoise)
{
    _filter->kalman.predict(dt, dVelNoise, Input{dVel});
}

// fuse the new sensor measurement into the EKF calculations
// This is called whenever we have a new measurement available
void PosVelEKF::fusePos(Metres pos, SquareMetres posVar)
{
    auto &kalman = _filter->kalman;
    kalman.r(OutputVariance{posVar});
    kalman.update(Output{pos});
}

PosVelEKF::Metres PosVelEKF::getPos() const
{
    return _filter->kalman.x().at<0>();
}

PosVelEKF::MetresPerSecond PosVelEKF::getVel() const
{
    return _filter->kalman.x().at<1>();
}

// Returns normalized innovation squared
PosVelEKF::Unitless PosVelEKF::getPosNIS(Metres pos, SquareMetres posVar)
{
    // NIS = innovation_residual.Transpose * Innovation_Covariance.Inverse * innovation_residual
    const auto &kalman = _filter->kalman;
    const Metres innovation_residual = pos - kalman.x().at<0>();
    const SquareMetres innovation_covariance = kalman.p().at<0, 0>() + posVar;
    return innovation_residual * innovation_residual / innovation_covariance;
}
