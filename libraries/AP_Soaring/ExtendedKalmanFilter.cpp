#include "ExtendedKalmanFilter.h"

#include <AP_LinearAlgebra/AP_LinearAlgebra_Kalman.h>

#include <math.h>

namespace {

using AP_LinearAlgebra::Units::Metres;
using AP_LinearAlgebra::Units::MetresPerSecond;
using namespace mp_units::si::unit_symbols;

// state: thermal strength, radius, north and east position
using State = AP_LinearAlgebra::ColumnVector<float, MetresPerSecond, Metres, Metres, Metres>;
// output: vertical air velocity at the aircraft
using Output = AP_LinearAlgebra::ColumnVector<float, MetresPerSecond>;
using Covariance = AP_LinearAlgebra::OuterProduct<State, State>;
using OutputVariance = AP_LinearAlgebra::OuterProduct<Output, Output>;
using OutputModel = AP_LinearAlgebra::Quotient<Output, State>;

float value(const auto &quantity)
{
    return quantity.numerical_value_in(quantity.unit);
}

// the Gaussian thermal model: strength * exp(-d^2 / radius^2) at a distance d
// from the thermal centre
float gaussian(const State &x, const Metres &px, const Metres &py)
{
    const Metres dx{x.at<2>() - px};
    const Metres dy{x.at<3>() - py};
    const Metres r{x.at<1>()};
    return expf(-value((dx * dx + dy * dy) / (r * r)));
}

auto make_filter()
{
    using namespace fcarouge;
    return kalman{
        state{State{0.0f * (m / s), 0.0f * m, 0.0f * m, 0.0f * m}},
        output<Output>,
        // set by reset(); the filter configuration requires initial values
        estimate_uncertainty{Covariance{}},
        process_uncertainty{Covariance{}},
        output_uncertainty{OutputVariance{0.0f * m2 / s2}},
        // H, the Jacobian of the output with respect to the state, from the
        // analytical derivation of the Gaussian updraft distribution
        output_model{[](const State &x, const Metres &px, const Metres &py) -> OutputModel {
            const Metres dx{x.at<2>() - px};
            const Metres dy{x.at<3>() - py};
            const Metres r{x.at<1>()};
            const float expon{gaussian(x, px, py)};
            OutputModel h{};
            h.at<0>(expon * mp_units::one);
            h.at<1>(2.0f * x.at<0>() * ((dx * dx + dy * dy) / (r * r * r)) * expon);
            h.at<2>(-2.0f * (x.at<0>() * dx / (r * r)) * expon);
            h.at<3>(-2.0f * (x.at<0>() * dy / (r * r)) * expon);
            return h;
        }},
        // the thermal drifts with the wind; the state transition is identity
        transition{[](const State &x, const Metres &drift_x, const Metres &drift_y) -> State {
            return x + State{0.0f * (m / s), 0.0f * m, drift_x, drift_y};
        }},
        observation{[](const State &x, const Metres &px, const Metres &py) -> Output {
            return Output{x.at<0>() * gaussian(x, px, py)};
        }},
        update_types<Metres, Metres>,
        prediction_types<Metres, Metres>};
}

} // namespace

struct ExtendedKalmanFilter::Thermal {
    decltype(make_filter()) filter{make_filter()};
};

void ExtendedKalmanFilter::reset(const float x[N], const float p[N], const float q[N], float r)
{
    // created on the first use, then reused
    if (_thermal == nullptr) {
        _thermal = NEW_NOTHROW Thermal;
    }
    for (uint8_t i = 0; i < N; i++) {
        X[i] = x[i];
    }
    if (_thermal == nullptr) {
        return;
    }

    Covariance pc{};
    Covariance::element<0, 0> p00{p[0] * m2 / s2};
    Covariance::element<1, 1> p11{p[1] * m2};
    Covariance::element<2, 2> p22{p[2] * m2};
    Covariance::element<3, 3> p33{p[3] * m2};
    pc.at<0, 0>(p00);
    pc.at<1, 1>(p11);
    pc.at<2, 2>(p22);
    pc.at<3, 3>(p33);

    Covariance qc{};
    Covariance::element<0, 0> q00{q[0] * m2 / s2};
    Covariance::element<1, 1> q11{q[1] * m2};
    Covariance::element<2, 2> q22{q[2] * m2};
    Covariance::element<3, 3> q33{q[3] * m2};
    qc.at<0, 0>(q00);
    qc.at<1, 1>(q11);
    qc.at<2, 2>(q22);
    qc.at<3, 3>(q33);

    auto &filter = _thermal->filter;
    filter.x(State{x[0] * (m / s), x[1] * m, x[2] * m, x[3] * m});
    filter.p(pc);
    filter.q(qc);
    filter.r(OutputVariance{r * m2 / s2});
}

void ExtendedKalmanFilter::update(float z, float Px, float Py, float driftX, float driftY)
{
    if (_thermal == nullptr) {
        return;
    }
    auto &filter = _thermal->filter;

    filter.predict(driftX * m, driftY * m);
    filter.update(Px * m, Py * m, Output{z * (m / s)});

    // make sure the radius stays above 40m
    const State x{filter.x()};
    if (x.at<1>() < 40.0f * m) {
        filter.x(State{x.at<0>(), 40.0f * m, x.at<2>(), x.at<3>()});
    }

    const State estimate{filter.x()};
    X[0] = value(estimate.at<0>());
    X[1] = value(estimate.at<1>());
    X[2] = value(estimate.at<2>());
    X[3] = value(estimate.at<3>());
}
