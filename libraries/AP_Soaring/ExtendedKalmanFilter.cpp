#include "ExtendedKalmanFilter.h"

#include <cmath>

using mp_units::si::metre;

namespace {

// The exponential of a ratio: mp-units' exp() finds only an exp() of the
// representation by argument-dependent lookup on a freestanding build, which a
// float does not have.
mp_units::quantity<mp_units::one, float> exponential(mp_units::quantity<mp_units::one, float> ratio)
{
    return std::exp(ratio.numerical_value_in(mp_units::one)) * mp_units::one;
}

}

ExtendedKalmanFilter::OutputModel ExtendedKalmanFilter::updraft_jacobian::operator()(const State &x, const Length &px, const Length &py) const
{
    const auto dx = x.at<2>() - px;
    const auto dy = x.at<3>() - py;
    const auto r = x.at<1>();
    const auto expon = exponential(-(dx * dx + dy * dy) / (r * r));
    OutputModel h{};
    h.at<0>(expon);
    h.at<1>(2 * x.at<0>() * ((dx * dx + dy * dy) / (r * r * r)) * expon);
    h.at<2>(-2 * (x.at<0>() * dx / (r * r)) * expon);
    h.at<3>(-2 * (x.at<0>() * dy / (r * r)) * expon);
    return h;
}

ExtendedKalmanFilter::State ExtendedKalmanFilter::drift::operator()(const State &x, const Length &drift_x, const Length &drift_y) const
{
    State y{x};
    y.at<2>(x.at<2>() + drift_x);
    y.at<3>(x.at<3>() + drift_y);
    return y;
}

ExtendedKalmanFilter::Output ExtendedKalmanFilter::updraft::operator()(const State &x, const Length &px, const Length &py) const
{
    const auto dx = x.at<2>() - px;
    const auto dy = x.at<3>() - py;
    const auto r = x.at<1>();
    return Output{x.at<0>() * exponential(-(dx * dx + dy * dy) / (r * r))};
}

void ExtendedKalmanFilter::reset(const State &x, const Covariance &p, const Covariance &q, Variance r)
{
    _filter.x(x);
    _filter.p(p);
    _filter.q(q);
    _filter.r(r);
}

void ExtendedKalmanFilter::update(Speed z, Length px, Length py, Length drift_x, Length drift_y)
{
    // the thermal drifts, and its uncertainty grows: X += drift, P += Q
    _filter.predict(drift_x, drift_y);

    // the variometer reading corrects it, P -= K (H P), and the covariance
    // stays symmetric, (P + Pᵀ) / 2
    _filter.update(px, py, z);

    // the radius stays above 40 metres
    if (_filter.x().at<1>() < 40.0f * metre) {
        State x{_filter.x()};
        x.at<1>(40.0f * metre);
        _filter.x(x);
    }
}
