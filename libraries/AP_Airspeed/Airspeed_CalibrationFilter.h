/*
  The airspeed calibration's Kalman filter, algorithm by Paul Riseborough: the
  wind north and east in m/s and the airspeed scale, fused from the true
  airspeed once a second, every covariance element in its own unit.

  Included by Airspeed_Calibration.cpp only: the libraries it brings in are
  costly to compile, and AP_Airspeed.h, which every vehicle source includes,
  sees the filter as storage of its size.
 */
#pragma once

#include <AP_Math/ftype.h>
#include <AP_Math/typed_linear_algebra.h>

#include <algorithm>
#include <cmath>

namespace airspeed_calibration {

using typed_linear_algebra::Element;
using typed_linear_algebra::Speed;

// The airspeed scale, the inverse square root of the airspeed ratio.
using Scale = typed_linear_algebra::Quantity<mp_units::one>;

// The wind north and east, and the airspeed scale.
using State = typed_linear_algebra::Vector<Speed, Speed, Scale>;
using Covariance = typed_linear_algebra::Matrix<State, State>;
// The true airspeed.
using Output = typed_linear_algebra::Vector<Speed>;
using OutputVariance = typed_linear_algebra::Matrix<Output, Output>;
using OutputModel = fcarouge::kalman_filter::internal::evaluate<
    fcarouge::kalman_filter::internal::quotient<Output, State>>;
using SpeedSquared = Element<Covariance, 0, 0>;

inline constexpr auto metre_per_second = mp_units::si::metre / mp_units::si::second;

// The square root of a squared speed: mp-units' sqrt() finds only a sqrt() of
// the representation by argument-dependent lookup on a freestanding build,
// which a float does not have.
inline Speed square_root(SpeedSquared value)
{
    return sqrtf(value.numerical_value_in(SpeedSquared::unit)) * metre_per_second;
}

template <typename Representation>
inline auto square_root(mp_units::quantity<SpeedSquared::reference, Representation> value)
{
    return sqrtF(value.numerical_value_in(SpeedSquared::unit)) * metre_per_second;
}

// The squared horizontal speed of the air relative to the aircraft.
inline SpeedSquared horizontal_squared(const State &x, Speed vgx, Speed vgy)
{
    return (vgy - x.at<1>()) * (vgy - x.at<1>()) + (vgx - x.at<0>()) * (vgx - x.at<0>());
}

// The true airspeed: the scaled norm of the ground velocity less the
// horizontal wind, the vertical wind ignored. As AP_Math's norm(), the last
// two squares are summed in float, the rest in ftype, double on boards with
// hardware double.
struct true_airspeed {
    Output operator()(const State &x, const Speed &vgx, const Speed &vgy, const Speed &vgz) const
    {
        const Speed dx = vgx - x.at<0>();
        const Speed dy = vgy - x.at<1>();
        const auto norm = square_root(mp_units::value_cast<ftype>(dx * dx) + mp_units::value_cast<ftype>(dy * dy + vgz * vgz));
        return Output{mp_units::value_cast<float>(mp_units::value_cast<ftype>(x.at<2>()) * norm)};
    }
};

// The observation Jacobian H_TAS, of the horizontal norm.
struct true_airspeed_jacobian {
    OutputModel operator()(const State &x, const Speed &vgx, const Speed &vgy, const Speed &) const
    {
        const auto SH2 = 1 / square_root(horizontal_squared(x, vgx, vgy));
        OutputModel h{};
        h.at<0>(-(x.at<2>() * SH2 * (2 * vgx - 2 * x.at<0>())) / 2);
        h.at<1>(-(x.at<2>() * SH2 * (2 * vgy - 2 * x.at<1>())) / 2);
        h.at<2>(1 / SH2);
        return h;
    }
};

inline Covariance diagonal(float speed_variance, float scale_variance)
{
    Covariance value{};
    value.at<0, 0>(speed_variance * Element<Covariance, 0, 0>::reference);
    value.at<1, 1>(speed_variance * Element<Covariance, 1, 1>::reference);
    value.at<2, 2>(scale_variance * Element<Covariance, 2, 2>::reference);
    return value;
}

// The filter, with the initial covariance, the process noise Q0 for the wind
// and Q1 for the scale, and a measurement noise of 1.0 m/s.
inline auto make_filter()
{
    return fcarouge::kalman{
        fcarouge::state{State{}}, fcarouge::output<Output>,
        fcarouge::estimate_uncertainty{diagonal(100, 0.000001f)},
        fcarouge::process_uncertainty{diagonal(0.01f, 0.0000005f)},
        fcarouge::output_uncertainty{OutputVariance{1.0f * Element<OutputVariance, 0, 0>::reference}},
        fcarouge::output_model{true_airspeed_jacobian{}},
        fcarouge::observation{true_airspeed{}}, fcarouge::update_types<Speed, Speed, Speed>,
        fcarouge::simple_form, fcarouge::symmetrize};
}

using Filter = decltype(make_filter());

// Sets the airspeed scale from the airspeed ratio.
inline void set_ratio(Filter &filter, float ratio)
{
    State x{filter.x()};
    x.at<2>(1.0f / sqrtf(ratio) * mp_units::one);
    filter.x(x);
}

// Updates the calibration with the true airspeed at a ratio of 1 [m/s] and
// the ground velocity [m/s], the wind constrained to the given speed [m/s];
// returns the airspeed scale.
inline float update(Filter &filter, float airspeed, float vgx_, float vgy_, float vgz_, float max_wind_)
{
    // Perform the covariance prediction: the states are time invariant
    // plus process noise, P = P + Q
    filter.predict();

    // avoid division by a small number in the observation Jacobian
    const Speed vgx = vgx_ * metre_per_second;
    const Speed vgy = vgy_ * metre_per_second;
    const Speed vgz = vgz_ * metre_per_second;
    if (horizontal_squared(filter.x(), vgx, vgy) < 0.000001f * SpeedSquared::reference) {
        return filter.x().at<2>().numerical_value_in(mp_units::one);
    }

    // fuse the true airspeed measurement, P -= K (H P)
    filter.update(vgx, vgy, vgz, airspeed * metre_per_second);

    // the filter forces symmetry on the covariance matrix - necessary due to
    // rounding errors - and the diagonals are constrained to be non-negative -
    // protects against rounding errors
    Covariance p{filter.p()};
    p.at<0, 0>(std::max(p.at<0, 0>(), 0.0f * Element<Covariance, 0, 0>::reference));
    p.at<1, 1>(std::max(p.at<1, 1>(), 0.0f * Element<Covariance, 1, 1>::reference));
    p.at<2, 2>(std::max(p.at<2, 2>(), 0.0f * Element<Covariance, 2, 2>::reference));
    filter.p(p);

    const Speed max_wind = max_wind_ * metre_per_second;
    State x{filter.x()};
    x.at<0>(std::clamp(x.at<0>(), -max_wind, max_wind));
    x.at<1>(std::clamp(x.at<1>(), -max_wind, max_wind));
    x.at<2>(std::clamp(x.at<2>(), 0.5f * mp_units::one, 1.0f * mp_units::one));
    filter.x(x);

    return x.at<2>().numerical_value_in(mp_units::one);
}

} // namespace airspeed_calibration
