/*
 *   auto_calibration.cpp - airspeed auto calibration
 *
 * Algorithm by Paul Riseborough
 *
 */

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <SRV_Channel/SRV_Channel.h>

#include "AP_Airspeed.h"

#if AP_AIRSPEED_AUTOCAL_ENABLE

#include "Airspeed_Calibration.h"
#include <AP_LinearAlgebra/AP_LinearAlgebra_Kalman.h>

#include <new>

namespace {

using MetresPerSecond = Airspeed_Calibration::MetresPerSecond;
using RootPressure = Airspeed_Calibration::RootPressure;
using Scale = Airspeed_Calibration::Scale;
using ScaleVariance = Airspeed_Calibration::ScaleVariance;
constexpr auto metre_per_second = mp_units::si::metre / mp_units::si::second;

// the scale is 1/sqrt(ratio): scale^2 ratio is dimensionless
static_assert(std::is_convertible_v<decltype(Scale{} * Scale{} * Airspeed_Calibration::Ratio{}), AP_LinearAlgebra::Units::Unitless>);

// state: wind north and east, and the scale 1/sqrt(ratio) from the airspeed
// to the square root of the differential pressure
using State = AP_LinearAlgebra::ColumnVector<float, MetresPerSecond, MetresPerSecond, Scale>;
// output: the square root of the differential pressure, corrected to true
// airspeed
using Output = AP_LinearAlgebra::ColumnVector<float, RootPressure>;
using Covariance = AP_LinearAlgebra::OuterProduct<State, State>;
using OutputVariance = AP_LinearAlgebra::OuterProduct<Output, Output>;
using OutputModel = AP_LinearAlgebra::Quotient<Output, State>;

// the horizontal airspeed implied by the ground velocity and the wind
float horizontal_airspeed_squared(const State &x, const MetresPerSecond &vg_x, const MetresPerSecond &vg_y)
{
    const MetresPerSecond ax = vg_x - x.at<0>();
    const MetresPerSecond ay = vg_y - x.at<1>();
    return (ax * ax + ay * ay).numerical_value_in(metre_per_second * metre_per_second);
}

auto make_filter()
{
    using namespace fcarouge;

    Covariance p{};
    p.at<0, 0>(100.0f * (metre_per_second * metre_per_second));
    p.at<1, 1>(100.0f * (metre_per_second * metre_per_second));
    p.at<2, 2>(0.000001f * ScaleVariance::reference);

    // the wind and scale are constant but for this process noise
    Covariance q{};
    q.at<0, 0>(0.01f * (metre_per_second * metre_per_second));
    q.at<1, 1>(0.01f * (metre_per_second * metre_per_second));
    q.at<2, 2>(0.0000005f * ScaleVariance::reference);

    return kalman{
        state{State{0.0f * metre_per_second, 0.0f * metre_per_second, 0.0f * Scale::reference}},
        output<Output>,
        estimate_uncertainty{p},
        process_uncertainty{q},
        // an observation noise of 1 Pa: 1 m/s of true airspeed at a ratio of 1
        output_uncertainty{OutputVariance{1.0f * mp_units::si::pascal}},
        // H, the Jacobian of the predicted observation with respect to the
        // state, ignoring the vertical wind component
        output_model{[](const State &x, const MetresPerSecond &vg_x, const MetresPerSecond &vg_y, const MetresPerSecond &) -> OutputModel {
            // the inverse of the horizontal airspeed, in s/m
            const auto SH2 = 1 / (sqrtf(horizontal_airspeed_squared(x, vg_x, vg_y)) * metre_per_second);
            const Scale scale = x.at<2>();
            OutputModel h{};
            h.at<0>(-(scale * SH2 * 2 * (vg_x - x.at<0>())) / 2);
            h.at<1>(-(scale * SH2 * 2 * (vg_y - x.at<1>())) / 2);
            h.at<2>(1 / SH2);
            return h;
        }},
        transition{[](const State &x) -> State {
            return x;
        }},
        // the predicted observation, the scaled ground relative airspeed
        observation{[](const State &x, const MetresPerSecond &vg_x, const MetresPerSecond &vg_y, const MetresPerSecond &vg_z) -> Output {
            const MetresPerSecond ax = vg_x - x.at<0>();
            const MetresPerSecond ay = vg_y - x.at<1>();
            return Output{x.at<2>() * norm(ax.numerical_value_in(metre_per_second),
                                           ay.numerical_value_in(metre_per_second),
                                           vg_z.numerical_value_in(metre_per_second)) * metre_per_second};
        }},
        update_types<MetresPerSecond, MetresPerSecond, MetresPerSecond>,
        // the configuration requires the (empty) prediction argument types
        prediction_types<>};
}

} // namespace

struct Airspeed_Calibration::Filter {
    decltype(make_filter()) kalman{make_filter()};
};

Airspeed_Calibration::Airspeed_Calibration()
{
    static_assert(sizeof(Filter) <= filter_size, "Airspeed_Calibration::filter_size is too small for the filter");
    static_assert(alignof(Filter) <= 8, "Airspeed_Calibration::_storage is under-aligned for the filter");
    _filter = new (_storage) Filter;
}

Airspeed_Calibration::~Airspeed_Calibration()
{
    _filter->~Filter();
}

Airspeed_Calibration::Scale Airspeed_Calibration::scale_of(Ratio ratio)
{
    return (1.0f / sqrtf(ratio.numerical_value_in(Ratio::unit))) * Scale::reference;
}

Airspeed_Calibration::Ratio Airspeed_Calibration::ratio_of(Scale scale)
{
    return (1.0f / sq(scale.numerical_value_in(Scale::unit))) * Ratio::reference;
}

/*
  initialise the ratio
 */
void Airspeed_Calibration::init(Ratio initial_ratio)
{
    set_scale(scale_of(initial_ratio));
}

void Airspeed_Calibration::set_scale(Scale scale)
{
    auto &kalman = _filter->kalman;
    const State x{kalman.x()};
    kalman.x(State{x.at<0>(), x.at<1>(), scale});
}

Airspeed_Calibration::Estimate Airspeed_Calibration::get_state() const
{
    const State x{_filter->kalman.x()};
    return Estimate{x.at<0>(), x.at<1>(), x.at<2>()};
}

Airspeed_Calibration::EstimateVariances Airspeed_Calibration::get_variances() const
{
    const Covariance p{_filter->kalman.p()};
    return EstimateVariances{p.at<0, 0>(), p.at<1, 1>(), p.at<2, 2>()};
}

/*
  update the state of the airspeed calibration - needs to be called
  once a second
 */
Airspeed_Calibration::Scale Airspeed_Calibration::update(RootPressure root_pressure, MetresPerSecond vg_north,
                                                         MetresPerSecond vg_east, MetresPerSecond vg_down,
                                                         MetresPerSecond max_wind)
{
    auto &kalman = _filter->kalman;

    // Perform the covariance prediction, P = P + Q. No state prediction
    // required because states are assumed to be time invariant plus
    // process noise
    kalman.predict();

    if (horizontal_airspeed_squared(kalman.x(), vg_north, vg_east) < 0.000001f) {
        // avoid division by a small number
        return kalman.x().at<2>();
    }

    kalman.update(vg_north, vg_east, vg_down, Output{root_pressure});

    // force symmetry on the covariance matrix - necessary due to rounding
    // errors - and constrain diagonals to be non-negative
    Covariance p{kalman.p()};
    const auto p01 = 0.5f * (p.at<0, 1>() + p.at<1, 0>());
    const auto p02 = 0.5f * (p.at<0, 2>() + p.at<2, 0>());
    const auto p12 = 0.5f * (p.at<1, 2>() + p.at<2, 1>());
    p.at<0, 1>(p01);
    p.at<1, 0>(p01);
    p.at<0, 2>(p02);
    p.at<2, 0>(p02);
    p.at<1, 2>(p12);
    p.at<2, 1>(p12);
    p.at<0, 0>(MAX(p.at<0, 0>().numerical_value_in(metre_per_second * metre_per_second), 0.0f) * (metre_per_second * metre_per_second));
    p.at<1, 1>(MAX(p.at<1, 1>().numerical_value_in(metre_per_second * metre_per_second), 0.0f) * (metre_per_second * metre_per_second));
    p.at<2, 2>(MAX(p.at<2, 2>().numerical_value_in(ScaleVariance::unit), 0.0f) * ScaleVariance::reference);
    kalman.p(p);

    // the scale between those of the ratios 4 and 1, and the wind within the limit
    const State x{kalman.x()};
    const float limit = max_wind.numerical_value_in(metre_per_second);
    const Scale scale = constrain_float(x.at<2>().numerical_value_in(Scale::unit), 0.5f, 1.0f) * Scale::reference;
    kalman.x(State{constrain_float(x.at<0>().numerical_value_in(metre_per_second), -limit, limit) * metre_per_second,
                   constrain_float(x.at<1>().numerical_value_in(metre_per_second), -limit, limit) * metre_per_second,
                   scale});

    return scale;
}

#endif // AP_AIRSPEED_AUTOCAL_ENABLE

/*
  called once a second to do calibration update
 */
void AP_Airspeed::update_calibration(uint8_t i, const Vector3f &vground, int16_t max_airspeed_allowed_during_cal)
{
#if AP_AIRSPEED_AUTOCAL_ENABLE
    if (!param[i].autocal && !calibration_enabled) {
        // auto-calibration not enabled
        return;
    }

    if (param[i].use == 2 && !is_zero(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle))) {
        // special case for gliders with airspeed sensors behind the
        // propeller. Allow airspeed to be disabled when throttle is
        // running
        return;
    }

    // set state.z based on current ratio, this allows the operator to
    // override the current ratio in flight with autocal, which is
    // very useful both for testing and to force a reasonable value.
    Airspeed_Calibration *calibration = state[i].calibration;
    if (calibration == nullptr) {
        return;
    }

    const Airspeed_Calibration::Ratio ratio = constrain_float(param[i].ratio, 1.0f, 4.0f) * Airspeed_Calibration::Ratio::reference;

    calibration->set_scale(Airspeed_Calibration::scale_of(ratio));

    // the square root of the differential pressure, corrected to true
    // airspeed: the true airspeed at an airspeed ratio of 1
    float dpress = MAX(get_differential_pressure(i), 0);
    const Airspeed_Calibration::RootPressure root_pressure = sqrtf(dpress) * AP::ahrs().get_EAS2TAS() * Airspeed_Calibration::RootPressure::reference;

    float zratio = calibration->update(root_pressure, vground.x * metre_per_second, vground.y * metre_per_second,
                                       vground.z * metre_per_second, max_airspeed_allowed_during_cal * metre_per_second)
                       .numerical_value_in(Airspeed_Calibration::Scale::unit);

    if (isnan(zratio) || isinf(zratio)) {
        return;
    }

    // this constrains the resulting ratio to between 1.0 and 4.0
    zratio = constrain_float(zratio, 0.5f, 1.0f);
    param[i].ratio.set(1/sq(zratio));
    if (state[i].counter > 60) {
        if (state[i].last_saved_ratio > 1.05f*param[i].ratio ||
            state[i].last_saved_ratio < 0.95f*param[i].ratio) {
            param[i].ratio.save();
            state[i].last_saved_ratio = param[i].ratio;
            state[i].counter = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Airspeed %u ratio reset: %f", i , static_cast<double> (param[i].ratio));
        }
    } else {
        state[i].counter++;
    }
#endif // AP_AIRSPEED_AUTOCAL_ENABLE
}

/*
  called once a second to do calibration update
 */
void AP_Airspeed::update_calibration(const Vector3f &vground, int16_t max_airspeed_allowed_during_cal)
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        update_calibration(i, vground, max_airspeed_allowed_during_cal);
    }
#if HAL_GCS_ENABLED && AP_AIRSPEED_AUTOCAL_ENABLE
    send_airspeed_calibration(vground);
#endif
}


#if HAL_GCS_ENABLED && AP_AIRSPEED_AUTOCAL_ENABLE
void AP_Airspeed::send_airspeed_calibration(const Vector3f &vground)
{
    /*
      the AIRSPEED_AUTOCAL message doesn't have an instance number
      so we can only send it for one sensor at a time
     */
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        if (!param[i].autocal && !calibration_enabled) {
            // auto-calibration not enabled on this sensor
            continue;
        }
        if (state[i].calibration == nullptr) {
            continue;
        }
        const Airspeed_Calibration::Estimate estimate = state[i].calibration->get_state();
        const Airspeed_Calibration::EstimateVariances variances = state[i].calibration->get_variances();
        const auto metre_per_second_squared = metre_per_second * metre_per_second;
        const mavlink_airspeed_autocal_t packet{
        vx: vground.x,
        vy: vground.y,
        vz: vground.z,
        diff_pressure: get_differential_pressure(i),
        EAS2TAS: AP::ahrs().get_EAS2TAS(),
        ratio: param[i].ratio.get(),
        state_x: estimate.wind_north.numerical_value_in(metre_per_second),
        state_y: estimate.wind_east.numerical_value_in(metre_per_second),
        state_z: estimate.scale.numerical_value_in(Airspeed_Calibration::Scale::unit),
        Pax: variances.wind_north.numerical_value_in(metre_per_second_squared),
        Pby: variances.wind_east.numerical_value_in(metre_per_second_squared),
        Pcz: variances.scale.numerical_value_in(Airspeed_Calibration::ScaleVariance::unit)
        };
        gcs().send_to_active_channels(MAVLINK_MSG_ID_AIRSPEED_AUTOCAL,
                                      (const char *)&packet);
        break; // we can only send for one sensor
    }
}
#endif  // HAL_GCS_ENABLED && AP_AIRSPEED_AUTOCAL_ENABLE

#endif  // AP_AIRSPEED_ENABLED
