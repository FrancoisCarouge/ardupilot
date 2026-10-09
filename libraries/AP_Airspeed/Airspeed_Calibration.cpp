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

#include <AP_LinearAlgebra/AP_LinearAlgebra_Kalman.h>

#include <new>


namespace {

using AP_LinearAlgebra::Units::MetresPerSecond;
using AP_LinearAlgebra::Units::Unitless;

constexpr auto metre_per_second = mp_units::si::metre / mp_units::si::second;
constexpr auto one = mp_units::one;

// state: wind north and east, and the scale factor 1/sqrt(ratio) from
// indicated to true airspeed
using State = AP_LinearAlgebra::ColumnVector<float, MetresPerSecond, MetresPerSecond, Unitless>;
// output: true airspeed
using Output = AP_LinearAlgebra::ColumnVector<float, MetresPerSecond>;
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
    p.at<2, 2>(0.000001f * one);

    // the wind and scale factor are constant but for this process noise
    Covariance q{};
    q.at<0, 0>(0.01f * (metre_per_second * metre_per_second));
    q.at<1, 1>(0.01f * (metre_per_second * metre_per_second));
    q.at<2, 2>(0.0000005f * one);

    return kalman{
        state{State{0.0f * metre_per_second, 0.0f * metre_per_second, 0.0f * one}},
        output<Output>,
        estimate_uncertainty{p},
        process_uncertainty{q},
        // a true airspeed measurement noise of 1.0 m/s
        output_uncertainty{OutputVariance{1.0f * (metre_per_second * metre_per_second)}},
        // H, the Jacobian of the predicted true airspeed with respect to the
        // state, ignoring the vertical wind component
        output_model{[](const State &x, const MetresPerSecond &vg_x, const MetresPerSecond &vg_y, const MetresPerSecond &) -> OutputModel {
            // the inverse of the horizontal airspeed, in s/m
            const auto SH2 = 1 / (sqrtf(horizontal_airspeed_squared(x, vg_x, vg_y)) * metre_per_second);
            const float scale = x.at<2>().numerical_value_in(one);
            OutputModel h{};
            h.at<0>(-(scale * SH2 * 2 * (vg_x - x.at<0>())) / 2);
            h.at<1>(-(scale * SH2 * 2 * (vg_y - x.at<1>())) / 2);
            h.at<2>(1 / SH2);
            return h;
        }},
        transition{[](const State &x) -> State {
            return x;
        }},
        // the predicted true airspeed, scaled ground relative airspeed
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
}

Airspeed_Calibration::~Airspeed_Calibration()
{
    if (_filter != nullptr) {
        _filter->~Filter();
    }
}

// the Kalman library's identity and zero values are dynamically initialized
// variables: a filter constructed during static initialization, as a member
// of a global object, may copy them before they are set
Airspeed_Calibration::Filter &Airspeed_Calibration::filter()
{
    if (_filter == nullptr) {
        _filter = new (_storage) Filter;
    }
    return *_filter;
}

/*
  initialise the ratio
 */
void Airspeed_Calibration::init(float initial_ratio)
{
    set_scale(1.0f / sqrtf(initial_ratio));
}

void Airspeed_Calibration::set_scale(float scale)
{
    auto &kalman = filter().kalman;
    const State x{kalman.x()};
    kalman.x(State{x.at<0>(), x.at<1>(), scale * one});
}

Vector3f Airspeed_Calibration::get_state() const
{
    if (_filter == nullptr) {
        return Vector3f();
    }
    const State x{_filter->kalman.x()};
    return Vector3f(x.at<0>().numerical_value_in(metre_per_second),
                    x.at<1>().numerical_value_in(metre_per_second),
                    x.at<2>().numerical_value_in(one));
}

Vector3f Airspeed_Calibration::get_variances() const
{
    if (_filter == nullptr) {
        return Vector3f(100, 100, 0.000001f);
    }
    const Covariance p{_filter->kalman.p()};
    return Vector3f(p.at<0, 0>().numerical_value_in(metre_per_second * metre_per_second),
                    p.at<1, 1>().numerical_value_in(metre_per_second * metre_per_second),
                    p.at<2, 2>().numerical_value_in(one));
}

/*
  update the state of the airspeed calibration - needs to be called
  once a second
 */
float Airspeed_Calibration::update(float airspeed, const Vector3f &vg, int16_t max_airspeed_allowed_during_cal)
{
    auto &kalman = filter().kalman;

    // Perform the covariance prediction, P = P + Q. No state prediction
    // required because states are assumed to be time invariant plus
    // process noise
    kalman.predict();

    const MetresPerSecond vg_x = vg.x * metre_per_second;
    const MetresPerSecond vg_y = vg.y * metre_per_second;
    const MetresPerSecond vg_z = vg.z * metre_per_second;

    if (horizontal_airspeed_squared(kalman.x(), vg_x, vg_y) < 0.000001f) {
        // avoid division by a small number
        return kalman.x().at<2>().numerical_value_in(one);
    }

    kalman.update(vg_x, vg_y, vg_z, Output{airspeed * metre_per_second});

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
    p.at<2, 2>(MAX(p.at<2, 2>().numerical_value_in(one), 0.0f) * one);
    kalman.p(p);

    const State x{kalman.x()};
    const float limit = max_airspeed_allowed_during_cal;
    const float scale = constrain_float(x.at<2>().numerical_value_in(one), 0.5f, 1.0f);
    kalman.x(State{constrain_float(x.at<0>().numerical_value_in(metre_per_second), -limit, limit) * metre_per_second,
                   constrain_float(x.at<1>().numerical_value_in(metre_per_second), -limit, limit) * metre_per_second,
                   scale * one});

    return scale;
}


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
    float ratio = constrain_float(param[i].ratio, 1.0f, 4.0f);

    state[i].calibration.set_scale(1.0f / sqrtf(ratio));

    // calculate true airspeed, assuming a airspeed ratio of 1.0
    float dpress = MAX(get_differential_pressure(i), 0);
    float true_airspeed = sqrtf(dpress) * AP::ahrs().get_EAS2TAS();

    float zratio = state[i].calibration.update(true_airspeed, vground, max_airspeed_allowed_during_cal);

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
        const Vector3f calibration_state = state[i].calibration.get_state();
        const Vector3f calibration_variances = state[i].calibration.get_variances();
        const mavlink_airspeed_autocal_t packet{
        vx: vground.x,
        vy: vground.y,
        vz: vground.z,
        diff_pressure: get_differential_pressure(i),
        EAS2TAS: AP::ahrs().get_EAS2TAS(),
        ratio: param[i].ratio.get(),
        state_x: calibration_state.x,
        state_y: calibration_state.y,
        state_z: calibration_state.z,
        Pax: calibration_variances.x,
        Pby: calibration_variances.y,
        Pcz: calibration_variances.z
        };
        gcs().send_to_active_channels(MAVLINK_MSG_ID_AIRSPEED_AUTOCAL,
                                      (const char *)&packet);
        break; // we can only send for one sensor
    }
}
#endif  // HAL_GCS_ENABLED && AP_AIRSPEED_AUTOCAL_ENABLE

#endif  // AP_AIRSPEED_ENABLED
