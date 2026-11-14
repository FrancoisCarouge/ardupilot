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


#include "Airspeed_CalibrationFilter.h"

#include <new>
#include <type_traits>

namespace {

using CalibrationFilter = airspeed_calibration::Filter;

template <typename Storage>
CalibrationFilter &filter_in(Storage &storage)
{
    return *std::launder(reinterpret_cast<CalibrationFilter *>(storage));
}

template <typename Storage>
const CalibrationFilter &filter_in(const Storage &storage)
{
    return *std::launder(reinterpret_cast<const CalibrationFilter *>(storage));
}

}

// constructor - fill in all the initial values
Airspeed_Calibration::Airspeed_Calibration()
{
    // The storage holds the filter exactly, which is never destroyed.
    static_assert(sizeof(_filter_storage) == sizeof(CalibrationFilter));
    static_assert(alignof(float) >= alignof(CalibrationFilter));
    static_assert(std::is_trivially_destructible_v<CalibrationFilter>);

    new (_filter_storage) CalibrationFilter{airspeed_calibration::make_filter()};
}

/*
  initialise the ratio
 */
void Airspeed_Calibration::init(float initial_ratio)
{
    airspeed_calibration::set_ratio(filter_in(_filter_storage), initial_ratio);
}

Vector3f Airspeed_Calibration::get_state() const
{
    using namespace airspeed_calibration;
    const State &x = filter_in(_filter_storage).x();
    return Vector3f{x.at<0>().numerical_value_in(metre_per_second),
                    x.at<1>().numerical_value_in(metre_per_second),
                    x.at<2>().numerical_value_in(mp_units::one)};
}

Vector3f Airspeed_Calibration::get_variances() const
{
    using namespace airspeed_calibration;
    const Covariance &p = filter_in(_filter_storage).p();
    return Vector3f{p.at<0, 0>().numerical_value_in(SpeedSquared::unit),
                    p.at<1, 1>().numerical_value_in(SpeedSquared::unit),
                    p.at<2, 2>().numerical_value_in(mp_units::one)};
}

/*
  update the state of the airspeed calibration - needs to be called
  once a second
 */
float Airspeed_Calibration::update(float airspeed, const Vector3f &vg, int16_t max_airspeed_allowed_during_cal)
{
    return airspeed_calibration::update(filter_in(_filter_storage), airspeed, vg.x, vg.y, vg.z, max_airspeed_allowed_during_cal);
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

    state[i].calibration.init(ratio);

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
        const mavlink_airspeed_autocal_t packet{
        vx: vground.x,
        vy: vground.y,
        vz: vground.z,
        diff_pressure: get_differential_pressure(i),
        EAS2TAS: AP::ahrs().get_EAS2TAS(),
        ratio: param[i].ratio.get(),
        state_x: state[i].calibration.get_state().x,
        state_y: state[i].calibration.get_state().y,
        state_z: state[i].calibration.get_state().z,
        Pax: state[i].calibration.get_variances().x,
        Pby: state[i].calibration.get_variances().y,
        Pcz: state[i].calibration.get_variances().z
        };
        gcs().send_to_active_channels(MAVLINK_MSG_ID_AIRSPEED_AUTOCAL,
                                      (const char *)&packet);
        break; // we can only send for one sensor
    }
}
#endif  // HAL_GCS_ENABLED && AP_AIRSPEED_AUTOCAL_ENABLE

#endif  // AP_AIRSPEED_ENABLED
