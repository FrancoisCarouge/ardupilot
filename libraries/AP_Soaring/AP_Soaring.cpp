#include "AP_Soaring.h"

#if HAL_SOARING_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_TECS/AP_TECS.h>
#include <GCS_MAVLink/GCS.h>
#include <stdint.h>

#include <new>
#include <type_traits>

#include "ExtendedKalmanFilter.h"

using mp_units::si::metre;
using mp_units::si::second;
using typed_linear_algebra::Element;

namespace {
// A diagonal covariance of the thermal: each variance in its own unit,
// (m/s)², m², m², m².
ExtendedKalmanFilter::Covariance diagonal_covariance(const float (&variances)[4])
{
    using Covariance = ExtendedKalmanFilter::Covariance;
    Covariance value{};
    value.at<0, 0>(variances[0] * Element<Covariance, 0, 0>::reference);
    value.at<1, 1>(variances[1] * Element<Covariance, 1, 1>::reference);
    value.at<2, 2>(variances[2] * Element<Covariance, 2, 2>::reference);
    value.at<3, 3>(variances[3] * Element<Covariance, 3, 3>::reference);
    return value;
}
}

float SoaringController::thermal_strength() const
{
    return _ekf.strength().numerical_value_in(metre / second);
}

float SoaringController::thermal_radius() const
{
    return _ekf.radius().numerical_value_in(metre);
}

float SoaringController::thermal_x() const
{
    return _ekf.position_x().numerical_value_in(metre);
}

float SoaringController::thermal_y() const
{
    return _ekf.position_y().numerical_value_in(metre);
}

#if HAL_SOARING_NVF_EKF_ENABLED
static constexpr uint32_t NVF_PUBLISHER_DELAY_MS = 250;
#endif // HAL_SOARING_NVF_EKF_ENABLED

// ArduSoar parameters
const AP_Param::GroupInfo SoaringController::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Is the soaring mode enabled or not
    // @Description: Toggles the soaring mode on and off
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, SoaringController, soar_active, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: VSPEED
    // @DisplayName: Vertical v-speed
    // @Description: Rate of climb to trigger themalling speed
    // @Units: m/s
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("VSPEED", 2, SoaringController, thermal_vspeed, 0.7f),

    // @Param: Q1
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process for strength
    // @Range: 0.0001 0.01
    // @User: Advanced
    AP_GROUPINFO("Q1", 3, SoaringController, thermal_q1, 0.001f),

    // @Param: Q2
    // @DisplayName: Process noise
    // @Description: Standard deviation of noise in process for position and radius
    // @Range: 0.01 1
    // @User: Advanced
    AP_GROUPINFO("Q2", 4, SoaringController, thermal_q2, 0.03f),

    // @Param: R
    // @DisplayName: Measurement noise
    // @Description: Standard deviation of noise in measurement
    // @Range: 0.01 1
    // @User: Advanced

    AP_GROUPINFO("R", 5, SoaringController, thermal_r, 0.45f),

    // @Param: DIST_AHEAD
    // @DisplayName: Distance to thermal center
    // @Description: Initial guess of the distance to the thermal center
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("DIST_AHEAD", 6, SoaringController, thermal_distance_ahead, 5.0f),

    // @Param: MIN_THML_S
    // @DisplayName: Minimum thermalling time
    // @Description: Minimum number of seconds to spend thermalling
    // @Units: s
    // @Range: 0 600
    // @User: Advanced
    AP_GROUPINFO("MIN_THML_S", 7, SoaringController, min_thermal_s, 20),

    // @Param: MIN_CRSE_S
    // @DisplayName: Minimum cruising time
    // @Description: Minimum number of seconds to spend cruising
    // @Units: s
    // @Range: 0 600
    // @User: Advanced
    AP_GROUPINFO("MIN_CRSE_S", 8, SoaringController, min_cruise_s, 10),

    // @Param: POLAR_CD0
    // @DisplayName: Zero lift drag coef.
    // @Description: Zero lift drag coefficient
    // @Range: 0.005 0.5
    // @User: Advanced
    AP_GROUPINFO("POLAR_CD0", 9, SoaringController, _polarParams.CD0, 0.027),

    // @Param: POLAR_B
    // @DisplayName: Induced drag coeffient
    // @Description: Induced drag coeffient
    // @Range: 0.005 0.05
    // @User: Advanced
    AP_GROUPINFO("POLAR_B", 10, SoaringController, _polarParams.B, 0.031),

    // @Param: POLAR_K
    // @DisplayName: Cl factor
    // @Description: Cl factor 2*m*g/(rho*S)
    // @Units: m.m/s/s
    // @Range: 20 400
    // @User: Advanced
    AP_GROUPINFO("POLAR_K", 11, SoaringController, _polarParams.K, 25.6),

    // @Param: ALT_MAX
    // @DisplayName: Maximum soaring altitude, relative to the home location
    // @Description: Don't thermal any higher than this.
    // @Units: m
    // @Range: 0 5000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_MAX", 12, SoaringController, alt_max, 350.0),

    // @Param: ALT_MIN
    // @DisplayName: Minimum soaring altitude, relative to the home location
    // @Description: Don't get any lower than this.
    // @Units: m
    // @Range: 0 1000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_MIN", 13, SoaringController, alt_min, 50.0),

    // @Param: ALT_CUTOFF
    // @DisplayName: Maximum power altitude, relative to the home location
    // @Description: Cut off throttle at this alt.
    // @Units: m
    // @Range: 0 5000.0
    // @User: Advanced
    AP_GROUPINFO("ALT_CUTOFF", 14, SoaringController, alt_cutoff, 250.0),
    
    // 15 was SOAR_ENABLE_CH, now RCX_OPTION

    // @Param: MAX_DRIFT
    // @DisplayName: (Optional) Maximum drift distance to allow when thermalling.
    // @Description: The previous mode will be restored if the horizontal distance to the thermalling start location exceeds this value. -1 to disable.
    // @Range: -1 1000
    // @User: Advanced
    AP_GROUPINFO("MAX_DRIFT", 16, SoaringController, max_drift, -1),

    // @Param: MAX_RADIUS
    // @DisplayName: (Optional) Maximum distance from home
    // @Description: RTL will be entered when a thermal is exited and the plane is more than this distance from home. -1 to disable.
    // @Range: -1 1000
    // @User: Advanced
    AP_GROUPINFO("MAX_RADIUS", 17, SoaringController, max_radius, -1),

    // @Param: THML_BANK
    // @DisplayName: Thermalling bank angle
    // @Description: This parameter sets the bank angle to use when thermalling. Typically 30 - 45 degrees works well.
    // @Range: 20 50
    // @User: Advanced
    // @Units: deg
    AP_GROUPINFO("THML_BANK", 18, SoaringController, thermal_bank, 30.0),

    // 19 reserved for POLAR_LEARN.

    // @Param: THML_ARSPD
    // @DisplayName: Specific setting for airspeed when soaring in THERMAL mode.
    // @Description: If non-zero this airspeed will be used when thermalling. A value of 0 will use AIRSPEED_CRUISE.
    // @Range: 0 50
    // @User: Advanced
    AP_GROUPINFO("THML_ARSPD", 20, SoaringController, soar_thermal_airspeed, 0),

    // @Param: CRSE_ARSPD
    // @DisplayName: Specific setting for airspeed when soaring in AUTO mode.
    // @Description: If non-zero this airspeed will be used when cruising between thermals in AUTO. If set to -1, airspeed will be selected based on speed-to-fly theory. If set to 0, then AIRSPEED_CRUISE will be used while cruising between thermals.
    // @Range: -1 50
    // @User: Advanced
    AP_GROUPINFO("CRSE_ARSPD", 21, SoaringController, soar_cruise_airspeed, 0),

    // @Param: THML_FLAP
    // @DisplayName: Flap percent to be used during thermalling flight.
    // @Description: This sets the flap when in LOITER with soaring active. Overrides the usual auto flap behaviour.
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("THML_FLAP", 22, SoaringController, soar_thermal_flap, 0),

    AP_GROUPEND
};

SoaringController::SoaringController(AP_TECS &tecs, const AP_FixedWing &parms) :
    _ekf(*new (_ekf_storage) ExtendedKalmanFilter{}),
    _tecs(tecs),
    _vario(parms,_polarParams),
    _speedToFly(_polarParams),
    _aparm(parms),
    _throttle_suppressed(true)
{
    // The storage holds the thermal filter exactly, which is never destroyed.
    static_assert(sizeof(_ekf_storage) == sizeof(ExtendedKalmanFilter));
    static_assert(alignof(float) >= alignof(ExtendedKalmanFilter));
    static_assert(std::is_trivially_destructible_v<ExtendedKalmanFilter>);

    AP_Param::setup_object_defaults(this, var_info);
}

void SoaringController::get_target(Location &wp)
{
    wp = AP::ahrs().get_home();
    wp.offset(_position_x_filter.get(), _position_y_filter.get());
}

bool SoaringController::suppress_throttle()
{
    float alt = _vario.alt;

    if (_throttle_suppressed && (alt < alt_min)) {
        // Time to throttle up
        set_throttle_suppressed(false);
    } else if ((!_throttle_suppressed) && (alt > alt_cutoff)) {
        // Start glide
        set_throttle_suppressed(true);

        // Zero the pitch integrator - the nose is currently raised to climb, we need to go back to glide.
        _tecs.reset_pitch_I();

        _cruise_start_time_us = AP_HAL::micros64();

        // Reset the filtered vario rate - it is currently elevated due to the climb rate and would otherwise take a while to fall again,
        // leading to false positives.
        _vario.reset_trigger_filter(0.0f);
    }

    return _throttle_suppressed;
}

bool SoaringController::check_thermal_criteria()
{
    return (_last_update_status == ActiveStatus::AUTO_MODE_CHANGE
            && ((AP_HAL::micros64() - _cruise_start_time_us) > ((unsigned)min_cruise_s * 1e6))
            && (_vario.get_trigger_value() - _vario.get_exp_thermalling_sink()) > thermal_vspeed
            && _vario.alt < alt_max
            && _vario.alt > alt_min);
}


SoaringController::LoiterStatus SoaringController::check_cruise_criteria(Vector2f prev_wp, Vector2f next_wp)
{
    // Check conditions for re-entering cruise. Note that the aircraft needs to also be aligned with the appropriate
    // heading before some of these conditions will actually trigger.
    // The GCS messages are emitted in mode_thermal.cpp. Update these if the logic here is changed.

    if (_last_update_status == ActiveStatus::SOARING_DISABLED) {
        return LoiterStatus::DISABLED;
    }

    LoiterStatus result = LoiterStatus::GOOD_TO_KEEP_LOITERING;
    const float alt = _vario.alt;

    if (_exit_commanded) {
        result = LoiterStatus::EXIT_COMMANDED;
    } else if (alt > alt_max) {
        result = LoiterStatus::ALT_TOO_HIGH;
    } else if (alt < alt_min) {
        result = LoiterStatus::ALT_TOO_LOW;
    } else if ((AP_HAL::micros64() - _thermal_start_time_us) > ((unsigned)min_thermal_s * 1e6)) {
        const float mcCreadyAlt = McCready(alt);
        if (_thermalability < mcCreadyAlt) {
            result = LoiterStatus::THERMAL_WEAK;
        } else if (alt < (-_thermal_start_pos.z) || _vario.get_filtered_climb() < 0.0) {
            result = LoiterStatus::ALT_LOST;
        } else if (check_drift(prev_wp, next_wp)) {
            result = LoiterStatus::DRIFT_EXCEEDED;
        }
    }

    return result;
}

void SoaringController::init_thermalling()
{
    // Calc filter matrices - so that changes to parameters can be updated by switching in and out of thermal mode
    float r      = powf(thermal_r,  2); // Measurement noise
    float cov_q1 = powf(thermal_q1, 2); // Process noise for strength
    float cov_q2 = powf(thermal_q2, 2); // Process noise for position and radius

    const ExtendedKalmanFilter::Covariance q{diagonal_covariance({cov_q1, cov_q2, cov_q2, cov_q2})};

    const ExtendedKalmanFilter::Covariance p{diagonal_covariance({INITIAL_STRENGTH_COVARIANCE,
                                                                  INITIAL_RADIUS_COVARIANCE,
                                                                  INITIAL_POSITION_COVARIANCE,
                                                                  INITIAL_POSITION_COVARIANCE})};

    const ExtendedKalmanFilter::Variance output_variance{r * ExtendedKalmanFilter::Variance::reference};

    Vector3f position;

    const AP_AHRS &_ahrs = AP::ahrs();
    if (!_ahrs.get_relative_position_NED_home(position)) {
        return;
    }

    // New state vector filter will be reset. Thermal location is placed in front of a/c
    const ExtendedKalmanFilter::State xr{_vario.get_trigger_value() * (metre / second),
                                         INITIAL_THERMAL_RADIUS * metre,
                                         (position.x + thermal_distance_ahead * cosf(_ahrs.get_yaw_rad())) * metre,
                                         (position.y + thermal_distance_ahead * sinf(_ahrs.get_yaw_rad())) * metre};

    // Also reset covariance matrix p so filter is not affected by previous data
    _ekf.reset(xr, p, q, output_variance);

    _prev_update_time = AP_HAL::micros64();
    _thermal_start_time_us = AP_HAL::micros64();
    _thermal_start_pos = position;

    _vario.reset_climb_filter(0.0);

    _position_x_filter.reset(thermal_x());
    _position_y_filter.reset(thermal_y());

    _exit_commanded = false;
}

void SoaringController::init_cruising()
{
    if (_last_update_status >= ActiveStatus::MANUAL_MODE_CHANGE) {
        _cruise_start_time_us = AP_HAL::micros64();
        // Start glide. Will be updated on the next loop.
        set_throttle_suppressed(true);
    }
}

void SoaringController::update_thermalling()
{
    float deltaT = (AP_HAL::micros64() - _prev_update_time) * 1e-6;

    Vector3f current_position;

    const AP_AHRS &_ahrs = AP::ahrs();
    if (!_ahrs.get_relative_position_NED_home(current_position)) {
        return;
    }

    Vector3f wind;
    // use the estimate even if it is not marked valid, to preserve
    // existing behaviour
    IGNORE_RETURN(_ahrs.get_wind(wind));

    Vector3f wind_drift = wind*deltaT*_vario.get_filtered_climb()/thermal_strength();

    // update the filter
    _ekf.update(_vario.reading * (metre / second), current_position.x * metre, current_position.y * metre, wind_drift.x * metre, wind_drift.y * metre);

    
    _thermalability = (thermal_strength()*expf(-powf(get_thermalling_radius()/thermal_radius(), 2))) - _vario.get_exp_thermalling_sink();

    _prev_update_time = AP_HAL::micros64();

    // Compute smoothed estimate of position
    _position_x_filter.set_cutoff_frequency(1/(3*_vario.tau));
    _position_y_filter.set_cutoff_frequency(1/(3*_vario.tau));

    _position_x_filter.apply(thermal_x(), deltaT);
    _position_y_filter.apply(thermal_y(), deltaT);

#if HAL_LOGGING_ENABLED
    // write log - save the data.
    // @LoggerMessage: SOAR
    // @Vehicles: Plane
    // @Description: Logged data from soaring feature
    // @URL: https://ardupilot.org/plane/docs/soaring.html
    // @Field: TimeUS: microseconds since system startup
    // @Field: nettorate: Estimate of vertical speed of surrounding airmass
    // @Field: x0: Thermal strength estimate
    // @Field: x1: Thermal radius estimate
    // @Field: x2: Thermal position estimate north from home
    // @Field: x3: Thermal position estimate east from home
    // @Field: north: Aircraft position north from home
    // @Field: east: Aircraft position east from home
    // @Field: alt: Aircraft altitude
    // @Field: dx_w: Wind speed north
    // @Field: dy_w: Wind speed east
    // @Field: th: Estimate of achievable climbrate in thermal
    AP::logger().WriteStreaming("SOAR", "TimeUS,nettorate,x0,x1,x2,x3,north,east,alt,dx_w,dy_w,th", "Qfffffffffff",
                                           AP_HAL::micros64(),
                                           (double)_vario.reading,
                                           (double)thermal_strength(),
                                           (double)thermal_radius(),
                                           (double)thermal_x(),
                                           (double)thermal_y(),
                                           current_position.x,
                                           current_position.y,
                                           (double)_vario.alt,
                                           (double)wind_drift.x,
                                           (double)wind_drift.y,
                                           (double)_thermalability);
#if HAL_SOARING_NVF_EKF_ENABLED
    auto const now_ms = AP_HAL::millis();
    if (now_ms - _prev_nvf_pub_time_ms > NVF_PUBLISHER_DELAY_MS) {
        gcs().send_named_float("SOAREKFX0", thermal_strength());
        gcs().send_named_float("SOAREKFX1", thermal_radius());
        gcs().send_named_float("SOAREKFX2", thermal_x());
        gcs().send_named_float("SOAREKFX3", thermal_y());
        _prev_nvf_pub_time_ms = now_ms;
    }

#endif // HAL_SOARING_NVF_EKF_ENABLED
#endif
}

void SoaringController::update_cruising()
{
    // Calculate the optimal airspeed for the current conditions of wind along current direction,
    // expected lift in next thermal and filtered sink rate.

    Vector3f wind;
    // use the estimate even if it is not marked valid, to preserve
    // existing behaviour
    IGNORE_RETURN(AP::ahrs().get_wind(wind));
    Vector3f wind_bf = AP::ahrs().earth_to_body(wind);

    const float wx = wind_bf.x;

    const float wz = _vario.get_stf_value();

    // Constraints on the airspeed calculation.
    const float CLmin = _polarParams.K/(_aparm.airspeed_max*_aparm.airspeed_max);
    const float CLmax = _polarParams.K/(_aparm.airspeed_min*_aparm.airspeed_min);

    // Update the calculation.
    _speedToFly.update(wx, wz, thermal_vspeed, CLmin, CLmax);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: SORC
    // @Vehicles: Plane
    // @Description: Soaring Cruise-phase data
    // @URL: https://ardupilot.org/plane/docs/soaring.html
    // @Field: TimeUS: Time since system startup
    // @Field: wx: body-frame wind estimate, x-axis
    // @Field: wz: body-frame wind estimate, z-axis
    // @Field: wexp: estimated thermal vertical speed
    // @Field: CLmin: expected climb-rate lower-limit
    // @Field: CLmax: expected climb-rate upper-limit
    // @Field: Vopt: calculated optimal speed to fly
    AP::logger().WriteStreaming("SORC", "TimeUS,wx,wz,wexp,CLmin,CLmax,Vopt", "Qffffff",
                                       AP_HAL::micros64(),
                                       (double)wx,
                                       (double)wz,
                                       (double)thermal_vspeed,
                                       (double)CLmin,
                                       (double)CLmax,
                                       (double)_speedToFly.speed_to_fly());
#endif
}

void SoaringController::update_vario()
{
    _vario.update(thermal_bank);
}


float SoaringController::McCready(float alt)
{
    // A method shell to be filled in later
    return thermal_vspeed;
}

SoaringController::ActiveStatus SoaringController::active_state(bool override_disable) const
{
    if (override_disable || !soar_active) {
        return ActiveStatus::SOARING_DISABLED;
    }

    return _pilot_desired_state;
}

void SoaringController::update_active_state(bool override_disable)
{
    ActiveStatus status = active_state(override_disable);
    bool state_changed = !(status == _last_update_status);

    if (state_changed) {
        switch (status) {
            case ActiveStatus::SOARING_DISABLED:
                // It's not enabled, but was enabled on the last loop.
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Soaring: Disabled.");
                set_throttle_suppressed(false);
                break;
            case ActiveStatus::MANUAL_MODE_CHANGE:
                // It's enabled, but wasn't on the last loop.
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Soaring: Enabled, manual mode changes.");
                break;
            case ActiveStatus::AUTO_MODE_CHANGE:
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Soaring: Enabled, automatic mode changes.");
                break;
        }

        if (_last_update_status == ActiveStatus::SOARING_DISABLED) {
            // We have switched from disabled into an active mode, start cruising.
            init_cruising();
        } else if (status != ActiveStatus::SOARING_DISABLED) {
            // We switched between active modes. If we're in THERMAL this means we should exit gracefully.
            // This has no effect if we're cruising as it is reset on thermal entry.
            _exit_commanded = true;
        }
    }

    _last_update_status = status;
}


void SoaringController::set_throttle_suppressed(bool suppressed)
{
    _throttle_suppressed = suppressed;

    // Let the TECS know.
    _tecs.set_gliding_requested_flag(suppressed);
}

bool SoaringController::check_drift(Vector2f prev_wp, Vector2f next_wp)
{
    // Check for -1 (disabled)
    if (max_drift<0) {
        return false;
    }

    // Check against the estimated thermal.
    Vector2f position(thermal_x(), thermal_y());

    Vector2f start_pos(_thermal_start_pos.x, _thermal_start_pos.y);

    Vector2f mission_leg = next_wp - prev_wp;

    if (prev_wp.is_zero() || mission_leg.length() < 0.1) {
        // Simple check of distance from initial start point.
        return (position - start_pos).length() > max_drift;
    } else {
        // Regard the effective start point as projected onto mission leg.
        // Calculate drift parallel and perpendicular to mission leg.
        // Drift parallel and in direction of mission leg is acceptable.
        Vector2f effective_start, vec1, vec2;

        // Calculate effective start point (on mission leg).
        vec1 = (start_pos - prev_wp).projected(mission_leg);
        effective_start = prev_wp + vec1;

        // Calculate parallel and perpendicular offsets.
        vec2 = position - effective_start;

        float parallel      = vec2 * mission_leg.normalized();
        float perpendicular = (vec2 - mission_leg.normalized()*parallel).length();

       // Check if we've drifted beyond the next wp.
        if (parallel>(next_wp - effective_start).length()) {
            return true;
        }

        // Check if we've drifted too far laterally or backwards. We don't count positive parallel offsets
        // as these are favourable (towards next wp)
        parallel = parallel>0 ? 0 : parallel;

        return (powf(parallel,2)+powf(perpendicular,2)) > powf(max_drift,2);
    }
}

float SoaringController::get_thermalling_radius() const
{
    // Thermalling radius is controlled by parameter SOAR_THML_BANK and true target airspeed.
    const float target_aspd = _tecs.get_target_airspeed() * AP::ahrs().get_EAS2TAS();
    const float radius = (target_aspd*target_aspd) / (GRAVITY_MSS * tanf(thermal_bank*DEG_TO_RAD));

    return radius;
}

float SoaringController::get_thermalling_target_airspeed()
{
    return soar_thermal_airspeed;
}

float SoaringController::get_cruising_target_airspeed()
{
    if (soar_cruise_airspeed<0) {
        return _speedToFly.speed_to_fly();
    }
    return soar_cruise_airspeed;
}

#endif // HAL_SOARING_ENABLED
