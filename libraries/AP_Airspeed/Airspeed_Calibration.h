/*
  In flight estimation of the airspeed ratio from the ground velocity.

  This header brings the unit types: include it only from the sources that use
  the calibration, AP_Airspeed holds it by pointer.
 */
#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_AUTOCAL_ENABLE

#include <AP_Common/AP_Common.h>
#include <AP_LinearAlgebra/AP_LinearAlgebra_Quantity.h>

/*
  The airspeed is sqrt(ratio * differential pressure). An extended Kalman filter
  estimates the horizontal wind and the scale 1/sqrt(ratio) by observing the
  square root of the differential pressure, corrected from equivalent to true
  airspeed, against the airspeed implied by the ground velocity and the wind.
 */
class Airspeed_Calibration {
public:
    template <auto Reference>
    using Quantity = AP_LinearAlgebra::Units::Quantity<Reference>;
    using MetresPerSecond = AP_LinearAlgebra::Units::MetresPerSecond;
    using SquareMetresPerSecondSquared = AP_LinearAlgebra::Units::SquareMetresPerSecondSquared;

// unit functions named like the maths functions ArduPilot replaces by macros
#include <AP_LinearAlgebra/AP_LinearAlgebra_MathsMacrosPush.h>
    // the square root of a pressure, the observation
    using RootPressure = Quantity<sqrt(mp_units::si::pascal)>;
    // the airspeed ratio, airspeed squared per differential pressure
    using Ratio = Quantity<square(mp_units::si::metre / mp_units::si::second) / mp_units::si::pascal>;
    // the scale state, 1/sqrt(ratio), and its variance
    using Scale = Quantity<sqrt(mp_units::si::pascal) / (mp_units::si::metre / mp_units::si::second)>;
    using ScaleVariance = Quantity<mp_units::si::pascal / square(mp_units::si::metre / mp_units::si::second)>;

#include <AP_LinearAlgebra/AP_LinearAlgebra_MathsMacrosPop.h>

    // the estimate: horizontal wind and scale
    struct Estimate {
        MetresPerSecond wind_north;
        MetresPerSecond wind_east;
        Scale scale;
    };
    struct EstimateVariances {
        SquareMetresPerSecondSquared wind_north;
        SquareMetresPerSecondSquared wind_east;
        ScaleVariance scale;
    };

    Airspeed_Calibration();
    ~Airspeed_Calibration();

    CLASS_NO_COPY(Airspeed_Calibration);

    // initialise the calibration
    void init(Ratio initial_ratio);

    // fuse one observation, the square root of the differential pressure
    // corrected to true airspeed, with the ground velocity (north, east, down);
    // the wind estimate is limited to max_wind; returns the scale estimate
    Scale update(RootPressure root_pressure, MetresPerSecond vg_north, MetresPerSecond vg_east,
                 MetresPerSecond vg_down, MetresPerSecond max_wind);

    // set the scale estimate, 1/sqrt(ratio)
    void set_scale(Scale scale);

    // the scale state of a ratio, 1/sqrt(ratio), and the ratio of a scale
    static Scale scale_of(Ratio ratio);
    static Ratio ratio_of(Scale scale);

    Estimate get_state() const;
    EstimateVariances get_variances() const;

private:
    // the typed Kalman filter, defined in Airspeed_Calibration.cpp to keep its
    // expensive headers out of this one, and constructed in this storage on
    // first use, not with this object: no heap, and not during static
    // initialization, whose order the filter's default values depend on
    struct Filter;
    static constexpr size_t filter_size = 184;
    alignas(8) uint8_t _storage[filter_size];
    Filter *_filter = nullptr;
    Filter &filter();
};

#endif // AP_AIRSPEED_AUTOCAL_ENABLE
