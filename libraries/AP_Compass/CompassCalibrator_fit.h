#pragma once

// The Levenberg-Marquardt steps of the compass calibration's sphere and
// ellipsoid fits, typed: separate from CompassCalibrator.cpp so that the unit
// tests reach them.

#include "AP_Compass_config.h"

#if COMPASS_CAL_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#include "CompassCalibrator.h"
#include <AP_Math/typed_linear_algebra.h>

#include <cmath>
#include <utility>

namespace compass_calibrator_fit {

namespace tla = typed_linear_algebra;

// The magnetic field unit of the samples and offsets.
inline constexpr struct milligauss final
    : mp_units::named_unit<"mG", mp_units::mag_ratio<1, 10000000> * mp_units::si::tesla> {} milligauss;

using Field = tla::Quantity<milligauss>;
using Ratio = tla::Quantity<mp_units::one>;

// The sphere fit's parameters, the radius and three offsets [mGauss], and
// its residual's Jacobian with respect to them [1]; the ellipsoid fit's,
// three offsets [mGauss] and six soft-iron factors [1], and its Jacobian, per
// offset [1] and per factor [mGauss].
struct sphere {
    static constexpr size_t count{COMPASS_CAL_NUM_SPHERE_PARAMS};
    using Parameters = tla::Vector<Field, Field, Field, Field>;
    using Jacobian = tla::Vector<Ratio, Ratio, Ratio, Ratio>;
};

struct ellipsoid {
    static constexpr size_t count{COMPASS_CAL_NUM_ELLIPSOID_PARAMS};
    using Parameters = tla::Vector<Field, Field, Field, Ratio, Ratio, Ratio, Ratio, Ratio, Ratio>;
    using Jacobian = tla::Vector<Ratio, Ratio, Ratio, Field, Field, Field, Field, Field, Field>;
};

template <typename Fit>
using Normal = tla::Matrix<typename Fit::Jacobian, typename Fit::Jacobian>;

template <typename Fit>
using Gradient = decltype(typename Fit::Jacobian{} * Field{});

// The typed Jacobian of the numerical values `values`, each in its element's unit.
template <typename Fit>
typename Fit::Jacobian typed_jacobian(const float (&values)[Fit::count])
{
    typename Fit::Jacobian result;
    [&]<size_t... Index>(std::index_sequence<Index...>) {
        (result.template at<Index>(values[Index] *
             std::remove_cvref_t<decltype(result.template at<Index>())>::reference), ...);
    }(std::make_index_sequence<Fit::count>{});
    return result;
}

// The normal equations JᵀJ and JᵀF over the samples, accumulated element by
// element, without temporaries: the embedded stack frame limit.
template <typename Fit, typename JacobianOf, typename ResidualOf>
NOINLINE void accumulate_normal_equations(Normal<Fit> &JTJ, Gradient<Fit> &JTFI, uint16_t sample_count,
                                          const JacobianOf &jacobian_of, const ResidualOf &residual_of)
{
    constexpr size_t N{Fit::count};
    for (uint16_t k = 0; k < sample_count; k++) {
        const typename Fit::Jacobian jacob{jacobian_of(k)};
        const Field residual{residual_of(k) * milligauss};
        [&]<size_t... Index>(std::index_sequence<Index...>) {
            ((JTJ.template at<Index / N, Index % N>(
                  JTJ.template at<Index / N, Index % N>() +
                  jacob.template at<Index / N>() * jacob.template at<Index % N>())), ...);
        }(std::make_index_sequence<N * N>{});
        [&]<size_t... Index>(std::index_sequence<Index...>) {
            ((JTFI.template at<Index>(JTFI.template at<Index>() + jacob.template at<Index>() * residual)), ...);
        }(std::make_index_sequence<N>{});
    }
}

// The Levenberg-Marquardt step Δ of (JᵀJ + λ) Δ = JᵀF, solved by elimination
// with partial pivoting, as numerical values in the parameters' units. The
// damping λ is added to each diagonal element in that element's own unit, as
// before: for the ellipsoid fit, 1 for the offsets and mGauss² for the
// factors, so that its weight depends on the unit of the field (Q11).
template <typename Fit>
NOINLINE void damped_step(Normal<Fit> JTJ, const Gradient<Fit> &JTFI, float lambda, float (&values)[Fit::count])
{
    [&]<size_t... Index>(std::index_sequence<Index...>) {
        ((JTJ.template at<Index, Index>(JTJ.template at<Index, Index>() +
              lambda * std::remove_cvref_t<decltype(JTJ.template at<Index, Index>())>::reference)), ...);
    }(std::make_index_sequence<Fit::count>{});
    // ((JᵀJ)⁻¹ JᵀF)ᵀ = (JᵀF)ᵀ (JᵀJ)⁻¹, JᵀJ symmetric
    const typename Fit::Parameters step{fcarouge::transposed(fcarouge::transposed(JTFI) / JTJ)};
    [&]<size_t... Index>(std::index_sequence<Index...>) {
        ((values[Index] = step.template at<Index>().numerical_value_in(
              std::remove_cvref_t<decltype(step.template at<Index>())>::unit)), ...);
    }(std::make_index_sequence<Fit::count>{});
}

template <size_t Count>
bool all_finite(const float (&values)[Count])
{
    for (size_t i = 0; i < Count; i++) {
        if (!std::isfinite(values[i])) {
            return false;
        }
    }
    return true;
}

} // namespace compass_calibrator_fit

#endif  // COMPASS_CAL_ENABLED
