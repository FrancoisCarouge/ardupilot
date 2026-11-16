/*
  polynomial fitting class, originally written by Siddharth Bharat Purohit
  re-worked for ArduPilot by Andrew Tridgell
*/

#include "polyfit.h"
#include <AP_Math/typed_linear_algebra.h>

#include <cmath>
#include <type_traits>
#include <utility>
#include "AP_Math.h"
#include "vector3.h"

template <uint8_t order, typename xtype, typename vtype>
void PolyFit<order,xtype,vtype>::update(xtype x, vtype y)
{
    xtype temp = 1;

    for (int8_t i = 2*(order-1); i >= 0; i--) {
        int8_t k = (i<order)?0:i - order + 1;
        for (int8_t j = i - k; j >= k; j--) {
            mat[j][i-j] += temp;
        }
        temp *= x;
    }

    temp = 1;
    for (int8_t i = order-1; i >= 0; i--) {
        vec[i] += y * temp;
        temp *= x;
    }
}

namespace {

namespace tla = typed_linear_algebra;

// The fitted output, an accelerometer's or a gyroscope's, is of its own
// abstract dimension: the fit checks the powers of the temperature [K]
// against it, whatever it is.
inline constexpr struct dim_output final : mp_units::base_dimension<"Y"> {} dim_output;
QUANTITY_SPEC(fitted_output, dim_output);
inline constexpr struct output_unit final : mp_units::named_unit<"y", mp_units::kind_of<fitted_output>> {} output_unit;

// The fit is of a temperature difference from the calibration's midpoint: a
// kelvin without the absolute zero origin, which powers do not allow. pow is a
// hidden friend of the unit, found by argument-dependent lookup only.
inline constexpr struct kelvin_difference final
    : mp_units::named_unit<"K", mp_units::kind_of<mp_units::isq::thermodynamic_temperature>> {} kelvin_difference;

template <size_t Power>
using TemperaturePower = tla::QuantityOf<pow<Power>(kelvin_difference), double>;
template <size_t Power>
using OutputPerTemperaturePower = tla::QuantityOf<output_unit / pow<Power>(kelvin_difference), double>;
template <size_t Power>
using OutputTimesTemperaturePower = tla::QuantityOf<output_unit * pow<Power>(kelvin_difference), double>;
using Ratio = tla::QuantityOf<mp_units::one, double>;

// The normal equations of the fit M C = V, highest power first: M of the
// powers of the temperature, V of the output's moments, C the coefficients;
// a column per axis.
template <size_t Order, typename = std::make_index_sequence<Order>>
struct normal_equations;

template <size_t Order, size_t... Index>
struct normal_equations<Order, std::index_sequence<Index...>> {
    using Powers = tla::VectorOf<double, TemperaturePower<Order - 1 - Index>...>;
    using Moments = tla::VectorOf<double, OutputTimesTemperaturePower<Order - 1 - Index>...>;
    using Coefficients = tla::VectorOf<double, OutputPerTemperaturePower<Order - 1 - Index>...>;
    using Axes = tla::VectorOf<double, Ratio, Ratio, Ratio>;
    using Matrix = tla::MatrixOf<double, Powers, Powers>;
    using Rhs = tla::MatrixOf<double, Moments, Axes>;
    using Solution = tla::MatrixOf<double, Coefficients, Axes>;
};

template <typename Typed, size_t Row, size_t Column>
using ElementOf = std::remove_cvref_t<decltype(std::declval<const Typed &>().template at<Row, Column>())>;

} // namespace

template <uint8_t order, typename xtype, typename vtype>
bool PolyFit<order,xtype,vtype>::get_polynomial(vtype res[order]) const
{
    using Equations = normal_equations<order>;
    typename Equations::Matrix M;
    typename Equations::Rhs V;
    [&]<size_t... Index>(std::index_sequence<Index...>) {
        ((M.template at<Index / order, Index % order>(
              double(mat[Index / order][Index % order]) *
              ElementOf<typename Equations::Matrix, Index / order, Index % order>::reference)), ...);
    }(std::make_index_sequence<order * order>{});
    [&]<size_t... Index>(std::index_sequence<Index...>) {
        ((V.template at<Index / 3, Index % 3>(
              double(vec[Index / 3][Index % 3]) *
              ElementOf<typename Equations::Rhs, Index / 3, Index % 3>::reference)), ...);
    }(std::make_index_sequence<order * 3>{});

    // C = M⁻¹ V by elimination with partial pivoting, in double precision:
    // Cᵀ = Vᵀ M⁻¹, M symmetric
    const typename Equations::Solution C{fcarouge::transposed(fcarouge::transposed(V) / M)};

    double values[order][3];
    bool finite = true;
    [&]<size_t... Index>(std::index_sequence<Index...>) {
        ((values[Index / 3][Index % 3] = C.template at<Index / 3, Index % 3>().numerical_value_in(
              ElementOf<typename Equations::Solution, Index / 3, Index % 3>::unit)), ...);
    }(std::make_index_sequence<order * 3>{});
    for (uint8_t j = 0; j < order; j++) {
        for (uint8_t axis = 0; axis < 3; axis++) {
            finite = finite && std::isfinite(values[j][axis]);
        }
    }
    if (!finite) {
        return false;
    }
    for (uint8_t j = 0; j < order; j++) {
        res[j].x = values[j][0];
        res[j].y = values[j][1];
        res[j].z = values[j][2];
    }
    return true;
}

// instantiate for order 4 double with Vector3f
template class PolyFit<4, double, Vector3f>;
