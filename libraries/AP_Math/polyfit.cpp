/*
  polynomial fitting class, originally written by Siddharth Bharat Purohit
  re-worked for ArduPilot by Andrew Tridgell
*/

#include "polyfit.h"
#include "AP_Math.h"
#include "vector3.h"
#include <AP_LinearAlgebra/AP_LinearAlgebra.h>
#include <AP_LinearAlgebra/AP_LinearAlgebra_Units.h>

#include <type_traits>
#include <utility>

namespace {

/*
  the fit knows neither the unit of x nor that of y: each has a dimension
  of its own here, so that the powers of x in the normal equations, and the
  units y/x^k of the coefficients, are checked at compile time
 */
inline constexpr struct dim_abscissa final : mp_units::base_dimension<"X"> {} dim_abscissa;
inline constexpr struct dim_ordinate final : mp_units::base_dimension<"Y"> {} dim_ordinate;
QUANTITY_SPEC(abscissa, dim_abscissa);
QUANTITY_SPEC(ordinate, dim_ordinate);
inline constexpr struct x_unit final : mp_units::named_unit<"x", mp_units::kind_of<abscissa>> {} x_unit;
inline constexpr struct y_unit final : mp_units::named_unit<"y", mp_units::kind_of<ordinate>> {} y_unit;

template <auto Reference>
using Quantity = mp_units::quantity<Reference, double>;

template <typename Indexes>
struct Polynomial;

/*
  the terms x^(order-1), ..., x, 1 of a polynomial, highest power first as
  get_polynomial() returns the coefficients; the normal equations
  sum(t * transpose(t)) * c = sum(t * transpose(y)) of the least squares fit
  of the coefficients c
 */
template <size_t... i>
struct Polynomial<std::index_sequence<i...>> {
    static constexpr size_t order = sizeof...(i);

    template <size_t k>
    // pow() on units is a hidden friend, found by argument dependent lookup
    static constexpr auto power = pow<order - 1 - k>(x_unit);

    using Terms = AP_LinearAlgebra::ColumnVector<double, Quantity<power<i>>...>;
    using Sample = AP_LinearAlgebra::RowVector<double, Quantity<y_unit>, Quantity<y_unit>, Quantity<y_unit>>;
    using Moments = decltype(Terms{} * fcarouge::transposed(Terms{}));
    using Sums = decltype(Terms{} * Sample{});
    // Moments being symmetric, solve transpose(c) * Moments = transpose(Sums)
    using TransposedCoefficients = decltype(fcarouge::transposed(Sums{}) / Moments{});
    using Coefficients = decltype(fcarouge::transposed(TransposedCoefficients{}));

    static Terms terms(double x)
    {
        double t[order];
        t[order - 1] = 1;
        for (size_t k = order - 1; k-- > 0;) {
            t[k] = t[k + 1] * x;
        }
        return Terms{(t[i] * power<i>)...};
    }

    static Sample sample(const Vector3f &y)
    {
        return Sample{double(y.x) * y_unit, double(y.y) * y_unit, double(y.z) * y_unit};
    }

    // the coefficients of x^(order-1-k) for each axis, in y/x^(order-1-k)
    template <size_t k>
    static Vector3d coefficient(const Coefficients &c)
    {
        constexpr auto unit = y_unit / power<k>;
        return Vector3d(c.template at<k, 0>().numerical_value_in(unit),
                        c.template at<k, 1>().numerical_value_in(unit),
                        c.template at<k, 2>().numerical_value_in(unit));
    }
};

// the normal equations are stored untyped in the class, whose header does
// not include the typed linear algebra; they are only ever written from
// their typed values
template <typename Typed, size_t rows, size_t columns, typename Raw>
Typed load(const Raw (&raw)[rows][columns])
{
    static_assert(Typed::rows == rows && Typed::columns == columns);
    Typed m{};
    for (size_t r = 0; r < rows; r++) {
        for (size_t c = 0; c < columns; c++) {
            m.data()(r, c) = raw[r][c];
        }
    }
    return m;
}

template <typename Typed, size_t rows, size_t columns, typename Raw>
void store(const Typed &m, Raw (&raw)[rows][columns])
{
    static_assert(Typed::rows == rows && Typed::columns == columns);
    for (size_t r = 0; r < rows; r++) {
        for (size_t c = 0; c < columns; c++) {
            raw[r][c] = m.data()(r, c);
        }
    }
}

template <typename Typed, size_t rows>
Typed load(const Vector3f (&raw)[rows])
{
    static_assert(Typed::rows == rows && Typed::columns == 3);
    Typed m{};
    for (size_t r = 0; r < rows; r++) {
        for (size_t c = 0; c < 3; c++) {
            m.data()(r, c) = raw[r][c];
        }
    }
    return m;
}

template <typename Typed, size_t rows>
void store(const Typed &m, Vector3f (&raw)[rows])
{
    static_assert(Typed::rows == rows && Typed::columns == 3);
    for (size_t r = 0; r < rows; r++) {
        for (size_t c = 0; c < 3; c++) {
            raw[r][c] = m.data()(r, c);
        }
    }
}

} // namespace

template <uint8_t order, typename xtype, typename vtype>
void PolyFit<order,xtype,vtype>::update(xtype x, vtype y)
{
    using P = Polynomial<std::make_index_sequence<order>>;

    const typename P::Terms t = P::terms(x);
    store(load<typename P::Moments>(mat) + t * fcarouge::transposed(t), mat);
    store(load<typename P::Sums>(vec) + t * P::sample(y), vec);
}

template <uint8_t order, typename xtype, typename vtype>
bool PolyFit<order,xtype,vtype>::get_polynomial(vtype res[order]) const
{
    using P = Polynomial<std::make_index_sequence<order>>;

    // the solution is in double precision to get good accuracy; the
    // moments, sums of squares, are positive definite unless too few
    // distinct x were sampled
    typename P::TransposedCoefficients ct;
    if (!AP_LinearAlgebra::divide_positive_definite(ct, fcarouge::transposed(load<typename P::Sums>(vec)),
                                                    load<typename P::Moments>(mat))) {
        return false;
    }
    const typename P::Coefficients c = fcarouge::transposed(ct);
    [&]<size_t... k>(std::index_sequence<k...>) {
        ((res[k] = P::template coefficient<k>(c).tofloat()), ...);
    }(std::make_index_sequence<order>{});
    return true;
}

// instantiate for order 4 double with Vector3f
template class PolyFit<4, double, Vector3f>;
