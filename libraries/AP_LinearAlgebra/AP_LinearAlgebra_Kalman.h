/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  FrancoisCarouge Kalman filters over the typed matrices of
  AP_LinearAlgebra.

  These headers are expensive to compile: include this file only from the
  source file that implements a filter, never from a header.
 */
#pragma once

#include "AP_LinearAlgebra_MathsMacrosPush.h"
#include <fcarouge/kalman.hpp>
#include "AP_LinearAlgebra_MathsMacrosPop.h"

#include "AP_LinearAlgebra.h"
#include "AP_LinearAlgebra_Units.h"

namespace fcarouge::kalman_filter::internal {

// typed matrices evaluate, transpose, and have identity and zero values
// through their backend matrix
template <typename Matrix, typename RowIndexes, typename ColumnIndexes>
struct evaluates<typed_matrix<Matrix, RowIndexes, ColumnIndexes>> {
    [[nodiscard]] static constexpr auto operator()()
    -> typed_matrix<evaluate<Matrix>, RowIndexes, ColumnIndexes>;
};

template <typename Matrix, typename RowIndexes, typename ColumnIndexes>
struct transposes<typed_matrix<Matrix, RowIndexes, ColumnIndexes>> {
    [[nodiscard]] static constexpr auto
    operator()(const typed_matrix<Matrix, RowIndexes, ColumnIndexes> &value)
    {
        return typed_matrix<evaluate<transpose<Matrix>>, ColumnIndexes, RowIndexes>{value.data().transpose()};
    }
};

template <typename Matrix, typename RowIndexes, typename ColumnIndexes>
inline typed_matrix<decltype(one<Matrix>), RowIndexes, ColumnIndexes>
one<typed_matrix<Matrix, RowIndexes, ColumnIndexes>> {one<Matrix>};

template <typename Matrix, typename RowIndexes, typename ColumnIndexes>
inline typed_matrix<decltype(zero<Matrix>), RowIndexes, ColumnIndexes>
zero<typed_matrix<Matrix, RowIndexes, ColumnIndexes>> {zero<Matrix>};

// a 1x1 typed product collapses to its element, an mp-units quantity
template <auto Reference, typename Representation>
inline mp_units::quantity<Reference, Representation>
one<mp_units::quantity<Reference, Representation>> {Representation{1} * Reference};

template <auto Reference, typename Representation>
inline mp_units::quantity<Reference, Representation>
zero<mp_units::quantity<Reference, Representation>> {Representation{0} * Reference};

} // namespace fcarouge::kalman_filter::internal

namespace AP_LinearAlgebra {

// the type of a * transpose(b), for example a covariance matrix
template <typename A, typename B>
using OuterProduct = fcarouge::kalman_filter::internal::evaluate<
    fcarouge::kalman_filter::internal::product<A, fcarouge::kalman_filter::internal::evaluate<fcarouge::kalman_filter::internal::transpose<B>>>>;

// the type of the matrix m with m * b of the type a, for example an output model
template <typename A, typename B>
using Quotient = fcarouge::kalman_filter::internal::evaluate<fcarouge::kalman_filter::internal::quotient<A, B>>;

} // namespace AP_LinearAlgebra
