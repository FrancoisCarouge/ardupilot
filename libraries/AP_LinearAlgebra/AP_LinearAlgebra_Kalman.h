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

// Kalman's integrations: typed matrices evaluate, transpose, and have identity
// and zero values through their backend matrix; a 1x1 typed product collapses
// to its element, an mp-units quantity, whose identity and zero they define
#include "AP_LinearAlgebra_MathsMacrosPush.h"
#include <typed/fcarouge/kalman_typed_linear_algebra.hpp>
#include <mp_units/fcarouge/kalman_mp_units.hpp>
#include "AP_LinearAlgebra_MathsMacrosPop.h"

namespace fcarouge::kalman_filter::internal {

// The backend's identity and zero are constant expressions, so Kalman's
// make_one() and make_zero() return the constant one<> and zero<> of a typed
// matrix, which Kalman defines only for types with a static Identity() or
// Zero() (Kalman todo branch BUG.md defect 11): define them from the factories.
template <typename Matrix, typename RowIndexes, typename ColumnIndexes>
inline constexpr auto one<typed_matrix<Matrix, RowIndexes, ColumnIndexes>> {
    ones<typed_matrix<Matrix, RowIndexes, ColumnIndexes>>{}()};

template <typename Matrix, typename RowIndexes, typename ColumnIndexes>
inline constexpr auto zero<typed_matrix<Matrix, RowIndexes, ColumnIndexes>> {
    zeros<typed_matrix<Matrix, RowIndexes, ColumnIndexes>>{}()};

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
