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
  Strongly typed linear algebra over a fixed size matrix backend.

  Matrix<T, R, C> is the storage backend of the FrancoisCarouge
  TypedLinearAlgebra typed matrices: fixed size, row major, owning, and
  allocation free. The typed aliases give each element its own type, for
  example an mp-units quantity, so that unit mismatches fail to compile.
 */
#pragma once

#include <stddef.h>

#include <cmath>
#include <limits>
#include <tuple>
#include <type_traits>
#include <utility>

#include "AP_LinearAlgebra_MathsMacrosPush.h"
#include <fcarouge/typed_linear_algebra.hpp>
#include "AP_LinearAlgebra_MathsMacrosPop.h"

namespace AP_LinearAlgebra {

template <typename T, size_t R, size_t C>
struct Matrix {
    T v[R * C] {};

    static constexpr Matrix Zero()
    {
        return Matrix{};
    }

    static constexpr Matrix Identity()
    {
        Matrix m{};
        for (size_t i = 0; i < R && i < C; i++) {
            m(i, i) = T(1);
        }
        return m;
    }

    // element access, as expected by TypedLinearAlgebra
    constexpr T &operator()(size_t i, size_t j)
    {
        return v[i * C + j];
    }
    constexpr const T &operator()(size_t i, size_t j) const
    {
        return v[i * C + j];
    }
    constexpr T &operator()(size_t i)
    {
        return v[i];
    }
    constexpr const T &operator()(size_t i) const
    {
        return v[i];
    }
    constexpr T &operator[](size_t i, size_t j)
    {
        return v[i * C + j];
    }
    constexpr const T &operator[](size_t i, size_t j) const
    {
        return v[i * C + j];
    }
    constexpr T &operator[](size_t i)
    {
        return v[i];
    }
    constexpr const T &operator[](size_t i) const
    {
        return v[i];
    }

    constexpr Matrix<T, C, R> transpose() const
    {
        Matrix<T, C, R> m{};
        for (size_t i = 0; i < R; i++) {
            for (size_t j = 0; j < C; j++) {
                m(j, i) = (*this)(i, j);
            }
        }
        return m;
    }

    friend constexpr Matrix operator-(Matrix a)
    {
        for (size_t k = 0; k < R * C; k++) {
            a.v[k] = -a.v[k];
        }
        return a;
    }
    friend constexpr Matrix operator+(Matrix a, const Matrix &b)
    {
        for (size_t k = 0; k < R * C; k++) {
            a.v[k] += b.v[k];
        }
        return a;
    }
    friend constexpr Matrix operator-(Matrix a, const Matrix &b)
    {
        for (size_t k = 0; k < R * C; k++) {
            a.v[k] -= b.v[k];
        }
        return a;
    }
    friend constexpr Matrix operator*(Matrix a, const T &s)
    {
        for (size_t k = 0; k < R * C; k++) {
            a.v[k] *= s;
        }
        return a;
    }
    friend constexpr Matrix operator*(const T &s, Matrix a)
    {
        return a * s;
    }
    friend constexpr Matrix operator/(Matrix a, const T &s)
    {
        for (size_t k = 0; k < R * C; k++) {
            a.v[k] /= s;
        }
        return a;
    }

    template <size_t N>
    friend constexpr Matrix<T, R, N> operator*(const Matrix &a, const Matrix<T, C, N> &b)
    {
        Matrix<T, R, N> m{};
        for (size_t i = 0; i < R; i++) {
            for (size_t j = 0; j < N; j++) {
                for (size_t k = 0; k < C; k++) {
                    m(i, j) += a(i, k) * b(k, j);
                }
            }
        }
        return m;
    }

    /*
      right division x = a / b, the solution of x * b = a, as in Eigen:
      for a square b this is a * inverse(b); otherwise the minimum norm
      (more rows than columns) or least squares (fewer rows) solution. Typed
      matrix libraries form, for example, a state vector divided by a state
      vector to get the type of a transition matrix. A rank deficient b gives
      non-finite elements.
     */
    template <size_t N>
    friend constexpr Matrix<T, R, N> operator/(const Matrix &a, const Matrix<T, N, C> &b)
    {
        if constexpr (N == C) {
            return solve_right(a, b);
        } else if constexpr (N > C) {
            return solve_right(a, b.transpose() * b) * b.transpose();
        } else {
            return solve_right(a * b.transpose(), b * b.transpose());
        }
    }

    // scalar s / b: the row vector x solving x * b = s, for a column vector b
    friend constexpr Matrix<T, 1, R> operator/(const T &s, const Matrix &b)
        requires (C == 1)
    {
        return Matrix<T, 1, 1>{{s}} / b;
    }

private:
    // x * b = a for a square b, by Gaussian elimination with partial pivoting
    template <size_t RA, size_t K>
    static constexpr Matrix<T, RA, K> solve_right(const Matrix<T, RA, K> &a, const Matrix<T, K, K> &b)
    {
        // solve transpose(b) * transpose(x) = transpose(a)
        Matrix<T, K, K> m = b.transpose();
        Matrix<T, K, RA> x = a.transpose();
        for (size_t col = 0; col < K; col++) {
            size_t pivot = col;
            for (size_t i = col + 1; i < K; i++) {
                const T candidate = m(i, col) < T(0) ? -m(i, col) : m(i, col);
                const T best = m(pivot, col) < T(0) ? -m(pivot, col) : m(pivot, col);
                if (candidate > best) {
                    pivot = i;
                }
            }
            if (pivot != col) {
                for (size_t j = 0; j < K; j++) {
                    const T t = m(col, j);
                    m(col, j) = m(pivot, j);
                    m(pivot, j) = t;
                }
                for (size_t j = 0; j < RA; j++) {
                    const T t = x(col, j);
                    x(col, j) = x(pivot, j);
                    x(pivot, j) = t;
                }
            }
            for (size_t i = col + 1; i < K; i++) {
                const T factor = m(i, col) / m(col, col);
                for (size_t j = col; j < K; j++) {
                    m(i, j) -= factor * m(col, j);
                }
                for (size_t j = 0; j < RA; j++) {
                    x(i, j) -= factor * x(col, j);
                }
            }
        }
        for (size_t col = K; col-- > 0;) {
            for (size_t j = 0; j < RA; j++) {
                T sum = x(col, j);
                for (size_t k = col + 1; k < K; k++) {
                    sum -= m(col, k) * x(k, j);
                }
                x(col, j) = sum / m(col, col);
            }
        }
        return x.transpose();
    }
};

/*
  solve x * b = a for x, b being symmetric positive definite, by the
  Cholesky factorization b = L * transpose(L), without pivoting: x holds a
  on entry, and b is overwritten. Each pivot is compared with the diagonal
  element of b at its position, of the same unit, so that the test does not
  depend on the units of a heterogeneous b. Returns false, x then undefined,
  when b is not positive definite to working precision, singular included.
 */
template <typename T, size_t RA, size_t K>
constexpr bool solve_right_positive_definite(Matrix<T, RA, K> &x, Matrix<T, K, K> &b)
{
    constexpr T tolerance = T(K) * std::numeric_limits<T>::epsilon();
    // L in the lower triangle of b
    for (size_t j = 0; j < K; j++) {
        T d = b(j, j);
        for (size_t k = 0; k < j; k++) {
            d -= b(j, k) * b(j, k);
        }
        // negated to also reject NaN
        if (!(d > tolerance * b(j, j))) {
            return false;
        }
        // parenthesized, as sqrt may be a macro
        const T l = (std::sqrt)(d);
        b(j, j) = l;
        for (size_t i = j + 1; i < K; i++) {
            T sum = b(i, j);
            for (size_t k = 0; k < j; k++) {
                sum -= b(i, k) * b(j, k);
            }
            b(i, j) = sum / l;
        }
    }
    for (size_t r = 0; r < RA; r++) {
        // y * transpose(L) = a, forward
        for (size_t i = 0; i < K; i++) {
            T sum = x(r, i);
            for (size_t k = 0; k < i; k++) {
                sum -= b(i, k) * x(r, k);
            }
            x(r, i) = sum / b(i, i);
        }
        // x * L = y, backward
        for (size_t i = K; i-- > 0;) {
            T sum = x(r, i);
            for (size_t k = i + 1; k < K; k++) {
                sum -= b(k, i) * x(r, k);
            }
            x(r, i) = sum / b(i, i);
        }
    }
    return true;
}

// a matrix whose element (i, j) has the type of the product of the i-th row
// index and the j-th column index
template <typename T, typename RowIndexes, typename ColumnIndexes>
using TypedMatrix = fcarouge::typed_matrix<
    Matrix<T, std::tuple_size_v<RowIndexes>, std::tuple_size_v<ColumnIndexes>>,
    RowIndexes, ColumnIndexes>;

// a column vector whose i-th element has the i-th type
template <typename T, typename... Types>
using ColumnVector = fcarouge::typed_column_vector<Matrix<T, sizeof...(Types), 1>, Types...>;

// a row vector whose j-th element has the j-th type
template <typename T, typename... Types>
using RowVector = fcarouge::typed_row_vector<Matrix<T, 1, sizeof...(Types)>, Types...>;

// the type of the element (i, j) of a typed matrix: vectors only have
// single index access
template <typename M, size_t i, size_t j>
consteval auto element_type()
{
    if constexpr (M::rows == 1) {
        return std::type_identity<std::remove_cvref_t<decltype(std::declval<const M &>().template at<j>())>>{};
    } else if constexpr (M::columns == 1) {
        return std::type_identity<std::remove_cvref_t<decltype(std::declval<const M &>().template at<i>())>>{};
    } else {
        return std::type_identity<std::remove_cvref_t<decltype(std::declval<const M &>().template at<i, j>())>>{};
    }
}

template <typename M, size_t i, size_t j>
using ElementType = typename decltype(element_type<M, i, j>())::type;

// whether each element of the typed matrix From converts to the element of
// To at its position: of the same dimension for mp-units quantities
template <typename From, typename To>
inline constexpr bool converts_elementwise = [] {
    if constexpr (From::rows != To::rows || From::columns != To::columns) {
        return false;
    } else {
        return []<size_t... k>(std::index_sequence<k...>) {
            return (std::is_convertible_v<ElementType<From, k / To::columns, k % To::columns>,
                                          ElementType<To, k / To::columns, k % To::columns>> && ...);
        }(std::make_index_sequence<To::rows * To::columns>{});
    }
}();

/*
  x = a / b, the typed division, for b symmetric positive definite, a
  normal matrix for example: solved by Cholesky factorization rather than
  elimination, in place, with a test of b's definiteness that does not
  depend on its units. Returns false when b is not positive definite to
  working precision.
 */
template <typename A, typename B>
constexpr bool divide_positive_definite(decltype(A{} / B{}) &x, const A &a, B b)
{
    static_assert(std::is_same_v<B, decltype(fcarouge::transposed(b))>,
                  "a symmetric matrix has a symmetric type");
    // the typed division derives its type from the first column of a and
    // b only: check that the equation is dimensionally consistent
    static_assert(converts_elementwise<decltype(x * b), A>,
                  "x * b = a must be dimensionally consistent");
    x.data() = a.data();
    return solve_right_positive_definite(x.data(), b.data());
}

} // namespace AP_LinearAlgebra
